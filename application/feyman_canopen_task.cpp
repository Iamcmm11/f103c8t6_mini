#include "feyman_canopen_task.hpp"

#include <cstdarg>
#include <cstdio>
#include <limits>

#include "FreeRTOS.h"
#include "managers/data_types.hpp"
#include "managers/sync_signal_manager.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace {

constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;
constexpr uint8_t kFeymanTimeStatusHasEpoch = 0x01;

using LibXR::ErrorCode;

}  // namespace

namespace Application {

FeymanCanopenTask::FeymanCanopenTask(LibXR::CAN* can,
                                     const FeymanCanopenConfig& config)
    : can_(can), config_(config) {}

ErrorCode FeymanCanopenTask::Start() {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (can_ == nullptr || config_.topic_name == nullptr ||
      config_.stack_size == 0U || config_.data_rate_hz == 0U) {
    return ErrorCode::ARG_ERR;
  }

  const size_t required_heap = static_cast<size_t>(config_.stack_size) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  if (topic_ != nullptr) {
    delete topic_;
    topic_ = nullptr;
  }
  topic_ = new LibXR::Topic(config_.topic_name, sizeof(Manager::FeymanPoseMsg),
                            nullptr, false, false, false);
  if (topic_ == nullptr) {
    return ErrorCode::NO_MEM;
  }

  const ErrorCode init_ec = device_.Init(can_, BuildDeviceConfig());
  if (init_ec != ErrorCode::OK) {
    delete topic_;
    topic_ = nullptr;
    return init_ec;
  }

  running_ = true;
  device_configured_ = false;
  thread_.Create(this, TaskEntry, "FeymanCAN", config_.stack_size,
                 static_cast<LibXR::Thread::Priority>(config_.priority));
  return ErrorCode::OK;
}

void FeymanCanopenTask::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;
  device_.Stop();
  LibXR::Thread::Sleep(config_.sdo_timeout_ms + 10U);
}

void FeymanCanopenTask::TaskEntry(FeymanCanopenTask* task) {
  if (task != nullptr) {
    task->Run();
  }
}

void FeymanCanopenTask::Run() {
  if (config_.connect_node_id == config_.node_id) {
    Logf("[feyman] start node=0x%02X baud=%lu rate=%lu", config_.node_id,
         static_cast<unsigned long>(config_.baudrate),
         static_cast<unsigned long>(config_.data_rate_hz));
  } else {
    Logf("[feyman] start node=0x%02X->0x%02X baud=%lu rate=%lu",
         config_.connect_node_id, config_.node_id,
         static_cast<unsigned long>(config_.baudrate),
         static_cast<unsigned long>(config_.data_rate_hz));
  }

  LibXR::Thread::Sleep(config_.startup_delay_ms);
  LogConfig("[feyman] configure begin");
  const ErrorCode ec = device_.Configure();
  if (ec != ErrorCode::OK) {
    Logf("[feyman] configure failed ec=%d", static_cast<int>(ec));
    LogCanErrorState("configure");
  } else {
    device_configured_ = true;
    Log("[feyman] configure ok");
  }

  while (running_) {
    FlushPendingCanErrorLog();
    PublishPoseIfReady();
    LibXR::Thread::Sleep(1U);
  }
}

void FeymanCanopenTask::PublishPoseIfReady() {
  if (!device_configured_) {
    return;
  }

  Module::FeymanMCS10Sample sample{};
  const ErrorCode ec = device_.PollSample(sample);
  if (ec != ErrorCode::OK) {
    return;
  }

  Manager::FeymanPoseMsg msg{};
  msg.timestamp_us = LibXR::Timebase::GetMicroseconds();
  msg.readout_mcu_tick_us =
      Manager::SyncSignalManager::ToSessionTickUs(msg.timestamp_us);
  msg.euler[0] = std::numeric_limits<float>::quiet_NaN();
  msg.euler[1] = std::numeric_limits<float>::quiet_NaN();
  msg.euler[2] = std::numeric_limits<float>::quiet_NaN();
  msg.acc[0] = sample.acc[0];
  msg.acc[1] = sample.acc[1];
  msg.acc[2] = sample.acc[2];
  msg.gyro[0] = sample.gyro[0];
  msg.gyro[1] = sample.gyro[1];
  msg.gyro[2] = sample.gyro[2];
  msg.quaternion[0] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[1] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[2] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[3] = std::numeric_limits<float>::quiet_NaN();
  msg.sample_timestamp = 0U;
  msg.status_flags = sample.status_flags;
  msg.sequence = sample.sequence;
  msg.heartbeat_state = sample.heartbeat_state;
  msg.status = 0U;

  Manager::SyncEventRecord epoch;
  if (Manager::SyncSignalManager::IsActive() &&
      Manager::SyncSignalManager::GetLatestEvent(
          Manager::SyncEventSource::TIM2_IMU_SYNC_1HZ, epoch)) {
    msg.time_status =
        static_cast<uint8_t>(msg.time_status | kFeymanTimeStatusHasEpoch);
    msg.sensor_mcu_tick_us = epoch.mcu_tick_us;
  }

  if (topic_ != nullptr) {
    topic_->Publish(msg);
  }
}

void FeymanCanopenTask::FlushPendingCanErrorLog() {
  Module::FeymanMCS10CanError error{};
  if (!device_.TakePendingCanError(error)) {
    return;
  }

  Logf("[feyman] can error cnt=%lu id=0x%08lX",
       static_cast<unsigned long>(error.count),
       static_cast<unsigned long>(error.last_error_id));
  if (error.state_valid) {
    Logf("[feyman] can state ctx=isr tec=%u rec=%u boff=%u ep=%u ew=%u",
         static_cast<unsigned>(error.state.tx_error_counter),
         static_cast<unsigned>(error.state.rx_error_counter),
         static_cast<unsigned>(error.state.bus_off),
         static_cast<unsigned>(error.state.error_passive),
         static_cast<unsigned>(error.state.error_warning));
  } else {
    Log("[feyman] can state ctx=isr unavailable");
  }
}

void FeymanCanopenTask::Log(const char* text) {
  if (config_.log_writer != nullptr && text != nullptr) {
    config_.log_writer(text);
  }
}

void FeymanCanopenTask::LogConfig(const char* text) {
  if (config_.verbose_config_log) {
    Log(text);
  }
}

void FeymanCanopenTask::Logf(const char* fmt, ...) {
  if (config_.log_writer == nullptr || fmt == nullptr) {
    return;
  }
  char line[160] = {0};
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  config_.log_writer(line);
}

void FeymanCanopenTask::LogConfigf(const char* fmt, ...) {
  if (!config_.verbose_config_log || config_.log_writer == nullptr ||
      fmt == nullptr) {
    return;
  }
  char line[160] = {0};
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  config_.log_writer(line);
}

void FeymanCanopenTask::LogCanErrorState(const char* context) {
  LibXR::CAN::ErrorState state{};
  const ErrorCode ec = device_.GetCanErrorState(state);
  if (ec != ErrorCode::OK) {
    Logf("[feyman] can state unavailable ctx=%s ec=%d",
         (context != nullptr) ? context : "-", static_cast<int>(ec));
    return;
  }

  Logf("[feyman] can state ctx=%s tec=%u rec=%u boff=%u ep=%u ew=%u",
       (context != nullptr) ? context : "-",
       static_cast<unsigned>(state.tx_error_counter),
       static_cast<unsigned>(state.rx_error_counter),
       static_cast<unsigned>(state.bus_off),
       static_cast<unsigned>(state.error_passive),
       static_cast<unsigned>(state.error_warning));
}

Module::FeymanMCS10Config FeymanCanopenTask::BuildDeviceConfig() const {
  Module::FeymanMCS10Config device_config;
  device_config.connect_node_id = config_.connect_node_id;
  device_config.node_id = config_.node_id;
  device_config.baudrate = config_.baudrate;
  device_config.data_rate_hz = config_.data_rate_hz;
  device_config.heartbeat_ms = config_.heartbeat_ms;
  device_config.sdo_timeout_ms = config_.sdo_timeout_ms;
  device_config.sdo_inter_request_delay_ms =
      config_.sdo_inter_request_delay_ms;
  device_config.work_mode_settle_ms = config_.work_mode_settle_ms;
  return device_config;
}

}  // namespace Application
