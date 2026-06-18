#include "feyman_manager.hpp"

#include <cstdarg>
#include <cstdio>
#include <limits>
#include "FreeRTOS.h"
#include "managers/sync_signal_manager.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace {

constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;
constexpr uint32_t kConfigureRetryPeriodMs = 1000;
constexpr uint32_t kAggregateCoalesceWindowUs = 2000;
constexpr uint8_t kFeymanTimeStatusHasEpoch = 0x01;

using LibXR::ErrorCode;

}  // namespace

namespace Manager {

ErrorCode FeymanManager::Init(const FeymanManagerConfig& config) {
  if (config.can == nullptr || config.devices == nullptr ||
      config.device_count == 0U || config.device_count > devices_.size() ||
      config.aggregate_topic_name == nullptr ||
      config.legacy_topic_name == nullptr || config.stack_size == 0U) {
    return ErrorCode::ARG_ERR;
  }

  config_ = config;
  device_count_ = static_cast<uint8_t>(config.device_count);
  for (uint8_t i = 0; i < device_count_; ++i) {
    devices_[i].config = FeymanManagedDeviceConfig{};
    devices_[i].latest = FeymanDeviceMsg{};
    devices_[i].initialized = false;
    devices_[i].configured = false;
    devices_[i].config = config.devices[i];
    devices_[i].latest.node_id = config.devices[i].node_id;
    devices_[i].latest.status =
        static_cast<uint8_t>(-static_cast<int8_t>(ErrorCode::INIT_ERR));
  }

  if (aggregate_topic_ != nullptr) {
    delete aggregate_topic_;
    aggregate_topic_ = nullptr;
  }
  if (legacy_topic_ != nullptr) {
    delete legacy_topic_;
    legacy_topic_ = nullptr;
  }

  aggregate_topic_ = new LibXR::Topic(config.aggregate_topic_name,
                                      sizeof(FeymanArrayMsg), nullptr, false,
                                      false, false);
  if (aggregate_topic_ == nullptr) {
    return ErrorCode::NO_MEM;
  }

  legacy_topic_ = new LibXR::Topic(config.legacy_topic_name,
                                   sizeof(FeymanPoseMsg), nullptr, false,
                                   false, false);
  if (legacy_topic_ == nullptr) {
    delete aggregate_topic_;
    aggregate_topic_ = nullptr;
    return ErrorCode::NO_MEM;
  }

  active_ = false;
  thread_alive_ = false;
  thread_created_ = false;
  publish_sequence_ = 0U;
  last_config_attempt_ms_ = 0U;
  ResetAggregateCoalesce();
  last_aggregate_ = FeymanArrayMsg{};
  last_aggregate_.primary_node_id = config.primary_node_id;
  return ErrorCode::OK;
}

ErrorCode FeymanManager::Start() {
  if (aggregate_topic_ == nullptr || legacy_topic_ == nullptr ||
      config_.can == nullptr) {
    return ErrorCode::INIT_ERR;
  }

  active_ = true;
  if (thread_created_) {
    return ErrorCode::OK;
  }

  const size_t required_heap = static_cast<size_t>(config_.stack_size) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  can_callback_ = LibXR::CAN::Callback::Create(OnCanFrame, this);
  config_.can->Register(can_callback_, LibXR::CAN::Type::STANDARD,
                        LibXR::CAN::FilterMode::ID_RANGE, 0x180, 0x7FF);
  config_.can->Register(can_callback_, LibXR::CAN::Type::ERROR);

  thread_alive_ = true;
  thread_.Create(this, TaskEntry, "FeymanMgr", config_.stack_size,
                 static_cast<LibXR::Thread::Priority>(config_.priority));
  thread_created_ = true;
  return ErrorCode::OK;
}

void FeymanManager::Stop() { active_ = false; }

void FeymanManager::TaskEntry(FeymanManager* manager) {
  if (manager != nullptr) {
    manager->Run();
  }
}

void FeymanManager::OnCanFrame(bool in_isr, FeymanManager* manager,
                               const LibXR::CAN::ClassicPack& pack) {
  if (manager != nullptr) {
    manager->HandleCanFrame(in_isr, pack);
  }
}

void FeymanManager::Run() {
  LibXR::Thread::Sleep(config_.startup_delay_ms);
  while (thread_alive_) {
    if (active_) {
      {
        LibXR::Mutex::LockGuard guard(op_mutex_);
        const bool state_changed = TryConfigureDevices();
        const bool sample_updated = PollDevicesAndPublish();
        FlushCanErrorLog();
        const uint64_t now_us = LibXR::Timebase::GetMicroseconds();
        if (state_changed) {
          PublishAggregate();
          ResetAggregateCoalesce();
        } else if ((sample_updated || aggregate_window_start_us_ != 0U) &&
                   ShouldPublishAggregate(now_us)) {
          PublishAggregate();
          ResetAggregateCoalesce();
        }
      }
    }
    LibXR::Thread::Sleep(1U);
  }
}

void FeymanManager::HandleCanFrame(bool in_isr,
                                   const LibXR::CAN::ClassicPack& pack) {
  if (pack.type == LibXR::CAN::Type::ERROR) {
    HandleCanError(pack);
    return;
  }
  if (pack.type != LibXR::CAN::Type::STANDARD) {
    return;
  }

  for (uint8_t i = 0; i < device_count_; ++i) {
    if (!devices_[i].config.enabled || !devices_[i].initialized) {
      continue;
    }
    devices_[i].device.ProcessCanFrame(in_isr, pack);
  }
}

void FeymanManager::HandleCanError(const LibXR::CAN::ClassicPack& pack) {
  last_can_error_id_ = pack.id;
  ++last_can_error_count_;
  (void)config_.can->GetErrorState(last_can_error_state_);
}

void FeymanManager::FlushCanErrorLog() {
  if (last_can_error_count_ == emitted_can_error_count_) {
    return;
  }

  emitted_can_error_count_ = last_can_error_count_;
  Logf("[feyman_mgr] can error cnt=%lu id=0x%08lX",
       static_cast<unsigned long>(last_can_error_count_),
       static_cast<unsigned long>(last_can_error_id_));
  Logf("[feyman_mgr] can state tec=%u rec=%u boff=%u ep=%u ew=%u",
       static_cast<unsigned>(last_can_error_state_.tx_error_counter),
       static_cast<unsigned>(last_can_error_state_.rx_error_counter),
       static_cast<unsigned>(last_can_error_state_.bus_off),
       static_cast<unsigned>(last_can_error_state_.error_passive),
       static_cast<unsigned>(last_can_error_state_.error_warning));
}

bool FeymanManager::TryConfigureDevices() {
  const uint32_t now_ms = LibXR::Thread::GetTime();
  if ((now_ms - last_config_attempt_ms_) < kConfigureRetryPeriodMs &&
      last_config_attempt_ms_ != 0U) {
    return false;
  }
  last_config_attempt_ms_ = now_ms;

  bool changed = false;
  for (uint8_t i = 0; i < device_count_; ++i) {
    auto& runtime = devices_[i];
    if (!runtime.config.enabled) {
      continue;
    }

    if (!runtime.initialized) {
      const ErrorCode init_ec = runtime.device.Init(
          config_.can, BuildDeviceConfig(runtime.config.node_id,
                                         runtime.config.node_id));
      runtime.initialized = (init_ec == ErrorCode::OK);
      runtime.configured = false;
      runtime.latest.node_id = runtime.config.node_id;
      runtime.latest.configured = 0U;
      runtime.latest.online = 0U;
      runtime.latest.status =
          static_cast<uint8_t>(-static_cast<int8_t>(init_ec));
      changed = true;
      if (init_ec != ErrorCode::OK) {
        Logf("[feyman_mgr] init node=0x%02X ec=%d", runtime.config.node_id,
             static_cast<int>(init_ec));
        continue;
      }
    }

    if (runtime.configured) {
      continue;
    }

    LogConfigf("[feyman_mgr] configure node=0x%02X", runtime.config.node_id);
    const ErrorCode cfg_ec = runtime.device.Configure();
    if (cfg_ec == ErrorCode::OK) {
      runtime.configured = true;
      runtime.latest.configured = 1U;
      runtime.latest.online = 1U;
      runtime.latest.status = 0U;
      Logf("[feyman_mgr] configure ok node=0x%02X", runtime.config.node_id);
    } else {
      runtime.latest.configured = 0U;
      runtime.latest.online = 0U;
      runtime.latest.status =
          static_cast<uint8_t>(-static_cast<int8_t>(cfg_ec));
      Logf("[feyman_mgr] configure fail node=0x%02X ec=%d",
           runtime.config.node_id, static_cast<int>(cfg_ec));
    }
    changed = true;
  }
  return changed;
}

bool FeymanManager::PollDevicesAndPublish() {
  bool any_updated = false;
  for (uint8_t i = 0; i < device_count_; ++i) {
    auto& runtime = devices_[i];
    if (!runtime.config.enabled || !runtime.initialized || !runtime.configured) {
      continue;
    }

    Module::FeymanMCS10Sample sample{};
    const ErrorCode ec = runtime.device.PollSample(sample);
    if (ec != ErrorCode::OK) {
      continue;
    }

    auto& msg = runtime.latest;
    msg.node_id = runtime.config.node_id;
    msg.online = 1U;
    msg.configured = 1U;
    msg.status = 0U;
    msg.timestamp_us = LibXR::Timebase::GetMicroseconds();
    msg.readout_mcu_tick_us =
        SyncSignalManager::ToSessionTickUs(msg.timestamp_us);
    msg.sensor_mcu_tick_us = 0U;
    msg.time_status = 0U;
    msg.status_flags = sample.status_flags;
    msg.sequence = sample.sequence;
    msg.heartbeat_state = sample.heartbeat_state;
    msg.acc[0] = sample.acc[0];
    msg.acc[1] = sample.acc[1];
    msg.acc[2] = sample.acc[2];
    msg.gyro[0] = sample.gyro[0];
    msg.gyro[1] = sample.gyro[1];
    msg.gyro[2] = sample.gyro[2];

    SyncEventRecord epoch;
    if (SyncSignalManager::IsActive() &&
        SyncSignalManager::GetLatestEvent(
            SyncEventSource::TIM2_IMU_SYNC_1HZ, epoch)) {
      msg.time_status =
          static_cast<uint8_t>(msg.time_status | kFeymanTimeStatusHasEpoch);
      msg.sensor_mcu_tick_us = epoch.mcu_tick_us;
    }

    if (runtime.config.node_id == config_.primary_node_id) {
      PublishLegacyIfPrimaryUpdated(msg);
    }
    if (aggregate_window_start_us_ == 0U) {
      aggregate_window_start_us_ = msg.timestamp_us;
    }
    updated_device_mask_ =
        static_cast<uint16_t>(updated_device_mask_ | (1U << i));
    any_updated = true;
  }
  return any_updated;
}

uint16_t FeymanManager::BuildReadyDeviceMask() const {
  uint16_t mask = 0U;
  for (uint8_t i = 0; i < device_count_; ++i) {
    const auto& runtime = devices_[i];
    if (!runtime.config.enabled || !runtime.initialized ||
        !runtime.configured) {
      continue;
    }
    const auto& latest = runtime.latest;
    if (latest.online == 0U || latest.configured == 0U ||
        latest.status != 0U) {
      continue;
    }
    mask = static_cast<uint16_t>(mask | (1U << i));
  }
  return mask;
}

bool FeymanManager::ShouldPublishAggregate(uint64_t now_us) const {
  if (aggregate_window_start_us_ == 0U || updated_device_mask_ == 0U) {
    return false;
  }

  const uint16_t ready_mask = BuildReadyDeviceMask();
  if (ready_mask == 0U) {
    return false;
  }

  if ((updated_device_mask_ & ready_mask) == ready_mask) {
    return true;
  }

  return (now_us - aggregate_window_start_us_) >= kAggregateCoalesceWindowUs;
}

void FeymanManager::ResetAggregateCoalesce() {
  updated_device_mask_ = 0U;
  aggregate_window_start_us_ = 0U;
}

void FeymanManager::PublishLegacyIfPrimaryUpdated(const FeymanDeviceMsg& msg) {
  if (legacy_topic_ == nullptr) {
    return;
  }

  FeymanPoseMsg legacy{};
  legacy.timestamp_us = msg.timestamp_us;
  legacy.sensor_mcu_tick_us = msg.sensor_mcu_tick_us;
  legacy.readout_mcu_tick_us = msg.readout_mcu_tick_us;
  legacy.acc[0] = msg.acc[0];
  legacy.acc[1] = msg.acc[1];
  legacy.acc[2] = msg.acc[2];
  legacy.gyro[0] = msg.gyro[0];
  legacy.gyro[1] = msg.gyro[1];
  legacy.gyro[2] = msg.gyro[2];
  legacy.status_flags = msg.status_flags;
  legacy.status = msg.status;
  legacy.time_status = msg.time_status;
  legacy.sequence = msg.sequence;
  legacy.heartbeat_state = msg.heartbeat_state;
  legacy.euler[0] = legacy.euler[1] = legacy.euler[2] =
      std::numeric_limits<float>::quiet_NaN();
  legacy.quaternion[0] = legacy.quaternion[1] = legacy.quaternion[2] =
      legacy.quaternion[3] = std::numeric_limits<float>::quiet_NaN();
  legacy_topic_->Publish(legacy);
}

void FeymanManager::PublishAggregate() {
  if (aggregate_topic_ == nullptr) {
    return;
  }

  FeymanArrayMsg msg{};
  msg.timestamp_us = LibXR::Timebase::GetMicroseconds();
  msg.primary_node_id = config_.primary_node_id;
  msg.publish_sequence = publish_sequence_++;

  for (uint8_t i = 0; i < device_count_ && msg.device_count < msg.devices.size();
       ++i) {
    if (!devices_[i].config.enabled) {
      continue;
    }
    msg.devices[msg.device_count++] = devices_[i].latest;
  }

  last_aggregate_ = msg;
  aggregate_topic_->Publish(msg);
}

void FeymanManager::Log(const char* text) {
  if (config_.log_writer != nullptr && text != nullptr) {
    config_.log_writer(text);
  }
}

void FeymanManager::Logf(const char* fmt, ...) {
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

void FeymanManager::LogConfigf(const char* fmt, ...) {
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

Module::FeymanMCS10Config FeymanManager::BuildDeviceConfig(
    uint8_t connect_node_id, uint8_t target_node_id) const {
  Module::FeymanMCS10Config cfg;
  cfg.connect_node_id = connect_node_id;
  cfg.node_id = target_node_id;
  cfg.baudrate = config_.baudrate;
  cfg.data_rate_hz = config_.data_rate_hz;
  cfg.heartbeat_ms = config_.heartbeat_ms;
  cfg.sdo_timeout_ms = config_.sdo_timeout_ms;
  cfg.sdo_inter_request_delay_ms = config_.sdo_inter_request_delay_ms;
  cfg.work_mode_settle_ms = config_.work_mode_settle_ms;
  cfg.register_can_callback = false;
  return cfg;
}

}  // namespace Manager
