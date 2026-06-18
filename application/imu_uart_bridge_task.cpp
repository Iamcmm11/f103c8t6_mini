#include "imu_uart_bridge_task.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "FreeRTOS.h"
#include "semaphore.hpp"
#include "task.h"

namespace {

constexpr uint8_t kBridgeSof0 = 0x55;
constexpr uint8_t kBridgeSof1 = 0xAA;
constexpr uint8_t kBridgeCmdPing = 0x01;
constexpr uint8_t kBridgeCmdTimeSync = 0x02;
constexpr uint8_t kBridgeCmdStreamControl = 0x03;
constexpr uint8_t kBridgeCmdI2CRead = 0x10;
constexpr uint8_t kBridgeCmdI2CWrite = 0x11;
constexpr uint8_t kBridgeCmdSPIWrite = 0x20;
constexpr uint8_t kBridgeCmdWS2812Control = 0x21;
constexpr uint8_t kBridgeCmdPosePush = 0x30;
constexpr uint8_t kBridgeCmdSyncEventPush = 0x32;
constexpr uint8_t kBridgeCmdGPIOButtonPush = 0x33;

constexpr std::array<uint8_t, Manager::ACTUAL_IMU_COUNT> kBridgeIMUPushIndexes = {
    static_cast<uint8_t>(Manager::ImuSlot::Forearm),
    static_cast<uint8_t>(Manager::ImuSlot::Hand),
    static_cast<uint8_t>(Manager::ImuSlot::ThumbRoot),
    static_cast<uint8_t>(Manager::ImuSlot::ThumbTip),
    static_cast<uint8_t>(Manager::ImuSlot::Extra0),
    static_cast<uint8_t>(Manager::ImuSlot::Extra1),
};

constexpr uint8_t kBridgePoseFloatCount = 7;
constexpr uint8_t kBridgeWITTimestampSize = sizeof(uint64_t);
constexpr uint8_t kBridgeWITCompactHeaderSize = sizeof(uint8_t) + sizeof(uint64_t);
constexpr uint8_t kBridgeWITCompactRecordSize =
    static_cast<uint8_t>(sizeof(uint8_t) + sizeof(uint16_t) +
                         (3U + 4U + 3U) * sizeof(int16_t));
constexpr uint8_t kBridgeTimeSyncSeqSize = sizeof(uint32_t);
constexpr uint8_t kBridgeTimeSyncRespPayloadSize = static_cast<uint8_t>(
    1 + kBridgeTimeSyncSeqSize + sizeof(uint64_t) + sizeof(uint64_t));
constexpr uint8_t kBridgeYISTimestampSize = sizeof(uint32_t);
constexpr uint8_t kBridgeYISExtendedTimestampSize =
    static_cast<uint8_t>(sizeof(uint32_t) + sizeof(uint64_t) +
                         sizeof(uint64_t) + sizeof(uint8_t));
constexpr uint8_t kBridgePoseRecordSize =
    static_cast<uint8_t>(1 + kBridgePoseFloatCount * sizeof(float));
constexpr uint8_t kBridgeExtendedFloatCount = 13;
constexpr uint8_t kBridgeExtendedRecordSize = static_cast<uint8_t>(
    1 + kBridgeExtendedFloatCount * sizeof(float));
constexpr uint8_t kBridgeWITPoseRecordSize = static_cast<uint8_t>(
    kBridgePoseRecordSize + kBridgeWITTimestampSize);
constexpr uint8_t kBridgeYISPoseRecordSize = static_cast<uint8_t>(
    kBridgePoseRecordSize + kBridgeYISTimestampSize);
constexpr uint8_t kBridgeYISExtendedPoseRecordSize = static_cast<uint8_t>(
    kBridgePoseRecordSize + kBridgeYISExtendedTimestampSize);
constexpr uint8_t kBridgeSyncEventRecordSize =
    static_cast<uint8_t>(sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t) +
                         sizeof(uint32_t) + sizeof(uint64_t) +
                         sizeof(uint32_t) + sizeof(uint32_t));
constexpr uint8_t kBridgeSyncEventMaxRecords = 8;
constexpr uint16_t kBridgeSyncEventMaxPayload = static_cast<uint16_t>(
    sizeof(uint8_t) + kBridgeSyncEventMaxRecords * kBridgeSyncEventRecordSize);
constexpr uint8_t kBridgeStatusOk = 0;
constexpr uint8_t kBridgeStatusError = 1;
constexpr uint8_t kBridgeStreamControlSeqSize = sizeof(uint32_t);
constexpr uint8_t kBridgeStreamControlRespPayloadSize =
    static_cast<uint8_t>(sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint8_t));
constexpr uint8_t kMaxRegisterCount = 32;
constexpr uint16_t kBridgeWITMaxPayload = static_cast<uint16_t>(
    kBridgeWITCompactHeaderSize +
    kBridgeIMUPushIndexes.size() * kBridgeWITCompactRecordSize);
constexpr uint16_t kBridgeYISMaxPayload =
    static_cast<uint16_t>(1 + kBridgeYISExtendedPoseRecordSize);
constexpr uint16_t kBridgeFeymanMaxPayload = static_cast<uint16_t>(
    1 + Manager::MAX_FEYMAN_DEVICE_COUNT * kBridgeExtendedRecordSize);
constexpr uint16_t kBridgePoseMaxPayload =
    (kBridgeWITMaxPayload > kBridgeYISMaxPayload)
        ? ((kBridgeWITMaxPayload > kBridgeFeymanMaxPayload)
               ? kBridgeWITMaxPayload
               : kBridgeFeymanMaxPayload)
        : ((kBridgeYISMaxPayload > kBridgeFeymanMaxPayload)
               ? kBridgeYISMaxPayload
               : kBridgeFeymanMaxPayload);
constexpr uint16_t kBridgeNonSyncMaxPayload =
    (kBridgePoseMaxPayload > kBridgeTimeSyncRespPayloadSize)
        ? kBridgePoseMaxPayload
        : kBridgeTimeSyncRespPayloadSize;
constexpr uint16_t kBridgeControlMaxPayload =
    (kBridgeNonSyncMaxPayload > kBridgeStreamControlRespPayloadSize)
        ? kBridgeNonSyncMaxPayload
        : kBridgeStreamControlRespPayloadSize;
constexpr uint16_t kBridgeMaxResponsePayload =
    (kBridgeControlMaxPayload > kBridgeSyncEventMaxPayload)
        ? kBridgeControlMaxPayload
        : kBridgeSyncEventMaxPayload;

std::array<uint8_t, kBridgeFeymanMaxPayload> g_bridge_feyman_payload{};
std::array<uint8_t, 5 + kBridgeMaxResponsePayload + 1>
    g_bridge_response_frame{};
Manager::FeymanArrayMsg g_bridge_feyman_snapshot{};

constexpr uint32_t kBridgeWriteRetryCount = 3;
constexpr uint32_t kBridgeWriteRetryDelayMs = 1;
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;
constexpr size_t kYISBridgeQueueDepth = 32;
constexpr uint8_t kYISBridgeAddress = 0x6A;
constexpr float kGravityMps2 = 9.80665f;

constexpr uint16_t BuildLeadingWITBridgeMask(uint8_t imu_count) {
  const uint8_t capped_count =
      (imu_count > Manager::ACTUAL_IMU_COUNT) ? Manager::ACTUAL_IMU_COUNT
                                              : imu_count;
  return (capped_count == 0U)
             ? 0U
             : static_cast<uint16_t>((1u << capped_count) - 1u);
}

int16_t FloatToInt16Clamped(float value) {
  if (std::isnan(value)) {
    return 0;
  }
  const float rounded = std::round(value);
  const float clamped =
      std::max(-32768.0f, std::min(32767.0f, rounded));
  return static_cast<int16_t>(clamped);
}

int16_t AccMps2ToMilliG(float acc_mps2) {
  return FloatToInt16Clamped(acc_mps2 / kGravityMps2 * 1000.0f);
}

int16_t QuatToQ15(float quat) { return FloatToInt16Clamped(quat * 32767.0f); }

uint16_t TickDeltaUs(uint64_t base_tick_us, uint64_t tick_us) {
  if (tick_us <= base_tick_us) {
    return 0U;
  }
  const uint64_t delta = tick_us - base_tick_us;
  return (delta > 0xFFFFULL) ? 0xFFFFU : static_cast<uint16_t>(delta);
}

}  // namespace

namespace Application {

using namespace LibXR;

IMUUartBridgeTask::IMUUartBridgeTask(UART* uart, I2C* i2c, SPI* spi,
                                     Manager::IMUManager* imu_mgr,
                                     Manager::WS2812Manager* ws2812_mgr,
                                     const IMUUartBridgeConfig& config)
    : uart_(uart),
      i2c_(i2c),
      spi_(spi),
      imu_mgr_(imu_mgr),
      ws2812_mgr_(ws2812_mgr),
      config_(config),
      running_(false),
      last_pose_push_ms_(0),
      streaming_enabled_(false),
      thread_(nullptr),
      wit_subscriber_(nullptr),
      yis_queue_(nullptr),
      yis_queue_subscriber_(nullptr),
      feyman_subscriber_(nullptr),
      gpio_button_queue_(8) {}

ErrorCode IMUUartBridgeTask::Start() {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (uart_ == nullptr || i2c_ == nullptr || imu_mgr_ == nullptr) {
    return ErrorCode::PTR_NULL;
  }

  const size_t required_heap = static_cast<size_t>(config_.stack_size) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  running_ = true;
  thread_ = new Thread();
  if (thread_ == nullptr) {
    running_ = false;
    return ErrorCode::NO_MEM;
  }

  thread_->Create(this, TaskEntry, "IMUBridge", config_.stack_size,
                  static_cast<Thread::Priority>(config_.priority));
  return ErrorCode::OK;
}

void IMUUartBridgeTask::Stop() {
  if (!running_) {
    return;
  }
  (void)SetStreamingEnabled(false);
  running_ = false;
  Thread::Sleep(config_.read_timeout_ms + 10);
}

bool IMUUartBridgeTask::PublishGPIOButtonCommand(char command) {
  return gpio_button_queue_.Push(static_cast<uint8_t>(command)) == ErrorCode::OK;
}

bool IMUUartBridgeTask::PublishGPIOButtonCommandCallback(void* context,
                                                         char command) {
  auto* bridge = static_cast<IMUUartBridgeTask*>(context);
  return bridge != nullptr && bridge->PublishGPIOButtonCommand(command);
}

void IMUUartBridgeTask::TaskEntry(IMUUartBridgeTask* arg) {
  if (arg != nullptr) {
    arg->Run();
  }
}

void IMUUartBridgeTask::Run() {
  if (config_.stream_relative_euler) {
    RunStreamMode();
  } else {
    RunBridgeMode();
  }
}

void IMUUartBridgeTask::RunStreamMode() {
  auto topic = Topic::Find("imu_data");
  if (topic != nullptr) {
    wit_subscriber_ =
        new Topic::ASyncSubscriber<Manager::IMUArrayMsg>(Topic(topic));
  }
  if (wit_subscriber_ != nullptr) {
    wit_subscriber_->StartWaiting();
  }
  (void)WriteString("roll_deg,pitch_deg,yaw_deg\r\n");
  if (wit_subscriber_ == nullptr) {
    (void)WriteString("# imu_data topic unavailable\r\n");
  }
  while (running_) {
    if (wit_subscriber_ == nullptr) {
      Thread::Sleep(20);
      continue;
    }
    if (wit_subscriber_->Available()) {
      auto& imu_msg = wit_subscriber_->GetData();
      if (imu_msg.IsValid(0)) {
        const auto& imu = imu_msg.imu_data[0];
        char line[128] = {0};
        const int len = std::snprintf(
            line, sizeof(line), "%.3f,%.3f,%.3f\r\n",
            static_cast<double>(imu.angle[0]),
            static_cast<double>(imu.angle[1]),
            static_cast<double>(imu.angle[2]));
        if (len > 0) {
          (void)WriteExact(reinterpret_cast<const uint8_t*>(line),
                           static_cast<uint16_t>(len));
        }
      }
      wit_subscriber_->StartWaiting();
    } else {
      Thread::Sleep(1);
    }
  }

  delete wit_subscriber_;
  wit_subscriber_ = nullptr;
}

void IMUUartBridgeTask::RunBridgeMode() {
  if (config_.push_imu_euler_in_bridge) {
    if (config_.pose_source == BridgePoseSource::WIT) {
      auto topic = Topic::Find("imu_data");
      if (topic != nullptr) {
        wit_subscriber_ =
            new Topic::ASyncSubscriber<Manager::IMUArrayMsg>(Topic(topic));
      }
      if (wit_subscriber_ != nullptr) {
        wit_subscriber_->StartWaiting();
        last_pose_push_ms_ = Thread::GetTime();
      }
    } else if (config_.pose_source == BridgePoseSource::YIS) {
      auto topic = Topic::Find("yis_imu_pose");
      if (topic != nullptr) {
        yis_queue_ = new LockFreeQueue<Manager::YISPoseMsg>(kYISBridgeQueueDepth);
        if (yis_queue_ != nullptr) {
          yis_queue_subscriber_ =
              new Topic::QueuedSubscriber(Topic(topic), *yis_queue_);
        }
      }
      if (yis_queue_subscriber_ != nullptr) {
        last_pose_push_ms_ = Thread::GetTime();
      }
    } else {
      auto topic = Topic::Find("feyman_imu_array");
      if (topic != nullptr) {
        feyman_subscriber_ =
            new Topic::ASyncSubscriber<Manager::FeymanArrayMsg>(Topic(topic));
      }
      if (feyman_subscriber_ != nullptr) {
        feyman_subscriber_->StartWaiting();
        last_pose_push_ms_ = Thread::GetTime();
      }
    }
  }
  while (running_) {
    bool did_work = ProcessPendingCommand();
    const PublishResult gpio_result = PublishPendingGPIOButtonCommand();
    did_work = did_work || (gpio_result == PublishResult::SENT);

    if (streaming_enabled_) {
      const PublishResult pose_result = PublishBridgePoseData();
      did_work = did_work || (pose_result == PublishResult::SENT);
      if (config_.push_sync_events_in_bridge &&
          pose_result != PublishResult::BACKPRESSURE) {
        const uint8_t sync_count_before = pending_sync_event_count_;
        PublishSyncEvents();
        did_work = did_work || (sync_count_before != 0U &&
                                pending_sync_event_count_ == 0U);
      }
      if (!did_work || gpio_result == PublishResult::BACKPRESSURE ||
          pose_result == PublishResult::BACKPRESSURE) {
        Thread::Sleep(1);
      }
    } else {
      ClearPendingPushData();
      Thread::Sleep(1);
    }
  }

  delete wit_subscriber_;
  wit_subscriber_ = nullptr;
  delete yis_queue_subscriber_;
  yis_queue_subscriber_ = nullptr;
  delete yis_queue_;
  yis_queue_ = nullptr;
  delete feyman_subscriber_;
  feyman_subscriber_ = nullptr;
}

IMUUartBridgeTask::PublishResult IMUUartBridgeTask::PublishPendingGPIOButtonCommand() {
  if (!has_pending_gpio_button_command_) {
    if (gpio_button_queue_.Pop(pending_gpio_button_command_) != ErrorCode::OK) {
      return PublishResult::NONE;
    }
    has_pending_gpio_button_command_ = true;
  }

  if (SendResponse(kBridgeCmdGPIOButtonPush, &pending_gpio_button_command_,
                   sizeof(pending_gpio_button_command_))) {
    has_pending_gpio_button_command_ = false;
    return PublishResult::SENT;
  }

  return PublishResult::BACKPRESSURE;
}

bool IMUUartBridgeTask::ProcessPendingCommand() {
  if (uart_ == nullptr || uart_->read_port_ == nullptr) {
    return false;
  }

  bool consumed = false;
  size_t pending_bytes = uart_->read_port_->Size();
  while (running_ && pending_bytes > 0U) {
    uint8_t byte = 0;
    ReadOperation op;
    if (uart_->Read({&byte, 1}, op) != ErrorCode::OK) {
      break;
    }
    consumed = true;
    ProcessCommandByte(byte);
    --pending_bytes;
  }

  return consumed;
}

void IMUUartBridgeTask::ResetCommandParser() {
  command_parser_state_ = CommandParserState::WAIT_SOF0;
  command_payload_len_ = 0;
  command_payload_pos_ = 0;
  command_cmd_ = 0;
  command_sum_ = 0;
}

void IMUUartBridgeTask::ProcessCommandByte(uint8_t byte) {
  switch (command_parser_state_) {
    case CommandParserState::WAIT_SOF0:
      if (byte == kBridgeSof0) {
        command_sum_ = byte;
        command_parser_state_ = CommandParserState::WAIT_SOF1;
      }
      return;

    case CommandParserState::WAIT_SOF1:
      if (byte == kBridgeSof1) {
        command_sum_ = static_cast<uint8_t>(command_sum_ + byte);
        command_parser_state_ = CommandParserState::READ_CMD;
      } else if (byte == kBridgeSof0) {
        command_sum_ = byte;
      } else {
        ResetCommandParser();
      }
      return;

    case CommandParserState::READ_CMD:
      command_cmd_ = byte;
      command_sum_ = static_cast<uint8_t>(command_sum_ + byte);
      command_parser_state_ = CommandParserState::READ_LEN0;
      return;

    case CommandParserState::READ_LEN0:
      command_payload_len_ = byte;
      command_sum_ = static_cast<uint8_t>(command_sum_ + byte);
      command_parser_state_ = CommandParserState::READ_LEN1;
      return;

    case CommandParserState::READ_LEN1:
      command_payload_len_ = static_cast<uint16_t>(
          command_payload_len_ | (static_cast<uint16_t>(byte) << 8U));
      command_sum_ = static_cast<uint8_t>(command_sum_ + byte);
      command_payload_pos_ = 0;
      if (command_payload_len_ > command_payload_.size()) {
        ResetCommandParser();
      } else if (command_payload_len_ == 0U) {
        command_parser_state_ = CommandParserState::READ_CHECKSUM;
      } else {
        command_parser_state_ = CommandParserState::READ_PAYLOAD;
      }
      return;

    case CommandParserState::READ_PAYLOAD:
      command_payload_[command_payload_pos_++] = byte;
      command_sum_ = static_cast<uint8_t>(command_sum_ + byte);
      if (command_payload_pos_ >= command_payload_len_) {
        command_parser_state_ = CommandParserState::READ_CHECKSUM;
      }
      return;

    case CommandParserState::READ_CHECKSUM:
      if (byte == command_sum_) {
        HandleCommandFrame(command_cmd_, command_payload_.data(),
                           command_payload_len_);
      }
      ResetCommandParser();
      return;
  }
}

IMUUartBridgeTask::PublishResult IMUUartBridgeTask::PublishBridgePoseData() {
  if (!streaming_enabled_ || !config_.push_imu_euler_in_bridge) {
    return PublishResult::NONE;
  }

  const uint32_t now_ms = Thread::GetTime();
  const bool interval_ok =
      (config_.stream_interval_ms == 0U) ||
      ((now_ms - last_pose_push_ms_) >= config_.stream_interval_ms);
  if (!interval_ok) {
    return PublishResult::NONE;
  }

  if (config_.pose_source == BridgePoseSource::WIT) {
    if (wit_subscriber_ == nullptr || !wit_subscriber_->Available()) {
      return PublishResult::NONE;
    }

    auto& imu_msg = wit_subscriber_->GetData();
    const uint16_t wit_push_slots_mask =
        BuildLeadingWITBridgeMask(config_.wit_push_imu_count);
    std::array<uint8_t, kBridgeWITMaxPayload> payload{};
    payload[0] = 0;
    uint64_t base_tick_us = 0U;
    for (const uint8_t imu_index : kBridgeIMUPushIndexes) {
      if ((wit_push_slots_mask & (1u << imu_index)) == 0U ||
          !imu_msg.IsValid(imu_index)) {
        continue;
      }
      base_tick_us = Manager::SyncSignalManager::ToSessionTickUs(
          imu_msg.imu_data[imu_index].mcu_tick_us);
      break;
    }
    std::memcpy(payload.data() + 1, &base_tick_us, sizeof(base_tick_us));
    uint16_t payload_len = kBridgeWITCompactHeaderSize;

    for (const uint8_t imu_index : kBridgeIMUPushIndexes) {
      if ((wit_push_slots_mask & (1u << imu_index)) == 0U) {
        continue;
      }
      const bool valid = imu_msg.IsValid(imu_index);
      if (!valid && !config_.push_all_slots_in_bridge) {
        continue;
      }

      const uint16_t base = payload_len;
      payload[base] =
          Manager::ResolveImuI2CAddress(imu_index, config_.imu_addr);
      uint16_t cursor = static_cast<uint16_t>(base + 1);
      const auto& imu = imu_msg.imu_data[imu_index];
      const uint64_t imu_tick_us =
          Manager::SyncSignalManager::ToSessionTickUs(imu.mcu_tick_us);
      const uint16_t tick_delta_us =
          valid ? TickDeltaUs(base_tick_us, imu_tick_us) : 0U;
      std::memcpy(payload.data() + cursor, &tick_delta_us,
                  sizeof(tick_delta_us));
      cursor = static_cast<uint16_t>(cursor + sizeof(tick_delta_us));

      std::array<int16_t, 3> acc_mg = {0, 0, 0};
      std::array<int16_t, 4> quat_q15 = {0, 0, 0, 0};
      std::array<int16_t, 3> mag_raw = {0, 0, 0};
      if (valid) {
        acc_mg = {AccMps2ToMilliG(imu.acc[0]), AccMps2ToMilliG(imu.acc[1]),
                  AccMps2ToMilliG(imu.acc[2])};
        quat_q15 = {QuatToQ15(imu.quaternion[0]), QuatToQ15(imu.quaternion[1]),
                    QuatToQ15(imu.quaternion[2]),
                    QuatToQ15(imu.quaternion[3])};
        mag_raw = {FloatToInt16Clamped(imu.mag[0]),
                   FloatToInt16Clamped(imu.mag[1]),
                   FloatToInt16Clamped(imu.mag[2])};
      }
      for (const int16_t value : acc_mg) {
        std::memcpy(payload.data() + cursor, &value, sizeof(value));
        cursor = static_cast<uint16_t>(cursor + sizeof(value));
      }
      for (const int16_t value : quat_q15) {
        std::memcpy(payload.data() + cursor, &value, sizeof(value));
        cursor = static_cast<uint16_t>(cursor + sizeof(value));
      }
      for (const int16_t value : mag_raw) {
        std::memcpy(payload.data() + cursor, &value, sizeof(value));
        cursor = static_cast<uint16_t>(cursor + sizeof(value));
      }

      ++payload[0];
      payload_len =
          static_cast<uint16_t>(payload_len + kBridgeWITCompactRecordSize);
    }

    PublishResult result = PublishResult::NONE;
    if (payload[0] > 0) {
      if (SendResponse(kBridgeCmdPosePush, payload.data(), payload_len)) {
        last_pose_push_ms_ = now_ms;
        result = PublishResult::SENT;
      } else {
        result = PublishResult::BACKPRESSURE;
      }
    }
    wit_subscriber_->StartWaiting();
    return result;
  }

  if (config_.pose_source == BridgePoseSource::YIS) {
    if (yis_queue_ == nullptr) {
      has_latest_yis_pose_ = false;
      return PublishResult::NONE;
    }

    Manager::YISPoseMsg yis_msg;
    bool got_new_yis_pose = false;
    while (yis_queue_->Pop(yis_msg) == ErrorCode::OK) {
      if (yis_msg.status == 0U) {
        latest_yis_pose_ = yis_msg;
        has_latest_yis_pose_ = true;
        got_new_yis_pose = true;
      }
    }

    if (!got_new_yis_pose) {
      has_latest_yis_pose_ = false;
      return PublishResult::NONE;
    }

    const Manager::YISPoseMsg& latest = latest_yis_pose_;
    std::array<uint8_t, 1 + kBridgeYISExtendedPoseRecordSize> payload{};
    payload[0] = 1;
    payload[1] = kYISBridgeAddress;

    uint16_t cursor = 2;
    const std::array<float, kBridgePoseFloatCount> values = {
        latest.euler[0],      latest.euler[1],      latest.euler[2],
        latest.quaternion[0], latest.quaternion[1], latest.quaternion[2],
        latest.quaternion[3]};
    for (const float value : values) {
      std::memcpy(payload.data() + cursor, &value, sizeof(float));
      cursor = static_cast<uint16_t>(cursor + sizeof(float));
    }
    std::memcpy(payload.data() + cursor, &latest.sample_timestamp,
                sizeof(latest.sample_timestamp));
    cursor = static_cast<uint16_t>(cursor + sizeof(latest.sample_timestamp));
    std::memcpy(payload.data() + cursor, &latest.sensor_mcu_tick_us,
                sizeof(latest.sensor_mcu_tick_us));
    cursor = static_cast<uint16_t>(cursor + sizeof(latest.sensor_mcu_tick_us));
    std::memcpy(payload.data() + cursor, &latest.readout_mcu_tick_us,
                sizeof(latest.readout_mcu_tick_us));
    cursor = static_cast<uint16_t>(cursor + sizeof(latest.readout_mcu_tick_us));
    payload[cursor++] = latest.time_status;

    if (SendResponse(kBridgeCmdPosePush, payload.data(), cursor)) {
      last_pose_push_ms_ = now_ms;
      has_latest_yis_pose_ = false;
      return PublishResult::SENT;
    }

    return PublishResult::BACKPRESSURE;
  }

  if (feyman_subscriber_ == nullptr || !feyman_subscriber_->Available()) {
    return PublishResult::NONE;
  }

  g_bridge_feyman_snapshot = feyman_subscriber_->GetData();
  feyman_subscriber_->StartWaiting();

  auto& payload = g_bridge_feyman_payload;
  uint8_t valid_count = 0U;
  uint16_t cursor = 1U;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  const uint8_t device_count = static_cast<uint8_t>(
      std::min<size_t>(g_bridge_feyman_snapshot.device_count,
                       g_bridge_feyman_snapshot.devices.size()));
  for (uint8_t i = 0; i < device_count; ++i) {
    const auto& device = g_bridge_feyman_snapshot.devices[i];
    if (device.online == 0U || device.configured == 0U || device.status != 0U) {
      continue;
    }
    if ((static_cast<size_t>(cursor) + kBridgeExtendedRecordSize) >
        payload.size()) {
      break;
    }

    payload[cursor++] = device.node_id;
    const std::array<float, kBridgeExtendedFloatCount> values = {
        nan,          nan,          nan,          device.acc[0], device.acc[1],
        device.acc[2], device.gyro[0], device.gyro[1], device.gyro[2], nan,
        nan,          nan,          nan};
    for (const float value : values) {
      std::memcpy(payload.data() + cursor, &value, sizeof(float));
      cursor = static_cast<uint16_t>(cursor + sizeof(float));
    }
    ++valid_count;
  }

  if (valid_count == 0U) {
    return PublishResult::NONE;
  }

  payload[0] = valid_count;

  if (SendResponse(kBridgeCmdPosePush, payload.data(), cursor)) {
    last_pose_push_ms_ = now_ms;
    return PublishResult::SENT;
  }

  return PublishResult::BACKPRESSURE;
}

void IMUUartBridgeTask::PublishSyncEvents() {
  if (!streaming_enabled_) {
    pending_sync_event_count_ = 0U;
    return;
  }

  std::array<uint8_t, kBridgeSyncEventMaxPayload> payload{};
  uint16_t cursor = 1;

  if (pending_sync_event_count_ == 0U) {
    Manager::SyncEventRecord event;
    while (pending_sync_event_count_ < kBridgeSyncEventMaxRecords &&
           Manager::SyncSignalManager::PopEvent(event)) {
      pending_sync_events_[pending_sync_event_count_++] = event;
    }
  }

  if (pending_sync_event_count_ == 0U) {
    return;
  }

  for (uint8_t i = 0; i < pending_sync_event_count_; ++i) {
    const auto& event = pending_sync_events_[i];
    payload[cursor++] = static_cast<uint8_t>(event.source);
    payload[cursor++] = event.flags;
    std::memcpy(payload.data() + cursor, &event.reserved,
                sizeof(event.reserved));
    cursor = static_cast<uint16_t>(cursor + sizeof(event.reserved));
    std::memcpy(payload.data() + cursor, &event.sequence,
                sizeof(event.sequence));
    cursor = static_cast<uint16_t>(cursor + sizeof(event.sequence));
    std::memcpy(payload.data() + cursor, &event.mcu_tick_us,
                sizeof(event.mcu_tick_us));
    cursor = static_cast<uint16_t>(cursor + sizeof(event.mcu_tick_us));
    std::memcpy(payload.data() + cursor, &event.nominal_period_us,
                sizeof(event.nominal_period_us));
    cursor = static_cast<uint16_t>(cursor + sizeof(event.nominal_period_us));
    std::memcpy(payload.data() + cursor, &event.dropped_count,
                sizeof(event.dropped_count));
    cursor = static_cast<uint16_t>(cursor + sizeof(event.dropped_count));
  }

  payload[0] = pending_sync_event_count_;
  if (SendResponse(kBridgeCmdSyncEventPush, payload.data(), cursor)) {
    pending_sync_event_count_ = 0U;
  }
}

bool IMUUartBridgeTask::SetStreamingEnabled(bool enable) {
  if (enable == streaming_enabled_) {
    return true;
  }

  if (enable) {
    ClearPendingPushData();
    pending_sync_event_count_ = 0U;
    const LibXR::ErrorCode ec =
        Manager::SyncSignalManager::StartRegisteredOutputs();
    if (ec != LibXR::ErrorCode::OK) {
      return false;
    }
    ClearPendingPoseData();
    last_pose_push_ms_ = Thread::GetTime();
    streaming_enabled_ = true;
    return true;
  }

  streaming_enabled_ = false;
  const LibXR::ErrorCode ec =
      Manager::SyncSignalManager::StopRegisteredOutputs();
  ClearPendingPushData();
  pending_sync_event_count_ = 0U;
  return ec == LibXR::ErrorCode::OK;
}

void IMUUartBridgeTask::ClearPendingPushData() {
  ClearPendingPoseData();
  Manager::SyncEventRecord sync_event;
  while (Manager::SyncSignalManager::PopEvent(sync_event)) {
  }
}

void IMUUartBridgeTask::ClearPendingPoseData() {
  if (wit_subscriber_ != nullptr && wit_subscriber_->Available()) {
    wit_subscriber_->StartWaiting();
  }
  if (yis_queue_ != nullptr) {
    Manager::YISPoseMsg yis_msg;
    while (yis_queue_->Pop(yis_msg) == ErrorCode::OK) {
    }
  }
  if (feyman_subscriber_ != nullptr && feyman_subscriber_->Available()) {
    (void)feyman_subscriber_->GetData();
    feyman_subscriber_->StartWaiting();
  }
  has_latest_yis_pose_ = false;
}

bool IMUUartBridgeTask::WriteString(const char* str) {
  if (str == nullptr) {
    return false;
  }

  size_t len = 0;
  while (str[len] != '\0') {
    ++len;
  }
  return WriteExact(reinterpret_cast<const uint8_t*>(str),
                    static_cast<uint16_t>(len));
}

bool IMUUartBridgeTask::WriteExact(const uint8_t* buf, uint16_t len) {
  if (len == 0) {
    return true;
  }

  for (uint32_t attempt = 0; attempt < kBridgeWriteRetryCount; ++attempt) {
    WriteOperation op;
    if (uart_->Write({buf, len}, op) == ErrorCode::OK) {
      return true;
    }
    if ((attempt + 1U) < kBridgeWriteRetryCount) {
      Thread::Sleep(kBridgeWriteRetryDelayMs);
    }
  }
  return false;
}

bool IMUUartBridgeTask::WriteSPI(const uint8_t* buf, uint16_t len) {
  if (spi_ == nullptr) {
    return false;
  }
  if (len == 0) {
    return true;
  }

  Semaphore sem(0);
  SPI::OperationRW op(sem);
  return spi_->Write({buf, len}, op) == ErrorCode::OK;
}

bool IMUUartBridgeTask::SendResponse(uint8_t cmd, const uint8_t* payload,
                                     uint16_t len) {
  if (len > kBridgeMaxResponsePayload) {
    return false;
  }

  auto& frame = g_bridge_response_frame;
  frame[0] = kBridgeSof0;
  frame[1] = kBridgeSof1;
  frame[2] = cmd;
  frame[3] = static_cast<uint8_t>(len & 0xFFU);
  frame[4] = static_cast<uint8_t>((len >> 8U) & 0xFFU);
  if (len > 0 && payload != nullptr) {
    std::memcpy(frame.data() + 5, payload, len);
  }
  frame[5 + len] = static_cast<uint8_t>(
      CalcSum(frame.data(), static_cast<uint16_t>(5 + len)));

  return WriteExact(frame.data(), static_cast<uint16_t>(6 + len));
}

void IMUUartBridgeTask::HandleCommandFrame(uint8_t cmd, const uint8_t* payload,
                                           uint16_t payload_len) {
  switch (cmd) {
    case kBridgeCmdPing:
      HandlePing(cmd, payload, payload_len);
      return;
    case kBridgeCmdTimeSync:
      HandleTimeSync(cmd, payload, payload_len);
      return;
    case kBridgeCmdStreamControl:
      HandleStreamControl(cmd, payload, payload_len);
      return;
    case kBridgeCmdI2CRead:
      HandleI2CRead(cmd, payload, payload_len);
      return;
    case kBridgeCmdI2CWrite:
      HandleI2CWrite(cmd, payload, payload_len);
      return;
    case kBridgeCmdSPIWrite:
      HandleSPIWrite(cmd, payload, payload_len);
      return;
    case kBridgeCmdWS2812Control:
      HandleWS2812Control(cmd, payload, payload_len);
      return;
    default:
      uint8_t resp = kBridgeStatusError;
      (void)SendResponse(cmd, &resp, 1);
      return;
  }
}

void IMUUartBridgeTask::HandlePing(uint8_t cmd, const uint8_t* payload,
                                   uint16_t payload_len) {
  (void)payload;
  if (payload_len != 0) {
    uint8_t resp = kBridgeStatusError;
    (void)SendResponse(cmd, &resp, 1);
    return;
  }
  const uint8_t ok = kBridgeStatusOk;
  (void)SendResponse(cmd, &ok, 1);
}

void IMUUartBridgeTask::HandleTimeSync(uint8_t cmd, const uint8_t* payload,
                                       uint16_t payload_len) {
  if (payload == nullptr || payload_len != kBridgeTimeSyncSeqSize) {
    uint8_t resp = kBridgeStatusError;
    (void)SendResponse(cmd, &resp, 1);
    return;
  }

  uint32_t seq = 0U;
  std::memcpy(&seq, payload, sizeof(seq));
  const uint64_t t2_mcu_tick_us = Timebase::GetMicroseconds();
  const uint64_t t3_mcu_tick_us = Timebase::GetMicroseconds();
  const uint64_t t2_session_tick_us =
      Manager::SyncSignalManager::ToSessionTickUs(t2_mcu_tick_us);
  const uint64_t t3_session_tick_us =
      Manager::SyncSignalManager::ToSessionTickUs(t3_mcu_tick_us);

  std::array<uint8_t, kBridgeTimeSyncRespPayloadSize> resp{};
  resp[0] = kBridgeStatusOk;
  uint16_t cursor = 1;
  std::memcpy(resp.data() + cursor, &seq, sizeof(seq));
  cursor = static_cast<uint16_t>(cursor + sizeof(seq));
  std::memcpy(resp.data() + cursor, &t2_session_tick_us,
              sizeof(t2_session_tick_us));
  cursor = static_cast<uint16_t>(cursor + sizeof(t2_session_tick_us));
  std::memcpy(resp.data() + cursor, &t3_session_tick_us,
              sizeof(t3_session_tick_us));
  cursor = static_cast<uint16_t>(cursor + sizeof(t3_session_tick_us));
  (void)SendResponse(cmd, resp.data(), cursor);
}

void IMUUartBridgeTask::HandleStreamControl(uint8_t cmd,
                                            const uint8_t* payload,
                                            uint16_t payload_len) {
  uint8_t resp_status = kBridgeStatusError;
  uint32_t request_id = 0U;
  bool command_valid = false;
  bool enable = false;

  if (payload != nullptr &&
      payload_len >= kBridgeStreamControlSeqSize + 4U) {
    std::memcpy(&request_id, payload, sizeof(request_id));
    const char* action =
        reinterpret_cast<const char*>(payload + kBridgeStreamControlSeqSize);
    const uint16_t action_len =
        static_cast<uint16_t>(payload_len - kBridgeStreamControlSeqSize);
    if (action_len == 5U && std::memcmp(action, "start", 5U) == 0) {
      command_valid = true;
      enable = true;
    } else if (action_len == 4U && std::memcmp(action, "stop", 4U) == 0) {
      command_valid = true;
      enable = false;
    }
  } else if (payload != nullptr && payload_len == 5U &&
             std::memcmp(payload, "start", 5U) == 0) {
    command_valid = true;
    enable = true;
  } else if (payload != nullptr && payload_len == 4U &&
             std::memcmp(payload, "stop", 4U) == 0) {
    command_valid = true;
    enable = false;
  }

  if (command_valid && SetStreamingEnabled(enable)) {
    resp_status = kBridgeStatusOk;
  }

  std::array<uint8_t, kBridgeStreamControlRespPayloadSize> resp{};
  resp[0] = resp_status;
  std::memcpy(resp.data() + 1, &request_id, sizeof(request_id));
  resp[1 + sizeof(request_id)] = streaming_enabled_ ? 1U : 0U;
  (void)SendResponse(cmd, resp.data(), resp.size());
}

void IMUUartBridgeTask::HandleI2CRead(uint8_t cmd, const uint8_t* payload,
                                      uint16_t payload_len) {
  if (payload == nullptr || payload_len != 4U) {
    uint8_t resp = kBridgeStatusError;
    (void)SendResponse(cmd, &resp, 1);
    return;
  }

  const uint8_t bus = payload[0];
  const uint8_t addr = payload[1];
  const uint8_t reg = payload[2];
  const uint8_t count = payload[3];

  std::array<uint8_t, 1 + kMaxRegisterCount * 2> resp{};
  resp[0] = kBridgeStatusError;
  uint16_t resp_len = 1;

  if (bus == config_.i2c_bus && addr == config_.imu_addr &&
      count <= kMaxRegisterCount) {
    resp_len = static_cast<uint16_t>(1 + count * 2U);
    Semaphore sem(0);
    ReadOperation op(sem, config_.read_timeout_ms);
    const bool locked = imu_mgr_->AcquireBus();
    if (locked) {
      const auto ec = i2c_->MemRead(static_cast<uint16_t>(addr) << 1U, reg,
                                    {resp.data() + 1, count * 2U}, op);
      imu_mgr_->ReleaseBus();
      if (ec == ErrorCode::OK) {
        resp[0] = kBridgeStatusOk;
      }
    }
  }

  (void)SendResponse(cmd, resp.data(), resp_len);
}

void IMUUartBridgeTask::HandleI2CWrite(uint8_t cmd, const uint8_t* payload,
                                       uint16_t payload_len) {
  if (payload == nullptr || payload_len != 5U) {
    uint8_t resp = kBridgeStatusError;
    (void)SendResponse(cmd, &resp, 1);
    return;
  }

  const uint8_t bus = payload[0];
  const uint8_t addr = payload[1];
  const uint8_t reg = payload[2];
  const uint8_t data[2] = {payload[3], payload[4]};
  uint8_t resp = kBridgeStatusError;

  if (bus == config_.i2c_bus && addr == config_.imu_addr) {
    Semaphore sem(0);
    WriteOperation op(sem);
    const bool locked = imu_mgr_->AcquireBus();
    if (locked) {
      const auto ec = i2c_->MemWrite(static_cast<uint16_t>(addr) << 1U, reg,
                                     {data, sizeof(data)}, op);
      imu_mgr_->ReleaseBus();
      if (ec == ErrorCode::OK) {
        resp = kBridgeStatusOk;
      }
    }
  }

  (void)SendResponse(cmd, &resp, 1);
}

void IMUUartBridgeTask::HandleSPIWrite(uint8_t cmd, const uint8_t* payload,
                                       uint16_t payload_len) {
  uint8_t resp = kBridgeStatusError;

  if (payload == nullptr || payload_len < 4U) {
    (void)SendResponse(cmd, &resp, 1);
    return;
  }

  const uint16_t spi_len = static_cast<uint16_t>(payload[2]) |
                           (static_cast<uint16_t>(payload[3]) << 8U);
  const uint16_t data_len = static_cast<uint16_t>(payload_len - 4U);

  const auto tx_buf =
      (spi_ != nullptr) ? spi_->GetTxBuffer() : RawData(nullptr, 0);
  auto* tx_ptr = reinterpret_cast<uint8_t*>(tx_buf.addr_);
  const bool frame_ok = (spi_len == data_len) && (tx_ptr != nullptr) &&
                        (spi_len <= tx_buf.size_);

  if (frame_ok) {
    if (spi_len > 0U) {
      std::memcpy(tx_ptr, payload + 4, spi_len);
    }
  }

  if (frame_ok && payload[0] == config_.spi_bus && WriteSPI(tx_ptr, spi_len)) {
    resp = kBridgeStatusOk;
  }
  (void)SendResponse(cmd, &resp, 1);
}

void IMUUartBridgeTask::HandleWS2812Control(uint8_t cmd, const uint8_t* payload,
                                            uint16_t payload_len) {
  uint8_t resp = kBridgeStatusError;

  if (payload == nullptr || payload_len != 7U) {
    (void)SendResponse(cmd, &resp, 1);
    return;
  }

  const uint8_t target = payload[0];
  const uint8_t flags = payload[1];
  const bool blink_enable = (flags & 0x01U) != 0U;
  const uint16_t interval_ms = static_cast<uint16_t>(payload[5]) |
                               (static_cast<uint16_t>(payload[6]) << 8U);
  const bool flags_ok = (flags & 0xFEU) == 0U;

  if (flags_ok && ws2812_mgr_ != nullptr &&
      ws2812_mgr_->SetLightControl(target, payload[2], payload[3], payload[4],
                                   blink_enable, interval_ms) ==
          ErrorCode::OK) {
    resp = kBridgeStatusOk;
  }
  (void)SendResponse(cmd, &resp, 1);
}

uint8_t IMUUartBridgeTask::CalcSum(const uint8_t* buf, uint16_t len) const {
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + buf[i]);
  }
  return sum;
}

}  // namespace Application
