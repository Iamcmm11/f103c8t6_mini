#include "imu_uart_bridge_task.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <limits>

#include "FreeRTOS.h"
#include "semaphore.hpp"
#include "task.h"

namespace {

// 桥接协议帧头：用于从串口字节流里定位一帧完整消息。
constexpr uint8_t kBridgeSof0 = 0x55;
constexpr uint8_t kBridgeSof1 = 0xAA;
// 上位机可发起的命令类型。
constexpr uint8_t kBridgeCmdPing = 0x01;
constexpr uint8_t kBridgeCmdI2CRead = 0x10;
constexpr uint8_t kBridgeCmdI2CWrite = 0x11;
constexpr uint8_t kBridgeCmdSPIWrite = 0x20;
constexpr uint8_t kBridgeCmdWS2812Control = 0x21;
// 板端主动推送使用的命令号。
constexpr uint8_t kBridgeCmdIMUEulerPush = 0x30;
constexpr uint8_t kBridgeCmdIMUDiagPush = 0x31;
// 当前桥接只挑选这几个关键槽位上送给上位机。
constexpr std::array<uint8_t, 4> kBridgeIMUPushIndexes = {
    static_cast<uint8_t>(Manager::ImuSlot::Forearm),
    static_cast<uint8_t>(Manager::ImuSlot::Hand),
    static_cast<uint8_t>(Manager::ImuSlot::ThumbRoot),
    static_cast<uint8_t>(Manager::ImuSlot::ThumbTip),
};
constexpr uint8_t kBridgeIMUPushFloatCount = 13;
constexpr uint8_t kBridgeIMUPushRecordSize =
    static_cast<uint8_t>(1 + kBridgeIMUPushFloatCount * sizeof(float));
constexpr uint8_t kBridgeStatusOk = 0;
constexpr uint8_t kBridgeStatusError = 1;
constexpr uint8_t kMaxRegisterCount = 32;
constexpr uint16_t kBridgeMaxResponsePayload = static_cast<uint16_t>(
    1 + kBridgeIMUPushIndexes.size() * kBridgeIMUPushRecordSize);
constexpr uint16_t kBridgeChunkSize = 32;
constexpr uint32_t kBridgeDiagIntervalMs = 1000;
constexpr uint32_t kBridgePollTimeoutMs = 2;
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;

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
      last_bridge_push_ms_(0),
      last_diag_push_ms_(0),
      last_valid_mask_(0),
      last_sequence_(0),
      thread_(nullptr),
      imu_subscriber_(nullptr) {}

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
  // 在真正建任务前先确认 FreeRTOS 剩余堆足够，避免启动阶段出现隐蔽内存问题。
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
  running_ = false;
  Thread::Sleep(config_.read_timeout_ms + 10);
}

void IMUUartBridgeTask::TaskEntry(IMUUartBridgeTask* arg) {
  if (arg != nullptr) {
    arg->Run();
  }
}

void IMUUartBridgeTask::Run() {
  // 同一个桥接任务支持两种模式：文本流模式和二进制桥接协议模式。
  if (config_.stream_relative_euler) {
    RunStreamMode();
  } else {
    RunBridgeMode();
  }
}

void IMUUartBridgeTask::RunStreamMode() {
  // 文本模式只关心 imu_data 主题，并把第一个 IMU 的欧拉角输出成 CSV。
  auto topic = Topic::Find("imu_data");
  if (topic != nullptr) {
    imu_subscriber_ =
        new Topic::ASyncSubscriber<Manager::IMUArrayMsg>(Topic(topic));
  }
  if (imu_subscriber_ != nullptr) {
    imu_subscriber_->StartWaiting();
  }
  (void)WriteString("roll_deg,pitch_deg,yaw_deg\r\n");
  if (imu_subscriber_ == nullptr) {
    (void)WriteString("# imu_data topic unavailable\r\n");
  }

  while (running_) {
    if (imu_subscriber_ == nullptr) {
      Thread::Sleep(20);
      continue;
    }
    if (imu_subscriber_->Available()) {
      auto& imu_msg = imu_subscriber_->GetData();
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
      imu_subscriber_->StartWaiting();
    } else {
      // 没有新数据时主动让出 CPU，避免桥接线程忙轮询。
      Thread::Sleep(1);
    }
  }

  delete imu_subscriber_;
  imu_subscriber_ = nullptr;
}

void IMUUartBridgeTask::RunBridgeMode() {
  if (config_.push_imu_euler_in_bridge) {
    // 桥接模式下额外订阅 imu_data，用于主动向上位机推送 IMU 帧。
    auto topic = Topic::Find("imu_data");
    if (topic != nullptr) {
      imu_subscriber_ =
          new Topic::ASyncSubscriber<Manager::IMUArrayMsg>(Topic(topic));
    }
    if (imu_subscriber_ != nullptr) {
      imu_subscriber_->StartWaiting();
      last_bridge_push_ms_ = Thread::GetTime();
    }
  }

  while (running_) {
    // 每轮先处理主动推送，再处理可能到来的上位机命令。
    PublishBridgeIMUData();
    PublishBridgeDiag();

    uint8_t byte = 0;
    if (!ReadByte(byte, kBridgePollTimeoutMs)) {
      continue;
    }
    if (byte != kBridgeSof0) {
      continue;
    }
    if (!ReadByte(byte, config_.read_timeout_ms) || byte != kBridgeSof1) {
      continue;
    }

    uint8_t hdr[5] = {kBridgeSof0, kBridgeSof1, 0, 0, 0};
    if (!ReadExact(&hdr[2], 3, config_.read_timeout_ms)) {
      continue;
    }

    const uint16_t payload_len = static_cast<uint16_t>(hdr[3]) |
                                 (static_cast<uint16_t>(hdr[4]) << 8U);
    // 头部解析完成后，交给统一命令分发器处理。
    HandleCommand(hdr[2], payload_len);
  }

  delete imu_subscriber_;
  imu_subscriber_ = nullptr;
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

bool IMUUartBridgeTask::ReadByte(uint8_t& byte, uint32_t timeout_ms) {
  return ReadExact(&byte, 1, timeout_ms);
}

bool IMUUartBridgeTask::ReadExact(uint8_t* buf, uint16_t len,
                                  uint32_t timeout_ms) {
  if (len == 0) {
    return true;
  }

  const uint32_t start_ms = Thread::GetTime();
  while (running_) {
    if (uart_->read_port_ != nullptr && uart_->read_port_->Size() >= len) {
      ReadOperation op;
      return uart_->Read({buf, len}, op) == ErrorCode::OK;
    }

    if ((Thread::GetTime() - start_ms) >= timeout_ms) {
      return false;
    }

    // 这里用“短睡眠 + 超时判断”的方式轮询串口缓冲，兼顾实时性和 CPU 占用。
    Thread::Sleep(1);
  }

  return false;
}

bool IMUUartBridgeTask::ReadChunked(uint8_t* buf, uint16_t len, uint8_t& sum) {
  // 大负载按固定小块读取，避免一次读太多导致缓冲和超时处理都变得笨重。
  uint16_t offset = 0;
  while (offset < len) {
    const uint16_t chunk = static_cast<uint16_t>(
        (len - offset) > kBridgeChunkSize ? kBridgeChunkSize : (len - offset));
    if (!ReadExact(buf + offset, chunk, config_.read_timeout_ms)) {
      return false;
    }
    sum = static_cast<uint8_t>(sum + CalcSum(buf + offset, chunk));
    offset = static_cast<uint16_t>(offset + chunk);
  }
  return true;
}

bool IMUUartBridgeTask::DiscardChunked(uint16_t len, uint8_t& sum) {
  std::array<uint8_t, kBridgeChunkSize> drop{};
  uint16_t offset = 0;
  while (offset < len) {
    const uint16_t chunk = static_cast<uint16_t>(
        (len - offset) > kBridgeChunkSize ? kBridgeChunkSize : (len - offset));
    if (!ReadExact(drop.data(), chunk, config_.read_timeout_ms)) {
      return false;
    }
    sum = static_cast<uint8_t>(sum + CalcSum(drop.data(), chunk));
    offset = static_cast<uint16_t>(offset + chunk);
  }
  return true;
}

bool IMUUartBridgeTask::WriteExact(const uint8_t* buf, uint16_t len) {
  if (len == 0) {
    return true;
  }

  Semaphore sem(0);
  WriteOperation op(sem);
  // 这里虽然接口看起来同步，但底层通常是 DMA + 中断完成通知。
  return uart_->Write({buf, len}, op) == ErrorCode::OK;
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

void IMUUartBridgeTask::SendResponse(uint8_t cmd, const uint8_t* payload,
                                     uint16_t len) {
  if (len > kBridgeMaxResponsePayload) {
    return;
  }

  std::array<uint8_t, 5 + kBridgeMaxResponsePayload + 1> frame{};
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

  // 统一按“帧头 + 命令 + 长度 + 负载 + 校验和”的格式回包。
  (void)WriteExact(frame.data(), static_cast<uint16_t>(6 + len));
}

void IMUUartBridgeTask::HandleCommand(uint8_t cmd, uint16_t payload_len) {
  const uint8_t hdr[5] = {kBridgeSof0, kBridgeSof1, cmd,
                          static_cast<uint8_t>(payload_len & 0xFFU),
                          static_cast<uint8_t>((payload_len >> 8U) & 0xFFU)};
  const uint8_t sum = CalcSum(hdr, sizeof(hdr));

  // 根据命令字分派给不同硬件资源；未知命令则丢弃负载并返回错误状态。
  switch (cmd) {
    case kBridgeCmdPing:
      HandlePing(cmd, payload_len, sum);
      return;
    case kBridgeCmdI2CRead:
      HandleI2CRead(cmd, payload_len, sum);
      return;
    case kBridgeCmdI2CWrite:
      HandleI2CWrite(cmd, payload_len, sum);
      return;
    case kBridgeCmdSPIWrite:
      HandleSPIWrite(cmd, payload_len, sum);
      return;
    case kBridgeCmdWS2812Control:
      HandleWS2812Control(cmd, payload_len, sum);
      return;
    default: {
      uint8_t frame_sum = sum;
      if (!DiscardChunked(payload_len, frame_sum) || !ReadChecksum(frame_sum)) {
        return;
      }
      uint8_t resp = kBridgeStatusError;
      SendResponse(cmd, &resp, 1);
      return;
    }
  }
}

void IMUUartBridgeTask::HandlePing(uint8_t cmd, uint16_t payload_len,
                                   uint8_t sum) {
  if (payload_len != 0) {
    if (!DiscardChunked(payload_len, sum) || !ReadChecksum(sum)) {
      return;
    }
    uint8_t resp = kBridgeStatusError;
    SendResponse(cmd, &resp, 1);
    return;
  }
  if (!ReadChecksum(sum)) {
    return;
  }
  const uint8_t ok = kBridgeStatusOk;
  SendResponse(cmd, &ok, 1);
}

void IMUUartBridgeTask::HandleI2CRead(uint8_t cmd, uint16_t payload_len,
                                      uint8_t sum) {
  uint8_t payload[4] = {0};
  if (payload_len != sizeof(payload)) {
    if (!DiscardChunked(payload_len, sum) || !ReadChecksum(sum)) {
      return;
    }
    uint8_t resp = kBridgeStatusError;
    SendResponse(cmd, &resp, 1);
    return;
  }
  if (!ReadChunked(payload, sizeof(payload), sum) || !ReadChecksum(sum)) {
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
    // 通过 IMUManager 统一仲裁 I2C，总线不会和后台采集线程发生冲突。
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

  SendResponse(cmd, resp.data(), resp_len);
}

void IMUUartBridgeTask::HandleI2CWrite(uint8_t cmd, uint16_t payload_len,
                                       uint8_t sum) {
  uint8_t payload[5] = {0};
  if (payload_len != sizeof(payload)) {
    if (!DiscardChunked(payload_len, sum) || !ReadChecksum(sum)) {
      return;
    }
    uint8_t resp = kBridgeStatusError;
    SendResponse(cmd, &resp, 1);
    return;
  }
  if (!ReadChunked(payload, sizeof(payload), sum) || !ReadChecksum(sum)) {
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
    // 写寄存器前同样要先拿到 I2C 总线互斥权限。
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

  SendResponse(cmd, &resp, 1);
}

void IMUUartBridgeTask::HandleSPIWrite(uint8_t cmd, uint16_t payload_len,
                                       uint8_t sum) {
  uint8_t meta[4] = {0};
  uint8_t resp = kBridgeStatusError;

  if (payload_len < sizeof(meta)) {
    if (!DiscardChunked(payload_len, sum) || !ReadChecksum(sum)) {
      return;
    }
    SendResponse(cmd, &resp, 1);
    return;
  }
  if (!ReadChunked(meta, sizeof(meta), sum)) {
    return;
  }

  const uint16_t spi_len = static_cast<uint16_t>(meta[2]) |
                           (static_cast<uint16_t>(meta[3]) << 8U);
  const uint16_t data_len = static_cast<uint16_t>(payload_len - sizeof(meta));

  const auto tx_buf =
      (spi_ != nullptr) ? spi_->GetTxBuffer() : RawData(nullptr, 0);
  auto* tx_ptr = reinterpret_cast<uint8_t*>(tx_buf.addr_);
  // 只有长度匹配且底层 SPI 发送缓冲足够时，才允许真正下发这帧数据。
  const bool frame_ok = (spi_len == data_len) && (tx_ptr != nullptr) &&
                        (spi_len <= tx_buf.size_);

  if (frame_ok) {
    if (!ReadChunked(tx_ptr, spi_len, sum) || !ReadChecksum(sum)) {
      return;
    }
  } else {
    if (!DiscardChunked(data_len, sum) || !ReadChecksum(sum)) {
      return;
    }
  }

  if (frame_ok && meta[0] == config_.spi_bus && WriteSPI(tx_ptr, spi_len)) {
    resp = kBridgeStatusOk;
  }
  SendResponse(cmd, &resp, 1);
}

void IMUUartBridgeTask::HandleWS2812Control(uint8_t cmd, uint16_t payload_len,
                                            uint8_t sum) {
  uint8_t payload[7] = {0};
  uint8_t resp = kBridgeStatusError;

  if (payload_len != sizeof(payload)) {
    if (!DiscardChunked(payload_len, sum) || !ReadChecksum(sum)) {
      return;
    }
    SendResponse(cmd, &resp, 1);
    return;
  }
  if (!ReadChunked(payload, sizeof(payload), sum) || !ReadChecksum(sum)) {
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
  SendResponse(cmd, &resp, 1);
}

void IMUUartBridgeTask::PublishBridgeIMUData() {
  if (!config_.push_imu_euler_in_bridge || imu_subscriber_ == nullptr) {
    return;
  }
  if (!imu_subscriber_->Available()) {
    return;
  }

  auto& imu_msg = imu_subscriber_->GetData();
  last_valid_mask_ = imu_msg.valid_mask;
  last_sequence_ = imu_msg.sequence;
  const uint32_t now_ms = Thread::GetTime();
  const bool interval_ok =
      (config_.stream_interval_ms == 0U) ||
      ((now_ms - last_bridge_push_ms_) >= config_.stream_interval_ms);

  if (interval_ok) {
    // 组帧时优先携带姿态、加速度、角速度和四元数，方便上位机一次拿到完整状态。
    std::array<uint8_t,
               1 + kBridgeIMUPushIndexes.size() * kBridgeIMUPushRecordSize>
        payload{};
    payload[0] = 0;
    uint16_t payload_len = 1;

    for (const uint8_t imu_index : kBridgeIMUPushIndexes) {
      const bool valid = imu_msg.IsValid(imu_index);
      if (!valid && !config_.push_all_slots_in_bridge) {
        continue;
      }

      // 某个槽位没有数据时填 NaN，明确告诉上位机这个槽位当前无效。
      std::array<float, kBridgeIMUPushFloatCount> values{};
      values.fill(std::numeric_limits<float>::quiet_NaN());
      if (valid) {
        const auto& imu = imu_msg.imu_data[imu_index];
        values = {imu.angle[0],      imu.angle[1],      imu.angle[2],
                  imu.acc[0],        imu.acc[1],        imu.acc[2],
                  imu.gyro[0],       imu.gyro[1],       imu.gyro[2],
                  imu.quaternion[0], imu.quaternion[1], imu.quaternion[2],
                  imu.quaternion[3]};
      }

      const uint16_t base = payload_len;
      payload[base] =
          Manager::ResolveImuI2CAddress(imu_index, config_.imu_addr);
      uint16_t cursor = static_cast<uint16_t>(base + 1);
      for (const float value : values) {
        std::memcpy(payload.data() + cursor, &value, sizeof(float));
        cursor = static_cast<uint16_t>(cursor + sizeof(float));
      }
      ++payload[0];
      payload_len =
          static_cast<uint16_t>(payload_len + kBridgeIMUPushRecordSize);
    }

    if (payload[0] > 0) {
      SendResponse(kBridgeCmdIMUEulerPush, payload.data(), payload_len);
      last_bridge_push_ms_ = now_ms;
    }
  }

  // 消费掉当前这帧消息后重新进入等待状态，准备接收下一次 Topic 发布。
  imu_subscriber_->StartWaiting();
}

void IMUUartBridgeTask::PublishBridgeDiag() {
  const uint32_t now_ms = Thread::GetTime();
  if (last_diag_push_ms_ != 0U &&
      ((now_ms - last_diag_push_ms_) < kBridgeDiagIntervalMs)) {
    return;
  }

  uint16_t online_mask = 0;
  const uint8_t configured_count = imu_mgr_->GetConfiguredCount();
  for (uint8_t i = 0; i < configured_count; ++i) {
    if (imu_mgr_->IsIMUOnline(i)) {
      online_mask = static_cast<uint16_t>(online_mask | (1u << i));
    }
  }

  uint8_t valid_count = 0;
  for (uint8_t i = 0; i < Manager::ACTUAL_IMU_COUNT; ++i) {
    if ((last_valid_mask_ & (1u << i)) != 0U) {
      ++valid_count;
    }
  }

  // 诊断帧主要用于让上位机快速了解采集序号、在线掩码、有效掩码和容量信息。
  std::array<uint8_t, 11> payload{};
  payload[0] = static_cast<uint8_t>(last_sequence_ & 0xFFU);
  payload[1] = static_cast<uint8_t>((last_sequence_ >> 8U) & 0xFFU);
  payload[2] = static_cast<uint8_t>((last_sequence_ >> 16U) & 0xFFU);
  payload[3] = static_cast<uint8_t>((last_sequence_ >> 24U) & 0xFFU);
  payload[4] = static_cast<uint8_t>(online_mask & 0xFFU);
  payload[5] = static_cast<uint8_t>((online_mask >> 8U) & 0xFFU);
  payload[6] = static_cast<uint8_t>(last_valid_mask_ & 0xFFU);
  payload[7] = static_cast<uint8_t>((last_valid_mask_ >> 8U) & 0xFFU);
  payload[8] = imu_mgr_->GetOnlineCount();
  payload[9] = valid_count;
  payload[10] = Manager::IMUArrayMsg::GetCapacity();

  SendResponse(kBridgeCmdIMUDiagPush, payload.data(),
               static_cast<uint16_t>(payload.size()));
  last_diag_push_ms_ = now_ms;
}

bool IMUUartBridgeTask::ReadChecksum(uint8_t expected_sum) {
  uint8_t rx_sum = 0;
  return ReadByte(rx_sum, config_.read_timeout_ms) && rx_sum == expected_sum;
}

uint8_t IMUUartBridgeTask::CalcSum(const uint8_t* buf, uint16_t len) const {
  // 当前桥接协议使用最简单的逐字节累加和，便于 MCU 和上位机两侧快速实现。
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + buf[i]);
  }
  return sum;
}

}  // namespace Application
