#include "imu_uart_bridge_task.hpp"

#include <array>
#include <cstdio>
#include <cstring>

#include "semaphore.hpp"

namespace {

constexpr uint8_t kBridgeSof0 = 0x55;
constexpr uint8_t kBridgeSof1 = 0xAA;
constexpr uint8_t kBridgeCmdPing = 0x01;
constexpr uint8_t kBridgeCmdI2CRead = 0x10;
constexpr uint8_t kBridgeCmdI2CWrite = 0x11;
constexpr uint8_t kBridgeCmdSPIWrite = 0x20;
constexpr uint8_t kBridgeCmdWS2812Frame = 0x21;
constexpr uint8_t kBridgeCmdIMUEulerPush = 0x30;
constexpr uint8_t kBridgeStatusOk = 0;
constexpr uint8_t kBridgeStatusError = 1;
constexpr uint8_t kMaxRegisterCount = 32;
constexpr uint16_t kBridgeChunkSize = 32;
constexpr uint32_t kBridgePollTimeoutMs = 2;

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
      thread_(nullptr),
      imu_subscriber_(nullptr) {}

ErrorCode IMUUartBridgeTask::Start() {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (uart_ == nullptr || i2c_ == nullptr || imu_mgr_ == nullptr) {
    return ErrorCode::PTR_NULL;
  }

  running_ = true;
  thread_ = new Thread();
  if (thread_ == nullptr) {
    running_ = false;
    return ErrorCode::NO_MEM;
  }

  // Thread::Create expects stack size in bytes.
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
  if (config_.stream_relative_euler) {
    RunStreamMode();
  } else {
    RunBridgeMode();
  }
}

void IMUUartBridgeTask::RunStreamMode() {
  imu_subscriber_ = new Topic::ASyncSubscriber<Manager::IMUArrayMsg>("imu_data");
  imu_subscriber_->StartWaiting();
  (void)WriteString("roll_deg,pitch_deg,yaw_deg\r\n");

  while (running_) {
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
      Thread::Sleep(1);
    }
  }

  delete imu_subscriber_;
  imu_subscriber_ = nullptr;
}

void IMUUartBridgeTask::RunBridgeMode() {
  if (config_.push_imu_euler_in_bridge) {
    imu_subscriber_ =
        new Topic::ASyncSubscriber<Manager::IMUArrayMsg>("imu_data");
    if (imu_subscriber_ != nullptr) {
      imu_subscriber_->StartWaiting();
      last_bridge_push_ms_ = Thread::GetTime();
    }
  }

  while (running_) {
    PublishBridgeIMUData();

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

    const uint16_t payload_len =
        static_cast<uint16_t>(hdr[3]) |
        (static_cast<uint16_t>(hdr[4]) << 8U);
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

    Thread::Sleep(1);
  }

  return false;
}

bool IMUUartBridgeTask::ReadChunked(uint8_t* buf, uint16_t len, uint8_t& sum) {
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
  const uint8_t hdr[5] = {kBridgeSof0, kBridgeSof1, cmd,
                          static_cast<uint8_t>(len & 0xFFU),
                          static_cast<uint8_t>((len >> 8U) & 0xFFU)};
  const uint8_t checksum = static_cast<uint8_t>(
      CalcSum(hdr, sizeof(hdr)) + CalcSum(payload, len));

  if (!WriteExact(hdr, sizeof(hdr))) {
    return;
  }
  if (len > 0 && !WriteExact(payload, len)) {
    return;
  }
  (void)WriteExact(&checksum, 1);
}

void IMUUartBridgeTask::HandleCommand(uint8_t cmd, uint16_t payload_len) {
  const uint8_t hdr[5] = {kBridgeSof0, kBridgeSof1, cmd,
                          static_cast<uint8_t>(payload_len & 0xFFU),
                          static_cast<uint8_t>((payload_len >> 8U) & 0xFFU)};
  const uint8_t sum = CalcSum(hdr, sizeof(hdr));

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
    case kBridgeCmdWS2812Frame:
      HandleWS2812Frame(cmd, payload_len, sum);
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
    const bool locked = imu_mgr_->AcquireBus();
    if (locked) {
      const auto ec =
          i2c_->MemRead(static_cast<uint16_t>(addr) << 1U, reg,
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

  const uint16_t spi_len =
      static_cast<uint16_t>(meta[2]) |
      (static_cast<uint16_t>(meta[3]) << 8U);
  const uint16_t data_len = static_cast<uint16_t>(payload_len - sizeof(meta));

  const auto tx_buf = (spi_ != nullptr) ? spi_->GetTxBuffer() : RawData(nullptr, 0);
  auto* tx_ptr = reinterpret_cast<uint8_t*>(tx_buf.addr_);
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

void IMUUartBridgeTask::HandleWS2812Frame(uint8_t cmd, uint16_t payload_len,
                                          uint8_t sum) {
  uint8_t meta[3] = {0};
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

  const uint16_t led_count =
      static_cast<uint16_t>(meta[1]) |
      (static_cast<uint16_t>(meta[2]) << 8U);
  const uint16_t rgb_len = static_cast<uint16_t>(payload_len - sizeof(meta));

  const auto tx_buf = (spi_ != nullptr) ? spi_->GetTxBuffer() : RawData(nullptr, 0);
  auto* tx_ptr = reinterpret_cast<uint8_t*>(tx_buf.addr_);
  const bool frame_ok = (tx_ptr != nullptr) && (rgb_len <= tx_buf.size_);

  if (frame_ok) {
    if (!ReadChunked(tx_ptr, rgb_len, sum) || !ReadChecksum(sum)) {
      return;
    }
  } else {
    if (!DiscardChunked(rgb_len, sum) || !ReadChecksum(sum)) {
      return;
    }
  }

  if (frame_ok && ws2812_mgr_ != nullptr &&
      ws2812_mgr_->ShowFrame(meta[0], tx_ptr, rgb_len, led_count) ==
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
  const uint32_t now_ms = Thread::GetTime();
  const bool interval_ok =
      (config_.stream_interval_ms == 0U) ||
      ((now_ms - last_bridge_push_ms_) >= config_.stream_interval_ms);

  if (interval_ok && imu_msg.IsValid(0)) {
    const auto& imu = imu_msg.imu_data[0];
    uint8_t payload[13] = {0};
    payload[0] = config_.imu_addr;
    std::memcpy(&payload[1], &imu.angle[0], sizeof(float));
    std::memcpy(&payload[5], &imu.angle[1], sizeof(float));
    std::memcpy(&payload[9], &imu.angle[2], sizeof(float));
    SendResponse(kBridgeCmdIMUEulerPush, payload, sizeof(payload));
    last_bridge_push_ms_ = now_ms;
  }

  imu_subscriber_->StartWaiting();
}

bool IMUUartBridgeTask::ReadChecksum(uint8_t expected_sum) {
  uint8_t rx_sum = 0;
  return ReadByte(rx_sum, config_.read_timeout_ms) && rx_sum == expected_sum;
}

uint8_t IMUUartBridgeTask::CalcSum(const uint8_t* buf, uint16_t len) const {
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + buf[i]);
  }
  return sum;
}

}  // namespace Application
