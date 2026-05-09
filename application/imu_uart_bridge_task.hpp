#pragma once

#include <cstdint>

#include "i2c.hpp"
#include "libxr_def.hpp"
#include "managers/imu_manager.hpp"
#include "message.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "uart.hpp"
#include "ws2812_manager.hpp"

namespace Application {

enum class BridgePoseSource : uint8_t {
  WIT = 0,
  YIS = 1,
};

struct IMUUartBridgeConfig {
  uint8_t imu_addr = 0x50;
  uint8_t i2c_bus = 0;
  uint8_t spi_bus = 0;
  uint32_t read_timeout_ms = 50;
  bool stream_relative_euler = true;
  bool push_imu_euler_in_bridge = true;
  bool push_all_slots_in_bridge = true;
  BridgePoseSource pose_source = BridgePoseSource::WIT;
  uint32_t stream_interval_ms = 20;
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  uint32_t stack_size = 768;
};

class IMUUartBridgeTask {
 public:
  IMUUartBridgeTask(LibXR::UART* uart, LibXR::I2C* i2c, LibXR::SPI* spi,
                    Manager::IMUManager* imu_mgr,
                    Manager::WS2812Manager* ws2812_mgr,
                    const IMUUartBridgeConfig& config = IMUUartBridgeConfig{});
  ~IMUUartBridgeTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

 private:
  static void TaskEntry(IMUUartBridgeTask* arg);
  void Run();
  void RunStreamMode();
  void RunBridgeMode();
  void PublishBridgePoseData();
  bool WriteString(const char* str);
  bool ReadByte(uint8_t& byte, uint32_t timeout_ms);
  bool ReadExact(uint8_t* buf, uint16_t len, uint32_t timeout_ms);
  bool ReadChunked(uint8_t* buf, uint16_t len, uint8_t& sum);
  bool DiscardChunked(uint16_t len, uint8_t& sum);
  bool WriteExact(const uint8_t* buf, uint16_t len);
  bool WriteSPI(const uint8_t* buf, uint16_t len);
  void SendResponse(uint8_t cmd, const uint8_t* payload, uint16_t len);
  void HandleCommand(uint8_t cmd, uint16_t payload_len);
  void HandlePing(uint8_t cmd, uint16_t payload_len, uint8_t sum);
  void HandleI2CRead(uint8_t cmd, uint16_t payload_len, uint8_t sum);
  void HandleI2CWrite(uint8_t cmd, uint16_t payload_len, uint8_t sum);
  void HandleSPIWrite(uint8_t cmd, uint16_t payload_len, uint8_t sum);
  void HandleWS2812Control(uint8_t cmd, uint16_t payload_len, uint8_t sum);
  bool ReadChecksum(uint8_t expected_sum);
  uint8_t CalcSum(const uint8_t* buf, uint16_t len) const;

  LibXR::UART* uart_;
  LibXR::I2C* i2c_;
  LibXR::SPI* spi_;
  Manager::IMUManager* imu_mgr_;
  Manager::WS2812Manager* ws2812_mgr_;
  IMUUartBridgeConfig config_;
  bool running_;
  uint32_t last_pose_push_ms_;
  LibXR::Thread* thread_;
  LibXR::Topic::ASyncSubscriber<Manager::IMUArrayMsg>* wit_subscriber_;
  LibXR::Topic::ASyncSubscriber<Manager::YISPoseMsg>* yis_subscriber_;
};

}  // namespace Application
