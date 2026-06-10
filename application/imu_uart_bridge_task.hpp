#pragma once

#include <array>
#include <cstdint>

#include "i2c.hpp"
#include "libxr_def.hpp"
#include "lockfree_queue.hpp"
#include "managers/imu_manager.hpp"
#include "managers/sync_signal_manager.hpp"
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
  bool push_sync_events_in_bridge = true;
  bool push_all_slots_in_bridge = true;
  uint8_t wit_push_imu_count = Manager::ACTUAL_IMU_COUNT;
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
  bool PublishGPIOButtonCommand(char command);

  static bool PublishGPIOButtonCommandCallback(void* context, char command);

 private:
  static void TaskEntry(IMUUartBridgeTask* arg);
  enum class CommandParserState : uint8_t {
    WAIT_SOF0,
    WAIT_SOF1,
    READ_CMD,
    READ_LEN0,
    READ_LEN1,
    READ_PAYLOAD,
    READ_CHECKSUM,
  };
  enum class PublishResult : uint8_t {
    NONE,
    SENT,
    BACKPRESSURE,
  };

  void Run();
  void RunStreamMode();
  void RunBridgeMode();
  PublishResult PublishPendingGPIOButtonCommand();
  bool ProcessPendingCommand();
  void ProcessCommandByte(uint8_t byte);
  void ResetCommandParser();
  PublishResult PublishBridgePoseData();
  void PublishSyncEvents();
  bool SetStreamingEnabled(bool enable);
  void ClearPendingPushData();
  void ClearPendingPoseData();
  bool WriteString(const char* str);
  bool WriteExact(const uint8_t* buf, uint16_t len);
  bool WriteSPI(const uint8_t* buf, uint16_t len);
  bool SendResponse(uint8_t cmd, const uint8_t* payload, uint16_t len);
  void HandleCommandFrame(uint8_t cmd, const uint8_t* payload,
                          uint16_t payload_len);
  void HandlePing(uint8_t cmd, const uint8_t* payload, uint16_t payload_len);
  void HandleI2CRead(uint8_t cmd, const uint8_t* payload,
                     uint16_t payload_len);
  void HandleI2CWrite(uint8_t cmd, const uint8_t* payload,
                      uint16_t payload_len);
  void HandleSPIWrite(uint8_t cmd, const uint8_t* payload,
                      uint16_t payload_len);
  void HandleWS2812Control(uint8_t cmd, const uint8_t* payload,
                           uint16_t payload_len);
  void HandleTimeSync(uint8_t cmd, const uint8_t* payload,
                      uint16_t payload_len);
  void HandleStreamControl(uint8_t cmd, const uint8_t* payload,
                           uint16_t payload_len);
  uint8_t CalcSum(const uint8_t* buf, uint16_t len) const;

  static constexpr uint16_t kCommandPayloadBufferSize = 1024;

  LibXR::UART* uart_;
  LibXR::I2C* i2c_;
  LibXR::SPI* spi_;
  Manager::IMUManager* imu_mgr_;
  Manager::WS2812Manager* ws2812_mgr_;
  IMUUartBridgeConfig config_;
  bool running_;
  uint32_t last_pose_push_ms_;
  bool streaming_enabled_;
  LibXR::Thread* thread_;
  LibXR::Topic::ASyncSubscriber<Manager::IMUArrayMsg>* wit_subscriber_;
  LibXR::LockFreeQueue<Manager::YISPoseMsg>* yis_queue_;
  LibXR::Topic::QueuedSubscriber* yis_queue_subscriber_;
  LibXR::LockFreeQueue<uint8_t> gpio_button_queue_;
  CommandParserState command_parser_state_ = CommandParserState::WAIT_SOF0;
  std::array<uint8_t, kCommandPayloadBufferSize> command_payload_{};
  uint16_t command_payload_len_ = 0;
  uint16_t command_payload_pos_ = 0;
  uint8_t command_cmd_ = 0;
  uint8_t command_sum_ = 0;
  uint8_t pending_gpio_button_command_ = 0;
  bool has_pending_gpio_button_command_ = false;
  Manager::YISPoseMsg latest_yis_pose_{};
  bool has_latest_yis_pose_ = false;
  std::array<Manager::SyncEventRecord, 8> pending_sync_events_{};
  uint8_t pending_sync_event_count_ = 0;
};

}  // namespace Application
