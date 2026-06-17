#pragma once

#include <cstdint>

#include "can.hpp"
#include "libxr_def.hpp"
#include "message.hpp"
#include "modules/feyman_mcs10/feyman_mcs10.hpp"
#include "thread.hpp"

namespace Application {

struct FeymanCanopenConfig {
  uint8_t node_id = 0x7F;
  uint32_t baudrate = 250000;
  uint32_t data_rate_hz = 20;
  uint32_t heartbeat_ms = 1000;
  uint32_t priority =
      static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  uint32_t stack_size = 2048;
  uint32_t sdo_timeout_ms = 200;
  uint32_t sdo_inter_request_delay_ms = 5;
  uint32_t work_mode_settle_ms = 50;
  uint32_t startup_delay_ms = 50;
  bool verbose_config_log = true;
  const char* topic_name = "feyman_imu_pose";
  void (*log_writer)(const char* text) = nullptr;
};

class FeymanCanopenTask {
 public:
  explicit FeymanCanopenTask(
      LibXR::CAN* can,
      const FeymanCanopenConfig& config = FeymanCanopenConfig{});
  ~FeymanCanopenTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

 private:
  static void TaskEntry(FeymanCanopenTask* task);

  void Run();
  void PublishPoseIfReady();
  void FlushPendingCanErrorLog();
  void Log(const char* text);
  void LogConfig(const char* text);
  void Logf(const char* fmt, ...);
  void LogConfigf(const char* fmt, ...);
  void LogCanErrorState(const char* context);
  Module::FeymanMCS10Config BuildDeviceConfig() const;

  LibXR::CAN* can_;
  FeymanCanopenConfig config_;
  LibXR::Thread thread_{};
  LibXR::Topic* topic_ = nullptr;
  Module::FeymanMCS10 device_{};
  volatile bool running_ = false;
  bool device_configured_ = false;
};

}  // namespace Application
