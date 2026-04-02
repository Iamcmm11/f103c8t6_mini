#pragma once

#include <cstdint>

#include "imu_manager.hpp"
#include "message.hpp"
#include "thread.hpp"

namespace Application {

struct IMUAcquisitionConfig {
  uint32_t period_ms = 10;
  uint32_t priority = 3;
  uint32_t stack_size = 512;
};

class IMUAcquisitionTask {
 public:
  explicit IMUAcquisitionTask(
      Manager::IMUManager* imu_mgr,
      const IMUAcquisitionConfig& config = IMUAcquisitionConfig{});
  ~IMUAcquisitionTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

  uint32_t GetCount() const { return count_; }
  const IMUAcquisitionConfig& GetConfig() const { return config_; }

 private:
  static void TaskEntry(IMUAcquisitionTask* arg);
  void Run();
  void ProcessIMUData();

  Manager::IMUManager* imu_mgr_;
  IMUAcquisitionConfig config_;
  bool running_;
  uint32_t count_;
  LibXR::Thread* thread_;
  LibXR::Topic imu_topic_;
};

}  // namespace Application
