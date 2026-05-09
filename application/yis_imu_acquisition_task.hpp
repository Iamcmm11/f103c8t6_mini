#pragma once

#include <cstdint>

#include "libxr_def.hpp"
#include "managers/data_types.hpp"
#include "message.hpp"
#include "modules/yesense_yis_imu/yis_imu.hpp"
#include "thread.hpp"

namespace Application {

struct YISIMUAcquisitionConfig {
  uint32_t frequency_hz = 50;
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  uint32_t stack_size = 1024;
  uint32_t log_interval = 100;
  const char* topic_name = "yis_imu_quat";
  void (*log_writer)(const char* text) = nullptr;
};

class YISIMUAcquisitionTask {
 public:
  explicit YISIMUAcquisitionTask(
      Module::YISIMU* imu,
      const YISIMUAcquisitionConfig& config = YISIMUAcquisitionConfig{});
  ~YISIMUAcquisitionTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

 private:
  static void TaskEntry(YISIMUAcquisitionTask* task);
  void Run();
  void Log(const char* text);
  void LogQuaternion(const Manager::YISQuaternionMsg& msg);
  void LogFailure(LibXR::ErrorCode ec, const int32_t raw_quat[4],
                  float norm_sq);
  void LogProbeRegisters();

  Module::YISIMU* imu_;
  YISIMUAcquisitionConfig config_;
  LibXR::Thread thread_;
  LibXR::Topic* topic_ = nullptr;
  volatile bool running_ = false;
  bool first_failure_logged_ = false;
  uint32_t sample_count_ = 0;
};

}  // namespace Application
