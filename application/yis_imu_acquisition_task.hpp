#pragma once

#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "managers/data_types.hpp"
#include "message.hpp"
#include "modules/yesense_yis_imu/yis_imu.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace Application {

struct YISIMUAcquisitionConfig {
  uint32_t frequency_hz = 50;
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  uint32_t stack_size = 1024;
  uint32_t log_interval = 100;
  uint32_t dr_wait_timeout_ms = 20;
  int32_t yis_epoch_offset_us = 0;
  const char* topic_name = "yis_imu_pose";
  void (*log_writer)(const char* text) = nullptr;
};

class YISIMUAcquisitionTask {
 public:
  explicit YISIMUAcquisitionTask(
      Module::YISIMU* imu,
      const YISIMUAcquisitionConfig& config = YISIMUAcquisitionConfig{},
      LibXR::GPIO* dr_gpio = nullptr);
  ~YISIMUAcquisitionTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

 private:
  static void OnDrInterrupt(bool in_isr, YISIMUAcquisitionTask* task);
  static void TaskEntry(YISIMUAcquisitionTask* task);
  void Run();
  void Log(const char* text);
  void LogEuler(const float euler_rpy[3], uint8_t status);
  void LogQuaternion(const Manager::YISPoseMsg& msg);
  void LogFailure(LibXR::ErrorCode ec, const int32_t raw_quat[4],
                  float norm_sq);
  void LogProbeRegisters();

  Module::YISIMU* imu_;
  YISIMUAcquisitionConfig config_;
  LibXR::GPIO* dr_gpio_;
  LibXR::Thread thread_;
  LibXR::Semaphore dr_sem_{0};
  LibXR::Topic* topic_ = nullptr;
  volatile uint64_t last_dr_mcu_tick_us_ = 0;
  uint32_t last_sample_timestamp_ = 0;
  volatile bool running_ = false;
  bool first_failure_logged_ = false;
  uint32_t sample_count_ = 0;
};

}  // namespace Application
