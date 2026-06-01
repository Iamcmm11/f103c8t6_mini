#include "yis_imu_acquisition_task.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "managers/sync_signal_manager.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace Application {

namespace {

constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;
constexpr uint8_t kYISTimeStatusHasEpoch = 0x01;
constexpr uint8_t kYISTimeStatusSampleWrapSeen = 0x02;
constexpr uint8_t kYISTimeStatusEpochMismatch = 0x04;

uint32_t FrequencyToPeriodMs(uint32_t frequency_hz) {
  if (frequency_hz == 0U) {
    return 0U;
  }

  uint32_t period_ms = 1000U / frequency_hz;
  if (period_ms == 0U) {
    period_ms = 1U;
  }
  return period_ms;
}

int32_t FloatToMicro(float value) {
  const float scaled = value * 1000000.0f;
  return static_cast<int32_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

int32_t AbsI32(int32_t value) {
  return (value < 0) ? -value : value;
}

int AppendFixed6(char* out, size_t size, int32_t micro) {
  if (out == nullptr || size == 0U) {
    return 0;
  }

  const char sign = (micro < 0) ? '-' : '+';
  const int32_t abs_value = AbsI32(micro);
  return std::snprintf(out, size, "%c%ld.%06ld", sign,
                       static_cast<long>(abs_value / 1000000L),
                       static_cast<long>(abs_value % 1000000L));
}

}  // namespace

YISIMUAcquisitionTask::YISIMUAcquisitionTask(
    Module::YISIMU* imu, const YISIMUAcquisitionConfig& config,
    LibXR::GPIO* dr_gpio)
    : imu_(imu), config_(config), dr_gpio_(dr_gpio) {}

LibXR::ErrorCode YISIMUAcquisitionTask::Start() {
  if (running_) {
    return LibXR::ErrorCode::BUSY;
  }
  if (imu_ == nullptr || config_.topic_name == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }
  if (config_.frequency_hz == 0U || config_.stack_size == 0U) {
    return LibXR::ErrorCode::ARG_ERR;
  }

  const size_t required_heap = static_cast<size_t>(config_.stack_size) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return LibXR::ErrorCode::NO_MEM;
  }

  if (topic_ != nullptr) {
    delete topic_;
    topic_ = nullptr;
  }
  topic_ = new LibXR::Topic(config_.topic_name,
                            sizeof(Manager::YISPoseMsg), nullptr, false,
                            false, false);
  if (topic_ == nullptr) {
    return LibXR::ErrorCode::NO_MEM;
  }

  running_ = true;
  first_failure_logged_ = false;
  sample_count_ = 0;
  last_dr_mcu_tick_us_ = 0;
  last_sample_timestamp_ = 0;
  while (dr_sem_.Wait(0U) == LibXR::ErrorCode::OK) {
  }
  if (dr_gpio_ != nullptr) {
    dr_gpio_->RegisterCallback(
        LibXR::GPIO::Callback::Create(OnDrInterrupt, this));
    const auto ec = dr_gpio_->EnableInterrupt();
    if (ec != LibXR::ErrorCode::OK) {
      running_ = false;
      delete topic_;
      topic_ = nullptr;
      return ec;
    }
  }
  thread_.Create(this, TaskEntry, "YISIMU", config_.stack_size,
                 static_cast<LibXR::Thread::Priority>(config_.priority));
  return LibXR::ErrorCode::OK;
}

void YISIMUAcquisitionTask::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;
  dr_sem_.Post();
  if (dr_gpio_ != nullptr) {
    (void)dr_gpio_->DisableInterrupt();
  }
  LibXR::Thread::Sleep(FrequencyToPeriodMs(config_.frequency_hz) + 1U);
}

void YISIMUAcquisitionTask::OnDrInterrupt(bool in_isr,
                                          YISIMUAcquisitionTask* task) {
  if (task != nullptr) {
    if (in_isr) {
      const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
      task->last_dr_mcu_tick_us_ = LibXR::Timebase::GetMicroseconds();
      taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
    } else {
      taskENTER_CRITICAL();
      task->last_dr_mcu_tick_us_ = LibXR::Timebase::GetMicroseconds();
      taskEXIT_CRITICAL();
    }
    task->dr_sem_.PostFromCallback(in_isr);
  }
}

void YISIMUAcquisitionTask::TaskEntry(YISIMUAcquisitionTask* task) {
  if (task != nullptr) {
    task->Run();
  }
}

void YISIMUAcquisitionTask::Run() {
  const uint32_t period_ms = FrequencyToPeriodMs(config_.frequency_hz);
  LibXR::MillisecondTimestamp last_wakeup(LibXR::Thread::GetTime());
  while (running_) {
    if (dr_gpio_ != nullptr) {
      if (dr_sem_.Wait(config_.dr_wait_timeout_ms) != LibXR::ErrorCode::OK) {
        if (!running_) {
          break;
        }
        continue;
      }
    }

    Manager::YISPoseMsg msg;
    msg.timestamp_us = LibXR::Timebase::GetMicroseconds();
    taskENTER_CRITICAL();
    const uint64_t last_dr_mcu_tick_us = last_dr_mcu_tick_us_;
    taskEXIT_CRITICAL();
    msg.readout_mcu_tick_us =
        (last_dr_mcu_tick_us != 0U) ? last_dr_mcu_tick_us : msg.timestamp_us;
    msg.sensor_mcu_tick_us = 0U;
    msg.euler[0] = 0.0f;
    msg.euler[1] = 0.0f;
    msg.euler[2] = 0.0f;
    msg.quaternion[0] = 1.0f;
    msg.quaternion[1] = 0.0f;
    msg.quaternion[2] = 0.0f;
    msg.quaternion[3] = 0.0f;
    msg.sample_timestamp = 0U;
    msg.time_status = 0U;

    int32_t raw_quat[4] = {0, 0, 0, 0};
    float norm_sq = 0.0f;
    const auto quat_ec = imu_->ReadQuaternion(msg.quaternion, raw_quat, &norm_sq);
    const auto euler_ec = imu_->ReadEuler(msg.euler);
    const auto sample_timestamp_ec =
        imu_->ReadSampleTimestamp(&msg.sample_timestamp);
    if (quat_ec == LibXR::ErrorCode::OK && euler_ec == LibXR::ErrorCode::OK) {
      msg.status = 0U;
      first_failure_logged_ = false;
    } else {
      const auto ec =
          (quat_ec != LibXR::ErrorCode::OK) ? quat_ec : euler_ec;
      msg.status = static_cast<uint8_t>(-static_cast<int8_t>(ec));
      if (!first_failure_logged_) {
        LogFailure(ec, raw_quat, norm_sq);
        if (ec == LibXR::ErrorCode::CHECK_ERR) {
          LogProbeRegisters();
        }
        first_failure_logged_ = true;
      }
    }

    if (msg.status == 0U && sample_timestamp_ec != LibXR::ErrorCode::OK) {
      msg.sample_timestamp = 0U;
    }
    if (msg.status == 0U && sample_timestamp_ec == LibXR::ErrorCode::OK) {
      Manager::SyncEventRecord epoch;
      if (Manager::SyncSignalManager::GetLatestEvent(
              Manager::SyncEventSource::TIM2_IMU_SYNC_1HZ, epoch)) {
        msg.time_status = static_cast<uint8_t>(msg.time_status |
                                               kYISTimeStatusHasEpoch);
        const int64_t sensor_tick =
            static_cast<int64_t>(epoch.mcu_tick_us) +
            static_cast<int64_t>(msg.sample_timestamp) +
            static_cast<int64_t>(config_.yis_epoch_offset_us);
        msg.sensor_mcu_tick_us =
            (sensor_tick > 0) ? static_cast<uint64_t>(sensor_tick) : 0U;

        const uint64_t epoch_end =
            epoch.mcu_tick_us + epoch.nominal_period_us + 2000U;
        if (msg.sensor_mcu_tick_us > epoch_end) {
          msg.time_status = static_cast<uint8_t>(msg.time_status |
                                                 kYISTimeStatusEpochMismatch);
        }
      }
      if (sample_count_ != 0U &&
          msg.sample_timestamp < last_sample_timestamp_) {
        msg.time_status = static_cast<uint8_t>(msg.time_status |
                                               kYISTimeStatusSampleWrapSeen);
      }
      last_sample_timestamp_ = msg.sample_timestamp;
    }

    if (topic_ != nullptr) {
      topic_->Publish(msg);
    }

    ++sample_count_;
    if (msg.status == 0U && config_.log_interval != 0U &&
        (sample_count_ % config_.log_interval) == 0U) {
      LogEuler(msg.euler, msg.status);
    }

    if (!running_) {
      break;
    }
    if (dr_gpio_ == nullptr) {
      LibXR::Thread::SleepUntil(last_wakeup, period_ms);
    }
  }
}

void YISIMUAcquisitionTask::Log(const char* text) {
  if (config_.log_writer != nullptr) {
    config_.log_writer(text);
  }
}

void YISIMUAcquisitionTask::LogEuler(const float euler_rpy[3], uint8_t status) {
  if (euler_rpy == nullptr) {
    return;
  }

  char line[96] = {0};
  char roll[16] = {0};
  char pitch[16] = {0};
  char yaw[16] = {0};
  AppendFixed6(roll, sizeof(roll), FloatToMicro(euler_rpy[0]));
  AppendFixed6(pitch, sizeof(pitch), FloatToMicro(euler_rpy[1]));
  AppendFixed6(yaw, sizeof(yaw), FloatToMicro(euler_rpy[2]));
  std::snprintf(line, sizeof(line), "[yis] rpy=[%s,%s,%s] st=%u", roll, pitch,
                yaw, static_cast<unsigned>(status));
  Log(line);
}

void YISIMUAcquisitionTask::LogQuaternion(
    const Manager::YISPoseMsg& msg) {
  char line[128] = {0};
  char q0[16] = {0};
  char q1[16] = {0};
  char q2[16] = {0};
  char q3[16] = {0};
  AppendFixed6(q0, sizeof(q0), FloatToMicro(msg.quaternion[0]));
  AppendFixed6(q1, sizeof(q1), FloatToMicro(msg.quaternion[1]));
  AppendFixed6(q2, sizeof(q2), FloatToMicro(msg.quaternion[2]));
  AppendFixed6(q3, sizeof(q3), FloatToMicro(msg.quaternion[3]));
  std::snprintf(line, sizeof(line), "[yis] q=[%s,%s,%s,%s] status=%u", q0,
                q1, q2, q3, static_cast<unsigned>(msg.status));
  Log(line);
}

void YISIMUAcquisitionTask::LogFailure(LibXR::ErrorCode ec,
                                       const int32_t raw_quat[4],
                                       float norm_sq) {
  char line[160] = {0};
  if (raw_quat != nullptr && ec == LibXR::ErrorCode::CHECK_ERR) {
    const int32_t norm_micro = FloatToMicro(norm_sq);
    char norm[16] = {0};
    AppendFixed6(norm, sizeof(norm), norm_micro);
    std::snprintf(line, sizeof(line),
                  "[yis] invalid quat raw=[%ld,%ld,%ld,%ld] norm2=%s",
                  static_cast<long>(raw_quat[0]),
                  static_cast<long>(raw_quat[1]),
                  static_cast<long>(raw_quat[2]),
                  static_cast<long>(raw_quat[3]), norm);
  } else {
    std::snprintf(line, sizeof(line), "[yis] read failed ec=%d",
                  static_cast<int>(ec));
  }
  Log(line);
}

void YISIMUAcquisitionTask::LogProbeRegisters() {
  uint8_t acc[12] = {0};
  uint8_t euler[12] = {0};
  const auto acc_ec = imu_->ReadRegister(0x10, acc, sizeof(acc));
  const auto euler_ec = imu_->ReadRegister(0x40, euler, sizeof(euler));

  char line[192] = {0};
  std::snprintf(line, sizeof(line),
                "[yis] probe acc_ec=%d acc=%02X%02X%02X%02X %02X%02X%02X%02X "
                "%02X%02X%02X%02X",
                static_cast<int>(acc_ec), acc[0], acc[1], acc[2], acc[3],
                acc[4], acc[5], acc[6], acc[7], acc[8], acc[9], acc[10],
                acc[11]);
  Log(line);

  std::snprintf(line, sizeof(line),
                "[yis] probe euler_ec=%d euler=%02X%02X%02X%02X "
                "%02X%02X%02X%02X %02X%02X%02X%02X",
                static_cast<int>(euler_ec), euler[0], euler[1], euler[2],
                euler[3], euler[4], euler[5], euler[6], euler[7], euler[8],
                euler[9], euler[10], euler[11]);
  Log(line);
}

}  // namespace Application
