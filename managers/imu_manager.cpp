#include "imu_manager.hpp"

#include <limits>

#include "FreeRTOS.h"
#include "libxr_rw.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace Manager {

using namespace LibXR;

namespace {

constexpr uint32_t kDataRegStart = 0x34;
constexpr uint32_t kDataRegCount = 13;
constexpr uint32_t kQuatRegStart = 0x51;
constexpr uint32_t kQuatRegCount = 4;
constexpr uint32_t kInitBootDelayMs = 120;
constexpr uint32_t kInitProbeRetryCount = 5;
constexpr uint32_t kInitProbeRetryDelayMs = 25;
constexpr uint32_t kAcquisitionStackBytes = 2048;
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;
constexpr float kDefaultVQFSampleHz = 100.0f;

}  // namespace

IMUManager::IMUManager(uint8_t imu_count)
    : i2c_(nullptr),
      imu_count_(imu_count > MAX_IMU_COUNT ? MAX_IMU_COUNT : imu_count),
      base_address_(kDefaultImuAddress),
      vqf_params_(),
      quaternion_source_(QuaternionSource::VQF),
      online_mask_(0),
      sequence_(0),
      data_topic_(nullptr),
      running_(false),
      frequency_hz_(100) {
  imus_.fill(nullptr);
  vqf_.fill(nullptr);
  InitVQF(static_cast<float>(frequency_hz_));
}

IMUManager::~IMUManager() {
  StopAcquisition();
  ReleaseVQF();

  for (auto*& imu : imus_) {
    delete imu;
    imu = nullptr;
  }
}

ErrorCode IMUManager::Init(I2C* i2c, uint8_t base_address) {
  if (i2c == nullptr) {
    return ErrorCode::PTR_NULL;
  }

  i2c_ = i2c;
  base_address_ = base_address;
  online_mask_ = 0;
  sequence_ = 0;

  Thread::Sleep(kInitBootDelayMs);

  uint8_t success_count = 0;
  for (uint8_t i = 0; i < imu_count_; ++i) {
    delete imus_[i];
    imus_[i] = new Module::WitIMU(i2c_, ResolveImuI2CAddress(i, base_address_));
    if (imus_[i] == nullptr) {
      continue;
    }

    if (imus_[i]->Init() != Module::WitIMU::ErrorCode::OK) {
      continue;
    }

    bool online = false;
    for (uint32_t attempt = 0; attempt < kInitProbeRetryCount; ++attempt) {
      if (ProbeIMU(i)) {
        online = true;
        break;
      }
      if ((attempt + 1U) < kInitProbeRetryCount) {
        Thread::Sleep(kInitProbeRetryDelayMs);
      }
    }

    if (online) {
      ++success_count;
    }
  }

  return (success_count > 0) ? ErrorCode::OK : ErrorCode::INIT_ERR;
}

ErrorCode IMUManager::ReadAll(IMUArrayMsg& msg) {
  Mutex::LockGuard guard(bus_mutex_);

  msg = IMUArrayMsg{};
  msg.sequence = sequence_++;

  uint8_t valid_count = 0;
  const uint8_t scan_count =
      (imu_count_ < ACTUAL_IMU_COUNT) ? imu_count_ : ACTUAL_IMU_COUNT;
  for (uint8_t i = 0; i < scan_count; ++i) {
    if (imus_[i] == nullptr || !IsIMUOnline(i)) {
      continue;
    }

    Module::WitIMU::ImuData raw{};
    if (!ReadRawIMU(i, raw)) {
      continue;
    }

    ConvertIMUData(raw, msg.imu_data[i]);
    ApplyQuaternionSource(i, msg.imu_data[i]);
    msg.imu_data[i].timestamp_us = Timebase::GetMicroseconds();
    msg.SetValid(i, true);
    ++valid_count;
  }

  msg.timestamp_us = Timebase::GetMicroseconds();
  return (valid_count > 0) ? ErrorCode::OK : ErrorCode::FAILED;
}

ErrorCode IMUManager::ReadSingle(uint8_t index, IMUData& data) {
  if (index >= imu_count_ || index >= ACTUAL_IMU_COUNT) {
    return ErrorCode::ARG_ERR;
  }
  if (imus_[index] == nullptr || !IsIMUOnline(index)) {
    return ErrorCode::NOT_FOUND;
  }

  Mutex::LockGuard guard(bus_mutex_);
  Module::WitIMU::ImuData raw{};
  if (!ReadRawIMU(index, raw)) {
    return ErrorCode::FAILED;
  }

  ConvertIMUData(raw, data);
  ApplyQuaternionSource(index, data);
  data.timestamp_us = Timebase::GetMicroseconds();
  return ErrorCode::OK;
}

ErrorCode IMUManager::StartAcquisition(uint32_t frequency_hz,
                                       const char* topic_name) {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (frequency_hz == 0U) {
    return ErrorCode::ARG_ERR;
  }
  if (online_mask_ == 0U) {
    return ErrorCode::INIT_ERR;
  }

  const size_t required_heap = static_cast<size_t>(kAcquisitionStackBytes) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  if (data_topic_ != nullptr) {
    delete data_topic_;
    data_topic_ = nullptr;
  }

  if (topic_name != nullptr) {
    data_topic_ =
        new Topic(topic_name, sizeof(IMUArrayMsg), nullptr, false, false, false);
    if (data_topic_ == nullptr) {
      return ErrorCode::NO_MEM;
    }
  }

  frequency_hz_ = frequency_hz;
  InitVQF(static_cast<float>(frequency_hz_));
  running_ = true;
  acquisition_thread_.Create(this, AcquisitionThreadFunc, "IMUMgrAcq",
                             kAcquisitionStackBytes, Thread::Priority::HIGH);
  return ErrorCode::OK;
}

void IMUManager::StopAcquisition() {
  if (!running_) {
    return;
  }

  running_ = false;

  uint32_t period_ms = (frequency_hz_ > 0U) ? (1000U / frequency_hz_) : 1U;
  if (period_ms == 0U) {
    period_ms = 1U;
  }
  Thread::Sleep(period_ms * 2U);

  if (data_topic_ != nullptr) {
    delete data_topic_;
    data_topic_ = nullptr;
  }
}

bool IMUManager::IsIMUOnline(uint8_t index) const {
  return (index < imu_count_) && ((online_mask_ & (1u << index)) != 0U);
}

uint8_t IMUManager::GetOnlineCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < imu_count_; ++i) {
    if (IsIMUOnline(i)) {
      ++count;
    }
  }
  return count;
}

void IMUManager::SetQuaternionSource(QuaternionSource source) {
  quaternion_source_ = source;
}

bool IMUManager::AcquireBus() { return bus_mutex_.Lock() == ErrorCode::OK; }

void IMUManager::ReleaseBus() { bus_mutex_.Unlock(); }

void IMUManager::InitVQF(float sample_hz) {
  if (sample_hz <= 0.0f) {
    sample_hz = kDefaultVQFSampleHz;
  }

  ReleaseVQF();

  vqf_params_ = ::VQFParams();
  vqf_params_.tauAcc = 2.0;
  vqf_params_.tauMag = 15.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
  vqf_params_.motionBiasEstEnabled = true;
#endif
  vqf_params_.restBiasEstEnabled = true;
  vqf_params_.magDistRejectionEnabled = true;
  vqf_params_.biasSigmaInit = 0.5;
  vqf_params_.biasForgettingTime = 50.0;
  vqf_params_.biasClip = 2.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
  vqf_params_.biasSigmaMotion = 0.05;
  vqf_params_.biasVerticalForgettingFactor = 0.0001;
#endif
  vqf_params_.biasSigmaRest = 0.02;
  vqf_params_.restMinT = 1.0;
  vqf_params_.restFilterTau = 0.5;
  vqf_params_.restThGyr = 3.0;
  vqf_params_.restThAcc = 0.8;
  vqf_params_.magCurrentTau = 0.08;
  vqf_params_.magRefTau = 30.0;
  vqf_params_.magNormTh = 0.08;
  vqf_params_.magDipTh = 8.0;
  vqf_params_.magNewTime = 60.0;
  vqf_params_.magNewFirstTime = 3.0;
  vqf_params_.magNewMinGyr = 15.0;
  vqf_params_.magMinUndisturbedTime = 0.2;
  vqf_params_.magMaxRejectionTime = 30.0;
  vqf_params_.magRejectionFactor = 1.2;

  const vqf_real_t sample_period_s =
      static_cast<vqf_real_t>(1.0 / static_cast<double>(sample_hz));
  for (uint8_t i = 0; i < imu_count_; ++i) {
    vqf_[i] = new ::VQF(vqf_params_, sample_period_s, -1.0, -1.0);
  }
}

void IMUManager::ReleaseVQF() {
  for (auto*& filter : vqf_) {
    delete filter;
    filter = nullptr;
  }
}

void IMUManager::ApplyQuaternionSource(uint8_t index, IMUData& data) {
  if (index >= MAX_IMU_COUNT || vqf_[index] == nullptr) {
    return;
  }

  const vqf_real_t gyr[3] = {
      static_cast<vqf_real_t>(data.gyro[0]),
      static_cast<vqf_real_t>(data.gyro[1]),
      static_cast<vqf_real_t>(data.gyro[2]),
  };
  const vqf_real_t acc[3] = {
      static_cast<vqf_real_t>(data.acc[0]),
      static_cast<vqf_real_t>(data.acc[1]),
      static_cast<vqf_real_t>(data.acc[2]),
  };
  const vqf_real_t mag[3] = {
      static_cast<vqf_real_t>(data.mag[0]),
      static_cast<vqf_real_t>(data.mag[1]),
      static_cast<vqf_real_t>(data.mag[2]),
  };

  vqf_[index]->update(gyr, acc, mag);
  if (quaternion_source_ != QuaternionSource::VQF) {
    return;
  }

  vqf_real_t q[4] = {1.0, 0.0, 0.0, 0.0};
  vqf_[index]->getQuat9D(q);
  data.quaternion[0] = static_cast<float>(q[0]);
  data.quaternion[1] = static_cast<float>(q[1]);
  data.quaternion[2] = static_cast<float>(q[2]);
  data.quaternion[3] = static_cast<float>(q[3]);
}

bool IMUManager::ReadRawIMU(uint8_t index,
                            Module::WitIMU::ImuData& raw_data) {
  if (index >= imu_count_ || imus_[index] == nullptr) {
    return false;
  }

  if (imus_[index]->ReadReg(kDataRegStart, kDataRegCount) !=
      Module::WitIMU::ErrorCode::OK) {
    return false;
  }

  const bool quat_ok =
      imus_[index]->ReadReg(kQuatRegStart, kQuatRegCount) ==
      Module::WitIMU::ErrorCode::OK;
  imus_[index]->UpdateDataFromRegisters();
  imus_[index]->GetData(raw_data);
  if (!quat_ok) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    raw_data.q0 = nan;
    raw_data.q1 = nan;
    raw_data.q2 = nan;
    raw_data.q3 = nan;
    raw_data.quat_valid = false;
  }

  return raw_data.acc_valid || raw_data.gyro_valid || raw_data.angle_valid;
}

bool IMUManager::ProbeIMU(uint8_t index) {
  if (index >= imu_count_ || imus_[index] == nullptr) {
    return false;
  }

  Module::WitIMU::ImuData raw{};
  if (!ReadRawIMU(index, raw)) {
    online_mask_ = static_cast<uint16_t>(online_mask_ & ~(1u << index));
    return false;
  }

  online_mask_ = static_cast<uint16_t>(online_mask_ | (1u << index));
  (void)imus_[index]->SetAxis9();
  return true;
}

void IMUManager::AcquisitionThreadFunc(IMUManager* manager) {
  if (manager == nullptr) {
    return;
  }

  uint32_t period_ms =
      (manager->frequency_hz_ > 0U) ? (1000U / manager->frequency_hz_) : 1U;
  if (period_ms == 0U) {
    period_ms = 1U;
  }

  MillisecondTimestamp last_wakeup(Thread::GetTime());
  while (manager->running_) {
    IMUArrayMsg msg;
    if (manager->ReadAll(msg) == ErrorCode::OK) {
      if (manager->data_topic_ != nullptr) {
        manager->data_topic_->Publish(msg);
      }
    }
    if (!manager->running_) {
      break;
    }
    Thread::SleepUntil(last_wakeup, period_ms);
  }
}

void IMUManager::ConvertIMUData(const Module::WitIMU::ImuData& src,
                                IMUData& dst) const {
  dst.acc[0] = src.acc_x * 9.80665f;
  dst.acc[1] = src.acc_y * 9.80665f;
  dst.acc[2] = src.acc_z * 9.80665f;

  constexpr float kDegToRad = 0.0174532925f;
  dst.gyro[0] = src.gyro_x * kDegToRad;
  dst.gyro[1] = src.gyro_y * kDegToRad;
  dst.gyro[2] = src.gyro_z * kDegToRad;

  dst.angle[0] = src.roll;
  dst.angle[1] = src.pitch;
  dst.angle[2] = src.yaw;

  dst.mag[0] = src.mag_x;
  dst.mag[1] = src.mag_y;
  dst.mag[2] = src.mag_z;
  dst.temperature = src.temperature;

  if (src.quat_valid) {
    dst.quaternion[0] = src.q0;
    dst.quaternion[1] = src.q1;
    dst.quaternion[2] = src.q2;
    dst.quaternion[3] = src.q3;
  } else {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    dst.quaternion[0] = nan;
    dst.quaternion[1] = nan;
    dst.quaternion[2] = nan;
    dst.quaternion[3] = nan;
  }

  dst.status = 0;
}

}  // namespace Manager
