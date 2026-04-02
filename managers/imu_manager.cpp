#include "imu_manager.hpp"

#include "stm32_timebase.hpp"

namespace Manager {

using namespace LibXR;

IMUManager::IMUManager(uint8_t imu_count)
    : i2c_(nullptr),
      imu_count_(imu_count > MAX_IMU_COUNT ? MAX_IMU_COUNT : imu_count),
      base_address_(kDefaultImuAddress),
      online_mask_(0),
      sequence_(0) {
  imus_.fill(nullptr);
}

IMUManager::~IMUManager() {
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

  uint8_t success_count = 0;
  for (uint8_t i = 0; i < imu_count_; ++i) {
    delete imus_[i];
    imus_[i] = new Module::WitIMU(i2c_, ResolveImuI2CAddress(i, base_address_));
    if (imus_[i] == nullptr) {
      continue;
    }

    if (imus_[i]->Init() == Module::WitIMU::ErrorCode::OK) {
      online_mask_ = static_cast<uint16_t>(online_mask_ | (1u << i));
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
  for (uint8_t i = 0; i < imu_count_ && i < ACTUAL_IMU_COUNT; ++i) {
    if (imus_[i] == nullptr || !IsIMUOnline(i)) {
      continue;
    }

    if (imus_[i]->ReadReg(0x34, 13) != Module::WitIMU::ErrorCode::OK) {
      continue;
    }

    imus_[i]->UpdateDataFromRegisters();
    Module::WitIMU::ImuData raw{};
    imus_[i]->GetData(raw);
    if (!(raw.acc_valid || raw.gyro_valid || raw.angle_valid)) {
      continue;
    }

    ConvertIMUData(raw, msg.imu_data[i]);
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
  if (imus_[index]->ReadReg(0x34, 13) != Module::WitIMU::ErrorCode::OK) {
    return ErrorCode::FAILED;
  }

  imus_[index]->UpdateDataFromRegisters();
  Module::WitIMU::ImuData raw{};
  imus_[index]->GetData(raw);
  if (!(raw.acc_valid || raw.gyro_valid || raw.angle_valid)) {
    return ErrorCode::FAILED;
  }

  ConvertIMUData(raw, data);
  data.timestamp_us = Timebase::GetMicroseconds();
  return ErrorCode::OK;
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

bool IMUManager::AcquireBus() { return bus_mutex_.Lock() == ErrorCode::OK; }

void IMUManager::ReleaseBus() { bus_mutex_.Unlock(); }

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
  dst.quaternion[0] = 1.0f;
  dst.quaternion[1] = 0.0f;
  dst.quaternion[2] = 0.0f;
  dst.quaternion[3] = 0.0f;
  dst.status = 0;
}

}  // namespace Manager
