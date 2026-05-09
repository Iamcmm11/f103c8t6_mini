#include "yis_imu.hpp"

#include <cmath>
#include <cstdint>

#include "libxr_rw.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace Module {

namespace {

constexpr uint32_t kInitBootDelayMs = 120;
constexpr uint32_t kInitProbeRetryCount = 5;
constexpr uint32_t kInitProbeRetryDelayMs = 25;

int32_t LoadLeI32(const uint8_t* data) {
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8U) |
                       (static_cast<uint32_t>(data[2]) << 16U) |
                       (static_cast<uint32_t>(data[3]) << 24U);
  return static_cast<int32_t>(raw);
}

uint32_t LoadLeU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

bool NormalizeQuaternion(float quat[4], float* norm_sq_out) {
  const float norm_sq = quat[0] * quat[0] + quat[1] * quat[1] +
                        quat[2] * quat[2] + quat[3] * quat[3];
  if (norm_sq_out != nullptr) {
    *norm_sq_out = norm_sq;
  }
  if (!std::isfinite(norm_sq) || norm_sq < 1.0e-12f) {
    return false;
  }

  const float inv_norm = 1.0f / std::sqrt(norm_sq);
  for (uint8_t i = 0; i < 4; ++i) {
    quat[i] *= inv_norm;
  }
  return std::isfinite(quat[0]) && std::isfinite(quat[1]) &&
         std::isfinite(quat[2]) && std::isfinite(quat[3]);
}

}  // namespace

YISIMU::YISIMU(LibXR::I2C* i2c, uint8_t addr)
    : i2c_(i2c), addr_(addr), slave_addr_(static_cast<uint16_t>(addr)) {}

LibXR::ErrorCode YISIMU::Init(LibXR::I2C* i2c, uint8_t addr) {
  if (i2c == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  i2c_ = i2c;
  addr_ = addr;
  slave_addr_ = static_cast<uint16_t>(addr_);
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode YISIMU::Init() {
  if (i2c_ == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  slave_addr_ = static_cast<uint16_t>(addr_);
  LibXR::Thread::Sleep(kInitBootDelayMs);

  LibXR::ErrorCode last_ec = LibXR::ErrorCode::CHECK_ERR;
  for (uint32_t attempt = 0; attempt < kInitProbeRetryCount; ++attempt) {
    last_ec = Probe();
    if (last_ec == LibXR::ErrorCode::OK) {
      return LibXR::ErrorCode::OK;
    }
    if ((attempt + 1U) < kInitProbeRetryCount) {
      LibXR::Thread::Sleep(kInitProbeRetryDelayMs);
    }
  }
  return last_ec;
}

LibXR::ErrorCode YISIMU::ReadEuler(float euler_rpy[3]) {
  return ReadEuler(euler_rpy, nullptr);
}

LibXR::ErrorCode YISIMU::ReadEuler(float euler_rpy[3], int32_t raw_euler_pry[3]) {
  if (i2c_ == nullptr || euler_rpy == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  uint8_t raw[12] = {0};
  const auto ec = ReadRegister(kEulerReg, raw, sizeof(raw));
  if (ec != LibXR::ErrorCode::OK) {
    return ec;
  }

  const int32_t pitch = LoadLeI32(raw + 0U);
  const int32_t roll = LoadLeI32(raw + 4U);
  const int32_t yaw = LoadLeI32(raw + 8U);

  if (raw_euler_pry != nullptr) {
    raw_euler_pry[0] = pitch;
    raw_euler_pry[1] = roll;
    raw_euler_pry[2] = yaw;
  }

  euler_rpy[0] = static_cast<float>(roll) * kEulerScale;
  euler_rpy[1] = static_cast<float>(pitch) * kEulerScale;
  euler_rpy[2] = static_cast<float>(yaw) * kEulerScale;
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode YISIMU::ReadQuaternion(float quat[4]) {
  return ReadQuaternion(quat, nullptr, nullptr);
}

LibXR::ErrorCode YISIMU::ReadQuaternion(float quat[4], int32_t raw_quat[4],
                                        float* norm_sq) {
  if (i2c_ == nullptr || quat == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  uint8_t raw[16] = {0};
  const auto ec = ReadRegister(kQuaternionReg, raw, sizeof(raw));
  if (ec != LibXR::ErrorCode::OK) {
    return ec;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    const int32_t value = LoadLeI32(raw + i * 4U);
    if (raw_quat != nullptr) {
      raw_quat[i] = value;
    }
    quat[i] = static_cast<float>(value) * kQuaternionScale;
  }

  return NormalizeQuaternion(quat, norm_sq) ? LibXR::ErrorCode::OK
                                            : LibXR::ErrorCode::CHECK_ERR;
}

LibXR::ErrorCode YISIMU::ReadSampleTimestamp(uint32_t* timestamp) {
  if (i2c_ == nullptr || timestamp == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  uint8_t raw[4] = {0};
  const auto ec = ReadRegister(kSampleTimestampReg, raw, sizeof(raw));
  if (ec != LibXR::ErrorCode::OK) {
    return ec;
  }

  *timestamp = LoadLeU32(raw);
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode YISIMU::ReadRegister(uint8_t reg, uint8_t* data,
                                      uint16_t len) {
  if (i2c_ == nullptr || data == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }
  if (len == 0U) {
    return LibXR::ErrorCode::ARG_ERR;
  }

  LibXR::Semaphore sem(0);
  LibXR::ReadOperation op(sem, kReadTimeoutMs);
  return i2c_->MemRead(slave_addr_, reg, {data, len}, op);
}

LibXR::ErrorCode YISIMU::ReadRegisterAt(uint16_t slave_addr, uint8_t reg,
                                        uint8_t* data, uint16_t len) {
  if (i2c_ == nullptr || data == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }
  if (len == 0U) {
    return LibXR::ErrorCode::ARG_ERR;
  }

  LibXR::Semaphore sem(0);
  LibXR::ReadOperation op(sem, kReadTimeoutMs);
  return i2c_->MemRead(slave_addr, reg, {data, len}, op);
}

bool YISIMU::HasNonZeroByte(const uint8_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) {
    if (data[i] != 0U) {
      return true;
    }
  }
  return false;
}

LibXR::ErrorCode YISIMU::Probe() {
  if (i2c_ == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  const uint16_t candidate = static_cast<uint16_t>(addr_);
  LibXR::ErrorCode last_ec = LibXR::ErrorCode::CHECK_ERR;

  uint8_t acc[12] = {0};
  const auto acc_ec = ReadRegisterAt(candidate, kAccelerationReg, acc, sizeof(acc));
  if (acc_ec != LibXR::ErrorCode::OK) {
    return acc_ec;
  }
  if (HasNonZeroByte(acc, sizeof(acc))) {
    slave_addr_ = candidate;
    return LibXR::ErrorCode::OK;
  }

  uint8_t euler[12] = {0};
  const auto euler_ec = ReadRegisterAt(candidate, kEulerReg, euler, sizeof(euler));
  if (euler_ec != LibXR::ErrorCode::OK) {
    return euler_ec;
  }
  if (HasNonZeroByte(euler, sizeof(euler))) {
    slave_addr_ = candidate;
    return LibXR::ErrorCode::OK;
  }

  uint8_t quat[16] = {0};
  const auto quat_ec = ReadRegisterAt(candidate, kQuaternionReg, quat, sizeof(quat));
  if (quat_ec != LibXR::ErrorCode::OK) {
    return quat_ec;
  }
  if (HasNonZeroByte(quat, sizeof(quat))) {
    slave_addr_ = candidate;
    return LibXR::ErrorCode::OK;
  }

  return last_ec;
}

}  // namespace Module
