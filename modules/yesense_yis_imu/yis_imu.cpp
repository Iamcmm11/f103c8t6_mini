#include "yis_imu.hpp"

#include <cmath>
#include <cstdint>

#include "libxr_rw.hpp"
#include "semaphore.hpp"

namespace Module {

namespace {

int32_t LoadLeI32(const uint8_t* data) {
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8U) |
                       (static_cast<uint32_t>(data[2]) << 16U) |
                       (static_cast<uint32_t>(data[3]) << 24U);
  return static_cast<int32_t>(raw);
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
    : i2c_(i2c), addr_(addr), slave_addr_(static_cast<uint16_t>(addr) << 1U) {}

LibXR::ErrorCode YISIMU::Init(LibXR::I2C* i2c, uint8_t addr) {
  if (i2c == nullptr) {
    return LibXR::ErrorCode::PTR_NULL;
  }

  i2c_ = i2c;
  addr_ = addr;
  slave_addr_ = static_cast<uint16_t>(addr_) << 1U;
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode YISIMU::Init() {
  return Probe();
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

  const uint16_t candidates[2] = {
      static_cast<uint16_t>(static_cast<uint16_t>(addr_) << 1U),
      static_cast<uint16_t>(addr_),
  };
  LibXR::ErrorCode last_ec = LibXR::ErrorCode::CHECK_ERR;

  for (const uint16_t candidate : candidates) {
    uint8_t acc[12] = {0};
    const auto acc_ec =
        ReadRegisterAt(candidate, kAccelerationReg, acc, sizeof(acc));
    if (acc_ec != LibXR::ErrorCode::OK) {
      last_ec = acc_ec;
      continue;
    }
    if (HasNonZeroByte(acc, sizeof(acc))) {
      slave_addr_ = candidate;
      return LibXR::ErrorCode::OK;
    }

    uint8_t euler[12] = {0};
    const auto euler_ec = ReadRegisterAt(candidate, kEulerReg, euler,
                                         sizeof(euler));
    if (euler_ec != LibXR::ErrorCode::OK) {
      last_ec = euler_ec;
      continue;
    }
    if (HasNonZeroByte(euler, sizeof(euler))) {
      slave_addr_ = candidate;
      return LibXR::ErrorCode::OK;
    }

    uint8_t quat[16] = {0};
    const auto quat_ec =
        ReadRegisterAt(candidate, kQuaternionReg, quat, sizeof(quat));
    if (quat_ec != LibXR::ErrorCode::OK) {
      last_ec = quat_ec;
      continue;
    }
    if (HasNonZeroByte(quat, sizeof(quat))) {
      slave_addr_ = candidate;
      return LibXR::ErrorCode::OK;
    }

    last_ec = LibXR::ErrorCode::CHECK_ERR;
  }

  return last_ec;
}

}  // namespace Module
