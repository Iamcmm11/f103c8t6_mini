#pragma once

#include <cstdint>

#include "i2c.hpp"
#include "libxr_def.hpp"

namespace Module {

class YISIMU {
 public:
  static constexpr uint8_t kDefaultAddress = 0x6A;

  YISIMU() = default;
  explicit YISIMU(LibXR::I2C* i2c, uint8_t addr = kDefaultAddress);

  LibXR::ErrorCode Init(LibXR::I2C* i2c,
                        uint8_t addr = kDefaultAddress);
  LibXR::ErrorCode Init();
  // Returns Euler angles in roll/pitch/yaw order, unit: degree.
  LibXR::ErrorCode ReadEuler(float euler_rpy[3]);
  LibXR::ErrorCode ReadEuler(float euler_rpy[3], int32_t raw_euler_pry[3]);
  LibXR::ErrorCode ReadQuaternion(float quat[4]);
  LibXR::ErrorCode ReadQuaternion(float quat[4], int32_t raw_quat[4],
                                  float* norm_sq);
  LibXR::ErrorCode ReadRegister(uint8_t reg, uint8_t* data, uint16_t len);
  LibXR::ErrorCode Probe();

  uint8_t address() const { return addr_; }
  uint16_t slave_address() const { return slave_addr_; }

 private:
  static constexpr uint8_t kAccelerationReg = 0x10;
  static constexpr uint8_t kEulerReg = 0x40;
  static constexpr uint16_t kQuaternionReg = 0x41;
  static constexpr uint32_t kReadTimeoutMs = 50;
  static constexpr float kEulerScale = 1.0e-6f;
  static constexpr float kQuaternionScale = 1.0e-6f;

  LibXR::ErrorCode ReadRegisterAt(uint16_t slave_addr, uint8_t reg,
                                  uint8_t* data, uint16_t len);
  static bool HasNonZeroByte(const uint8_t* data, uint16_t len);

  LibXR::I2C* i2c_ = nullptr;
  uint8_t addr_ = kDefaultAddress;
  uint16_t slave_addr_ = static_cast<uint16_t>(kDefaultAddress);
};

}  // namespace Module
