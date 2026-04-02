#pragma once

#include <array>
#include <cstdint>

#include "i2c.hpp"

namespace Module {

class WitIMU {
 public:
  enum class ErrorCode : int32_t {
    OK = 0,
    ERROR = -1,
    INVAL = -2,
    NO_MEM = -3,
  };

  struct ImuData {
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;

    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    float temperature = 0.0f;
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    uint32_t timestamp_ms = 0;
    bool acc_valid = false;
    bool gyro_valid = false;
    bool mag_valid = false;
    bool angle_valid = false;
    bool quat_valid = false;
  };

  explicit WitIMU(LibXR::I2C* i2c, uint8_t addr = 0x50);

  ErrorCode Init();
  void GetData(ImuData& data) const;
  ErrorCode ReadReg(uint32_t reg, uint32_t count);
  ErrorCode WriteReg(uint32_t reg, uint16_t data);
  void UpdateDataFromRegisters();

 private:
  static constexpr uint32_t kRegisterCount = 0x80;
  static constexpr uint32_t kMaxReadRegisters = 16;

  static constexpr uint32_t kRegAccX = 0x34;
  static constexpr uint32_t kRegAccY = 0x35;
  static constexpr uint32_t kRegAccZ = 0x36;
  static constexpr uint32_t kRegGyroX = 0x37;
  static constexpr uint32_t kRegGyroY = 0x38;
  static constexpr uint32_t kRegGyroZ = 0x39;
  static constexpr uint32_t kRegMagX = 0x3A;
  static constexpr uint32_t kRegMagY = 0x3B;
  static constexpr uint32_t kRegMagZ = 0x3C;
  static constexpr uint32_t kRegRoll = 0x3D;
  static constexpr uint32_t kRegPitch = 0x3E;
  static constexpr uint32_t kRegYaw = 0x3F;
  static constexpr uint32_t kRegTemperature = 0x40;

  LibXR::I2C* i2c_;
  uint8_t addr_;
  std::array<int16_t, kRegisterCount> registers_;
  ImuData data_;
};

}  // namespace Module
