#include "mod_wit_imu.hpp"

#include <array>

#include "libxr.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace Module {

using namespace LibXR;

WitIMU::WitIMU(I2C* i2c, uint8_t addr) : i2c_(i2c), addr_(addr) {
  registers_.fill(0);
}

WitIMU::ErrorCode WitIMU::Init() {
  if (i2c_ == nullptr) {
    return ErrorCode::INVAL;
  }

  if (ReadReg(kRegAccX, 13) != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  UpdateDataFromRegisters();
  return ErrorCode::OK;
}

void WitIMU::GetData(ImuData& data) const { data = data_; }

WitIMU::ErrorCode WitIMU::ReadReg(uint32_t reg, uint32_t count) {
  if (i2c_ == nullptr || reg >= kRegisterCount || count == 0 ||
      (reg + count) > kRegisterCount) {
    return ErrorCode::INVAL;
  }
  if (count > kMaxReadRegisters) {
    return ErrorCode::NO_MEM;
  }

  std::array<uint8_t, kMaxReadRegisters * 2> raw{};
  Semaphore sem(0);
  ReadOperation op(sem, 50);
  const auto ec = i2c_->MemRead(static_cast<uint16_t>(addr_) << 1U,
                                static_cast<uint16_t>(reg),
                                {raw.data(), count * 2U}, op);
  if (ec != LibXR::ErrorCode::OK) {
    return ErrorCode::ERROR;
  }

  for (uint32_t i = 0; i < count; ++i) {
    registers_[reg + i] = static_cast<int16_t>(
        static_cast<uint16_t>(raw[i * 2U]) |
        (static_cast<uint16_t>(raw[i * 2U + 1U]) << 8U));
  }
  return ErrorCode::OK;
}

WitIMU::ErrorCode WitIMU::WriteReg(uint32_t reg, uint16_t data) {
  if (i2c_ == nullptr || reg >= kRegisterCount) {
    return ErrorCode::INVAL;
  }

  const uint8_t raw[2] = {
      static_cast<uint8_t>(data & 0xFFU),
      static_cast<uint8_t>((data >> 8U) & 0xFFU),
  };
  Semaphore sem(0);
  WriteOperation op(sem);
  const auto ec = i2c_->MemWrite(static_cast<uint16_t>(addr_) << 1U,
                                 static_cast<uint16_t>(reg), {raw, 2}, op);
  return (ec == LibXR::ErrorCode::OK) ? ErrorCode::OK : ErrorCode::ERROR;
}

void WitIMU::UpdateDataFromRegisters() {
  data_.acc_x = static_cast<int16_t>(registers_[kRegAccX]) / 32768.0f * 16.0f;
  data_.acc_y = static_cast<int16_t>(registers_[kRegAccY]) / 32768.0f * 16.0f;
  data_.acc_z = static_cast<int16_t>(registers_[kRegAccZ]) / 32768.0f * 16.0f;

  data_.gyro_x =
      static_cast<int16_t>(registers_[kRegGyroX]) / 32768.0f * 2000.0f;
  data_.gyro_y =
      static_cast<int16_t>(registers_[kRegGyroY]) / 32768.0f * 2000.0f;
  data_.gyro_z =
      static_cast<int16_t>(registers_[kRegGyroZ]) / 32768.0f * 2000.0f;

  data_.mag_x = static_cast<float>(registers_[kRegMagX]);
  data_.mag_y = static_cast<float>(registers_[kRegMagY]);
  data_.mag_z = static_cast<float>(registers_[kRegMagZ]);

  data_.roll = static_cast<int16_t>(registers_[kRegRoll]) / 32768.0f * 180.0f;
  data_.pitch =
      static_cast<int16_t>(registers_[kRegPitch]) / 32768.0f * 180.0f;
  data_.yaw = static_cast<int16_t>(registers_[kRegYaw]) / 32768.0f * 180.0f;

  data_.temperature =
      static_cast<int16_t>(registers_[kRegTemperature]) / 100.0f;
  data_.q0 = 1.0f;
  data_.q1 = 0.0f;
  data_.q2 = 0.0f;
  data_.q3 = 0.0f;
  data_.timestamp_ms = Thread::GetTime();
  data_.acc_valid = true;
  data_.gyro_valid = true;
  data_.mag_valid = true;
  data_.angle_valid = true;
  data_.quat_valid = false;
}

}  // namespace Module
