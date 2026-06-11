#include "mod_wit_imu.hpp"

#include <cstring>

namespace Module {

using namespace LibXR;

namespace {

constexpr uint32_t kI2CReadTimeoutTicks = 10;

}  // namespace

WitIMU::WitIMU(UART* uart, Protocol protocol, uint8_t addr)
    : protocol_(protocol),
      addr_(addr),
      uart_(uart),
      data_sem_(0),
      tx_sem_(1),
      data_{} {
  std::memset(data_buff_, 0, sizeof(data_buff_));
  std::memset(registers_, 0, sizeof(registers_));
}

WitIMU::WitIMU(I2C* i2c, uint8_t addr)
    : protocol_(Protocol::I2C),
      addr_(addr),
      i2c_(i2c),
      data_sem_(0),
      tx_sem_(1),
      data_{} {
  std::memset(data_buff_, 0, sizeof(data_buff_));
  std::memset(registers_, 0, sizeof(registers_));
}

WitIMU::ErrorCode WitIMU::Init() {
  if (protocol_ != Protocol::I2C) {
    return (uart_ != nullptr) ? ErrorCode::INVAL : ErrorCode::EMPTY;
  }
  return (i2c_ != nullptr) ? ErrorCode::OK : ErrorCode::EMPTY;
}

void WitIMU::DeInit() {
  update_callback_ = nullptr;
  data_cnt_ = 0;
}

void WitIMU::GetData(ImuData& data) { data = data_; }

uint16_t WitIMU::CalculateCRC16(uint8_t* data, uint16_t len) {
  (void)data;
  (void)len;
  return 0;
}

uint8_t WitIMU::CalculateChecksum(uint8_t* data, uint32_t len) {
  uint8_t sum = 0;
  for (uint32_t i = 0; i < len; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return sum;
}

void WitIMU::ProcessWitData(uint8_t index, uint16_t* data, uint32_t len) {
  (void)index;
  (void)data;
  (void)len;
}

void WitIMU::SerialDataIn(uint8_t data) { (void)data; }

void WitIMU::RxThreadFunc(WitIMU* imu) { (void)imu; }

WitIMU::ErrorCode WitIMU::WriteReg(uint32_t reg, uint16_t data) {
  if (protocol_ != Protocol::I2C || i2c_ == nullptr) {
    return (uart_ != nullptr) ? ErrorCode::INVAL : ErrorCode::EMPTY;
  }
  if (reg >= kRegSize) {
    return ErrorCode::INVAL;
  }

  const uint8_t raw[2] = {
      static_cast<uint8_t>(data & 0xFFU),
      static_cast<uint8_t>(data >> 8U),
  };
  WriteOperation op;
  return (i2c_->MemWrite(static_cast<uint16_t>(addr_) << 1U,
                         static_cast<uint16_t>(reg), {raw, sizeof(raw)}, op) ==
          LibXR::ErrorCode::OK)
             ? ErrorCode::OK
             : ErrorCode::ERROR;
}

WitIMU::ErrorCode WitIMU::ReadReg(uint32_t reg, uint32_t count) {
  if (protocol_ != Protocol::I2C || i2c_ == nullptr) {
    return (uart_ != nullptr) ? ErrorCode::INVAL : ErrorCode::EMPTY;
  }
  if (count == 0U || (reg + count) >= kRegSize) {
    return ErrorCode::INVAL;
  }

  const uint16_t raw_len = static_cast<uint16_t>(count << 1U);
  if (raw_len > kDataBuffSize) {
    return ErrorCode::NOMEM;
  }

  Semaphore sem(0);
  ReadOperation op(sem, kI2CReadTimeoutTicks);
  if (i2c_->MemRead(static_cast<uint16_t>(addr_) << 1U,
                    static_cast<uint16_t>(reg), {data_buff_, raw_len}, op) !=
      LibXR::ErrorCode::OK) {
    return ErrorCode::ERROR;
  }

  for (uint32_t i = 0; i < count; ++i) {
    registers_[reg + i] = static_cast<int16_t>(
        static_cast<uint16_t>(data_buff_[i << 1U]) |
        (static_cast<uint16_t>(data_buff_[(i << 1U) + 1U]) << 8U));
  }
  read_reg_index_ = reg;
  return ErrorCode::OK;
}

void WitIMU::UpdateDataFromRegisters() {
  data_.acc_x = static_cast<int16_t>(registers_[kRegAccX]) / 32768.0f * 16.0f;
  data_.acc_y = static_cast<int16_t>(registers_[kRegAccY]) / 32768.0f * 16.0f;
  data_.acc_z = static_cast<int16_t>(registers_[kRegAccZ]) / 32768.0f * 16.0f;
  data_.acc_valid = true;

  data_.gyro_x =
      static_cast<int16_t>(registers_[kRegGyroX]) / 32768.0f * 2000.0f;
  data_.gyro_y =
      static_cast<int16_t>(registers_[kRegGyroY]) / 32768.0f * 2000.0f;
  data_.gyro_z =
      static_cast<int16_t>(registers_[kRegGyroZ]) / 32768.0f * 2000.0f;
  data_.gyro_valid = true;

  data_.mag_x = static_cast<int16_t>(registers_[kRegMagX]);
  data_.mag_y = static_cast<int16_t>(registers_[kRegMagY]);
  data_.mag_z = static_cast<int16_t>(registers_[kRegMagZ]);
  data_.mag_valid = true;

  data_.roll = static_cast<int16_t>(registers_[kRegRoll]) / 32768.0f * 180.0f;
  data_.pitch =
      static_cast<int16_t>(registers_[kRegPitch]) / 32768.0f * 180.0f;
  data_.yaw = static_cast<int16_t>(registers_[kRegYaw]) / 32768.0f * 180.0f;
  data_.angle_valid = true;

  data_.temperature = static_cast<int16_t>(registers_[kRegTemp]) / 100.0f;
  data_.q0 = static_cast<int16_t>(registers_[kRegQ0]) / 32768.0f;
  data_.q1 = static_cast<int16_t>(registers_[kRegQ1]) / 32768.0f;
  data_.q2 = static_cast<int16_t>(registers_[kRegQ2]) / 32768.0f;
  data_.q3 = static_cast<int16_t>(registers_[kRegQ3]) / 32768.0f;
  data_.quat_valid = true;
  data_.timestamp_ms = Thread::GetTime();
}

WitIMU::ErrorCode WitIMU::UnlockRegisters() {
  return WriteReg(kRegKey, kKeyUnlock);
}

void WitIMU::Delay(uint16_t ms) { Thread::Sleep(ms); }

WitIMU::ErrorCode WitIMU::StartAccCalibration() {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegCalSw, static_cast<uint16_t>(CalibMode::ACC_GYRO));
}

WitIMU::ErrorCode WitIMU::StopAccCalibration() {
  if (WriteReg(kRegCalSw, static_cast<uint16_t>(CalibMode::NORMAL)) !=
      ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegSave, kSaveParam);
}

WitIMU::ErrorCode WitIMU::StartMagCalibration() {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegCalSw, static_cast<uint16_t>(CalibMode::MAG_MM));
}

WitIMU::ErrorCode WitIMU::StopMagCalibration() {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegCalSw, static_cast<uint16_t>(CalibMode::NORMAL));
}

WitIMU::ErrorCode WitIMU::SetOutputRate(OutputRate rate) {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegRate, static_cast<uint16_t>(rate));
}

WitIMU::ErrorCode WitIMU::SetBandwidth(Bandwidth bw) {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegBandwidth, static_cast<uint16_t>(bw));
}

WitIMU::ErrorCode WitIMU::SetAxis9() {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegAxis6, 0);
}

WitIMU::ErrorCode WitIMU::SetReturnContent(uint16_t content_mask) {
  if (UnlockRegisters() != ErrorCode::OK) {
    return ErrorCode::ERROR;
  }
  return WriteReg(kRegReturnContent, content_mask);
}

WitIMU::ErrorCode WitIMU::SaveConfig() {
  return WriteReg(kRegSave, kSaveParam);
}

WitIMU::ErrorCode WitIMU::Reset() {
  return WriteReg(kRegSave, kSaveReset);
}

void WitIMU::RegisterCallback(DataCallback& callback) {
  update_callback_ = &callback;
}

}  // namespace Module
