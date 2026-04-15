#pragma once

#include <cstdint>
#include <cstring>

#include "i2c.hpp"
#include "libxr.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timer.hpp"
#include "uart.hpp"

namespace Module {

class WitIMU {
 public:
  enum class Protocol : uint8_t {
    NORMAL = 0,
    MODBUS = 1,
    CAN = 2,
    I2C = 3,
  };

  enum class ErrorCode : int32_t {
    OK = 0,
    BUSY = -1,
    TIMEOUT = -2,
    ERROR = -3,
    NOMEM = -4,
    EMPTY = -5,
    INVAL = -6,
  };

  enum class CalibMode : uint8_t {
    NORMAL = 0x00,
    ACC_GYRO = 0x01,
    MAG = 0x02,
    ALTITUDE = 0x03,
    ANGLE_Z = 0x04,
    MAG_MM = 0x07,
  };

  enum class OutputRate : uint8_t {
    RATE_02HZ = 0x01,
    RATE_05HZ = 0x02,
    RATE_1HZ = 0x03,
    RATE_2HZ = 0x04,
    RATE_5HZ = 0x05,
    RATE_10HZ = 0x06,
    RATE_20HZ = 0x07,
    RATE_50HZ = 0x08,
    RATE_100HZ = 0x09,
    RATE_200HZ = 0x0B,
    RATE_NONE = 0x0D,
  };

  enum class Bandwidth : uint8_t {
    BW_256HZ = 0,
    BW_184HZ = 1,
    BW_94HZ = 2,
    BW_44HZ = 3,
    BW_21HZ = 4,
    BW_10HZ = 5,
    BW_5HZ = 6,
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

    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    int32_t pressure = 0;
    int32_t altitude = 0;
    uint32_t timestamp_ms = 0;

    bool acc_valid = false;
    bool gyro_valid = false;
    bool mag_valid = false;
    bool angle_valid = false;
    bool quat_valid = false;
  };

  using DataCallback = LibXR::Callback<>;

  WitIMU(LibXR::UART* uart, Protocol protocol = Protocol::NORMAL,
         uint8_t addr = 0x50);
  explicit WitIMU(LibXR::I2C* i2c, uint8_t addr = 0x50);

  ErrorCode Init();
  void DeInit();
  void GetData(ImuData& data);

  ErrorCode StartAccCalibration();
  ErrorCode StopAccCalibration();
  ErrorCode StartMagCalibration();
  ErrorCode StopMagCalibration();
  ErrorCode SetOutputRate(OutputRate rate);
  ErrorCode SetBandwidth(Bandwidth bw);
  ErrorCode SetAxis9();
  ErrorCode SetReturnContent(uint16_t content_mask);
  ErrorCode SaveConfig();
  ErrorCode Reset();

  ErrorCode ReadReg(uint32_t reg, uint32_t count);
  void UpdateDataFromRegisters();
  ErrorCode WriteReg(uint32_t reg, uint16_t data);
  void RegisterCallback(DataCallback& callback);

 private:
  Protocol protocol_;
  uint8_t addr_;
  LibXR::UART* uart_ = nullptr;
  LibXR::I2C* i2c_ = nullptr;

  LibXR::Semaphore data_sem_;
  LibXR::Semaphore tx_sem_;

  static constexpr size_t kDataBuffSize = 256;
  static constexpr size_t kRegSize = 0x90;
  uint8_t data_buff_[kDataBuffSize];
  uint32_t data_cnt_ = 0;
  uint32_t read_reg_index_ = 0;
  int16_t registers_[kRegSize];

  ImuData data_;
  DataCallback* update_callback_ = nullptr;
  LibXR::Thread rx_thread_;

  void SerialDataIn(uint8_t data);
  void ProcessWitData(uint8_t index, uint16_t* data, uint32_t len);
  uint16_t CalculateCRC16(uint8_t* data, uint16_t len);
  uint8_t CalculateChecksum(uint8_t* data, uint32_t len);
  ErrorCode UnlockRegisters();
  void Delay(uint16_t ms);
  static void RxThreadFunc(WitIMU* imu);

  static constexpr uint32_t kRegSave = 0x00;
  static constexpr uint32_t kRegCalSw = 0x01;
  static constexpr uint32_t kRegReturnContent = 0x02;
  static constexpr uint32_t kRegRate = 0x03;
  static constexpr uint32_t kRegBaud = 0x04;
  static constexpr uint32_t kRegAxis6 = 0x24;
  static constexpr uint32_t kRegBandwidth = 0x1F;
  static constexpr uint32_t kRegKey = 0x69;

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
  static constexpr uint32_t kRegTemp = 0x40;
  static constexpr uint32_t kRegQ0 = 0x51;
  static constexpr uint32_t kRegQ1 = 0x52;
  static constexpr uint32_t kRegQ2 = 0x53;
  static constexpr uint32_t kRegQ3 = 0x54;

  static constexpr uint16_t kKeyUnlock = 0xB588;
  static constexpr uint16_t kSaveParam = 0x00;
  static constexpr uint16_t kSaveReset = 0xFF;

  static const uint8_t kCrcHiTable[256];
  static const uint8_t kCrcLoTable[256];
};

}  // namespace Module
