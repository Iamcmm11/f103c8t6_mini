#pragma once

#include <array>
#include <cstdint>

#include "data_types.hpp"
#include "i2c.hpp"
#include "libxr.hpp"
#include "modules/wit_imu_jy901b/mod_wit_imu.hpp"
#include "mutex.hpp"

namespace Manager {

class IMUManager {
 public:
  explicit IMUManager(uint8_t imu_count = ACTUAL_IMU_COUNT);
  ~IMUManager();

  LibXR::ErrorCode Init(LibXR::I2C* i2c,
                        uint8_t base_address = kDefaultImuAddress);
  LibXR::ErrorCode ReadAll(IMUArrayMsg& msg);
  LibXR::ErrorCode ReadSingle(uint8_t index, IMUData& data);

  bool IsIMUOnline(uint8_t index) const;
  uint8_t GetConfiguredCount() const { return imu_count_; }
  uint8_t GetOnlineCount() const;

  bool AcquireBus();
  void ReleaseBus();

 private:
  void ConvertIMUData(const Module::WitIMU::ImuData& src, IMUData& dst) const;

  LibXR::I2C* i2c_;
  uint8_t imu_count_;
  uint8_t base_address_;
  std::array<Module::WitIMU*, MAX_IMU_COUNT> imus_;
  uint16_t online_mask_;
  uint32_t sequence_;
  mutable LibXR::Mutex bus_mutex_;
};

}  // namespace Manager
