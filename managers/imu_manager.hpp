#pragma once

#include <array>
#include <cstdint>

#include "data_types.hpp"
#include "i2c.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "modules/wit_imu_jy901b/mod_wit_imu.hpp"
#include "mutex.hpp"
#include "thread.hpp"
#include "vqf.hpp"

namespace Manager {

enum class QuaternionSource : uint8_t {
  VQF = 0,
  ImuRaw = 1,
};

class IMUManager {
 public:
  explicit IMUManager(uint8_t imu_count = ACTUAL_IMU_COUNT);
  ~IMUManager();

  LibXR::ErrorCode Init(LibXR::I2C* i2c,
                        uint8_t base_address = kDefaultImuAddress);
  LibXR::ErrorCode ReadAll(IMUArrayMsg& msg);
  LibXR::ErrorCode ReadSingle(uint8_t index, IMUData& data);
  LibXR::ErrorCode StartAcquisition(uint32_t frequency_hz,
                                    const char* topic_name = "imu_data");
  void StopAcquisition();

  bool IsIMUOnline(uint8_t index) const;
  uint8_t GetConfiguredCount() const { return imu_count_; }
  uint8_t GetOnlineCount() const;
  uint32_t GetFrequency() const { return frequency_hz_; }

  void SetQuaternionSource(QuaternionSource source);
  QuaternionSource GetQuaternionSource() const { return quaternion_source_; }

  bool AcquireBus();
  void ReleaseBus();

 private:
  void InitVQF(float sample_hz);
  void ReleaseVQF();
  void ApplyQuaternionSource(uint8_t index, IMUData& data);
  void ConvertIMUData(const Module::WitIMU::ImuData& src, IMUData& dst) const;
  bool ReadRawIMU(uint8_t index, Module::WitIMU::ImuData& raw_data);
  bool ProbeIMU(uint8_t index);
  static void AcquisitionThreadFunc(IMUManager* manager);

  LibXR::I2C* i2c_;
  uint8_t imu_count_;
  uint8_t base_address_;
  std::array<Module::WitIMU*, MAX_IMU_COUNT> imus_;
  std::array<::VQF*, MAX_IMU_COUNT> vqf_;
  ::VQFParams vqf_params_;
  QuaternionSource quaternion_source_;
  uint16_t online_mask_;
  uint32_t sequence_;
  LibXR::Thread acquisition_thread_;
  LibXR::Topic* data_topic_;
  volatile bool running_;
  uint32_t frequency_hz_;
  mutable LibXR::Mutex bus_mutex_;
};

}  // namespace Manager
