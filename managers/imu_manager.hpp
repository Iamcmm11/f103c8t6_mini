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

struct IMUManagerAcquisitionConfig {
  uint32_t frequency_hz = 50;
  const char* topic_name = "imu_data";
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  uint32_t stack_size = 2048;
};

// 四元数输出源：
// - VQF: 使用 acc/gyro/mag 经 VQF 融合得到的姿态
// - ImuRaw: 直接使用 IMU 自身寄存器提供的四元数
enum class QuaternionSource : uint8_t {
  VQF = 0,
  ImuRaw = 1,
};

class IMUManager {
 public:
  explicit IMUManager(uint8_t imu_count = ACTUAL_IMU_COUNT);
  ~IMUManager();

  // 绑定 I2C 总线，并按默认地址布局创建/探测全部 IMU。
  LibXR::ErrorCode Init(LibXR::I2C* i2c,
                        uint8_t base_address = kDefaultImuAddress);
  // 读取当前全部在线 IMU，输出一帧标准化 IMUArrayMsg。
  LibXR::ErrorCode ReadAll(IMUArrayMsg& msg);
  // 单独读取一个槽位，常用于调试或局部查询。
  LibXR::ErrorCode ReadSingle(uint8_t index, IMUData& data);
  // 启动后台采集线程，并持续向 topic_name 发布 IMU 数据。
  LibXR::ErrorCode StartAcquisition(
      const IMUManagerAcquisitionConfig& config =
          IMUManagerAcquisitionConfig{});
  void StopAcquisition();

  bool IsIMUOnline(uint8_t index) const;
  uint8_t GetConfiguredCount() const { return imu_count_; }
  uint8_t GetOnlineCount() const;
  uint32_t GetFrequency() const { return frequency_hz_; }

  void SetQuaternionSource(QuaternionSource source);
  QuaternionSource GetQuaternionSource() const { return quaternion_source_; }

  // 桥接层直接访问 I2C 时，需要先通过 manager 做总线互斥。
  bool AcquireBus();
  void ReleaseBus();

 private:
  // 采样频率变化时重建每路 IMU 对应的 VQF 实例。
  void InitVQF(float sample_hz);
  void ReleaseVQF();
  // 无论最终输出源是否选择 VQF，都会先更新一次滤波器状态。
  void ApplyQuaternionSource(uint8_t index, IMUData& data);
  void ConvertIMUData(const Module::WitIMU::ImuData& src, IMUData& dst) const;
  bool ReadRawIMU(uint8_t index, Module::WitIMU::ImuData& raw_data);
  bool ProbeIMU(uint8_t index);
  static bool IsPlausibleProbeData(const Module::WitIMU::ImuData& raw_data);
  static void AcquisitionThreadFunc(IMUManager* manager);

  LibXR::I2C* i2c_;
  uint8_t imu_count_;
  uint8_t base_address_;
  std::array<Module::WitIMU*, MAX_IMU_COUNT> imus_;
  // 每路 IMU 各自维护一份 VQF 状态，避免多路传感器互相污染。
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
