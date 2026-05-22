#pragma once

#include <array>
#include <cstdint>

#include "data_types.hpp"
#include "i2c.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "modules/wit_imu_jy901b/mod_wit_imu.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "vqf.hpp"

namespace Manager {

struct IMUManagerAcquisitionConfig {
  uint32_t frequency_hz = 50;
  const char* topic_name = "imu_data";
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  uint32_t stack_size = 2048;
  bool use_hardware_trigger = false;
  uint8_t enabled_imu_count = ACTUAL_IMU_COUNT;
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
  static void OnHardwareTriggerTimerInterrupt(bool in_isr);

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
  static constexpr uint16_t BuildLeadingSlotsMask(uint8_t imu_count) {
    const uint8_t capped_count =
        (imu_count > ACTUAL_IMU_COUNT) ? ACTUAL_IMU_COUNT : imu_count;
    return (capped_count == 0U)
               ? 0U
               : static_cast<uint16_t>((1u << capped_count) - 1u);
  }
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
  uint16_t enabled_slots_mask_ = 0x000F;
  bool use_hardware_trigger_ = false;
  volatile bool trigger_pending_ = false;
  volatile uint32_t trigger_overrun_count_ = 0;
  volatile uint32_t trigger_sequence_ = 0;
  volatile uint64_t trigger_mcu_tick_us_ = 0;
  LibXR::Semaphore acquisition_trigger_sem_{0};
  mutable LibXR::Mutex bus_mutex_;

  static IMUManager* hardware_trigger_manager_;
};

}  // namespace Manager
