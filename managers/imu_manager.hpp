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

namespace Manager {

/*
 * Manager: IMUManager
 * - 负责 IMU 设备创建、地址映射、在线探测、I2C 总线互斥和周期采集。
 * - 对上层暴露稳定的 Read / StartAcquisition 接口，把底层 Module::WitIMU 封装起来。
 */
class IMUManager {
 public:
  explicit IMUManager(uint8_t imu_count = ACTUAL_IMU_COUNT);
  ~IMUManager();

  // 绑定 I2C 总线并按默认地址布局初始化全部 IMU 模块对象。
  LibXR::ErrorCode Init(LibXR::I2C* i2c,
                        uint8_t base_address = kDefaultImuAddress);
  // 读取当前全部在线 IMU，输出一帧标准化 IMUArrayMsg。
  LibXR::ErrorCode ReadAll(IMUArrayMsg& msg);
  // 读取单个槽位的 IMU 数据，适合调试或局部功能查询。
  LibXR::ErrorCode ReadSingle(uint8_t index, IMUData& data);
  // 启动后台采集线程，并持续向指定 Topic 发布 IMU 数据。
  LibXR::ErrorCode StartAcquisition(uint32_t frequency_hz,
                                    const char* topic_name = "imu_data");
  void StopAcquisition();

  // 查询某个逻辑槽位的 IMU 当前是否在线。
  bool IsIMUOnline(uint8_t index) const;
  uint8_t GetConfiguredCount() const { return imu_count_; }
  uint8_t GetOnlineCount() const;
  uint32_t GetFrequency() const { return frequency_hz_; }

  // 提供给桥接层的总线仲裁接口，避免它和采集线程同时访问同一条 I2C。
  bool AcquireBus();
  void ReleaseBus();

 private:
  // 把 Module 层原始量纲转换成上层统一使用的 IMUData 结构。
  void ConvertIMUData(const Module::WitIMU::ImuData& src, IMUData& dst) const;
  // 从单个底层 IMU 模块中读取寄存器并刷新原始数据结构。
  bool ReadRawIMU(uint8_t index, Module::WitIMU::ImuData& raw_data);
  // 用一次读数据流程确认设备是否在线，并顺带做必要的工作模式设置。
  bool ProbeIMU(uint8_t index);
  // 后台采集线程入口，周期执行 ReadAll 并把结果发布到 Topic。
  static void AcquisitionThreadFunc(IMUManager* manager);

  LibXR::I2C* i2c_;
  uint8_t imu_count_;
  uint8_t base_address_;
  std::array<Module::WitIMU*, MAX_IMU_COUNT> imus_;
  uint16_t online_mask_;
  uint32_t sequence_;
  LibXR::Thread acquisition_thread_;
  LibXR::Topic* data_topic_;
  volatile bool running_;
  uint32_t frequency_hz_;
  mutable LibXR::Mutex bus_mutex_;
};

}  // namespace Manager
