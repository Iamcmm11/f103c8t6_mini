#pragma once

#include <array>
#include <cstdint>

namespace Manager {

// 预留的 IMU 最大槽位数，方便后续扩展更多传感器。
constexpr uint8_t MAX_IMU_COUNT = 16;
// 当前工程实际启用的 IMU 槽位数，业务层按这个数量组织消息和地址映射。
constexpr uint8_t ACTUAL_IMU_COUNT = 4;

// 逻辑槽位定义，用固定部位名称把 IMU 与手部位置绑定起来。
enum class ImuSlot : uint8_t {
  Forearm = 0,
  Hand = 1,
  ThumbRoot = 2,
  ThumbTip = 3,
};

// 默认地址布局：4 个 IMU 依次占用 0x50 ~ 0x53。
static constexpr uint8_t kForearmImuI2CAddr = 0x50;
static constexpr uint8_t kHandImuI2CAddr = 0x51;
static constexpr uint8_t kThumbRootImuI2CAddr = 0x52;
static constexpr uint8_t kThumbTipImuI2CAddr = 0x53;
static constexpr uint8_t kDefaultImuAddress = kForearmImuI2CAddr;

// 根据槽位索引和基地址推导真实 I2C 地址，便于 manager 统一创建传感器对象。
constexpr uint8_t ResolveImuI2CAddress(
    uint8_t index, uint8_t base_address = kDefaultImuAddress) {
  switch (index) {
    case static_cast<uint8_t>(ImuSlot::Forearm):
      return base_address;
    case static_cast<uint8_t>(ImuSlot::Hand):
      return static_cast<uint8_t>(base_address + 1U);
    case static_cast<uint8_t>(ImuSlot::ThumbRoot):
      return static_cast<uint8_t>(base_address + 2U);
    case static_cast<uint8_t>(ImuSlot::ThumbTip):
      return static_cast<uint8_t>(base_address + 3U);
    default:
      return base_address;
  }
}

static_assert(ACTUAL_IMU_COUNT <= MAX_IMU_COUNT,
              "ACTUAL_IMU_COUNT cannot exceed MAX_IMU_COUNT");

// 单个 IMU 的标准化输出结构，单位和字段名对上层业务保持稳定。
struct IMUData {
  uint64_t timestamp_us = 0;
  uint64_t mcu_tick_us = 0;
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float angle[3] = {0.0f, 0.0f, 0.0f};
  float mag[3] = {0.0f, 0.0f, 0.0f};
  float temperature = 0.0f;
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  uint8_t status = 0;
  uint8_t reserved[3] = {0, 0, 0};
};

struct YISPoseMsg {
  uint64_t timestamp_us = 0;
  float euler[3] = {0.0f, 0.0f, 0.0f};
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  uint32_t sample_timestamp = 0;
  uint8_t status = 0;
  uint8_t reserved[3] = {0, 0, 0};
};

// 一帧 IMU 主题消息，包含时间戳、序号、有效位和全部槽位数据。
struct IMUArrayMsg {
  uint64_t timestamp_us = 0;
  uint64_t trigger_mcu_tick_us = 0;
  uint32_t sequence = 0;
  uint32_t trigger_sequence = 0;
  uint32_t trigger_overrun_count = 0;
  uint16_t valid_mask = 0;
  uint16_t reserved = 0;
  std::array<IMUData, ACTUAL_IMU_COUNT> imu_data{};

  // 检查指定槽位在当前消息中是否携带了有效数据。
  bool IsValid(uint8_t index) const {
    return (index < ACTUAL_IMU_COUNT) && ((valid_mask & (1u << index)) != 0U);
  }

  // 设置或清除某个槽位的有效位，供采集线程在填充消息时使用。
  void SetValid(uint8_t index, bool valid = true) {
    if (index >= ACTUAL_IMU_COUNT) {
      return;
    }
    if (valid) {
      valid_mask = static_cast<uint16_t>(valid_mask | (1u << index));
    } else {
      valid_mask = static_cast<uint16_t>(valid_mask & ~(1u << index));
    }
  }

  // 统计当前这一帧里实际有效的 IMU 数量。
  uint8_t GetValidCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < ACTUAL_IMU_COUNT; ++i) {
      if (IsValid(i)) {
        ++count;
      }
    }
    return count;
  }

  // 返回消息固定容量，方便桥接层做协议打包。
  static constexpr uint8_t GetCapacity() { return ACTUAL_IMU_COUNT; }
};

}  // namespace Manager
