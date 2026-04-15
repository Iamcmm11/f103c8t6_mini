#pragma once

#include <array>
#include <cstdint>

namespace Manager {

constexpr uint8_t MAX_IMU_COUNT = 16;
constexpr uint8_t ACTUAL_IMU_COUNT = 4;

enum class ImuSlot : uint8_t {
  Forearm = 0,
  Hand = 1,
  ThumbRoot = 2,
  ThumbTip = 3,
};

static constexpr uint8_t kForearmImuI2CAddr = 0x50;
static constexpr uint8_t kHandImuI2CAddr = 0x51;
static constexpr uint8_t kThumbRootImuI2CAddr = 0x52;
static constexpr uint8_t kThumbTipImuI2CAddr = 0x53;
static constexpr uint8_t kDefaultImuAddress = kForearmImuI2CAddr;

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

struct IMUData {
  uint64_t timestamp_us = 0;
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float angle[3] = {0.0f, 0.0f, 0.0f};
  float mag[3] = {0.0f, 0.0f, 0.0f};
  float temperature = 0.0f;
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  uint8_t status = 0;
  uint8_t reserved[3] = {0, 0, 0};
};

struct IMUArrayMsg {
  uint64_t timestamp_us = 0;
  uint32_t sequence = 0;
  uint16_t valid_mask = 0;
  uint16_t reserved = 0;
  std::array<IMUData, ACTUAL_IMU_COUNT> imu_data{};

  bool IsValid(uint8_t index) const {
    return (index < ACTUAL_IMU_COUNT) && ((valid_mask & (1u << index)) != 0U);
  }

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

  uint8_t GetValidCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < ACTUAL_IMU_COUNT; ++i) {
      if (IsValid(i)) {
        ++count;
      }
    }
    return count;
  }

  static constexpr uint8_t GetCapacity() { return ACTUAL_IMU_COUNT; }
};

}  // namespace Manager
