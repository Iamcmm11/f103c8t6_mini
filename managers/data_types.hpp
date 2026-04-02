#pragma once

#include <array>
#include <cstdint>

namespace Manager {

constexpr uint8_t MAX_IMU_COUNT = 16;
constexpr uint8_t ACTUAL_IMU_COUNT = 1;
constexpr uint8_t kDefaultImuAddress = 0x50;

constexpr uint8_t ResolveImuI2CAddress(uint8_t index,
                                       uint8_t base_address =
                                           kDefaultImuAddress) {
  return static_cast<uint8_t>(base_address + index);
}

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
