#pragma once

#include <array>
#include <cstdint>

#include "libxr_def.hpp"
#include "pwm.hpp"

namespace Manager {

enum class SyncEventSource : uint8_t {
  TIM2_IMU_SYNC_1HZ = 1,
  TIM5_CAMERA_TRIGGER_30HZ = 2,
};

struct SyncEventRecord {
  SyncEventSource source = SyncEventSource::TIM2_IMU_SYNC_1HZ;
  uint8_t flags = 0;
  uint16_t reserved = 0;
  uint32_t sequence = 0;
  uint64_t mcu_tick_us = 0;
  uint32_t nominal_period_us = 0;
  uint32_t dropped_count = 0;
};

struct SyncPwmOutputConfig {
  SyncEventSource source = SyncEventSource::TIM2_IMU_SYNC_1HZ;
  LibXR::PWM* pwm = nullptr;
  uint32_t nominal_period_us = 0;
  void (*before_enable)(void*) = nullptr;
  void* context = nullptr;
};

class SyncSignalManager {
 public:
  LibXR::ErrorCode RegisterPwmOutput(const SyncPwmOutputConfig& config);
  LibXR::ErrorCode StartAll();
  LibXR::ErrorCode StopAll();
  LibXR::ErrorCode GetLastStartResult(SyncEventSource source) const;

  static LibXR::ErrorCode StartRegisteredOutputs();
  static LibXR::ErrorCode StopRegisteredOutputs();
  static void RecordEventFromISR(SyncEventSource source, uint64_t mcu_tick_us);
  static void RecordEvent(SyncEventSource source, uint64_t mcu_tick_us);
  static bool GetLatestEvent(SyncEventSource source, SyncEventRecord& event);
  static bool PopEvent(SyncEventRecord& event);
  static bool IsActive();
  static uint32_t GetSessionId();
  static uint64_t GetSessionStartMcuTickUs();
  static uint64_t ToSessionTickUs(uint64_t absolute_mcu_tick_us);

 private:
  static constexpr size_t kMaxPwmOutputs = 4;

  std::array<SyncPwmOutputConfig, kMaxPwmOutputs> outputs_{};
  std::array<LibXR::ErrorCode, kMaxPwmOutputs> last_start_results_{};
  size_t output_count_ = 0;
};

}  // namespace Manager
