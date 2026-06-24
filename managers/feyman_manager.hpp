#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "can.hpp"
#include "data_types.hpp"
#include "libxr_def.hpp"
#include "message.hpp"
#include "modules/feyman_mcs10/feyman_mcs10.hpp"
#include "mutex.hpp"
#include "thread.hpp"

namespace Manager {

struct FeymanManagedDeviceConfig {
  uint8_t node_id = 0x7F;
  bool enabled = true;
};

struct FeymanManagerConfig {
  LibXR::CAN* can = nullptr;
  const FeymanManagedDeviceConfig* devices = nullptr;
  size_t device_count = 0;
  uint8_t primary_node_id = 0x7F;
  const char* aggregate_topic_name = "feyman_imu_array";
  const char* legacy_topic_name = "feyman_imu_pose";
  const char* sample_topic_name = "feyman_imu_sample";
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  uint32_t stack_size = 3072;
  uint32_t startup_delay_ms = 50;
  uint32_t baudrate = 250000;
  uint32_t data_rate_hz = 100;
  uint32_t heartbeat_ms = 1000;
  uint32_t sdo_timeout_ms = 200;
  uint32_t sdo_inter_request_delay_ms = 5;
  uint32_t work_mode_settle_ms = 50;
  bool verbose_config_log = true;
  void (*log_writer)(const char* text) = nullptr;
};

class FeymanManager {
 public:
  FeymanManager() = default;
  ~FeymanManager() = default;

  LibXR::ErrorCode Init(const FeymanManagerConfig& config);
  LibXR::ErrorCode Start();
  void Stop();

 private:
  struct DeviceRuntime {
    FeymanManagedDeviceConfig config{};
    Module::FeymanMCS10 device{};
    FeymanDeviceMsg latest{};
    bool initialized = false;
    bool configured = false;
  };

  static void TaskEntry(FeymanManager* manager);
  static void OnCanFrame(bool in_isr, FeymanManager* manager,
                         const LibXR::CAN::ClassicPack& pack);

  void Run();
  void HandleCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleCanError(const LibXR::CAN::ClassicPack& pack);
  void FlushCanErrorLog();
  bool TryConfigureDevices();
  bool PollDevicesAndPublish();
  uint16_t BuildReadyDeviceMask() const;
  bool ShouldPublishAggregate(uint64_t now_us) const;
  void ResetAggregateCoalesce();
  void PublishLegacyIfPrimaryUpdated(const FeymanDeviceMsg& msg);
  void PublishAggregate();
  void Log(const char* text);
  void Logf(const char* fmt, ...);
  void LogConfigf(const char* fmt, ...);
  Module::FeymanMCS10Config BuildDeviceConfig(
      uint8_t connect_node_id, uint8_t target_node_id) const;

  FeymanManagerConfig config_{};
  std::array<DeviceRuntime, MAX_FEYMAN_DEVICE_COUNT> devices_{};
  uint8_t device_count_ = 0;
  LibXR::Thread thread_{};
  LibXR::CAN::Callback can_callback_{};
  LibXR::Topic* aggregate_topic_ = nullptr;
  LibXR::Topic* legacy_topic_ = nullptr;
  LibXR::Topic* sample_topic_ = nullptr;
  LibXR::Mutex op_mutex_{};
  volatile bool thread_alive_ = false;
  volatile bool active_ = false;
  bool thread_created_ = false;
  uint32_t publish_sequence_ = 0;
  uint32_t last_config_attempt_ms_ = 0;
  uint16_t updated_device_mask_ = 0;
  uint64_t aggregate_window_start_us_ = 0;
  FeymanArrayMsg last_aggregate_{};
  LibXR::CAN::ErrorState last_can_error_state_{};
  uint32_t last_can_error_id_ = 0;
  uint32_t last_can_error_count_ = 0;
  uint32_t emitted_can_error_count_ = 0;
};

}  // namespace Manager
