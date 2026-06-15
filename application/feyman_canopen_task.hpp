#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "libxr_def.hpp"
#include "managers/data_types.hpp"
#include "message.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace Application {

struct FeymanCanopenConfig {
  uint8_t node_id = 0x7F;
  uint32_t baudrate = 250000;
  uint32_t data_rate_hz = 20;
  uint32_t heartbeat_ms = 1000;
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  uint32_t stack_size = 2048;
  uint32_t sdo_timeout_ms = 200;
  uint32_t sdo_inter_request_delay_ms = 5;
  uint32_t work_mode_settle_ms = 50;
  uint32_t startup_delay_ms = 50;
  bool verbose_config_log = true;
  const char* topic_name = "feyman_imu_pose";
  void (*log_writer)(const char* text) = nullptr;
};

class FeymanCanopenTask {
 public:
  explicit FeymanCanopenTask(
      LibXR::CAN* can,
      const FeymanCanopenConfig& config = FeymanCanopenConfig{});
  ~FeymanCanopenTask() = default;

  LibXR::ErrorCode Start();
  void Stop();

 private:
  struct SdoResponse {
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint8_t command = 0;
    std::array<uint8_t, 4> data = {0, 0, 0, 0};
    bool abort = false;
    uint32_t abort_code = 0;
  };

  struct SdoTransaction {
    bool active = false;
    uint16_t index = 0;
    uint8_t subindex = 0;
    SdoResponse response{};
  };

  enum class PdoKind : uint8_t {
    TPDO1_ACCEL = 0,
    TPDO2_GYRO = 1,
  };

  struct PdoState {
    std::array<int16_t, 3> acc_raw = {0, 0, 0};
    std::array<int16_t, 3> gyro_raw = {0, 0, 0};
    uint16_t status_flags = 0;
    uint16_t sequence = 0;
    uint8_t valid_mask = 0;
    uint8_t heartbeat_state = 0;
    uint64_t latest_pdo_tick_us = 0;
    uint64_t latest_heartbeat_tick_us = 0;
  };

  struct PendingCanError {
    bool pending = false;
    uint32_t count = 0;
    uint32_t last_error_id = 0;
    LibXR::CAN::ErrorState state{};
    bool state_valid = false;
  };

  static void TaskEntry(FeymanCanopenTask* task);
  static void OnCanFrame(bool in_isr, FeymanCanopenTask* task,
                         const LibXR::CAN::ClassicPack& pack);

  void Run();
  void HandleCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleCanError(const LibXR::CAN::ClassicPack& pack);
  void HandleSdoResponse(const LibXR::CAN::ClassicPack& pack);
  void HandleHeartbeat(const LibXR::CAN::ClassicPack& pack);
  void HandlePdo(PdoKind kind, const LibXR::CAN::ClassicPack& pack);
  void PublishPoseIfReady();
  void FlushPendingCanErrorLog();

  LibXR::ErrorCode ConfigureDevice();
  LibXR::ErrorCode ConfigureBasicParameters();
  LibXR::ErrorCode ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                 uint8_t map_count,
                                 const uint32_t* mappings);
  LibXR::ErrorCode SendNmt(uint8_t command);
  LibXR::ErrorCode SdoReadU32(uint16_t index, uint8_t subindex,
                              uint32_t* value_out);
  LibXR::ErrorCode SdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value);
  LibXR::ErrorCode SdoWriteU16(uint16_t index, uint8_t subindex, uint16_t value);
  LibXR::ErrorCode SdoWriteU32(uint16_t index, uint8_t subindex, uint32_t value);
  LibXR::ErrorCode SendSdoRequest(uint8_t command, uint16_t index,
                                  uint8_t subindex,
                                  const std::array<uint8_t, 4>& data);
  LibXR::ErrorCode WaitSdoResponse(uint16_t index, uint8_t subindex,
                                   SdoResponse& response);
  void DelayBetweenSdoRequests();
  LibXR::ErrorCode SendCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc,
                                LibXR::CAN::Type type);

  void Log(const char* text);
  void LogConfig(const char* text);
  void Logf(const char* fmt, ...);
  void LogConfigf(const char* fmt, ...);
  void LogCanErrorState(const char* context);

  static float DecodeAccelMps2(int16_t raw);
  static float DecodeGyroDps(int16_t raw);

  LibXR::CAN* can_;
  FeymanCanopenConfig config_;
  LibXR::Thread thread_{};
  LibXR::Topic* topic_ = nullptr;
  volatile bool running_ = false;
  LibXR::CAN::Callback can_callback_{};
  LibXR::Semaphore sdo_sem_{0};
  SdoTransaction sdo_transaction_{};
  PdoState pdo_state_{};
  PendingCanError pending_can_error_{};
  uint64_t last_publish_tick_us_ = 0;
  bool device_configured_ = false;
};

}  // namespace Application
