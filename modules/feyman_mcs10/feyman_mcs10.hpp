#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "libxr_def.hpp"
#include "semaphore.hpp"

namespace Module {

struct FeymanMCS10Config {
  uint8_t connect_node_id = 0x7F;
  uint8_t node_id = 0x7F;
  uint32_t baudrate = 250000;
  uint32_t data_rate_hz = 20;
  uint32_t heartbeat_ms = 1000;
  uint32_t sdo_timeout_ms = 200;
  uint32_t sdo_inter_request_delay_ms = 5;
  uint32_t work_mode_settle_ms = 50;
  bool register_can_callback = true;
};

struct FeymanMCS10Sample {
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  uint16_t sequence = 0;
  uint16_t heartbeat_state = 0;
  uint16_t status_flags = 0;
  uint64_t latest_pdo_tick_us = 0;
};

struct FeymanMCS10CanError {
  bool pending = false;
  uint32_t count = 0;
  uint32_t last_error_id = 0;
  LibXR::CAN::ErrorState state{};
  bool state_valid = false;
};

class FeymanMCS10 {
 public:
  FeymanMCS10() = default;
  FeymanMCS10(LibXR::CAN* can, const FeymanMCS10Config& config);
  FeymanMCS10(const FeymanMCS10&) = delete;
  FeymanMCS10& operator=(const FeymanMCS10&) = delete;
  FeymanMCS10(FeymanMCS10&&) = delete;
  FeymanMCS10& operator=(FeymanMCS10&&) = delete;

  LibXR::ErrorCode Init(LibXR::CAN* can, const FeymanMCS10Config& config);
  LibXR::ErrorCode Configure();
  LibXR::ErrorCode ReadNodeId(uint8_t* node_id_out);
  LibXR::ErrorCode Readdress();
  LibXR::ErrorCode PollSample(FeymanMCS10Sample& sample);
  void Stop();

  void ProcessCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  bool TakePendingCanError(FeymanMCS10CanError& error);
  LibXR::ErrorCode GetCanErrorState(LibXR::CAN::ErrorState& state) const;

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

  static void OnCanFrame(bool in_isr, FeymanMCS10* device,
                         const LibXR::CAN::ClassicPack& pack);

  void HandleCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleCanError(const LibXR::CAN::ClassicPack& pack);
  void HandleSdoResponse(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleHeartbeat(const LibXR::CAN::ClassicPack& pack);
  void HandlePdo(PdoKind kind, const LibXR::CAN::ClassicPack& pack);

  LibXR::ErrorCode ConfigureBasicParameters();
  LibXR::ErrorCode WriteTargetNodeId();
  LibXR::ErrorCode ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                 uint8_t map_count,
                                 const uint32_t* mappings);
  LibXR::ErrorCode SendNmt(uint8_t command);
  LibXR::ErrorCode SendNmt(uint8_t command, uint8_t node_id);
  LibXR::ErrorCode SdoReadU32(uint16_t index, uint8_t subindex,
                              uint32_t* value_out);
  LibXR::ErrorCode SdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value);
  LibXR::ErrorCode SdoWriteU16(uint16_t index, uint8_t subindex,
                               uint16_t value);
  LibXR::ErrorCode SdoWriteU32(uint16_t index, uint8_t subindex,
                               uint32_t value);
  LibXR::ErrorCode SendSdoRequest(uint8_t command, uint16_t index,
                                  uint8_t subindex,
                                  const std::array<uint8_t, 4>& data);
  LibXR::ErrorCode WaitSdoResponse(uint16_t index, uint8_t subindex,
                                   SdoResponse& response);
  void DelayBetweenSdoRequests();
  LibXR::ErrorCode SendCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc,
                                LibXR::CAN::Type type);

  static float DecodeAccelMps2(int16_t raw);
  static float DecodeGyroDps(int16_t raw);

  LibXR::CAN* can_ = nullptr;
  FeymanMCS10Config config_{};
  LibXR::CAN::Callback can_callback_{};
  LibXR::Semaphore sdo_sem_{0};
  SdoTransaction sdo_transaction_{};
  PdoState pdo_state_{};
  uint8_t pending_pdo_mask_ = 0;
  FeymanMCS10CanError pending_can_error_{};
  uint64_t last_polled_tick_us_ = 0;
  uint8_t active_node_id_ = 0x7F;
  volatile bool running_ = false;
};

}  // namespace Module
