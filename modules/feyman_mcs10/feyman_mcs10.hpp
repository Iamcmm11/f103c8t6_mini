#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "libxr_def.hpp"
#include "semaphore.hpp"

namespace Module {

// FEYMAN MCS10 的底层配置。
// 这一层只关心设备节点、CANopen 参数和超时，不负责线程、topic 或业务日志。
struct FeymanMCS10Config {
  uint8_t node_id = 0x7F;
  uint32_t baudrate = 250000;
  uint32_t data_rate_hz = 20;
  uint32_t heartbeat_ms = 1000;
  uint32_t sdo_timeout_ms = 200;
  uint32_t sdo_inter_request_delay_ms = 5;
  uint32_t work_mode_settle_ms = 50;
};

// 模块层对外暴露的最新一帧解码结果。
// 保持为纯设备数据，交给 application 层决定如何发布成消息。
struct FeymanMCS10Sample {
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  uint16_t sequence = 0;
  uint16_t heartbeat_state = 0;
  uint16_t status_flags = 0;
  uint64_t latest_pdo_tick_us = 0;
};

// ISR 里先缓存 CAN 错误摘要，避免在中断里直接做串口日志这类重操作。
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

  // 注册 CAN 回调并初始化本地状态。
  LibXR::ErrorCode Init(LibXR::CAN* can, const FeymanMCS10Config& config);
  // 执行完整的 FEYMAN CANopen 配置流程。
  LibXR::ErrorCode Configure();
  // 取出一帧“尚未被上层消费过”的最新样本。
  LibXR::ErrorCode PollSample(FeymanMCS10Sample& sample);
  void Stop();

  // application 层周期性取走待打印的错误摘要。
  bool TakePendingCanError(FeymanMCS10CanError& error);
  LibXR::ErrorCode GetCanErrorState(LibXR::CAN::ErrorState& state) const;

 private:
  // 当前只用到 expedited SDO，因此响应缓存固定为 4 字节数据区。
  struct SdoResponse {
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint8_t command = 0;
    std::array<uint8_t, 4> data = {0, 0, 0, 0};
    bool abort = false;
    uint32_t abort_code = 0;
  };

  // 用于把异步到来的 SDO 响应和当前正在等待的请求匹配起来。
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

  // 中断里更新、线程里读取的 PDO/heartbeat 快照。
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

  // LibXR 的 CAN 回调适配入口。
  static void OnCanFrame(bool in_isr, FeymanMCS10* device,
                         const LibXR::CAN::ClassicPack& pack);

  // 按帧类型把收到的 CAN 数据分发到不同处理路径。
  void HandleCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleCanError(const LibXR::CAN::ClassicPack& pack);
  void HandleSdoResponse(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  void HandleHeartbeat(const LibXR::CAN::ClassicPack& pack);
  void HandlePdo(PdoKind kind, const LibXR::CAN::ClassicPack& pack);

  // Configure() 的子步骤：基础对象字典参数配置、PDO 映射配置。
  LibXR::ErrorCode ConfigureBasicParameters();
  LibXR::ErrorCode ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                 uint8_t map_count,
                                 const uint32_t* mappings);
  LibXR::ErrorCode SendNmt(uint8_t command);
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

  // FEYMAN 原始量转换。
  static float DecodeAccelMps2(int16_t raw);
  static float DecodeGyroDps(int16_t raw);

  LibXR::CAN* can_ = nullptr;
  FeymanMCS10Config config_{};
  LibXR::CAN::Callback can_callback_{};
  LibXR::Semaphore sdo_sem_{0};
  SdoTransaction sdo_transaction_{};
  PdoState pdo_state_{};
  // 等待 accel 和 gyro 两路 PDO 都更新后，再把 sequence 向前推进一帧。
  uint8_t pending_pdo_mask_ = 0;
  FeymanMCS10CanError pending_can_error_{};
  // 防止同一帧数据被 application 层重复消费。
  uint64_t last_polled_tick_us_ = 0;
  volatile bool running_ = false;
};

}  // namespace Module
