#include "feyman_mcs10.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "stm32_timebase.hpp"
#include "task.h"
#include "thread.hpp"

namespace {

// 标准 CANopen COB-ID 基址，实际帧 ID = 基址 + node_id。
constexpr uint32_t kSdoReqIdBase = 0x600;
constexpr uint32_t kSdoRespIdBase = 0x580;
constexpr uint32_t kHeartbeatIdBase = 0x700;
constexpr uint32_t kTpdoIdBase[] = {0x180, 0x280};

// 本模块当前只需要最小的一组 NMT 命令。
constexpr uint8_t kNmtStartRemoteNode = 0x01;
constexpr uint8_t kNmtEnterPreOperational = 0x80;
constexpr uint8_t kNmtResetCommunication = 0x82;

// expedited SDO 命令字。
constexpr uint8_t kSdoRead4Byte = 0x40;
constexpr uint8_t kSdoWrite1Byte = 0x2F;
constexpr uint8_t kSdoWrite2Byte = 0x2B;
constexpr uint8_t kSdoWrite4Byte = 0x23;
constexpr uint8_t kSdoAbort = 0x80;

// FEYMAN MCS10 当前联调用到的对象字典项。
constexpr uint16_t kObjProducerHeartbeat = 0x1017;
constexpr uint16_t kObjTpdoCommBase = 0x1800;
constexpr uint16_t kObjTpdoMapBase = 0x1A00;
constexpr uint16_t kObjBaudRate = 0x3003;
constexpr uint16_t kObjNodeId = 0x3004;
constexpr uint16_t kObjDataFrequency = 0x3006;
constexpr uint16_t kObjWorkMode = 0x300B;
constexpr uint16_t kObjJ1939Enable = 0x300D;

// 仅启用 CANopen，关闭 J1939。
constexpr uint8_t kWorkModeCanopen = 0x02;
constexpr uint8_t kJ1939Disabled = 0x00;

// 标记当前本地快照里哪几路 PDO 已经到齐。
constexpr uint8_t kPdoValidAccel = 0x01;
constexpr uint8_t kPdoValidGyro = 0x02;

// FEYMAN 原始量程转换系数。
constexpr float kAccelScaleG = 8.0f / 32000.0f;
constexpr float kGyroScale = 500.0f / 32000.0f;
constexpr float kGravityMps2 = 9.80665f;

using LibXR::CAN;
using LibXR::ErrorCode;

// 把 1/2/4 字节值按小端写入 SDO payload。
template <typename T>
void WriteLe(std::array<uint8_t, 4>& dst, T value) {
  static_assert(sizeof(T) <= 4, "payload too large");
  std::memset(dst.data(), 0, dst.size());
  std::memcpy(dst.data(), &value, sizeof(T));
}

// 从 PDO 负载里取出 int16 原始值。
int16_t ReadI16Le(const uint8_t* data) {
  int16_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

}  // namespace

namespace Module {

FeymanMCS10::FeymanMCS10(LibXR::CAN* can,
                         const FeymanMCS10Config& config) {
  (void)Init(can, config);
}

ErrorCode FeymanMCS10::Init(LibXR::CAN* can,
                            const FeymanMCS10Config& config) {
  if (can == nullptr || config.data_rate_hz == 0U) {
    return ErrorCode::ARG_ERR;
  }

  can_ = can;
  config_ = config;
  running_ = true;
  sdo_transaction_ = SdoTransaction{};
  pdo_state_ = PdoState{};
  pending_pdo_mask_ = 0U;
  pending_can_error_ = FeymanMCS10CanError{};
  last_polled_tick_us_ = 0U;
  while (sdo_sem_.Wait(0U) == ErrorCode::OK) {
  }

  // 模块层直接订阅 FEYMAN 相关 CAN 数据，上层不再碰 CAN 回调细节。
  can_callback_ = LibXR::CAN::Callback::Create(OnCanFrame, this);
  can_->Register(can_callback_, CAN::Type::STANDARD, CAN::FilterMode::ID_RANGE,
                 0x180, 0x7FF);
  can_->Register(can_callback_, CAN::Type::ERROR);
  return ErrorCode::OK;
}

ErrorCode FeymanMCS10::Configure() {
  if (can_ == nullptr || !running_) {
    return ErrorCode::INIT_ERR;
  }

  // 先进入 pre-operational，再写对象字典，最后 reset/start。
  ErrorCode ec = SendNmt(kNmtEnterPreOperational);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  LibXR::Thread::Sleep(20U);
  ec = ConfigureBasicParameters();
  if (ec != ErrorCode::OK) {
    return ec;
  }

  const uint32_t accel_map[] = {0x40010110U, 0x40010210U, 0x40010310U};
  const uint32_t gyro_map[] = {0x40020110U, 0x40020210U, 0x40020310U};

  ec = ConfigureTpdo(0, static_cast<uint16_t>(kTpdoIdBase[0] + config_.node_id),
                     static_cast<uint8_t>(sizeof(accel_map) /
                                          sizeof(accel_map[0])),
                     accel_map);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  ec = ConfigureTpdo(1, static_cast<uint16_t>(kTpdoIdBase[1] + config_.node_id),
                     static_cast<uint8_t>(sizeof(gyro_map) /
                                          sizeof(gyro_map[0])),
                     gyro_map);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  ec = SendNmt(kNmtResetCommunication);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LibXR::Thread::Sleep(50U);

  return SendNmt(kNmtStartRemoteNode);
}

ErrorCode FeymanMCS10::PollSample(FeymanMCS10Sample& sample) {
  PdoState snapshot{};
  taskENTER_CRITICAL();
  snapshot = pdo_state_;
  taskEXIT_CRITICAL();

  // 只有 accel + gyro 都收到，并且是 application 尚未消费过的新序列，才返回 OK。
  constexpr uint8_t required_mask =
      static_cast<uint8_t>(kPdoValidAccel | kPdoValidGyro);
  if ((snapshot.valid_mask & required_mask) != required_mask) {
    return ErrorCode::NOT_FOUND;
  }
  if (snapshot.latest_pdo_tick_us == last_polled_tick_us_) {
    return ErrorCode::EMPTY;
  }

  sample.acc[0] = DecodeAccelMps2(snapshot.acc_raw[0]);
  sample.acc[1] = DecodeAccelMps2(snapshot.acc_raw[1]);
  sample.acc[2] = DecodeAccelMps2(snapshot.acc_raw[2]);
  sample.gyro[0] = DecodeGyroDps(snapshot.gyro_raw[0]);
  sample.gyro[1] = DecodeGyroDps(snapshot.gyro_raw[1]);
  sample.gyro[2] = DecodeGyroDps(snapshot.gyro_raw[2]);
  sample.sequence = snapshot.sequence;
  sample.heartbeat_state = snapshot.heartbeat_state;
  sample.status_flags = snapshot.status_flags;
  sample.latest_pdo_tick_us = snapshot.latest_pdo_tick_us;
  last_polled_tick_us_ = snapshot.latest_pdo_tick_us;
  return ErrorCode::OK;
}

void FeymanMCS10::Stop() {
  running_ = false;
  sdo_sem_.Post();
}

bool FeymanMCS10::TakePendingCanError(FeymanMCS10CanError& error) {
  taskENTER_CRITICAL();
  if (!pending_can_error_.pending) {
    taskEXIT_CRITICAL();
    return false;
  }
  error = pending_can_error_;
  pending_can_error_.pending = false;
  pending_can_error_.count = 0;
  taskEXIT_CRITICAL();
  return true;
}

ErrorCode FeymanMCS10::GetCanErrorState(LibXR::CAN::ErrorState& state) const {
  if (can_ == nullptr) {
    return ErrorCode::PTR_NULL;
  }
  return can_->GetErrorState(state);
}

void FeymanMCS10::OnCanFrame(bool in_isr, FeymanMCS10* device,
                             const LibXR::CAN::ClassicPack& pack) {
  if (device != nullptr) {
    device->HandleCanFrame(in_isr, pack);
  }
}

void FeymanMCS10::HandleCanFrame(bool in_isr,
                                 const LibXR::CAN::ClassicPack& pack) {
  if (pack.type == CAN::Type::ERROR) {
    HandleCanError(pack);
    return;
  }
  if (pack.type != CAN::Type::STANDARD) {
    return;
  }

  const uint32_t node = config_.node_id;
  if (pack.id == (kSdoRespIdBase + node)) {
    HandleSdoResponse(in_isr, pack);
    return;
  }
  if (pack.id == (kHeartbeatIdBase + node)) {
    HandleHeartbeat(pack);
    return;
  }
  if (pack.id == (kTpdoIdBase[0] + node)) {
    HandlePdo(PdoKind::TPDO1_ACCEL, pack);
    return;
  }
  if (pack.id == (kTpdoIdBase[1] + node)) {
    HandlePdo(PdoKind::TPDO2_GYRO, pack);
  }
}

void FeymanMCS10::HandleCanError(const LibXR::CAN::ClassicPack& pack) {
  // 中断里只做摘要缓存，不直接打印。
  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  pending_can_error_.pending = true;
  ++pending_can_error_.count;
  pending_can_error_.last_error_id = pack.id;
  pending_can_error_.state_valid =
      (can_ != nullptr &&
       can_->GetErrorState(pending_can_error_.state) == ErrorCode::OK);
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

void FeymanMCS10::HandleSdoResponse(bool in_isr,
                                    const LibXR::CAN::ClassicPack& pack) {
  if (pack.dlc < 8U) {
    return;
  }

  const uint16_t index = static_cast<uint16_t>(
      pack.data[1] | (static_cast<uint16_t>(pack.data[2]) << 8U));
  const uint8_t subindex = pack.data[3];

  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  const bool match = running_ && sdo_transaction_.active &&
                     sdo_transaction_.index == index &&
                     sdo_transaction_.subindex == subindex;
  if (match) {
    sdo_transaction_.response.index = index;
    sdo_transaction_.response.subindex = subindex;
    sdo_transaction_.response.command = pack.data[0];
    sdo_transaction_.response.abort = (pack.data[0] == kSdoAbort);
    std::memcpy(sdo_transaction_.response.data.data(), pack.data + 4,
                sdo_transaction_.response.data.size());
    if (sdo_transaction_.response.abort) {
      std::memcpy(&sdo_transaction_.response.abort_code, pack.data + 4,
                  sizeof(uint32_t));
    } else {
      sdo_transaction_.response.abort_code = 0U;
    }
    sdo_transaction_.active = false;
  }
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);

  if (match) {
    // 这里必须使用 callback-safe post，避免 ISR 里误走普通 RTOS API。
    sdo_sem_.PostFromCallback(in_isr);
  }
}

void FeymanMCS10::HandleHeartbeat(const LibXR::CAN::ClassicPack& pack) {
  if (pack.dlc == 0U) {
    return;
  }

  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  pdo_state_.heartbeat_state = pack.data[0];
  pdo_state_.latest_heartbeat_tick_us = LibXR::Timebase::GetMicroseconds();
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

void FeymanMCS10::HandlePdo(PdoKind kind,
                            const LibXR::CAN::ClassicPack& pack) {
  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  uint8_t received_mask = 0U;
  switch (kind) {
    case PdoKind::TPDO1_ACCEL:
      if (pack.dlc < 6U) {
        taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
        return;
      }
      pdo_state_.acc_raw[0] = ReadI16Le(pack.data + 0);
      pdo_state_.acc_raw[1] = ReadI16Le(pack.data + 2);
      pdo_state_.acc_raw[2] = ReadI16Le(pack.data + 4);
      pdo_state_.valid_mask =
          static_cast<uint8_t>(pdo_state_.valid_mask | kPdoValidAccel);
      received_mask = kPdoValidAccel;
      break;

    case PdoKind::TPDO2_GYRO:
      if (pack.dlc < 6U) {
        taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
        return;
      }
      pdo_state_.gyro_raw[0] = ReadI16Le(pack.data + 0);
      pdo_state_.gyro_raw[1] = ReadI16Le(pack.data + 2);
      pdo_state_.gyro_raw[2] = ReadI16Le(pack.data + 4);
      pdo_state_.valid_mask =
          static_cast<uint8_t>(pdo_state_.valid_mask | kPdoValidGyro);
      received_mask = kPdoValidGyro;
      break;
  }

  pending_pdo_mask_ = static_cast<uint8_t>(pending_pdo_mask_ | received_mask);
  constexpr uint8_t required_mask =
      static_cast<uint8_t>(kPdoValidAccel | kPdoValidGyro);
  if ((pending_pdo_mask_ & required_mask) == required_mask) {
    // 只有两路 TPDO 都到齐，才推进一帧完整样本的时间戳和 sequence。
    pdo_state_.latest_pdo_tick_us = LibXR::Timebase::GetMicroseconds();
    ++pdo_state_.sequence;
    pending_pdo_mask_ = 0U;
  }
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

ErrorCode FeymanMCS10::ConfigureBasicParameters() {
  uint32_t readback = 0U;
  ErrorCode ec = SdoReadU32(kObjBaudRate, 0x00U, &readback);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  ec = SdoWriteU32(kObjBaudRate, 0x00U, config_.baudrate);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  ec = SdoWriteU8(kObjNodeId, 0x00U, config_.node_id);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  ec = SdoWriteU32(kObjDataFrequency, 0x00U, config_.data_rate_hz);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  ec = SdoWriteU16(kObjProducerHeartbeat, 0x00U,
                   static_cast<uint16_t>(config_.heartbeat_ms));
  if (ec != ErrorCode::OK) {
    return ec;
  }
  ec = SdoWriteU8(kObjWorkMode, 0x00U, kWorkModeCanopen);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  if (config_.work_mode_settle_ms != 0U) {
    LibXR::Thread::Sleep(config_.work_mode_settle_ms);
  }

  ec = SdoWriteU8(kObjJ1939Enable, 0x00U, kJ1939Disabled);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  uint32_t node_readback = 0U;
  return SdoReadU32(kObjNodeId, 0x00U, &node_readback);
}

ErrorCode FeymanMCS10::ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                     uint8_t map_count,
                                     const uint32_t* mappings) {
  if (mappings == nullptr || map_count == 0U) {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t event_timer_ms = 1000U / config_.data_rate_hz;
  if (event_timer_ms == 0U || event_timer_ms > UINT16_MAX) {
    return ErrorCode::OUT_OF_RANGE;
  }

  const uint16_t comm_index =
      static_cast<uint16_t>(kObjTpdoCommBase + pdo_index);
  const uint16_t map_index =
      static_cast<uint16_t>(kObjTpdoMapBase + pdo_index);

  // 按 CANopen 流程：先禁用 PDO -> 清空映射 -> 写入映射 -> 设置 event timer -> 重新使能。
  ErrorCode ec = SdoWriteU32(comm_index, 0x01U,
                             static_cast<uint32_t>(0x80000000UL | cob_id));
  if (ec != ErrorCode::OK) {
    return ec;
  }

  ec = SdoWriteU8(map_index, 0x00U, 0U);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  for (uint8_t i = 0; i < map_count; ++i) {
    ec = SdoWriteU32(map_index, static_cast<uint8_t>(i + 1U), mappings[i]);
    if (ec != ErrorCode::OK) {
      return ec;
    }
  }

  ec = SdoWriteU8(map_index, 0x00U, map_count);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  ec = SdoWriteU16(comm_index, 0x05U,
                   static_cast<uint16_t>(event_timer_ms));
  if (ec != ErrorCode::OK) {
    return ec;
  }

  return SdoWriteU32(comm_index, 0x01U, static_cast<uint32_t>(cob_id));
}

ErrorCode FeymanMCS10::SendNmt(uint8_t command) {
  const uint8_t payload[2] = {command, config_.node_id};
  return SendCanFrame(0x000U, payload, 2U, CAN::Type::STANDARD);
}

ErrorCode FeymanMCS10::SdoReadU32(uint16_t index, uint8_t subindex,
                                  uint32_t* value_out) {
  if (value_out == nullptr) {
    return ErrorCode::PTR_NULL;
  }
  ErrorCode ec = SendSdoRequest(kSdoRead4Byte, index, subindex, {0, 0, 0, 0});
  if (ec != ErrorCode::OK) {
    return ec;
  }

  SdoResponse response{};
  ec = WaitSdoResponse(index, subindex, response);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  std::memcpy(value_out, response.data.data(), sizeof(uint32_t));
  return ErrorCode::OK;
}

ErrorCode FeymanMCS10::SdoWriteU8(uint16_t index, uint8_t subindex,
                                  uint8_t value) {
  std::array<uint8_t, 4> data{};
  WriteLe(data, value);
  ErrorCode ec = SendSdoRequest(kSdoWrite1Byte, index, subindex, data);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  SdoResponse response{};
  return WaitSdoResponse(index, subindex, response);
}

ErrorCode FeymanMCS10::SdoWriteU16(uint16_t index, uint8_t subindex,
                                   uint16_t value) {
  std::array<uint8_t, 4> data{};
  WriteLe(data, value);
  ErrorCode ec = SendSdoRequest(kSdoWrite2Byte, index, subindex, data);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  SdoResponse response{};
  return WaitSdoResponse(index, subindex, response);
}

ErrorCode FeymanMCS10::SdoWriteU32(uint16_t index, uint8_t subindex,
                                   uint32_t value) {
  std::array<uint8_t, 4> data{};
  WriteLe(data, value);
  ErrorCode ec = SendSdoRequest(kSdoWrite4Byte, index, subindex, data);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  SdoResponse response{};
  return WaitSdoResponse(index, subindex, response);
}

ErrorCode FeymanMCS10::SendSdoRequest(
    uint8_t command, uint16_t index, uint8_t subindex,
    const std::array<uint8_t, 4>& data) {
  std::array<uint8_t, 8> frame{};
  frame[0] = command;
  frame[1] = static_cast<uint8_t>(index & 0xFFU);
  frame[2] = static_cast<uint8_t>((index >> 8U) & 0xFFU);
  frame[3] = subindex;
  std::memcpy(frame.data() + 4, data.data(), data.size());

  taskENTER_CRITICAL();
  sdo_transaction_.active = true;
  sdo_transaction_.index = index;
  sdo_transaction_.subindex = subindex;
  sdo_transaction_.response = SdoResponse{};
  taskEXIT_CRITICAL();

  // 清空可能残留的旧信号量状态，避免把上一次事务的通知误当成这一次的响应。
  while (sdo_sem_.Wait(0U) == ErrorCode::OK) {
  }

  return SendCanFrame(kSdoReqIdBase + config_.node_id, frame.data(), 8U,
                      CAN::Type::STANDARD);
}

ErrorCode FeymanMCS10::WaitSdoResponse(uint16_t index, uint8_t subindex,
                                       SdoResponse& response) {
  const ErrorCode ec = sdo_sem_.Wait(config_.sdo_timeout_ms);
  if (ec != ErrorCode::OK) {
    taskENTER_CRITICAL();
    sdo_transaction_.active = false;
    taskEXIT_CRITICAL();
    return ec;
  }

  taskENTER_CRITICAL();
  response = sdo_transaction_.response;
  taskEXIT_CRITICAL();

  // 除了 abort，也顺手校验响应是否真的属于当前等待的 index/subindex。
  if (response.index != index || response.subindex != subindex) {
    return ErrorCode::CHECK_ERR;
  }
  if (response.abort) {
    return ErrorCode::FAILED;
  }
  DelayBetweenSdoRequests();
  return ErrorCode::OK;
}

void FeymanMCS10::DelayBetweenSdoRequests() {
  if (config_.sdo_inter_request_delay_ms != 0U) {
    LibXR::Thread::Sleep(config_.sdo_inter_request_delay_ms);
  }
}

ErrorCode FeymanMCS10::SendCanFrame(uint32_t id, const uint8_t* data,
                                    uint8_t dlc, CAN::Type type) {
  if (can_ == nullptr || data == nullptr || dlc > 8U) {
    return ErrorCode::ARG_ERR;
  }
  CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = type;
  pack.dlc = dlc;
  std::memcpy(pack.data, data, dlc);
  return can_->AddMessage(pack);
}

float FeymanMCS10::DecodeAccelMps2(int16_t raw) {
  return static_cast<float>(raw) * kAccelScaleG * kGravityMps2;
}

float FeymanMCS10::DecodeGyroDps(int16_t raw) {
  return static_cast<float>(raw) * kGyroScale;
}

}  // namespace Module
