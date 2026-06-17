#include "feyman_canopen_task.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "managers/sync_signal_manager.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace {

// 创建 FreeRTOS/LibXR 线程时，除用户指定栈外还需要预留一部分堆空间。
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;

// CANopen 标准 COB-ID 基址；实际帧 ID = 基址 + 节点 ID。
constexpr uint32_t kSdoReqIdBase = 0x600;
constexpr uint32_t kSdoRespIdBase = 0x580;
constexpr uint32_t kHeartbeatIdBase = 0x700;
constexpr uint32_t kTpdoIdBase[] = {0x180, 0x280};

// NMT 网络管理命令：启动节点、进入预操作、复位通信。
constexpr uint8_t kNmtStartRemoteNode = 0x01;
constexpr uint8_t kNmtEnterPreOperational = 0x80;
constexpr uint8_t kNmtResetCommunication = 0x82;

// expedited SDO 命令字：读 4 字节、写 1/2/4 字节、abort 响应。
constexpr uint8_t kSdoRead4Byte = 0x40;
constexpr uint8_t kSdoWrite1Byte = 0x2F;
constexpr uint8_t kSdoWrite2Byte = 0x2B;
constexpr uint8_t kSdoWrite4Byte = 0x23;
constexpr uint8_t kSdoAbort = 0x80;

// FEYMAN CANopen 对象字典索引。
constexpr uint16_t kObjProducerHeartbeat = 0x1017;
constexpr uint16_t kObjTpdoCommBase = 0x1800;
constexpr uint16_t kObjTpdoMapBase = 0x1A00;
constexpr uint16_t kObjBaudRate = 0x3003;
constexpr uint16_t kObjNodeId = 0x3004;
constexpr uint16_t kObjDataFrequency = 0x3006;
constexpr uint16_t kObjWorkMode = 0x300B;
constexpr uint16_t kObjJ1939Enable = 0x300D;

// 设备工作模式：启用 CANopen，关闭 J1939。
constexpr uint8_t kWorkModeCanopen = 0x02;
constexpr uint8_t kJ1939Disabled = 0x00;

// 本地 PDO 有效位与时间同步状态位。
constexpr uint8_t kPdoValidAccel = 0x01;
constexpr uint8_t kPdoValidGyro = 0x02;
constexpr uint8_t kFeymanTimeStatusHasEpoch = 0x01;

// FEYMAN 原始量程换算系数：加速度 raw -> g，角速度 raw -> deg/s。
constexpr float kAccelScaleG = 8.0f / 32000.0f;
constexpr float kGyroScale = 500.0f / 32000.0f;
constexpr float kGravityMps2 = 9.80665f;

using LibXR::CAN;
using LibXR::ErrorCode;

/// 将 1/2/4 字节数值按小端序写入 SDO payload 数据区。
template <typename T>
void WriteLe(std::array<uint8_t, 4>& dst, T value) {
  static_assert(sizeof(T) <= 4, "payload too large");
  std::memset(dst.data(), 0, dst.size());
  std::memcpy(dst.data(), &value, sizeof(T));
}

/// 从 PDO payload 中读取小端 int16 原始值。
int16_t ReadI16Le(const uint8_t* data) {
  int16_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

}  // namespace

namespace Application {

/// 保存外部 CAN 指针和配置；真正的资源分配在 Start 中完成。
FeymanCanopenTask::FeymanCanopenTask(LibXR::CAN* can,
                                     const FeymanCanopenConfig& config)
    : can_(can), config_(config) {}

/// 启动 FEYMAN CANopen 任务：检查参数、创建 topic、注册 CAN 回调并创建线程。
ErrorCode FeymanCanopenTask::Start() {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (can_ == nullptr || config_.topic_name == nullptr ||
      config_.stack_size == 0U || config_.data_rate_hz == 0U) {
    return ErrorCode::ARG_ERR;
  }

  const size_t required_heap = static_cast<size_t>(config_.stack_size) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  if (topic_ != nullptr) {
    delete topic_;
    topic_ = nullptr;
  }
  topic_ = new LibXR::Topic(config_.topic_name, sizeof(Manager::FeymanPoseMsg),
                            nullptr, false, false, false);
  if (topic_ == nullptr) {
    return ErrorCode::NO_MEM;
  }

  running_ = true;
  device_configured_ = false;
  pdo_state_ = PdoState{};
  pending_pdo_mask_ = 0U;
  sdo_transaction_ = SdoTransaction{};
  pending_can_error_ = PendingCanError{};
  while (sdo_sem_.Wait(0U) == ErrorCode::OK) {
  }

  can_callback_ = LibXR::CAN::Callback::Create(OnCanFrame, this);
  can_->Register(can_callback_, CAN::Type::STANDARD, CAN::FilterMode::ID_RANGE,
                 0x180, 0x7FF);
  can_->Register(can_callback_, CAN::Type::ERROR);

  thread_.Create(this, TaskEntry, "FeymanCAN", config_.stack_size,
                 static_cast<LibXR::Thread::Priority>(config_.priority));
  return ErrorCode::OK;
}

/// 请求任务退出，并唤醒可能正在等待 SDO 响应的线程。
void FeymanCanopenTask::Stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  sdo_sem_.Post();
  LibXR::Thread::Sleep(config_.sdo_timeout_ms + 10U);
}

/// LibXR 线程入口，转发到对象的 Run 方法。
void FeymanCanopenTask::TaskEntry(FeymanCanopenTask* task) {
  if (task != nullptr) {
    task->Run();
  }
}

/// CAN 驱动回调入口，转发到对象的帧处理方法。
void FeymanCanopenTask::OnCanFrame(bool in_isr, FeymanCanopenTask* task,
                                   const LibXR::CAN::ClassicPack& pack) {
  if (task != nullptr) {
    task->HandleCanFrame(in_isr, pack);
  }
}

/// 任务主循环：上电等待、配置设备，然后持续发布数据并输出延迟日志。
void FeymanCanopenTask::Run() {
  Logf("[feyman] start node=0x%02X baud=%lu rate=%lu", config_.node_id,
       static_cast<unsigned long>(config_.baudrate),
       static_cast<unsigned long>(config_.data_rate_hz));

  LibXR::Thread::Sleep(config_.startup_delay_ms);
  const ErrorCode ec = ConfigureDevice();
  if (ec != ErrorCode::OK) {
    Logf("[feyman] configure failed ec=%d", static_cast<int>(ec));
  } else {
    device_configured_ = true;
    Log("[feyman] configure ok");
  }

  while (running_) {
    FlushPendingCanErrorLog();
    PublishPoseIfReady();
    LibXR::Thread::Sleep(1U);
  }
}

/// 按 CAN 帧 ID 和类型分发到 SDO、heartbeat、TPDO 或错误处理。
void FeymanCanopenTask::HandleCanFrame(bool in_isr,
                                       const LibXR::CAN::ClassicPack& pack) {
  UNUSED(in_isr);
  if (pack.type == CAN::Type::ERROR) {
    HandleCanError(pack);
    return;
  }
  if (pack.type != CAN::Type::STANDARD) {
    return;
  }

  const uint32_t node = config_.node_id;
  if (pack.id == (kSdoRespIdBase + node)) {
    HandleSdoResponse(pack);
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

/// CAN 错误可能来自中断上下文，仅记录摘要，实际打印放到任务线程。
void FeymanCanopenTask::HandleCanError(const LibXR::CAN::ClassicPack& pack) {
  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  pending_can_error_.pending = true;
  ++pending_can_error_.count;
  pending_can_error_.last_error_id = pack.id;
  pending_can_error_.state_valid =
      (can_ != nullptr &&
       can_->GetErrorState(pending_can_error_.state) == ErrorCode::OK);
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

/// 解析 SDO 响应帧，并唤醒正在 WaitSdoResponse 的任务线程。
void FeymanCanopenTask::HandleSdoResponse(const LibXR::CAN::ClassicPack& pack) {
  if (pack.dlc < 8U) {
    return;
  }

  const uint16_t index =
      static_cast<uint16_t>(pack.data[1] | (static_cast<uint16_t>(pack.data[2]) << 8U));
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
    sdo_sem_.PostFromCallback(true);
  }
}

/// 记录 heartbeat 中的 CANopen 节点状态和接收时间。
void FeymanCanopenTask::HandleHeartbeat(const LibXR::CAN::ClassicPack& pack) {
  if (pack.dlc == 0U) {
    return;
  }

  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  pdo_state_.heartbeat_state = pack.data[0];
  pdo_state_.latest_heartbeat_tick_us = LibXR::Timebase::GetMicroseconds();
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

/// 解析 TPDO1/TPDO2 的三轴 int16 数据，并更新本地 PDO 快照。
void FeymanCanopenTask::HandlePdo(PdoKind kind,
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
      pdo_state_.valid_mask = static_cast<uint8_t>(pdo_state_.valid_mask | kPdoValidAccel);
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
      pdo_state_.valid_mask = static_cast<uint8_t>(pdo_state_.valid_mask | kPdoValidGyro);
      received_mask = kPdoValidGyro;
      break;
  }

  pending_pdo_mask_ = static_cast<uint8_t>(pending_pdo_mask_ | received_mask);
  const uint8_t required_mask =
      static_cast<uint8_t>(kPdoValidAccel | kPdoValidGyro);
  if ((pending_pdo_mask_ & required_mask) == required_mask) {
    pdo_state_.latest_pdo_tick_us = LibXR::Timebase::GetMicroseconds();
    ++pdo_state_.sequence;
    pending_pdo_mask_ = 0U;
  }
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

/// 当加速度和角速度都已收到且数据更新时，发布 FeymanPoseMsg 到 topic。
void FeymanCanopenTask::PublishPoseIfReady() {
  if (!device_configured_) {
    return;
  }
  PdoState snapshot{};
  taskENTER_CRITICAL();
  snapshot = pdo_state_;
  taskEXIT_CRITICAL();

  const uint8_t required_mask =
      static_cast<uint8_t>(kPdoValidAccel | kPdoValidGyro);
  if ((snapshot.valid_mask & required_mask) != required_mask) {
    return;
  }
  if (snapshot.latest_pdo_tick_us == last_publish_tick_us_) {
    return;
  }

  Manager::FeymanPoseMsg msg{};
  msg.timestamp_us = LibXR::Timebase::GetMicroseconds();
  msg.readout_mcu_tick_us =
      Manager::SyncSignalManager::ToSessionTickUs(msg.timestamp_us);
  // 当前 TPDO 只映射加速度和角速度，姿态角与四元数暂用 NaN 表示不可用。
  msg.euler[0] = std::numeric_limits<float>::quiet_NaN();
  msg.euler[1] = std::numeric_limits<float>::quiet_NaN();
  msg.euler[2] = std::numeric_limits<float>::quiet_NaN();
  msg.acc[0] = DecodeAccelMps2(snapshot.acc_raw[0]);
  msg.acc[1] = DecodeAccelMps2(snapshot.acc_raw[1]);
  msg.acc[2] = DecodeAccelMps2(snapshot.acc_raw[2]);
  msg.gyro[0] = DecodeGyroDps(snapshot.gyro_raw[0]);
  msg.gyro[1] = DecodeGyroDps(snapshot.gyro_raw[1]);
  msg.gyro[2] = DecodeGyroDps(snapshot.gyro_raw[2]);
  msg.quaternion[0] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[1] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[2] = std::numeric_limits<float>::quiet_NaN();
  msg.quaternion[3] = std::numeric_limits<float>::quiet_NaN();
  msg.sample_timestamp = 0U;
  msg.status_flags = snapshot.status_flags;
  msg.sequence = snapshot.sequence;
  msg.heartbeat_state = snapshot.heartbeat_state;
  msg.status = 0U;

  Manager::SyncEventRecord epoch;
  if (Manager::SyncSignalManager::IsActive() &&
      Manager::SyncSignalManager::GetLatestEvent(
          Manager::SyncEventSource::TIM2_IMU_SYNC_1HZ, epoch)) {
    msg.time_status = static_cast<uint8_t>(msg.time_status | kFeymanTimeStatusHasEpoch);
    msg.sensor_mcu_tick_us = epoch.mcu_tick_us;
  }

  if (topic_ != nullptr) {
    topic_->Publish(msg);
  }
  last_publish_tick_us_ = snapshot.latest_pdo_tick_us;
}

/// 在普通任务上下文中输出 CAN 错误摘要，避免回调/中断里阻塞打印。
void FeymanCanopenTask::FlushPendingCanErrorLog() {
  PendingCanError error{};
  taskENTER_CRITICAL();
  if (!pending_can_error_.pending) {
    taskEXIT_CRITICAL();
    return;
  }
  error = pending_can_error_;
  pending_can_error_.pending = false;
  pending_can_error_.count = 0;
  taskEXIT_CRITICAL();

  Logf("[feyman] can error cnt=%lu id=0x%08lX",
       static_cast<unsigned long>(error.count),
       static_cast<unsigned long>(error.last_error_id));
  if (error.state_valid) {
    Logf("[feyman] can state ctx=isr tec=%u rec=%u boff=%u ep=%u ew=%u",
         static_cast<unsigned>(error.state.tx_error_counter),
         static_cast<unsigned>(error.state.rx_error_counter),
         static_cast<unsigned>(error.state.bus_off),
         static_cast<unsigned>(error.state.error_passive),
         static_cast<unsigned>(error.state.error_warning));
  } else {
    Log("[feyman] can state ctx=isr unavailable");
  }
}

/// 完整配置 FEYMAN：进入预操作、写基础参数、配置 TPDO、复位通信并启动节点。
ErrorCode FeymanCanopenTask::ConfigureDevice() {
  LogConfig("[feyman] configure begin");
  LogConfig("[feyman] nmt pre-op");
  ErrorCode ec = SendNmt(kNmtEnterPreOperational);
  if (ec != ErrorCode::OK) {
    Logf("[feyman] nmt pre-op send failed ec=%d", static_cast<int>(ec));
    return ec;
  }

  LibXR::Thread::Sleep(20U);
  ec = ConfigureBasicParameters();
  if (ec != ErrorCode::OK) {
    return ec;
  }

  // CANopen PDO 映射格式：index(16bit) + subindex(8bit) + bit length(8bit)。
  const uint32_t accel_map[] = {0x40010110U, 0x40010210U, 0x40010310U};
  const uint32_t gyro_map[] = {0x40020110U, 0x40020210U, 0x40020310U};

  LogConfig("[feyman] map tpdo accel");
  ec = ConfigureTpdo(0, static_cast<uint16_t>(kTpdoIdBase[0] + config_.node_id),
                     static_cast<uint8_t>(sizeof(accel_map) /
                                          sizeof(accel_map[0])),
                     accel_map);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfig("[feyman] map tpdo gyro");
  ec = ConfigureTpdo(1, static_cast<uint16_t>(kTpdoIdBase[1] + config_.node_id),
                     static_cast<uint8_t>(sizeof(gyro_map) /
                                          sizeof(gyro_map[0])),
                     gyro_map);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  LogConfig("[feyman] nmt reset-comm");
  ec = SendNmt(kNmtResetCommunication);
  if (ec != ErrorCode::OK) {
    Logf("[feyman] nmt reset-comm send failed ec=%d", static_cast<int>(ec));
    return ec;
  }
  LibXR::Thread::Sleep(50U);

  LogConfig("[feyman] nmt start");
  return SendNmt(kNmtStartRemoteNode);
}

/// 配置设备基础对象字典参数，并读取关键参数用于日志确认。
ErrorCode FeymanCanopenTask::ConfigureBasicParameters() {
  LogConfig("[feyman] basic params");

  uint32_t readback = 0U;
  LogConfig("[feyman] sdo read 0x3003");
  ErrorCode ec = SdoReadU32(kObjBaudRate, 0x00U, &readback);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfigf("[feyman] baud current=%lu",
             static_cast<unsigned long>(readback));

  LogConfig("[feyman] sdo write 0x3003");
  ec = SdoWriteU32(kObjBaudRate, 0x00U, config_.baudrate);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfig("[feyman] sdo write 0x3004");
  ec = SdoWriteU8(kObjNodeId, 0x00U, config_.node_id);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfig("[feyman] sdo write 0x3006");
  ec = SdoWriteU32(kObjDataFrequency, 0x00U, config_.data_rate_hz);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfig("[feyman] sdo write 0x1017");
  ec = SdoWriteU16(kObjProducerHeartbeat, 0x00U,
                   static_cast<uint16_t>(config_.heartbeat_ms));
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfig("[feyman] sdo write 0x300B");
  ec = SdoWriteU8(kObjWorkMode, 0x00U, kWorkModeCanopen);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  if (config_.work_mode_settle_ms != 0U) {
    LibXR::Thread::Sleep(config_.work_mode_settle_ms);
  }

  LogConfig("[feyman] sdo write 0x300D");
  ec = SdoWriteU8(kObjJ1939Enable, 0x00U, kJ1939Disabled);
  if (ec != ErrorCode::OK) {
    return ec;
  }

  uint32_t node_readback = 0U;
  LogConfig("[feyman] sdo read 0x3004");
  ec = SdoReadU32(kObjNodeId, 0x00U, &node_readback);
  if (ec != ErrorCode::OK) {
    return ec;
  }
  LogConfigf("[feyman] node current=0x%02lX",
             static_cast<unsigned long>(node_readback & 0xFFUL));
  return ErrorCode::OK;
}

/// 配置指定 TPDO 的通信参数、映射项和事件触发周期。
ErrorCode FeymanCanopenTask::ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                           uint8_t map_count,
                                           const uint32_t* mappings) {
  const uint32_t event_timer_ms = 1000U / config_.data_rate_hz;
  if (event_timer_ms == 0U || event_timer_ms > UINT16_MAX) {
    return ErrorCode::OUT_OF_RANGE;
  }

  const uint16_t comm_index =
      static_cast<uint16_t>(kObjTpdoCommBase + pdo_index);
  const uint16_t map_index =
      static_cast<uint16_t>(kObjTpdoMapBase + pdo_index);
  LogConfigf("[feyman] tpdo cfg comm=0x%04X map=0x%04X cob=0x%03X",
             static_cast<unsigned>(comm_index),
             static_cast<unsigned>(map_index), static_cast<unsigned>(cob_id));
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

/// 发送 NMT 命令到当前节点。
ErrorCode FeymanCanopenTask::SendNmt(uint8_t command) {
  const uint8_t payload[2] = {command, config_.node_id};
  return SendCanFrame(0x000U, payload, 2U, CAN::Type::STANDARD);
}

/// 通过 expedited SDO 读取 32 位无符号值。
ErrorCode FeymanCanopenTask::SdoReadU32(uint16_t index, uint8_t subindex,
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

/// 通过 expedited SDO 写入 8 位无符号值。
ErrorCode FeymanCanopenTask::SdoWriteU8(uint16_t index, uint8_t subindex,
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

/// 通过 expedited SDO 写入 16 位无符号值。
ErrorCode FeymanCanopenTask::SdoWriteU16(uint16_t index, uint8_t subindex,
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

/// 通过 expedited SDO 写入 32 位无符号值。
ErrorCode FeymanCanopenTask::SdoWriteU32(uint16_t index, uint8_t subindex,
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

/// 组装 SDO 请求帧，登记待匹配事务，然后发送到 0x600 + node_id。
ErrorCode FeymanCanopenTask::SendSdoRequest(
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

  while (sdo_sem_.Wait(0U) == ErrorCode::OK) {
  }

  return SendCanFrame(kSdoReqIdBase + config_.node_id, frame.data(), 8U,
                      CAN::Type::STANDARD);
}

/// 等待 SDO 响应；超时或 abort 时返回错误并输出诊断日志。
ErrorCode FeymanCanopenTask::WaitSdoResponse(uint16_t index, uint8_t subindex,
                                             SdoResponse& response) {
  const ErrorCode ec = sdo_sem_.Wait(config_.sdo_timeout_ms);
  if (ec != ErrorCode::OK) {
    taskENTER_CRITICAL();
    sdo_transaction_.active = false;
    taskEXIT_CRITICAL();
    Logf("[feyman] sdo timeout idx=0x%04X sub=0x%02X", index, subindex);
    LogCanErrorState("sdo-timeout");
    return ec;
  }

  taskENTER_CRITICAL();
  response = sdo_transaction_.response;
  taskEXIT_CRITICAL();
  if (response.abort) {
    Logf("[feyman] sdo abort idx=0x%04X sub=0x%02X code=0x%08lX", index,
         subindex, static_cast<unsigned long>(response.abort_code));
    return ErrorCode::FAILED;
  }
  DelayBetweenSdoRequests();
  return ErrorCode::OK;
}

/// 根据配置给设备留出连续 SDO 请求之间的处理时间。
void FeymanCanopenTask::DelayBetweenSdoRequests() {
  if (config_.sdo_inter_request_delay_ms != 0U) {
    LibXR::Thread::Sleep(config_.sdo_inter_request_delay_ms);
  }
}

/// 构造 LibXR Classic CAN 帧并提交给 CAN 驱动发送队列。
ErrorCode FeymanCanopenTask::SendCanFrame(uint32_t id, const uint8_t* data,
                                          uint8_t dlc, CAN::Type type) {
  if (data == nullptr || dlc > 8U) {
    return ErrorCode::ARG_ERR;
  }
  CAN::ClassicPack pack{};
  pack.id = id;
  pack.type = type;
  pack.dlc = dlc;
  std::memcpy(pack.data, data, dlc);
  return can_->AddMessage(pack);
}

/// 输出一行普通日志；未配置 log_writer 时静默忽略。
void FeymanCanopenTask::Log(const char* text) {
  if (config_.log_writer != nullptr && text != nullptr) {
    config_.log_writer(text);
  }
}

/// verbose_config_log 开启时输出配置流程日志。
void FeymanCanopenTask::LogConfig(const char* text) {
  if (config_.verbose_config_log) {
    Log(text);
  }
}

/// 带格式化的一行普通日志输出。
void FeymanCanopenTask::Logf(const char* fmt, ...) {
  if (config_.log_writer == nullptr || fmt == nullptr) {
    return;
  }
  char line[160] = {0};
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  config_.log_writer(line);
}

/// verbose_config_log 开启时输出带格式化的配置日志。
void FeymanCanopenTask::LogConfigf(const char* fmt, ...) {
  if (!config_.verbose_config_log || config_.log_writer == nullptr ||
      fmt == nullptr) {
    return;
  }
  char line[160] = {0};
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  config_.log_writer(line);
}

/// 读取 CAN 控制器错误计数/状态并输出，常用于 SDO 超时诊断。
void FeymanCanopenTask::LogCanErrorState(const char* context) {
  if (can_ == nullptr) {
    return;
  }

  LibXR::CAN::ErrorState state{};
  const ErrorCode ec = can_->GetErrorState(state);
  if (ec != ErrorCode::OK) {
    Logf("[feyman] can state unavailable ctx=%s ec=%d",
         (context != nullptr) ? context : "-", static_cast<int>(ec));
    return;
  }

  Logf("[feyman] can state ctx=%s tec=%u rec=%u boff=%u ep=%u ew=%u",
       (context != nullptr) ? context : "-",
       static_cast<unsigned>(state.tx_error_counter),
       static_cast<unsigned>(state.rx_error_counter),
       static_cast<unsigned>(state.bus_off),
       static_cast<unsigned>(state.error_passive),
       static_cast<unsigned>(state.error_warning));
}

/// FEYMAN 加速度原始值换算：raw -> g -> m/s^2。
float FeymanCanopenTask::DecodeAccelMps2(int16_t raw) {
  return static_cast<float>(raw) * kAccelScaleG * kGravityMps2;
}

/// FEYMAN 角速度原始值换算：raw -> deg/s。
float FeymanCanopenTask::DecodeGyroDps(int16_t raw) {
  return static_cast<float>(raw) * kGyroScale;
}

}  // namespace Application
