#include "imu_manager.hpp"

#include <limits>

#include "FreeRTOS.h"
#include "libxr_rw.hpp"
#include "stm32_timebase.hpp"
#include "task.h"

namespace Manager {

using namespace LibXR;

namespace {

// JY901B 连续数据区起始寄存器：加速度、角速度、磁场、欧拉角等都从这里往后读取。
constexpr uint32_t kDataRegStart = 0x34;
constexpr uint32_t kDataRegCount = 13;
// 四元数数据单独放在另一段寄存器区。
constexpr uint32_t kQuatRegStart = 0x51;
constexpr uint32_t kQuatRegCount = 4;
// 给 IMU 上电稳定留一点时间，再开始初始化探测。
constexpr uint32_t kInitBootDelayMs = 120;
constexpr uint32_t kInitProbeRetryCount = 5;
constexpr uint32_t kInitProbeRetryDelayMs = 25;
// 采集线程使用的任务栈和预留堆空间。
constexpr uint32_t kAcquisitionStackBytes = 1024;
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;

}  // namespace

IMUManager::IMUManager(uint8_t imu_count)
    : i2c_(nullptr),
      imu_count_(imu_count > MAX_IMU_COUNT ? MAX_IMU_COUNT : imu_count),
      base_address_(kDefaultImuAddress),
      online_mask_(0),
      sequence_(0),
      data_topic_(nullptr),
      running_(false),
      frequency_hz_(100) {
  imus_.fill(nullptr);
}

IMUManager::~IMUManager() {
  StopAcquisition();

  // manager 销毁时统一释放底层 Module::WitIMU 对象，避免资源泄漏。
  for (auto*& imu : imus_) {
    delete imu;
    imu = nullptr;
  }
}

ErrorCode IMUManager::Init(I2C* i2c, uint8_t base_address) {
  if (i2c == nullptr) {
    return ErrorCode::PTR_NULL;
  }

  i2c_ = i2c;
  base_address_ = base_address;
  online_mask_ = 0;
  sequence_ = 0;

  // 让传感器有充足的上电稳定时间，避免刚上电时探测失败。
  Thread::Sleep(kInitBootDelayMs);

  uint8_t success_count = 0;
  for (uint8_t i = 0; i < imu_count_; ++i) {
    delete imus_[i];
    imus_[i] = new Module::WitIMU(i2c_, ResolveImuI2CAddress(i, base_address_));
    if (imus_[i] == nullptr) {
      continue;
    }

    if (imus_[i]->Init() != Module::WitIMU::ErrorCode::OK) {
      continue;
    }

    // 上电阶段可能存在偶发探测失败，这里做有限次重试提高成功率。
    bool online = false;
    for (uint32_t attempt = 0; attempt < kInitProbeRetryCount; ++attempt) {
      if (ProbeIMU(i)) {
        online = true;
        break;
      }
      if ((attempt + 1U) < kInitProbeRetryCount) {
        Thread::Sleep(kInitProbeRetryDelayMs);
      }
    }
    if (online) {
      ++success_count;
    }
  }

  return (success_count > 0) ? ErrorCode::OK : ErrorCode::INIT_ERR;
}

ErrorCode IMUManager::ReadAll(IMUArrayMsg& msg) {
  // 采集线程和桥接线程都可能访问 I2C，总线读流程必须串行化。
  Mutex::LockGuard guard(bus_mutex_);

  msg = IMUArrayMsg{};
  msg.sequence = sequence_++;

  uint8_t valid_count = 0;
  const uint8_t scan_count =
      (imu_count_ < ACTUAL_IMU_COUNT) ? imu_count_ : ACTUAL_IMU_COUNT;
  for (uint8_t i = 0; i < scan_count; ++i) {
    if (imus_[i] == nullptr || !IsIMUOnline(i)) {
      continue;
    }

    Module::WitIMU::ImuData raw{};
    if (!ReadRawIMU(i, raw)) {
      continue;
    }

    // 这里把底层模块量纲转换成对业务层友好的统一结构，并补上时间戳。
    ConvertIMUData(raw, msg.imu_data[i]);
    msg.imu_data[i].timestamp_us = Timebase::GetMicroseconds();
    msg.SetValid(i, true);
    ++valid_count;
  }

  msg.timestamp_us = Timebase::GetMicroseconds();
  return (valid_count > 0) ? ErrorCode::OK : ErrorCode::FAILED;
}

ErrorCode IMUManager::ReadSingle(uint8_t index, IMUData& data) {
  if (index >= imu_count_ || index >= ACTUAL_IMU_COUNT) {
    return ErrorCode::ARG_ERR;
  }
  if (imus_[index] == nullptr || !IsIMUOnline(index)) {
    return ErrorCode::NOT_FOUND;
  }

  Mutex::LockGuard guard(bus_mutex_);
  Module::WitIMU::ImuData raw{};
  if (!ReadRawIMU(index, raw)) {
    return ErrorCode::FAILED;
  }

  ConvertIMUData(raw, data);
  data.timestamp_us = Timebase::GetMicroseconds();
  return ErrorCode::OK;
}

ErrorCode IMUManager::StartAcquisition(uint32_t frequency_hz,
                                       const char* topic_name) {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (frequency_hz == 0U) {
    return ErrorCode::ARG_ERR;
  }
  if (online_mask_ == 0U) {
    return ErrorCode::INIT_ERR;
  }

  const size_t required_heap = static_cast<size_t>(kAcquisitionStackBytes) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  // 在线程创建前先检查 FreeRTOS 剩余堆，避免任务起到一半因为内存不足失败。
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  if (data_topic_ != nullptr) {
    delete data_topic_;
    data_topic_ = nullptr;
  }

  if (topic_name != nullptr) {
    data_topic_ =
        new Topic(topic_name, sizeof(IMUArrayMsg), nullptr, false, false, false);
    if (data_topic_ == nullptr) {
      return ErrorCode::NO_MEM;
    }
  }

  frequency_hz_ = frequency_hz;
  running_ = true;
  // 采集任务优先级设成 HIGH，保证姿态数据尽量按频率稳定产生。
  acquisition_thread_.Create(this, AcquisitionThreadFunc, "IMUMgrAcq",
                             kAcquisitionStackBytes, Thread::Priority::HIGH);
  return ErrorCode::OK;
}

void IMUManager::StopAcquisition() {
  if (!running_) {
    return;
  }

  running_ = false;

  uint32_t period_ms = (frequency_hz_ > 0U) ? (1000U / frequency_hz_) : 1U;
  if (period_ms == 0U) {
    period_ms = 1U;
  }
  // 给线程留出 1~2 个周期自然退出的时间，再回收 Topic 资源。
  Thread::Sleep(period_ms * 2U);

  if (data_topic_ != nullptr) {
    delete data_topic_;
    data_topic_ = nullptr;
  }
}

bool IMUManager::IsIMUOnline(uint8_t index) const {
  return (index < imu_count_) && ((online_mask_ & (1u << index)) != 0U);
}

uint8_t IMUManager::GetOnlineCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < imu_count_; ++i) {
    if (IsIMUOnline(i)) {
      ++count;
    }
  }
  return count;
}

bool IMUManager::AcquireBus() { return bus_mutex_.Lock() == ErrorCode::OK; }

void IMUManager::ReleaseBus() { bus_mutex_.Unlock(); }

bool IMUManager::ReadRawIMU(uint8_t index, Module::WitIMU::ImuData& raw_data) {
  if (index >= imu_count_ || imus_[index] == nullptr) {
    return false;
  }

  if (imus_[index]->ReadReg(kDataRegStart, kDataRegCount) !=
      Module::WitIMU::ErrorCode::OK) {
    return false;
  }

  // 先读主数据区，再补读四元数区；四元数失败时其余姿态数据仍然允许保留。
  const bool quat_ok =
      imus_[index]->ReadReg(kQuatRegStart, kQuatRegCount) ==
      Module::WitIMU::ErrorCode::OK;
  imus_[index]->UpdateDataFromRegisters();
  imus_[index]->GetData(raw_data);
  if (!quat_ok) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    raw_data.q0 = nan;
    raw_data.q1 = nan;
    raw_data.q2 = nan;
    raw_data.q3 = nan;
    raw_data.quat_valid = false;
  }

  return raw_data.acc_valid || raw_data.gyro_valid || raw_data.angle_valid;
}

bool IMUManager::ProbeIMU(uint8_t index) {
  if (index >= imu_count_ || imus_[index] == nullptr) {
    return false;
  }

  // Probe 本质上就是做一次真实读操作：能读通就记为在线，并切到 9 轴融合模式。
  Module::WitIMU::ImuData raw{};
  if (!ReadRawIMU(index, raw)) {
    online_mask_ = static_cast<uint16_t>(online_mask_ & ~(1u << index));
    return false;
  }

  online_mask_ = static_cast<uint16_t>(online_mask_ | (1u << index));
  (void)imus_[index]->SetAxis9();
  return true;
}

void IMUManager::AcquisitionThreadFunc(IMUManager* manager) {
  if (manager == nullptr) {
    return;
  }

  uint32_t period_ms =
      (manager->frequency_hz_ > 0U) ? (1000U / manager->frequency_hz_) : 1U;
  if (period_ms == 0U) {
    period_ms = 1U;
  }

  MillisecondTimestamp last_wakeup(Thread::GetTime());
  while (manager->running_) {
    IMUArrayMsg msg;
    if (manager->ReadAll(msg) == ErrorCode::OK) {
      if (manager->data_topic_ != nullptr) {
        // 采到新数据后立刻发布，桥接层和其它订阅者只需监听 imu_data 即可。
        manager->data_topic_->Publish(msg);
      }
    }
    if (!manager->running_) {
      break;
    }
    Thread::SleepUntil(last_wakeup, period_ms);
  }
}

void IMUManager::ConvertIMUData(const Module::WitIMU::ImuData& src,
                                IMUData& dst) const {
  // 底层模块给出的是芯片原始量纲，这里统一转换为业务层固定使用的单位。
  dst.acc[0] = src.acc_x * 9.80665f;
  dst.acc[1] = src.acc_y * 9.80665f;
  dst.acc[2] = src.acc_z * 9.80665f;

  // JY901B 输出角速度单位为 deg/s，这里转成 rad/s 方便后续算法统一处理。
  constexpr float kDegToRad = 0.0174532925f;
  dst.gyro[0] = src.gyro_x * kDegToRad;
  dst.gyro[1] = src.gyro_y * kDegToRad;
  dst.gyro[2] = src.gyro_z * kDegToRad;

  dst.angle[0] = src.roll;
  dst.angle[1] = src.pitch;
  dst.angle[2] = src.yaw;

  dst.mag[0] = src.mag_x;
  dst.mag[1] = src.mag_y;
  dst.mag[2] = src.mag_z;
  dst.temperature = src.temperature;
  if (src.quat_valid) {
    dst.quaternion[0] = src.q0;
    dst.quaternion[1] = src.q1;
    dst.quaternion[2] = src.q2;
    dst.quaternion[3] = src.q3;
  } else {
    // 四元数未读到时保留 NaN，方便上层一眼看出该字段当前不可用。
    const float nan = std::numeric_limits<float>::quiet_NaN();
    dst.quaternion[0] = nan;
    dst.quaternion[1] = nan;
    dst.quaternion[2] = nan;
    dst.quaternion[3] = nan;
  }
  dst.status = 0;
}

}  // namespace Manager
