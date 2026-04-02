#include "imu_acquisition_task.hpp"

namespace Application {

using namespace LibXR;
using namespace Manager;

IMUAcquisitionTask::IMUAcquisitionTask(IMUManager* imu_mgr,
                                       const IMUAcquisitionConfig& config)
    : imu_mgr_(imu_mgr),
      config_(config),
      running_(false),
      count_(0),
      thread_(nullptr),
      imu_topic_("imu_data", sizeof(IMUArrayMsg), nullptr, false, true, true) {}

ErrorCode IMUAcquisitionTask::Start() {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (imu_mgr_ == nullptr) {
    return ErrorCode::PTR_NULL;
  }

  running_ = true;
  thread_ = new Thread();
  if (thread_ == nullptr) {
    running_ = false;
    return ErrorCode::NO_MEM;
  }

  // Thread::Create expects stack size in bytes.
  thread_->Create(this, TaskEntry, "IMUAcq", config_.stack_size,
                  static_cast<Thread::Priority>(config_.priority));
  return ErrorCode::OK;
}

void IMUAcquisitionTask::Stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  Thread::Sleep(config_.period_ms > 0 ? config_.period_ms : 1);
}

void IMUAcquisitionTask::TaskEntry(IMUAcquisitionTask* arg) {
  if (arg != nullptr) {
    arg->Run();
  }
}

void IMUAcquisitionTask::Run() {
  const uint32_t period = config_.period_ms > 0 ? config_.period_ms : 1;
  MillisecondTimestamp last_wakeup(Thread::GetTime());

  while (running_) {
    ProcessIMUData();
    if (!running_) {
      break;
    }
    Thread::SleepUntil(last_wakeup, period);
  }
}

void IMUAcquisitionTask::ProcessIMUData() {
  IMUArrayMsg imu_msg;
  if (imu_mgr_->ReadAll(imu_msg) == ErrorCode::OK) {
    imu_topic_.Publish(imu_msg);
    ++count_;
  }
}

}  // namespace Application
