#include "gpio_manager.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace Manager {

using namespace LibXR;

namespace {

constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;

}  // namespace

GPIOManager::GPIOManager()
    : command_callback_(nullptr),
      command_context_(nullptr),
      button_count_(0),
      scan_period_ms_(5),
      debounce_ms_(30),
      publish_retry_count_(3),
      publish_retry_delay_ms_(1),
      priority_(static_cast<uint32_t>(Thread::Priority::LOW)),
      stack_size_(512),
      initialized_(false),
      running_(false) {}

ErrorCode GPIOManager::Init(const GPIOManagerConfig& config) {
  if (running_) {
    return ErrorCode::BUSY;
  }
  if (config.command_callback == nullptr || config.commands == nullptr ||
      config.command_count == 0U) {
    return ErrorCode::ARG_ERR;
  }
  if (config.command_count > kMaxButtonCommands) {
    return ErrorCode::OUT_OF_RANGE;
  }
  if (config.scan_period_ms == 0U || config.debounce_ms == 0U ||
      config.publish_retry_count == 0U || config.stack_size == 0U) {
    return ErrorCode::ARG_ERR;
  }

  buttons_.fill(ButtonState{});
  for (size_t i = 0; i < config.command_count; ++i) {
    if (config.commands[i].gpio == nullptr ||
        config.commands[i].command == '\0') {
      return ErrorCode::ARG_ERR;
    }
    buttons_[i].gpio = config.commands[i].gpio;
    buttons_[i].command = config.commands[i].command;
  }

  command_callback_ = config.command_callback;
  command_context_ = config.command_context;
  button_count_ = config.command_count;
  scan_period_ms_ = config.scan_period_ms;
  debounce_ms_ = config.debounce_ms;
  publish_retry_count_ = config.publish_retry_count;
  publish_retry_delay_ms_ = config.publish_retry_delay_ms;
  priority_ = config.priority;
  stack_size_ = config.stack_size;
  initialized_ = true;
  return ErrorCode::OK;
}

ErrorCode GPIOManager::Start() {
  if (!initialized_) {
    return ErrorCode::INIT_ERR;
  }
  if (running_) {
    return ErrorCode::BUSY;
  }

  const size_t required_heap = static_cast<size_t>(stack_size_) +
                               kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  InitializeButtonStates(Thread::GetTime());
  running_ = true;
  button_thread_.Create(this, ButtonThreadFunc, "GPIOButton", stack_size_,
                        static_cast<Thread::Priority>(priority_));
  return ErrorCode::OK;
}

void GPIOManager::Stop() { running_ = false; }

GPIOManagerDiagStats GPIOManager::GetDiagStats() const {
  GPIOManagerDiagStats stats;
  stats.edge_detect_count = edge_detect_count_;
  stats.publish_attempt_count = publish_attempt_count_;
  stats.publish_ok_count = publish_ok_count_;
  stats.publish_fail_count = publish_fail_count_;
  stats.last_command = last_command_;
  return stats;
}

void GPIOManager::ButtonThreadFunc(GPIOManager* manager) {
  if (manager != nullptr) {
    manager->RunButtonThread();
  }
}

void GPIOManager::RunButtonThread() {
  while (running_) {
    const uint32_t now_ms = Thread::GetTime();
    for (size_t i = 0; i < button_count_; ++i) {
      PollButton(buttons_[i], now_ms);
    }
    Thread::Sleep(scan_period_ms_);
  }
}

void GPIOManager::InitializeButtonStates(uint32_t now_ms) {
  for (size_t i = 0; i < button_count_; ++i) {
    auto& button = buttons_[i];
    const bool level = button.gpio->Read();
    button.idle_level = level;
    button.raw_level = level;
    button.stable_level = level;
    button.raw_change_ms = now_ms;
    button.sent_for_active_level = false;
  }
}

void GPIOManager::PollButton(ButtonState& button, uint32_t now_ms) {
  const bool raw_level = button.gpio->Read();
  if (raw_level != button.raw_level) {
    button.raw_level = raw_level;
    button.raw_change_ms = now_ms;
    return;
  }

  if (raw_level == button.stable_level ||
      (now_ms - button.raw_change_ms) < debounce_ms_) {
    return;
  }

  button.stable_level = raw_level;
  const bool active = button.stable_level != button.idle_level;
  if (active && !button.sent_for_active_level) {
    ++edge_detect_count_;
    last_command_ = button.command;
    (void)PublishCommand(button.command);
    button.sent_for_active_level = true;
  } else if (!active) {
    button.sent_for_active_level = false;
  }
}

bool GPIOManager::PublishCommand(char command) {
  ++publish_attempt_count_;
  for (uint32_t attempt = 0; attempt < publish_retry_count_; ++attempt) {
    if (command_callback_(command_context_, command)) {
      ++publish_ok_count_;
      return true;
    }
    Thread::Sleep(publish_retry_delay_ms_);
  }

  ++publish_fail_count_;
  return false;
}

}  // namespace Manager
