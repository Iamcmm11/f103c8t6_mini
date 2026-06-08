#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "thread.hpp"

namespace Manager {

using GPIOButtonCommandCallback = bool (*)(void* context, char command);

struct GPIOButtonCommand {
  LibXR::GPIO* gpio = nullptr;
  char command = '\0';
};

struct GPIOManagerConfig {
  GPIOButtonCommandCallback command_callback = nullptr;
  void* command_context = nullptr;
  const GPIOButtonCommand* commands = nullptr;
  size_t command_count = 0;
  uint32_t scan_period_ms = 5;
  uint32_t debounce_ms = 30;
  uint32_t publish_retry_count = 3;
  uint32_t publish_retry_delay_ms = 1;
  uint32_t priority = static_cast<uint32_t>(LibXR::Thread::Priority::LOW);
  uint32_t stack_size = 512;
};

struct GPIOManagerDiagStats {
  uint32_t edge_detect_count = 0;
  uint32_t publish_attempt_count = 0;
  uint32_t publish_ok_count = 0;
  uint32_t publish_fail_count = 0;
  char last_command = '\0';
};

class GPIOManager {
 public:
  static constexpr size_t kMaxButtonCommands = 8;

  GPIOManager();
  ~GPIOManager() = default;

  LibXR::ErrorCode Init(const GPIOManagerConfig& config);
  LibXR::ErrorCode Start();
  void Stop();
  bool IsRunning() const { return running_; }
  GPIOManagerDiagStats GetDiagStats() const;

 private:
  struct ButtonState {
    LibXR::GPIO* gpio = nullptr;
    char command = '\0';
    bool idle_level = false;
    bool raw_level = false;
    bool stable_level = false;
    uint32_t raw_change_ms = 0;
    bool sent_for_active_level = false;
  };

  static void ButtonThreadFunc(GPIOManager* manager);
  void RunButtonThread();
  void InitializeButtonStates(uint32_t now_ms);
  void PollButton(ButtonState& button, uint32_t now_ms);
  bool PublishCommand(char command);

  GPIOButtonCommandCallback command_callback_;
  void* command_context_;
  std::array<ButtonState, kMaxButtonCommands> buttons_;
  size_t button_count_;
  uint32_t scan_period_ms_;
  uint32_t debounce_ms_;
  uint32_t publish_retry_count_;
  uint32_t publish_retry_delay_ms_;
  uint32_t priority_;
  uint32_t stack_size_;
  bool initialized_;
  volatile bool running_;
  volatile uint32_t edge_detect_count_ = 0;
  volatile uint32_t publish_attempt_count_ = 0;
  volatile uint32_t publish_ok_count_ = 0;
  volatile uint32_t publish_fail_count_ = 0;
  volatile char last_command_ = '\0';
  LibXR::Thread button_thread_;
};

}  // namespace Manager
