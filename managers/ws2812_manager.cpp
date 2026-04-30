#include "ws2812_manager.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace Manager {

using namespace LibXR;

namespace {

constexpr uint32_t kControlTickMs = 10;
constexpr uint32_t kControlStackBytes = 768;
constexpr uint32_t kTaskCreateOverheadBytes = 384;
constexpr uint32_t kTaskCreateSafetyBytes = 256;

bool TimeReached(uint32_t now_ms, uint32_t due_ms) {
  return static_cast<int32_t>(now_ms - due_ms) >= 0;
}

}  // namespace

WS2812Manager::WS2812Manager()
    : strip_(nullptr),
      spi_bus_(0),
      led_count_(0),
      running_(false),
      initialized_(false),
      dirty_(false) {}

ErrorCode WS2812Manager::Init(Module::WS2812Strip* strip, uint8_t spi_bus,
                              uint16_t led_count) {
  // manager 层至少要先拿到底层 strip 对象，后续语义灯控才有实际输出目标。
  if (strip == nullptr || led_count == 0U) {
    return ErrorCode::ARG_ERR;
  }
  if (led_count > kMaxManagedLedCount) {
    return ErrorCode::OUT_OF_RANGE;
  }
  if (led_count > strip->GetMaxLedCount()) {
    return ErrorCode::NO_MEM;
  }

  const size_t required_heap = kControlStackBytes + kTaskCreateOverheadBytes +
                               kTaskCreateSafetyBytes;
  if (!running_ && xPortGetFreeHeapSize() < required_heap) {
    return ErrorCode::NO_MEM;
  }

  Mutex::LockGuard guard(mutex_);
  strip_ = strip;
  spi_bus_ = spi_bus;
  led_count_ = led_count;
  lights_.fill(LightState{});
  frame_.fill(0);
  initialized_ = true;
  dirty_ = true;

  (void)FlushVisibleFrameLocked();

  if (!running_) {
    running_ = true;
    control_thread_.Create(this, ControlThreadFunc, "WS2812Ctl",
                           kControlStackBytes, Thread::Priority::LOW);
  }

  return ErrorCode::OK;
}

bool WS2812Manager::IsReady() const {
  // 同时要求 manager 已绑定 strip，且 strip 自身的 SPI 发送缓冲也已经可用。
  return initialized_ && strip_ != nullptr && strip_->IsReady();
}

uint16_t WS2812Manager::GetMaxLedCount() const {
  return (strip_ != nullptr) ? strip_->GetMaxLedCount() : 0;
}

ErrorCode WS2812Manager::SetLightControl(uint8_t target, uint8_t red,
                                         uint8_t green, uint8_t blue,
                                         bool blink_enable,
                                         uint16_t interval_ms) {
  if (!IsReady()) {
    return ErrorCode::INIT_ERR;
  }
  if (!IsTargetValid(target)) {
    return ErrorCode::ARG_ERR;
  }
  if (blink_enable && interval_ms < kMinBlinkIntervalMs) {
    return ErrorCode::ARG_ERR;
  }

  Mutex::LockGuard guard(mutex_);
  const uint32_t now_ms = Thread::GetTime();
  if (target == kAllLedsTarget) {
    for (uint16_t i = 0; i < led_count_; ++i) {
      ApplyLightState(static_cast<uint8_t>(i), red, green, blue, blink_enable,
                      interval_ms, now_ms);
    }
  } else {
    ApplyLightState(target, red, green, blue, blink_enable, interval_ms,
                    now_ms);
  }

  dirty_ = true;
  return FlushVisibleFrameLocked();
}

void WS2812Manager::ControlThreadFunc(WS2812Manager* manager) {
  if (manager != nullptr) {
    manager->RunControlThread();
  }
}

void WS2812Manager::RunControlThread() {
  while (running_) {
    Thread::Sleep(kControlTickMs);

    Mutex::LockGuard guard(mutex_);
    if (!IsReady()) {
      continue;
    }

    const uint32_t now_ms = Thread::GetTime();
    for (uint16_t i = 0; i < led_count_; ++i) {
      auto& light = lights_[i];
      if (!light.blink_enable) {
        continue;
      }
      if (TimeReached(now_ms, light.next_toggle_ms)) {
        light.visible_on = !light.visible_on;
        light.next_toggle_ms =
            now_ms + static_cast<uint32_t>(light.interval_ms);
        dirty_ = true;
      }
    }

    if (dirty_) {
      (void)FlushVisibleFrameLocked();
    }
  }
}

bool WS2812Manager::IsTargetValid(uint8_t target) const {
  return target == kAllLedsTarget || target < led_count_;
}

void WS2812Manager::ApplyLightState(uint8_t index, uint8_t red, uint8_t green,
                                    uint8_t blue, bool blink_enable,
                                    uint16_t interval_ms, uint32_t now_ms) {
  if (index >= led_count_) {
    return;
  }

  auto& light = lights_[index];
  light.red = red;
  light.green = green;
  light.blue = blue;
  light.blink_enable = blink_enable;
  light.interval_ms = blink_enable ? interval_ms : 0U;
  light.visible_on = true;
  light.next_toggle_ms = blink_enable
                             ? now_ms + static_cast<uint32_t>(interval_ms)
                             : 0U;
}

ErrorCode WS2812Manager::FlushVisibleFrameLocked() {
  if (!IsReady()) {
    return ErrorCode::INIT_ERR;
  }

  for (uint16_t i = 0; i < led_count_; ++i) {
    const auto& light = lights_[i];
    const uint16_t base = static_cast<uint16_t>(i * 3U);
    if (light.blink_enable && !light.visible_on) {
      frame_[base] = 0;
      frame_[base + 1U] = 0;
      frame_[base + 2U] = 0;
    } else {
      frame_[base] = light.red;
      frame_[base + 1U] = light.green;
      frame_[base + 2U] = light.blue;
    }
  }

  const ErrorCode ec = strip_->ShowRGB(frame_.data(), led_count_);
  if (ec == ErrorCode::OK) {
    dirty_ = false;
  }
  return ec;
}

}  // namespace Manager
