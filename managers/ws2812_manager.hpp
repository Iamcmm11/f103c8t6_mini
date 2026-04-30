#pragma once

#include <array>
#include <cstdint>

#include "libxr.hpp"
#include "modules/ws2812/ws2812_strip.hpp"
#include "mutex.hpp"
#include "thread.hpp"

namespace Manager {

/*
 * Manager: WS2812Manager
 * - 这一层不直接关心灯带编码细节，而是负责资源绑定、参数校验和总线编号管理。
 * - Module 层的 WS2812Strip 只负责“怎么发”，Manager 层负责“允不允许发、该走哪条总线”。
 */
class WS2812Manager {
 public:
  static constexpr uint8_t kAllLedsTarget = 0xFF;
  static constexpr uint16_t kMinBlinkIntervalMs = 20;
  // 当前 SPI TX buffer 为 768B；扣除 128B reset 后，最多容纳 21 颗 LED 的编码数据。
  static constexpr uint16_t kMaxManagedLedCount = 21;

  WS2812Manager();
  ~WS2812Manager() = default;

  // 绑定底层灯带驱动，并记录它对应的逻辑 SPI 总线编号和实际灯珠数量。
  LibXR::ErrorCode Init(Module::WS2812Strip* strip, uint8_t spi_bus,
                        uint16_t led_count);
  // 判断底层 strip 是否已经完成可用资源装配。
  bool IsReady() const;
  uint8_t GetSPIBus() const { return spi_bus_; }
  uint16_t GetLedCount() const { return led_count_; }
  // 查询当前发送缓冲最多还能驱动多少颗 LED。
  uint16_t GetMaxLedCount() const;
  // 接收语义化灯控命令：target 为 0xFF 表示全部，否则表示单颗灯索引。
  LibXR::ErrorCode SetLightControl(uint8_t target, uint8_t red, uint8_t green,
                                   uint8_t blue, bool blink_enable,
                                   uint16_t interval_ms);

 private:
  struct LightState {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    bool blink_enable = false;
    uint16_t interval_ms = 0;
    bool visible_on = true;
    uint32_t next_toggle_ms = 0;
  };

  static void ControlThreadFunc(WS2812Manager* manager);
  void RunControlThread();
  bool IsTargetValid(uint8_t target) const;
  void ApplyLightState(uint8_t index, uint8_t red, uint8_t green, uint8_t blue,
                       bool blink_enable, uint16_t interval_ms,
                       uint32_t now_ms);
  LibXR::ErrorCode FlushVisibleFrameLocked();

  Module::WS2812Strip* strip_;
  uint8_t spi_bus_;
  uint16_t led_count_;
  volatile bool running_;
  bool initialized_;
  bool dirty_;
  LibXR::Thread control_thread_;
  mutable LibXR::Mutex mutex_;
  std::array<LightState, kMaxManagedLedCount> lights_;
  std::array<uint8_t, kMaxManagedLedCount * 3U> frame_;
};

}  // namespace Manager
