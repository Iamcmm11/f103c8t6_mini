#pragma once

#include <cstdint>

#include "libxr.hpp"
#include "modules/ws2812/ws2812_strip.hpp"

namespace Manager {

/*
 * Manager: WS2812Manager
 * - 这一层不直接关心灯带编码细节，而是负责资源绑定、参数校验和总线编号管理。
 * - Module 层的 WS2812Strip 只负责“怎么发”，Manager 层负责“允不允许发、该走哪条总线”。
 */
class WS2812Manager {
 public:
  WS2812Manager();
  ~WS2812Manager() = default;

  // 绑定底层灯带驱动，并记录它对应的逻辑 SPI 总线编号。
  LibXR::ErrorCode Init(Module::WS2812Strip* strip, uint8_t spi_bus);
  // 判断底层 strip 是否已经完成可用资源装配。
  bool IsReady() const;
  uint8_t GetSPIBus() const { return spi_bus_; }
  // 查询当前发送缓冲最多还能驱动多少颗 LED。
  uint16_t GetMaxLedCount() const;
  // 对桥接层传来的整帧 RGB 数据做参数检查后，转交给 Module 层输出。
  LibXR::ErrorCode ShowFrame(uint8_t spi_bus, const uint8_t* rgb,
                             uint16_t rgb_len, uint16_t led_count);

 private:
  Module::WS2812Strip* strip_;
  uint8_t spi_bus_;
};

}  // namespace Manager
