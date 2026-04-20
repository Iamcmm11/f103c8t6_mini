#pragma once

#include <cstddef>
#include <cstdint>

#include "spi.hpp"

namespace Module {

/*
 * Module: WS2812Strip
 * - 负责把 RGB 数据编码成 WS2812 所需的时序位流，并借助 SPI 发出去。
 * - 这一层不关心“逻辑总线编号”和上层业务语义，只关心编码和发送本身。
 */
class WS2812Strip {
 public:
  // 帧首和帧尾都要补一段低电平复位时间，这里对应编码后的 reset 区长度。
  static constexpr uint16_t kResetBytes = 64;

  explicit WS2812Strip(LibXR::SPI* spi = nullptr);

  // 动态切换底层 SPI 设备，方便未来做资源重绑定。
  void SetSPI(LibXR::SPI* spi);
  // 检查 SPI 发送缓冲是否足以容纳至少一颗灯的编码结果。
  bool IsReady() const;
  // 根据发送缓冲大小反推当前最多可以驱动多少颗灯。
  uint16_t GetMaxLedCount() const;
  // 接收标准 RGB 帧并触发编码 + 发送流程。
  LibXR::ErrorCode ShowRGB(const uint8_t* rgb, uint16_t led_count);

 private:
  static constexpr uint8_t kEncodedBytesPerColorByte = 10;
  static constexpr uint8_t kEncodedBytesPerLed = 30;
  static constexpr uint16_t kTotalResetBytes = kResetBytes * 2U;
  static constexpr uint16_t kWsBit0Pattern = 0x0380;
  static constexpr uint16_t kWsBit1Pattern = 0x03F8;

  void EncodeByte(uint8_t byte, uint8_t* out) const;
  LibXR::ErrorCode EncodeFrameInPlace(const uint8_t* rgb,
                                      uint16_t led_count) const;
  LibXR::ErrorCode WriteEncoded(uint16_t encoded_len) const;

  LibXR::SPI* spi_;
};

}  // namespace Module
