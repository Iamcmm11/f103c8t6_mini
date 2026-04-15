#pragma once

#include <cstddef>
#include <cstdint>

#include "spi.hpp"

namespace Module {

class WS2812Strip {
 public:
  static constexpr uint16_t kResetBytes = 64;

  explicit WS2812Strip(LibXR::SPI* spi = nullptr);

  void SetSPI(LibXR::SPI* spi);
  bool IsReady() const;
  uint16_t GetMaxLedCount() const;
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
