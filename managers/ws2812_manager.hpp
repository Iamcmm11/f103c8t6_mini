#pragma once

#include <cstdint>

#include "libxr.hpp"
#include "modules/ws2812/ws2812_strip.hpp"

namespace Manager {

class WS2812Manager {
 public:
  WS2812Manager();
  ~WS2812Manager() = default;

  LibXR::ErrorCode Init(Module::WS2812Strip* strip, uint8_t spi_bus);
  bool IsReady() const;
  uint8_t GetSPIBus() const { return spi_bus_; }
  uint16_t GetMaxLedCount() const;
  LibXR::ErrorCode ShowFrame(uint8_t spi_bus, const uint8_t* rgb,
                             uint16_t rgb_len, uint16_t led_count);

 private:
  Module::WS2812Strip* strip_;
  uint8_t spi_bus_;
};

}  // namespace Manager
