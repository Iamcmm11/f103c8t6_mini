#include "ws2812_manager.hpp"

namespace Manager {

using namespace LibXR;

WS2812Manager::WS2812Manager() : strip_(nullptr), spi_bus_(0) {}

ErrorCode WS2812Manager::Init(Module::WS2812Strip* strip, uint8_t spi_bus) {
  if (strip == nullptr) {
    return ErrorCode::ARG_ERR;
  }

  strip_ = strip;
  spi_bus_ = spi_bus;
  return ErrorCode::OK;
}

bool WS2812Manager::IsReady() const {
  return strip_ != nullptr && strip_->IsReady();
}

uint16_t WS2812Manager::GetMaxLedCount() const {
  return (strip_ != nullptr) ? strip_->GetMaxLedCount() : 0;
}

ErrorCode WS2812Manager::ShowFrame(uint8_t spi_bus, const uint8_t* rgb,
                                   uint16_t rgb_len, uint16_t led_count) {
  if (!IsReady() || rgb == nullptr || led_count == 0) {
    return ErrorCode::ARG_ERR;
  }
  if (spi_bus != spi_bus_) {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t expected_len = static_cast<uint32_t>(led_count) * 3U;
  if (rgb_len != expected_len) {
    return ErrorCode::ARG_ERR;
  }
  if (led_count > GetMaxLedCount()) {
    return ErrorCode::NO_MEM;
  }

  return strip_->ShowRGB(rgb, led_count);
}

}  // namespace Manager
