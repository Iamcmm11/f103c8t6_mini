#include "ws2812_manager.hpp"

namespace Manager {

using namespace LibXR;

WS2812Manager::WS2812Manager() : strip_(nullptr), spi_bus_(0) {}

ErrorCode WS2812Manager::Init(Module::WS2812Strip* strip, uint8_t spi_bus) {
  // manager 层至少要先拿到底层 strip 对象，后续 ShowFrame 才有实际输出目标。
  if (strip == nullptr) {
    return ErrorCode::ARG_ERR;
  }

  strip_ = strip;
  spi_bus_ = spi_bus;
  return ErrorCode::OK;
}

bool WS2812Manager::IsReady() const {
  // 同时要求 manager 已绑定 strip，且 strip 自身的 SPI 发送缓冲也已经可用。
  return strip_ != nullptr && strip_->IsReady();
}

uint16_t WS2812Manager::GetMaxLedCount() const {
  return (strip_ != nullptr) ? strip_->GetMaxLedCount() : 0;
}

ErrorCode WS2812Manager::ShowFrame(uint8_t spi_bus, const uint8_t* rgb,
                                   uint16_t rgb_len, uint16_t led_count) {
  // 先做管理层参数校验，避免把非法请求直接下放到底层驱动。
  if (!IsReady() || rgb == nullptr || led_count == 0) {
    return ErrorCode::ARG_ERR;
  }
  if (spi_bus != spi_bus_) {
    return ErrorCode::ARG_ERR;
  }

  // WS2812 每颗灯固定使用 3 字节 RGB 数据，所以长度必须与灯珠数量严格匹配。
  const uint32_t expected_len = static_cast<uint32_t>(led_count) * 3U;
  if (rgb_len != expected_len) {
    return ErrorCode::ARG_ERR;
  }
  if (led_count > GetMaxLedCount()) {
    return ErrorCode::NO_MEM;
  }

  // 真正的编码与发送交给 Module 层处理。
  return strip_->ShowRGB(rgb, led_count);
  // return ErrorCode::OK;
}

}  // namespace Manager
