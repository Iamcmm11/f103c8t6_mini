#include "ws2812_strip.hpp"

#include <cstring>

#include "semaphore.hpp"

namespace Module {

using namespace LibXR;

WS2812Strip::WS2812Strip(SPI* spi) : spi_(spi) {}

void WS2812Strip::SetSPI(SPI* spi) { spi_ = spi; }

bool WS2812Strip::IsReady() const {
  const auto tx =
      (spi_ != nullptr) ? spi_->GetTxBuffer() : RawData(nullptr, 0);
  // 至少要能容纳 reset 区和一颗 LED 的编码数据，灯带驱动才算可用。
  return (tx.addr_ != nullptr) &&
         (tx.size_ >= kTotalResetBytes + kEncodedBytesPerLed);
}

uint16_t WS2812Strip::GetMaxLedCount() const {
  if (spi_ == nullptr) {
    return 0;
  }

  const auto tx = spi_->GetTxBuffer();
  if (tx.addr_ == nullptr || tx.size_ <= kTotalResetBytes) {
    return 0;
  }

  return static_cast<uint16_t>((tx.size_ - kTotalResetBytes) /
                               kEncodedBytesPerLed);
}

ErrorCode WS2812Strip::ShowRGB(const uint8_t* rgb, uint16_t led_count) {
  if (spi_ == nullptr || rgb == nullptr || led_count == 0) {
    return ErrorCode::ARG_ERR;
  }

  // 先在发送缓冲里原地编码，再把整帧一次性通过 SPI 发出去。
  const ErrorCode encode_ec = EncodeFrameInPlace(rgb, led_count);
  if (encode_ec != ErrorCode::OK) {
    return encode_ec;
  }

  const uint16_t encoded_len =
      static_cast<uint16_t>(led_count * kEncodedBytesPerLed + kTotalResetBytes);
  return WriteEncoded(encoded_len);
  // return ErrorCode::OK;
}

void WS2812Strip::EncodeByte(uint8_t byte, uint8_t* out) const {
  std::memset(out, 0, kEncodedBytesPerColorByte);

  // WS2812 的一个颜色字节会被展开成固定长度的高低电平模式，这里逐位做映射。
  uint8_t bit_pos = 0;
  for (int src_bit = 7; src_bit >= 0; --src_bit) {
    const uint16_t pattern =
        ((byte & (1 << src_bit)) != 0) ? kWsBit1Pattern : kWsBit0Pattern;
    for (int pat_bit = 9; pat_bit >= 0; --pat_bit) {
      if ((pattern & (1U << pat_bit)) != 0U) {
        const uint8_t byte_index = static_cast<uint8_t>(bit_pos >> 3);
        const uint8_t bit_index = static_cast<uint8_t>(7U - (bit_pos & 0x07U));
        out[byte_index] =
            static_cast<uint8_t>(out[byte_index] | (1U << bit_index));
      }
      ++bit_pos;
    }
  }
}

ErrorCode WS2812Strip::EncodeFrameInPlace(const uint8_t* rgb,
                                          uint16_t led_count) const {
  if (spi_ == nullptr || rgb == nullptr) {
    return ErrorCode::ARG_ERR;
  }

  const auto tx = spi_->GetTxBuffer();
  auto* tx_ptr = reinterpret_cast<uint8_t*>(tx.addr_);
  const uint32_t encoded_len =
      static_cast<uint32_t>(led_count) * kEncodedBytesPerLed + kTotalResetBytes;
  if (tx_ptr == nullptr || encoded_len > tx.size_) {
    return ErrorCode::NO_MEM;
  }

  // 从后往前编码可以避免当输入和输出缓冲可能靠近时发生覆盖问题。
  for (int32_t led = static_cast<int32_t>(led_count) - 1; led >= 0; --led) {
    const uint8_t* src = rgb + led * 3;
    uint8_t* dst = tx_ptr + kResetBytes + led * kEncodedBytesPerLed;
    const uint8_t red = src[0];
    const uint8_t green = src[1];
    const uint8_t blue = src[2];
    EncodeByte(green, dst);
    EncodeByte(red, dst + kEncodedBytesPerColorByte);
    EncodeByte(blue, dst + kEncodedBytesPerColorByte * 2U);
  }

  // WS2812 要求帧前和帧后都保留一段低电平复位区，这里统一清零处理。
  std::memset(tx_ptr, 0, kResetBytes);
  const uint16_t used =
      static_cast<uint16_t>(kResetBytes + led_count * kEncodedBytesPerLed);
  if (used < encoded_len) {
    std::memset(tx_ptr + used, 0, encoded_len - used);
  }

  return ErrorCode::OK;
}

ErrorCode WS2812Strip::WriteEncoded(uint16_t encoded_len) const {
  if (spi_ == nullptr) {
    return ErrorCode::ARG_ERR;
  }

  const auto tx = spi_->GetTxBuffer();
  if (tx.addr_ == nullptr || encoded_len > tx.size_) {
    return ErrorCode::NO_MEM;
  }

  Semaphore sem(0);
  SPI::OperationRW op(sem,20);
  // 对调用方而言这是一次“同步完成”的发送，但底层 SPI 往往会用 DMA/中断完成搬运。
  return spi_->Write({tx.addr_, encoded_len}, op);
  // return ErrorCode::OK;
}

}  // namespace Module
