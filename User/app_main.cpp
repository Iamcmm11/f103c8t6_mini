#include "app_main.h"

#include "cdc_uart.hpp"
#include "flash_map.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"

using namespace LibXR;

/* User Code Begin 1 */
#include <cstdio>
#include <cstring>

#include "application/imu_uart_bridge_task.hpp"
#include "application/yis_imu_acquisition_task.hpp"
#include "managers/imu_manager.hpp"
#include "managers/ws2812_manager.hpp"
#include "modules/yesense_yis_imu/yis_imu.hpp"
#include "modules/ws2812/ws2812_strip.hpp"

extern UART_HandleTypeDef huart5;

namespace {

void Uart5Print(const char* text) {
  if (text == nullptr) {
    return;
  }

  const auto len = static_cast<uint16_t>(std::strlen(text));
  (void)HAL_UART_Transmit(&huart5, reinterpret_cast<const uint8_t*>(text), len,
                          100);
}

void Uart5PrintLine(const char* text) {
  Uart5Print(text);
  Uart5Print("\r\n");
}

}  // namespace
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

/* DMA Resources */
static uint8_t spi1_tx_buf[768];
static uint8_t usart1_tx_buf[512];
static uint8_t usart1_rx_buf[128];
static uint8_t i2c1_buf[96];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  Uart5PrintLine("[boot] app_main");
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32TimerTimebase timebase(&htim1);
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO PA4(GPIOA, GPIO_PIN_4);

  STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);

  // STM32UART uart5(&huart5,
  //             {nullptr, 0}, {nullptr, 0}, 5);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  char line[128] = {0};

  // ========================================================================
  // Create Manager / Module instances  创建设备管理
  // ========================================================================

  constexpr uint8_t yis_i2c_addr = 0x6A;
  static ::Module::WS2812Strip ws2812_strip(&spi1);
  static ::Manager::WS2812Manager ws2812_manager;
  static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);
  static ::Module::YISIMU yis_imu(&i2c1, yis_i2c_addr);

  // ========================================================================
  // Initialize Manager / Module  设备管理初始化
  // ========================================================================

  constexpr uint16_t ws2812_led_count = 16;
  const auto ws2812_ec =
      ws2812_manager.Init(&ws2812_strip, 0, ws2812_led_count);

  std::snprintf(line, sizeof(line), "[boot] ws2812 ec=%d leds=%u",
                static_cast<int>(ws2812_ec),
                static_cast<unsigned>(ws2812_led_count));
  Uart5PrintLine(line);

  const auto imu_init_ec =
      imu_manager.Init(&i2c1, ::Manager::kDefaultImuAddress);

  std::snprintf(line, sizeof(line), "[boot] wit init=%d online=%u",
                static_cast<int>(imu_init_ec),
                static_cast<unsigned>(imu_manager.GetOnlineCount()));
  Uart5PrintLine(line);

  const auto yis_init_ec = yis_imu.Init();

  std::snprintf(line, sizeof(line), "[boot] yis init=%d addr=0x%02X hal=0x%02X",
                static_cast<int>(yis_init_ec),
                static_cast<unsigned>(yis_imu.address()),
                static_cast<unsigned>(yis_imu.slave_address()));
  Uart5PrintLine(line);

  // ========================================================================
  // Create Application task instances (dedicated threads)  创建任务线程
  // ========================================================================

  // WIT acquisition config: 50Hz, topic "imu_data", high priority,
  // 2048-byte stack.
  ::Manager::IMUManagerAcquisitionConfig wit_acq_config;
  wit_acq_config.frequency_hz = 50;
  wit_acq_config.topic_name = "imu_data";
  wit_acq_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  wit_acq_config.stack_size = 2048;

  // YIS acquisition config: 50Hz, topic "yis_imu_quat",
  // medium priority, 1024-byte stack, log every 100 samples.
  ::Application::YISIMUAcquisitionConfig yis_config;
  yis_config.frequency_hz = 50;
  yis_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  yis_config.stack_size = 1024;
  yis_config.log_interval = 100;
  yis_config.topic_name = "yis_imu_quat";
  yis_config.log_writer = Uart5PrintLine;
  static ::Application::YISIMUAcquisitionTask yis_task(&yis_imu, yis_config);

  // Bridge config: 20ms push period, medium priority, 1024-byte stack,
  // absolute Euler output, push enabled only when WIT acquisition starts.
  ::Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;
  bridge_config.push_imu_euler_in_bridge = false;
  bridge_config.stream_interval_ms = 20;
  bridge_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  bridge_config.stack_size = 1024;

  // ========================================================================
  // Start Application threads  启动任务线程
  // ========================================================================

  // 启动901B采集任务
  ErrorCode imu_acq_ec = ErrorCode::INIT_ERR;
  if (imu_init_ec == ErrorCode::OK) {
    imu_acq_ec = imu_manager.StartAcquisition(wit_acq_config);
  }
  bridge_config.push_imu_euler_in_bridge = (imu_acq_ec == ErrorCode::OK);

  // 启动YIS采集任务
  LibXR::ErrorCode yis_start_ec = LibXR::ErrorCode::INIT_ERR;
  if (yis_init_ec == ErrorCode::OK) {
    yis_start_ec = yis_task.Start();
  }

  // 启动串口桥收发任务
  static ::Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
  const auto bridge_start_ec = imu_bridge.Start();

  std::snprintf(line, sizeof(line), "[boot] wit start=%d freq=%lu stack=%lu",
                static_cast<int>(imu_acq_ec),
                static_cast<unsigned long>(wit_acq_config.frequency_hz),
                static_cast<unsigned long>(wit_acq_config.stack_size));
  Uart5PrintLine(line);

  std::snprintf(line, sizeof(line), "[boot] yis start=%d freq=%lu stack=%lu",
                static_cast<int>(yis_start_ec),
                static_cast<unsigned long>(yis_config.frequency_hz),
                static_cast<unsigned long>(yis_config.stack_size));
  Uart5PrintLine(line);

  std::snprintf(line, sizeof(line), "[boot] bridge start=%d push=%u period=%lu",
                static_cast<int>(bridge_start_ec),
                static_cast<unsigned>(bridge_config.push_imu_euler_in_bridge),
                static_cast<unsigned long>(bridge_config.stream_interval_ms));
  Uart5PrintLine(line);

  // ========================================================================
  // Idle loop
  // ========================================================================

  while (true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}
