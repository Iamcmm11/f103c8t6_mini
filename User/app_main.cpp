#include "app_main.h"

#include "cdc_uart.hpp"
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
#include "flash_map.hpp"

using namespace LibXR;

/* User Code Begin 1 */
#include <cstdio>
#include <cstring>

#include "application/imu_uart_bridge_task.hpp"
#include "managers/imu_manager.hpp"
#include "managers/ws2812_manager.hpp"
#include "modules/ws2812/ws2812_strip.hpp"

extern UART_HandleTypeDef huart5;

namespace {

constexpr bool kEnableUart5PlaintextDiag = false;
constexpr bool kEnableUart5BootLog = true;

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
  if (kEnableUart5PlaintextDiag) {
    Uart5PrintLine("");
    Uart5PrintLine("[diag] app_main entered");
    Uart5PrintLine("[diag] UART5 plaintext diagnostic mode enabled");
    Uart5PrintLine("[diag] Expect 115200 8N1 on UART5");
  } else if (kEnableUart5BootLog) {
    Uart5PrintLine("");
    Uart5PrintLine("[boot] app_main entered");
    Uart5PrintLine("[boot] USART1 bridge mode enabled");
    Uart5PrintLine("[boot] UART5 blocking log enabled");
  }
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32TimerTimebase timebase(&htim1);
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO PA4(GPIOA, GPIO_PIN_4);



  STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);

  STM32UART uart5(&huart5,
              {nullptr, 0}, {nullptr, 0}, 5);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  /* Terminal Configuration */
  STDIO::read_ = usart1.read_port_;
  STDIO::write_ = usart1.write_port_;

  RamFS ramfs("XRobot");
  Terminal<32, 32, 5, 5> terminal(ramfs);
  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  static ::Module::WS2812Strip ws2812_strip(&spi1);
  static ::Manager::WS2812Manager ws2812_manager;
  const auto ws2812_ec = ws2812_manager.Init(&ws2812_strip, 0);

  static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);
  const auto imu_init_ec =
      imu_manager.Init(&i2c1, ::Manager::kDefaultImuAddress);
  const auto imu_acq_ec = imu_manager.StartAcquisition(50, "imu_data");

  if (kEnableUart5BootLog) {
    char line[128] = {0};
    std::snprintf(line, sizeof(line),
                  "[boot] ws2812=%d imu_init=%d imu_start=%d online=%u",
                  static_cast<int>(ws2812_ec), static_cast<int>(imu_init_ec),
                  static_cast<int>(imu_acq_ec),
                  static_cast<unsigned>(imu_manager.GetOnlineCount()));
    Uart5PrintLine(line);
  }

  if (kEnableUart5PlaintextDiag) {
    char line[128] = {0};
    std::snprintf(line, sizeof(line),
                  "[diag] ws2812_init=%d imu_init=%d imu_start=%d online=%u",
                  static_cast<int>(ws2812_ec), static_cast<int>(imu_init_ec),
                  static_cast<int>(imu_acq_ec),
                  static_cast<unsigned>(imu_manager.GetOnlineCount()));
    Uart5PrintLine(line);
    Uart5PrintLine("[diag] bridge disabled in plaintext diagnostic mode");

    uint32_t heartbeat = 0;
    while (true) {
      std::snprintf(line, sizeof(line),
                    "[diag] heartbeat=%lu online=%u freq=%lu",
                    static_cast<unsigned long>(heartbeat++),
                    static_cast<unsigned>(imu_manager.GetOnlineCount()),
                    static_cast<unsigned long>(imu_manager.GetFrequency()));
      Uart5PrintLine(line);
      Thread::Sleep(1000);
    }
  }

  static ::Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;
  bridge_config.push_imu_euler_in_bridge = true;
  bridge_config.stream_interval_ms = 20;
  bridge_config.stack_size = 1000;
  static ::Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
  (void)imu_bridge.Start();

  if (kEnableUart5BootLog) {
    Uart5PrintLine("[boot] USART1 bridge ready");
  }

  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}