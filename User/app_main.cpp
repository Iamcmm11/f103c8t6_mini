#include "app_main.h"

#include "libxr.hpp"
#include "main.h"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"

using namespace LibXR;

/* User Code Begin 1 */
#include "application/imu_uart_bridge_task.hpp"
#include "managers/imu_manager.hpp"
#include "managers/ws2812_manager.hpp"
#include "modules/ws2812/ws2812_strip.hpp"
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

/* DMA Resources */
static uint8_t spi1_tx_buf[768];
static uint8_t usart1_tx_buf[512];
static uint8_t usart1_rx_buf[128];
static uint8_t i2c1_buf[96];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32TimerTimebase timebase(&htim1);
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */



  STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, UINT32_MAX);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  /* Terminal Configuration */
  // STDIO::write_ = usart1.write_port_;

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  static Module::WS2812Strip ws2812_strip(&spi1);
  static Manager::WS2812Manager ws2812_manager;
  (void)ws2812_manager.Init(&ws2812_strip, 0);

  static Manager::IMUManager imu_manager(Manager::ACTUAL_IMU_COUNT);
  (void)imu_manager.Init(&i2c1, Manager::kDefaultImuAddress);
  (void)imu_manager.StartAcquisition(50, "imu_data");

  static Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;
  bridge_config.push_imu_euler_in_bridge = true;
  bridge_config.stream_interval_ms = 20;
  bridge_config.stack_size = 1000;
  static Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
  (void)imu_bridge.Start();

  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}
