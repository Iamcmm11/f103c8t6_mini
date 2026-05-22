#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "tim.h"
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
#include "application/yis_imu_acquisition_task.hpp"
#include "managers/imu_manager.hpp"
#include "managers/ws2812_manager.hpp"
#include "modules/yesense_yis_imu/yis_imu.hpp"
#include "modules/ws2812/ws2812_strip.hpp"

extern UART_HandleTypeDef huart5;

extern "C" void app_on_tim6_period_elapsed(void) {
  Manager::IMUManager::OnHardwareTriggerTimerInterrupt(true);
}

namespace {

constexpr bool kEnableUart5LogPush = false;

void Uart5Print(const char* text) {
  if (!kEnableUart5LogPush) {
    (void)text;
    return;
  }

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

uint32_t GetTim5ClockHz() {
  uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
#ifdef RCC_CFGR_PPRE1
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    clock_hz *= 2U;
  }
#endif
  return clock_hz;
}

}  // namespace
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

/* DMA Resources */
static uint8_t spi1_tx_buf[768];
static uint8_t usart1_tx_buf[512];
static uint8_t usart1_rx_buf[256];
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
  STM32GPIO PA6(GPIOA, GPIO_PIN_6, EXTI9_5_IRQn);


  STM32PWM pwm_tim5_ch1(&htim5, TIM_CHANNEL_1, false);

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

  const auto pwm_sync_ec = pwm_tim5_ch1.Enable();
  const uint32_t tim5_clk_hz = GetTim5ClockHz();
  const uint32_t tim5_psc = static_cast<uint32_t>(htim5.Init.Prescaler) + 1U;
  const uint32_t tim5_arr = static_cast<uint32_t>(htim5.Init.Period) + 1U;
  const uint32_t tim5_ccr = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
  const uint32_t sync_freq_hz =
      (tim5_psc != 0U && tim5_arr != 0U) ? (tim5_clk_hz / tim5_psc / tim5_arr)
                                         : 0U;

  std::snprintf(
      line, sizeof(line),
      "[boot] sync pwm ec=%d clk=%lu psc=%lu arr=%lu ccr=%lu freq=%lu",
      static_cast<int>(pwm_sync_ec), static_cast<unsigned long>(tim5_clk_hz),
      static_cast<unsigned long>(htim5.Init.Prescaler),
      static_cast<unsigned long>(htim5.Init.Period),
      static_cast<unsigned long>(tim5_ccr),
      static_cast<unsigned long>(sync_freq_hz));
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
  wit_acq_config.use_hardware_trigger = true;
  // 1 => only 0x50, 2 => 0x50+0x51, 3 => 0x50+0x51+0x52, 4 => 0x50~0x53.
  wit_acq_config.enabled_imu_count = 1;

  // YIS acquisition config: 50Hz, topic "yis_imu_pose",
  // medium priority, 1024-byte stack.
  ::Application::YISIMUAcquisitionConfig yis_config;
  yis_config.frequency_hz = 200;
  yis_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  yis_config.stack_size = 1024;
  yis_config.dr_wait_timeout_ms = 20;
  yis_config.topic_name = "yis_imu_pose";
  static ::Application::YISIMUAcquisitionTask yis_task(&yis_imu, yis_config, &PA6);

  // Bridge config: 20ms push period, medium priority, 1024-byte stack.
  // Select bridge pose source here: WIT or YIS.
  ::Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;
  bridge_config.push_imu_euler_in_bridge = true;
  bridge_config.pose_source = ::Application::BridgePoseSource::WIT;
  bridge_config.push_all_slots_in_bridge = false;
  bridge_config.stream_interval_ms = 20;
  bridge_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  bridge_config.stack_size = 1024;

  // ========================================================================
  // Start Application threads  启动任务线程
  // ========================================================================

  // 启动901B采集任务
  ErrorCode imu_acq_ec = ErrorCode::INIT_ERR;
  if (imu_init_ec == ErrorCode::OK) {
    (void)HAL_TIM_Base_Start_IT(&htim6);
    imu_acq_ec = imu_manager.StartAcquisition(wit_acq_config);
  }

  // 启动YIS采集任务
  LibXR::ErrorCode yis_start_ec = LibXR::ErrorCode::INIT_ERR;
  if (yis_init_ec == ErrorCode::OK) {
    yis_start_ec = yis_task.Start();
  }


  // 启动串口桥收发任务
  static ::Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
  const auto bridge_start_ec = imu_bridge.Start();
  (void)bridge_start_ec;

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
