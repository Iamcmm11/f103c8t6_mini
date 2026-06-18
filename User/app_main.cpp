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
#include "application/yis_imu_acquisition_task.hpp"
#include "managers/feyman_manager.hpp"
#include "managers/gpio_manager.hpp"
#include "managers/imu_manager.hpp"
#include "managers/sync_signal_manager.hpp"
#include "managers/ws2812_manager.hpp"
#include "modules/ws2812/ws2812_strip.hpp"
#include "modules/yesense_yis_imu/yis_imu.hpp"

extern UART_HandleTypeDef huart5;

extern "C" void app_on_tim2_period_elapsed(void) {
  Manager::SyncSignalManager::RecordEventFromISR(
      Manager::SyncEventSource::TIM2_IMU_SYNC_1HZ,
      LibXR::Timebase::GetMicroseconds());
}

extern "C" void app_on_tim5_period_elapsed(void) {
  Manager::SyncSignalManager::RecordEventFromISR(
      Manager::SyncEventSource::TIM5_CAMERA_TRIGGER_30HZ,
      LibXR::Timebase::GetMicroseconds());
}

extern "C" void app_on_tim6_period_elapsed(void) {
  Manager::IMUManager::OnHardwareTriggerTimerInterrupt(true);
}

namespace {

constexpr bool kEnableUart5LogPush = true;

void Uart5Print(const char* text) {
  if (!kEnableUart5LogPush || text == nullptr) {
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

void EnableTimUpdateInterrupt(void* context) {
  auto* timer = static_cast<TIM_HandleTypeDef*>(context);
  if (timer == nullptr) {
    return;
  }

  __HAL_TIM_DISABLE_IT(timer, TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(timer, 0U);
  __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
}

}  // namespace
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

/* DMA Resources */
static uint8_t spi1_tx_buf[768];
static uint8_t usart1_tx_buf[2048];
static uint8_t usart1_rx_buf[512];
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
  STM32GPIO PA15(GPIOA, GPIO_PIN_15);
  STM32GPIO PA4(GPIOA, GPIO_PIN_4);
  STM32GPIO PA6(GPIOA, GPIO_PIN_6, EXTI9_5_IRQn);
  STM32GPIO PB3(GPIOB, GPIO_PIN_3);
  STM32GPIO PB4(GPIOB, GPIO_PIN_4);
  STM32GPIO PB8(GPIOB, GPIO_PIN_8, EXTI9_5_IRQn);
  STM32GPIO PC10(GPIOC, GPIO_PIN_10);


  STM32PWM pwm_tim2_ch2(&htim2, TIM_CHANNEL_2, false);
  STM32PWM pwm_tim2_ch3(&htim2, TIM_CHANNEL_3, false);

  STM32PWM pwm_tim5_ch1(&htim5, TIM_CHANNEL_1, false);

  STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);

  // STM32UART uart5(&huart5,
  //             {nullptr, 0}, {nullptr, 0}, 5);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  STM32CAN can2(&hcan2, 5);

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
  static ::Manager::FeymanManager feyman_manager;
  static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);
  static ::Manager::GPIOManager gpio_manager;
  static ::Manager::SyncSignalManager sync_signal_manager;
  static ::Module::YISIMU yis_imu(&i2c1, yis_i2c_addr);
  static ::Manager::FeymanManagedDeviceConfig feyman_devices[] = {
      {0x7E, true},
      {0x7F, true},
  };

  // ========================================================================
  // Create Application task instances (dedicated threads)  创建任务线程
  // ========================================================================

  // 正常运行：静态设备表里直接写最终 node_id。
  // 单机改址：临时只保留一台设备，并把它的 node_id 改成目标地址；
  // 设备对象内部仍按 connect_node_id -> node_id 的配置式流程完成改址，
  // 改完后断电重上，再把静态表改回新的最终地址。
  ::Manager::FeymanManagerConfig feyman_config;
  feyman_config.can = &can2;
  feyman_config.devices = feyman_devices;
  feyman_config.device_count =
      sizeof(feyman_devices) / sizeof(feyman_devices[0]);
  feyman_config.primary_node_id = 0x7E;
  feyman_config.aggregate_topic_name = "feyman_imu_array";
  feyman_config.legacy_topic_name = "feyman_imu_pose";
  feyman_config.priority =
      static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  feyman_config.stack_size = 3072;
  feyman_config.startup_delay_ms = 50;
  feyman_config.baudrate = 250000;
  feyman_config.data_rate_hz = 100;
  feyman_config.heartbeat_ms = 1000;
  feyman_config.sdo_timeout_ms = 200;
  feyman_config.sdo_inter_request_delay_ms = 5;
  feyman_config.work_mode_settle_ms = 50;
  feyman_config.verbose_config_log = true;
  feyman_config.log_writer = Uart5PrintLine;


  // WIT acquisition config: 50Hz, topic "imu_data", high priority,
  // 2048-byte stack.
  ::Manager::IMUManagerAcquisitionConfig wit_acq_config;
  wit_acq_config.frequency_hz = 50;
  wit_acq_config.topic_name = "imu_data";
  wit_acq_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  wit_acq_config.stack_size = 2048;
  wit_acq_config.use_hardware_trigger = true;
  // 1 => only 0x50, 6 => 0x50~0x55.
  wit_acq_config.enabled_imu_count = 6;

  // YIS acquisition config: 200Hz, topic "yis_imu_pose",
  // high priority, 1536-byte stack.
  ::Application::YISIMUAcquisitionConfig yis_config;
  yis_config.frequency_hz = 200;
  yis_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  yis_config.stack_size = 1536;
  yis_config.dr_wait_timeout_ms = 20;
  yis_config.topic_name = "yis_imu_pose";
  static ::Application::YISIMUAcquisitionTask yis_task(&yis_imu, yis_config,&PB8);

  // Bridge config: data-driven push, medium priority, 1536-byte stack.
  // Select bridge pose source here: WIT, YIS, or FEYMAN.
  ::Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;    //字符流输出
  bridge_config.push_imu_euler_in_bridge = true;  //是否主动向 UART 推送姿态数据
  bridge_config.push_sync_events_in_bridge = true; //是否主动向 UART 推送 TIM2/TIM5 同步事件
  bridge_config.pose_source = ::Application::BridgePoseSource::FEYMAN; //桥接数据来源选择，WIT、YIS 或 FEYMAN
  bridge_config.push_all_slots_in_bridge = false; //WIT 多 IMU 推送时，是否把无效槽位也打包进去
  bridge_config.wit_push_imu_count = wit_acq_config.enabled_imu_count;
  bridge_config.stream_interval_ms = 0;   //桥接推送频率不再设置为50hz或者200hz，根据底层数据输出频率决定，上层不加限制
  bridge_config.priority = static_cast<uint32_t>(LibXR::Thread::Priority::MEDIUM);
  bridge_config.stack_size = 1536;
  static ::Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);

  constexpr uint16_t ws2812_led_count = 16;
  const auto ws2812_ec =
      ws2812_manager.Init(&ws2812_strip, 0, ws2812_led_count);
  constexpr uint8_t ws2812_boot_red = 135;
  constexpr uint8_t ws2812_boot_green = 206;
  constexpr uint8_t ws2812_boot_blue = 250;
  const auto ws2812_boot_ec =
      (ws2812_ec == ErrorCode::OK)
          ? ws2812_manager.SetLightControl(
                ::Manager::WS2812Manager::kAllLedsTarget, ws2812_boot_red,
                ws2812_boot_green, ws2812_boot_blue, false, 0U)
          : ws2812_ec;

  const auto imu_init_ec =
      imu_manager.Init(&i2c1, ::Manager::kDefaultImuAddress);
  const auto yis_init_ec = yis_imu.Init();
  const auto feyman_init_ec = feyman_manager.Init(feyman_config);

// 同步信号管理器统一管理通用的 PWM 启动与事件记录流程。
// 该钩子函数用于在本板级层中配置 STM32 定时器更新中断。
  ::Manager::SyncPwmOutputConfig imu_sync_output;
  imu_sync_output.source = ::Manager::SyncEventSource::TIM2_IMU_SYNC_1HZ;
  imu_sync_output.pwm = &pwm_tim2_ch2;
  imu_sync_output.nominal_period_us = 1000000U;
  imu_sync_output.before_enable = EnableTimUpdateInterrupt;
  imu_sync_output.context = &htim2;
  (void)sync_signal_manager.RegisterPwmOutput(imu_sync_output);

  ::Manager::SyncPwmOutputConfig imu_sync_pb10_output = imu_sync_output;
  imu_sync_pb10_output.pwm = &pwm_tim2_ch3;
  imu_sync_pb10_output.before_enable = nullptr;
  imu_sync_pb10_output.context = nullptr;
  (void)sync_signal_manager.RegisterPwmOutput(imu_sync_pb10_output);

// 定时器 5 控制相机触发信号；定时器 2 输出 YIS 1 赫兹时间戳基准信号。
  ::Manager::SyncPwmOutputConfig camera_trigger_output;
  camera_trigger_output.source =
      ::Manager::SyncEventSource::TIM5_CAMERA_TRIGGER_30HZ;
  camera_trigger_output.pwm = &pwm_tim5_ch1;
  camera_trigger_output.nominal_period_us = 33333U;
  camera_trigger_output.before_enable = EnableTimUpdateInterrupt;
  camera_trigger_output.context = &htim5;
  (void)sync_signal_manager.RegisterPwmOutput(camera_trigger_output);

  const uint32_t tim_clk_hz = GetTim5ClockHz();
  const uint32_t tim2_psc =
      static_cast<uint32_t>(htim2.Init.Prescaler) + 1U;
  const uint32_t tim2_arr = static_cast<uint32_t>(htim2.Init.Period) + 1U;
  const uint32_t tim2_ccr = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
  const uint32_t sync_freq_hz =
      (tim2_psc != 0U && tim2_arr != 0U) ? (tim_clk_hz / tim2_psc / tim2_arr)
                                         : 0U;

  const uint32_t tim5_psc =
      static_cast<uint32_t>(htim5.Init.Prescaler) + 1U;
  const uint32_t tim5_arr = static_cast<uint32_t>(htim5.Init.Period) + 1U;
  const uint32_t tim5_ccr = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1);
  const uint32_t camera_freq_hz =
      (tim5_psc != 0U && tim5_arr != 0U) ? (tim_clk_hz / tim5_psc / tim5_arr)
                                         : 0U;

  const ::Manager::GPIOButtonCommand button_commands[] = {
      {&PC10, 'A'},
      {&PA15, 'B'},
      {&PB3, 'C'},
      {&PB4, 'D'},
  };

  ::Manager::GPIOManagerConfig gpio_config;
  gpio_config.command_callback = ::Application::IMUUartBridgeTask::PublishGPIOButtonCommandCallback;
  gpio_config.command_context = &imu_bridge;
  gpio_config.commands = button_commands;
  gpio_config.command_count = sizeof(button_commands) / sizeof(button_commands[0]);
  gpio_config.stack_size = 1024;
  const auto gpio_init_ec = gpio_manager.Init(gpio_config);

  std::snprintf(line, sizeof(line), "[boot] ws2812 ec=%d leds=%u",
                static_cast<int>(ws2812_ec),
                static_cast<unsigned>(ws2812_led_count));
  Uart5PrintLine(line);
  std::snprintf(line, sizeof(line),
                "[boot] ws2812 default=%d rgb=(%u,%u,%u)",
                static_cast<int>(ws2812_boot_ec),
                static_cast<unsigned>(ws2812_boot_red),
                static_cast<unsigned>(ws2812_boot_green),
                static_cast<unsigned>(ws2812_boot_blue));
  Uart5PrintLine(line);
  std::snprintf(line, sizeof(line),
                "[boot] sync pwm registered clk=%lu psc=%lu arr=%lu ccr=%lu freq=%lu",
                static_cast<unsigned long>(tim_clk_hz),
                static_cast<unsigned long>(htim2.Init.Prescaler),
                static_cast<unsigned long>(htim2.Init.Period),
                static_cast<unsigned long>(tim2_ccr),
                static_cast<unsigned long>(sync_freq_hz));
  Uart5PrintLine(line);
  std::snprintf(line, sizeof(line),
                "[boot] camera pwm registered psc=%lu arr=%lu ccr=%lu freq=%lu",
                static_cast<unsigned long>(htim5.Init.Prescaler),
                static_cast<unsigned long>(htim5.Init.Period),
                static_cast<unsigned long>(tim5_ccr),
                static_cast<unsigned long>(camera_freq_hz));
  Uart5PrintLine(line);
  std::snprintf(line, sizeof(line),
                "[boot] yis init=%d addr=0x%02X hal=0x%02X",
                static_cast<int>(yis_init_ec),
                static_cast<unsigned>(yis_imu.address()),
                static_cast<unsigned>(yis_imu.slave_address()));
  Uart5PrintLine(line);
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

  const auto feyman_start_ec =
      (feyman_init_ec == ErrorCode::OK) ? feyman_manager.Start()
                                        : feyman_init_ec;

  // 启动串口桥收发任务
  const auto bridge_start_ec = imu_bridge.Start();

   // 启动按键桥发任务
  const auto gpio_start_ec =
      (gpio_init_ec == ErrorCode::OK && bridge_start_ec == ErrorCode::OK)
          ? gpio_manager.Start()
          : ((gpio_init_ec != ErrorCode::OK) ? gpio_init_ec : bridge_start_ec);

  std::snprintf(line, sizeof(line),
                "[boot] feyman mgr init=%d start=%d devices=%u primary=0x%02X",
                static_cast<int>(feyman_init_ec),
                static_cast<int>(feyman_start_ec),
                static_cast<unsigned>(feyman_config.device_count),
                static_cast<unsigned>(feyman_config.primary_node_id));
  Uart5PrintLine(line);
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
  std::snprintf(line, sizeof(line),
                "[boot] gpio init=%d start=%d PC10=A PA15=B PB3=C PB4=D",
                static_cast<int>(gpio_init_ec),
                static_cast<int>(gpio_start_ec));
  Uart5PrintLine(line);

  // ========================================================================
  // Idle loop
  // ========================================================================

  while (true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}