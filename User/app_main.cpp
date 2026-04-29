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

/*
 * User Layer Helpers
 * - 这一段属于 app_main 的用户层辅助区，放的是启动日志和调试输出开关。
 * - 这些符号都放在匿名命名空间里，表示它们只在当前文件内部使用。
 * - 后面的 Begin2 / Begin3 会根据这里的开关决定是进入诊断模式还是桥接模式。
 */
constexpr bool kEnableUart5PlaintextDiag = false;
constexpr bool kEnableUart5BootLog = true;

// 通过 HAL 直接向 UART5 发送字符串，适合上电阶段做简单阻塞日志输出。
void Uart5Print(const char* text) {
  if (text == nullptr) {
    return;
  }

  const auto len = static_cast<uint16_t>(std::strlen(text));
  (void)HAL_UART_Transmit(&huart5, reinterpret_cast<const uint8_t*>(text), len,
                          100);
}

// 在纯文本日志后补上 CRLF，方便串口工具按行显示。
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
  /*
   * User Layer Boot Entry
   * - 这一段只做入口阶段的模式提示，不负责真正的业务初始化。
   * - 纯文本诊断模式下，会提示 UART5 的串口参数，方便先验证最基础的可观测性。
   * - 普通启动模式下，会提前告诉使用者后续主要通过 USART1 进入桥接交互。
   */
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

  // STM32UART uart5(&huart5,
  //             {nullptr, 0}, {nullptr, 0}, 5);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  /*
   * Module / Manager: WS2812
   * - WS2812Strip 属于 Module 层，直接依赖 SPI，负责把 RGB 数据编码后送到底层总线。
   * - WS2812Manager 属于 Manager 层，负责上层资源管理、参数检查和总线编号匹配。
   * - 这里先完成灯带相关资源装配，后续桥接协议就能通过 manager 间接控制灯带输出。
   */
  static ::Module::WS2812Strip ws2812_strip(&spi1);
  static ::Manager::WS2812Manager ws2812_manager;
  const auto ws2812_ec =
      ws2812_manager.Init(&ws2812_strip, 0);  // 0 表示当前灯带挂在逻辑 spi_bus 0 上。

  /*
   * Manager: IMU
   * - IMUManager 负责 IMU 设备探测、I2C 总线互斥、周期采集和 Topic 数据发布。
   * - ACTUAL_IMU_COUNT 表示当前工程实际启用的 IMU 槽位数量。
   * - 这里完成传感器管理器初始化，并拉起 50Hz 的采集任务，把数据发布到 imu_data。
   */
  static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);
  const auto imu_init_ec =
      imu_manager.Init(&i2c1,
                       ::Manager::kDefaultImuAddress);  // 从默认基地址开始探测 IMU。
  const auto imu_acq_ec =
      imu_manager.StartAcquisition(50, "imu_data");  // 以 50Hz 采集，并向 imu_data 主题发布。

  /*
   * Boot Log
   * - 这一段是用户层启动可观测性输出，不参与核心业务控制。
   * - 它把灯带初始化、IMU 初始化、采集线程启动和在线数量汇总到 UART5。
   * - 上电后先看这一段日志，能最快判断系统是否已经完成关键资源装配。
   */
  if (kEnableUart5BootLog) {
    char line[128] = {0};
    std::snprintf(line, sizeof(line),
                  "[boot] ws2812=%d imu_init=%d imu_start=%d online=%u",
                  static_cast<int>(ws2812_ec), static_cast<int>(imu_init_ec),
                  static_cast<int>(imu_acq_ec),
                  static_cast<unsigned>(
                      imu_manager.GetOnlineCount()));  // online 表示当前探测到的在线 IMU 数量。
    Uart5PrintLine(line);
  }

  /*
   * Diag Mode
   * - 纯文本诊断模式只做状态输出，不再继续启动后面的桥接任务。
   * - 重点观察 WS2812/IMU 初始化状态、IMU 在线数量和当前采集频率。
   * - 适合先确认系统是否“活着”，再排查更高层的串口桥接问题。
   */
  if (kEnableUart5PlaintextDiag) {
    char line[128] = {0};
    std::snprintf(line, sizeof(line),
                  "[diag] ws2812_init=%d imu_init=%d imu_start=%d online=%u",
                  static_cast<int>(ws2812_ec), static_cast<int>(imu_init_ec),
                  static_cast<int>(imu_acq_ec),
                  static_cast<unsigned>(imu_manager.GetOnlineCount()));
    Uart5PrintLine(line);
    Uart5PrintLine("[diag] bridge disabled in plaintext diagnostic mode");

    uint32_t heartbeat = 0;  // 心跳计数器，用来确认当前任务仍在持续运行。
    while (true) {
      std::snprintf(line, sizeof(line),
                    "[diag] heartbeat=%lu online=%u freq=%lu",
                    static_cast<unsigned long>(heartbeat++),
                    static_cast<unsigned>(imu_manager.GetOnlineCount()),
                    static_cast<unsigned long>(imu_manager.GetFrequency()));
      Uart5PrintLine(line);
      Thread::Sleep(1000);  // 每 1 秒主动让出 CPU 一次，而不是裸机忙等。
    }
  }

  /*
   * Application: Bridge
   * - 这一段进入 Application 层，把前面装配好的 UART / I2C / SPI / Manager 注入桥接任务。
   * - 当前配置不走简单文本欧拉角流，而是走二进制桥接协议模式。
   * - 同时开启桥接中的 IMU 主动推送，让上位机能周期拿到姿态数据。
   */
  static ::Application::IMUUartBridgeConfig bridge_config;
  bridge_config.stream_relative_euler = false;  // false: 不走简单文本流模式，改走桥接协议模式。
  bridge_config.push_imu_euler_in_bridge =
      true;  // 在桥接模式下主动推送 IMU 姿态数据。
  bridge_config.stream_interval_ms = 20;  // 20ms 一次推送，对应 50Hz。
  bridge_config.stack_size = 1000;  // 桥接任务栈大小，供串口协议解析和资源访问使用。
  // 把 USART1 / I2C1 / SPI1 / IMUManager / WS2812Manager 注入到应用层桥接任务。
  static ::Application::IMUUartBridgeTask imu_bridge(
      &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
  (void)imu_bridge.Start();  // 真正创建并启动桥接线程。

  // 这里表示桥接任务已经完成启动请求，后续可通过 USART1 进入桥接交互。
  if (kEnableUart5BootLog) {
    Uart5PrintLine("[boot] USART1 bridge ready");
  }

  /*
   * Runtime / Idle
   * - app_main() 到这里已经完成“装配和启动”职责。
   * - 后续长期运行依赖的是已创建的采集线程、桥接线程和定时任务，而不是这个入口继续执行业务。
   * - 让当前默认任务长期休眠，可以避免空转占用 CPU。
   */
  while (true) {
    Thread::Sleep(UINT32_MAX);  // 让默认任务长期挂起，保持系统由其他工作任务驱动。
  }
  /* User Code End 3 */
}