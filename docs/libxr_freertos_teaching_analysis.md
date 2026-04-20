# libxr + FreeRTOS 工程全链路教学分析

本文档按“构建入口 -> 启动入口 -> `app_main.cpp` 主流程 -> 直接依赖 -> libxr 模块映射 -> RTOS 原理”的顺序讲解，目标是让刚接触 `libxr + STM32 + FreeRTOS` 的同学，能顺着源码把这套工程真正看懂。

说明：

- 本文不修改任何源码，只做教学型分析。
- 主分析对象是 `User/app_main.cpp`，但会把它直接依赖的 `IMUManager`、`IMUUartBridgeTask`、`WS2812Manager/WS2812Strip` 一起讲透。
- 对 `app_main.cpp` 采用“逐段为主、关键行逐行补充”的讲法。
- 对第三方 `libxr` 框架，只讲当前工程实际走到的那条路径，不无边界展开。

---

## 1. 先看整体图：这套工程到底在干什么

### 1.1 一句话概括

这套工程是一个基于 **STM32F105RCT6 + FreeRTOS + libxr** 的手部/IMU 数据桥接工程：

- `I2C1` 挂着多个 IMU 传感器；
- `SPI1` 负责驱动 WS2812 灯带；
- `USART1` 既当终端输入输出口，也当桥接协议口；
- `UART5` 只做上电日志/诊断打印；
- `IMUManager` 周期采集 IMU 数据并发布到 `imu_data` 主题；
- `IMUUartBridgeTask` 订阅 `imu_data`，同时处理上位机经 `USART1` 发来的桥接命令。

### 1.2 上电后的执行链

```text
上电复位
-> main()
-> HAL_Init()
-> SystemClock_Config()
-> GPIO / DMA / I2C / SPI / UART 外设初始化
-> osKernelInitialize()
-> MX_FREERTOS_Init()
-> 创建 defaultTask
-> osKernelStart()
-> defaultTask 开始运行
-> app_main()
-> 构造 libxr 驱动对象(STM32UART / STM32SPI / STM32I2C / STM32TimerTimebase ...)
-> 初始化终端、WS2812、IMU
-> IMUManager 创建采集线程并发布 imu_data
-> IMUUartBridgeTask 创建桥接线程
-> 桥接线程经 USART1 对外响应命令 / 推送数据
-> UART5 继续输出启动日志
```

### 1.3 这套工程的人为分层思路

这份代码的人为分层其实很清晰：

```text
HAL 句柄层
-> 例如 huart1 / hi2c1 / hspi1 / htim1

libxr 驱动包装层
-> STM32UART / STM32I2C / STM32SPI / STM32TimerTimebase

manager 资源管理层
-> IMUManager / WS2812Manager

application 业务协议层
-> IMUUartBridgeTask

用户业务入口
-> app_main()
```

这样做的好处是：

- CubeMX 只负责把底层外设“点亮”；
- `libxr` 负责给外设统一抽象；
- `manager` 层专门管资源、设备状态、共享总线；
- `application` 层只写业务协议和任务逻辑；
- `app_main()` 负责装配这些对象。

---

## 2. 构建层：CMake 怎么把整套工程拼起来

### 2.1 顶层 `CMakeLists.txt`

顶层 `CMakeLists.txt` 做了几件关键事：

1. 定义工程名 `f105rct6`。
2. 创建最终可执行目标 `add_executable(${CMAKE_PROJECT_NAME})`。
3. `add_subdirectory(cmake/stm32cubemx)` 引入 CubeMX 生成的 HAL / FreeRTOS / 启动文件。
4. `include(cmake/LibXR.CMake)` 把 `libxr` 接入进来。
5. 再把 `modules`、`managers`、`application` 三层库接进来。
6. 最后让最终可执行文件链接 `application`。

换句话说，顶层只做“装配”，不直接关心业务逻辑细节。

### 2.2 `cmake/LibXR.CMake`

这个文件是当前工程把 `libxr` 真正接入进来的核心：

- `set(LIBXR_SYSTEM FreeRTOS)`：告诉 `libxr`，系统抽象层要使用 `system/FreeRTOS`。
- `set(LIBXR_DRIVER st)`：告诉 `libxr`，驱动层要使用 `driver/st`，也就是 STM32 HAL 版本。
- `add_subdirectory(Middlewares/Third_Party/LibXR)`：把 `libxr` 作为子工程加入。
- 把 `xr` 链接到 `stm32cubemx`，这样 `libxr` 可以用到 HAL 句柄、头文件、底层启动环境。
- 再把 `xr` 和 `User/*.cpp` 加到主工程里。

所以这一步本质是在说：

> “当前工程运行在 FreeRTOS 上，底层芯片驱动采用 STM32 HAL，`libxr` 就按这两个适配层来编译。”

### 2.3 `modules / managers / application`

这三个目录对应三层静态库：

- `modules`
  - 放设备级模块；
  - 当前有 `wit_imu_jy901b`、`ws2812`。
- `managers`
  - 在模块之上做资源和状态管理；
  - 当前有 `IMUManager`、`WS2812Manager`。
- `application`
  - 再往上做业务任务；
  - 当前是 `IMUUartBridgeTask`。

链接关系是：

```text
stm32cubemx
-> xr
-> module
-> managers
-> application
-> f105rct6(最终可执行文件)
```

这个顺序很重要，因为上层依赖下层：

- `application` 依赖 `managers + xr`
- `managers` 依赖 `module + xr`
- `module` 依赖 `xr`
- `xr` 依赖 `stm32cubemx`

---

## 3. 启动层：程序是怎么从 `main()` 走到 `app_main()` 的

### 3.1 `main.c` 的职责

`Core/Src/main.c` 是 CubeMX 生成的 MCU 启动入口，它做的事非常标准：

1. `HAL_Init()`
   - 初始化 HAL；
   - 初始化 Flash 接口；
   - 初始化 HAL 时基。
2. `SystemClock_Config()`
   - 配系统时钟。
3. 初始化外设：
   - `MX_GPIO_Init()`
   - `MX_DMA_Init()`
   - `MX_I2C1_Init()`
   - `MX_SPI1_Init()`
   - `MX_UART5_Init()`
   - `MX_USART1_UART_Init()`
4. `osKernelInitialize()`
   - 初始化 RTOS 内核对象；
   - 这时调度器还没跑起来。
5. `MX_FREERTOS_Init()`
   - 创建线程、队列、信号量等 RTOS 对象。
6. `osKernelStart()`
   - 启动 FreeRTOS 调度器；
   - 从这之后，CPU 的控制权就交给调度器了。

这说明一件很关键的事：

> `app_main()` 并不是裸机入口，而是在 FreeRTOS 线程环境里运行的。

### 3.2 `freertos.c` 的职责

`Core/Src/freertos.c` 当前只创建了一个默认线程：

- 线程名：`defaultTask`
- 栈大小：`1024 * 4`
- 优先级：`osPriorityNormal`

它的线程函数是：

```c
void StartDefaultTask(void *argument)
{
  app_main();
  for(;;)
  {
    osDelay(1);
  }
}
```

这段代码的意思是：

- FreeRTOS 调度器启动后，`defaultTask` 开始运行；
- `defaultTask` 直接调用 `app_main()`；
- 如果 `app_main()` 将来返回了，线程就每 1ms 空转一次；
- 但在你这个工程里，`app_main()` 本身不会返回。

### 3.3 新手一定要明白的 RTOS 概念

#### `defaultTask` 是什么

它就是一个 **普通 FreeRTOS 任务**，只是 CubeMX 帮你先建好了一个默认入口，方便你把用户逻辑挂进去。

#### 为什么 `app_main()` 不是裸跑

因为 `app_main()` 是从 `StartDefaultTask()` 里被调用的，所以它天然带着：

- 任务上下文；
- 可被调度；
- 可以调用 `osDelay` / `Thread::Sleep`；
- 可以再创建更多任务。

这和裸机 `while(1)` 最大的区别是：

> 裸机只有一条主线；RTOS 下，`app_main()` 只是众多任务中的起点之一。

---

## 4. `app_main.cpp` 整体功能、流程、输入输出

### 4.1 这段代码整体在做什么

`User/app_main.cpp` 的核心职责可以压缩成 6 步：

1. 打印启动日志；
2. 构造 `libxr` 时间基准和硬件抽象对象；
3. 把 `USART1` 挂到 `libxr` 终端 STDIO；
4. 初始化 WS2812 管理器；
5. 初始化 IMU 管理器并启动采集线程；
6. 创建 UART 桥接任务，让上位机通过 `USART1` 与 IMU / SPI / WS2812 交互。

### 4.2 输入

这份代码的输入来源主要有 4 类：

- `I2C1`
  - 来自 IMU 传感器的数据。
- `USART1`
  - 来自上位机的桥接协议命令；
  - 也作为终端输入。
- `DMA / 中断`
  - 辅助 UART、SPI、I2C 完成异步数据搬运。
- `RTOS Tick`
  - 给线程休眠、超时、周期调度提供时基。

### 4.3 输出

输出也主要有 4 类：

- `USART1`
  - 输出终端交互内容；
  - 输出桥接协议响应；
  - 主动推送 IMU 数据。
- `UART5`
  - 输出启动日志和诊断日志。
- `SPI1`
  - 输出 WS2812 编码后的灯带数据。
- `Topic("imu_data")`
  - 在 `libxr` 内部作为消息总线，把 IMU 数据发给订阅者。

---

## 5. `app_main.cpp` 逐段 / 逐行讲解

下面按代码自然分块来讲。

---

### 5.1 头文件与命名空间

原代码：

```cpp
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
```

逐段解释：

- `#include "app_main.h"`
  - 声明 `app_main()`；
  - 给 `freertos.c` 调用用。
- `#include "libxr.hpp"`
  - `libxr` 总头文件；
  - 里面会把常用系统抽象、驱动抽象、工具组件带进来。
- `#include "main.h"`
  - CubeMX 生成的主头文件；
  - 含 HAL、芯片型号、常量定义。
- `stm32_xxx.hpp`
  - 这些是 `libxr` 的 STM32 驱动包装层；
  - 作用是把 HAL 句柄包装成统一接口。
- `flash_map.hpp`
  - 用户定义的 Flash 映射；
  - 本文件里实际没用到，但属于工程公共头。
- `using namespace LibXR;`
  - 后面直接写 `Thread`、`Timer`、`STM32UART` 就行，不用每次都写 `LibXR::`。

这里要特别指出：

- `stm32_adc.hpp`、`stm32_can.hpp`、`stm32_canfd.hpp`、`stm32_dac.hpp`、`stm32_pwm.hpp`、`stm32_usb_dev.hpp`、`stm32_watchdog.hpp` 在当前 `app_main.cpp` 里 **被包含但没有实际参与执行逻辑**。
- 这通常说明这个入口文件是按统一模板搭起来的，后续按需启用。

#### 这一段用到的 libxr 模块

- `driver/st`
  - `stm32_gpio.hpp`
  - `stm32_i2c.hpp`
  - `stm32_power.hpp`
  - `stm32_spi.hpp`
  - `stm32_timebase.hpp`
  - `stm32_uart.hpp`

#### 设计思想

这就是典型的“**HAL 句柄不直接四处乱用，而是先包成统一驱动对象**”。

---

### 5.2 用户业务头文件和 UART5 日志辅助函数

原代码：

```cpp
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
```

逐行解释：

- `#include <cstdio>`
  - 用 `std::snprintf` 组装日志字符串。
- `#include <cstring>`
  - 用 `std::strlen` 计算字符串长度。
- `#include "application/imu_uart_bridge_task.hpp"`
  - 引入桥接任务类。
- `#include "managers/imu_manager.hpp"`
  - 引入 IMU 管理器。
- `#include "managers/ws2812_manager.hpp"`
  - 引入 WS2812 管理器。
- `#include "modules/ws2812/ws2812_strip.hpp"`
  - 引入 WS2812 底层驱动模块。
- `extern UART_HandleTypeDef huart5;`
  - 声明这个 HAL 句柄来自 CubeMX 生成的 `usart.c`。
- `namespace { ... }`
  - 匿名命名空间；
  - 里面的常量和函数只在本文件内部可见。
- `kEnableUart5PlaintextDiag`
  - 是否进入“纯文本诊断模式”；
  - 当前是 `false`。
- `kEnableUart5BootLog`
  - 是否打印 UART5 启动日志；
  - 当前是 `true`。
- `Uart5Print`
  - 封装一个最简单的 UART5 阻塞发送函数。
- `if (text == nullptr) return;`
  - 防空指针。
- `std::strlen(text)`
  - 计算待发送字符串长度。
- `HAL_UART_Transmit(...)`
  - 直接使用 HAL 阻塞式发送；
  - 最后一个参数 `100` 是超时时间，单位 ms。
- `Uart5PrintLine`
  - 先发正文，再补 `\r\n`。

#### 为什么这里直接用 `HAL_UART_Transmit`，而不是 `STM32UART`

这是一个很实用的工程选择：

- `USART1` 是主业务串口，要交给 `libxr` 包装成异步端口；
- `UART5` 只是辅助日志口，需求简单；
- 直接 `HAL_UART_Transmit` 可以避免把日志路径也卷进更复杂的端口/队列逻辑里。

对新手来说可以这样理解：

> `libxr UART` 更像“正式通信通道”，适合队列、DMA、异步读写；
> `HAL_UART_Transmit` 更像“临时喊一嗓子”，适合上电日志、故障打印。

#### RTOS 角度怎么理解这段

这里用的是 **阻塞式串口发送**。阻塞的意思是：

- 调用函数后，当前任务要在这里等；
- 等到发送完成或者超时才继续往下走。

因为这里只在启动阶段打印少量日志，所以问题不大；如果高频打印，就可能拖慢任务。

---

### 5.3 HAL 外设句柄声明与 DMA 缓冲区

原代码：

```cpp
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

static uint8_t spi1_tx_buf[768];
static uint8_t usart1_tx_buf[512];
static uint8_t usart1_rx_buf[128];
static uint8_t i2c1_buf[96];
```

逐行解释：

- `extern I2C_HandleTypeDef hi2c1;`
  - 引用 CubeMX 生成的 I2C1 句柄。
- `extern SPI_HandleTypeDef hspi1;`
  - 引用 SPI1 句柄。
- `extern TIM_HandleTypeDef htim1;`
  - 引用 TIM1 句柄；
  - 用来做 `libxr` 时间基准。
- `extern UART_HandleTypeDef huart1;`
  - 引用 USART1 句柄；
  - 业务桥接和终端都用它。
- `extern UART_HandleTypeDef huart5;`
  - 引用 UART5 句柄；
  - 用来打日志。

静态缓冲区：

- `spi1_tx_buf[768]`
  - SPI1 的发送缓冲；
  - 主要给 WS2812 编码数据用。
- `usart1_tx_buf[512]`
  - USART1 的 DMA 发送缓冲。
- `usart1_rx_buf[128]`
  - USART1 的 DMA 接收缓冲。
- `i2c1_buf[96]`
  - I2C1 的 DMA / 临时数据缓冲。

#### 为什么大小不一样

因为不同外设的数据特征不同：

- WS2812 发的是编码后的灯带波形，数据量明显更大；
- UART 日志/协议帧中等；
- I2C 一次读寄存器一般数据较短。

#### RTOS / DMA 角度怎么理解

这些数组本质上是“任务和 DMA 之间的中转仓库”。

可以把 DMA 理解成一个“自动搬运工”：

- CPU 只要告诉它“从哪搬到哪、搬多少”；
- DMA 就自己搬；
- 搬完后通过中断告诉 CPU：“我搬完了”。

所以你经常会看到这种组合：

```text
任务提交读写请求
-> DMA 用缓冲区搬数据
-> 中断回调触发
-> 信号量/状态更新
-> 任务继续往下执行
```

---

### 5.4 `app_main()` 开头：启动日志

原代码：

```cpp
extern "C" void app_main(void) {
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
```

逐行解释：

- `extern "C" void app_main(void)`
  - 用 C 链接方式导出，方便 C 文件里的 `freertos.c` 调用。
- `if (kEnableUart5PlaintextDiag)`
  - 如果进入纯文本诊断模式，就打印诊断启动信息。
- `else if (kEnableUart5BootLog)`
  - 否则只打印普通启动日志。

当前配置下：

- `kEnableUart5PlaintextDiag = false`
- `kEnableUart5BootLog = true`

所以实际运行的是：

- 打一行空行；
- 打出 `app_main entered`；
- 告诉你当前是“USART1 bridge mode”；
- 告诉你“UART5 用阻塞日志”。

#### 这段的意义

这不是业务核心，但很有工程价值：

- 上电后你立刻能确认程序有没有跑进 `app_main()`；
- 你能知道当前串口工作模式；
- 你能区分“代码没跑起来”和“后面某个模块初始化失败”。

---

### 5.5 时间基准、平台初始化、电源对象

原代码：

```cpp
STM32TimerTimebase timebase(&htim1);
PlatformInit(2, 1024);
STM32PowerManager power_manager;
```

逐行解释：

- `STM32TimerTimebase timebase(&htim1);`
  - 用 TIM1 构造 `libxr` 的时间基准对象；
  - 以后 `Timebase::GetMicroseconds()`、`GetMilliseconds()` 这些接口就有底层来源了。
- `PlatformInit(2, 1024);`
  - 初始化 `libxr` 的 FreeRTOS 平台层；
  - `2` 是 `libxr` 软件定时器线程优先级；
  - `1024` 是它的栈深度，单位字节。
- `STM32PowerManager power_manager;`
  - 构造电源管理对象；
  - 当前文件里没继续使用，但构造本身说明该平台支持电源管理抽象。

#### 这里用到的 libxr 模块

- `driver/st`
  - `STM32TimerTimebase`
  - `STM32PowerManager`
- `system/FreeRTOS`
  - `PlatformInit`
- `src/system`
  - `Timer`

#### 为什么一定要先建 `timebase` 再 `PlatformInit`

`PlatformInit()` 的实现里会检查 `Timebase::timebase` 是否已经存在。也就是说：

- 没有时间基准；
- `libxr` 的系统层就没法正常工作。

你可以把时间基准理解成：

> “整个框架里所有‘多久、几点、是否超时、多久执行一次’的公共时钟。”

#### TIM1、HAL tick、FreeRTOS tick 在这个工程里的关系

这部分最容易让新手混乱，分开看：

- **HAL tick**
  - 由 `HAL_GetTick()` 维护；
  - 当前工程用 `TIM1` 中断每 1ms 调一次 `HAL_IncTick()` 来累加。
- **FreeRTOS tick**
  - 由 FreeRTOS 内核维护；
  - 当前配置 `configTICK_RATE_HZ = 1000`，也是 1ms 一个 tick。
- **libxr timebase**
  - `STM32TimerTimebase` 通过 `HAL_GetTick()` 和 TIM 计数器，提供更细粒度时间戳；
  - 给 `libxr` 的 `Timebase::GetMicroseconds()` 使用。

可以先这样记：

- `HAL tick` 更像 HAL 世界里的时间；
- `FreeRTOS tick` 更像任务调度世界里的时间；
- `libxr timebase` 是在两者之上给你更统一、细一点的时间接口。

---

### 5.6 GPIO 和外设驱动对象构造

原代码：

```cpp
STM32GPIO PA4(GPIOA, GPIO_PIN_4);

STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);

STM32UART uart5(&huart5,
            {nullptr, 0}, {nullptr, 0}, 5);

STM32UART usart1(&huart1,
            usart1_rx_buf, usart1_tx_buf, 5);

STM32I2C i2c1(&hi2c1, i2c1_buf, 3);
```

逐行解释：

- `STM32GPIO PA4(GPIOA, GPIO_PIN_4);`
  - 用 `libxr` 的 GPIO 包装类构造一个 PA4 对象；
  - 当前文件里没有继续使用，可能是预留片选/控制脚。

- `STM32SPI spi1(&hspi1, {nullptr, 0}, spi1_tx_buf, 3);`
  - 用 HAL 的 `hspi1` 构造一个 `libxr` SPI 对象；
  - 接收缓冲区传 `{nullptr, 0}`，说明当前更关注发送；
  - 发送缓冲区用 `spi1_tx_buf`；
  - `3` 表示数据长度大于 3 字节时优先走 DMA。

- `STM32UART uart5(&huart5, {nullptr, 0}, {nullptr, 0}, 5);`
  - 给 UART5 也构造了一个 `libxr` UART 对象；
  - 但没提供 RX/TX DMA 缓冲，所以它并不是当前主通信路径。

- `STM32UART usart1(&huart1, usart1_rx_buf, usart1_tx_buf, 5);`
  - 用 HAL 的 `huart1` 构造真正的业务串口对象；
  - 接收缓冲 `usart1_rx_buf`；
  - 发送缓冲 `usart1_tx_buf`；
  - 队列大小 5。

- `STM32I2C i2c1(&hi2c1, i2c1_buf, 3);`
  - 构造 `libxr` I2C 对象；
  - `i2c1_buf` 作为 DMA / 中间缓冲；
  - 长度大于 3 字节时可走 DMA。

#### 这一段最重要的思想

这一步是在做：

> “把 CubeMX 生成的 HAL 句柄，升级成 `libxr` 的统一驱动对象。”

从此往后：

- 业务层不直接面对 `HAL_I2C_Mem_Read_DMA(...)`；
- 而是面对更统一的 `i2c1.MemRead(...)`；
- 不直接面对 `HAL_UART_Transmit_DMA(...)`；
- 而是面对 `usart1.Write(...)`。

这就是框架带来的抽象价值。

#### RTOS / DMA 角度怎么理解

例如 `STM32UART usart1(...)` 的构造过程中，底层会做几件事：

- 把接收 DMA 设成循环模式；
- 调 `HAL_UARTEx_ReceiveToIdle_DMA(...)` 开始持续接收；
- 收到数据后，中断回调把数据推进 `read_port_` 队列。

也就是说，`USART1` 在 `app_main()` 里一构造出来，底层接收链路就已经开始转了。

---

### 5.7 STDIO、RamFS、Terminal、Timer

原代码：

```cpp
STDIO::read_ = usart1.read_port_;
STDIO::write_ = usart1.write_port_;

RamFS ramfs("XRobot");
Terminal<32, 32, 5, 5> terminal(ramfs);
auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
Timer::Add(terminal_task);
Timer::Start(terminal_task);
```

逐行解释：

- `STDIO::read_ = usart1.read_port_;`
  - 把 `libxr` 标准输入指向 `USART1` 的读端口。
- `STDIO::write_ = usart1.write_port_;`
  - 把 `libxr` 标准输出指向 `USART1` 的写端口。

从这两句开始，`USART1` 就被当成“系统终端口”了。

- `RamFS ramfs("XRobot");`
  - 创建一个内存文件系统；
  - 根目录名叫 `XRobot`。
- `Terminal<32, 32, 5, 5> terminal(ramfs);`
  - 创建终端对象；
  - 读缓冲 32；
  - 最大行长 32；
  - 最多 5 个参数；
  - 最多 5 条历史记录。
- `auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);`
  - 创建一个 libxr 定时任务；
  - 每 10ms 调一次终端任务函数。
- `Timer::Add(terminal_task);`
  - 把这个任务加入 `libxr` 定时器列表。
- `Timer::Start(terminal_task);`
  - 启动这个定时任务。

#### 为什么终端不是单独线程，而是挂到 `Timer`

因为终端在这里不是“高实时、长阻塞”的任务，而是“定期轮询一下有没有串口输入”。

好处是：

- 少建一个独立线程；
- 少占一份任务栈；
- 结构更轻量。

你可以把它理解成：

> “终端不是全天候霸占 CPU 的服务员，而是每 10ms 出来看看有没有人说话。”

#### `Timer` 在这套工程里不是 FreeRTOS 软件定时器

这点很重要。

这里的 `libxr::Timer` 是框架自己实现的一层周期调度器：

- 它内部会创建一个 `libxr_timer_task` 线程；
- 线程每 1ms 刷新一次任务表；
- 到周期就执行对应回调。

所以当前终端任务的真实调用链是：

```text
PlatformInit()
-> Timer 系统准备好
-> Timer::Add()
-> libxr_timer_task 周期刷新
-> 每 10ms 调一次 terminal.TaskFun(&terminal)
```

这是一种“**线程驱动的软轮询定时器**”，不是 FreeRTOS 自带的 `xTimer` 那一套。

---

### 5.8 WS2812 管理器与 IMU 管理器初始化

原代码：

```cpp
static ::Module::WS2812Strip ws2812_strip(&spi1);
static ::Manager::WS2812Manager ws2812_manager;
const auto ws2812_ec = ws2812_manager.Init(&ws2812_strip, 0);

static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);
const auto imu_init_ec =
    imu_manager.Init(&i2c1, ::Manager::kDefaultImuAddress);
const auto imu_acq_ec = imu_manager.StartAcquisition(50, "imu_data");
```

逐行解释：

- `static ::Module::WS2812Strip ws2812_strip(&spi1);`
  - 创建 WS2812 底层驱动模块；
  - 它直接使用 `spi1` 输出编码后的灯带数据。
- `static ::Manager::WS2812Manager ws2812_manager;`
  - 创建 WS2812 管理器；
  - 它负责做更上层的参数检查和资源管理。
- `ws2812_manager.Init(&ws2812_strip, 0);`
  - 把底层驱动交给管理器；
  - 指定它属于 `spi_bus = 0`。

- `static ::Manager::IMUManager imu_manager(::Manager::ACTUAL_IMU_COUNT);`
  - 创建 IMU 管理器；
  - 当前配置的实际 IMU 数量是 `ACTUAL_IMU_COUNT = 4`。

- `imu_manager.Init(&i2c1, ::Manager::kDefaultImuAddress);`
  - 把 I2C1 交给 IMU 管理器；
  - 以默认地址 `0x50` 为起始地址探测 IMU。

- `imu_manager.StartAcquisition(50, "imu_data");`
  - 启动 IMU 采集线程；
  - 采样频率 50Hz；
  - 把数据发布到主题 `imu_data`。

#### 这里发生了哪些关键事

`IMUManager::Init()` 做的事：

1. 等传感器上电稳定；
2. 给每个槽位创建 `WitIMU` 对象；
3. 按地址 `0x50~0x53` 探测 4 个 IMU；
4. 记录哪些 IMU 在线。

`IMUManager::StartAcquisition()` 做的事：

1. 检查当前是否已有采集线程；
2. 检查频率参数；
3. 检查是否至少有一个 IMU 在线；
4. 检查 FreeRTOS 堆是否足够创建新任务；
5. 创建主题 `imu_data`；
6. 创建一个高优先级采集线程；
7. 周期读取所有在线 IMU 并发布消息。

#### IMU 地址映射规则

`data_types.hpp` 里定义的映射是：

- `Forearm` -> `0x50`
- `Hand` -> `0x51`
- `ThumbRoot` -> `0x52`
- `ThumbTip` -> `0x53`

这说明当前工程把 4 个 IMU 看成 4 个固定槽位，而不是“随便插几个就算几个”。

#### RTOS 角度怎么理解：为什么这里要有 `Mutex`

`IMUManager` 内部有一个 `bus_mutex_`，原因很简单：

- IMU 采集线程要访问 I2C；
- 桥接线程收到上位机命令时，也可能访问同一条 I2C；
- 如果两条线程同时用 I2C，总线就会乱。

所以这里加互斥锁，含义就是：

> “谁先拿到钥匙，谁先用 I2C，总线一次只服务一个任务。”

这就是互斥锁最典型的用途：保护共享资源。

---

### 5.9 启动阶段状态打印

原代码：

```cpp
if (kEnableUart5BootLog) {
  char line[128] = {0};
  std::snprintf(line, sizeof(line),
                "[boot] ws2812=%d imu_init=%d imu_start=%d online=%u",
                static_cast<int>(ws2812_ec), static_cast<int>(imu_init_ec),
                static_cast<int>(imu_acq_ec),
                static_cast<unsigned>(imu_manager.GetOnlineCount()));
  Uart5PrintLine(line);
}
```

这一段很实用：

- `ws2812_ec`
  - WS2812 初始化结果；
- `imu_init_ec`
  - IMU 探测初始化结果；
- `imu_acq_ec`
  - IMU 采集线程启动结果；
- `online`
  - 最终有多少个 IMU 在线。

如果启动失败，第一现场日志就会告诉你是：

- 灯带没配好；
- IMU 没探到；
- 还是任务栈/堆不够导致线程没建起来。

---

### 5.10 纯文本诊断模式分支

原代码：

```cpp
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
```

逐段解释：

- 如果开启诊断模式；
- 程序不会启动桥接任务；
- 而是每秒往 UART5 打一次心跳日志。

为什么这很有用？

- 你可以先排除“系统有没有活着”；
- 先看 IMU 在线数量；
- 再看采集频率是不是预期值；
- 让 `USART1` 桥接逻辑暂时退出战场，方便隔离问题。

#### `Thread::Sleep(1000)` 在这里的含义

它对应到底层 FreeRTOS 的 `vTaskDelay(1000)`。

通俗地说：

- 当前任务主动说：“我 1000ms 内不用 CPU 了”；
- 调度器就去运行别的任务。

这就是 RTOS 和裸机延时的一个本质差别：

> RTOS 延时不是傻等，而是“我先让开，让别的任务跑”。

---

### 5.11 桥接任务配置与启动

原代码：

```cpp
static ::Application::IMUUartBridgeConfig bridge_config;
bridge_config.stream_relative_euler = false;
bridge_config.push_imu_euler_in_bridge = true;
bridge_config.stream_interval_ms = 20;
bridge_config.stack_size = 1000;
static ::Application::IMUUartBridgeTask imu_bridge(
    &usart1, &i2c1, &spi1, &imu_manager, &ws2812_manager, bridge_config);
(void)imu_bridge.Start();
```

逐行解释：

- `IMUUartBridgeConfig bridge_config;`
  - 创建桥接任务配置结构体。
- `stream_relative_euler = false;`
  - 不走简单欧拉角流模式；
  - 而是走桥接协议模式。
- `push_imu_euler_in_bridge = true;`
  - 在桥接协议模式下，允许主动推送 IMU 数据。
- `stream_interval_ms = 20;`
  - 20ms 推一次；
  - 对应 50Hz。
- `stack_size = 1000;`
  - 给桥接任务分配 1000 字节栈。
- `IMUUartBridgeTask imu_bridge(...)`
  - 注入桥接任务所需的全部资源：
    - `USART1`
    - `I2C1`
    - `SPI1`
    - `IMUManager`
    - `WS2812Manager`
- `imu_bridge.Start()`
  - 真正创建桥接线程。

#### 这个桥接任务到底做什么

它干两类事：

1. **被动响应命令**
   - 上位机通过 `USART1` 发送协议帧；
   - 它解析命令；
   - 转去操作 I2C / SPI / WS2812；
   - 再返回响应帧。

2. **主动推送数据**
   - 订阅 `imu_data`；
   - 到时间就把 IMU 数据主动打包发给上位机；
   - 同时周期发送诊断帧。

#### RTOS 角度怎么理解：为什么建线程前先查堆

桥接任务和 IMU 采集任务都先用了 `xPortGetFreeHeapSize()`。

原因很现实：

- 创建任务要分配任务控制块；
- 还要分配任务栈；
- 还可能顺带分配对象自身内存。

如果不提前查堆，等创建到一半失败，现场会更难排查。

你可以把这理解成：

> “开一个新任务前，先看宿舍里还有没有空床位。”

---

### 5.12 尾部保活循环

原代码：

```cpp
if (kEnableUart5BootLog) {
  Uart5PrintLine("[boot] USART1 bridge ready");
}

while(true) {
  Thread::Sleep(UINT32_MAX);
}
```

逐行解释：

- 如果启用了启动日志；
  - 打出桥接已经就绪。
- 然后进入死循环；
  - 但不是空转；
  - 而是一直超长休眠。

为什么这里不直接 `return`？

因为 `app_main()` 是从 `defaultTask` 调进来的，如果它返回了：

- `defaultTask` 会继续执行后面的空循环；
- 逻辑上反而更绕。

这里显式睡死，表达得更清楚：

> `app_main()` 的初始化工作做完后，它自己不再承担业务循环；
> 真正长期工作的，是已经创建出来的那些子任务。

---

## 6. `app_main.cpp` 小结：它的执行流程

把 `app_main()` 再压缩一下：

1. 打启动日志；
2. 初始化 `libxr` 时间基准和平台层；
3. 用 HAL 句柄构造 `libxr` 驱动对象；
4. 把 `USART1` 设成终端 STDIO；
5. 启动终端定时任务；
6. 初始化 WS2812；
7. 初始化 IMU 并启动采集线程；
8. 根据模式决定是否进入纯文本诊断；
9. 启动桥接线程；
10. 主线程长期休眠。

所以 `app_main()` 不是“主循环业务处理函数”，而更像：

> 整个机器人应用的“装配车间”和“调度启动器”。

---

## 7. 依赖层下钻 1：`IMUManager`

`IMUManager` 是这个工程最关键的资源管理器之一。

### 7.1 它的职责

它负责：

- 管 IMU 数量与槽位；
- 根据地址创建 `WitIMU` 对象；
- 探测哪些 IMU 在线；
- 读取所有 IMU 数据；
- 做数据格式转换；
- 开采集线程；
- 把数据发布到 `imu_data` 主题；
- 用互斥锁保护 I2C 总线。

### 7.2 初始化流程

`Init()` 的核心流程是：

```text
保存 I2C 指针和基地址
-> 延时等待 IMU 上电稳定
-> 按槽位循环创建 WitIMU 对象
-> 调用每个 WitIMU::Init()
-> 反复 Probe，确认设备是否在线
-> 在线则记录 online_mask
```

这里有两个新手很容易忽略的点：

#### 1. 先等 120ms

`Thread::Sleep(kInitBootDelayMs);`

这不是多余，而是给传感器上电稳定时间。很多外设刚上电时立刻通信，成功率并不高。

#### 2. 不只探测一次，而是重试多次

这说明作者很清楚硬件上电阶段可能存在短暂不稳定，所以不是“一次失败就判死刑”。

### 7.3 数据读取流程

`ReadAll()` 做的事：

1. 先上互斥锁；
2. 清空输出消息；
3. 递增序号 `sequence`；
4. 对每个在线 IMU：
   - 读寄存器；
   - 解析原始数据；
   - 转成工程自己的 `IMUData` 结构；
   - 补时间戳；
   - 置 `valid_mask`。
5. 整体消息再补一个总时间戳；
6. 返回读取结果。

#### 为什么要 `Mutex::LockGuard guard(bus_mutex_)`

这是 C++ RAII 风格的互斥锁保护。

它的意思是：

- 进入函数时自动加锁；
- 函数退出时自动解锁；
- 哪怕中途 `return` 了，也不会忘记开锁。

这比手写：

```cpp
lock();
...
unlock();
```

更安全，因为不容易漏。

### 7.4 采集线程是怎么工作的

`StartAcquisition(50, "imu_data")` 会：

- 创建一个 `Topic("imu_data", sizeof(IMUArrayMsg), ...)`
- 再创建高优先级线程 `IMUMgrAcq`

线程函数 `AcquisitionThreadFunc()` 的节奏是：

```text
计算周期 period_ms = 1000 / frequency_hz
-> 记录上次唤醒时间 last_wakeup
-> while(running_)
   -> ReadAll(msg)
   -> Publish(msg)
   -> SleepUntil(last_wakeup, period_ms)
```

#### 为什么采集线程用 `SleepUntil` 而不是 `Sleep`

这是 RTOS 周期任务里很经典的做法。

如果你每轮都写：

```cpp
读数据
Sleep(20)
```

那每轮实际周期会变成：

```text
读数据耗时 + 20ms
```

周期会慢慢飘。

而 `SleepUntil(last_wakeup, 20)` 的意思是：

> “无论这轮执行花了多少时间，我都尽量在上一个周期基准上对齐下一次。”

这样采样频率更稳。

### 7.5 `Topic("imu_data")` 在这里扮演什么角色

它是 `libxr` 里一个发布-订阅消息总线。

`IMUManager` 是发布者：

- 采完数据后 `Publish(msg)`。

`IMUUartBridgeTask` 是订阅者：

- 用 `ASyncSubscriber<Manager::IMUArrayMsg>("imu_data")` 订阅。

这比“互相拿指针直接调用”更松耦合，因为：

- 采集层不用知道谁在用数据；
- 桥接层也不用自己去拉取底层状态；
- 只关心同一个主题名就行。

---

## 8. 依赖层下钻 2：`IMUUartBridgeTask`

这个类是当前工程的“协议中控台”。

### 8.1 它的职责

它一手抓两边：

- 一边抓 `USART1` 的桥接协议；
- 一边抓本地硬件资源：
  - IMU / I2C
  - SPI
  - WS2812

你可以把它理解成：

> “上位机和板上资源之间的翻译官 + 调度员。”

### 8.2 启动流程

`Start()` 的流程：

1. 检查是否已经在运行；
2. 检查关键指针是否为空；
3. 检查堆空间够不够；
4. `new Thread()`；
5. `thread_->Create(...)`；
6. 线程入口进入 `Run()`。

#### `Thread::Create` 为什么本质上会落到 FreeRTOS 任务

因为 `libxr/system/FreeRTOS/thread.hpp` 里，`Thread::Create` 的底层就是：

```cpp
xTaskCreate(...)
```

所以 `libxr::Thread` 不是另外一套线程系统，而是对 FreeRTOS 任务的一层 C++ 包装。

### 8.3 两种运行模式

`Run()` 里根据配置分两条路：

- `RunStreamMode()`
  - 简单串口文本流模式；
  - 输出 `roll,pitch,yaw` CSV。
- `RunBridgeMode()`
  - 二进制桥接协议模式；
  - 当前工程实际走的是这条。

因为 `stream_relative_euler = false`，所以现在走 `RunBridgeMode()`。

### 8.4 桥接模式主循环

桥接主循环每轮大概做三件事：

1. `PublishBridgeIMUData();`
   - 有新 IMU 数据就主动推送。
2. `PublishBridgeDiag();`
   - 定期主动推送诊断信息。
3. 轮询串口输入；
   - 如果收到一帧命令，就解析并执行。

这说明桥接线程同时承担：

- “主动上报”
- “被动响应”

两类职责。

### 8.5 为什么这里会频繁 `Thread::Sleep(1)`

像 `ReadExact()` 里是这样写的：

```cpp
while (running_) {
  if (uart_->read_port_ != nullptr && uart_->read_port_->Size() >= len) {
    ...
  }
  if ((Thread::GetTime() - start_ms) >= timeout_ms) {
    return false;
  }
  Thread::Sleep(1);
}
```

这是一种“轻量轮询 + 主动让出 CPU”的写法：

- 每次先看看串口缓冲里数据够不够；
- 不够就睡 1ms；
- 不一直死占 CPU。

对新手来说，可以把它理解成：

> 线程每隔 1ms 抬头看一眼门口有没有快递，不是一直站在门口发呆。

### 8.6 `Semaphore` 为什么常用于等传输完成

例如 `WriteExact()`：

```cpp
Semaphore sem(0);
WriteOperation op(sem);
return uart_->Write({buf, len}, op) == ErrorCode::OK;
```

这里的思路是：

1. 创建一个初值为 0 的信号量；
2. 发起 UART 写请求；
3. 底层 DMA / 中断完成后，在回调里把信号量 `Post()`；
4. 发送方等到信号量后继续执行。

它就像：

- 任务说：“我把东西交给搬运工了，搬完你敲门通知我。”
- 中断回调就是那个“敲门”的动作。

这就是信号量最常见的用法之一：**任务和中断之间的同步通知**。

### 8.7 协议处理的关键命令

桥接协议支持的核心命令有：

- `Ping`
  - 测试链路是否通。
- `I2CRead`
  - 读 IMU 寄存器。
- `I2CWrite`
  - 写 IMU 寄存器。
- `SPIWrite`
  - 向 SPI 发数据。
- `WS2812Frame`
  - 直接发一帧 RGB 数据点亮灯带。
- `IMUEulerPush`
  - 主动上报 IMU 数据。
- `IMUDiagPush`
  - 主动上报诊断数据。

### 8.8 为什么桥接访问 I2C 时也要走 `imu_mgr_->AcquireBus()`

因为 IMU 采集线程也在读 I2C。

桥接线程如果直接插进去读写寄存器，就会和采集线程打架。所以桥接里读写 I2C 前，会：

1. `AcquireBus()`
2. 调 `i2c_->MemRead/MemWrite(...)`
3. `ReleaseBus()`

这就是 manager 层的价值：

- 底层 I2C 驱动只负责“怎么读写”；
- manager 层负责“谁在什么时机有资格用总线”。

### 8.9 它如何订阅 IMU 数据

桥接里用的是：

```cpp
new Topic::ASyncSubscriber<Manager::IMUArrayMsg>("imu_data");
```

这个异步订阅者的行为可以简单理解成：

- 先 `StartWaiting()` 表示“我准备好接收新数据了”；
- 一旦发布者 `Publish(msg)`；
- `Topic` 框架会把数据拷贝到订阅者缓冲里；
- 并把状态改成 `DATA_READY`；
- 桥接线程下一轮轮询时发现可用，就拿出来发串口。

这不是消息队列那种“排很多条”，而更像：

> “邮箱里永远只放最新一封信。”

所以它适合高频状态数据，不适合要求每条都不丢的日志流。

---

## 9. 依赖层下钻 3：`WS2812Manager` 和 `WS2812Strip`

这两个类配合得很典型，正好能看出“管理层”和“驱动层”的分工。

### 9.1 `WS2812Manager` 的职责

它主要做“上层管理”：

- 记录这个灯带走哪条 SPI 总线；
- 检查参数是否合法；
- 检查 LED 数量是否超上限；
- 再把真正的数据输出交给底层 `WS2812Strip`。

所以它不直接关心位编码细节。

### 9.2 `WS2812Strip` 的职责

它负责“底层时序适配”：

- 把 RGB 数据编码成 WS2812 需要的特殊波形位流；
- 把编码结果写进 SPI 发送缓冲；
- 再通过 SPI 发出去。

这类模块非常典型：**表面上是“灯带驱动”，本质上是在用 SPI 模拟 WS2812 时序**。

### 9.3 为什么 `spi1_tx_buf` 要这么大

WS2812 每个颜色字节都要编码成更多字节的波形数据。

在 `WS2812Strip` 里：

- 每个颜色字节编码成 10 个字节；
- 每颗 LED 3 个颜色；
- 所以每颗 LED 编码后要 30 个字节；
- 再加前后 reset 区。

因此灯带数据量会比原始 RGB 数据大很多。

所以：

```cpp
spi1_tx_buf[768]
```

不是浪费，而是为了给编码后的发送帧留足空间。

### 9.4 分层思维总结

这里能很清楚地看到：

- `WS2812Strip`
  - 负责“怎么发”；
- `WS2812Manager`
  - 负责“允不允许发、参数对不对、属于哪条总线”；
- `IMUUartBridgeTask`
  - 负责“什么时候响应上位机去发”。

这就是一层层把复杂度拆开的例子。

---

## 10. 依赖层下钻 4：`data_types.hpp`

这个文件很短，但信息量不小。

### 10.1 IMU 槽位定义

它把 4 个 IMU 逻辑位置固定成：

- `Forearm`
- `Hand`
- `ThumbRoot`
- `ThumbTip`

这说明当前工程不是“动态发现后任意编号”，而是“按固定物理部位建模”。

### 10.2 地址映射

`ResolveImuI2CAddress(index, base_address)` 做的是：

- 第 0 个 IMU 用 `base_address`
- 第 1 个 `base + 1`
- 第 2 个 `base + 2`
- 第 3 个 `base + 3`

这就是为什么 `IMUManager` 初始化时只给一个基地址，剩下几个地址框架自己推出来。

### 10.3 `IMUData`

单个 IMU 数据里包含：

- 加速度 `acc`
- 角速度 `gyro`
- 欧拉角 `angle`
- 磁场 `mag`
- 温度 `temperature`
- 四元数 `quaternion`
- 时间戳 `timestamp_us`

这说明桥接协议上送的不只是姿态角，还有完整惯导状态。

### 10.4 `IMUArrayMsg`

这是主题 `imu_data` 的消息体：

- `timestamp_us`
  - 整包时间戳
- `sequence`
  - 递增序号
- `valid_mask`
  - 哪几个 IMU 当前有效
- `imu_data[ACTUAL_IMU_COUNT]`
  - 4 个槽位的数据

这就是采集线程和桥接线程之间的“统一数据协议”。

---

## 11. libxr 框架知识点标注：当前代码到底用了哪些模块

下面按模块归类。

### 11.1 `driver/st`

当前工程实际走到的有：

- `STM32TimerTimebase`
  - 用 TIM1 给 `libxr` 提供时间基准。
- `STM32PowerManager`
  - 电源管理抽象对象。
- `STM32GPIO`
  - GPIO 包装。
- `STM32SPI`
  - SPI HAL 句柄包装。
- `STM32UART`
  - UART HAL 句柄包装。
- `STM32I2C`
  - I2C HAL 句柄包装。

这一层的定位是：

> “把不同芯片平台的外设操作，统一收口成 `libxr` 的驱动接口。”

### 11.2 `system/FreeRTOS`

当前实际用到：

- `PlatformInit`
- `Thread`
- `Semaphore`
- `Mutex`

这层的定位是：

> “把 FreeRTOS 的任务、信号量、互斥锁、时间相关接口，包装成统一系统抽象。”

这样上层代码写的是：

- `Thread::Create`
- `Thread::Sleep`
- `Semaphore`
- `Mutex`

而不是直接到处写：

- `xTaskCreate`
- `vTaskDelay`
- `xSemaphoreTake`
- `xSemaphoreGive`

### 11.3 `src/system`

当前实际用到：

- `Timer`

这层是 `libxr` 自己的一套轻量周期任务调度器。

### 11.4 `src/middleware`

当前实际用到：

- `Topic`
- `Topic::ASyncSubscriber`
- `Terminal`
- `RamFS`
- `STDIO`

这一层已经不是“驱动”，而是“中间件”了：

- `Topic` 解决模块之间传消息；
- `Terminal` 解决交互式命令行；
- `RamFS` 解决内存文件系统；
- `STDIO` 解决“标准输入输出挂到哪个端口上”。

---

## 12. RTOS 相关知识，用新手能懂的话讲

下面把这份代码里涉及到的 RTOS 知识点，按“代码出现在哪里 -> 它本质是什么”来解释。

### 12.1 任务（Task）

代码里有这些任务：

- `defaultTask`
- `IMUMgrAcq`
- `IMUBridge`
- `libxr_timer_task`

新手可以把任务理解成：

> “RTOS 里的一个独立工作线程，有自己的栈、自己的执行位置，可以被调度器暂停和恢复。”

在你这个工程里：

- `defaultTask` 负责跑 `app_main()`；
- `IMUMgrAcq` 负责周期采集 IMU；
- `IMUBridge` 负责串口桥接协议；
- `libxr_timer_task` 负责周期执行 Terminal 这样的软定时任务。

### 12.2 调度（Scheduling）

FreeRTOS 调度器做的事很像“排班”：

- 哪个任务现在最该运行；
- 哪个任务该先暂停；
- 哪个任务睡够了该醒来；
- 哪个任务等到信号量了该继续跑。

你当前工程开启了抢占式调度：

- `configUSE_PREEMPTION = 1`

意思是：

- 高优先级任务就绪后，可以抢占低优先级任务。

### 12.3 队列（Queue）

虽然你业务层没显式写 FreeRTOS 队列，但 `libxr` 的端口和消息系统内部大量用了队列思想。

例如：

- `USART1` 收到的数据先进入读缓冲/队列；
- `STM32UART` 的写请求会先进入写队列；
- `Topic` 的某些订阅模式本质上也是在转发数据。

可以简单理解为：

> 队列就是“先来的消息先排队”，让生产者和消费者不必同一时刻对上。

### 12.4 信号量（Semaphore）

代码中它最常见的用途是：

- “我发起了一次 DMA 传输”；
- “先别往下走，等中断回调告诉我完成”。

所以它更像一个“通知铃”：

- 任务先去等铃响；
- 中断处理完后把铃摇一下；
- 任务醒来继续执行。

### 12.5 互斥锁（Mutex）

`IMUManager` 的 `bus_mutex_` 就是标准案例。

它不是用来传消息的，而是用来保护共享资源：

- 共享资源这里就是 `I2C1` 总线。

它更像“门钥匙”：

- 谁拿到钥匙谁进去；
- 其他人先在门外等。

### 12.6 中断（Interrupt）

当前工程里，中断主要服务于：

- `TIM1`：维护 HAL tick；
- `DMA`：SPI / UART / I2C 传输完成通知；
- `USART1` / `UART5` / `SPI1` / `I2C1`：外设事件回调。

中断的特点是：

- 来得突然；
- 优先级高；
- 要求处理快。

所以常见做法不是“在中断里干完整业务”，而是：

1. 中断里只做最小动作；
2. 更新状态/发信号量；
3. 让任务线程去做后续重活。

这也是你工程里 `DMA + 中断 + 任务` 配合的本质。

### 12.7 内存（Heap / Stack）

代码里多次出现：

- `xPortGetFreeHeapSize()`
- `new Thread()`
- `new Topic(...)`
- `new Topic::ASyncSubscriber(...)`

RTOS 下要分清两种内存：

- **堆（Heap）**
  - 动态分配用；
  - `new`、`pvPortMalloc` 都会从这里拿。
- **栈（Stack）**
  - 每个任务自己的一块工作空间；
  - 放局部变量、函数调用现场。

为什么代码里建任务前先查堆？

因为任务栈通常就是从堆里分出来的。如果堆不够，任务就建不起来。

### 12.8 时钟（Tick / Timebase）

当前工程的时间相关接口很多，但新手先抓住两点：

1. `Thread::Sleep(ms)`
   - 依赖 FreeRTOS tick；
   - 用来做任务级延时。
2. `Timebase::GetMicroseconds()`
   - 依赖 `STM32TimerTimebase`；
   - 用来给 IMU 数据打更细的时间戳。

前者偏“调度”，后者偏“测量”。

---

## 13. UART / SPI / I2C 在当前工程里怎么和 DMA / 中断配合

### 13.1 UART

`STM32UART` 在构造 `USART1` 时会：

- 开启 RX DMA 循环接收；
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA(...)`；
- 在接收事件回调里，把 DMA 新收到的数据推到 `read_port_`；
- 写数据时优先走 `HAL_UART_Transmit_DMA(...)`；
- 发送完成后通过回调更新写操作状态。

所以 `USART1` 的读写都不是简单阻塞轮询，而是“DMA 搬运 + 回调通知”的模式。

### 13.2 I2C

`STM32I2C` 的逻辑是：

- 小包数据：直接同步读写；
- 大于阈值的数据：优先走 DMA；
- DMA 完成后，在 `HAL_I2C_*CpltCallback` 里更新操作状态；
- 阻塞调用方通过信号量等完成。

所以你会看到业务层只写：

```cpp
i2c_->MemRead(...)
```

但底下其实可能已经是 DMA 在跑。

### 13.3 SPI

`STM32SPI` 也是类似思路：

- 数据小：直接同步传输；
- 数据大：走 DMA；
- WS2812 正是吃这套机制的典型场景，因为它要推一大块编码后的数据帧。

---

## 14. 文字版执行链 + 资源关系总表

### 14.1 最终执行链

```text
上电
-> HAL 初始化
-> 外设初始化
-> FreeRTOS 启动
-> defaultTask 进入 app_main
-> 建立 libxr 时间基准和平台层
-> 包装 HAL 外设为 libxr 驱动对象
-> USART1 挂到 STDIO，启动终端软定时任务
-> 初始化 WS2812 管理器
-> 初始化 IMU 管理器
-> IMUManager 创建采集线程，周期发布 imu_data
-> 创建 IMUUartBridgeTask 桥接线程
-> 桥接线程订阅 imu_data，并经 USART1 推送/响应协议帧
-> UART5 只负责打印启动日志与诊断日志
```

### 14.2 关键资源表

| 资源 | 当前用途 | 谁在用 |
| --- | --- | --- |
| `TIM1` | HAL tick + libxr 时间基准 | HAL / `STM32TimerTimebase` |
| `USART1` | 终端 + 桥接协议 | `Terminal` / `IMUUartBridgeTask` |
| `UART5` | 启动日志 | `app_main.cpp` 里的辅助函数 |
| `I2C1` | IMU 寄存器访问 | `IMUManager` / `IMUUartBridgeTask` |
| `SPI1` | WS2812 数据输出 | `WS2812Strip` / `IMUUartBridgeTask` |
| `Topic("imu_data")` | IMU 数据总线 | `IMUManager` 发布，桥接任务订阅 |
| `bus_mutex_` | I2C 总线互斥 | `IMUManager` / `IMUUartBridgeTask` |

### 14.3 关键周期

- IMU 采集周期：`1000 / 50 = 20ms`
- 桥接 IMU 推送周期：`20ms`
- 桥接诊断推送周期：`1000ms`
- Terminal 软定时任务周期：`10ms`

### 14.4 关键阻塞点

- UART5 启动日志：阻塞式 HAL 发送
- I2C / SPI / UART 业务传输：对调用方而言常常是“发起异步 + 等待完成”
- 任务睡眠：`Thread::Sleep` / `SleepUntil`

---

## 15. 最后给新手的阅读建议

如果你准备继续深入这份工程，最推荐按这个顺序往下看：

1. 先把 `main.c` 和 `freertos.c` 看懂，弄清楚谁创建谁。
2. 再反复读 `app_main.cpp`，把“装配逻辑”吃透。
3. 然后读 `IMUManager`，理解“资源管理 + 周期采集 + Topic 发布”。
4. 再读 `IMUUartBridgeTask`，理解“协议处理 + 订阅 + 总线仲裁”。
5. 最后再回头看 `libxr` 的 `Thread`、`Semaphore`、`Topic`、`Timer`，你会更容易把抽象和业务代码对上。

如果只记一句话，我建议记这句：

> 这套工程不是“一个大 while 循环在干活”，而是“`app_main()` 把外设、框架和业务任务装配好以后，真正的工作交给多个 FreeRTOS 任务并发完成”。

