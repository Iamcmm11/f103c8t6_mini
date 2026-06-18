# STM32F105RCT6 MCU 引脚规划表

整理日期：2026-06-17

整理依据：

- `f105rct6.ioc`
- `Core/Src/gpio.c`
- `Core/Src/i2c.c`
- `Core/Src/spi.c`
- `Core/Src/usart.c`
- `Core/Src/tim.c`
- `Core/Src/can.c`
- `Core/Src/dma.c`
- `Core/Src/main.c`
- `User/app_main.cpp`

## 1. MCU 与时钟概览

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F105RCT6 |
| 封装 | LQFP64 |
| SYSCLK/HCLK | 64 MHz |
| APB1 | 32 MHz，APB1 timer clock = 64 MHz |
| APB2 | 64 MHz |
| HSE | 使用外部高速晶振，`PD0-OSC_IN` / `PD1-OSC_OUT` |
| LSE | `.ioc` 中占用 `PC14` / `PC15`，但当前 `SystemClock_Config()` 未启用 LSE |
| 调试口 | SWD：`PA13 SWDIO` / `PA14 SWCLK` |
| JTAG | 已关闭 JTAG，仅保留 SWD；`PA15/PB3/PB4` 释放为 GPIO |

## 2. 外设与引脚规划

| 外设/功能 | 引脚 | 信号/方向 | GPIO 模式 | 关键配置 | 当前用途/备注 |
| --- | --- | --- | --- | --- | --- |
| RCC HSE | `PD0` | `OSC_IN` | 外部晶振脚 | HSE ON，PLL x8 | 系统时钟源 |
| RCC HSE | `PD1` | `OSC_OUT` | 外部晶振脚 | HSE ON，PLL x8 | 系统时钟源 |
| RCC LSE | `PC14` | `OSC32_IN` | 外部 32.768 kHz 晶振脚 | `.ioc` 占用 | 当前生成代码未启用 LSE |
| RCC LSE | `PC15` | `OSC32_OUT` | 外部 32.768 kHz 晶振脚 | `.ioc` 占用 | 当前生成代码未启用 LSE |
| SYS/SWD | `PA13` | `SWDIO` | Serial Wire | SWD enabled | 下载/调试 |
| SYS/SWD | `PA14` | `SWCLK` | Serial Wire | SWD enabled | 下载/调试 |
| I2C1 | `PB6` | `I2C1_SCL` | AF Open-Drain，高速 | 400 kHz Fast Mode，DMA TX/RX，中断 | WIT/JY901B IMU、YIS IMU、桥接调试读写 |
| I2C1 | `PB7` | `I2C1_SDA` | AF Open-Drain，高速 | 400 kHz Fast Mode，DMA TX/RX，中断 | WIT/JY901B IMU、YIS IMU、桥接调试读写 |
| SPI1 | `PA5` | `SPI1_SCK` | AF Push-Pull，高速 | Master，8-bit，CPOL=0，CPHA=1Edge，Prescaler=8，TX DMA | WS2812 波形输出、桥接 SPI write |
| SPI1 | `PA7` | `SPI1_MOSI` | AF Push-Pull，高速 | 约 8 Mbit/s，软件 NSS | WS2812 数据输出 |
| USART1 | `PA9` | `USART1_TX` | AF Push-Pull，高速 | 460800, 8N1，TX DMA | 主业务桥接串口 |
| USART1 | `PA10` | `USART1_RX` | Input Pull-Up | 460800, 8N1，RX DMA | 主业务桥接串口 |
| UART5 | `PC12` | `UART5_TX` | AF Push-Pull，高速 | 115200, 8N1，中断，无 DMA | boot/debug 日志输出 |
| UART5 | `PD2` | `UART5_RX` | Input No-Pull | 115200, 8N1，中断，无 DMA | 预留接收 |
| CAN2 | `PB12` | `CAN2_RX` | Input No-Pull | 250 kbit/s，Normal Mode，中断 | FEYMAN MCS10 CANopen |
| CAN2 | `PB13` | `CAN2_TX` | AF Push-Pull，高速 | Prescaler=8，BS1=13TQ，BS2=2TQ，ABOM=Enable | FEYMAN MCS10 CANopen |
| TIM2 | `PA1` | `TIM2_CH2` | AF Push-Pull，低速 | PSC=6399，ARR=9999，当前 CCR2=0 | 注册为 `TIM2_IMU_SYNC_1HZ` 同步输出 |
| TIM2 | `PB10` | `TIM2_CH3` | AF Push-Pull，低速 | PSC=6399，ARR=9999，当前 CCR3=10 | 已配置 PWM，但当前应用层未注册使用 |
| TIM5 | `PA0` | `TIM5_CH1` | AF Push-Pull，低速 | PSC=639，ARR=3332，CCR1=100 | 注册为 `TIM5_CAMERA_TRIGGER_30HZ` 相机触发 |
| TIM6 | 无外部引脚 | Base Timer | 无 | PSC=63，ARR=19999 | WIT IMU 50 Hz 硬件采集触发 |
| TIM1 | 无外部引脚 | HAL timebase | 无 | 1 ms tick | HAL tick，LibXR timebase |
| GPIO | `PA4` | Output | Push-Pull，No-Pull，低速，初始 Low | 无 | 构造为 `STM32GPIO PA4`，当前未见业务绑定，疑似预留控制/片选 |
| GPIO/EXTI | `PA6` | EXTI6 input | Rising Edge，No-Pull | `EXTI9_5_IRQn` | 构造为 `STM32GPIO PA6`，当前未见回调绑定 |
| GPIO/EXTI | `PB8` | EXTI8 input | Rising Edge，No-Pull | `EXTI9_5_IRQn` | YIS IMU DR/Data Ready 中断输入 |
| GPIO Button | `PC10` | Input | Pull-Up | 5 ms 扫描，30 ms 消抖 | 按键命令 `A`，经 USART1 桥接上报 |
| GPIO Button | `PA15` | Input | Pull-Up | 5 ms 扫描，30 ms 消抖 | 按键命令 `B`，JTAG 关闭后复用 |
| GPIO Button | `PB3` | Input | Pull-Up | 5 ms 扫描，30 ms 消抖 | 按键命令 `C`，JTAG 关闭后复用 |
| GPIO Button | `PB4` | Input | Pull-Up | 5 ms 扫描，30 ms 消抖 | 按键命令 `D`，JTAG 关闭后复用 |

## 3. 按引脚速查

| 引脚 | 当前功能 | 备注 |
| --- | --- | --- |
| `PA0-WKUP` | `TIM5_CH1` | 30 Hz 相机触发 PWM |
| `PA1` | `TIM2_CH2` | 1 Hz IMU/YIS 同步输出，当前 CCR2=0 |
| `PA4` | GPIO Output | 初始 Low，当前未见业务绑定 |
| `PA5` | `SPI1_SCK` | WS2812/SPI 输出时钟 |
| `PA6` | `EXTI6` | Rising edge，当前未见业务回调 |
| `PA7` | `SPI1_MOSI` | WS2812/SPI 输出数据 |
| `PA9` | `USART1_TX` | 主桥接串口 TX |
| `PA10` | `USART1_RX` | 主桥接串口 RX，上拉 |
| `PA13` | `SWDIO` | 调试下载 |
| `PA14` | `SWCLK` | 调试下载 |
| `PA15` | GPIO Input | 按键 `B`，上拉 |
| `PB3` | GPIO Input | 按键 `C`，上拉 |
| `PB4` | GPIO Input | 按键 `D`，上拉 |
| `PB6` | `I2C1_SCL` | IMU/YIS I2C 总线 |
| `PB7` | `I2C1_SDA` | IMU/YIS I2C 总线 |
| `PB8` | `EXTI8` | YIS DR/Data Ready |
| `PB10` | `TIM2_CH3` | 已配置 PWM，当前应用层未注册 |
| `PB12` | `CAN2_RX` | FEYMAN CANopen |
| `PB13` | `CAN2_TX` | FEYMAN CANopen |
| `PC10` | GPIO Input | 按键 `A`，上拉 |
| `PC12` | `UART5_TX` | boot/debug 日志 |
| `PC14` | `OSC32_IN` | `.ioc` 占用，当前代码未启用 LSE |
| `PC15` | `OSC32_OUT` | `.ioc` 占用，当前代码未启用 LSE |
| `PD0` | `OSC_IN` | HSE |
| `PD1` | `OSC_OUT` | HSE |
| `PD2` | `UART5_RX` | debug 串口 RX 预留 |

## 4. DMA 资源

| DMA 通道 | 绑定外设 | 方向 | 优先级 | NVIC 优先级 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `DMA1_Channel3` | `SPI1_TX` | Memory -> Peripheral | Low | 6 | WS2812/SPI 发送 |
| `DMA1_Channel4` | `USART1_TX` | Memory -> Peripheral | Low | 5 | 主桥接串口发送 |
| `DMA1_Channel5` | `USART1_RX` | Peripheral -> Memory | Low | 5 | 主桥接串口接收 |
| `DMA1_Channel6` | `I2C1_TX` | Memory -> Peripheral | Very High | 5 | I2C 写 |
| `DMA1_Channel7` | `I2C1_RX` | Peripheral -> Memory | High | 5 | I2C 读 |

## 5. 中断资源

| IRQ | 优先级 | 用途 |
| --- | --- | --- |
| `EXTI9_5_IRQn` | 5 | `PA6 EXTI6`、`PB8 EXTI8` |
| `I2C1_EV_IRQn` | 5 | I2C1 event |
| `I2C1_ER_IRQn` | 5 | I2C1 error |
| `SPI1_IRQn` | 5 | SPI1 |
| `USART1_IRQn` | 5 | USART1 |
| `UART5_IRQn` | 5 | UART5 |
| `CAN2_TX_IRQn` | 5 | CAN2 TX |
| `CAN2_RX0_IRQn` | 5 | CAN2 RX FIFO0 |
| `CAN2_RX1_IRQn` | 5 | CAN2 RX FIFO1 |
| `CAN2_SCE_IRQn` | 5 | CAN2 status/error |
| `TIM2_IRQn` | 5 | 1 Hz 同步事件记录 |
| `TIM5_IRQn` | 5 | 30 Hz 相机触发事件记录 |
| `TIM6_IRQn` | 5 | 50 Hz WIT IMU 采集触发 |
| `TIM1_UP_IRQn` | 15 | HAL 1 ms tick |
| `DMA1_Channel3_IRQn` | 6 | SPI1 TX DMA |
| `DMA1_Channel4_IRQn` | 5 | USART1 TX DMA |
| `DMA1_Channel5_IRQn` | 5 | USART1 RX DMA |
| `DMA1_Channel6_IRQn` | 5 | I2C1 TX DMA |
| `DMA1_Channel7_IRQn` | 5 | I2C1 RX DMA |

## 6. 业务连接关系

| 业务模块 | 底层外设/引脚 | 说明 |
| --- | --- | --- |
| WIT/JY901B IMU 阵列 | `I2C1 PB6/PB7` | 默认地址 `0x50` ~ `0x55`，当前业务启用 6 路 |
| YIS IMU | `I2C1 PB6/PB7` + `PB8 EXTI8` | I2C 地址 `0x6A`，`PB8` 用作 DR/Data Ready |
| FEYMAN MCS10 | `CAN2 PB12/PB13` | CANopen，默认节点 `0x7F`，250 kbit/s |
| WS2812 灯带 | `SPI1 PA5/PA7` | 使用 SPI 编码 WS2812 时序，当前 16 颗 LED |
| 主机桥接协议 | `USART1 PA9/PA10` | 460800 8N1，二进制桥接协议 |
| boot/debug 日志 | `UART5 PC12/PD2` | 当前主要用 `PC12 TX` 阻塞发送日志 |
| IMU/YIS 同步脉冲 | `TIM2_CH2 PA1` | 目标 1 Hz，同步事件 source=1 |
| 相机触发脉冲 | `TIM5_CH1 PA0` | 约 30 Hz，同步事件 source=2 |
| WIT 采集触发 | `TIM6` | 50 Hz，无外部引脚 |
| GPIO 按键 | `PC10/PA15/PB3/PB4` | 上报命令 `A/B/C/D` |

## 7. 注意事项

1. `TIM2_CH2 / PA1` 当前被应用层注册为 `TIM2_IMU_SYNC_1HZ`，但 `Core/Src/tim.c` 里 `TIM_CHANNEL_2` 的 `Pulse` 为 `0`。如果需要外部真实 1 Hz 脉冲，应确认是否要把 `CCR2` 改成非零值。
2. `TIM2_CH3 / PB10` 当前配置了 `Pulse=10`，并构造了 `STM32PWM pwm_tim2_ch3`，但没有注册到 `SyncSignalManager`，因此默认业务不会启用它。
3. `SPI1` 当前只初始化了 `SCK/MOSI`，没有配置 `MISO/NSS` 引脚；`NSS` 为软件管理，当前业务也是只写链路。
4. `PA4` 是普通输出并构造了 GPIO 对象，但当前 `STM32SPI` 传入的片选为 `{nullptr, 0}`，因此它不是现有 SPI 驱动的自动片选。
5. `PC14/PC15` 在 `.ioc` 里被 LSE 占用，但生成的 `SystemClock_Config()` 未开启 LSE；如果板上没有 32.768 kHz 晶振，可以考虑在 CubeMX 中释放。
6. `CAN2` 初始化时同时打开 `CAN1` 时钟，这是 STM32F105 双 CAN 的正常要求；当前没有启用 CAN1 引脚或 CAN1 收发逻辑。
