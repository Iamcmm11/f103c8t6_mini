# USART1 协议总览

这份文档面向第一次接手本项目的人，目标是回答三件事：

1. `USART1` 现在到底拿来干什么。
2. 当前真正跑在板子上的协议是什么。
3. 如果要接收、调试、扩展，应该从哪里下手。

---

## 1. 先说结论

当前项目里，`USART1` 是主业务串口，默认跑的是 **二进制桥接协议**，不是裸文本，不是 shell，也不是 `printf` 日志口。

当前默认形态可以概括成：

- 物理口：`PA9=USART1_TX`，`PA10=USART1_RX`
- 串口参数：`460800`, `8N1`
- 驱动方式：DMA + 中断
- 当前业务模式：`IMUUartBridgeTask::RunBridgeMode()`
- 当前主要上行数据：
  - `0x30` IMU/YIS 姿态推送
  - `0x32` TIM2/TIM5 同步事件推送
  - `0x33` GPIO 按钮事件推送
- 当前主要下行控制：
  - `0x01` ping
  - `0x02` time sync
  - `0x03` start/stop stream
  - `0x21` WS2812 灯控
  - `0x10/0x11/0x20` 低层 I2C/SPI 调试命令

一个非常重要的现实点：

- `UART5` 才是当前 boot 日志口。
- `USART1` 不应该再被当成“看日志的串口”。
- 对端如果要接这个口，应该按桥协议帧解析，不要按裸字节流解析。

---

## 2. 相关代码入口

接手时优先看这几个文件：

- `User/app_main.cpp`
  - 创建 `STM32UART usart1`
  - 配置 `IMUUartBridgeTask`
  - 选择当前是桥模式还是文本流模式
- `application/imu_uart_bridge_task.hpp`
  - 桥协议配置项
- `application/imu_uart_bridge_task.cpp`
  - 协议解析、命令处理、推送打包
- `utils/python/imu_uart_bridge_test.py`
  - 当前主机侧参考实现
- `managers/gpio_manager.hpp/.cpp`
  - 按钮事件来源

---

## 3. USART1 的硬件与驱动参数

### 3.1 物理配置

- `TX`: `PA9`
- `RX`: `PA10`
- `RX` 配了上拉

### 3.2 串口配置

- 波特率：`460800`
- 数据位：`8`
- 校验：`None`
- 停止位：`1`

### 3.3 DMA / 中断

- `USART1_TX` -> `DMA1_Channel4`
- `USART1_RX` -> `DMA1_Channel5`
- `USART1_IRQn` 已启用

### 3.4 软件缓冲

当前 `usart1` 相关缓冲配置：

- TX buffer: `2048`
- RX buffer: `512`
- TX queue size: `5`

---

## 4. 当前实际启用的两种 USART1 模式

`IMUUartBridgeTask` 有两种工作模式：

### 4.1 桥模式：当前默认

由 `bridge_config.stream_relative_euler = false` 进入：

- 入口：`RunBridgeMode()`
- 特征：所有数据都按二进制帧收发
- 支持命令 + 主动推送
- 这是当前项目实际在用的模式

### 4.2 文本流模式：历史/调试模式

由 `bridge_config.stream_relative_euler = true` 进入：

- 入口：`RunStreamMode()`
- 输出格式：
  - 首行：`roll_deg,pitch_deg,yaw_deg\r\n`
  - 后续每行：`%.3f,%.3f,%.3f\r\n`
- 只输出第一路 WIT IMU 的欧拉角
- 没有桥协议帧
- 不适合当前 NV 侧桥接脚本

接手时不要把这两种模式混起来。当前项目应该默认按桥模式理解。

---

## 5. 当前桥协议的帧格式

所有桥协议帧统一格式如下：

```text
SOF0   1 byte   0x55
SOF1   1 byte   0xAA
CMD    1 byte
LEN    2 bytes  little-endian
PAYLOAD LEN bytes
SUM    1 byte   (SOF0 + SOF1 + CMD + LEN0 + LEN1 + PAYLOAD...) & 0xFF
```

### 5.1 例子

`ping` 请求没有 payload，因此整帧是：

```text
55 AA 01 00 00 00
```

解释：

- `55 AA`: 帧头
- `01`: `CMD_PING`
- `00 00`: payload 长度 0
- `00`: 校验和

### 5.2 几个协议约束

- 长度字段是小端
- 校验和不是 CRC，就是简单字节求和
- 当前主机脚本把 `1024` 作为最大 payload 长度
- MCU 收到未知 `cmd` 时，会用**相同 `cmd`** 回一个 1 字节错误状态

---

## 6. 命令总表

| CMD | 方向 | 含义 | 当前状态 |
| --- | --- | --- | --- |
| `0x01` | Host -> MCU -> Host | Ping | 在用 |
| `0x02` | Host -> MCU -> Host | 时间同步 | 在用 |
| `0x03` | Host -> MCU -> Host | 开始/停止流 | 在用 |
| `0x10` | Host -> MCU -> Host | I2C 读 | 保留调试 |
| `0x11` | Host -> MCU -> Host | I2C 写 | 保留调试 |
| `0x20` | Host -> MCU -> Host | SPI 写 | 保留调试 |
| `0x21` | Host -> MCU -> Host | WS2812 灯控 | 在用 |
| `0x30` | MCU -> Host | IMU/YIS 推送 | 在用 |
| `0x31` | MCU -> Host | IMU 诊断推送 | Host 兼容，MCU 当前未发送 |
| `0x32` | MCU -> Host | 同步事件推送 | 在用 |
| `0x33` | MCU -> Host | GPIO 按钮事件推送 | 在用 |

---

## 7. 下行命令详解

### 7.1 `0x01 CMD_PING`

请求：

```text
payload = empty
```

响应：

```text
u8 status
```

约定：

- `0` = OK
- `1` = ERROR

---

### 7.2 `0x02 CMD_TIME_SYNC`

用途：

- 建立 MCU 时间轴到上位机时间轴的映射
- 是 `capture`、`sync-monitor`、双同步分析的基础

请求 payload：

```text
<I seq>
```

响应 payload：

```text
<B I Q Q>
status
seq
t2_mcu_tick_us
t3_mcu_tick_us
```

说明：

- `seq` 原样回显
- `t2/t3` 是 MCU 侧时间戳
- 主机侧脚本用四时间戳法生成 `offset_us`
- 这里的 `mcu_tick_us` 不是单纯裸硬件时间，而是桥协议统一使用的 MCU 会话时间轴

---

### 7.3 `0x03 CMD_STREAM_CONTROL`

用途：

- 控制 MCU 侧是否开始持续推送姿态和同步事件

当前主机侧发送格式：

```text
payload = <I seq> + b"start"
payload = <I seq> + b"stop"
```

MCU 还兼容历史格式：

```text
payload = b"start"
payload = b"stop"
```

响应 payload：

```text
<B I B>
status
request_id
active
```

语义：

- `active = 1` 表示流处于开启态
- `active = 0` 表示流处于关闭态

重要副作用：

- `start` 不只是“允许发包”
- 它还会启动已注册的同步输出
  - `TIM2_IMU_SYNC_1HZ`
  - `TIM5_CAMERA_TRIGGER_30HZ`
- `stop` 会停止这些同步输出并清空 pending 数据

因此，`0x03` 是这个系统的“会话开关”。

---

### 7.4 `0x10 CMD_I2C_READ`

请求 payload：

```text
u8 bus
u8 addr
u8 reg
u8 count
```

响应 payload：

```text
u8 status
raw_data[count * 2]
```

注意：

- 这里的 `count` 表示按当前实现读取 `count * 2` 个字节
- 它更像是“读若干个 16-bit 寄存器”
- 当前实现只允许：
  - `bus == config_.i2c_bus`
  - `addr == config_.imu_addr`

在当前默认配置里，这通常对应老的 WIT 访问路径，不是任意 I2C 透传。

---

### 7.5 `0x11 CMD_I2C_WRITE`

请求 payload：

```text
u8 bus
u8 addr
u8 reg
u8 data0
u8 data1
```

响应 payload：

```text
u8 status
```

说明：

- 当前一次写 2 字节
- 同样只允许写当前配置里的目标 IMU 地址

---

### 7.6 `0x20 CMD_SPI_WRITE`

请求 payload：

```text
u8  bus
u8  reserved
u16 spi_len
u8  data[spi_len]
```

响应 payload：

```text
u8 status
```

说明：

- 当前是“只写 SPI”，没有读回
- `payload[1]` 目前 MCU 侧没有使用，按保留字节理解
- 更像 bring-up/debug 接口，不是主业务链路

---

### 7.7 `0x21 CMD_WS2812_CONTROL`

请求 payload：

```text
<B B B B B H>
target
flags
red
green
blue
interval_ms
```

字段说明：

- `target`
  - `0xFF` = 全部灯
  - 其他值 = 指定灯索引
- `flags`
  - bit0 = blink enable
- `interval_ms`
  - 闪烁周期参数

响应 payload：

```text
u8 status
```

---

## 8. 上行推送详解

### 8.1 `0x30 CMD_IMU_EULER_PUSH`

这是最容易被误解的一类，因为 Python 侧兼容了多种历史格式。

对新接手者来说，只需要记住：

- **当前工程默认真正发送的是 YIS 扩展格式**
- 如果以后切回 `pose_source = WIT`，则会发送 WIT 压缩格式

#### 当前默认：YIS 扩展格式

当前 `app_main.cpp` 里 `pose_source = YIS`，因此 payload 实际上是：

```text
u8 count
repeated count times:
  u8    imu_addr
  float roll_deg
  float pitch_deg
  float yaw_deg
  float quat_w
  float quat_x
  float quat_y
  float quat_z
  u32   sample_timestamp
  u64   sensor_mcu_tick_us
  u64   readout_mcu_tick_us
  u8    time_status
```

当前默认只有 1 条记录，`imu_addr` 通常是 `0x6A`。

字段理解：

- `sample_timestamp`
  - YIS 自身的周期内时间戳
- `sensor_mcu_tick_us`
  - 统一到 MCU 时间轴后的正式采样时间
  - 上位机最终应优先用它做时间对齐
- `readout_mcu_tick_us`
  - DR/读出附近时间，主要用于诊断
- `time_status`
  - 更细的含义建议同时看 `docs/YIS_Cam双同步.md`

#### 可选：WIT 压缩格式

如果以后把 `pose_source` 改回 `WIT`，则 payload 会变成：

```text
u8 count
u64 base_tick_us
repeated count times:
  u8  imu_addr
  u16 tick_delta_us
  i16 acc_x_mg
  i16 acc_y_mg
  i16 acc_z_mg
  i16 quat_w_q15
  i16 quat_x_q15
  i16 quat_y_q15
  i16 quat_z_q15
  i16 mag_x
  i16 mag_y
  i16 mag_z
```

解释：

- 每条记录的实际时间：
  - `mcu_tick_us = base_tick_us + tick_delta_us`
- 加速度单位是 `mg`
- 四元数是 `q15`
- 这个格式更紧凑，适合多路 WIT

#### Host 侧还兼容哪些历史格式

`utils/python/imu_uart_bridge_test.py` 还保留了旧格式兼容：

- 纯欧拉角
- 欧拉角 + 四元数
- WIT 旧 pose 格式
- YIS 旧 pose 格式
- 更老的扩展浮点格式

但这些只是“能解析”，不是“当前 MCU 还会发”。

---

### 8.2 `0x31 CMD_IMU_DIAG_PUSH`

Host 脚本支持解析：

```text
<I H H B B B>
seq
online_mask
valid_mask
online_count
valid_count
capacity
```

但需要明确：

- 当前 `application/imu_uart_bridge_task.cpp` 没有发送这个包
- 所以它现在更像是历史兼容入口

---

### 8.3 `0x32 CMD_SYNC_EVENT_PUSH`

payload：

```text
u8 count
repeated count times:
  u8  source
  u8  flags
  u16 reserved
  u32 sequence
  u64 mcu_tick_us
  u32 nominal_period_us
  u32 dropped_count
```

当前 source 映射：

- `1` -> `TIM2_IMU_SYNC_1HZ`
- `2` -> `TIM5_CAMERA_TRIGGER_30HZ`

理解方式：

- 这是 MCU 给上位机的“同步基准事件”
- 用来把 IMU 时间和相机触发时间放到同一条 MCU 时间轴上

注意：

- `0x32` 只有在 `streaming_enabled = true` 时才会推送
- 它受 `CMD_STREAM_CONTROL start/stop` 控制

---

### 8.4 `0x33 CMD_GPIO_BUTTON_PUSH`

payload：

```text
u8 ascii_command
```

当前默认映射：

- `PC10 -> 'A'`
- `PA15 -> 'B'`
- `PB3  -> 'C'`
- `PB4  -> 'D'`

这个事件的特点：

- 不是请求响应型命令
- 是 MCU 主动 push
- **不受 `stream start/stop` 约束**
  - 只要 bridge 任务在跑，按键事件就可以直接发出

这点和 `0x30/0x32` 不一样，接手时很容易搞混。

---

## 9. 当前工程“实际上会看到什么”

如果你拿当前工程直接上板，并且按默认配置运行，`USART1` 上最可能看到的是：

1. 上位机先发 `0x03 start`
2. MCU 返回 start ACK
3. MCU 周期性推送：
   - `0x30` YIS pose
   - `0x32` sync events
4. 按键触发时，额外收到：
   - `0x33` gpio button push
5. 上位机周期性发：
   - `0x02` time sync
6. 灯控时会有：
   - `0x21` request/response

反过来说，当前默认**不会**看到：

- shell 终端提示符
- 纯文本 CSV 姿态流
- `printf` 风格业务日志
- `0x31` IMU diag push

---

## 10. Host 侧参考脚本

当前主机侧标准参考是：

- `utils/python/imu_uart_bridge_test.py`

常用命令：

```bash
python utils/python/imu_uart_bridge_test.py --port COM13 ping
python utils/python/imu_uart_bridge_test.py --port COM13 start
python utils/python/imu_uart_bridge_test.py --port COM13 stop
python utils/python/imu_uart_bridge_test.py --port COM13 console
python utils/python/imu_uart_bridge_test.py --port COM13 sync
python utils/python/imu_uart_bridge_test.py --port COM13 capture --output sessions/x.json
```

脚本里已经实现了：

- 协议帧封装/解包
- `0x02` 时间同步
- `0x03` start/stop 重试
- `0x21` 灯控
- `0x30/0x32/0x33` 推送解析
- `capture` 时把 IMU / sync event / gpio event 分别落盘

---

## 11. 新人最容易踩的坑

### 11.1 把 USART1 当日志口

不要这么干。当前 boot 日志走 `UART5`，不是 `USART1`。

### 11.2 对端按裸字节解析按钮事件

按钮现在走的是：

- `0x33` 桥协议帧

不是裸串口字符。

### 11.3 误以为 `start` 只影响姿态推送

实际上 `CMD_STREAM_CONTROL start` 还会启动同步输出链路。

### 11.4 误以为 `0x10/0x11` 能访问任意 I2C 设备

当前不是任意透传，只允许访问配置里的目标 IMU 地址。

### 11.5 误以为脚本支持的所有格式，MCU 现在都会发

不是。Host 脚本保留了历史兼容，当前 MCU 默认只发少数几种。

---

## 12. 如果要扩协议，建议怎么做

建议遵守这几个原则：

1. 新增主动事件时，优先放到 `0x3x` 段
2. 继续沿用统一帧头 `55 AA`
3. 响应继续复用相同 `cmd`
4. payload 继续用 little-endian
5. 先改 MCU，再同步改 `utils/python/imu_uart_bridge_test.py`
6. 新增字段时优先做“尾部追加”，减少旧脚本崩溃概率

---

## 13. 深入阅读建议

如果要继续往下查，推荐阅读顺序：

1. 本文
2. `application/imu_uart_bridge_task.cpp`
3. `utils/python/imu_uart_bridge_test.py`
4. `docs/YIS_Cam双同步.md`
5. `docs/IMU_UART_bridge_start_stop_debug_2026-06.md`

如果你只想快速验证串口链路是否通：

1. 先 `ping`
2. 再 `start`
3. 再 `console` 或 `capture`
4. 最后再测按钮和灯控

