# F103C8T6 IMU + WS2812 + UART Bridge 上板联调清单

本文档用于按 `utils/python/imu_uart_bridge_test.py` 的命令顺序，对当前 `f103c8t6_mini` 固件进行逐项上板验证。

## 1. 联调目标

- 验证 `USART1` 上的 UART bridge 协议可正常收发
- 验证 `SPI1` 驱动的 WS2812 控制链路可用
- 验证 `I2C1` 上的 JY901B 可被固件读取
- 验证固件会通过 bridge 主动推送 IMU 欧拉角

## 2. 测试前准备

### 2.1 硬件连接确认

- MCU 工程已烧录当前最新固件
- `USART1` 已连接到 PC 串口工具
- `I2C1` 已连接 JY901B，默认地址为 `0x50`
- `SPI1` 已连接 WS2812 灯带/灯珠
- 板卡供电稳定，WS2812 电源和地与主控共地

### 2.2 PC 端环境准备

- Python 3 可用
- 已安装 `pyserial`

```bash
pip install pyserial
```

### 2.3 串口号确认

先确认板子在 PC 上对应的串口号，例如：

- Windows: `COM5`
- Linux: `/dev/ttyUSB0`

下文统一用 `COMx` 代指串口号，实际执行时请替换。

## 3. 固件基础确认

### 3.1 编译与烧录

- 确认本地工程已成功构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

- 将生成的 `build/Debug/f103c8t6_mini.elf` / 对应烧录文件烧录到开发板

### 3.2 上电静态现象

- 上电后串口不应持续输出普通调试日志
- 串口应保持安静，等待 bridge 命令或主动输出 IMU 数据
- 若串口持续打印乱码/日志，说明 `USART1` 仍被其他输出占用，需要先排查

## 4. Python 脚本逐项验证

脚本位置：

```bash
python utils/python/imu_uart_bridge_test.py --help
```

### 4.1 `ping`：验证 bridge 基础通信

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx ping
```

期望结果：

- 终端输出 `PING ok`
- 无超时、无校验错误

若失败：

- 检查串口号是否正确
- 检查波特率是否为 `115200`
- 检查固件是否成功启动到 `IMUUartBridgeTask`

### 4.2 `off`：验证 WS2812 全灭控制

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx off
```

期望结果：

- 输出类似 `OFF ok, spi_bus=0, led_count=16`
- WS2812 全部熄灭

若失败：

- 检查 `SPI1` 接线
- 检查灯带供电
- 检查 `led_count` 是否与实际灯数差异过大

### 4.3 `rgb`：验证 WS2812 整体设色

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx rgb 135 206 250
```

期望结果：

- 输出 `RGB ok`
- 全部灯珠显示统一颜色

建议追加验证纯色：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx rgb 255 0 0
python utils/python/imu_uart_bridge_test.py --port COMx rgb 0 255 0
python utils/python/imu_uart_bridge_test.py --port COMx rgb 0 0 255
```

### 4.4 `led`：验证单灯寻址

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx led 3 255 0 0
```

期望结果：

- 输出 `LED ok`
- 仅第 `3` 号灯亮红色，其余熄灭

建议再验证：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx led 0 0 255 0
python utils/python/imu_uart_bridge_test.py --port COMx led 15 0 0 255
```

用于确认首尾索引是否正常。

### 4.5 `blink`：验证持续控制与时序稳定性

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx blink 135 206 250
```

期望结果：

- 灯带持续闪烁
- 运行期间无脚本异常退出
- 串口链路稳定，无卡死

停止方式：

- `Ctrl + C`

停止后建议补一条：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx off
```

### 4.6 `console`：验证 WS2812 + IMU 欧拉角推送并存

执行：

```bash
python utils/python/imu_uart_bridge_test.py --port COMx console
```

进入后先观察：

- 终端应持续输出形如：

```text
imu,0x50,roll,pitch,yaw
```

注意点：

- 数值会随 IMU 姿态变化
- 当前数据应来自模块原生欧拉角，而不是四元数换算

在 `console` 模式下继续输入以下命令做混合验证。

#### a. 控制链路仍可工作

```text
ping
rgb 255 255 255
led 2 255 0 0
off
```

期望结果：

- 每条命令都有对应成功反馈
- IMU 推送不中断或仅短暂插空

#### b. 观察姿态变化

手动旋转 IMU，确认：

- 绕 X 轴转动时 `roll` 明显变化
- 绕 Y 轴转动时 `pitch` 明显变化
- 平面旋转时 `yaw` 明显变化

若输出方向与物理动作不一致，记录现象，后续再做坐标系修正。

#### c. 运行闪烁期间观察 IMU 推送

在 `console` 中输入：

```text
blink 0 0 255 300
```

期望结果：

- WS2812 正常闪烁
- 终端仍持续输出 `imu,0x50,...`

停止：

```text
stop
off
```

退出：

```text
quit
```

## 5. 通过判定标准

满足以下条件即可判定本轮联调通过：

- `ping` 成功
- `off/rgb/led/blink` 全部可正常控制 WS2812
- `console` 下能稳定收到 `imu,0x50,roll,pitch,yaw`
- 旋转 IMU 时欧拉角有连续变化
- WS2812 控制与 IMU 推送可同时工作，不互相打断

## 6. 问题排查建议

### 6.1 `ping` 超时

优先检查：

- 波特率是否 `115200`
- 串口号是否正确
- 板卡是否真的跑到了新固件
- `USART1 TX/RX` 是否接反

### 6.2 WS2812 无反应

优先检查：

- `SPI1 MOSI` 接线
- 灯带供电是否足够
- 地线是否共地
- 灯珠输入端方向是否接反

### 6.3 `console` 无 IMU 输出

优先检查：

- JY901B 地址是否为 `0x50`
- `I2C1` 上拉是否正常
- `SCL/SDA` 是否接反
- 模块是否已上电

### 6.4 IMU 数值不变化或异常跳变

优先检查：

- I2C 供电与线序
- 模块输出模式
- 模块是否被其他主机占用
- 板上电源噪声是否过大

## 7. 测试记录模板

建议每次上板后填写一次：

| 项目 | 命令 | 结果 | 备注 |
| --- | --- | --- | --- |
| Bridge Ping | `ping` | 通过 / 失败 |  |
| WS2812 Off | `off` | 通过 / 失败 |  |
| WS2812 RGB | `rgb 255 0 0` | 通过 / 失败 |  |
| WS2812 Single LED | `led 3 255 0 0` | 通过 / 失败 |  |
| WS2812 Blink | `blink 0 0 255` | 通过 / 失败 |  |
| IMU Push | `console` | 通过 / 失败 |  |
| Roll/Pitch/Yaw 响应 | 手动转动 IMU | 正常 / 异常 |  |

