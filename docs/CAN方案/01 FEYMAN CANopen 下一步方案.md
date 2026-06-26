# FEYMAN CANopen 下一步方案

## 配置核对结论
- CAN1 配置基本正确：`PA11/PA12`、`250 kbit/s`、`Prescaler=8`、`BS1=13TQ`、`BS2=2TQ`、`SJW=1TQ`，匹配 MCS10 默认 `Node ID 0x7F / 250 kbit/s`。
- 生成代码已接上：`MX_CAN1_Init()` 已在 `main.c` 调用，`HAL_CAN_MODULE_ENABLED` 已启用，`can.c/h` 存在，CAN1 TX/RX0/RX1/SCE 中断都已生成。
- CMake 构建图已包含 `Core/Src/can.c`、`stm32f1xx_hal_can.c`、LibXR `stm32_can.cpp`；`cmake --build build\Debug` 当前无报错。
- `AutoRetransmission = ENABLE` 最终生成正确；`.ioc` 里的 `NART=ENABLE` 名字容易误导，但以 `can.c` 为准。
- 小注意：`f105rct6.ioc` 的 `ProjectManager.functionlistsort` 里没看到 `MX_CAN1_Init`，但 `main.c` 里实际有调用。之后如果 CubeMX 再生成代码，要确认它没有把 CAN init 调用删掉。

## Implementation
- 新增 `FeymanCanopenTask`，使用现有 `STM32CAN can1`，固定默认参数：`node_id=0x7F`、`baudrate=250000`、`data_rate_hz=20`。
- 实现最小 CANopen 主站：NMT Pre-Operational、SDO expedited read/write、NMT Operational、heartbeat/TPDO 接收。
- 上电后动态配置 TPDO：
  - TPDO1 `0x1FF`：Euler，映射 `0x4000:01/02/03`
  - TPDO2 `0x2FF`：Accel，映射 `0x4001:01/02/03`
  - TPDO3 `0x3FF`：Gyro，映射 `0x4002:01/02/03`
  - TPDO4 `0x4FF`：Quaternion，映射 `0x4005:01/02/03/04`
- 解码缩放：Euler `raw * 180 / 32000`，Accel `raw * 8 / 32000`，Gyro `raw * 500 / 32000`，Quaternion `raw / 32000`。
- 新增 `feyman_imu_pose` topic，并给 UART 桥接层增加 `BridgePoseSource::FEYMAN`，方便沿用现有上位机/串口桥看姿态效果。
- FEYMAN 任务启动时显式启动现有 `TIM2_CH3 / PA2` 1Hz 同步 PWM，避免只注册未输出的情况。

## Test Plan
- 先用示波器确认 `PA2` 输出 1Hz、约 1ms 高电平脉冲。
- 上电后用 UART5 日志确认 CANopen SDO 能读到 `0x3003=250000`、`0x3004=0x7F`、`0x3006=20`。
- 进入 Operational 后确认收到 `0x1FF/0x2FF/0x3FF/0x4FF`，频率约 20Hz。
- 手动旋转 FEYMAN，检查 Euler 和 Quaternion 连续变化，Accel/Gyro 方向和量级合理。
- 最后再跑一次完整构建，确认新增任务不影响现有 WIT/YIS/UART 桥。

## Assumptions
- 本轮只走 CANopen TPDO，不启用 J1939。
- TPDO 配置每次启动动态写入，暂不保存到 FEYMAN Flash。
- `PA11/PA12` 已实际连接到 CAN 收发器；总线两端有 120Ω 终端电阻并共地。
