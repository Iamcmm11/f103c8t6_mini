# FEYMAN MCS10 CANopen TPDO 测试方案

## Summary
- 当前实现使用 `CAN2` 作为 FEYMAN MCS10 的测试总线，默认参数为 `Node ID = 0x7F`、`Baudrate = 250 kbit/s`、`Data Frequency = 20 Hz`。
- 本轮已收窄数据目标，只接收并解析 `Accelerometer` 和 `Gyroscope`，不再依赖 Euler 或 Quaternion。
- 外部同步脉冲使用现有 `TIM2_CH2 / PA1` 输出 1 Hz 脉冲；`CAN2` 工作时必须同时打开 `CAN1` RCC 时钟，但不需要启用 `CAN1` 引脚或收发逻辑。

## CubeMX / Board 配置
- `CAN2` 使用默认引脚 `PB12 = CAN2_RX`、`PB13 = CAN2_TX`。
- 位时序按 APB1=32 MHz 配置为 `250 kbit/s`：
  - `Prescaler = 8`
  - `BS1 = 13 TQ`
  - `BS2 = 2 TQ`
  - `SJW = 1 TQ`
- `CAN2` 需要额外打开 `CAN1` RCC：
  - `__HAL_RCC_CAN2_CLK_ENABLE()`
  - `__HAL_RCC_CAN1_CLK_ENABLE()`
- 1 Hz 外部同步信号由 `TIM2_CH2 / PA1` 输出，当前周期约 1 s，脉宽约 1 ms。

## Current Firmware Behavior
- `FeymanCanopenTask` 启动后执行以下流程：
  - 发送 `NMT Pre-Operational`
  - 使用 SDO 访问 `0x3003 / 0x3004 / 0x3006 / 0x1017 / 0x300B / 0x300D`
  - 动态配置两路 TPDO
  - 发送 `NMT Reset Communication`
  - 发送 `NMT Start`
- 当前只配置两路 TPDO：
  - `TPDO1 = 0x1FF`，映射 `0x4001:01/02/03`，即加速度计 XYZ
  - `TPDO2 = 0x2FF`，映射 `0x4002:01/02/03`，即陀螺仪 XYZ
- 数据缩放：
  - `Accel = raw * 8 / 32000 * 9.80665`，单位 `m/s^2`
  - `Gyro = raw * 500 / 32000`，单位 `deg/s`
- 数据发布到 topic：`feyman_imu_pose`
- UART 桥接在 `BridgePoseSource::FEYMAN` 下输出扩展 IMU record：
  - `acc_x/acc_y/acc_z`
  - `gyro_x/gyro_y/gyro_z`
  - `roll/pitch/yaw`、`quat` 统一填 `NaN`

## Validation Status
- 已完成并验证通过：
  - `CAN2` 切换完成
  - `CAN2 + CAN1 RCC` 组合生效
  - FEYMAN 设备成功响应 `SDO read 0x3003`
  - 参数写入成功
  - TPDO 映射成功
  - `NMT reset-comm` 与 `NMT start` 执行成功
  - 启动日志出现 `[feyman] configure ok`
- 当前成功日志关键路径：
  - `[feyman] configure begin`
  - `[feyman] nmt pre-op`
  - `[feyman] basic params`
  - `[feyman] sdo read 0x3003`
  - `[feyman] baud current=250000`
  - `[feyman] map tpdo accel`
  - `[feyman] map tpdo gyro`
  - `[feyman] nmt start`
  - `[feyman] configure ok`

## Known Pitfalls
- `CAN2` 只开自身 RCC 会导致总线行为异常，必须同时打开 `CAN1` RCC。
- `CAN` 中断回调里不能调用普通 `Semaphore::Post()`，必须使用 `PostFromCallback(true)`，否则会触发 FreeRTOS `vPortEnterCritical()` 断言。
- 在 `CAN2` 方案下，如果 1 Hz 脉冲切换到 `TIM2_CH2`，需要保证 `TIM2_CH2` 的 `Pulse` 非零，否则外部同步脚无波形。

## Next Test Items
- 上位机或串口桥确认是否已持续收到 `acc` / `gyro` 数据。
- 如需进一步缩小带宽，可把 `Data Frequency` 从 `20 Hz` 调整为更低值后再重复验证。
- 若后续需要姿态数据，再单独恢复 Euler 或 Quaternion 的 TPDO 映射，不与本轮加速度计/陀螺仪调试混合。
