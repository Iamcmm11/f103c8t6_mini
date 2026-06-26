# FEYMAN MCS10 CANopen 联调记录 2026-06

## 背景

本轮目标是将 FEYMAN MCS10 IMU 接入当前 STM32F105RCT6 工程，使用 CANopen 方式完成基础联调，并通过现有 UART bridge 输出调试数据。

联调过程中，先后经历了：

- `CAN1` 默认引脚占用，切换到 `CAN2`
- `SDO` 初始超时
- FreeRTOS 在 `vPortEnterCritical()` 断言
- 最终完成 SDO 建链、参数写入、TPDO 映射和 `configure ok`

## 最终成功日志

本轮成功启动日志如下：

```text
[boot] app_main
[boot] ws2812 ec=0 leds=16
[boot] ws2812 default=0 rgb=(135,206,250)
[boot] wit init=-2 online=0
[boot] sync pwm registered clk=64000000 psc=6399 arr=9999 ccr=10 freq=1
[boot] camera pwm registered psc=639 arr=3332 ccr=100 freq=30
[boot] yis init=-15 addr=0x6A hal=0x6A
[feyman] start node=0x7F baud=250000 rate=20
[boot] wit start=-2 freq=200 stack=2048
[boot] yis start=-2 freq=200 stack=1536
[boot] feyman sync=0 start=0 node=0x7F rate=20
[boot] bridge start=0 push=1 period=0
[boot] gpio init=0 start=0 PC10=A PA15=B PB3=C PB4=D
[feyman] configure begin
[feyman] nmt pre-op
[feyman] basic params
[feyman] sdo read 0x3003
[feyman] baud current=250000
[feyman] sdo write 0x3003
[feyman] sdo write 0x3004
[feyman] sdo write 0x3006
[feyman] sdo write 0x1017
[feyman] sdo write 0x300B
[feyman] sdo write 0x300D
[feyman] sdo read 0x3004
[feyman] node current=0x7F
[feyman] map tpdo accel
[feyman] tpdo cfg comm=0x1800 map=0x1A00 cob=0x1FF
[feyman] map tpdo gyro
[feyman] tpdo cfg comm=0x1801 map=0x1A01 cob=0x2FF
[feyman] nmt reset-comm
[feyman] nmt start
[feyman] configure ok
```

## 错误统计

### 1. CAN 总线资源选择错误

现象：
- 评估板 `CAN1` 默认引脚已被占用，原始方案无法落板验证。

处理：
- 切换到 `CAN2`
- 使用 `PB12/PB13`

结果：
- `CAN2` 初始化、中断、任务接线全部切换完成。

### 2. CAN2 仅开自身 RCC 导致链路异常

现象：
- 切到 `CAN2` 后，早期版本依旧无法收到有效 `SDO` 响应。

根因：
- `STM32F105` 上 `CAN2` 依赖共享的 bxCAN/filter RAM，`CAN2` 工作时必须同时打开 `CAN1` RCC 时钟。

修复：
- 在 `Core/Src/can.c` 中保留：
  - `__HAL_RCC_CAN2_CLK_ENABLE();`
  - `__HAL_RCC_CAN1_CLK_ENABLE();`

结果：
- `CAN2` 可以正常建立 CANopen 通讯。

### 3. FEYMAN 任务仍绑定旧的 can1 实例

现象：
- 切换到 `CAN2` 后，`app_main.cpp` 里 FEYMAN 任务仍指向旧变量，构建失败。

修复：
- `FeymanCanopenTask` 构造参数改为 `&can2`

结果：
- 构建恢复正常。

### 4. 1 Hz 同步输出切到 TIM2_CH2 后脉宽为 0

现象：
- 同步输出逻辑从 `TIM2_CH3` 调整到 `TIM2_CH2` 后，若 `Pulse=0`，则实际无脉冲输出。

修复：
- 将 `TIM2_CH2` 的 `Pulse` 改为非零值，当前为 `10`

结果：
- `1 Hz` 外部同步脉冲恢复有效。

### 5. FreeRTOS 断言：在中断里误用普通 API

现象：
- 调试暂停后停在：

```c
void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
    if( uxCriticalNesting == 1 )
    {
        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
    }
}
```

根因：
- `HandleSdoResponse()` 运行在 CAN 中断上下文
- 响应匹配后调用了普通版 `sdo_sem_.Post()`
- 该路径内部走 `xSemaphoreGive()`，最终触发 `vPortEnterCritical()` 断言

修复：
- 将 `sdo_sem_.Post()` 改为 `sdo_sem_.PostFromCallback(true)`

结果：
- 中断上下文下的信号量通知恢复正确，不再触发该断言。

### 6. ISR 中直接做 UART 阻塞日志风险

现象：
- 在 CAN 错误回调路径中直接打印日志，会增加 ISR 中阻塞和重入风险，影响真实问题暴露。

修复：
- ISR 中只记录错误状态
- 回到 FEYMAN 任务线程后统一输出 `can error / can state` 日志

结果：
- 错误诊断链路更稳，避免 ISR 内部再引入新的同步问题。

## 当前实现收敛结果

- 总线：`CAN2`
- 默认通信参数：
  - `Node ID = 0x7F`
  - `Baudrate = 250000`
  - `Data Frequency = 20`
- 外部同步：
  - `TIM2_CH2 / PA1`
  - `1 Hz`
- 当前仅解析并输出：
  - `Accelerometer`
  - `Gyroscope`
- 当前不依赖：
  - Euler
  - Quaternion

## 当前阶段结论

本轮联调已经确认：

- FEYMAN MCS10 可在当前板卡上通过 `CAN2 + CANopen` 成功完成基础配置
- SDO 通讯正常
- TPDO 映射正常
- `NMT reset-comm` 与 `NMT start` 正常
- 系统侧和 FreeRTOS 侧关键断言问题已修复

下一阶段重点应转向：

- 验证 `TPDO1_ACCEL` 与 `TPDO2_GYRO` 的持续输出
- 用现有 `UART bridge + imu_uart_bridge_test.py` 观察 `acc/gyro` 连续数据
- 根据现场数据稳定性再决定是否恢复更多 TPDO 映射
