# FEYMAN bridge 异步订阅定位记录 2026-06

## 现象

FEYMAN CANopen 任务本身已经可以正常工作，关键日志如下：

```text
[feyman] start node=0x7F baud=250000 rate=20
[feyman] configure ok
```

同时在保持 FEYMAN 任务开启的情况下，桥接行为出现明显分化：

- 当 `pose_source = YIS` 时，`USART1` 的 `ping` 正常
- 当 `pose_source = FEYMAN` 时，`USART1` 的 `ping` 异常
- 当完全注销 FEYMAN bridge 路径时，`ping` 恢复正常
- 当 FEYMAN bridge 改成异步订阅后，`ping` 恢复正常
- 并且打开 FEYMAN 数据推送后，可以正常收到：

```text
imu_bundle,1,0x7F,acc=(0.130,-0.218,-9.763),gyro=(0.031,-0.031,-0.719)
```

## 结论

问题不在：

- `USART1` 物理链路
- `UART bridge` 基础协议
- FEYMAN CANopen 配置流程
- FEYMAN 的 TPDO 数据解析本身

问题被定位在：

- `FEYMAN topic -> bridge 订阅/缓存/转发` 这一段内部实现

更准确地说，是 **FEYMAN 使用 `QueuedSubscriber` 路径时会影响 bridge 命令响应，而使用 `ASyncSubscriber` 路径时恢复正常**。

## 为什么异步订阅没有问题

### 1. `ASyncSubscriber` 只保留最新一帧

`ASyncSubscriber` 的工作方式是：

- topic 发布时，把最新数据直接覆盖到订阅者缓冲区
- 同时把状态从 `WAITING` 改成 `DATA_READY`
- bridge 线程读取一次后，再调用 `StartWaiting()`

这个模型的特点是：

- 没有额外队列
- 没有积压
- bridge 每次最多处理 1 帧最新数据
- 单次处理复杂度稳定，时延可控

### 2. `QueuedSubscriber` 会引入额外缓存和排空过程

`QueuedSubscriber` 的工作方式是：

- topic 发布时，立刻把数据拷贝进 `LockFreeQueue`
- bridge 线程在 `PublishBridgePoseData()` 中用 `while (Pop(...))` 排空队列

对于 FEYMAN 这种连续输出源，这条路径会带来几个副作用：

- topic 发布频率越高，队列越容易持续非空
- bridge 线程会在一个周期内做更多次 `Pop`
- 命令处理 `ProcessPendingCommand()` 与 pose 推送竞争同一个 bridge 线程时间片
- 当桥接推送、同步事件推送、按钮推送叠加时，`ping` 这种短命令更容易表现为“不正常”或“响应变差”

也就是说，这不是 FEYMAN 数据格式的问题，而是 **bridge 线程在 FEYMAN 队列路径下被额外工作量拖慢了**。

### 3. 差分实验已经足够说明问题位置

本轮做过的差分测试有：

1. FEYMAN bridge 全注销  
   结果：`ping` 恢复正常

2. FEYMAN bridge 保留，但只禁用最终发包  
   结果：依旧不正常

3. FEYMAN bridge 改为 `ASyncSubscriber`，并恢复发包  
   结果：`ping` 恢复正常，且可以正常输出 `acc/gyro`

这说明真正的分界点不是：

- 是否发送串口包
- 是否解析 FEYMAN 数据

而是：

- **是否使用队列式订阅**

## 当前收敛后的实现

当前 FEYMAN bridge 方案已经固定为：

- FEYMAN CANopen 任务发布 topic：`feyman_imu_pose`
- bridge 对 FEYMAN 使用 `ASyncSubscriber`
- bridge 串口仍走原有 `0x55 0xAA` 协议
- 当前只输出：
  - `acc_x/acc_y/acc_z`
  - `gyro_x/gyro_y/gyro_z`
- `rpy/quat` 在桥接扩展 record 中填 `NaN`

## 工程建议

后续如果再接入类似 FEYMAN 这种持续输出型单设备源，优先建议：

- 默认使用 `ASyncSubscriber`
- 只在确实需要“逐帧不丢”的场景下再考虑 `QueuedSubscriber`

原因很简单：

- bridge 是单线程处理命令和推送
- 最新值语义的姿态/IMU 数据更适合异步覆盖模型
- 队列模型更适合需要逐条保序消费的事件流，而不是高频连续状态量
