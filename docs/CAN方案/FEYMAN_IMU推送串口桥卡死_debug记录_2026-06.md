# FEYMAN IMU 推送串口桥卡死 Debug 记录 2026-06

## 现象

- `BridgePoseSource::FEYMAN` 下，FEYMAN IMU 主动推送会导致 USART1 串口桥表现为卡死或响应异常。
- WIT 六个 IMU 走 I2C 聚合推送时没有同样问题。
- 当前 FEYMAN 配置为两台 CANopen 设备，`data_rate_hz = 20`，只推 `acc + gyro`。

## 排查结论

问题不是 USART1/CAN 带宽不足。

- 两台 FEYMAN：`2 device * 2 TPDO * 20 Hz`，CAN 帧量很低。
- USART1 为 `460800`，两台 FEYMAN 扩展 record 的实际吞吐也远低于串口极限。
- 因此“包太大把串口打满”不是当前主因。

最终更符合现象的根因是：**IMUBridge 任务栈压力过大，FEYMAN 多设备路径引入的大对象在函数栈上分配，导致串口桥线程不稳定。**

关键风险点：

- `PublishBridgePoseData()` 中 FEYMAN 路径需要处理 `FeymanArrayMsg` 快照。
- FEYMAN payload 按 `MAX_FEYMAN_DEVICE_COUNT` 计算最大容量。
- `SendResponse()` 原本每次在栈上创建最大 response frame。
- `IMUBridge` 当前任务栈只有 `1536` bytes，叠加局部数组后容易栈溢出。

## 已实施修复

- 保留 FEYMAN bridge 使用 `feyman_imu_array` 聚合 topic。
- `ASyncSubscriber` 数据先复制为快照，再打包，避免读取过程中被覆盖。
- FEYMAN payload 增加边界检查：写入 record 前确认 `cursor + kBridgeExtendedRecordSize <= payload.size()`。
- 将大块缓冲从函数栈迁移到文件级静态区：
  - `g_bridge_feyman_payload`
  - `g_bridge_response_frame`
  - `g_bridge_feyman_snapshot`
- USART1 二进制协议不变，仍为 `0x30` pose push，payload 形状保持 `count + node_id + 13 float`。
- CANopen TPDO 模式不变，仍使用 event timer，不启用 CANopen SYNC。

涉及代码：

- `application/imu_uart_bridge_task.cpp`
- `managers/feyman_manager.cpp`
- `managers/feyman_manager.hpp`

## 验证结果

构建通过：

```text
cmake --build build\Debug
```

修复后内存变化：

- 修复前 RAM 使用约 `45960 B`
- 修复后 RAM 使用约 `47352 B`
- RAM 增加是预期结果：原本压在 `IMUBridge` 小栈上的大对象被转移到全局静态区。

上板反馈：

- 串口桥卡死问题已解决。

## 40 Hz 统计现象说明

后续观察到上位机统计：

```text
rate_addr,addr,0x7F,records,400,record_hz,40.06,
host_step_avg_ms,25.025,expected_hz,20.00,ratio,200.3%
```

这不表示 `0x7F` 设备真实采样变成 40 Hz。

当前 manager 发布的是“全量快照”：

- `0x7E` 更新时，发布一次包含 `0x7E + 0x7F` 的聚合包。
- `0x7F` 更新时，再发布一次包含 `0x7E + 0x7F` 的聚合包。
- 如果两台设备相位错开约 25 ms，上位机按 record 出现次数统计，就会看到每个地址约 40 Hz。

也就是说，`record_hz` 统计的是“地址 record 在桥输出中出现的频率”，不是该设备 sequence/timestamp 的真实更新频率。

## 后续建议

- 如果希望上位机看到每个地址 record 频率等于真实采样率，bridge 应只输出本轮更新过的设备 record。
- 如果希望每包都带全量设备，则上位机统计真实采样率时应按每设备 `sequence` 或 `readout_mcu_tick_us` 去重。
- 当前不要用增大 coalesce window 来强行压到 20 Hz；那会增加延迟，也掩盖多设备相位差。

