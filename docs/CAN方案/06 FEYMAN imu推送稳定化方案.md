# FEYMAN IMU 推送稳定化方案

## Summary
- 当前两台 FEYMAN、20 Hz、acc+gyro 的数据量不大，USART1/CAN 带宽不是主因。
- 优先修复 `FEYMAN topic -> USART1 bridge` 的快照读取语义，避免 `ASyncSubscriber` 缓冲区在打包过程中被新数据覆盖。
- Manager 层增加轻量聚合窗口，让多设备 CAN 数据尽量以“同一批快照”推送，但不假装已经实现硬同步。
- 不改 USART1 二进制协议，不改上位机解析格式，不先用限流掩盖问题。

## Key Changes
- Bridge FEYMAN 路径改为本地快照：
  - `GetData()` 后立即拷贝成局部 `Manager::FeymanArrayMsg latest`。
  - 后续打包只读局部副本，不再读 subscriber 内部 buffer。
  - `StartWaiting()` 放在本地拷贝完成后即可，或放在本轮打包/发送结束后；推荐“本地拷贝后立刻 StartWaiting”，兼顾安全和不漏最新值。
- Bridge 打包增加边界保护：
  - 每写入一条 FEYMAN record 前检查 `cursor + kBridgeExtendedRecordSize <= payload.size()`。
  - `valid_count == 0` 时正常丢弃本帧，不重发旧帧。
  - 保持现有 `0x30` payload 形状：`count + node_id + 13 float`，不改上位机协议。
- FeymanManager 增加聚合发布策略：
  - 每个设备 `PollSample()` 成功后更新自己的 `latest`，并设置 `updated_device_mask_`。
  - 私有常量 `kAggregateCoalesceWindowUs = 2000`。
  - 当所有 enabled/configured/online 设备都更新过，立即 `PublishAggregate()`。
  - 如果 2 ms 内没有凑齐，也发布当前快照，避免某个设备错相或掉线导致整体阻塞。
  - `state_changed` 仍立即发布一次状态快照。
- CAN 同步策略：
  - v1 继续使用当前 TPDO event timer，不启用 CANopen SYNC。
  - 聚合消息保留每设备 `readout_mcu_tick_us`，用时间戳表达相位差。
  - 后续只有在确认 FEYMAN MCS10 支持 CANopen SYNC / synchronous TPDO 后，再单独做 SYNC 方案。

## Test Plan
- 构建验证：`cmake --build build\\Debug` 通过。
- 串口桥稳定性：
  - `pose_source = FEYMAN`，两台 `0x7E/0x7F` 在线，连续 stream 60 秒。
  - 上位机持续 `ping` 不超时，capture 无 checksum/length 错帧。
  - pose push 中只出现有效在线节点，node_id 稳定为 `0x7E/0x7F`。
- 数据节奏验证：
  - FEYMAN `data_rate_hz = 20` 时，聚合输出约 20 Hz；若两设备相位明显错开，允许短时接近 40 Hz，但每帧必须是完整一致快照。
  - 每设备 sequence 单调变化，无半帧混杂、无异常 NaN 扩散到 acc/gyro。
- 回归验证：
  - WIT 路径保持原行为。
  - YIS 路径保持原行为。
  - UART5 不出现 CAN error / USART bridge backpressure 持续增长迹象。

## Assumptions
- 当前主要风险是 async subscriber 缓冲区生命周期和聚合发布节奏，不是带宽不足。
- `uart_->Write()` 已把栈上 frame 拷贝进 LibXR 写队列/DMA buffer，因此 `SendResponse()` 的栈 buffer 本身不是主风险。
- 上位机当前依赖 FEYMAN 扩展 record 的 13-float 格式，本轮不改 wire protocol。
- 不把 CAN event-timer TPDO 称为硬同步；当前目标是稳定、低延迟、可观测的多设备快照推送。
