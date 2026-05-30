# UART/YIS 实时断流修复计划

## Summary
首轮修复聚焦实时性：命令解析非阻塞、YIS pose 采用 latest-only 发送策略、sync event 不抢 pose、提升 YIS 采集优先级。首轮不加入 sync 周期限速，不做失败帧可靠重发。

## Key Changes
- 将桥任务命令解析改成字节级非阻塞状态机。
  - `ProcessPendingCommand()` 不再用 `read_timeout_ms` 阻塞等待半包。
  - 每轮只消费 UART RX 队列中已有字节，完整帧校验通过后再分发。
  - 半包、错 checksum、错误 SOF、长度超限都复位 parser，不影响 YIS pose 推送。

- 重构命令 handler，避免 handler 再阻塞读 UART。
  - 新增完整帧分发入口，例如 `HandleCommandFrame(cmd, payload, payload_len)`。
  - `Ping/TimeSync/I2C/SPI/WS2812` 直接使用已缓存 payload。
  - 保留现有 wire protocol，不改上位机帧格式。

- YIS pose 改为 latest-only 发送语义。
  - 不做失败帧原样重发，避免旧样本延迟写入对端 JSON。
  - 桥任务每轮从 YIS 队列中尽量取到最新 pose，丢弃积压旧 pose。
  - 如果 UART 当前发不出去，只保留最新 pose 作为候选；下一轮若有更新样本，直接替换旧候选。
  - 一旦 UART 恢复，只发送当时最新 pose，保证实时性优先于完整性。

- sync event 不抢占 pose。
  - 主循环顺序保持：命令解析、pose、sync。
  - 如果 pose 因 UART backpressure 未能发送，本轮不发送 sync event。
  - sync event 继续使用现有 pending 缓存；首轮不加周期限制。

- 提升 YIS 采集任务优先级。
  - `User/app_main.cpp` 中将 `yis_config.priority` 从 `MEDIUM` 调为 `HIGH`。
  - bridge 任务保持 `MEDIUM`，让采集优先于串口桥接。

## Interface / Type Changes
- `IMUUartBridgeTask` 私有成员增加命令 parser 状态和 payload 缓冲。
- 增加 latest-only pose 候选缓存，例如 `latest_yis_pose_ + has_latest_yis_pose_`。
- `PublishBridgePoseData()` 可返回内部结果枚举，例如 `None/Sent/Backpressure`，供主循环决定是否允许 sync 推送。
- 不改公开配置结构体，不改 UART 协议帧格式。

## Test Plan
- 编译检查相关文件。
- 协议回归：`ping`、`time sync`、`i2c read/write`、`spi write`、`ws2812 control` 正常响应。
- 异常输入：半包、错 checksum、错误 SOF、超长 payload 不阻塞 YIS 输出。
- 实时流验证：YIS 200Hz、`stream_interval_ms=0` 下持续输出；制造 UART 堵塞后恢复，确认输出恢复为最新 pose，而不是补发旧 pose。
- sync 验证：打开 `push_sync_events_in_bridge=true` 后，确认 sync event 不会在 pose 发送受阻时继续占用 TX 队列。

## Assumptions
- 对端 JSON 写入按接收顺序处理，因此首轮以实时性优先，允许丢旧样本。
- pose 帧中的 `sample_timestamp/sensor_mcu_tick_us/readout_mcu_tick_us` 保留，用于后续需要时做离线时间轴校正。
- 首轮不加入 sync 周期限速；只修复阻塞、发送优先级和 latest-only 语义。
