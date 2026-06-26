# FEYMAN PPS 数采输出方案

## Summary
- 将 FEYMAN 从“全量状态快照输出”改为“增量样本流输出”：每个设备产生一条新 sample，bridge 才输出该设备一条 record。
- 继续使用 `0x30` IMU push 命令，但新增一种 FEYMAN 数采 record 格式，保留 WIT/YIS/旧 FEYMAN record 解析兼容。
- v1 不依赖 FEYMAN 内部 PPS 时间字段；使用 MCU 输出的 1Hz PPS epoch、FEYMAN module sequence、MCU readout tick 做数采对齐。
- 目标统计口径：两台 FEYMAN 各 100Hz 时，单地址约 100Hz，总 records 约 200Hz。

## Key Changes
- Manager 层新增 FEYMAN sample stream：
  - `FeymanManagerConfig` 增加 `sample_topic_name = "feyman_imu_sample"`。
  - 每次 `PollSample()` 成功，除更新 aggregate/latest 外，额外发布一条单设备 `FeymanDeviceMsg` 到 sample topic。
  - sample 中保留 `node_id / sequence / status_flags / heartbeat_state / acc / gyro / readout_mcu_tick_us`。
  - `sensor_mcu_tick_us` 在 FEYMAN v1 中明确表示“最近一次 TIM2 PPS epoch tick”，`time_status` 标记该 epoch 是否有效。
- Bridge FEYMAN 路径改为增量数采：
  - FEYMAN bridge 订阅 `feyman_imu_sample`，使用 bounded queue，不再从 `feyman_imu_array` 全量快照生成 `0x30` 数采 record。
  - 每轮 bridge loop 最多打包若干条新 sample record，避免一次性排空队列影响 ping/命令响应。
  - 不重复发送旧样本；同一 `node_id + sequence` 只输出一次。
- `0x30` 新增 FEYMAN sample record 格式：
  - record layout 固定为 little-endian：
    `node_id:u8, status:u8, time_status:u8, sequence:u16, status_flags:u16, heartbeat_state:u16, pps_epoch_mcu_tick_us:u64, readout_mcu_tick_us:u64, acc[3]:float, gyro[3]:float`
  - record size 为 49 bytes，与现有 53-byte FEYMAN extended record 区分。
  - payload 仍为 `count + records...`，不改 frame header/checksum。
- Python 工具同步升级：
  - `imu_uart_bridge_test.py` 增加 49-byte FEYMAN sample record 解析。
  - `IMUPushRecord` 增加 `sequence/status/status_flags/heartbeat_state` 字段。
  - `rate-monitor` 增加推荐参数：`--expected-total-hz` 和 `--expected-addr-hz`；旧 `--expected-hz` 保留为兼容显示。
  - rate 统计优先按 `node_id + sequence` 去重，host receive time 只用于链路间隔观察。

## Test Plan
- 构建验证：
  - `cmake --build build\Debug` 通过。
- 协议解析验证：
  - Python 能同时解析 WIT compact、YIS extended、旧 FEYMAN extended、新 FEYMAN sample record。
  - `0x30` checksum/length 校验不变。
- 上板数采验证：
  - 两台 FEYMAN，`data_rate_hz = 100`。
  - `rate-monitor --expected-total-hz 200 --expected-addr-hz 100` 下，总 record_hz 接近 200，`0x7E/0x7F` 各接近 100。
  - 每个地址 `sequence` 单调递增，无重复 record；若有丢样，脚本能通过 sequence gap 显示。
- PPS 对齐验证：
  - 开启 stream 后，PB10/PA1 TIM2 1Hz PPS 与 `0x32` sync event 连续出现。
  - FEYMAN sample record 中 `pps_epoch_mcu_tick_us` 每秒更新一次；同一秒内多条 sample 共享同一 epoch。
  - Python 可按 epoch 分组统计每秒每设备样本数，目标约 100/sample/epoch/device。
- 回归验证：
  - WIT/YIS bridge 输出不变。
  - USART1 ping 在 FEYMAN 200 total records/s 下仍正常响应。
  - FEYMAN array topic 继续保留给状态/监控使用，但不再作为数采主输出源。

## Assumptions
- v1 暂时没有 FEYMAN 内部 PPS 后采样时间/采样序号的 CANopen 对象字典映射。
- FEYMAN module 当前 `sequence` 表示 acc+gyro 两个 TPDO 都到齐后形成的一条完整 sample。
- MCU TIM2 PPS 是所有 FEYMAN 设备共同的 epoch 边界；CAN 到达时间只用于 readout/链路诊断，不作为传感器采样时刻。
- 数采主语义优先于全量状态快照；如需 UI 最新状态，继续消费 `feyman_imu_array`。

## Debug 收敛结论
- 第一类问题是 FEYMAN bridge 旧实现的栈压力过大：
  - FEYMAN 多设备快照、最大 payload、最大 response frame 同时压在 `IMUBridge` 任务栈上。
  - `IMUBridge` 任务栈较小，表现为 `USART1` ping 异常、桥接卡死或线程不稳定。
  - 已通过将大块缓冲迁移到文件级静态区收敛。
- 第二类问题是“全量快照输出”不适合作数采统计：
  - manager 任一设备更新都会发布一包包含全部设备的快照。
  - 上位机按 record 出现次数统计时，会把重复带出的旧设备 record 也算进去，导致单地址 `record_hz` 虚高。
  - 已改为“增量样本流输出”，每个设备产生新 sample 才输出一条 record。
- 第三类问题是 FEYMAN 数采 sample 接入初版在 bridge 启动阶段使用线程内动态 `new LockFreeQueue<Manager::FeymanDeviceMsg>(64)`：
  - 实际现象为 `UART5` 日志停在 `pose=FEYMAN topic=1` 后，`USART1` 无法 ping 通。
  - 该问题在当前堆/线程环境下风险较高，表现更像 bridge 线程在 sample 队列创建阶段异常退出或卡死。
  - 已改为 bridge 内部固定容量环形缓冲 + topic callback 入队，不再依赖该路径上的动态堆分配。
- 曾临时补充过 `UART5` bridge 周期诊断日志，用于区分线程启动、`USART1` TX 背压和 sample 环形缓冲满等问题；当前问题收敛后，该周期诊断日志已从固件中移除，避免长期占用 UART5 日志带宽。

## 当前测试结果
- 构建与工具检查通过：
  - `cmake --build build\Debug`
  - `python -m py_compile utils\python\imu_uart_bridge_test.py`
- 当前验证条件：
  - 两台 FEYMAN 在线，`data_rate_hz = 100`
  - Python 使用 `rate-monitor --expected-total-hz 200 --expected-addr-hz 100`
- 最近一次 10 秒窗口结果：
  - `rate_final, elapsed_s=9.968, bundles=1957, bundle_hz=196.33, records=1997, record_hz=200.34`
  - `rate_final_expected, total_hz=200.00, total_ratio=100.2%`
  - `0x7E: records=998, record_hz=100.12, seq_first=30965, seq_last=31962, seq_missing=0, seq_dup=0, seq_backtrack=0`
  - `0x7F: records=999, record_hz=100.22, seq_first=30932, seq_last=31930, seq_missing=0, seq_dup=0, seq_backtrack=0`
- 结果解释：
  - 总体接收率已经与目标 `200 Hz` 基本一致。
  - 两个设备单地址接收率均稳定在 `100 Hz` 左右。
  - `sequence` 连续、无缺失、无重复、无回退，说明当前 10 秒窗口内数采链路未见掉样。
  - `host_step_min_ms=0.0`、`host_step_max_ms=32.0` 只反映 host 接收与 bundle 合包节拍抖动，不代表 sample 丢失。

## USART1 速率预算
- 当前 USART1 配置为 `460800 baud`，8N1 串口有效线速约为 `46080 byte/s`。
- FEYMAN 新 sample record 固定为 `49 byte`；外层 `0x30` frame 还包含 `count`、帧头、长度和 checksum。
- 若一帧只带 1 条 FEYMAN record，链路开销约为 `56 byte/record`；若经常合包 2 条 record，约为 `52.5 byte/record`；4 条 record 时约为 `50.75 byte/record`。
- 两台 FEYMAN 同时输出时，串口预算建议如下：
  - `data_rate_hz = 100`：总 `200 records/s`，约 `10.2~11.2 KB/s`，当前实测稳定。
  - `data_rate_hz = 200`：总 `400 records/s`，约 `20.3~22.4 KB/s`，仍在 `460800 baud` 合理范围内，建议作为当前 USART1 协议下的高速数采档。
  - `data_rate_hz = 500`：总 `1000 records/s`，约 `50.8~56.0 KB/s`，已经接近或超过 `460800 baud` 理论线速，不适合用当前 49-byte float record 直接输出。
  - `data_rate_hz = 1000`：总 `2000 records/s`，约 `101.5~112.0 KB/s`，当前 USART1 配置明确无法承载。
- 当前建议：
  - 常规数采使用 `100 Hz/device`。
  - 高速数采优先验证 `200 Hz/device`。
  - `500 Hz/device` 以上只适合先验证 CAN 侧采集能力；若要稳定从 USART1 输出，需要提高波特率到 `921600`、`1500000` 或 `2000000`，并考虑压缩 record 格式或更换高速输出通道。
