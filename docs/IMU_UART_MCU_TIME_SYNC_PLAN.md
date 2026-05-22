# IMU UART MCU 时间同步方案

## 目的

本文档用于记录外部 IMU/MCU 与 NV/SOC 之间的时间对齐方案。

当前目标是先形成可评审的设计口径，不直接修改采集代码。后续实现时，再根据 MCU 固件能力和串口协议约束落到具体包格式。

核心目标：

- 让外部 IMU 数据具备可对齐的视频时间戳，而不是依赖 NV 收包时刻或其它模组兼容字段。
- 让 IMU 数据可以和视频帧 `wall_us` / SEI 时间戳在同一 SOC 时间轴上对齐。
- 保留 NV 收包时间，用于诊断串口传输、Linux 调度和 MCU 排队造成的延迟。

## 当前问题

当前外部 IMU/MCU 同步方案不应依赖 `ts_iso`。

说明：

- `ts_iso` 是接收脚本为兼容另一个 IMU 模组时可能生成的字段。
- 本方案讨论的外部 IMU 数据流可以没有 `ts_iso`。
- 即使后续兼容层写出 `ts_iso`，它也只应被视为 NV 侧接收/解析时间，不是 IMU 真实采样时间。

相关语义是：

- 外部 IMU/MCU 持续向 NV 推送数据。
- NV 开始采集后打开 UART 接收和记录。
- 如果只在 NV 侧给数据打接收时间戳，这个时间更接近“NV 用户态收到并解析到这一包的时间”。
- NV 接收时间不是 IMU 的真实采样时刻，也不是 MCU 读取 IMU 数据时刻。

因此，不能把 NV 接收时间直接当作 IMU 采样时间去对齐视频。这样会包含以下不确定延迟：

- IMU 采样到 MCU 读取的延迟。
- MCU 内部打包、排队和发送延迟。
- UART 线上传输时间。
- Linux 串口驱动和用户态调度延迟。
- Python 解析和回调调度延迟。

这些延迟可能大体稳定，但不能直接假设完全固定。尤其在系统负载变化、串口 buffer 堆积、MCU 主循环被其它任务打断时，延迟会出现抖动。

## 方案概述

建议采用类似 NTP 的四时间戳模型，在 NV 与 MCU 之间周期性做轻量对时。

同步后，MCU 每条 IMU 样本都携带本地时间戳 `mcu_tick`。NV 维护一个从 MCU 本地时间到 NV/SOC 时间的线性映射：

```text
nv_time_us = a * mcu_tick + b
```

其中：

- `mcu_tick`：MCU 本地单调计数，建议由硬件 timer 提供。
- `a`：MCU tick 到微秒的比例系数，同时可包含频率漂移修正。
- `b`：MCU 时间轴映射到 NV 时间轴的偏移。
- `nv_time_us`：换算后的 NV/SOC 时间，后续用于和视频帧对齐。

初版可以先固定 `a` 为 MCU 标称 tick 频率，只估计 `b`。如果长时间测试发现漂移明显，再用多轮同步样本拟合 `a` 和 `b`。

## 时间轴约束

本方案里有两类时间轴，必须在实现时明确区分：

- 同步计算内部时间轴：NV 侧建议使用 `CLOCK_MONOTONIC_RAW` 或等价单调时钟，避免 wall clock 被 NTP/PTP/系统校时调整。
- 对外数据时间轴：最终写入 IMU JSON 的 `timestamp_us` 必须和视频 `wall_us` / SEI 时间戳处在同一个 SOC 时间轴上。

如果视频侧 `wall_us` 本身就是 Unix wall clock 微秒时间，而同步内部使用 `CLOCK_MONOTONIC_RAW`，则不能直接用 `imu.timestamp_us - video_t0_us`。此时 NV 侧需要在采集开始时建立一组本地桥接关系，例如：

```text
mono_raw_t0_ns = clock_gettime(CLOCK_MONOTONIC_RAW)
wall_t0_us     = clock_gettime(CLOCK_REALTIME) 或视频模块提供的 wall_us
```

然后把由 `mcu_tick` 映射得到的 monotonic 时间换算到视频使用的 `wall_us` 轴，或者反过来让视频侧也输出 monotonic 时间。总原则是：视频和 IMU 对齐计算只能在同一时间轴上进行。

## 四时间戳同步协议

一次同步交互记录四个时间戳：

```text
NV  发送 sync 请求前: t1_nv_ns
MCU 收到 sync 请求时: t2_mcu_tick
MCU 发送 sync 回复前: t3_mcu_tick
NV  收到 sync 回复后: t4_nv_ns
```

推荐要求：

- NV 侧同步内部使用 `CLOCK_MONOTONIC_RAW` 或等价单调时钟，不使用 wall clock。
- MCU 侧使用单调递增硬件 timer，不使用会被校时或重置影响的软时间。
- MCU 收包后尽快记录 `t2_mcu_tick`。
- MCU 发回包前尽量贴近串口写入动作记录 `t3_mcu_tick`。
- NV 收到完整回复包后立即记录 `t4_nv_ns`。

实现落地时建议进一步约束：

- `t1_nv_ns` 在 NV 侧持有串口发送锁并即将写入 sync 请求 frame 前记录。
- `t4_nv_ns` 在 RX 线程完成 sync 回复包 checksum 校验后立即记录，不能等到业务线程从 queue 中取出响应后再记录。
- `t2_mcu_tick` 初版可在 MCU 确认收到完整 sync 请求并校验通过后立即记录；如果 UART 驱动能提供首字节或 DMA 接收完成中断时间，则优先使用更靠近实际到达的时间。
- `t3_mcu_tick` 在 MCU 即将提交 sync 回复 frame 到 UART 写接口前记录；如果 UART 写接口只是入队，也应记录入队前时间，并在日志中保留实现语义。
- 同步命令需要带 `seq` 并和普通控制命令区分，避免 IMU push frame 与 sync response 在 NV 侧响应队列中混淆。

同步包返回内容至少包含：

```json
{
  "type": "time_sync_resp",
  "seq": 123,
  "t2_mcu_tick": 1000000,
  "t3_mcu_tick": 1000200
}
```

NV 本地为同一个 `seq` 保存：

```json
{
  "seq": 123,
  "t1_nv_ns": 987654321000,
  "t4_nv_ns": 987655421000
}
```

## Offset 估计

若 MCU tick 已可换算为 MCU 微秒时间，则一次同步的近似计算为：

```text
mcu_mid_us = (mcu_us(t2_mcu_tick) + mcu_us(t3_mcu_tick)) / 2
nv_mid_us  = (t1_nv_ns / 1000 + t4_nv_ns / 1000) / 2
offset_us  = nv_mid_us - mcu_mid_us

rtt_us = (t4_nv_ns - t1_nv_ns) / 1000 - (mcu_us(t3_mcu_tick) - mcu_us(t2_mcu_tick))
```

其中：

- `offset_us` 表示本轮估计出的 MCU 到 NV 时间轴偏移。
- `rtt_us` 表示扣除 MCU 内部处理耗时后的往返链路耗时。

实际使用时，不建议简单平均所有同步包。推荐每次校准发送多次 sync，例如 10 到 30 次，然后选择 RTT 最小的若干样本估计 offset。

原因是：

- RTT 越小，通常说明本轮串口排队、Linux 调度和 MCU 内部阻塞越少。
- 高 RTT 样本更可能包含偶发排队或调度抖动。
- 使用低 RTT 样本可以降低同步误差。

需要注意，四时间戳公式隐含“上下行链路延迟近似对称”的假设。UART 请求包和回复包长度可能不同，持续 IMU push 也可能插队造成固定偏差或偶发偏差。建议：

- 记录 sync 请求和回复 frame 的字节数、波特率和理论线传时间。
- 同步轮期间尽量降低或短暂停止 IMU push，至少避免大 payload 连续占用串口。
- 如果不能暂停 IMU push，离线分析时把 sync RTT、当时 IMU push 频率和串口利用率一起记录。
- 对固定线传长度差造成的偏差，可以按 `byte_count * 10 / baudrate` 粗略估算并纳入误差预算。

## 时间映射策略

### 初版策略

初版实现可以采用：

```text
nv_time_us = mcu_tick / ticks_per_us + offset_us
```

其中 `ticks_per_us` 来自 MCU timer 标称频率。

每隔一段时间重新同步一次，例如：

- 采集开始前连续同步 10 到 30 次，建立初始 offset。
- 采集过程中每 5 到 30 秒同步一轮。
- 每轮选择 RTT 最小的若干样本更新 offset。

更新 offset 时不建议突然大跳，除非确认当前映射明显失效。可以记录新旧 offset 差值，用于诊断时钟漂移和同步质量。

### 长时间漂移修正

如果 30 到 60 分钟长采集发现 offset 持续单向漂移，说明 MCU timer 与 NV 时钟存在频率差。

此时应从“只估 offset”升级为拟合线性映射：

```text
nv_time_us = a * mcu_tick + b
```

做法：

- 保存多轮低 RTT 同步样本。
- 使用样本中的 `mcu_mid_tick` 和 `nv_mid_us` 拟合 `a`、`b`。
- 对异常高 RTT 样本做过滤，不参与拟合。
- 拟合结果用于后续 IMU 样本时间戳换算。

### Tick 位宽和回绕

协议层优先使用 64 位 `mcu_tick_us` 或 64 位原始 tick。这样可以降低回绕处理复杂度。

如果 MCU 固件只能发送 32 位 tick，NV 侧必须做 unwrap：

- 协议中明确 `mcu_tick` 的单位、频率和是否从上电开始计数。
- NV 对每个数据源维护上一帧 tick，检测 32 位回绕并扩展为本地 64 位 tick。
- 32 位微秒 tick 约 71.6 分钟回绕，已经接近 30 到 60 分钟长时测试范围，不能忽略。
- 如果发送的是硬件 timer 原始 tick，还需要记录 `ticks_per_us` 或 timer clock Hz。

## IMU 数据格式建议

后续 IMU JSON 行建议保留现有字段，并新增时间相关字段。

建议字段：

```json
{
  "timestamp_us": 1760000000123456,
  "rx_timestamp_us": 1760000000126789,
  "mcu_tick": 1234567890,
  "mcu_tick_by_addr": {
    "0x50": 1234567890,
    "0x51": 1234568010
  },
  "sync_quality": {
    "offset_us": 1234,
    "rtt_us": 2200,
    "sync_seq": 100,
    "mapping_version": 3
  },
  "poses_by_addr": {},
  "acc_by_addr": {},
  "gyro_by_addr": {},
  "quaternion_by_addr": {},
  "sample_timestamp_by_addr": {}
}
```

字段语义：

- `timestamp_us`：根据 `mcu_tick -> nv_time_us` 映射换算出的 SOC 时间戳，后续对齐视频优先使用它。
- `rx_timestamp_us`：NV 实际收到并解析 IMU 包的时间，用于诊断串口延迟。
- `mcu_tick`：MCU 给该 IMU 样本打的本地时间戳。
- `mcu_tick_by_addr`：当一包中包含多个 IMU 样本时，记录每个地址各自的 MCU 本地时间戳。多 IMU 顺序读取时不建议只保留一个顶层 `mcu_tick`。
- `sync_quality.offset_us`：生成该 `timestamp_us` 时使用的 offset 或当前映射偏移。
- `sync_quality.rtt_us`：最近一次参与映射更新的同步 RTT。
- `sync_quality.sync_seq`：最近一次参与映射更新的同步序号。
- `sync_quality.mapping_version`：NV 本地映射版本号，便于离线分析不同时间段的映射变化。

字段和当前工程语义建议如下：

- `ts_iso`：只作为兼容展示或 NV 接收时间的字符串表达，不参与 IMU/视频严肃对齐。
- `rx_timestamp_us`：NV 解析到完整 IMU frame 后立即记录的接收时间，建议和最终输出时间轴一致。
- `mcu_tick` / `mcu_tick_by_addr`：STM32 本地单调时间，来自 MCU 对 IMU 数据打点，不应由 NV 侧生成。
- `sample_timestamp_by_addr`：保留 IMU 模组内部样本时间戳或诊断时间戳；除非确认其频率、零点、回绕和采样语义，否则不要直接等同于 `mcu_tick`。
- `timestamp_us`：由 NV 根据当前 `mcu_tick -> SOC time` 映射换算出的最终对齐时间戳。

关键要求：

- MCU 应在尽量靠近真实采样的位置打 `mcu_tick`。
- 如果 MCU 是收到 IMU 数据 ready 中断后读取数据，应在中断或读取完成附近记录 `mcu_tick`。
- 不建议在 MCU 准备把数据发给 NV 时才打 `mcu_tick`，否则时间戳会包含 MCU 内部排队延迟。
- 如果一包中聚合多个 IMU，MCU 应优先为每个 IMU record 单独携带 `mcu_tick`。只有能证明这些样本严格同一采样时刻时，才使用顶层公共 `mcu_tick`。

## 视频与 IMU 对齐策略

视频侧已有两类可用时间戳：

- `*_timestamps.csv` 中的 `wall_us`。
- MP4 H.264 SEI 中的帧时间戳。

对齐时建议：

1. 先确认 IMU `timestamp_us` 与视频 `wall_us` / SEI 使用同一 SOC 时间轴。
2. 以视频第一帧 `wall_us` 作为本次 session 的时间零点。
3. 对每条 IMU 样本使用 `timestamp_us` 计算相对时间。
4. 需要诊断延迟时，再查看 `rx_timestamp_us - timestamp_us`。

示例：

```text
video_t0_us = first_video_wall_us
video_frame_rel_ms = (frame_wall_us - video_t0_us) / 1000
imu_rel_ms = (imu.timestamp_us - video_t0_us) / 1000
imu_rx_delay_ms = (imu.rx_timestamp_us - imu.timestamp_us) / 1000
```

如果 `imu_rx_delay_ms` 基本稳定，说明串口链路和系统调度比较平稳。如果该值出现明显尖峰，应优先怀疑串口 buffer 堆积、MCU 发送排队或 NV 用户态调度延迟。

## 测试方案

### 1. 离线同步日志检查

目标：

- 确认四时间戳同步包能稳定往返。
- 统计 RTT、offset 和低 RTT 样本稳定性。

检查项：

- `seq` 是否连续。
- RTT 是否存在明显尖峰。
- 低 RTT 样本的 offset 是否稳定。
- offset 是否随时间单向漂移。

建议输出：

```text
sync_count
rtt_min_us
rtt_p50_us
rtt_p95_us
rtt_max_us
offset_min_us
offset_p50_us
offset_p95_us
offset_max_us
offset_drift_us_per_min
```

### 2. 短时采集测试

采集 1 到 3 分钟，检查：

- IMU `timestamp_us` 是否单调递增。
- IMU 采样间隔是否符合预期频率。
- `rx_timestamp_us - timestamp_us` 的分布是否稳定。
- 视频第一帧与 IMU 时间范围是否有合理交集。

通过口径：

- 无明显时间戳倒退。
- 无大段 IMU 时间戳空洞。
- `rx_timestamp_us - timestamp_us` 没有频繁大尖峰。

### 3. 长时漂移测试

采集 30 到 60 分钟，检查：

- 周期同步得到的 offset 是否持续单边漂移。
- 只估 `b` 是否足够。
- 是否需要启用 `a/b` 线性拟合。

通过口径：

- 若 offset 漂移在业务可接受范围内，初版固定 `a` 可继续使用。
- 若 offset 随时间稳定增长或减小，应升级为线性拟合。

### 4. 事件级验证

做 5 到 10 次明显事件，例如：

- 快速晃动 IMU。
- 轻敲 IMU 固定结构。
- 让相机能看到同一动作。

分析：

- 在 IMU 中找 gyro/acc 峰值时间 `timestamp_us`。
- 在视频中找动作首次出现或峰值帧时间 `wall_us`。
- 计算多次事件的 `video_event_us - imu_event_us`。

判断：

- 如果差值稳定，说明可用固定事件 offset 做业务解释。
- 如果差值抖动很大，应检查同步 RTT、IMU 打点位置和串口排队。

### 5. 硬件 marker 验证

如果需要证明严格同步，建议增加硬件 marker：

- MCU 控制 LED 闪烁，让 LED 入画。
- MCU 同时在 IMU 数据流中插入 marker。
- 视频侧检测 LED 首亮帧。
- IMU 侧读取 marker 的 `mcu_tick` / `timestamp_us`。

这种方式可以绕开“人体动作识别不准”和“峰值定义不一致”的问题，更适合做最终验收。

## 风险和注意事项

- 串口延迟不是绝对固定，只能通过低 RTT 筛选和周期校准降低影响。
- NV 用户态调度会造成偶发收包延迟，因此 `rx_timestamp_us` 不应作为最终采样时间戳。
- MCU 必须保证 `mcu_tick` 单调，并处理计数器回绕。
- 如果 MCU tick 频率不准，长时间采集一定会出现漂移，需要线性拟合修正。
- 如果 IMU 数据在 MCU 内部已经排队很久才被打时间戳，时间同步本身无法修复这个误差。

## 当前结论

该方案可行，并且比单纯用 NV 收包时间给 IMU 数据打时间戳更可靠。

建议实施顺序：

1. MCU 每条 IMU 样本携带本地 `mcu_tick`。
2. NV 增加四时间戳同步协议和同步日志。
3. NV 初版固定 `a`，用低 RTT 样本估计 `b`。
4. IMU JSON 增加 `timestamp_us`、`rx_timestamp_us`、`mcu_tick` 和同步质量字段。
5. 通过短时、长时、事件级和硬件 marker 测试验证对齐质量。
