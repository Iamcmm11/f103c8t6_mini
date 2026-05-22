# IMU 同步执行计划

## 当前工程状态和边界

当前工程里有两类 IMU 同步路径：

- YIS：已有外部脉冲/DR 相关路径。`app_main.cpp` 中 TIM5 CH1 已输出同步 PWM，PA6 作为 EXTI 输入交给 `YISIMUAcquisitionTask` 等待 data ready，因此 YIS 可以按硬件事件驱动采集。
- WIT/JY901B：当前由 `IMUManager::AcquisitionThreadFunc()` 用 `Thread::SleepUntil()` 周期性轮询 I2C，`ReadAll()` 里顺序读取多个 IMU，并在每个 IMU 读取完成后写 `timestamp_us`。这不是硬件同步，只是 RTOS 周期轮询。

WIT 如果没有外部 sync/data-ready 输入，就不能像 YIS 一样把 IMU 内部采样相位锁到 MCU 脉冲。我们本地侧能做的是：

- 用 MCU 硬件 timer 产生稳定采集节拍。
- 在 timer 事件附近触发 WIT 读取并记录精确 `mcu_tick`。
- 记录每个 WIT 读取的计划触发时间、实际读取开始/完成时间和 I2C 顺序偏移。
- 通过硬件 marker/LED 入画验证“MCU 时间轴 -> 视频时间轴”的对齐质量。

因此，WIT 第一版应定义为“硬件定时采集和打点”，而不是严格意义上的“IMU 内部硬同步”。如果业务必须要求 IMU 内部采样相位也被外部脉冲锁住，需要换成支持 sync/data-ready 的 IMU，或增加能输出/输入同步信号的硬件方案。

## WIT 本地侧需要新增内容

### 1. 硬件定时采集触发

目标是替换当前 `Thread::SleepUntil()` 周期轮询，让 WIT 采集由硬件 timer 节拍驱动。

建议：

- 复用或新增一个硬件 timer 作为 WIT 采集节拍源。
- timer 周期与 WIT 目标输出频率一致，例如 50 Hz 或 100 Hz。
- timer ISR 只做轻量动作：记录 `trigger_mcu_tick`，递增 `trigger_seq`，post semaphore 给 WIT 采集线程。
- WIT 采集线程收到 semaphore 后立即执行 `ReadAll()`。
- 如果 timer 事件堆积，记录 missed trigger 计数，不要默默丢失。

注意：

- TIM5 现在已经用于 PA0 同步 PWM。如果这个 PWM 仍要给 YIS 或视频 marker 使用，不建议直接把 TIM5 改成 WIT 采集调度，除非确认频率、占空比和中断用途都兼容。
- 可以让 TIM5 继续输出同步脉冲，同时在 TIM5 update/compare 事件里 post WIT semaphore；也可以新增独立 timer，避免互相耦合。

### 2. WIT 采样时间字段

WIT 每个 IMU record 至少新增：

```text
trigger_seq
trigger_mcu_tick_us
read_start_mcu_tick_us
read_done_mcu_tick_us
sample_mcu_tick_us
```

字段建议语义：

- `trigger_seq`：硬件 timer 采集节拍序号。
- `trigger_mcu_tick_us`：timer 事件对应的 MCU 单调时间。
- `read_start_mcu_tick_us`：开始读该 IMU I2C 前的 MCU 时间。
- `read_done_mcu_tick_us`：读完该 IMU 并完成数据转换后的 MCU 时间。
- `sample_mcu_tick_us`：对外用于同步映射的主时间戳。初版建议取 `read_done_mcu_tick_us`；如果后续测得 I2C 读取耗时稳定，可以改成 `read_start` 和 `read_done` 中点，或做固定延迟校正。

多 WIT 顺序读取时，必须保留 `mcu_tick_by_addr` 或每条 record 自带 `sample_mcu_tick_us`。不要只用一个顶层 `mcu_tick` 代表整包，因为 0x50 到 0x53 的读取时刻实际不同。

### 3. 数据结构和 Topic 扩展

本地数据结构需要扩展：

- `Manager::IMUData` 增加 64 位 `mcu_tick_us`，可选增加 `read_start_us` / `read_done_us`。
- `Manager::IMUArrayMsg` 增加 `trigger_seq`、`trigger_mcu_tick_us`、`missed_trigger_count`。
- `IMUManager::ReadAll()` 接收本轮 trigger 信息，或由采集线程在调用前后填充。
- 保留现有姿态、acc、gyro、quat 字段不变，避免影响上层消费者。

### 4. UART bridge 协议扩展

WIT 当前 bridge payload 主要是地址 + 姿态/四元数，缺少 MCU 时间字段。需要新增版本化 payload：

- 保留旧格式解析，新增 WIT timestamp record 格式。
- 每条 record 携带 `imu_addr`、姿态/四元数、`sample_mcu_tick_us`。
- 如果要诊断 WIT 顺序读取误差，可选携带 `read_start_us`、`read_done_us`、`trigger_seq`。
- Python 端解析后输出 `mcu_tick_by_addr`，并在只有单个 IMU 时可同时填顶层 `mcu_tick`。

建议不要把 YIS 的 `sample_timestamp` 复用为 WIT `mcu_tick`。WIT 的 `mcu_tick` 应由 STM32 本地时间源生成。

### 5. NV-MCU time sync 命令

本地 MCU bridge 需要新增 time sync 命令：

- 新增命令号，例如 `CMD_TIME_SYNC = 0x02`，具体值只要不和现有 `ping=0x01`、灯控 `0x21`、IMU push `0x30` 冲突即可。
- 请求 payload：`seq`。
- MCU 收到完整请求并校验通过后记录 `t2_mcu_tick_us`。
- MCU 回复前记录 `t3_mcu_tick_us`。
- 回复 payload：`status`、`seq`、`t2_mcu_tick_us`、`t3_mcu_tick_us`、可选 `tick_freq_hz`。
- sync 回复应尽量优先于普通 IMU push，避免被连续大 payload 阻塞。

### 6. 硬件 marker

WIT 没有真正外部硬同步输入时，硬件 marker 是验收关键。

建议本地侧新增：

- 一个 MCU 控制的可见 LED 或 GPIO marker，最好由 timer 事件直接翻转或在高优先级任务里翻转。
- marker 事件同时写入 IMU 数据流，包含 `marker_seq` 和 `marker_mcu_tick_us`。
- 视频侧拍到 LED 首亮/翻转帧，用 `wall_us` 或 SEI 时间戳和 IMU marker 做对齐验证。

WS2812 可以作为可见 marker，但它有刷新耗时和状态机延迟。若要更精确，优先使用普通 GPIO LED。

### 7. 本地侧验收指标

WIT 本地侧先验收这些指标，再进入 NV 映射：

- timer trigger 周期稳定，`trigger_mcu_tick_us` 间隔符合目标频率。
- `read_start_mcu_tick_us - trigger_mcu_tick_us` 分布稳定。
- `read_done_mcu_tick_us - read_start_mcu_tick_us` 分布稳定。
- 0x50 到 0x53 的读取时间偏移可量化。
- 无持续 missed trigger。
- UART push 不会反向阻塞 WIT 采集线程。

阶段 1：MCU 先具备基础时间能力
MCU 侧开发：

提供单调递增 mcu_tick

使用硬件 timer。
明确 tick 频率，例如 1 MHz 或 10 MHz。
明确回绕周期和回绕处理方式。
保证采集期间不会重置。
IMU 每条样本携带 mcu_tick

打点位置尽量靠近真实采样时刻。
优先级：IMU data ready 中断时刻 > 读取完成时刻 > 打包发送时刻。
不建议在准备串口发送时才打时间戳。
IMU 数据包增加字段

mcu_tick
原有 IMU 姿态、acc、gyro、quat 数据保持不变。
如果现有协议已有 sample_timestamp，需要明确它是不是 MCU tick；如果不是，新增独立字段。
NV 侧开发：

接收脚本解析 mcu_tick
在 scripts/imu_uart_bridge_test.py 里扩展包解析。
先不做同步换算也可以，先把 mcu_tick 原样写进 JSON。
同时记录 rx_timestamp_us，即 NV 收到完整 IMU 包后的本地时间。
阶段 1 验收：

IMU JSON 中每条数据都有 mcu_tick。
mcu_tick 单调递增。
相邻样本 tick 差值符合 IMU 频率。
rx_timestamp_us 单调递增。
阶段 2：做 NV-MCU 四时间戳同步
MCU 侧开发：

支持 time sync 请求

NV 发 time_sync_req(seq)。
MCU 收到后立即记录 t2_mcu_tick。
MCU 回复前记录 t3_mcu_tick。
回复包含 seq/t2_mcu_tick/t3_mcu_tick。
保证 sync 回复优先级

尽量避免 sync 包被 IMU 大量数据长时间排队。
如果协议允许，sync 回复优先发送。
NV 侧开发：

增加同步请求逻辑

发送前记录 t1_nv_ns。
收到完整回复后记录 t4_nv_ns。
保存 seq/t1/t2/t3/t4。
计算同步质量

计算 RTT。
计算 offset。
每轮发 10 到 30 个 sync 包。
选 RTT 最小的若干个样本更新 offset。
写同步日志

建议落到 session 目录，例如 imu_time_sync.csv/jsonl。
字段包含 seq,t1_nv_ns,t2_mcu_tick,t3_mcu_tick,t4_nv_ns,rtt_us,offset_us,selected。
阶段 2 验收：

sync seq 连续。
RTT 有稳定低值。
低 RTT 样本 offset 稳定。
offset 没有明显随机大跳。
阶段 3：NV 侧生成可对齐时间戳
MCU 侧开发：

不需要新增大功能，主要保证 mcu_tick 和 sync 回复稳定。
NV 侧开发：

维护时间映射
初版：

timestamp_us = mcu_tick / ticks_per_us + offset_us
后续长时间漂移明显时升级为：

timestamp_us = a * mcu_tick + b
IMU JSON 输出新增字段
{
  "timestamp_us": 1760000000123456,
  "rx_timestamp_us": 1760000000126789,
  "mcu_tick": 1234567890,
  "sync_quality": {
    "offset_us": 1234,
    "rtt_us": 2200,
    "sync_seq": 100,
    "mapping_version": 3
  }
}
周期校准
采集开始前做一轮同步。
采集中每 5 到 30 秒做一轮同步。
每次更新映射时增加 mapping_version。
阶段 3 验收：

timestamp_us 单调递增。
rx_timestamp_us - timestamp_us 分布稳定。
IMU timestamp_us 和视频 wall_us 时间范围有合理交集。
不再依赖 ts_iso。
阶段 4：对齐验证工具
NV 侧开发：

写一个分析脚本

输入 session 目录。
读取视频 *_timestamps.csv。
读取 imu_uart_bridge.json。
输出：
第一帧视频时间。
第一条 IMU timestamp_us。
IMU pre-roll / late-start。
rx_timestamp_us - timestamp_us 统计。
IMU 采样间隔统计。
sync RTT / offset 统计。
可选输出 CSV/HTML 报告

方便和现有视频时间戳分析链路一起看。
MCU 侧开发：

暂无新增，除非发现 sync 抖动来自 MCU 排队。
阶段 4 验收：

1 到 3 分钟短采集报告正常。
30 到 60 分钟长采集没有明显漂移。
如果有漂移，NV 侧升级 a/b 线性拟合。
阶段 5：事件级 / 硬件 marker 验收
MCU 侧开发：

支持 marker

MCU 在 IMU 数据流中插入 marker 事件。
可选：同时控制 LED 闪烁。
LED 入画

MCU 控 LED，视频能拍到。
IMU 数据里同一时刻写 marker 和 mcu_tick。
NV 侧开发：

分析 marker
从 IMU 找 marker timestamp_us。
从视频找 LED 首亮帧 wall_us。
计算 video_marker_us - imu_marker_us。
阶段 5 验收：

多次 marker offset 稳定。
没有随时间单边漂移。
如果 offset 固定但非零，可以作为系统固定延迟记录。
如果 offset 抖动大，回查 sync RTT、MCU 打点位置、串口排队。
推荐分工总结

MCU 侧必须做：

mcu_tick。
每条 IMU 样本带 mcu_tick。
time sync 请求/回复，返回 t2_mcu_tick/t3_mcu_tick。
可选硬件 marker / LED 验收能力。
NV 侧必须做：

解析 mcu_tick。
记录 rx_timestamp_us。
实现四时间戳同步。
维护 mcu_tick -> timestamp_us 映射。
写出 timestamp_us/rx_timestamp_us/mcu_tick/sync_quality。
做 session 级分析和验收报告。
建议先不要一上来做复杂线性拟合。第一版先完成 mcu_tick + 四时间戳 + 低 RTT offset + timestamp_us，跑短时和长时测试后，再决定是否需要拟合 a/b。
