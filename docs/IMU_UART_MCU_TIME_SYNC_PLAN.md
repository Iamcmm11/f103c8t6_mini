# WIT UART MCU 时间同步方案

## 目的

本文档记录当前已经落地的 WIT MCU-NV 时间同步方案，以及最终对外的数据时间轴口径。

本文档描述的是当前实现，不再保留旧的“执行计划”内容。

## 当前实现总览

当前链路分为四段：

1. STM32 用硬件定时触发 WIT 采集。
2. STM32 为每个 WIT 样本打 `mcu_tick_us`。
3. STM32 通过 UART bridge 持续推送压缩后的 IMU 数据。
4. NV 侧通过 `CMD_TIME_SYNC=0x02` 周期性对时，把 `mcu_tick_us` 映射到最终的 `timestamp_us`。

当前 WIT 不是“传感器内部被外部同步脉冲锁相”的方案，而是：

- MCU 侧采集节拍由硬件定时器控制。
- 多个 WIT 仍然是顺序 I2C 读取。
- 每个 IMU 记录各自的 MCU 本地时间戳。

因此当前 `mcu_tick_us` 的语义是“MCU 侧读出该 IMU 数据时刻附近的本地单调时间”，不是 WIT 芯片内部原生采样时刻。

## MCU 侧方案

### 1. 采集触发

当前 `app_main.cpp` 中：

- `wit_acq_config.use_hardware_trigger = true`
- `HAL_TIM_Base_Start_IT(&htim6)`
- `TIM6` 周期回调里调用 `Manager::IMUManager::OnHardwareTriggerTimerInterrupt(true)`

也就是说，WIT 采集线程不是靠 `SleepUntil()` 自由轮询，而是由 `TIM6` 中断驱动。

### 2. 采集线程与触发元数据

`IMUManager` 内部维护以下触发信息：

- `trigger_sequence`
- `trigger_mcu_tick_us`
- `trigger_overrun_count`

每次硬件触发到来时：

- 记录一次 MCU 本地单调时间
- 增加触发序号
- 唤醒采集线程

随后采集线程执行 `ReadAll()`，生成一帧 `IMUArrayMsg`。

### 3. 每个 IMU 的时间戳

当前 `ReadAll()` 中，WIT 每个槽位在读完并转换完成后写入：

- `imu_data[i].timestamp_us = Timebase::GetMicroseconds()`
- `imu_data[i].mcu_tick_us = imu_data[i].timestamp_us`

当前实现里，这两个字段在 MCU 侧数值相同，都是 STM32 本地微秒单调时钟。

因此：

- `timestamp_us`：MCU 本地时间
- `mcu_tick_us`：MCU 本地时间

在 MCU 侧它们还不是 NV/SOC 时间轴。

### 4. 多 IMU 的时间关系

当前 WIT 是顺序 I2C 读取 6 路：

- `0x50`
- `0x51`
- `0x52`
- `0x53`
- `0x54`
- `0x55`

因此不能把一整包只看成一个统一采样时刻。当前实现已经保留了每个 IMU 各自的 `mcu_tick_us`。

## UART bridge 推送格式

### 1. Push 命令号

WIT push 走 `CMD_IMU_EULER_PUSH = 0x30`。

### 2. 当前 WIT 紧凑格式

当前 WIT push 使用紧凑格式：

- Header: `<BQ>`
  - `imu_count: uint8`
  - `base_tick_us: uint64`
- Record: `<B H hhh hhhh>`
  - `imu_addr: uint8`
  - `tick_delta_us: uint16`
  - `acc_x_mg, acc_y_mg, acc_z_mg: int16`
  - `quat_w_q15, quat_x_q15, quat_y_q15, quat_z_q15: int16`

NV 侧收到后重建：

- `record.mcu_tick_us = base_tick_us + tick_delta_us`

注意：

- 当前紧凑 push 不再直接携带 RPY 浮点角度。
- 当前主要携带的是地址、MCU 时间、加速度、四元数。

## NV 侧时间同步方案

## 1. 对时命令

NV 侧通过 `CMD_TIME_SYNC = 0x02` 发起对时。

请求：

- payload: `seq: uint32`

MCU 回复：

- `status`
- `seq`
- `t2_mcu_tick_us`
- `t3_mcu_tick_us`

其中：

- `t2_mcu_tick_us`：MCU 读完完整 sync 请求后的本地时间
- `t3_mcu_tick_us`：MCU 发出 sync 回复前的本地时间

### 2. 四时间戳模型

一次对时交互的四个时间戳是：

- `t1_nv_ns`：NV 发送 sync 请求前的单调时钟
- `t2_mcu_tick_us`：MCU 收到请求后的本地时钟
- `t3_mcu_tick_us`：MCU 发送回复前的本地时钟
- `t4_nv_ns`：NV 收到回复后的单调时钟

NV 侧当前计算：

```text
rtt_us = (t4_nv_ns - t1_nv_ns) / 1000
midpoint_nv_us = (t1_nv_ns + t4_nv_ns) / 2000
midpoint_mcu_us = (t2_mcu_tick_us + t3_mcu_tick_us) / 2
offset_us = midpoint_nv_us - midpoint_mcu_us
```

当前实现中，`mcu_tick_us` 已经是 MCU 微秒时间，所以当前没有再拟合比例项 `a`，默认斜率为 1。

### 3. 样本选择

一轮 burst 会发多次 sync 请求。当前 NV 侧选择：

- RTT 最小的那一条样本

并把它作为当前有效映射。

当前输出到同步 CSV 的核心字段有：

- `seq`
- `t1_nv_ns`
- `t2_mcu_tick_us`
- `t3_mcu_tick_us`
- `t4_nv_ns`
- `rtt_us`
- `offset_us`
- `selected`

## 时间轴约束

当前文档里的“时间轴约束”分两层：

- 同步计算内部时间轴：NV/SOC 的单调时钟轴
- 最终对外对齐时间轴：NV/SOC wall_us 轴

### 1. 同步计算内部时间轴

代码里当前用的是 `time.monotonic_ns()`，不是 wall clock。

同步 CSV 里的：

- `t1_nv_ns`
- `t4_nv_ns`
- `offset_us`

都是在这条轴上算的。

### 2. 最终对外对齐时间轴

最终对外对齐时间轴是 NV/SOC `wall_us` 轴，也就是视频 CSV 里的 `wall_us` 所在时间轴。

当前 IMU JSON 里的 `timestamp_us` 已经被换算到这条轴上，`sync_quality.time_axis` 也会写：

```json
"time_axis": "nv_wall_us"
```

### 3. 当前映射关系

当前映射关系是：

```text
mcu_to_mono_offset_us = selected_sync.offset_us

imu_mono_us = mcu_tick_us + mcu_to_mono_offset_us

mono_to_wall_us = anchor_wall_us - anchor_monotonic_ns / 1000

timestamp_us = imu_mono_us + mono_to_wall_us
```

当前代码层面的等价实现是：

- `mcu_to_mono_offset_us = mapping.sample.offset_us`
- `imu_mono_us = mcu_tick_us + mapping.sample.offset_us`
- 如果没有显式 anchor，则临时使用当前接收帧的  
  `mono_to_wall_us = rx_wall_us - rx_monotonic_ns / 1000`
- `timestamp_us = imu_mono_us + mono_to_wall_us`

所以结论是：现在落盘的 IMU `timestamp_us` 用的是 NV/SOC `wall_us` 时间轴，应该直接和视频 `*_timestamps.csv` 里的 `wall_us` 对齐。

## 当前落盘字段口径

当前 IMU JSON 里，和时间相关的字段应按下面理解。

### 正式对齐字段

正式对齐只用：

```text
imu.imus[].timestamp_us  <->  video_csv.wall_us
```

### 诊断字段

不要用这些字段做正式对齐：

- `rx_timestamp_us`
- `rx_delay_us`
- `ts_iso`
- `mcu_tick_us`
- `pts_ns`

它们的用途分别是：

- `rx_timestamp_us`：NV 收到并解析完整 IMU frame 的 wall_us 时间
- `rx_delay_us`：接收延迟/调度延迟诊断
- `ts_iso`：可读字符串时间
- `mcu_tick_us`：MCU 原始时间
- `pts_ns`：视频内部 PTS

### 当前推荐解释

- `timestamp_us`：最终用于和视频 `wall_us` 对齐的唯一正式时间戳
- `mcu_tick_us`：STM32 本地单调微秒时间，用于同步映射，不直接拿来和视频对齐
- `sync_quality.offset_us`：本次使用的 MCU->NV monotonic 偏移
- `sync_quality.mono_to_wall_us`：NV monotonic 到 NV wall 的桥接偏移
- `sync_quality.rtt_us`：最近一次被选中 sync 样本的 RTT
- `sync_quality.sync_seq`：最近一次被选中 sync 样本序号
- `sync_quality.mapping_version`：映射版本号

## 当前方案的限制

### 1. WIT 不是传感器内部硬同步

当前只能保证：

- MCU 侧采集节拍稳定
- 多 IMU 各自有独立 MCU 时间戳

不能保证：

- WIT 内部 ADC/滤波/输出相位被外部同步脉冲真正锁定

### 2. 多 IMU 仍然是顺序读

因此：

- 6 路 IMU 的 `mcu_tick_us` 不会完全相同
- 分析时应按每个 IMU 自己的 `timestamp_us` 使用

### 3. 当前未做长时间线性拟合

当前只做 offset 映射，没有拟合：

```text
nv_time_us = a * mcu_tick_us + b
```

如果后续长时间采集发现 STM32 与 NV 存在明显时钟漂移，再考虑引入 `a/b` 拟合。

## 文档整理说明

旧文档 [IMU同步执行计划.md](./IMU同步执行计划.md) 中的计划性内容已经并入本文档。后续以本文档为准。
