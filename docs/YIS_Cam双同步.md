# YIS + Camera 双同步时间戳方案

## Summary

本方案用 MCU 微秒时间轴作为中间统一时间轴，把 YIS IMU 采样时间和相机触发时间都落到同一个 `mcu_tick_us` 上，再由 NV 侧通过 `CMD_TIME_SYNC` 映射到 NV monotonic/wall 时间轴。

最终 NV 落盘和视频对齐只使用 `timestamp_us`。`mcu_tick_us`、`sample_timestamp`、`readout_mcu_tick_us` 等字段只作为中间量或诊断字段保留。

## 同步信号

- `TIM5_IMU_SYNC_1HZ`
  - 1Hz 同步脉冲。
  - 作为 YIS `sample_timestamp` 的 epoch 锚点。
  - 每次事件记录：
    ```text
    source = TIM5_IMU_SYNC_1HZ
    sequence
    mcu_tick_us
    nominal_period_us = 1000000
    ```

- `TIM2_CAMERA_TRIGGER_30HZ`
  - 约 30Hz 相机触发脉冲。
  - 事件时间 `mcu_tick_us` 作为相机曝光上升沿时间。
  - 每次事件记录：
    ```text
    source = TIM2_CAMERA_TRIGGER_30HZ
    sequence
    mcu_tick_us
    nominal_period_us = 33333
    ```

## YIS 采样时间

YIS 原始 `sample_timestamp` 表示当前 1Hz 同步周期内的微秒偏移。正式采样时间不用 PA1 DR/读取时间，而是用最新 TIM5 epoch 重建：

```text
sensor_mcu_tick_us = latest_TIM5_epoch_mcu_tick_us
                   + sample_timestamp
                   + yis_epoch_offset_us
```

字段含义：

```text
sample_timestamp       YIS 周期内原始微秒偏移
sensor_mcu_tick_us     正式 YIS 采样时间，NV 对齐优先使用
readout_mcu_tick_us    PA1 DR/读取附近时间，仅用于诊断
time_status            bit0=has_epoch, bit1=sample_wrap_seen, bit2=epoch_mismatch
```

当还没有 TIM5 epoch 时：

```text
sensor_mcu_tick_us = 0
time_status.has_epoch = 0
```

NV 不应为这类 YIS 样本生成正式 `timestamp_us`。

## UART 事件与姿态数据

同步事件通过 `CMD_SYNC_EVENT_PUSH = 0x32` 推送：

```text
uint8 count
repeated:
  uint8  source
  uint8  flags               // bit0=overflow_seen
  uint16 reserved
  uint32 sequence
  uint64 mcu_tick_us
  uint32 nominal_period_us
  uint32 dropped_count
```

YIS 姿态 record 保留旧版兼容，同时新版携带：

```text
sample_timestamp
sensor_mcu_tick_us
readout_mcu_tick_us
time_status
```

## NV 使用规则

NV 侧仍用 `CMD_TIME_SYNC=0x02` 建立 MCU 时间轴到 NV 时间轴的映射：

```text
mcu_tick_us -> timestamp_mono_us -> timestamp_us
```

YIS 数据：

- 若 `sensor_mcu_tick_us != 0`，用 `sensor_mcu_tick_us` 映射生成 `imus[].timestamp_us`。
- 禁止使用裸 `sample_timestamp` 直接做 NV 时间映射。
- `readout_mcu_tick_us` 只作为诊断字段输出，不作为正式采样时间。

Camera 数据：

- 使用 `TIM2_CAMERA_TRIGGER_30HZ` 事件的 `mcu_tick_us` 映射生成相机触发 `timestamp_us`。
- `sync_events.csv.timestamp_us` 可直接与视频侧 `wall_us` 对齐。

落盘字段：

```text
IMU JSON:
  sample_timestamp
  sensor_mcu_tick_us
  readout_mcu_tick_us
  timestamp_us
  sync_version
  time_status

sync_events.csv:
  source
  sequence
  mcu_tick_us
  timestamp_mono_us
  timestamp_us
  rx_timestamp_us
  nominal_period_us
  dropped_count
  flags
```

## 验收标准

- TIM5 同步事件约 1 条/秒，连续事件 `mcu_tick_us` 差值约 `1000000 us`。
- TIM2 相机触发事件约 30 条/秒，连续事件 `mcu_tick_us` 差值约 `33333 us`。
- `dropped_count` 长时间不增长，`flags` 不持续出现 overflow。
- YIS `sensor_mcu_tick_us` 跨秒单调递增。
- `sync-monitor` 完整周期稳定为：
  ```text
  frames,200
  sample_step_avg ~= 5000 us
  ```
- NV 输出中，正式对齐只依赖 `timestamp_us` 与视频 `wall_us`。

## Assumptions

- YIS `sample_timestamp` 是相对 TIM5 1Hz 同步周期的周期内微秒偏移。
- TIM5 上升沿是 YIS sample timestamp 的 epoch 参考点。
- `yis_epoch_offset_us` 初始为 0，后续可按实测校准。
- 相机曝光以 TIM2 上升沿作为触发参考。
- 200Hz YIS + 30Hz event 推送建议使用 `460800` 或 `921600` 波特率。
