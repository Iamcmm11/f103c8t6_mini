#!/usr/bin/env python3
"""
Host-side tool for the STM32 UART bridge.

Run from repo root with utils/python path:
  python utils/python/imu_uart_bridge_test.py --port COM13 ping  python utils/python/imu_uart_bridge_test.py --port COM13 rgb 135 206 250
  python utils/python/imu_uart_bridge_test.py --port COM13 led 3 255 0 0
  python utils/python/imu_uart_bridge_test.py --port COM13 off
  python utils/python/imu_uart_bridge_test.py --port COM13 blink 135 206 250 --delay-ms 300
  python utils/python/imu_uart_bridge_test.py --port COM13 led-blink 3 255 0 0 --delay-ms 300
  python utils/python/imu_uart_bridge_test.py --port COM13 console
  python utils/python/imu_uart_bridge_test.py --port COM13 --baud 115200 sync-monitor
Run from repo root with scripts path:
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 ping
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 rgb 135 206 250
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 led 3 254 0 0
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 
  
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 blink 135 206 250 --delay-ms 300
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 led-blink 3 255 0 0 --delay-ms 300
  python scripts/imu_uart_bridge_test.py --port /dev/ttyTHS1 console
  
  python utils/python/imu_uart_bridge_test.py --port /dev/ttyTHS1 --baud 115200 sync-monitor

Short soft-sync capture test:
  mkdir -p sessions/imu_softsync_test log
  python3 scripts/imu_uart_bridge_test.py \
    --port /dev/ttyTCU0 \
    --baud 115200 \
    capture \
    --output sessions/imu_softsync_test/imu_uart_bridge.json \
    --sync-output log/imu_softsync_test_time_sync.csv \
    --sync-burst-count 10

""" 

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime
from typing import Callable, Optional

try:
    import serial
    from serial import Serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: pyserial\n"
        "Install with: pip install pyserial"
    ) from exc


SOF0 = 0x55
SOF1 = 0xAA

CMD_PING = 0x01
CMD_TIME_SYNC = 0x02
CMD_WS2812_CONTROL = 0x21
CMD_IMU_EULER_PUSH = 0x30
CMD_IMU_DIAG_PUSH = 0x31
CMD_SYNC_EVENT_PUSH = 0x32

STATUS_OK = 0
DEFAULT_LED_COUNT = 21
TARGET_ALL_LEDS = 0xFF
FLAG_BLINK_ENABLE = 0x01
MIN_BLINK_INTERVAL_MS = 20
MAX_BLINK_INTERVAL_MS = 0xFFFF
MAX_FRAME_PAYLOAD = 1024
IMU_PUSH_LEGACY_RECORD_SIZE = struct.calcsize("<Bfff")
IMU_PUSH_POSE_FLOAT_COUNT = 7
IMU_PUSH_POSE_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT)
)
IMU_PUSH_WIT_POSE_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "Q"
)
IMU_PUSH_WIT_COMPACT_HEADER_SIZE = struct.calcsize("<BQ")
IMU_PUSH_WIT_COMPACT_RECORD_SIZE = struct.calcsize("<B H hhh hhhh")
IMU_PUSH_YIS_POSE_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "I"
)
IMU_PUSH_YIS_EXTENDED_POSE_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "IQQB"
)
SYNC_EVENT_RECORD_SIZE = struct.calcsize("<BBHIQII")
IMU_PUSH_EXTENDED_FLOAT_COUNT = 13
IMU_PUSH_EXTENDED_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_EXTENDED_FLOAT_COUNT)
)
DEFAULT_IMU_ADDR_COLUMNS = [0x50, 0x51, 0x52, 0x53, 0x54, 0x55]
ANSI_RESET = "\033[0m"
ANSI_ADDR_COLOR = {
    0x50: "\033[91m",  # red
    0x51: "\033[94m",  # blue
    0x52: "\033[92m",  # green
    0x53: "\033[93m",  # yellow
    0x54: "\033[95m",  # magenta
    0x55: "\033[96m",  # cyan
}
SYNC_EVENT_SOURCE_NAMES = {
    1: "TIM2_IMU_SYNC_1HZ",
    2: "TIM5_CAMERA_TRIGGER_30HZ",
}


@dataclass(frozen=True)
class IMUPushRecord:
    imu_addr: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    mcu_tick_us: Optional[int] = None
    sample_timestamp: Optional[int] = None
    sensor_mcu_tick_us: Optional[int] = None
    readout_mcu_tick_us: Optional[int] = None
    time_status: Optional[int] = None
    acc_x: float = math.nan
    acc_y: float = math.nan
    acc_z: float = math.nan
    gyro_x: float = math.nan
    gyro_y: float = math.nan
    gyro_z: float = math.nan
    quat_w: float = math.nan
    quat_x: float = math.nan
    quat_y: float = math.nan
    quat_z: float = math.nan

    def has_motion_data(self) -> bool:
        return not math.isnan(self.acc_x)

    def has_quaternion(self) -> bool:
        return not math.isnan(self.quat_w)


@dataclass(frozen=True)
class SyncEventRecord:
    source: int
    flags: int
    sequence: int
    mcu_tick_us: int
    nominal_period_us: int
    dropped_count: int


@dataclass
class SyncCycleStats:
    imu_addr: int
    cycle_index: int = 0
    frame_count: int = 0
    start_host_ms: Optional[int] = None
    prev_host_ms: Optional[int] = None
    start_sample_ts: Optional[int] = None
    prev_sample_ts: Optional[int] = None
    min_sample_ts: Optional[int] = None
    max_sample_ts: Optional[int] = None
    sample_step_sum: int = 0
    sample_step_count: int = 0
    sample_step_min: Optional[int] = None
    sample_step_max: Optional[int] = None
    host_step_sum_ms: int = 0
    host_step_count: int = 0
    host_step_min_ms: Optional[int] = None
    host_step_max_ms: Optional[int] = None

    def update(self, host_ms: int, sample_ts: int) -> None:
        if self.frame_count == 0:
            self.start_host_ms = host_ms
            self.start_sample_ts = sample_ts
            self.min_sample_ts = sample_ts
            self.max_sample_ts = sample_ts
        else:
            assert self.prev_sample_ts is not None
            assert self.prev_host_ms is not None
            sample_step = sample_ts - self.prev_sample_ts
            host_step_ms = host_ms - self.prev_host_ms
            self.sample_step_sum += sample_step
            self.sample_step_count += 1
            self.host_step_sum_ms += host_step_ms
            self.host_step_count += 1
            if self.sample_step_min is None or sample_step < self.sample_step_min:
                self.sample_step_min = sample_step
            if self.sample_step_max is None or sample_step > self.sample_step_max:
                self.sample_step_max = sample_step
            if self.host_step_min_ms is None or host_step_ms < self.host_step_min_ms:
                self.host_step_min_ms = host_step_ms
            if self.host_step_max_ms is None or host_step_ms > self.host_step_max_ms:
                self.host_step_max_ms = host_step_ms
            if self.min_sample_ts is None or sample_ts < self.min_sample_ts:
                self.min_sample_ts = sample_ts
            if self.max_sample_ts is None or sample_ts > self.max_sample_ts:
                self.max_sample_ts = sample_ts

        self.frame_count += 1
        self.prev_host_ms = host_ms
        self.prev_sample_ts = sample_ts

    def format_summary(self) -> str:
        host_span_ms = 0
        if self.start_host_ms is not None and self.prev_host_ms is not None:
            host_span_ms = self.prev_host_ms - self.start_host_ms
        sample_step_avg = (
            self.sample_step_sum / self.sample_step_count
            if self.sample_step_count > 0
            else 0.0
        )
        host_step_avg_ms = (
            self.host_step_sum_ms / self.host_step_count
            if self.host_step_count > 0
            else 0.0
        )
        return (
            f"sync_cycle,addr,0x{self.imu_addr:02X},"
            f"idx,{self.cycle_index},"
            f"frames,{self.frame_count},"
            f"host_span_ms,{host_span_ms},"
            f"sample_ts_start,{self.start_sample_ts},"
            f"sample_ts_min,{self.min_sample_ts},"
            f"sample_ts_max,{self.max_sample_ts},"
            f"sample_step_avg,{sample_step_avg:.2f},"
            f"sample_step_min,{self.sample_step_min},"
            f"sample_step_max,{self.sample_step_max},"
            f"host_step_avg_ms,{host_step_avg_ms:.2f},"
            f"host_step_min_ms,{self.host_step_min_ms},"
            f"host_step_max_ms,{self.host_step_max_ms}"
        )

    def reset_for_next_cycle(self) -> None:
        self.cycle_index += 1
        self.frame_count = 0
        self.start_host_ms = None
        self.prev_host_ms = None
        self.start_sample_ts = None
        self.prev_sample_ts = None
        self.min_sample_ts = None
        self.max_sample_ts = None
        self.sample_step_sum = 0
        self.sample_step_count = 0
        self.sample_step_min = None
        self.sample_step_max = None
        self.host_step_sum_ms = 0
        self.host_step_count = 0
        self.host_step_min_ms = None
        self.host_step_max_ms = None


@dataclass(frozen=True)
class RXMetadata:
    rx_wall_us: int
    rx_monotonic_ns: int


@dataclass(frozen=True)
class TimeSyncSample:
    seq: int
    t1_nv_ns: int
    t2_mcu_tick_us: int
    t3_mcu_tick_us: int
    t4_nv_ns: int
    rtt_us: int
    offset_us: int
    selected: bool = False


@dataclass(frozen=True)
class TimeSyncMapping:
    sample: TimeSyncSample
    mapping_version: int
    mono_to_wall_us: Optional[int] = None
    frozen: bool = False


class TimeSyncMapper:
    def __init__(
        self,
        *,
        mono_to_wall_us: Optional[int] = None,
        frozen: bool = False,
    ) -> None:
        self._lock = threading.Lock()
        self._mapping: Optional[TimeSyncMapping] = None
        self._mono_to_wall_us = mono_to_wall_us
        self._frozen = frozen

    def update(
        self,
        sample: TimeSyncSample,
        *,
        mono_to_wall_us: Optional[int] = None,
        frozen: Optional[bool] = None,
    ) -> TimeSyncMapping:
        with self._lock:
            version = 1 if self._mapping is None else self._mapping.mapping_version + 1
            if mono_to_wall_us is not None:
                self._mono_to_wall_us = mono_to_wall_us
            if frozen is not None:
                self._frozen = frozen
            self._mapping = TimeSyncMapping(
                sample=sample,
                mapping_version=version,
                mono_to_wall_us=self._mono_to_wall_us,
                frozen=self._frozen,
            )
            return self._mapping

    def get(self) -> Optional[TimeSyncMapping]:
        with self._lock:
            return self._mapping

    def map_to_wall_us(
        self,
        mcu_tick_us: Optional[int],
        rx_meta: Optional[RXMetadata],
    ) -> tuple[Optional[int], Optional[dict]]:
        timestamp_mono_us, timestamp_wall_us, quality = self.map_to_times(
            mcu_tick_us, rx_meta
        )
        return timestamp_wall_us, quality

    def map_to_times(
        self,
        mcu_tick_us: Optional[int],
        rx_meta: Optional[RXMetadata],
    ) -> tuple[Optional[int], Optional[int], Optional[dict]]:
        if mcu_tick_us is None:
            return None, None, None

        mapping = self.get()
        if mapping is None:
            return None, None, None

        mono_to_wall_us = mapping.mono_to_wall_us
        if mono_to_wall_us is None:
            if rx_meta is None:
                return None, None, None
            mono_to_wall_us = rx_meta.rx_wall_us - (rx_meta.rx_monotonic_ns // 1000)
        timestamp_mono_us = mcu_tick_us + mapping.sample.offset_us
        timestamp_wall_us = timestamp_mono_us + mono_to_wall_us
        quality = {
            "offset_us": mapping.sample.offset_us,
            "mcu_to_mono_offset_us": mapping.sample.offset_us,
            "mono_to_wall_us": mono_to_wall_us,
            "rtt_us": mapping.sample.rtt_us,
            "sync_seq": mapping.sample.seq,
            "mapping_version": mapping.mapping_version,
            "time_axis": "nv_wall_us",
            "frozen": mapping.frozen,
        }
        return timestamp_mono_us, timestamp_wall_us, quality


def parse_time_sync_response(payload: bytes) -> tuple[int, int, int]:
    if len(payload) == 1:
        raise ValueError(
            "unexpected time-sync response length: 1 "
            f"(raw_status=0x{payload[0]:02X}, likely unknown CMD_TIME_SYNC on MCU "
            "or MCU returned 1-byte error ACK)"
        )
    if len(payload) != 1 + 4 + 8 + 8:
        raise ValueError(
            f"unexpected time-sync response length: {len(payload)}, "
            f"payload={payload.hex(' ')}"
        )

    status, seq, t2_mcu_tick_us, t3_mcu_tick_us = struct.unpack("<BIQQ", payload)
    if status != STATUS_OK:
        raise RuntimeError(f"time-sync returned error status={status}")
    return seq, t2_mcu_tick_us, t3_mcu_tick_us


def send_time_sync(client: "BridgeClient", seq: int) -> TimeSyncSample:
    t1_nv_ns = time.monotonic_ns()
    payload, rx_meta = client.request_with_rx_meta(
        CMD_TIME_SYNC,
        struct.pack("<I", seq & 0xFFFFFFFF),
        timeout=1.5,
    )
    t4_nv_ns = rx_meta.rx_monotonic_ns
    resp_seq, t2_mcu_tick_us, t3_mcu_tick_us = parse_time_sync_response(payload)
    if resp_seq != (seq & 0xFFFFFFFF):
        raise ValueError(f"time-sync seq mismatch: req={seq}, resp={resp_seq}")

    rtt_us = max(0, (t4_nv_ns - t1_nv_ns) // 1000)
    midpoint_nv_us = (t1_nv_ns + t4_nv_ns) // 2000
    midpoint_mcu_us = (t2_mcu_tick_us + t3_mcu_tick_us) // 2
    offset_us = int(midpoint_nv_us - midpoint_mcu_us)
    return TimeSyncSample(
        seq=resp_seq,
        t1_nv_ns=t1_nv_ns,
        t2_mcu_tick_us=t2_mcu_tick_us,
        t3_mcu_tick_us=t3_mcu_tick_us,
        t4_nv_ns=t4_nv_ns,
        rtt_us=int(rtt_us),
        offset_us=offset_us,
    )


class TimeSyncSession:
    def __init__(
        self,
        client: "BridgeClient",
        sync_mapper: TimeSyncMapper,
        csv_path: str,
        *,
        burst_count: int,
        interval_s: float,
    ) -> None:
        self._client = client
        self._sync_mapper = sync_mapper
        self._csv_path = csv_path
        self._burst_count = burst_count
        self._interval_s = interval_s
        self._seq = 0
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._csv_lock = threading.Lock()
        self._csv_initialized = False
        self._next_run_monotonic: Optional[float] = None

    def start(self) -> None:
        csv_dir = os.path.dirname(self._csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        self._ensure_csv_initialized()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="time-sync", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._interval_s + 1.0))
            self._thread = None

    def run_burst(self) -> Optional[TimeSyncMapping]:
        self._ensure_csv_initialized()
        samples: list[TimeSyncSample] = []
        for _ in range(self._burst_count):
            seq = self._seq
            self._seq += 1
            try:
                sample = send_time_sync(self._client, seq)
            except TimeoutError:
                # Missing CMD_TIME_SYNC responses are tolerated during RTT/capture tests.
                pass
            else:
                samples.append(sample)
            if self._stop_event.is_set():
                break
            time.sleep(0.01)

        self._next_run_monotonic = time.monotonic() + self._interval_s
        if not samples:
            return None

        selected_index = min(range(len(samples)), key=lambda idx: samples[idx].rtt_us)
        mapping: Optional[TimeSyncMapping] = None
        for idx, sample in enumerate(samples):
            selected_sample = replace(sample, selected=(idx == selected_index))
            if selected_sample.selected:
                mapping = self._sync_mapper.update(selected_sample)
            self._append_csv_row(selected_sample)
        return mapping

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if self._next_run_monotonic is not None:
                while not self._stop_event.is_set() and time.monotonic() < self._next_run_monotonic:
                    time.sleep(0.1)
                if self._stop_event.is_set():
                    break
            try:
                self.run_burst()
            except Exception as exc:
                print(f"[bridge] time-sync error: {exc}", file=sys.stderr, flush=True)

    def _write_csv_header(self) -> None:
        csv_dir = os.path.dirname(self._csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        with self._csv_lock:
            with open(self._csv_path, "w", encoding="utf-8", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    [
                        "seq",
                        "t1_nv_ns",
                        "t2_mcu_tick_us",
                        "t3_mcu_tick_us",
                        "t4_nv_ns",
                        "rtt_us",
                        "offset_us",
                        "selected",
                    ]
                )
        self._csv_initialized = True

    def _ensure_csv_initialized(self) -> None:
        if not self._csv_initialized:
            self._write_csv_header()

    def _append_csv_row(self, sample: TimeSyncSample) -> None:
        with self._csv_lock:
            with open(self._csv_path, "a", encoding="utf-8", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    [
                        sample.seq,
                        sample.t1_nv_ns,
                        sample.t2_mcu_tick_us,
                        sample.t3_mcu_tick_us,
                        sample.t4_nv_ns,
                        sample.rtt_us,
                        sample.offset_us,
                        1 if sample.selected else 0,
                    ]
                )


def calc_sum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(cmd: int, payload: bytes) -> bytes:
    header = bytes(
        [
            SOF0,
            SOF1,
            cmd & 0xFF,
            len(payload) & 0xFF,
            (len(payload) >> 8) & 0xFF,
        ]
    )
    checksum = (calc_sum(header) + calc_sum(payload)) & 0xFF
    return header + payload + bytes([checksum])


def format_hex_bytes(data: bytes) -> str:
    return data.hex(" ")


def parse_byte(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 255:
        raise argparse.ArgumentTypeError(f"byte value out of range: {text}")
    return value


def parse_led_index(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value >= DEFAULT_LED_COUNT:
        raise argparse.ArgumentTypeError(
            f"LED index out of range: {text}, valid range is 0..{DEFAULT_LED_COUNT - 1}"
        )
    return value


def parse_interval_ms(text: str) -> int:
    value = int(float(text))
    if value < MIN_BLINK_INTERVAL_MS or value > MAX_BLINK_INTERVAL_MS:
        raise argparse.ArgumentTypeError(
            f"blink interval out of range: {text}, valid range is "
            f"{MIN_BLINK_INTERVAL_MS}..{MAX_BLINK_INTERVAL_MS} ms"
        )
    return value


def parse_console_delay_ms(
    tokens: list[str],
    delay_index: int,
    default_ms: int,
    usage: str,
) -> int:
    if len(tokens) == delay_index:
        return default_ms
    if len(tokens) == delay_index + 1:
        return parse_interval_ms(tokens[delay_index])
    if len(tokens) == delay_index + 2 and tokens[delay_index] == "--delay-ms":
        return parse_interval_ms(tokens[delay_index + 1])
    raise ValueError(f"usage: {usage}")


def build_light_control_payload(
    target: int,
    red: int,
    green: int,
    blue: int,
    *,
    blink_enable: bool,
    interval_ms: int = 0,
) -> bytes:
    flags = FLAG_BLINK_ENABLE if blink_enable else 0
    return struct.pack(
        "<BBBBBH",
        target & 0xFF,
        flags,
        red & 0xFF,
        green & 0xFF,
        blue & 0xFF,
        interval_ms & 0xFFFF,
    )


def format_addr_with_color(addr: int) -> str:
    text = f"0x{addr:02X}"
    color = ANSI_ADDR_COLOR.get(addr)
    if not color:
        return text
    return f"{color}{text}{ANSI_RESET}"


def _round_or_none(value: float, digits: int = 2) -> Optional[float]:
    if math.isnan(value):
        return None
    return round(value, digits)


def format_imu_json_row(
    ts_unix_ms: int,
    imu_records: list[IMUPushRecord],
    rx_meta: Optional[RXMetadata] = None,
    sync_mapper: Optional[TimeSyncMapper] = None,
) -> dict:
    imus: list[dict] = []

    for record in imu_records:
        tick = record.sensor_mcu_tick_us
        if tick is None:
            tick = record.mcu_tick_us

        timestamp_us = None
        timestamp_mono_us = None
        sync_quality = None
        if sync_mapper is not None:
            timestamp_mono_us, timestamp_us, sync_quality = sync_mapper.map_to_times(
                tick, rx_meta
            )

        imu = {
            "addr": f"0x{record.imu_addr:02X}",
        }
        if timestamp_us is not None:
            imu["timestamp_us"] = timestamp_us
            if rx_meta is not None:
                imu["rx_delay_us"] = rx_meta.rx_wall_us - timestamp_us
        if timestamp_mono_us is not None:
            imu["timestamp_mono_us"] = timestamp_mono_us
        if record.sensor_mcu_tick_us is not None:
            imu["sensor_mcu_tick_us"] = record.sensor_mcu_tick_us
        if record.readout_mcu_tick_us is not None:
            imu["readout_mcu_tick_us"] = record.readout_mcu_tick_us
        if record.mcu_tick_us is not None:
            imu["mcu_tick_us"] = record.mcu_tick_us
        if record.sample_timestamp is not None:
            imu["sample_timestamp"] = record.sample_timestamp
        if record.time_status is not None:
            imu["time_status"] = record.time_status
        if sync_quality is not None:
            sync_version = sync_quality.get("mapping_version")
            if sync_version is not None:
                imu["sync_version"] = sync_version

        rpy_deg = {
            "roll": _round_or_none(record.roll_deg),
            "pitch": _round_or_none(record.pitch_deg),
            "yaw": _round_or_none(record.yaw_deg),
        }
        rpy_deg = {key: value for key, value in rpy_deg.items() if value is not None}
        if rpy_deg:
            imu["rpy_deg"] = rpy_deg

        if record.has_quaternion():
            quat = {
                "w": _round_or_none(record.quat_w, 4),
                "x": _round_or_none(record.quat_x, 4),
                "y": _round_or_none(record.quat_y, 4),
                "z": _round_or_none(record.quat_z, 4),
            }
            imu["quat"] = {key: value for key, value in quat.items() if value is not None}

        if record.has_motion_data():
            acc = {
                "x": _round_or_none(record.acc_x),
                "y": _round_or_none(record.acc_y),
                "z": _round_or_none(record.acc_z),
            }
            gyro = {
                "x": _round_or_none(record.gyro_x),
                "y": _round_or_none(record.gyro_y),
                "z": _round_or_none(record.gyro_z),
            }
            acc = {key: value for key, value in acc.items() if value is not None}
            gyro = {key: value for key, value in gyro.items() if value is not None}
            if acc:
                imu["acc"] = acc
            if gyro:
                imu["gyro"] = gyro

        imus.append(imu)

    row: dict = {
        "ts_iso": datetime.fromtimestamp(ts_unix_ms / 1000.0).isoformat(timespec="milliseconds"),
        "imus": imus,
    }
    if rx_meta is not None:
        row["rx_timestamp_us"] = rx_meta.rx_wall_us

    return row


class BridgeClient:
    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float,
        *,
        print_imu: bool = True,
        imu_print_hz: float = 10.0,
        imu_record_callback: Optional[
            Callable[[int, list[IMUPushRecord], RXMetadata], None]
        ] = None,
        sync_event_callback: Optional[
            Callable[[int, list[SyncEventRecord], RXMetadata], None]
        ] = None,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._print_imu = print_imu
        self._imu_print_interval_s = (
            0.0 if imu_print_hz <= 0.0 else 1.0 / imu_print_hz
        )
        self._imu_record_callback = imu_record_callback
        self._sync_event_callback = sync_event_callback
        self._serial: Optional[Serial] = None
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._last_imu_print_s = 0.0
        self._last_diag_print_s = 0.0
        self._last_imu_line_len = 0
        self._response_queues = {
            CMD_PING: queue.Queue(),
            CMD_TIME_SYNC: queue.Queue(),
            CMD_WS2812_CONTROL: queue.Queue(),
        }

    def set_imu_record_callback(
        self,
        callback: Optional[
            Callable[[int, list[IMUPushRecord], RXMetadata], None]
        ],
    ) -> None:
        self._imu_record_callback = callback

    def set_sync_event_callback(
        self,
        callback: Optional[
            Callable[[int, list[SyncEventRecord], RXMetadata], None]
        ],
    ) -> None:
        self._sync_event_callback = callback

    def __enter__(self) -> "BridgeClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._serial is not None:
            return

        self._serial = serial.Serial(
            self._port,
            self._baud,
            timeout=0,  # non-blocking read loop
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

        self._end_imu_single_line()

        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def request(self, cmd: int, payload: bytes, timeout: float = 1.5) -> bytes:
        resp, _rx_meta = self.request_with_rx_meta(cmd, payload, timeout)
        return resp

    def request_with_rx_meta(
        self,
        cmd: int,
        payload: bytes,
        timeout: float = 1.5,
    ) -> tuple[bytes, RXMetadata]:
        if self._serial is None:
            raise RuntimeError("serial port is not open")

        with self._request_lock:
            resp_queue = self._response_queues.setdefault(cmd, queue.Queue())
            while True:
                try:
                    resp_queue.get_nowait()
                except queue.Empty:
                    break

            frame = build_frame(cmd, payload)
            with self._tx_lock:
                self._serial.write(frame)
                self._serial.flush()

            try:
                resp = resp_queue.get(timeout=timeout)
            except queue.Empty as exc:
                raise TimeoutError(f"timeout waiting for response to cmd=0x{cmd:02X}") from exc
            if isinstance(resp, tuple):
                return resp
            return resp, RXMetadata(
                rx_wall_us=time.time_ns() // 1000,
                rx_monotonic_ns=time.monotonic_ns(),
            )

    def _read_exact(self, size: int, timeout: Optional[float] = None) -> Optional[bytes]:
        if self._serial is None:
            return None

        deadline = time.monotonic() + (self._timeout if timeout is None else timeout)
        chunks = bytearray()

        while len(chunks) < size and not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None

            old_timeout = self._serial.timeout
            self._serial.timeout = remaining
            try:
                chunk = self._serial.read(size - len(chunks))
            finally:
                self._serial.timeout = old_timeout

            if not chunk:
                continue

            chunks.extend(chunk)

        if len(chunks) != size:
            return None

        return bytes(chunks)

    def _rx_loop(self) -> None:
        rx_buf = bytearray()
        while not self._stop_event.is_set():
            try:
                if self._serial is None:
                    return

                want = self._serial.in_waiting
                if want <= 0:
                    # Non-blocking poll, avoid busy-spin.
                    time.sleep(0.001)
                    continue
                chunk = self._serial.read(want)
                if not chunk:
                    continue
                rx_buf.extend(chunk)
                self._process_rx_buffer(rx_buf)
            except serial.SerialException:
                return
            except Exception as exc:
                print(f"[bridge] rx error: {exc}", file=sys.stderr, flush=True)

    def _process_rx_buffer(self, rx_buf: bytearray) -> None:
        while True:
            sof_index = rx_buf.find(bytes([SOF0, SOF1]))
            if sof_index < 0:
                # Keep one tail byte in case it's SOF0 for the next chunk.
                if len(rx_buf) > 1:
                    del rx_buf[:-1]
                return

            if sof_index > 0:
                del rx_buf[:sof_index]

            if len(rx_buf) < 5:
                return

            cmd = rx_buf[2]
            payload_len = rx_buf[3] | (rx_buf[4] << 8)
            if payload_len > MAX_FRAME_PAYLOAD:
                # Invalid header length, slide one byte and resync.
                del rx_buf[0]
                continue

            total_len = 5 + payload_len + 1
            if len(rx_buf) < total_len:
                return

            header = bytes(rx_buf[:5])
            payload = bytes(rx_buf[5 : 5 + payload_len])
            checksum = rx_buf[5 + payload_len]
            expect = (calc_sum(header) + calc_sum(payload)) & 0xFF
            if checksum != expect:
                # Slide one byte instead of dropping the whole candidate frame.
                del rx_buf[0]
                continue

            rx_meta = RXMetadata(
                rx_wall_us=time.time_ns() // 1000,
                rx_monotonic_ns=time.monotonic_ns(),
            )
            frame = bytes(rx_buf[:total_len])
            del rx_buf[:total_len]
            self._dispatch_frame(cmd, payload, rx_meta)

    def _read_frame(self) -> Optional[tuple[int, bytes]]:
        if self._serial is None:
            return None

        while not self._stop_event.is_set():
            first = self._read_exact(1)
            if first is None or len(first) == 0:
                return None
            if first[0] != SOF0:
                continue

            second = self._read_exact(1)
            if second is None or len(second) == 0:
                return None
            if second[0] != SOF1:
                continue

            rest = self._read_exact(3)
            if rest is None or len(rest) != 3:
                return None

            header = first + second + rest
            payload_len = header[3] | (header[4] << 8)
            if payload_len > MAX_FRAME_PAYLOAD:
                continue
            payload = self._read_exact(payload_len)
            if payload is None or len(payload) != payload_len:
                return None

            checksum_raw = self._read_exact(1)
            if checksum_raw is None or len(checksum_raw) != 1:
                return None

            checksum = checksum_raw[0]
            expect = (calc_sum(header) + calc_sum(payload)) & 0xFF
            if checksum != expect:
                continue

            return header[2], payload

        return None

    def _dispatch_frame(
        self,
        cmd: int,
        payload: bytes,
        rx_meta: RXMetadata,
    ) -> None:
        if cmd == CMD_IMU_EULER_PUSH:
            self._handle_imu_push(payload, rx_meta)
            return
        if cmd == CMD_IMU_DIAG_PUSH:
            self._handle_imu_diag_push(payload)
            return
        if cmd == CMD_SYNC_EVENT_PUSH:
            self._handle_sync_event_push(payload, rx_meta)
            return

        resp_queue = self._response_queues.setdefault(cmd, queue.Queue())
        resp_queue.put((payload, rx_meta))

    def _handle_imu_push(self, payload: bytes, rx_meta: RXMetadata) -> None:
        imu_records = self._parse_imu_push(payload)
        if imu_records is None:
            print(
                f"[bridge] invalid IMU push length: {len(payload)}",
                file=sys.stderr,
                flush=True,
            )
            return

        callback = self._imu_record_callback
        if callback is not None and imu_records:
            ts_unix_ms = rx_meta.rx_wall_us // 1000
            try:
                callback(ts_unix_ms, imu_records, rx_meta)
            except Exception as exc:
                print(f"[bridge] imu record callback error: {exc}", file=sys.stderr, flush=True)

        if self._print_imu and imu_records:
            now = time.monotonic()
            if (
                self._imu_print_interval_s > 0.0
                and (now - self._last_imu_print_s) < self._imu_print_interval_s
            ):
                return
            self._last_imu_print_s = now
            parts = ["imu_bundle", str(len(imu_records))]
            for record in imu_records:
                rpy_values = (record.roll_deg, record.pitch_deg, record.yaw_deg)
                has_rpy = not any(math.isnan(value) for value in rpy_values)
                parts.extend(
                    [
                        format_addr_with_color(record.imu_addr),
                    ]
                )
                if has_rpy:
                    parts.append(
                        f"rpy=({record.roll_deg:.3f},{record.pitch_deg:.3f},{record.yaw_deg:.3f})"
                    )
                if record.sample_timestamp is not None:
                    parts.append(f"sample_ts={record.sample_timestamp}")
                if record.sensor_mcu_tick_us is not None:
                    parts.append(f"sensor_mcu_tick_us={record.sensor_mcu_tick_us}")
                if record.readout_mcu_tick_us is not None:
                    parts.append(f"readout_mcu_tick_us={record.readout_mcu_tick_us}")
                if record.time_status is not None:
                    parts.append(f"time_status=0x{record.time_status:02X}")
                if record.has_motion_data():
                    parts.append(
                        f"acc=({record.acc_x:.3f},{record.acc_y:.3f},{record.acc_z:.3f})"
                    )
                    gyro_values = (record.gyro_x, record.gyro_y, record.gyro_z)
                    if not any(math.isnan(value) for value in gyro_values):
                        parts.append(
                            f"gyro=({record.gyro_x:.3f},{record.gyro_y:.3f},{record.gyro_z:.3f})"
                        )
                if record.has_quaternion():
                    parts.append(
                        f"quat=({record.quat_w:.4f},{record.quat_x:.4f},{record.quat_y:.4f},{record.quat_z:.4f})"
                    )
                if record.mcu_tick_us is not None:
                    parts.append(f"mcu_tick_us={record.mcu_tick_us}")
            line = ",".join(parts)
            self._print_imu_single_line(line)

    def _print_imu_single_line(self, line: str) -> None:
        extra = self._last_imu_line_len - len(line)
        if extra > 0:
            line = line + (" " * extra)
        sys.stdout.write("\r" + line)
        sys.stdout.flush()
        self._last_imu_line_len = len(line)

    def _end_imu_single_line(self) -> None:
        if self._last_imu_line_len > 0:
            sys.stdout.write("\n")
            sys.stdout.flush()
            self._last_imu_line_len = 0

    def _parse_imu_push(
        self, payload: bytes
    ) -> Optional[list[IMUPushRecord]]:
        if len(payload) == IMU_PUSH_LEGACY_RECORD_SIZE:
            return [self._unpack_imu_record(payload)]

        if len(payload) < 1:
            return None

        imu_count = payload[0]
        if (
            len(payload)
            == IMU_PUSH_WIT_COMPACT_HEADER_SIZE
            + imu_count * IMU_PUSH_WIT_COMPACT_RECORD_SIZE
        ):
            _count, base_tick_us = struct.unpack_from("<BQ", payload, 0)
            imu_records: list[IMUPushRecord] = []
            offset = IMU_PUSH_WIT_COMPACT_HEADER_SIZE
            for _ in range(imu_count):
                imu_records.append(
                    self._unpack_wit_compact_record(
                        payload[offset : offset + IMU_PUSH_WIT_COMPACT_RECORD_SIZE],
                        base_tick_us,
                    )
                )
                offset += IMU_PUSH_WIT_COMPACT_RECORD_SIZE
            return imu_records

        records_raw = payload[1:]
        if imu_count == 0:
            return []
        if len(records_raw) % imu_count != 0:
            return None

        record_size = len(records_raw) // imu_count
        if record_size not in (
            IMU_PUSH_LEGACY_RECORD_SIZE,
            IMU_PUSH_POSE_RECORD_SIZE,
            IMU_PUSH_WIT_POSE_RECORD_SIZE,
            IMU_PUSH_YIS_POSE_RECORD_SIZE,
            IMU_PUSH_YIS_EXTENDED_POSE_RECORD_SIZE,
            IMU_PUSH_EXTENDED_RECORD_SIZE,
        ):
            return None

        imu_records: list[IMUPushRecord] = []
        for offset in range(0, len(records_raw), record_size):
            imu_records.append(
                self._unpack_imu_record(records_raw[offset : offset + record_size])
            )
        return imu_records

    def _unpack_wit_compact_record(
        self, payload: bytes, base_tick_us: int
    ) -> IMUPushRecord:
        if len(payload) != IMU_PUSH_WIT_COMPACT_RECORD_SIZE:
            raise ValueError(f"unsupported compact imu record size: {len(payload)}")
        (
            imu_addr,
            tick_delta_us,
            acc_x_mg,
            acc_y_mg,
            acc_z_mg,
            quat_w_q15,
            quat_x_q15,
            quat_y_q15,
            quat_z_q15,
        ) = struct.unpack("<B H hhh hhhh", payload)
        gravity_mps2 = 9.80665
        return IMUPushRecord(
            imu_addr=imu_addr,
            roll_deg=math.nan,
            pitch_deg=math.nan,
            yaw_deg=math.nan,
            mcu_tick_us=base_tick_us + tick_delta_us,
            acc_x=acc_x_mg * gravity_mps2 / 1000.0,
            acc_y=acc_y_mg * gravity_mps2 / 1000.0,
            acc_z=acc_z_mg * gravity_mps2 / 1000.0,
            quat_w=quat_w_q15 / 32767.0,
            quat_x=quat_x_q15 / 32767.0,
            quat_y=quat_y_q15 / 32767.0,
            quat_z=quat_z_q15 / 32767.0,
        )

    def _unpack_imu_record(self, payload: bytes) -> IMUPushRecord:
        if len(payload) == IMU_PUSH_LEGACY_RECORD_SIZE:
            imu_addr, roll_deg, pitch_deg, yaw_deg = struct.unpack("<Bfff", payload)
            return IMUPushRecord(
                imu_addr=imu_addr,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
            )

        if len(payload) == IMU_PUSH_POSE_RECORD_SIZE:
            values = struct.unpack(
                "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT),
                payload,
            )
            return IMUPushRecord(
                imu_addr=values[0],
                roll_deg=values[1],
                pitch_deg=values[2],
                yaw_deg=values[3],
                quat_w=values[4],
                quat_x=values[5],
                quat_y=values[6],
                quat_z=values[7],
            )

        if len(payload) == IMU_PUSH_WIT_POSE_RECORD_SIZE:
            values = struct.unpack(
                "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "Q",
                payload,
            )
            return IMUPushRecord(
                imu_addr=values[0],
                roll_deg=values[1],
                pitch_deg=values[2],
                yaw_deg=values[3],
                quat_w=values[4],
                quat_x=values[5],
                quat_y=values[6],
                quat_z=values[7],
                mcu_tick_us=values[8],
            )

        if len(payload) == IMU_PUSH_YIS_POSE_RECORD_SIZE:
            values = struct.unpack(
                "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "I",
                payload,
            )
            return IMUPushRecord(
                imu_addr=values[0],
                roll_deg=values[1],
                pitch_deg=values[2],
                yaw_deg=values[3],
                quat_w=values[4],
                quat_x=values[5],
                quat_y=values[6],
                quat_z=values[7],
                sample_timestamp=values[8],
            )

        if len(payload) == IMU_PUSH_YIS_EXTENDED_POSE_RECORD_SIZE:
            values = struct.unpack(
                "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "IQQB",
                payload,
            )
            sensor_mcu_tick_us = values[9] if values[9] != 0 else None
            return IMUPushRecord(
                imu_addr=values[0],
                roll_deg=values[1],
                pitch_deg=values[2],
                yaw_deg=values[3],
                quat_w=values[4],
                quat_x=values[5],
                quat_y=values[6],
                quat_z=values[7],
                sample_timestamp=values[8],
                sensor_mcu_tick_us=sensor_mcu_tick_us,
                readout_mcu_tick_us=values[10],
                time_status=values[11],
            )

        if len(payload) == IMU_PUSH_EXTENDED_RECORD_SIZE:
            values = struct.unpack(
                "<B" + ("f" * IMU_PUSH_EXTENDED_FLOAT_COUNT),
                payload,
            )
            return IMUPushRecord(
                imu_addr=values[0],
                roll_deg=values[1],
                pitch_deg=values[2],
                yaw_deg=values[3],
                acc_x=values[4],
                acc_y=values[5],
                acc_z=values[6],
                gyro_x=values[7],
                gyro_y=values[8],
                gyro_z=values[9],
                quat_w=values[10],
                quat_x=values[11],
                quat_y=values[12],
                quat_z=values[13],
            )

        raise ValueError(f"unsupported imu record size: {len(payload)}")

    def _handle_imu_diag_push(self, payload: bytes) -> None:
        if len(payload) != 11:
            print(
                f"[bridge] invalid IMU diag length: {len(payload)}",
                file=sys.stderr,
                flush=True,
            )
            return

        now = time.monotonic()
        if (
            self._imu_print_interval_s > 0.0
            and (now - self._last_diag_print_s) < self._imu_print_interval_s
        ):
            return
        self._last_diag_print_s = now
        self._end_imu_single_line()

        seq, online_mask, valid_mask, online_count, valid_count, capacity = struct.unpack(
            "<IHHBBB", payload
        )
        print(
            "imu_diag,"
            f"seq,{seq},"
            f"online_mask,0x{online_mask:04X},"
            f"valid_mask,0x{valid_mask:04X},"
            f"online_count,{online_count},"
            f"valid_count,{valid_count},"
            f"capacity,{capacity}",
        )

    def _parse_sync_event_push(
        self, payload: bytes
    ) -> Optional[list[SyncEventRecord]]:
        if len(payload) < 1:
            return None
        count = payload[0]
        expected_len = 1 + count * SYNC_EVENT_RECORD_SIZE
        if len(payload) != expected_len:
            return None

        events: list[SyncEventRecord] = []
        offset = 1
        for _ in range(count):
            source, flags, _reserved, sequence, mcu_tick_us, nominal_period_us, dropped_count = struct.unpack_from(
                "<BBHIQII",
                payload,
                offset,
            )
            events.append(
                SyncEventRecord(
                    source=source,
                    flags=flags,
                    sequence=sequence,
                    mcu_tick_us=mcu_tick_us,
                    nominal_period_us=nominal_period_us,
                    dropped_count=dropped_count,
                )
            )
            offset += SYNC_EVENT_RECORD_SIZE
        return events

    def _handle_sync_event_push(self, payload: bytes, rx_meta: RXMetadata) -> None:
        events = self._parse_sync_event_push(payload)
        if events is None:
            print(
                f"[bridge] invalid sync event length: {len(payload)}",
                file=sys.stderr,
                flush=True,
            )
            return

        callback = self._sync_event_callback
        if callback is not None and events:
            ts_unix_ms = rx_meta.rx_wall_us // 1000
            try:
                callback(ts_unix_ms, events, rx_meta)
            except Exception as exc:
                print(f"[bridge] sync event callback error: {exc}", file=sys.stderr, flush=True)

        if self._print_imu and events:
            self._end_imu_single_line()
            for event in events:
                source_name = SYNC_EVENT_SOURCE_NAMES.get(event.source, f"source_{event.source}")
                print(
                    "sync_event,"
                    f"source,{source_name},"
                    f"seq,{event.sequence},"
                    f"mcu_tick_us,{event.mcu_tick_us},"
                    f"period_us,{event.nominal_period_us},"
                    f"dropped,{event.dropped_count},"
                    f"flags,0x{event.flags:02X}",
                    flush=True,
                )


def send_ping(client: BridgeClient) -> None:
    payload = client.request(CMD_PING, b"")
    if len(payload) != 1 or payload[0] != STATUS_OK:
        raise RuntimeError(f"ping failed, payload={payload.hex(' ')}")


def send_light_control(
    client: BridgeClient,
    target: int,
    red: int,
    green: int,
    blue: int,
    *,
    blink_enable: bool = False,
    interval_ms: int = 0,
) -> int:
    payload = build_light_control_payload(
        target,
        red,
        green,
        blue,
        blink_enable=blink_enable,
        interval_ms=interval_ms,
    )
    try:
        resp = client.request(CMD_WS2812_CONTROL, payload, timeout=1.5)
    except TimeoutError:
        # Under heavy IMU push traffic, retry once to tolerate transient response delay.
        time.sleep(0.02)
        resp = client.request(CMD_WS2812_CONTROL, payload, timeout=1.5)
    if len(resp) != 1:
        raise RuntimeError(
            f"unexpected light-control response length: {len(resp)}, "
            f"payload={resp.hex(' ')}"
        )
    return resp[0]


def send_light_control_checked(
    client: BridgeClient,
    target: int,
    red: int,
    green: int,
    blue: int,
    *,
    blink_enable: bool = False,
    interval_ms: int = 0,
) -> None:
    status = send_light_control(
        client,
        target,
        red,
        green,
        blue,
        blink_enable=blink_enable,
        interval_ms=interval_ms,
    )
    if status != STATUS_OK:
        raise RuntimeError(
            f"light-control returned error status={status}, target=0x{target:02X}, "
            f"blink={blink_enable}, interval_ms={interval_ms}"
        )


def cmd_ping(client: BridgeClient, _args: argparse.Namespace) -> int:
    send_ping(client)
    print("PING ok")
    return 0


def cmd_off(client: BridgeClient, args: argparse.Namespace) -> int:
    send_light_control_checked(client, TARGET_ALL_LEDS, 0, 0, 0)
    print(f"OFF ok, led_count={DEFAULT_LED_COUNT}")
    return 0


def cmd_rgb(client: BridgeClient, args: argparse.Namespace) -> int:
    send_light_control_checked(client, TARGET_ALL_LEDS, args.red, args.green, args.blue)
    print(
        f"RGB ok, led_count={DEFAULT_LED_COUNT}, "
        f"rgb=({args.red},{args.green},{args.blue})"
    )
    return 0


def cmd_led(client: BridgeClient, args: argparse.Namespace) -> int:
    send_light_control_checked(
        client,
        args.index,
        args.red,
        args.green,
        args.blue,
    )
    print(
        f"LED ok, index={args.index}, rgb=({args.red},{args.green},{args.blue})"
    )
    return 0


def cmd_blink(client: BridgeClient, args: argparse.Namespace) -> int:
    send_light_control_checked(
        client,
        TARGET_ALL_LEDS,
        args.red,
        args.green,
        args.blue,
        blink_enable=True,
        interval_ms=args.delay_ms,
    )
    print(
        f"BLINK ok, led_count={DEFAULT_LED_COUNT}, "
        f"rgb=({args.red},{args.green},{args.blue}), delay_ms={args.delay_ms}"
    )
    return 0


def cmd_led_blink(client: BridgeClient, args: argparse.Namespace) -> int:
    send_light_control_checked(
        client,
        args.index,
        args.red,
        args.green,
        args.blue,
        blink_enable=True,
        interval_ms=args.delay_ms,
    )
    print(
        f"LED-BLINK ok, index={args.index}, "
        f"rgb=({args.red},{args.green},{args.blue}), delay_ms={args.delay_ms}"
    )
    return 0


def cmd_console(client: BridgeClient, args: argparse.Namespace) -> int:
    print("Console mode started. IMU push will print automatically.")
    print(
        "Commands: ping | off | rgb R G B | led I R G B | "
        "blink R G B [delay_ms|--delay-ms N] | "
        "led-blink I R G B [delay_ms|--delay-ms N] | stop | quit"
    )

    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break

        if not line:
            continue

        tokens = line.split()
        cmd = tokens[0].lower()

        try:
            if cmd in {"quit", "exit"}:
                break
            if cmd == "ping":
                send_ping(client)
                print("PING ok")
            elif cmd == "off":
                send_light_control_checked(client, TARGET_ALL_LEDS, 0, 0, 0)
                print("OFF ok")
            elif cmd == "rgb" and len(tokens) == 4:
                red = parse_byte(tokens[1])
                green = parse_byte(tokens[2])
                blue = parse_byte(tokens[3])
                send_light_control_checked(client, TARGET_ALL_LEDS, red, green, blue)
                print(f"RGB ok: ({red},{green},{blue})")
            elif cmd == "led" and len(tokens) == 5:
                index = parse_led_index(tokens[1])
                red = parse_byte(tokens[2])
                green = parse_byte(tokens[3])
                blue = parse_byte(tokens[4])
                send_light_control_checked(client, index, red, green, blue)
                print(f"LED ok: index={index}, rgb=({red},{green},{blue})")
            elif cmd == "blink":
                red = parse_byte(tokens[1])
                green = parse_byte(tokens[2])
                blue = parse_byte(tokens[3])
                delay_ms = parse_console_delay_ms(
                    tokens,
                    4,
                    300,
                    "blink R G B [delay_ms|--delay-ms N]",
                )
                send_light_control_checked(
                    client,
                    TARGET_ALL_LEDS,
                    red,
                    green,
                    blue,
                    blink_enable=True,
                    interval_ms=delay_ms,
                )
                print(f"BLINK ok: ({red},{green},{blue}), delay_ms={delay_ms}")
            elif cmd == "led-blink":
                index = parse_led_index(tokens[1])
                red = parse_byte(tokens[2])
                green = parse_byte(tokens[3])
                blue = parse_byte(tokens[4])
                delay_ms = parse_console_delay_ms(
                    tokens,
                    5,
                    300,
                    "led-blink I R G B [delay_ms|--delay-ms N]",
                )
                send_light_control_checked(
                    client,
                    index,
                    red,
                    green,
                    blue,
                    blink_enable=True,
                    interval_ms=delay_ms,
                )
                print(
                    f"LED-BLINK ok: index={index}, "
                    f"rgb=({red},{green},{blue}), delay_ms={delay_ms}"
                )
            elif cmd == "stop":
                send_light_control_checked(client, TARGET_ALL_LEDS, 0, 0, 0)
                print("STOP ok")
            else:
                print("Unknown command")
        except Exception as exc:
            print(f"command error: {exc}", file=sys.stderr)

    return 0


def cmd_monitor(client: BridgeClient, _args: argparse.Namespace) -> int:
    print("Monitor mode started. Waiting for IMU push frames, press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        return 0


def cmd_sync_monitor(client: BridgeClient, _args: argparse.Namespace) -> int:
    print("Sync-monitor mode started. Waiting for sample_timestamp resets, press Ctrl+C to stop.")
    sync_states: dict[int, SyncCycleStats] = {}

    def _record(
        ts_unix_ms: int,
        imu_records: list[IMUPushRecord],
        _rx_meta: RXMetadata,
    ) -> None:
        for record in imu_records:
            if record.sample_timestamp is None:
                continue

            state = sync_states.get(record.imu_addr)
            if state is None:
                state = SyncCycleStats(imu_addr=record.imu_addr)
                sync_states[record.imu_addr] = state

            current_sample_ts = record.sample_timestamp
            prev_sample_ts = state.prev_sample_ts
            if (
                state.frame_count > 0
                and prev_sample_ts is not None
                and current_sample_ts < prev_sample_ts
            ):
                print(state.format_summary())
                state.reset_for_next_cycle()

            state.update(ts_unix_ms, current_sample_ts)

    client.set_imu_record_callback(_record)
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        for addr in sorted(sync_states):
            state = sync_states[addr]
            if state.frame_count > 0:
                print(state.format_summary())
        return 0
    finally:
        client.set_imu_record_callback(None)


def cmd_sync(client: BridgeClient, args: argparse.Namespace) -> int:
    samples: list[TimeSyncSample] = []
    error_count = 0
    for seq in range(args.count):
        try:
            sample = send_time_sync(client, seq)
        except TimeoutError:
            error_count += 1
        except (ValueError, RuntimeError) as exc:
            error_count += 1
            print(f"sync error seq={seq}: {exc}", file=sys.stderr, flush=True)
        else:
            samples.append(sample)
            print(
                f"sync seq={sample.seq} "
                f"t2={sample.t2_mcu_tick_us} "
                f"t3={sample.t3_mcu_tick_us} "
                f"rtt_us={sample.rtt_us} "
                f"offset_us={sample.offset_us}"
            )
        if seq + 1 < args.count:
            time.sleep(args.interval_s)

    if samples:
        best = min(samples, key=lambda item: item.rtt_us)
        print(
            f"best seq={best.seq} "
            f"rtt_us={best.rtt_us} "
            f"offset_us={best.offset_us}"
        )
    print(f"sync summary: ok={len(samples)} error={error_count} requested={args.count}")
    return 0


def cmd_capture(client: BridgeClient, args: argparse.Namespace) -> int:
    output_path = os.path.abspath(args.output)
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    event_output_path = os.path.abspath(args.event_output)
    event_output_dir = os.path.dirname(event_output_path)
    if event_output_dir:
        os.makedirs(event_output_dir, exist_ok=True)
    sync_path = os.path.abspath(args.sync_output)
    sync_mapper = TimeSyncMapper()
    sync_session = TimeSyncSession(
        client,
        sync_mapper,
        sync_path,
        burst_count=args.sync_burst_count,
        interval_s=args.sync_interval_s,
    )

    stop_event = threading.Event()

    def _handle_signal(_signum, _frame):
        stop_event.set()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        initial_mapping = sync_session.run_burst()
        if initial_mapping is not None:
            print(
                f"time-sync ready: seq={initial_mapping.sample.seq}, "
                f"rtt_us={initial_mapping.sample.rtt_us}, "
                f"offset_us={initial_mapping.sample.offset_us}"
            )
        else:
            print(
                "time-sync warning: initial burst produced no valid mapping; "
                "timestamp_us will remain null until sync succeeds",
                file=sys.stderr,
                flush=True,
            )
        if args.sync_interval_s > 0.0:
            sync_session.start()
            print(
                f"time-sync periodic enabled: interval_s={args.sync_interval_s}, "
                f"burst_count={args.sync_burst_count}"
            )
        else:
            print("time-sync periodic disabled: using initial mapping only")
        with open(output_path, "w", encoding="utf-8", buffering=1) as f, open(
            event_output_path, "w", encoding="utf-8", newline="", buffering=1
        ) as event_file:
            event_writer = csv.writer(event_file)
            event_writer.writerow(
                [
                    "source",
                    "sequence",
                    "mcu_tick_us",
                    "timestamp_mono_us",
                    "timestamp_us",
                    "rx_timestamp_us",
                    "nominal_period_us",
                    "dropped_count",
                    "flags",
                    "sync_version",
                ]
            )
            f.write("[\n")
            first_row = True

            def _record(
                ts_unix_ms: int,
                imu_records: list[IMUPushRecord],
                rx_meta: RXMetadata,
            ) -> None:
                nonlocal first_row
                row = format_imu_json_row(
                    ts_unix_ms,
                    imu_records,
                    rx_meta,
                    sync_mapper=sync_mapper,
                )

                if not first_row:
                    f.write(",\n")
                f.write(json.dumps(row, ensure_ascii=False))
                f.flush()
                first_row = False

            def _sync_event(
                _ts_unix_ms: int,
                events: list[SyncEventRecord],
                rx_meta: RXMetadata,
            ) -> None:
                for event in events:
                    timestamp_mono_us, timestamp_us, sync_quality = sync_mapper.map_to_times(
                        event.mcu_tick_us,
                        rx_meta,
                    )
                    source_name = SYNC_EVENT_SOURCE_NAMES.get(
                        event.source, f"source_{event.source}"
                    )
                    sync_version = (
                        sync_quality.get("mapping_version")
                        if sync_quality is not None
                        else None
                    )
                    event_writer.writerow(
                        [
                            source_name,
                            event.sequence,
                            event.mcu_tick_us,
                            timestamp_mono_us,
                            timestamp_us,
                            rx_meta.rx_wall_us,
                            event.nominal_period_us,
                            event.dropped_count,
                            f"0x{event.flags:02X}",
                            sync_version,
                        ]
                    )
                event_file.flush()

            client.set_imu_record_callback(_record)
            client.set_sync_event_callback(_sync_event)
            while not stop_event.is_set():
                time.sleep(0.2)
            client.set_imu_record_callback(None)
            client.set_sync_event_callback(None)
            if not first_row:
                f.write("\n")
            f.write("]\n")
            f.flush()
    finally:
        client.set_imu_record_callback(None)
        client.set_sync_event_callback(None)
        sync_session.stop()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UART bridge tool for IMU push and WS2812 control")
    parser.add_argument("--port", required=True, help="serial port, e.g. COM12 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800, help="serial baudrate")
    parser.add_argument("--timeout", type=float, default=0.1, help="serial timeout in seconds")
    parser.add_argument(
        "--imu-print-hz",
        type=float,
        default=10.0,
        help="max print rate for imu_bundle/imu_diag (0 means print every frame)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    ping_parser = subparsers.add_parser("ping", help="send bridge ping")
    ping_parser.set_defaults(func=cmd_ping)

    off_parser = subparsers.add_parser("off", help="turn off all WS2812 LEDs")
    off_parser.set_defaults(func=cmd_off)

    rgb_parser = subparsers.add_parser("rgb", help="set all LEDs to one RGB color")
    rgb_parser.add_argument("red", type=parse_byte, help="red 0-255")
    rgb_parser.add_argument("green", type=parse_byte, help="green 0-255")
    rgb_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    rgb_parser.set_defaults(func=cmd_rgb)

    led_parser = subparsers.add_parser(
        "led", help="set one LED by index, keeping other LEDs unchanged"
    )
    led_parser.add_argument("index", type=parse_led_index, help="LED index, 0-based")
    led_parser.add_argument("red", type=parse_byte, help="red 0-255")
    led_parser.add_argument("green", type=parse_byte, help="green 0-255")
    led_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    led_parser.set_defaults(func=cmd_led)

    blink_parser = subparsers.add_parser("blink", help="blink all LEDs with one RGB color")
    blink_parser.add_argument("red", type=parse_byte, help="red 0-255")
    blink_parser.add_argument("green", type=parse_byte, help="green 0-255")
    blink_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    blink_parser.add_argument(
        "--delay-ms",
        type=parse_interval_ms,
        default=300,
        help="toggle interval in milliseconds",
    )
    blink_parser.set_defaults(func=cmd_blink)

    led_blink_parser = subparsers.add_parser(
        "led-blink", help="blink one LED by index, keeping other LEDs unchanged"
    )
    led_blink_parser.add_argument("index", type=parse_led_index, help="LED index, 0-based")
    led_blink_parser.add_argument("red", type=parse_byte, help="red 0-255")
    led_blink_parser.add_argument("green", type=parse_byte, help="green 0-255")
    led_blink_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    led_blink_parser.add_argument(
        "--delay-ms",
        type=parse_interval_ms,
        default=300,
        help="toggle interval in milliseconds",
    )
    led_blink_parser.set_defaults(func=cmd_led_blink)

    console_parser = subparsers.add_parser(
        "console",
        help="keep receiving IMU push while typing ping/off/rgb/led/blink commands",
    )
    console_parser.set_defaults(func=cmd_console)

    monitor_parser = subparsers.add_parser(
        "monitor",
        help="receive and print IMU push frames only",
    )
    monitor_parser.set_defaults(func=cmd_monitor)

    sync_monitor_parser = subparsers.add_parser(
        "sync-monitor",
        help="analyze sample_timestamp reset cycles for externally synchronized YIS data",
    )
    sync_monitor_parser.set_defaults(func=cmd_sync_monitor)

    sync_parser = subparsers.add_parser(
        "sync",
        help="send NV-MCU time-sync requests and print raw four-timestamp results",
    )
    sync_parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="number of time-sync requests to send",
    )
    sync_parser.add_argument(
        "--interval-s",
        type=float,
        default=0.2,
        help="delay between time-sync requests in seconds",
    )
    sync_parser.set_defaults(func=cmd_sync)

    capture_parser = subparsers.add_parser(
        "capture",
        help="background capture mode, write IMU pushes to JSON",
    )
    capture_parser.add_argument(
        "--output",
        required=True,
        help="output JSON path",
    )
    capture_parser.add_argument(
        "--sync-output",
        default="log/imu_time_sync.csv",
        help="output CSV path for raw time-sync samples",
    )
    capture_parser.add_argument(
        "--event-output",
        default="log/sync_events.csv",
        help="output CSV path for TIM2/TIM5 sync event samples",
    )
    capture_parser.add_argument(
        "--sync-interval-s",
        type=float,
        default=0.0,
        help="period between time-sync bursts in seconds; <=0 disables periodic sync",
    )
    capture_parser.add_argument(
        "--sync-burst-count",
        type=int,
        default=10,
        help="number of sync requests per burst, best low-RTT sample is selected",
    )
    capture_parser.set_defaults(func=cmd_capture)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        print_imu = args.command not in {"capture", "sync", "sync-monitor"}
        with BridgeClient(
            args.port,
            args.baud,
            args.timeout,
            print_imu=print_imu,
            imu_print_hz=args.imu_print_hz,
        ) as client:
            return args.func(client, args)
    except serial.SerialException as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 2
    except TimeoutError as exc:
        print(f"timeout: {exc}", file=sys.stderr)
        return 3
    except ValueError as exc:
        print(f"protocol error: {exc}", file=sys.stderr)
        return 4
    except RuntimeError as exc:
        print(f"device error: {exc}", file=sys.stderr)
        return 5
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
