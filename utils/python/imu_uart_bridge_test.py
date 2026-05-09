#!/usr/bin/env python3
"""
Host-side tool for the STM32 UART bridge.

Run from repo root with utils/python path:
  python utils/python/imu_uart_bridge_test.py --port COM4 ping
  python utils/python/imu_uart_bridge_test.py --port COM4 rgb 135 206 250
  python utils/python/imu_uart_bridge_test.py --port COM4 led 3 255 0 0
  python utils/python/imu_uart_bridge_test.py --port COM4 off
  python utils/python/imu_uart_bridge_test.py --port COM4 blink 135 206 250 --delay-ms 300
  python utils/python/imu_uart_bridge_test.py --port COM4 led-blink 3 255 0 0 --delay-ms 300
  python utils/python/imu_uart_bridge_test.py --port COM4 console

Run from repo root with scripts path:
  python scripts/imu_uart_bridge_test.py --port COM4 ping
  python scripts/imu_uart_bridge_test.py --port COM4 rgb 135 206 250
  python scripts/imu_uart_bridge_test.py --port COM4 led 3 255 0 0
  python scripts/imu_uart_bridge_test.py --port COM4 off
  python scripts/imu_uart_bridge_test.py --port COM4 blink 135 206 250 --delay-ms 300
  python scripts/imu_uart_bridge_test.py --port COM4 led-blink 3 255 0 0 --delay-ms 300
  python scripts/imu_uart_bridge_test.py --port COM4 console

""" 

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
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
CMD_WS2812_CONTROL = 0x21
CMD_IMU_EULER_PUSH = 0x30
CMD_IMU_DIAG_PUSH = 0x31

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
IMU_PUSH_YIS_POSE_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_POSE_FLOAT_COUNT) + "I"
)
IMU_PUSH_EXTENDED_FLOAT_COUNT = 13
IMU_PUSH_EXTENDED_RECORD_SIZE = struct.calcsize(
    "<B" + ("f" * IMU_PUSH_EXTENDED_FLOAT_COUNT)
)
# JSON 固定列顺序（按用户要求：0x50/0x51/0x52/0x53）
DEFAULT_IMU_ADDR_COLUMNS = [0x50, 0x51, 0x52, 0x53]
ANSI_RESET = "\033[0m"
ANSI_ADDR_COLOR = {
    0x50: "\033[91m",  # red
    0x51: "\033[94m",  # blue
    0x52: "\033[92m",  # green
    0x53: "\033[93m",  # yellow
}


@dataclass(frozen=True)
class IMUPushRecord:
    imu_addr: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    sample_timestamp: Optional[int] = None
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
            Callable[[int, list[IMUPushRecord]], None]
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
            CMD_WS2812_CONTROL: queue.Queue(),
        }

    def set_imu_record_callback(
        self,
        callback: Optional[
            Callable[[int, list[IMUPushRecord]], None]
        ],
    ) -> None:
        self._imu_record_callback = callback

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
                return resp_queue.get(timeout=timeout)
            except queue.Empty as exc:
                raise TimeoutError(f"timeout waiting for response to cmd=0x{cmd:02X}") from exc

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

            del rx_buf[:total_len]
            self._dispatch_frame(cmd, payload)

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

    def _dispatch_frame(self, cmd: int, payload: bytes) -> None:
        if cmd == CMD_IMU_EULER_PUSH:
            self._handle_imu_push(payload)
            return
        if cmd == CMD_IMU_DIAG_PUSH:
            self._handle_imu_diag_push(payload)
            return

        resp_queue = self._response_queues.setdefault(cmd, queue.Queue())
        resp_queue.put(payload)

    def _handle_imu_push(self, payload: bytes) -> None:
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
            ts_unix_ms = int(time.time() * 1000)
            try:
                callback(ts_unix_ms, imu_records)
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
                parts.extend(
                    [
                        format_addr_with_color(record.imu_addr),
                        f"rpy=({record.roll_deg:.3f},{record.pitch_deg:.3f},{record.yaw_deg:.3f})",
                    ]
                )
                if record.sample_timestamp is not None:
                    parts.append(f"sample_ts={record.sample_timestamp}")
                if record.has_motion_data():
                    parts.extend(
                        [
                            f"acc=({record.acc_x:.3f},{record.acc_y:.3f},{record.acc_z:.3f})",
                            f"gyro=({record.gyro_x:.3f},{record.gyro_y:.3f},{record.gyro_z:.3f})",
                        ]
                    )
                if record.has_quaternion():
                    parts.append(
                        f"quat=({record.quat_w:.4f},{record.quat_x:.4f},{record.quat_y:.4f},{record.quat_z:.4f})"
                    )
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
        records_raw = payload[1:]
        if imu_count == 0:
            return []
        if len(records_raw) % imu_count != 0:
            return None

        record_size = len(records_raw) // imu_count
        if record_size not in (
            IMU_PUSH_LEGACY_RECORD_SIZE,
            IMU_PUSH_POSE_RECORD_SIZE,
            IMU_PUSH_YIS_POSE_RECORD_SIZE,
            IMU_PUSH_EXTENDED_RECORD_SIZE,
        ):
            return None

        imu_records: list[IMUPushRecord] = []
        for offset in range(0, len(records_raw), record_size):
            imu_records.append(
                self._unpack_imu_record(records_raw[offset : offset + record_size])
            )
        return imu_records

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
        raise RuntimeError(f"unexpected light-control response length: {len(resp)}")
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

    def _record(ts_unix_ms: int, imu_records: list[IMUPushRecord]) -> None:
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


def cmd_capture(client: BridgeClient, args: argparse.Namespace) -> int:
    output_path = os.path.abspath(args.output)
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    stop_event = threading.Event()

    def _handle_signal(_signum, _frame):
        stop_event.set()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        with open(output_path, "w", encoding="utf-8", buffering=1) as f:
            f.write("[\n")
            first_row = True

            def _round_or_none(value: float, digits: int = 2) -> Optional[float]:
                if math.isnan(value):
                    return None
                return round(value, digits)

            def _record(ts_unix_ms: int, imu_records: list[IMUPushRecord]) -> None:
                nonlocal first_row
                poses_by_addr = {}
                acc_by_addr = {}
                gyro_by_addr = {}
                quaternion_by_addr = {}
                sample_timestamp_by_addr = {}
                for record in imu_records:
                    key = f"0x{record.imu_addr:02X}"
                    poses_by_addr[key] = {
                        "roll_X": round(record.roll_deg, 2),
                        "pitch_Y": round(record.pitch_deg, 2),
                        "yaw_Z": round(record.yaw_deg, 2),
                    }
                    acc_by_addr[key] = {
                        "x": _round_or_none(record.acc_x),
                        "y": _round_or_none(record.acc_y),
                        "z": _round_or_none(record.acc_z),
                    }
                    gyro_by_addr[key] = {
                        "x": _round_or_none(record.gyro_x),
                        "y": _round_or_none(record.gyro_y),
                        "z": _round_or_none(record.gyro_z),
                    }
                    quaternion_by_addr[key] = {
                        "w": _round_or_none(record.quat_w, 4),
                        "x": _round_or_none(record.quat_x, 4),
                        "y": _round_or_none(record.quat_y, 4),
                        "z": _round_or_none(record.quat_z, 4),
                    }
                    sample_timestamp_by_addr[key] = record.sample_timestamp

                row = {
                    "ts_iso": datetime.fromtimestamp(ts_unix_ms / 1000.0).isoformat(timespec="milliseconds"),
                    "poses_by_addr": {},
                    "acc_by_addr": {},
                    "gyro_by_addr": {},
                    "quaternion_by_addr": {},
                    "sample_timestamp_by_addr": {},
                }

                # 固定列顺序，便于逐行对比同一地址的数据变化。
                for addr in DEFAULT_IMU_ADDR_COLUMNS:
                    key = f"0x{addr:02X}"
                    row["poses_by_addr"][key] = poses_by_addr.get(key)
                    row["acc_by_addr"][key] = acc_by_addr.get(key)
                    row["gyro_by_addr"][key] = gyro_by_addr.get(key)
                    row["quaternion_by_addr"][key] = quaternion_by_addr.get(key)
                    row["sample_timestamp_by_addr"][key] = sample_timestamp_by_addr.get(key)

                # 追加非默认地址，避免丢信息。
                for key in sorted(poses_by_addr.keys()):
                    if key not in row["poses_by_addr"]:
                        row["poses_by_addr"][key] = poses_by_addr[key]
                        row["acc_by_addr"][key] = acc_by_addr.get(key)
                        row["gyro_by_addr"][key] = gyro_by_addr.get(key)
                        row["quaternion_by_addr"][key] = quaternion_by_addr.get(key)
                        row["sample_timestamp_by_addr"][key] = sample_timestamp_by_addr.get(key)

                if not first_row:
                    f.write(",\n")
                f.write(json.dumps(row, ensure_ascii=False))
                f.flush()
                first_row = False

            client.set_imu_record_callback(_record)
            while not stop_event.is_set():
                time.sleep(0.2)
            client.set_imu_record_callback(None)
            if not first_row:
                f.write("\n")
            f.write("]\n")
            f.flush()
    finally:
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="UART bridge tool for IMU push and WS2812 control")
    parser.add_argument("--port", required=True, help="serial port, e.g. COM12 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="serial baudrate")
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

    capture_parser = subparsers.add_parser(
        "capture",
        help="background capture mode, write IMU pushes to JSON",
    )
    capture_parser.add_argument(
        "--output",
        required=True,
        help="output JSON path",
    )
    capture_parser.set_defaults(func=cmd_capture)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        print_imu = args.command not in {"capture", "sync-monitor"}
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
