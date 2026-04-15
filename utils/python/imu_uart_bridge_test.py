#!/usr/bin/env python3
"""
Host-side tool for the STM32 UART bridge.

docker exec -it my_realtime_container bash

Raw serial monitor:
  python3 -m serial.tools.miniterm /dev/ttyTCU0 115200

Continuous IMU monitor:
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 console
  python utils/python/imu_uart_bridge_test.py --port COM4 console 
Examples:
  python3 scripts/imu_uart_bridge_test.py --port /dev/ttyTCU0 ping
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 rgb 135 206 250
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 led 3 255 0 0
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 off
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 blink 135 206 250
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 console
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
CMD_WS2812_FRAME = 0x21
CMD_IMU_EULER_PUSH = 0x30
CMD_IMU_DIAG_PUSH = 0x31

STATUS_OK = 0
DEFAULT_SPI_BUS = 0
DEFAULT_LED_COUNT = 16
MAX_FRAME_PAYLOAD = 1024
IMU_PUSH_LEGACY_RECORD_SIZE = struct.calcsize("<Bfff")
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

    def has_extended_data(self) -> bool:
        return not math.isnan(self.acc_x)


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


def make_solid_frame(led_count: int, red: int, green: int, blue: int) -> bytes:
    return bytes([red, green, blue]) * led_count


def make_single_led_frame(
    led_count: int, led_index: int, red: int, green: int, blue: int
) -> bytes:
    if led_index < 0 or led_index >= led_count:
        raise ValueError(
            f"led index out of range: {led_index}, valid range is 0..{led_count - 1}"
        )

    frame = bytearray(led_count * 3)
    base = led_index * 3
    frame[base] = red
    frame[base + 1] = green
    frame[base + 2] = blue
    return bytes(frame)


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
            CMD_WS2812_FRAME: queue.Queue(),
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
                if record.has_extended_data():
                    parts.extend(
                        [
                            f"acc=({record.acc_x:.3f},{record.acc_y:.3f},{record.acc_z:.3f})",
                            f"gyro=({record.gyro_x:.3f},{record.gyro_y:.3f},{record.gyro_z:.3f})",
                            f"quat=({record.quat_w:.4f},{record.quat_x:.4f},{record.quat_y:.4f},{record.quat_z:.4f})",
                        ]
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


class BlinkWorker:
    def __init__(self, client: BridgeClient, spi_bus: int, led_count: int) -> None:
        self._client = client
        self._spi_bus = spi_bus
        self._led_count = led_count
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def start(self, red: int, green: int, blue: int, delay_ms: float) -> None:
        self.stop(turn_off=False)
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, args=(red, green, blue, delay_ms), daemon=True
        )
        self._thread.start()

    def stop(self, *, turn_off: bool = True) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if turn_off:
            send_ws2812_checked(
                self._client, self._spi_bus, make_solid_frame(self._led_count, 0, 0, 0)
            )

    def _run(self, red: int, green: int, blue: int, delay_ms: float) -> None:
        on = True
        while not self._stop_event.is_set():
            frame = (
                make_solid_frame(self._led_count, red, green, blue)
                if on
                else make_solid_frame(self._led_count, 0, 0, 0)
            )
            try:
                send_ws2812_checked(self._client, self._spi_bus, frame)
            except TimeoutError as exc:
                print(f"[bridge] blink timeout: {exc}", file=sys.stderr, flush=True)
            except Exception as exc:
                print(f"[bridge] blink error: {exc}", file=sys.stderr, flush=True)
            on = not on
            self._stop_event.wait(delay_ms / 1000.0)


def send_ping(client: BridgeClient) -> None:
    payload = client.request(CMD_PING, b"")
    if len(payload) != 1 or payload[0] != STATUS_OK:
        raise RuntimeError(f"ping failed, payload={payload.hex(' ')}")


def send_ws2812_frame(client: BridgeClient, spi_bus: int, rgb: bytes) -> int:
    if len(rgb) % 3 != 0:
        raise ValueError(f"RGB payload length must be a multiple of 3, got {len(rgb)}")

    led_count = len(rgb) // 3
    payload = bytes(
        [
            spi_bus & 0xFF,
            led_count & 0xFF,
            (led_count >> 8) & 0xFF,
        ]
    ) + rgb

    try:
        resp = client.request(CMD_WS2812_FRAME, payload, timeout=1.5)
    except TimeoutError:
        # Under heavy IMU push traffic, retry once to tolerate transient response delay.
        time.sleep(0.02)
        resp = client.request(CMD_WS2812_FRAME, payload, timeout=1.5)
    if len(resp) != 1:
        raise RuntimeError(f"unexpected WS2812 response length: {len(resp)}")
    return resp[0]


def send_ws2812_checked(client: BridgeClient, spi_bus: int, rgb: bytes) -> None:
    status = send_ws2812_frame(client, spi_bus, rgb)
    if status != STATUS_OK:
        raise RuntimeError(
            f"WS2812 bridge returned error status={status}, spi_bus={spi_bus}, led_count={len(rgb) // 3}"
        )


def cmd_ping(client: BridgeClient, _args: argparse.Namespace) -> int:
    send_ping(client)
    print("PING ok")
    return 0


def cmd_off(client: BridgeClient, args: argparse.Namespace) -> int:
    send_ws2812_checked(client, args.spi_bus, make_solid_frame(args.led_count, 0, 0, 0))
    print(f"OFF ok, spi_bus={args.spi_bus}, led_count={args.led_count}")
    return 0


def cmd_rgb(client: BridgeClient, args: argparse.Namespace) -> int:
    send_ws2812_checked(
        client,
        args.spi_bus,
        make_solid_frame(args.led_count, args.red, args.green, args.blue),
    )
    print(
        f"RGB ok, spi_bus={args.spi_bus}, led_count={args.led_count}, "
        f"rgb=({args.red},{args.green},{args.blue})"
    )
    return 0


def cmd_led(client: BridgeClient, args: argparse.Namespace) -> int:
    send_ws2812_checked(
        client,
        args.spi_bus,
        make_single_led_frame(
            args.led_count, args.index, args.red, args.green, args.blue
        ),
    )
    print(
        f"LED ok, spi_bus={args.spi_bus}, led_count={args.led_count}, "
        f"index={args.index}, rgb=({args.red},{args.green},{args.blue})"
    )
    return 0


def cmd_blink(client: BridgeClient, args: argparse.Namespace) -> int:
    print("Running blink, IMU push will continue printing, press Ctrl+C to stop...")
    on = True
    try:
        while True:
            frame = (
                make_solid_frame(args.led_count, args.red, args.green, args.blue)
                if on
                else make_solid_frame(args.led_count, 0, 0, 0)
            )
            send_ws2812_checked(client, args.spi_bus, frame)
            on = not on
            time.sleep(args.delay_ms / 1000.0)
    finally:
        send_ws2812_checked(client, args.spi_bus, make_solid_frame(args.led_count, 0, 0, 0))


def cmd_console(client: BridgeClient, args: argparse.Namespace) -> int:
    blink_worker = BlinkWorker(client, args.spi_bus, args.led_count)
    print("Console mode started. IMU push will print automatically.")
    print(
        "Commands: ping | off | rgb R G B | led I R G B | "
        "blink R G B [delay_ms] | stop | quit"
    )

    try:
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
                    blink_worker.stop(turn_off=False)
                    send_ws2812_checked(
                        client, args.spi_bus, make_solid_frame(args.led_count, 0, 0, 0)
                    )
                    print("OFF ok")
                elif cmd == "rgb" and len(tokens) == 4:
                    blink_worker.stop(turn_off=False)
                    red = parse_byte(tokens[1])
                    green = parse_byte(tokens[2])
                    blue = parse_byte(tokens[3])
                    send_ws2812_checked(
                        client,
                        args.spi_bus,
                        make_solid_frame(args.led_count, red, green, blue),
                    )
                    print(f"RGB ok: ({red},{green},{blue})")
                elif cmd == "led" and len(tokens) == 5:
                    blink_worker.stop(turn_off=False)
                    index = int(tokens[1], 0)
                    red = parse_byte(tokens[2])
                    green = parse_byte(tokens[3])
                    blue = parse_byte(tokens[4])
                    send_ws2812_checked(
                        client,
                        args.spi_bus,
                        make_single_led_frame(
                            args.led_count, index, red, green, blue
                        ),
                    )
                    print(f"LED ok: index={index}, rgb=({red},{green},{blue})")
                elif cmd == "blink" and len(tokens) in {4, 5}:
                    red = parse_byte(tokens[1])
                    green = parse_byte(tokens[2])
                    blue = parse_byte(tokens[3])
                    delay_ms = float(tokens[4]) if len(tokens) == 5 else 300.0
                    blink_worker.start(red, green, blue, delay_ms)
                    print(f"BLINK started: ({red},{green},{blue}), delay_ms={delay_ms}")
                elif cmd == "stop":
                    blink_worker.stop()
                    print("BLINK stopped")
                else:
                    print("Unknown command")
            except Exception as exc:
                print(f"command error: {exc}", file=sys.stderr)
    finally:
        blink_worker.stop()

    return 0


def cmd_monitor(client: BridgeClient, _args: argparse.Namespace) -> int:
    print("Monitor mode started. Waiting for IMU push frames, press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        return 0


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

                row = {
                    "ts_iso": datetime.fromtimestamp(ts_unix_ms / 1000.0).isoformat(timespec="milliseconds"),
                    "poses_by_addr": {},
                    "acc_by_addr": {},
                    "gyro_by_addr": {},
                    "quaternion_by_addr": {},
                }

                # 固定列顺序，便于逐行对比同一地址的数据变化。
                for addr in DEFAULT_IMU_ADDR_COLUMNS:
                    key = f"0x{addr:02X}"
                    row["poses_by_addr"][key] = poses_by_addr.get(key)
                    row["acc_by_addr"][key] = acc_by_addr.get(key)
                    row["gyro_by_addr"][key] = gyro_by_addr.get(key)
                    row["quaternion_by_addr"][key] = quaternion_by_addr.get(key)

                # 追加非默认地址，避免丢信息。
                for key in sorted(poses_by_addr.keys()):
                    if key not in row["poses_by_addr"]:
                        row["poses_by_addr"][key] = poses_by_addr[key]
                        row["acc_by_addr"][key] = acc_by_addr.get(key)
                        row["gyro_by_addr"][key] = gyro_by_addr.get(key)
                        row["quaternion_by_addr"][key] = quaternion_by_addr.get(key)

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


def add_ws_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--spi-bus", type=lambda x: int(x, 0), default=DEFAULT_SPI_BUS, help="SPI bus id"
    )
    parser.add_argument(
        "--led-count",
        type=lambda x: int(x, 0),
        default=DEFAULT_LED_COUNT,
        help=f"number of LEDs (default: {DEFAULT_LED_COUNT})",
    )


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
    add_ws_common_args(off_parser)
    off_parser.set_defaults(func=cmd_off)

    rgb_parser = subparsers.add_parser("rgb", help="set all LEDs to one RGB color")
    add_ws_common_args(rgb_parser)
    rgb_parser.add_argument("red", type=parse_byte, help="red 0-255")
    rgb_parser.add_argument("green", type=parse_byte, help="green 0-255")
    rgb_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    rgb_parser.set_defaults(func=cmd_rgb)

    led_parser = subparsers.add_parser(
        "led", help="set one LED by index, turn all others off"
    )
    add_ws_common_args(led_parser)
    led_parser.add_argument("index", type=int, help="LED index, 0-based")
    led_parser.add_argument("red", type=parse_byte, help="red 0-255")
    led_parser.add_argument("green", type=parse_byte, help="green 0-255")
    led_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    led_parser.set_defaults(func=cmd_led)

    blink_parser = subparsers.add_parser("blink", help="blink one RGB color")
    add_ws_common_args(blink_parser)
    blink_parser.add_argument("red", type=parse_byte, help="red 0-255")
    blink_parser.add_argument("green", type=parse_byte, help="green 0-255")
    blink_parser.add_argument("blue", type=parse_byte, help="blue 0-255")
    blink_parser.add_argument(
        "--delay-ms", type=float, default=300.0, help="toggle interval in milliseconds"
    )
    blink_parser.set_defaults(func=cmd_blink)

    console_parser = subparsers.add_parser(
        "console",
        help="keep receiving IMU push while typing ping/off/rgb/blink commands",
    )
    add_ws_common_args(console_parser)
    console_parser.set_defaults(func=cmd_console)

    monitor_parser = subparsers.add_parser(
        "monitor",
        help="receive and print IMU push frames only",
    )
    monitor_parser.set_defaults(func=cmd_monitor)

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
        print_imu = args.command != "capture"
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
