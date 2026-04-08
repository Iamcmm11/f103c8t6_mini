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
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 ping
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 rgb 135 206 250
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 led 3 255 0 0
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 off
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 blink 135 206 250
  python3 /tmp/imu_uart_bridge_test.py --port /dev/ttyTCU0 console
"""

from __future__ import annotations

import argparse
import queue
import struct
import sys
import threading
import time
from typing import Optional

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
IMU_PUSH_RECORD_SIZE = 13


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


class BridgeClient:
    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float,
        *,
        print_imu: bool = True,
        imu_print_hz: float = 10.0,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._print_imu = print_imu
        self._imu_print_interval_s = (
            0.0 if imu_print_hz <= 0.0 else 1.0 / imu_print_hz
        )
        self._serial: Optional[Serial] = None
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._last_checksum_log_s = 0.0
        self._checksum_drop_count = 0
        self._last_imu_print_s = 0.0
        self._last_diag_print_s = 0.0
        self._last_imu_line_len = 0
        self._frame_ok_count = 0
        self._frame_bad_count = 0
        self._last_stats_print_s = 0.0
        self._response_queues = {
            CMD_PING: queue.Queue(),
            CMD_WS2812_FRAME: queue.Queue(),
        }

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
                    self._print_rx_stats()
                    continue
                chunk = self._serial.read(want)
                if not chunk:
                    self._print_rx_stats()
                    continue
                rx_buf.extend(chunk)
                self._process_rx_buffer(rx_buf)
                self._print_rx_stats()
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
                self._frame_bad_count += 1
                self._checksum_drop_count += 1
                now = time.monotonic()
                if now - self._last_checksum_log_s > 1.0:
                    print(
                        f"[bridge] checksum mismatch: rx=0x{checksum:02X}, expect=0x{expect:02X}, drops={self._checksum_drop_count}",
                        file=sys.stderr,
                        flush=True,
                    )
                    self._last_checksum_log_s = now
                    self._checksum_drop_count = 0
                # Slide one byte instead of dropping the whole candidate frame.
                del rx_buf[0]
                continue

            del rx_buf[:total_len]
            self._frame_ok_count += 1
            self._dispatch_frame(cmd, payload)

    def _print_rx_stats(self) -> None:
        now = time.monotonic()
        if now - self._last_stats_print_s < 1.0:
            return
        total = self._frame_ok_count + self._frame_bad_count
        if total > 0:
            bad_pct = (self._frame_bad_count * 100.0) / total
            print(
                f"[bridge] rx stats: total={total}, ok={self._frame_ok_count}, bad={self._frame_bad_count}, bad_pct={bad_pct:.2f}%",
                file=sys.stderr,
            )
        self._frame_ok_count = 0
        self._frame_bad_count = 0
        self._last_stats_print_s = now

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
                self._checksum_drop_count += 1
                now = time.monotonic()
                if now - self._last_checksum_log_s > 1.0:
                    print(
                        f"[bridge] checksum mismatch: rx=0x{checksum:02X}, expect=0x{expect:02X}, drops={self._checksum_drop_count}",
                        file=sys.stderr,
                        flush=True,
                    )
                    self._last_checksum_log_s = now
                    self._checksum_drop_count = 0
                continue

            return header[2], payload

        return None

    def _dispatch_frame(self, cmd: int, payload: bytes) -> None:
        if cmd == CMD_IMU_EULER_PUSH:
            self._handle_imu_push(payload)
            return
        if cmd == CMD_IMU_DIAG_PUSH:
            # Diag frames are intentionally ignored in monitor output.
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

        if self._print_imu and imu_records:
            now = time.monotonic()
            if (
                self._imu_print_interval_s > 0.0
                and (now - self._last_imu_print_s) < self._imu_print_interval_s
            ):
                return
            self._last_imu_print_s = now
            line = ",".join(
                ["imu_bundle", str(len(imu_records))]
                + [
                    item
                    for imu_addr, roll_deg, pitch_deg, yaw_deg in imu_records
                    for item in (
                        f"0x{imu_addr:02X}",
                        f"{roll_deg:.3f}",
                        f"{pitch_deg:.3f}",
                        f"{yaw_deg:.3f}",
                    )
                ]
            )
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
    ) -> Optional[list[tuple[int, float, float, float]]]:
        if len(payload) == IMU_PUSH_RECORD_SIZE:
            return [struct.unpack("<Bfff", payload)]

        if len(payload) < 1:
            return None

        imu_count = payload[0]
        records_raw = payload[1:]
        if len(records_raw) != imu_count * IMU_PUSH_RECORD_SIZE:
            return None

        imu_records: list[tuple[int, float, float, float]] = []
        for offset in range(0, len(records_raw), IMU_PUSH_RECORD_SIZE):
            imu_records.append(
                struct.unpack(
                    "<Bfff",
                    records_raw[offset : offset + IMU_PUSH_RECORD_SIZE],
                )
            )
        return imu_records

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

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        with BridgeClient(
            args.port,
            args.baud,
            args.timeout,
            print_imu=True,
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
