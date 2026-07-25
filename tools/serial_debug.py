#!/usr/bin/env python3
"""Serial console for the STM32F103 USART1 debug protocol."""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    sys.exit(
        "pyserial is required; run: "
        "python -m pip install -r tools/requirements.txt"
    )


def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class SerialOutput:
    def __init__(self, show_hex: bool) -> None:
        self.show_hex = show_hex
        self.rx_line = bytearray()
        self.print_lock = threading.Lock()

    def record(self, direction: str, data: bytes, partial: bool = False) -> None:
        text_data = data.rstrip(b"\r\n")
        text = text_data.decode("ascii", errors="replace")
        suffix = " [partial]" if partial else ""
        with self.print_lock:
            print(f"{timestamp()} [{direction}] {text}{suffix}", flush=True)
            if self.show_hex:
                hex_data = " ".join(f"{byte:02X}" for byte in data)
                print(
                    f"{timestamp()} [{direction} HEX] {hex_data}",
                    flush=True,
                )

    def accept(self, data: bytes) -> None:
        for byte in data:
            self.rx_line.append(byte)
            if byte == 0x0A:
                self.record("RX", bytes(self.rx_line))
                self.rx_line.clear()

    def flush_partial(self) -> None:
        if self.rx_line:
            self.record("RX", bytes(self.rx_line), partial=True)
            self.rx_line.clear()


def show_ports() -> int:
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        details = [port.description]
        if port.vid is not None and port.pid is not None:
            details.append(f"VID:PID={port.vid:04X}:{port.pid:04X}")
        print(f"{port.device}\t{' | '.join(details)}")
    return 0


def send_line(
    port: serial.Serial,
    output: SerialOutput,
    line: str,
) -> None:
    try:
        payload = line.encode("ascii") + b"\n"
    except UnicodeEncodeError as exc:
        raise ValueError("commands must contain ASCII characters only") from exc
    port.write(payload)
    port.flush()
    output.record("TX", payload)


def read_serial(
    port: serial.Serial,
    output: SerialOutput,
    stop_event: threading.Event,
    errors: list[BaseException],
) -> None:
    try:
        while not stop_event.is_set():
            waiting = port.in_waiting
            data = port.read(waiting if waiting > 0 else 1)
            if data:
                output.accept(data)
    except (OSError, serial.SerialException) as exc:
        errors.append(exc)
        stop_event.set()


def non_negative_seconds(value: str) -> float:
    seconds = float(value)
    if seconds < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return seconds


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and write the STM32F103 USART1 debug console."
    )
    parser.add_argument("--list", action="store_true", help="list serial ports")
    parser.add_argument("--port", default="COM14", help="serial port (default: COM14)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument(
        "--send",
        action="append",
        default=[],
        metavar="COMMAND",
        help="send an ASCII command; may be specified more than once",
    )
    parser.add_argument(
        "--listen-seconds",
        type=non_negative_seconds,
        default=10.0,
        help="receive duration; zero means until Ctrl+C (default: 10)",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="read commands from the keyboard; /quit exits",
    )
    parser.add_argument("--hex", action="store_true", help="show raw bytes in hex")
    return parser.parse_args()


def run(args: argparse.Namespace) -> int:
    if args.list:
        return show_ports()
    if not 1 <= args.baud <= 4_000_000:
        print("baud rate must be between 1 and 4000000", file=sys.stderr)
        return 2

    output = SerialOutput(args.hex)
    stop_event = threading.Event()
    reader_errors: list[BaseException] = []
    port = serial.Serial(
        port=None,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    port.dtr = False
    port.rts = False
    port.port = args.port

    reader: threading.Thread | None = None
    try:
        port.open()
        print(f"{timestamp()} [OPEN] {args.port} {args.baud}-8-N-1", flush=True)

        reader = threading.Thread(
            target=read_serial,
            args=(port, output, stop_event, reader_errors),
            name="serial-reader",
            daemon=True,
        )
        reader.start()

        for command in args.send:
            send_line(port, output, command)

        if args.interactive:
            while not stop_event.is_set():
                try:
                    line = input("> ")
                except EOFError:
                    break
                if line == "/quit":
                    break
                send_line(port, output, line)
        else:
            deadline = (
                None
                if args.listen_seconds == 0
                else time.monotonic() + args.listen_seconds
            )
            while not stop_event.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, serial.SerialException) as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_event.set()
        if reader is not None:
            reader.join(timeout=0.2)
        output.flush_partial()
        if port.is_open:
            port.close()
            print(f"{timestamp()} [CLOSE] {args.port}", flush=True)

    if reader_errors:
        print(f"serial error: {reader_errors[0]}", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
