#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Iterable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BUTTONS = {
    "Y": 0x0001,
    "B": 0x0002,
    "A": 0x0004,
    "X": 0x0008,
    "L": 0x0010,
    "R": 0x0020,
    "ZL": 0x0040,
    "ZR": 0x0080,
    "MINUS": 0x0100,
    "-": 0x0100,
    "PLUS": 0x0200,
    "+": 0x0200,
    "LCLICK": 0x0400,
    "LS": 0x0400,
    "RCLICK": 0x0800,
    "RS": 0x0800,
    "HOME": 0x1000,
    "CAPTURE": 0x2000,
}

HATS = {
    "UP": 0x00,
    "UP_RIGHT": 0x01,
    "RIGHT": 0x02,
    "DOWN_RIGHT": 0x03,
    "DOWN": 0x04,
    "DOWN_LEFT": 0x05,
    "LEFT": 0x06,
    "UP_LEFT": 0x07,
    "NEUTRAL": 0x08,
    "CENTER": 0x08,
    "-": 0x08,
}

STICK_VALUES = {
    "MIN": 0x00,
    "MAX": 0xFF,
    "CENTER": 0x80,
    "NEUTRAL": 0x80,
}


@dataclass(frozen=True)
class Report:
    buttons: int = 0
    hat: int = 0x08
    lx: int = 0x80
    ly: int = 0x80
    rx: int = 0x80
    ry: int = 0x80
    key_mode: int = 0x00
    key_value: int = 0x00
    reserved: int = 0x00

    def to_bytes(self) -> bytes:
        values = [
            0xAB,
            self.buttons & 0xFF,
            (self.buttons >> 8) & 0xFF,
            self.hat & 0xFF,
            self.lx & 0xFF,
            self.ly & 0xFF,
            self.rx & 0xFF,
            self.ry & 0xFF,
            self.key_mode & 0xFF,
            self.key_value & 0xFF,
            self.reserved & 0xFF,
        ]
        return bytes(values)


def require_pyserial() -> None:
    if serial is None:
        print("pyserial is not installed. Run: pip install pyserial", file=sys.stderr)
        raise SystemExit(1)


def parse_byte(value: str) -> int:
    text = value.strip().upper()
    if text in STICK_VALUES:
        return STICK_VALUES[text]
    base = 16 if text.startswith("0X") else 10
    number = int(text, base)
    if not 0 <= number <= 0xFF:
        raise argparse.ArgumentTypeError(f"byte value out of range: {value}")
    return number


def parse_hat(value: str) -> int:
    text = value.strip().upper().replace("-", "_")
    if text not in HATS:
        valid = ", ".join(sorted(HATS))
        raise argparse.ArgumentTypeError(f"unknown hat: {value}. valid: {valid}")
    return HATS[text]


def parse_buttons(values: Optional[Iterable[str]]) -> int:
    result = 0
    if not values:
        return result
    for value in values:
        key = value.strip().upper().replace("-", "_")
        if key not in BUTTONS:
            valid = ", ".join(sorted(BUTTONS))
            raise ValueError(f"unknown button: {value}. valid: {valid}")
        result |= BUTTONS[key]
    return result


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_raw_hex(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", "").replace("0X", "")
    parts = [p for p in cleaned.split() if p]
    if len(parts) == 1 and len(parts[0]) == 22:
        parts = [parts[0][i:i + 2] for i in range(0, 22, 2)]
    data = bytes(int(p, 16) for p in parts)
    if len(data) != 11:
        raise ValueError(f"raw frame must be exactly 11 bytes, got {len(data)}")
    return data


def open_serial(port: str, baud: int, timeout: float):
    require_pyserial()
    return serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=timeout)


def send_frame(ser, data: bytes, repeat: int = 1, interval: float = 0.02, verbose: bool = True) -> None:
    if len(data) != 11:
        raise ValueError(f"frame must be exactly 11 bytes, got {len(data)}")
    for index in range(repeat):
        ser.write(data)
        ser.flush()
        if verbose:
            print(f"sent[{index + 1}/{repeat}]: {hex_bytes(data)}")
        if index + 1 < repeat:
            time.sleep(interval)


def build_bin_packets(bin_data: bytes, report: Report = Report()) -> list[bytes]:
    if len(bin_data) != 540:
        raise ValueError(f"bin file must be exactly 540 bytes, got {len(bin_data)}")
    prefix = report.to_bytes()[:10]
    packets = []
    for offset in range(0, len(bin_data), 54):
        chunk = bin_data[offset:offset + 54]
        packet = prefix + chunk
        if len(packet) != 64:
            raise ValueError(f"packet must be exactly 64 bytes, got {len(packet)}")
        packets.append(packet)
    if len(packets) != 10:
        raise ValueError(f"expected 10 packets, got {len(packets)}")
    return packets


def send_packet(ser, data: bytes, interval: float = 0.02, verbose: bool = True, index: int = 1, total: int = 1) -> None:
    if len(data) != 64:
        raise ValueError(f"packet must be exactly 64 bytes, got {len(data)}")
    ser.write(data)
    ser.flush()
    if verbose:
        print(f"sent[{index}/{total}]: {hex_bytes(data)}")
    if index < total:
        time.sleep(interval)


def send_report(ser, report: Report, repeat: int, interval: float, verbose: bool) -> None:
    send_frame(ser, report.to_bytes(), repeat=repeat, interval=interval, verbose=verbose)


def send_neutral(ser, repeat: int = 3, interval: float = 0.02, verbose: bool = True) -> None:
    send_report(ser, Report(), repeat=repeat, interval=interval, verbose=verbose)


def hold_then_release(ser, report: Report, duration: float, repeat_hz: float, verbose: bool) -> None:
    interval = 1.0 / repeat_hz
    repeat = max(1, int(duration * repeat_hz))
    send_report(ser, report, repeat=repeat, interval=interval, verbose=verbose)
    send_neutral(ser, repeat=3, interval=0.02, verbose=verbose)


def list_serial_ports() -> None:
    require_pyserial()
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports found")
        return
    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")


def command_neutral(args) -> None:
    with open_serial(args.port, args.baud, args.timeout) as ser:
        send_neutral(ser, repeat=args.repeat, interval=args.interval, verbose=not args.quiet)


def command_press(args) -> None:
    buttons = parse_buttons(args.buttons)
    report = Report(buttons=buttons)
    with open_serial(args.port, args.baud, args.timeout) as ser:
        hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)


def command_hat(args) -> None:
    report = Report(hat=parse_hat(args.direction))
    with open_serial(args.port, args.baud, args.timeout) as ser:
        hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)


def command_stick(args) -> None:
    report = Report(lx=args.lx, ly=args.ly, rx=args.rx, ry=args.ry)
    with open_serial(args.port, args.baud, args.timeout) as ser:
        hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)


def command_raw(args) -> None:
    data = parse_raw_hex(args.hex)
    with open_serial(args.port, args.baud, args.timeout) as ser:
        send_frame(ser, data, repeat=args.repeat, interval=args.interval, verbose=not args.quiet)
        if args.release:
            send_neutral(ser, repeat=3, interval=0.02, verbose=not args.quiet)


def command_send_bin(args) -> None:
    bin_path = Path(args.bin_file)
    bin_data = bin_path.read_bytes()
    report = Report(
        buttons=parse_buttons(args.buttons),
        hat=parse_hat(args.hat),
        lx=args.lx,
        ly=args.ly,
        rx=args.rx,
        ry=args.ry,
        key_mode=args.key_mode,
        key_value=args.key_value,
    )
    packets = build_bin_packets(bin_data, report=report)
    with open_serial(args.port, args.baud, args.timeout) as ser:
        for index, packet in enumerate(packets, start=1):
            send_packet(ser, packet, interval=args.interval, verbose=not args.quiet, index=index, total=len(packets))
        if args.release:
            send_neutral(ser, repeat=3, interval=0.02, verbose=not args.quiet)


def command_demo(args) -> None:
    with open_serial(args.port, args.baud, args.timeout) as ser:
        send_neutral(ser, verbose=not args.quiet)
        tests = [
            ("A", Report(buttons=BUTTONS["A"])),
            ("B", Report(buttons=BUTTONS["B"])),
            ("X", Report(buttons=BUTTONS["X"])),
            ("Y", Report(buttons=BUTTONS["Y"])),
            ("UP", Report(hat=HATS["UP"])),
            ("RIGHT", Report(hat=HATS["RIGHT"])),
            ("DOWN", Report(hat=HATS["DOWN"])),
            ("LEFT", Report(hat=HATS["LEFT"])),
            ("LX_MIN", Report(lx=0x00)),
            ("LX_MAX", Report(lx=0xFF)),
            ("LY_MIN", Report(ly=0x00)),
            ("LY_MAX", Report(ly=0xFF)),
        ]
        for name, report in tests:
            print(f"test: {name}")
            hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)
            time.sleep(args.pause)
        send_neutral(ser, verbose=not args.quiet)


def command_interactive(args) -> None:
    help_text = """
commands:
  neutral
  press A
  press A B
  hat UP
  hat DOWN_RIGHT
  stick lx=0 ly=128 rx=128 ry=128
  raw AB 04 00 08 80 80 80 80 00 00 00
  quit
"""
    print(help_text.strip())
    with open_serial(args.port, args.baud, args.timeout) as ser:
        send_neutral(ser, verbose=not args.quiet)
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line.lower() in {"q", "quit", "exit"}:
                break
            try:
                words = line.split()
                command = words[0].lower()
                if command == "neutral":
                    send_neutral(ser, verbose=not args.quiet)
                elif command == "press":
                    report = Report(buttons=parse_buttons(words[1:]))
                    hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)
                elif command == "hat":
                    report = Report(hat=parse_hat(words[1]))
                    hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)
                elif command == "stick":
                    stick = {"lx": 0x80, "ly": 0x80, "rx": 0x80, "ry": 0x80}
                    for item in words[1:]:
                        key, value = item.split("=", 1)
                        key = key.lower()
                        if key not in stick:
                            raise ValueError(f"unknown stick field: {key}")
                        stick[key] = parse_byte(value)
                    report = Report(**stick)
                    hold_then_release(ser, report, args.duration, args.repeat_hz, verbose=not args.quiet)
                elif command == "raw":
                    data = parse_raw_hex(" ".join(words[1:]))
                    send_frame(ser, data, repeat=1, interval=args.interval, verbose=not args.quiet)
                else:
                    print("unknown command")
            except Exception as exc:
                print(f"error: {exc}", file=sys.stderr)
        send_neutral(ser, verbose=not args.quiet)


def add_common_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--quiet", action="store_true")


def add_timing_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--duration", type=float, default=0.12)
    parser.add_argument("--repeat-hz", type=float, default=50.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NX2 v2.08 serial frame tester")
    subparsers = parser.add_subparsers(dest="command", required=True)

    p = subparsers.add_parser("list")
    p.set_defaults(func=lambda args: list_serial_ports())

    p = subparsers.add_parser("neutral")
    add_common_serial_args(p)
    p.add_argument("--repeat", type=int, default=3)
    p.add_argument("--interval", type=float, default=0.02)
    p.set_defaults(func=command_neutral)

    p = subparsers.add_parser("press")
    add_common_serial_args(p)
    add_timing_args(p)
    p.add_argument("buttons", nargs="+")
    p.set_defaults(func=command_press)

    p = subparsers.add_parser("hat")
    add_common_serial_args(p)
    add_timing_args(p)
    p.add_argument("direction")
    p.set_defaults(func=command_hat)

    p = subparsers.add_parser("stick")
    add_common_serial_args(p)
    add_timing_args(p)
    p.add_argument("--lx", type=parse_byte, default=0x80)
    p.add_argument("--ly", type=parse_byte, default=0x80)
    p.add_argument("--rx", type=parse_byte, default=0x80)
    p.add_argument("--ry", type=parse_byte, default=0x80)
    p.set_defaults(func=command_stick)

    p = subparsers.add_parser("raw")
    add_common_serial_args(p)
    p.add_argument("--hex", required=True)
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--interval", type=float, default=0.02)
    p.add_argument("--release", action="store_true")
    p.set_defaults(func=command_raw)

    p = subparsers.add_parser("send-bin")
    add_common_serial_args(p)
    p.add_argument("bin_file")
    p.add_argument("--interval", type=float, default=0.02)
    p.add_argument("--release", action="store_true")
    p.add_argument("--buttons", nargs="*", default=[])
    p.add_argument("--hat", default="NEUTRAL")
    p.add_argument("--lx", type=parse_byte, default=0x80)
    p.add_argument("--ly", type=parse_byte, default=0x80)
    p.add_argument("--rx", type=parse_byte, default=0x80)
    p.add_argument("--ry", type=parse_byte, default=0x80)
    p.add_argument("--key-mode", type=parse_byte, default=0x00)
    p.add_argument("--key-value", type=parse_byte, default=0x00)
    p.set_defaults(func=command_send_bin)

    p = subparsers.add_parser("demo")
    add_common_serial_args(p)
    add_timing_args(p)
    p.add_argument("--pause", type=float, default=0.25)
    p.set_defaults(func=command_demo)

    p = subparsers.add_parser("interactive")
    add_common_serial_args(p)
    add_timing_args(p)
    p.add_argument("--interval", type=float, default=0.02)
    p.set_defaults(func=command_interactive)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
        return 0
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
