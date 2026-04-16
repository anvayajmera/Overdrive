#!/usr/bin/env python3
import argparse
import glob
import os
from pathlib import Path
import sys
import time

import serial


START_BYTE = 0xAA
SET_SERVO_ANGLE = 0x02
SET_RELAY_STATE = 0x03
SERVO_COUNT = 4
SERVO_DEFAULT_ANGLES = [100, 100, 100, 100]
RELAY_DEFAULT_ON = False


def _import_constants():
    repo_root = Path(__file__).resolve().parents[1]
    src_path = repo_root / "src"
    if src_path.exists():
        sys.path.insert(0, str(src_path))
        try:
            from constants import (  # type: ignore
                START_BYTE as _START_BYTE,
                SET_SERVO_ANGLE as _SET_SERVO_ANGLE,
                SET_RELAY_STATE as _SET_RELAY_STATE,
                SERVO_COUNT as _SERVO_COUNT,
                SERVO_DEFAULT_ANGLES as _SERVO_DEFAULT_ANGLES,
                RELAY_DEFAULT_ON as _RELAY_DEFAULT_ON,
            )
        except Exception:
            return

        globals()["START_BYTE"] = _START_BYTE
        globals()["SET_SERVO_ANGLE"] = _SET_SERVO_ANGLE
        globals()["SET_RELAY_STATE"] = _SET_RELAY_STATE
        globals()["SERVO_COUNT"] = _SERVO_COUNT
        globals()["SERVO_DEFAULT_ANGLES"] = list(_SERVO_DEFAULT_ANGLES)
        globals()["RELAY_DEFAULT_ON"] = _RELAY_DEFAULT_ON


def guess_port() -> str:
    candidates = []
    candidates.extend(sorted(glob.glob("/dev/serial/by-id/*CP2102*")))
    candidates.extend(sorted(glob.glob("/dev/serial/by-id/*USB*UART*")))
    candidates.extend(["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"])
    for path in candidates:
        if os.path.exists(path):
            return path
    return "/dev/ttyUSB0"


def send_packet(ser: serial.Serial, cmd: int, arg1: int, arg2: int, arg3: int) -> None:
    packet = bytearray(5)
    packet[0] = START_BYTE
    packet[1] = cmd & 0xFF
    packet[2] = arg1 & 0xFF
    packet[3] = arg2 & 0xFF
    packet[4] = arg3 & 0xFF
    ser.write(packet)


def parse_angles(value: str, default_angles: list[int]) -> list[int]:
    if not value:
        return list(default_angles)
    parts = [p.strip() for p in value.split(",") if p.strip()]
    angles = []
    for part in parts:
        try:
            angles.append(int(part))
        except ValueError:
            continue
    if not angles:
        return list(default_angles)
    # If only one angle provided, apply to all servos.
    if len(angles) == 1:
        angles = angles * SERVO_COUNT
    # Pad or trim to SERVO_COUNT.
    if len(angles) < SERVO_COUNT:
        angles.extend([angles[-1]] * (SERVO_COUNT - len(angles)))
    return angles[:SERVO_COUNT]


def open_serial(port: str, baud: int, wait_s: float) -> serial.Serial | None:
    deadline = time.monotonic() + max(0.0, wait_s)
    last_err = None
    while True:
        try:
            return serial.Serial(port, baud, timeout=0.2, write_timeout=0.2)
        except Exception as exc:  # pragma: no cover - best effort
            last_err = exc
            if time.monotonic() >= deadline:
                print(f"Failed to open {port}: {exc}")
                return None
            time.sleep(0.3)


def main() -> int:
    _import_constants()

    parser = argparse.ArgumentParser(
        description="Send safe servo angles immediately after Jetson boot."
    )
    parser.add_argument("--port", default=guess_port())
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument(
        "--angles",
        default="",
        help="Comma-separated servo angles (0-180). One value applies to all.",
    )
    parser.add_argument("--repeat-seconds", type=float, default=2.0)
    parser.add_argument("--interval", type=float, default=0.12)
    parser.add_argument("--wait-seconds", type=float, default=8.0)
    parser.add_argument("--relay-off", action="store_true", default=True)
    args = parser.parse_args()

    angles = parse_angles(args.angles, SERVO_DEFAULT_ANGLES)

    ser = open_serial(args.port, args.baud, args.wait_seconds)
    if ser is None:
        return 1

    start = time.monotonic()
    while time.monotonic() - start < args.repeat_seconds:
        for idx in range(SERVO_COUNT):
            angle = max(0, min(180, int(angles[idx])))
            send_packet(ser, SET_SERVO_ANGLE, idx, angle, 0)
        if args.relay_off:
            send_packet(ser, SET_RELAY_STATE, 1 if RELAY_DEFAULT_ON else 0, 0, 0)
        time.sleep(args.interval)

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
