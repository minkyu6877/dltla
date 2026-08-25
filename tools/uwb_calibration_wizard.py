#!/usr/bin/env python3
"""Run DW1000 anchor calibration over USB serial and save the result."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


def parse_machine_line(line: str) -> tuple[str, dict[str, str]]:
    parts = [part.strip() for part in line.split(",")]
    kind = parts[0] if parts else ""
    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key.strip()] = value.strip()
    return kind, fields


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = available_ports()
    if not ports:
        raise RuntimeError("No serial ports found. Connect the calibration anchor.")
    if len(ports) == 1:
        print(f"Using the only detected port: {ports[0]}")
        return ports[0]
    print("Detected serial ports:")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}: {port}")
    selection = int(input("Select port number: ").strip())
    if selection < 1 or selection > len(ports):
        raise ValueError("Invalid port selection")
    return ports[selection - 1]


def send_line(connection: serial.Serial, message: str) -> None:
    connection.write((message + "\n").encode("ascii"))
    connection.flush()


def wait_for_ready(connection: serial.Serial, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = connection.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(line)
        kind, _ = parse_machine_line(line)
        if kind in ("CAL_READY", "CAL_HELP"):
            return
    raise TimeoutError("Anchor did not become ready. Check firmware and baud rate.")


def save_result(path: Path, anchor: int, result: dict[str, str]) -> None:
    if path.exists():
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            document = {}
    else:
        document = {}
    document.setdefault("format", "dw1000-anchor-calibration-v1")
    document.setdefault("anchors", {})
    integer_fields = {"anchor", "delay", "samples"}
    float_fields = {
        "target_m", "mean_m", "std_m", "min_m", "max_m", "error_m"
    }
    typed_result: dict[str, str | int | float] = {}
    for key, value in result.items():
        if key in integer_fields:
            typed_result[key] = int(value)
        elif key in float_fields:
            typed_result[key] = float(value)
        else:
            typed_result[key] = value
    document["anchors"][f"A{anchor}"] = {
        **typed_result,
        "calibrated_at": datetime.now().isoformat(timespec="seconds"),
    }
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example COM7 or /dev/ttyUSB0")
    parser.add_argument("--anchor", type=int, required=True, choices=range(1, 5))
    parser.add_argument("--distance", type=float, required=True, help="Measured metres")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("uwb_calibration_results.json"),
    )
    args = parser.parse_args()
    if not 0.5 <= args.distance <= 20.0:
        parser.error("--distance must be between 0.5 and 20.0 metres")

    port = choose_port(args.port)
    print(f"Opening {port} at {args.baud} baud")
    with serial.Serial(port, args.baud, timeout=0.25) as connection:
        time.sleep(2.0)  # ESP32 normally resets when the serial port opens.
        connection.reset_input_buffer()
        send_line(connection, "P")
        wait_for_ready(connection, 5.0)

        print(f"Setting anchor number to A{args.anchor}; the ESP32 will restart")
        send_line(connection, f"A,{args.anchor}")
        time.sleep(3.0)
        connection.reset_input_buffer()
        send_line(connection, "P")
        wait_for_ready(connection, 8.0)

        send_line(connection, f"D,{args.distance:.4f}")
        send_line(connection, "S")
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            raw = connection.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(line)
            kind, fields = parse_machine_line(line)
            if kind == "CAL_RESULT":
                save_result(args.output, args.anchor, fields)
                print(f"Saved: {args.output.resolve()}")
                return 0
            if kind == "CAL_ERROR":
                raise RuntimeError(fields.get("reason", line))

    raise TimeoutError("Calibration timed out. Check line of sight and tag power.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
