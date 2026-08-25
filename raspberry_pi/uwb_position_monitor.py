#!/usr/bin/env python3
"""Show a fixed, non-scrolling dashboard for the UWB robot tag."""

from __future__ import annotations

import argparse
import csv
import math
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path


DEFAULT_PORT = 4220
OFFLINE_AFTER_SEC = 2.0


@dataclass
class PositionSnapshot:
    source_ip: str
    received_at: float
    fields: dict[str, str]
    raw: str

    def number(self, name: str) -> float | None:
        try:
            return float(self.fields[name])
        except (KeyError, ValueError):
            return None


def parse_packet(payload: bytes, source_ip: str, now: float) -> PositionSnapshot | None:
    text = payload.decode("utf-8", errors="replace").strip()
    parts = text.split(",")
    if not parts or parts[0] != "UWB_POS":
        return None
    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()
    return PositionSnapshot(source_ip, now, fields, text)


def robot_center(
    tag_x: float,
    tag_y: float,
    heading_deg: float,
    forward_m: float,
    left_m: float,
) -> tuple[float, float]:
    heading = math.radians(heading_deg)
    center_x = tag_x - forward_m * math.cos(heading) + left_m * math.sin(heading)
    center_y = tag_y - forward_m * math.sin(heading) - left_m * math.cos(heading)
    return center_x, center_y


def draw(
    snapshot: PositionSnapshot | None,
    now: float,
    packet_times: deque[float],
    heading_deg: float,
    forward_m: float,
    left_m: float,
    port: int,
) -> None:
    while packet_times and now - packet_times[0] > 2.0:
        packet_times.popleft()
    rate_hz = len(packet_times) / 2.0

    lines = [
        "UWB POSITION MONITOR  (fixed dashboard; no scrolling)",
        f"UDP port={port}  tag offset=forward {forward_m:.3f} m, left {left_m:.3f} m",
        "",
    ]
    if snapshot is None or now - snapshot.received_at > OFFLINE_AFTER_SEC:
        age = "never" if snapshot is None else f"{now - snapshot.received_at:.1f}s ago"
        lines.extend(
            [
                "TAG  OFFLINE",
                f"Last packet: {age}",
                "Check tag power, hotspot, Raspberry Pi IP, and four anchor powers.",
            ]
        )
    else:
        tag_x = snapshot.number("tag_x_m")
        tag_y = snapshot.number("tag_y_m")
        raw_x = snapshot.number("raw_x_m")
        raw_y = snapshot.number("raw_y_m")
        rmse = snapshot.number("rmse_m")
        ranges = snapshot.fields.get("ranges_m", "-").split(":")
        quality = snapshot.fields.get("quality", "?")
        rssi = snapshot.fields.get("wifi_rssi", "?")
        age = now - snapshot.received_at

        lines.append(
            f"TAG  ONLINE  {snapshot.source_ip}  age={age:.2f}s  rate={rate_hz:.1f} Hz"
        )
        lines.append(f"Quality={quality}  range RMSE={rmse if rmse is not None else '-'} m  Wi-Fi RSSI={rssi} dBm")
        lines.append("")
        if tag_x is not None and tag_y is not None:
            center_x, center_y = robot_center(
                tag_x, tag_y, heading_deg, forward_m, left_m
            )
            lines.extend(
                [
                    f"Filtered tag position : x={tag_x:7.3f} m   y={tag_y:7.3f} m",
                    f"Raw tag position      : x={raw_x if raw_x is not None else 0.0:7.3f} m   y={raw_y if raw_y is not None else 0.0:7.3f} m",
                    f"Robot center position : x={center_x:7.3f} m   y={center_y:7.3f} m",
                    f"Center correction uses MANUAL heading={heading_deg:.1f} deg",
                ]
            )
        else:
            lines.append("Invalid position packet")
        lines.append("")
        lines.append("ANCHOR RANGES")
        for index in range(4):
            value = ranges[index] if index < len(ranges) else "-"
            lines.append(f"  A{index + 1}: {value:>7} m")
        if quality != "OK":
            lines.extend(
                [
                    "",
                    "WARNING: quality is CHECK. Inspect anchor placement and line of sight.",
                ]
            )

    lines.extend(
        [
            "",
            "Keep the robot facing +x while using manual heading 0 deg.",
            "When the robot rotates, tag position stays valid but center correction needs live yaw.",
            "Exit: Ctrl+C",
        ]
    )
    sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(lines) + "\n")
    sys.stdout.flush()


def append_csv(path: Path, snapshot: PositionSnapshot, now_wall: float) -> None:
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        if new_file:
            writer.writerow(
                [
                    "timestamp",
                    "source_ip",
                    "tag_x_m",
                    "tag_y_m",
                    "raw_x_m",
                    "raw_y_m",
                    "rmse_m",
                    "ranges_m",
                    "quality",
                ]
            )
        writer.writerow(
            [
                f"{now_wall:.3f}",
                snapshot.source_ip,
                snapshot.fields.get("tag_x_m", ""),
                snapshot.fields.get("tag_y_m", ""),
                snapshot.fields.get("raw_x_m", ""),
                snapshot.fields.get("raw_y_m", ""),
                snapshot.fields.get("rmse_m", ""),
                snapshot.fields.get("ranges_m", ""),
                snapshot.fields.get("quality", ""),
            ]
        )


def run(args: argparse.Namespace) -> None:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", args.port))
    udp.setblocking(False)

    snapshot: PositionSnapshot | None = None
    packet_times: deque[float] = deque()
    next_draw = 0.0
    try:
        while True:
            now = time.monotonic()
            while True:
                try:
                    payload, address = udp.recvfrom(1024)
                except BlockingIOError:
                    break
                parsed = parse_packet(payload, address[0], time.monotonic())
                if parsed is None:
                    continue
                snapshot = parsed
                packet_times.append(parsed.received_at)
                if args.csv is not None:
                    append_csv(args.csv, parsed, time.time())

            if now >= next_draw:
                draw(
                    snapshot,
                    now,
                    packet_times,
                    args.heading_deg,
                    args.tag_forward_m,
                    args.tag_left_m,
                    args.port,
                )
                next_draw = now + 0.2
            time.sleep(0.01)
    finally:
        udp.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--heading-deg", type=float, default=0.0)
    parser.add_argument("--tag-forward-m", type=float, default=0.110)
    parser.add_argument("--tag-left-m", type=float, default=0.025)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    run(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nUWB position monitor stopped.")
        raise SystemExit(0)
    except OSError as error:
        print(f"ERROR: Cannot open UDP port: {error}")
        raise SystemExit(1)
