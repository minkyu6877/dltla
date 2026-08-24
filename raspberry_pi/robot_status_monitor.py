#!/usr/bin/env python3
"""Show a non-scrolling UDP status dashboard for both ESP32 robots."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path


WHEELS = ("FL", "FR", "RL", "RR")
OFFLINE_AFTER_SEC = 1.5


@dataclass
class RobotSnapshot:
    source_ip: str
    received_at: float
    fields: dict[str, str]
    raw: str

    def vector(self, name: str) -> tuple[float, float, float, float] | None:
        raw_value = self.fields.get(name)
        if raw_value is None:
            return None
        try:
            values = tuple(float(value) for value in raw_value.split(":"))
        except ValueError:
            return None
        if len(values) != 4:
            return None
        return values  # type: ignore[return-value]


def parse_status(payload: bytes, source_ip: str, now: float) -> RobotSnapshot | None:
    text = payload.decode("utf-8", errors="replace").strip()
    parts = text.split(",")
    if not parts or parts[0] != "STATUS":
        return None

    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()
    return RobotSnapshot(source_ip, now, fields, text)


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing config value: {key}")
    if len(config["robot_ips"]) != 2:
        raise ValueError("config.json robot_ips must contain exactly two IP addresses")
    return config


def wheel_health(target: float, rpm: float, pwm: float) -> str:
    if abs(target) < 3.0:
        return "STOP"
    if abs(rpm) < 1.0 and abs(pwm) >= 0.45:
        return "ENCODER?"
    if abs(rpm) >= 3.0 and target * rpm < 0.0:
        return "DIRECTION?"
    return "OK"


def robot_lines(
    robot_number: int,
    expected_ip: str,
    snapshot: RobotSnapshot | None,
    now: float,
) -> list[str]:
    if snapshot is None or now - snapshot.received_at > OFFLINE_AFTER_SEC:
        age = "never" if snapshot is None else f"{now - snapshot.received_at:.1f}s ago"
        return [
            f"ROBOT {robot_number}  OFFLINE  {expected_ip}  (last: {age})",
            "  Waiting for STATUS... Check power, hotspot, IP, and UDP ports.",
        ]

    fields = snapshot.fields
    age = now - snapshot.received_at
    state = fields.get("state", "?")
    firmware = fields.get("fw", "?")
    rssi = fields.get("rssi", "?")
    event = fields.get("event", "-")
    targets = snapshot.vector("target_rpm")
    rpms = snapshot.vector("rpm")
    pwms = snapshot.vector("drive_pwm")

    lines = [
        f"ROBOT {robot_number}  ONLINE   {expected_ip}  state={state}  RSSI={rssi} dBm",
        f"  firmware={firmware}  update={age:.1f}s  event={event}",
    ]
    if "distance_cm" in fields:
        lines.append(
            f"  ULTRASONIC  distance={fields.get('distance_cm', '-')} cm  "
            f"role={fields.get('sensor_role', '-')}  "
            f"stop={fields.get('stop_cm', '-')} cm  latch={fields.get('obstacle', '-')}"
        )
    if "imu_ok" in fields:
        lines.append(
            f"  IMU  ok={fields.get('imu_ok', '-')}  "
            f"accel={fields.get('accel_g', '-')} g  "
            f"gyro={fields.get('gyro_dps', '-')} dps  "
            f"att={fields.get('att_deg', '-')} deg"
        )
    if targets is None or rpms is None or pwms is None:
        lines.append("  Invalid or incomplete STATUS packet")
        lines.append(f"  RAW: {snapshot.raw[:180]}")
        return lines

    lines.extend(
        [
            "  WHEEL     TARGET RPM    ACTUAL RPM     PWM       CHECK",
            "  ---------------------------------------------------------",
        ]
    )
    for wheel, target, rpm, pwm in zip(WHEELS, targets, rpms, pwms):
        health = wheel_health(target, rpm, pwm)
        lines.append(f"  {wheel:<5} {target:>11.1f} {rpm:>13.1f} {pwm:>9.2f}   {health}")
    return lines


def draw_dashboard(
    robot_ips: list[str],
    snapshots: dict[str, RobotSnapshot],
    now: float,
    packet_count: int,
) -> None:
    lines = [
        "ESP32 ROBOT STATUS MONITOR  (fixed dashboard; no scrolling)",
        "Order: FL=front-left, FR=front-right, RL=rear-left, RR=rear-right",
        "CHECK: ENCODER?=no RPM feedback, DIRECTION?=RPM sign is reversed",
        "",
    ]
    for index, ip in enumerate(robot_ips, start=1):
        lines.extend(robot_lines(index, ip, snapshots.get(ip), now))
        lines.append("")
    lines.extend(
        [
            f"Received packets: {packet_count}",
            "Monitor sends PING only; it never sends a movement command.",
            "Exit: Ctrl+C",
        ]
    )
    sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(lines) + "\n")
    sys.stdout.flush()


def run(config: dict) -> None:
    robot_ips = [str(ip) for ip in config["robot_ips"]]
    command_port = int(config["command_port"])
    status_port = int(config["status_port"])
    addresses = [(ip, command_port) for ip in robot_ips]

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp.bind(("0.0.0.0", status_port))
    except OSError as exc:
        udp.close()
        raise RuntimeError(
            f"Cannot use UDP status port {status_port}. Stop manual_drive.py and "
            "qr_dual_robot.py before running this standalone monitor."
        ) from exc
    udp.setblocking(False)

    snapshots: dict[str, RobotSnapshot] = {}
    packet_count = 0
    next_ping = 0.0
    next_draw = 0.0

    try:
        while True:
            now = time.monotonic()
            if now >= next_ping:
                for address in addresses:
                    udp.sendto(b"PING", address)
                next_ping = now + 1.0

            while True:
                try:
                    payload, address = udp.recvfrom(1024)
                except BlockingIOError:
                    break
                packet_count += 1
                snapshot = parse_status(payload, address[0], time.monotonic())
                if snapshot is not None:
                    snapshots[address[0]] = snapshot

            if now >= next_draw:
                draw_dashboard(robot_ips, snapshots, now, packet_count)
                next_draw = now + 0.2
            time.sleep(0.01)
    finally:
        udp.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    args = parser.parse_args()
    run(load_config(args.config))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStatus monitor stopped.")
        raise SystemExit(0)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
