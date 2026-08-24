#!/usr/bin/env python3
"""Safely test mecanum motions with the keyboard over UDP.

Works in both Windows terminals and Raspberry Pi/Linux terminals without
third-party keyboard packages.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Motion:
    vx: float
    vy: float
    w: float
    name: str

    def packet(self) -> str:
        return f"V,{self.vx:.3f},{self.vy:.3f},{self.w:.3f},MANUAL_{self.name}"


class KeyboardReader:
    """Non-blocking single-key input for Windows and POSIX terminals."""

    ARROW_MAP_WINDOWS = {
        "H": "w",  # up
        "P": "s",  # down
        "K": "a",  # left
        "M": "d",  # right
    }
    ARROW_MAP_POSIX = {
        "[A": "w",
        "[B": "s",
        "[D": "a",
        "[C": "d",
    }

    def __init__(self):
        self._windows = os.name == "nt"
        self._old_terminal = None
        self._fd = None
        self._msvcrt = None

    def __enter__(self) -> "KeyboardReader":
        if self._windows:
            import msvcrt

            self._msvcrt = msvcrt
        else:
            import termios
            import tty

            if not sys.stdin.isatty():
                raise RuntimeError("manual_drive.py must run in an interactive terminal")
            self._fd = sys.stdin.fileno()
            self._old_terminal = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if not self._windows and self._old_terminal is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_terminal)

    def read_key(self) -> str | None:
        if self._windows:
            if not self._msvcrt.kbhit():
                return None
            key = self._msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                arrow_code = self._msvcrt.getwch()
                return self.ARROW_MAP_WINDOWS.get(arrow_code)
            return key.lower()

        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return None

        key = sys.stdin.read(1)
        if key != "\x1b":
            return key.lower()

        sequence = ""
        deadline = time.monotonic() + 0.02
        while len(sequence) < 2 and time.monotonic() < deadline:
            ready, _, _ = select.select([sys.stdin], [], [], 0.005)
            if ready:
                sequence += sys.stdin.read(1)
        return self.ARROW_MAP_POSIX.get(sequence)


class ManualFleet:
    def __init__(self, robot_ips: list[str], command_port: int, status_port: int):
        if len(robot_ips) != 2:
            raise ValueError("config.json robot_ips must contain exactly two IP addresses")
        self.addresses = [(ip, command_port) for ip in robot_ips]
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind(("0.0.0.0", status_port))
        except OSError as exc:
            self.socket.close()
            raise RuntimeError(
                f"Cannot bind UDP status port {status_port}. "
                "Stop qr_dual_robot.py before running manual_drive.py."
            ) from exc
        self.socket.setblocking(False)
        self.last_status: dict[str, tuple[float, str]] = {}

    def send_to(self, message: str, selected: tuple[int, ...]) -> None:
        payload = message.encode("utf-8")
        for index in selected:
            self.socket.sendto(payload, self.addresses[index])

    def stop_all(self) -> None:
        for _ in range(5):
            self.send_to("STOP", (0, 1))
            time.sleep(0.03)

    def receive_status(self) -> None:
        while True:
            try:
                payload, address = self.socket.recvfrom(768)
            except BlockingIOError:
                return
            text = payload.decode("utf-8", errors="replace").strip()
            self.last_status[address[0]] = (time.monotonic(), text)

    def online(self, ip: str, now: float, timeout: float = 1.5) -> bool:
        status = self.last_status.get(ip)
        return status is not None and now - status[0] <= timeout

    def status_fields(self, ip: str) -> dict[str, str]:
        status = self.last_status.get(ip)
        if status is None:
            return {}
        fields: dict[str, str] = {}
        for part in status[1].split(",")[1:]:
            if "=" in part:
                key, value = part.split("=", 1)
                fields[key.strip()] = value.strip()
        return fields

    def close(self) -> None:
        self.socket.close()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing config value: {key}")
    return config


def motion_map(linear: float, rotation: float) -> dict[str, Motion]:
    # Halving both diagonal components keeps the fastest wheel command similar
    # to the straight/strafe command.
    diagonal = linear / 2.0
    return {
        "w": Motion(linear, 0.0, 0.0, "FORWARD"),
        "s": Motion(-linear, 0.0, 0.0, "BACKWARD"),
        "a": Motion(0.0, linear, 0.0, "STRAFE_LEFT"),
        "d": Motion(0.0, -linear, 0.0, "STRAFE_RIGHT"),
        "q": Motion(diagonal, diagonal, 0.0, "DIAG_FORWARD_LEFT"),
        "e": Motion(diagonal, -diagonal, 0.0, "DIAG_FORWARD_RIGHT"),
        "z": Motion(-diagonal, diagonal, 0.0, "DIAG_BACKWARD_LEFT"),
        "c": Motion(-diagonal, -diagonal, 0.0, "DIAG_BACKWARD_RIGHT"),
        "j": Motion(0.0, 0.0, rotation, "ROTATE_CCW"),
        "l": Motion(0.0, 0.0, -rotation, "ROTATE_CW"),
    }


def paired_motion_map(linear: float, rotation: float) -> dict[str, tuple[str, dict[int, Motion]]]:
    """Robot 2 strafes around Robot 1 while keeping its nose pointed inward.

    Initial pose: Robot 2 is directly behind Robot 1 and faces Robot 1's rear.
    Matching the yaw direction of both robots keeps Robot 2 pointed at Robot 1.
    """
    return {
        "u": (
            "R1_CCW + R2_CCW_ORBIT_INWARD",
            {
                0: Motion(0.0, 0.0, rotation, "R1_ROTATE_CCW"),
                1: Motion(0.0, -linear, rotation, "R2_ORBIT_CCW_INWARD"),
            },
        ),
        "i": (
            "R1_CW + R2_CW_ORBIT_INWARD",
            {
                0: Motion(0.0, 0.0, -rotation, "R1_ROTATE_CW"),
                1: Motion(0.0, linear, -rotation, "R2_ORBIT_CW_INWARD"),
            },
        ),
    }


def clear_and_draw(
    config: dict,
    fleet: ManualFleet,
    selected: tuple[int, ...],
    motion_name: str,
    now: float,
    deadline: float,
    linear: float,
    rotation: float,
    speed_step: float,
    safety_message: str,
) -> None:
    selection_name = {(0,): "ROBOT 1", (1,): "ROBOT 2", (0, 1): "BOTH"}[selected]
    robot_ips = list(config["robot_ips"])
    remaining = max(0.0, deadline - now) if motion_name != "STOP" else 0.0

    lines = [
        "Manual Mecanum Controller",
        "",
        f"Selected : {selection_name}",
        f"Motion   : {motion_name}  (dead-man {remaining:.1f}s)",
        f"Speed    : linear={linear:.2f}  rotation={rotation:.2f}  step={speed_step:.2f}",
        f"Safety   : {safety_message or 'READY'}",
    ]
    for number, ip in enumerate(robot_ips, start=1):
        online = fleet.online(ip, now)
        fields = fleet.status_fields(ip)
        lines.append(
            f"Robot {number}  : {'ONLINE ' if online else 'OFFLINE'}  {ip}  "
            f"state={fields.get('state', '?')}  RSSI={fields.get('rssi', '?')}"
        )
        lines.append(f"  target RPM: {fields.get('target_rpm', '-')}  [FL:FR:RL:RR]")
        lines.append(f"  actual RPM: {fields.get('rpm', '-')}  [FL:FR:RL:RR]")
        lines.append(f"  drive PWM : {fields.get('drive_pwm', '-')}  [FL:FR:RL:RR]")
        lines.append(
            f"  ultrasonic: {fields.get('distance_cm', '-')} cm  "
            f"role={fields.get('sensor_role', '-')}  "
            f"stop={fields.get('stop_cm', '-')} cm  latch={fields.get('obstacle', '-')}"
        )

    lines.extend([
        "",
        "Select: 1=Robot1  2=Robot2  3=Both",
        "Move  : W/S=forward/back  A/D=left/right strafe",
        "Diag  : Q/E=forward-left/right  Z/C=back-left/right",
        "Rotate: J=CCW  L=CW",
        "Orbit : U=CCW  I=CW (R2 starts behind R1 and faces R1 rear)",
        "Speed : [ / ]=linear down/up   - / = (or +)=rotation down/up",
        "Safety: SPACE=STOP  X=STOP and exit",
        "",
        "Hold/repeat a motion key. Releasing it triggers an automatic STOP.",
    ])
    sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(lines) + "\n")
    sys.stdout.flush()


def selected_robots_online(
    config: dict,
    fleet: ManualFleet,
    selected: tuple[int, ...],
    now: float,
) -> bool:
    robot_ips = list(config["robot_ips"])
    return all(fleet.online(robot_ips[index], now) for index in selected)


def obstacle_alert(
    config: dict,
    fleet: ManualFleet,
    commands: dict[int, Motion],
    now: float,
) -> str | None:
    robot_ips = list(config["robot_ips"])
    for index, motion in commands.items():
        ip = robot_ips[index]
        if not fleet.online(ip, now):
            continue
        fields = fleet.status_fields(ip)
        if fields.get("obstacle") != "1":
            continue
        reverse_only = (
            motion.vx < -0.01
            and abs(motion.vy) <= 0.01
            and abs(motion.w) <= 0.01
        )
        if reverse_only:
            continue
        distance = fields.get("distance_cm", "?")
        if index == 0:
            return f"ROBOT 1 LEADER OBSTACLE ({distance} cm)"
        return f"ROBOT 2 COLLISION GUARD ({distance} cm)"
    return None


def run_controller(config: dict, fleet: ManualFleet, keyboard: KeyboardReader) -> None:
    linear = float(config.get("manual_linear_speed", 0.15))
    rotation = float(config.get("manual_rotation_speed", 0.15))
    speed_step = max(0.01, float(config.get("manual_speed_step", 0.02)))
    min_speed = max(0.01, float(config.get("manual_min_speed", 0.05)))
    max_speed = max(min_speed, float(config.get("manual_max_speed", 0.50)))
    linear = min(max(linear, min_speed), max_speed)
    rotation = min(max(rotation, min_speed), max_speed)
    deadman = float(config.get("manual_deadman_sec", 0.70))
    command_hz = max(2.0, float(config.get("command_hz", 10.0)))
    motions = motion_map(linear, rotation)
    paired_motions = paired_motion_map(linear, rotation)

    selected: tuple[int, ...] = (0, 1)
    active_commands: dict[int, Motion] = {}
    active_name = "STOP"
    safety_message = ""
    motion_deadline = 0.0
    next_send = 0.0
    next_ping = 0.0
    next_draw = 0.0

    fleet.stop_all()

    while True:
        now = time.monotonic()
        key = keyboard.read_key()

        if key == "x":
            return
        if key == "1":
            fleet.stop_all()
            selected = (0,)
            active_commands = {}
            active_name = "STOP"
            safety_message = ""
        elif key == "2":
            fleet.stop_all()
            selected = (1,)
            active_commands = {}
            active_name = "STOP"
            safety_message = ""
        elif key == "3":
            fleet.stop_all()
            selected = (0, 1)
            active_commands = {}
            active_name = "STOP"
            safety_message = ""
        elif key == " ":
            fleet.stop_all()
            active_commands = {}
            active_name = "STOP"
            safety_message = "MANUAL STOP"
        elif key in ("[", "]", "-", "=", "+"):
            fleet.stop_all()
            active_commands = {}
            active_name = "STOP"
            safety_message = "SPEED CHANGED; PRESS MOTION KEY AGAIN"
            if key == "[":
                linear = max(min_speed, linear - speed_step)
            elif key == "]":
                linear = min(max_speed, linear + speed_step)
            elif key == "-":
                rotation = max(min_speed, rotation - speed_step)
            else:  # '=' or '+'
                rotation = min(max_speed, rotation + speed_step)
            motions = motion_map(linear, rotation)
            paired_motions = paired_motion_map(linear, rotation)
        elif key in motions:
            if selected_robots_online(config, fleet, selected, now):
                active_commands = {index: motions[key] for index in selected}
                active_name = motions[key].name
                safety_message = ""
                motion_deadline = now + deadman
            else:
                fleet.stop_all()
                active_commands = {}
                active_name = "STOP"
                safety_message = "SELECTED ROBOT OFFLINE"
        elif key in paired_motions:
            fixed_robots = (0, 1)
            if selected_robots_online(config, fleet, fixed_robots, now):
                active_name, active_commands = paired_motions[key]
                safety_message = ""
                motion_deadline = now + deadman
            else:
                fleet.stop_all()
                active_commands = {}
                active_name = "STOP"
                safety_message = "BOTH ROBOTS MUST BE ONLINE"

        if (
            active_commands
            and not selected_robots_online(
                config, fleet, tuple(active_commands.keys()), now
            )
        ):
            fleet.stop_all()
            active_commands = {}
            active_name = "STOP"
            safety_message = "ACTIVE ROBOT OFFLINE"

        if active_commands and now >= motion_deadline:
            fleet.stop_all()
            active_commands = {}
            active_name = "STOP"

        if now >= next_send:
            if active_commands:
                for index, motion in active_commands.items():
                    fleet.send_to(motion.packet(), (index,))
            else:
                fleet.send_to("STOP", (0, 1))
            next_send = now + 1.0 / command_hz

        if now >= next_ping:
            fleet.send_to("PING", (0, 1))
            next_ping = now + 1.0

        fleet.receive_status()

        if active_commands:
            alert = obstacle_alert(
                config, fleet, active_commands, time.monotonic()
            )
            if alert is not None:
                fleet.stop_all()
                active_commands = {}
                active_name = "SAFETY_STOP"
                safety_message = alert

        if now >= next_draw:
            clear_and_draw(
                config,
                fleet,
                selected,
                active_name,
                now,
                motion_deadline,
                linear,
                rotation,
                speed_step,
                safety_message,
            )
            next_draw = now + 0.10

        time.sleep(0.01)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    args = parser.parse_args()
    config = load_config(args.config)
    fleet = ManualFleet(
        list(config["robot_ips"]),
        int(config["command_port"]),
        int(config["status_port"]),
    )
    try:
        with KeyboardReader() as keyboard:
            run_controller(config, fleet, keyboard)
    finally:
        try:
            fleet.stop_all()
        finally:
            fleet.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nSTOP: Ctrl+C")
        raise SystemExit(0)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
