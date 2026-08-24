#!/usr/bin/env python3
"""Browser-based manual control and live telemetry dashboard for two robots."""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Motion:
    vx: float
    vy: float
    w: float
    name: str

    def packet(self) -> str:
        return f"V,{self.vx:.3f},{self.vy:.3f},{self.w:.3f},WEB_{self.name}"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_status(payload: bytes) -> dict[str, str]:
    text = payload.decode("utf-8", errors="replace").strip()
    parts = text.split(",")
    if not parts or parts[0] != "STATUS":
        return {}
    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key.strip()] = value.strip()
    return fields


def regular_motion(key: str, linear: float, rotation: float) -> Motion | None:
    diagonal = linear / 2.0
    motions = {
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
    return motions.get(key)


def orbit_commands(
    direction: int,
    max_linear: float,
    requested_rotation: float,
    gap_cm: float,
    target_gap_cm: float,
    rotation_radius_cm: float,
    center_offset_cm: float,
    distance_kp: float,
    radial_max: float,
) -> tuple[dict[int, Motion], dict[str, float]]:
    """Closed-loop inward-facing orbit using Robot 2's live ultrasonic gap."""
    center_radius_cm = max(rotation_radius_cm, gap_cm + center_offset_cm)

    # Firmware mixes w as the equivalent wheel-edge velocity K * omega.
    # Reduce yaw when needed so the required tangential speed stays at or
    # below the GUI's linear-speed limit.
    effective_rotation = min(
        abs(requested_rotation),
        max_linear * rotation_radius_cm / center_radius_cm,
    )
    tangent = effective_rotation * center_radius_cm / rotation_radius_cm
    radial = clamp(distance_kp * (gap_cm - target_gap_cm), -radial_max, radial_max)

    if direction > 0:  # CCW: from behind Robot 1, tangent is Robot 2's right.
        robot1 = Motion(0.0, 0.0, effective_rotation, "R1_ROTATE_CCW")
        robot2 = Motion(radial, -tangent, effective_rotation, "R2_ORBIT_CCW_INWARD")
    else:  # CW: from behind Robot 1, tangent is Robot 2's left.
        robot1 = Motion(0.0, 0.0, -effective_rotation, "R1_ROTATE_CW")
        robot2 = Motion(radial, tangent, -effective_rotation, "R2_ORBIT_CW_INWARD")

    return {0: robot1, 1: robot2}, {
        "gap_cm": gap_cm,
        "target_gap_cm": target_gap_cm,
        "center_radius_cm": center_radius_cm,
        "tangent_command": tangent,
        "radial_command": radial,
        "rotation_command": effective_rotation,
    }


class RobotCore:
    def __init__(self, config: dict):
        robot_ips = [str(value) for value in config["robot_ips"]]
        if len(robot_ips) != 2:
            raise ValueError("config.json robot_ips must contain exactly two IP addresses")
        self.robot_ips = robot_ips
        command_port = int(config["command_port"])
        status_port = int(config["status_port"])
        self.addresses = [(ip, command_port) for ip in robot_ips]
        self.command_hz = max(2.0, float(config.get("command_hz", 10.0)))
        self.deadman = clamp(float(config.get("web_deadman_sec", 0.45)), 0.25, 1.0)
        self.min_speed = max(0.01, float(config.get("manual_min_speed", 0.05)))
        self.max_speed = max(self.min_speed, float(config.get("manual_max_speed", 0.50)))
        self.default_linear = clamp(
            float(config.get("manual_linear_speed", 0.15)), self.min_speed, self.max_speed
        )
        self.default_rotation = clamp(
            float(config.get("manual_rotation_speed", 0.15)), self.min_speed, self.max_speed
        )
        wheelbase_cm = float(config.get("mecanum_wheelbase_cm", 19.4))
        track_cm = float(config.get("mecanum_track_cm", 30.0))
        self.rotation_radius_cm = (wheelbase_cm + track_cm) / 2.0
        if self.rotation_radius_cm <= 0.0:
            raise ValueError("Mecanum wheelbase/track dimensions must be positive")
        self.orbit_center_offset_cm = max(
            0.0, float(config.get("orbit_center_offset_cm", wheelbase_cm))
        )
        self.orbit_distance_kp = max(0.0, float(config.get("orbit_distance_kp", 0.012)))
        self.orbit_radial_max = clamp(
            float(config.get("orbit_radial_max_speed", 0.08)), 0.01, self.max_speed
        )
        self.orbit_yaw_rpm_kp = max(0.0, float(config.get("orbit_yaw_rpm_kp", 0.002)))
        self.orbit_yaw_correction_max = clamp(
            float(config.get("orbit_yaw_correction_max", 0.04)), 0.0, self.max_speed
        )
        self.orbit_min_gap_cm = max(8.0, float(config.get("orbit_min_gap_cm", 10.0)))
        self.orbit_max_gap_cm = max(
            self.orbit_min_gap_cm, float(config.get("orbit_max_gap_cm", 150.0))
        )

        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.udp.bind(("0.0.0.0", status_port))
        except OSError as exc:
            self.udp.close()
            raise RuntimeError(
                f"Cannot use UDP status port {status_port}. Stop manual_drive.py, "
                "qr_dual_robot.py, and robot_status_monitor.py first."
            ) from exc
        self.udp.setblocking(False)

        self.lock = threading.RLock()
        self.snapshots: dict[str, tuple[float, dict[str, str]]] = {}
        self.active_commands: dict[int, Motion] = {}
        self.active_name = "STOP"
        self.deadline = 0.0
        self.safety_message = "READY"
        self.orbit_direction = 0
        self.orbit_target_gap_cm: float | None = None
        self.orbit_linear_limit = self.default_linear
        self.orbit_rotation_request = self.default_rotation
        self.orbit_telemetry: dict[str, float] = {}
        self.orbit_yaw_error_filtered = 0.0
        self.running = threading.Event()
        self.running.set()
        self.worker = threading.Thread(target=self._run, name="robot-udp", daemon=True)

    def start(self) -> None:
        self._send_stop(repeat=3)
        self.worker.start()

    def _send(self, message: str, indexes: tuple[int, ...]) -> None:
        payload = message.encode("utf-8")
        for index in indexes:
            self.udp.sendto(payload, self.addresses[index])

    def _send_stop(self, repeat: int = 1) -> None:
        for attempt in range(repeat):
            self._send("STOP", (0, 1))
            if attempt + 1 < repeat:
                time.sleep(0.02)

    def stop(self, reason: str = "MANUAL STOP") -> None:
        with self.lock:
            self.active_commands = {}
            self.active_name = "STOP"
            self.deadline = 0.0
            self.safety_message = reason
            self.orbit_direction = 0
        self._send_stop(repeat=3)

    def _fields(self, index: int, now: float, timeout: float = 1.5) -> dict[str, str]:
        snapshot = self.snapshots.get(self.robot_ips[index])
        if snapshot is None or now - snapshot[0] > timeout:
            return {}
        return snapshot[1]

    def _online(self, indexes: tuple[int, ...], now: float) -> bool:
        return all(bool(self._fields(index, now)) for index in indexes)

    @staticmethod
    def _reverse_only(motion: Motion) -> bool:
        return motion.vx < -0.01 and abs(motion.vy) <= 0.01 and abs(motion.w) <= 0.01

    def _alert(
        self,
        indexes: tuple[int, ...],
        now: float,
        commands: dict[int, Motion] | None = None,
    ) -> str | None:
        for index in indexes:
            fields = self._fields(index, now)
            if fields.get("obstacle") != "1":
                continue
            if commands is not None:
                motion = commands.get(index)
                if motion is not None and self._reverse_only(motion):
                    continue
            distance = fields.get("distance_cm", "?")
            if index == 0:
                return f"Robot 1 leader obstacle: {distance} cm"
            return f"Robot 2 collision guard: {distance} cm"
        return None

    def _yaw_rpm(self, index: int, now: float) -> float | None:
        raw = self._fields(index, now).get("rpm")
        if raw is None:
            return None
        try:
            fl, fr, rl, rr = (float(value) for value in raw.split(":"))
        except (ValueError, TypeError):
            return None
        return (-fl + fr - rl + rr) / 4.0

    def _robot2_gap(self, now: float) -> float | None:
        raw = self._fields(1, now).get("distance_cm")
        try:
            gap = float(raw) if raw is not None else None
        except ValueError:
            return None
        if gap is None or not self.orbit_min_gap_cm <= gap <= self.orbit_max_gap_cm:
            return None
        return gap

    def _update_orbit_commands(self, now: float) -> bool:
        if self.orbit_direction == 0 or self.orbit_target_gap_cm is None:
            return True
        gap = self._robot2_gap(now)
        if gap is None:
            return False
        commands, telemetry = orbit_commands(
            self.orbit_direction,
            self.orbit_linear_limit,
            self.orbit_rotation_request,
            gap,
            self.orbit_target_gap_cm,
            self.rotation_radius_cm,
            self.orbit_center_offset_cm,
            self.orbit_distance_kp,
            self.orbit_radial_max,
        )
        yaw_rpm_1 = self._yaw_rpm(0, now)
        yaw_rpm_2 = self._yaw_rpm(1, now)
        yaw_correction = 0.0
        if yaw_rpm_1 is not None and yaw_rpm_2 is not None:
            yaw_error = yaw_rpm_1 - yaw_rpm_2
            self.orbit_yaw_error_filtered = (
                0.8 * self.orbit_yaw_error_filtered + 0.2 * yaw_error
            )
            yaw_correction = clamp(
                self.orbit_yaw_error_filtered * self.orbit_yaw_rpm_kp,
                -self.orbit_yaw_correction_max,
                self.orbit_yaw_correction_max,
            )
            robot2 = commands[1]
            commands[1] = Motion(
                robot2.vx,
                robot2.vy,
                clamp(robot2.w + yaw_correction, -self.max_speed, self.max_speed),
                robot2.name,
            )
            telemetry.update(
                {
                    "yaw_rpm_robot1": yaw_rpm_1,
                    "yaw_rpm_robot2": yaw_rpm_2,
                    "yaw_rpm_error": yaw_error,
                    "yaw_correction": yaw_correction,
                }
            )
        self.active_commands = commands
        self.orbit_telemetry = telemetry
        return True

    def move(
        self,
        key: str,
        selected: tuple[int, ...],
        linear: float,
        rotation: float,
    ) -> tuple[bool, str]:
        key = key.strip().lower()
        linear = clamp(float(linear), self.min_speed, self.max_speed)
        rotation = clamp(float(rotation), self.min_speed, self.max_speed)
        orbit_direction = 1 if key == "u" else -1 if key == "i" else 0
        if orbit_direction == 0:
            motion = regular_motion(key, linear, rotation)
            if motion is None:
                return False, "Unknown motion key"
            if not selected or any(index not in (0, 1) for index in selected):
                return False, "Invalid robot selection"
            commands = {index: motion for index in selected}
            indexes = tuple(commands.keys())
        else:
            indexes = (0, 1)
            commands = {}
        now = time.monotonic()
        with self.lock:
            if not self._online(indexes, now):
                self.active_commands = {}
                self.active_name = "STOP"
                self.safety_message = "Selected robot is offline"
                self.orbit_direction = 0
                self._send_stop()
                return False, self.safety_message
            alert = self._alert(indexes, now, commands)
            if alert is not None:
                self.active_commands = {}
                self.active_name = "STOP"
                self.safety_message = alert
                self.orbit_direction = 0
                self._send_stop()
                return False, alert
            if orbit_direction != 0:
                gap = self._robot2_gap(now)
                if gap is None:
                    self.active_commands = {}
                    self.active_name = "STOP"
                    self.safety_message = "Robot 2 ultrasonic gap is invalid"
                    self.orbit_direction = 0
                    self._send_stop()
                    return False, self.safety_message
                if self.orbit_direction != orbit_direction or self.orbit_target_gap_cm is None:
                    self.orbit_target_gap_cm = gap
                    self.orbit_yaw_error_filtered = 0.0
                self.orbit_direction = orbit_direction
                self.orbit_linear_limit = linear
                self.orbit_rotation_request = rotation
                if not self._update_orbit_commands(now):
                    return False, "Robot 2 ultrasonic gap is invalid"
                commands = self.active_commands
            else:
                self.orbit_direction = 0
            self.active_commands = commands
            self.active_name = " + ".join(motion.name for motion in commands.values())
            self.deadline = now + self.deadman
            retreating = any(
                self._fields(index, now).get("obstacle") == "1"
                and self._reverse_only(motion)
                for index, motion in commands.items()
            )
            self.safety_message = (
                "RETREAT ONLY: release the button after clearing the obstacle"
                if retreating
                else "READY"
            )
        return True, "OK"

    def _receive(self) -> None:
        while True:
            try:
                payload, address = self.udp.recvfrom(1024)
            except BlockingIOError:
                return
            except OSError:
                # Some hosts surface an ICMP "port unreachable" as a UDP read
                # error. It must not terminate the safety/control worker.
                return
            fields = parse_status(payload)
            if fields and address[0] in self.robot_ips:
                with self.lock:
                    self.snapshots[address[0]] = (time.monotonic(), fields)

    def _run(self) -> None:
        next_command = 0.0
        next_ping = 0.0
        next_idle_stop = 0.0
        while self.running.is_set():
            self._receive()
            now = time.monotonic()
            with self.lock:
                indexes = tuple(self.active_commands.keys())
                if indexes:
                    alert = self._alert(indexes, now, self.active_commands)
                    if alert is not None:
                        self.active_commands = {}
                        self.active_name = "SAFETY_STOP"
                        self.safety_message = alert
                        self.orbit_direction = 0
                        self._send_stop(repeat=3)
                    elif not self._online(indexes, now):
                        self.active_commands = {}
                        self.active_name = "COMMUNICATION_STOP"
                        self.safety_message = "Active robot communication lost"
                        self.orbit_direction = 0
                        self._send_stop(repeat=3)
                    elif now >= self.deadline:
                        self.active_commands = {}
                        self.active_name = "DEADMAN_STOP"
                        self.safety_message = "Control button released"
                        self.orbit_direction = 0
                        self._send_stop(repeat=3)
                    elif self.orbit_direction != 0 and not self._update_orbit_commands(now):
                        self.active_commands = {}
                        self.active_name = "ORBIT_SENSOR_STOP"
                        self.safety_message = "Robot 2 ultrasonic gap lost during orbit"
                        self.orbit_direction = 0
                        self._send_stop(repeat=3)

                if now >= next_command and self.active_commands:
                    for index, motion in self.active_commands.items():
                        self._send(motion.packet(), (index,))
                    next_command = now + 1.0 / self.command_hz
                elif not self.active_commands and now >= next_idle_stop:
                    self._send_stop()
                    next_idle_stop = now + 0.5

                if now >= next_ping:
                    self._send("PING", (0, 1))
                    next_ping = now + 1.0
            time.sleep(0.01)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            robots = []
            for index, ip in enumerate(self.robot_ips):
                snapshot = self.snapshots.get(ip)
                online = snapshot is not None and now - snapshot[0] <= 1.5
                robots.append(
                    {
                        "number": index + 1,
                        "ip": ip,
                        "online": online,
                        "age": None if snapshot is None else round(now - snapshot[0], 2),
                        "fields": {} if snapshot is None else snapshot[1],
                    }
                )
            return {
                "robots": robots,
                "motion": self.active_name,
                "safety": self.safety_message,
                "moving": bool(self.active_commands),
                "orbit": {
                    "active": self.orbit_direction != 0,
                    "rotation_radius_cm": round(self.rotation_radius_cm, 2),
                    "center_offset_cm": round(self.orbit_center_offset_cm, 2),
                    **{key: round(value, 3) for key, value in self.orbit_telemetry.items()},
                },
                "defaults": {
                    "linear": self.default_linear,
                    "rotation": self.default_rotation,
                    "minimum": self.min_speed,
                    "maximum": self.max_speed,
                },
            }

    def close(self) -> None:
        self.stop("PROGRAM EXIT")
        self.running.clear()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        self.udp.close()


HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dual Robot Control</title>
<style>
:root{--bg:#07111f;--panel:#101d2e;--line:#24364d;--text:#e8f0fa;--muted:#91a4bb;--good:#35d39a;--warn:#ffbf47;--bad:#ff5c6c;--blue:#51a8ff}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#13243b 0,var(--bg) 48%);color:var(--text);font-family:Inter,"Noto Sans KR",system-ui,sans-serif}
.wrap{max-width:1280px;margin:auto;padding:24px}.top{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:18px}.top h1{font-size:24px;margin:0}.sub{color:var(--muted);font-size:13px;margin-top:5px}
.pill{border:1px solid var(--line);background:#0b1727;border-radius:999px;padding:9px 14px;font-weight:700}.pill.good{color:var(--good)}.pill.bad{color:var(--bad)}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}.card{background:rgba(16,29,46,.96);border:1px solid var(--line);border-radius:16px;padding:18px;box-shadow:0 16px 35px #0005}.card h2{font-size:18px;margin:0 0 14px}.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.between{justify-content:space-between}.label{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}.value{font-weight:700}.online{color:var(--good)}.offline{color:var(--bad)}
.distance{font-size:42px;font-weight:800;margin:10px 0 6px}.bar{height:10px;border-radius:99px;background:#06101d;overflow:hidden}.bar>div{height:100%;width:0;background:var(--good);transition:width .2s}.meta{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin-top:14px}.metric{background:#0a1625;border:1px solid #1b2c42;border-radius:10px;padding:10px}.metric .value{font-size:14px;margin-top:4px;overflow-wrap:anywhere}
table{width:100%;border-collapse:collapse;margin-top:14px;font-variant-numeric:tabular-nums}th,td{padding:7px 5px;text-align:right;border-bottom:1px solid #203147;font-size:13px}th:first-child,td:first-child{text-align:left}th{color:var(--muted);font-weight:600}
.controls{margin-top:16px}.selection button,.pair button{width:auto;padding:10px 14px}.selected{outline:2px solid var(--blue);background:#173657!important}.pad{display:grid;grid-template-columns:repeat(3,74px);gap:8px;justify-content:center;margin:16px 0}.pad button,.rotate button,.pair button{touch-action:none;user-select:none}.rotate,.pair{display:flex;justify-content:center;gap:8px;flex-wrap:wrap}
button{border:1px solid #35506e;border-radius:11px;background:#162a42;color:var(--text);font-weight:750;padding:14px;cursor:pointer;transition:.1s}button:hover{background:#1d3a5c}button:active,.pressed{transform:scale(.96);background:#24527e!important}.stop{background:#7f2030;border-color:#bd4354;font-size:18px;width:100%;margin-top:15px}.stop:hover{background:#a02b3d}
.sliders{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-top:15px}.sliderbox{background:#0a1625;border-radius:11px;padding:12px}.sliderbox input{width:100%}.alert{margin-top:16px;border-radius:12px;padding:13px 16px;border:1px solid #37506d;background:#0b1828;font-weight:700}.alert.bad{color:#ffd6db;border-color:#7f3140;background:#35141d}
.orbitinfo{margin-top:12px;padding:12px;border:1px solid #294662;border-radius:11px;background:#0a1828;color:#bcd0e5;font-variant-numeric:tabular-nums}
@media(max-width:850px){.grid{grid-template-columns:1fr}.wrap{padding:14px}.sliders{grid-template-columns:1fr}.meta{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body><div class="wrap">
<div class="top"><div><h1>Dual Robot Control</h1><div class="sub">실시간 UDP 상태 · 버튼을 누르고 있는 동안만 이동</div></div><div id="fleet" class="pill bad">연결 확인 중</div></div>
<div id="robots" class="grid"></div>
<div class="card controls">
  <div class="row between"><h2>수동 조작</h2><div>현재 동작: <span id="motion" class="value">STOP</span></div></div>
  <div class="row selection"><span class="label">로봇 선택</span><button data-select="0">Robot 1</button><button data-select="1">Robot 2</button><button data-select="0,1" class="selected">Both</button></div>
  <div class="sliders">
    <div class="sliderbox"><div class="row between"><span>이동 속도</span><strong id="linearValue">0.15</strong></div><input id="linear" type="range" min="0.05" max="0.50" value="0.15" step="0.01"></div>
    <div class="sliderbox"><div class="row between"><span>회전 속도</span><strong id="rotationValue">0.15</strong></div><input id="rotation" type="range" min="0.05" max="0.50" value="0.15" step="0.01"></div>
  </div>
  <div id="orbitInfo" class="orbitinfo">공전 대기 · 시작 시 현재 초음파 간격을 목표로 자동 저장합니다.</div>
  <div class="pad">
    <button data-motion="q">↖</button><button data-motion="w">↑ 전진</button><button data-motion="e">↗</button>
    <button data-motion="a">← 횡이동</button><button data-stop>STOP</button><button data-motion="d">횡이동 →</button>
    <button data-motion="z">↙</button><button data-motion="s">↓ 후진</button><button data-motion="c">↘</button>
  </div>
  <div class="rotate"><button data-motion="j">↺ 제자리 반시계</button><button data-motion="l">↻ 제자리 시계</button></div>
  <div class="pair"><button data-motion="u">반시계: R2가 R1을 보며 공전</button><button data-motion="i">시계: R2가 R1을 보며 공전</button></div>
  <button class="stop" data-stop>두 로봇 비상정지</button>
  <div id="alert" class="alert">READY</div>
</div></div>
<script>
let selected=[0,1], repeat=null, initialized=false;
const $=id=>document.getElementById(id);
const esc=v=>String(v??'-').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function vector(v){const a=String(v||'').split(':');return a.length===4?a:['-','-','-','-']}
function robotCard(r){const f=r.fields||{},dist=parseFloat(f.distance_cm),stop=parseFloat(f.stop_cm),valid=Number.isFinite(dist);let pct=valid?Math.min(100,dist):0;let color=!valid?'#64748b':(f.obstacle==='1'||(Number.isFinite(stop)&&dist<=stop))?'#ff5c6c':dist<30?'#ffbf47':'#35d39a';const t=vector(f.target_rpm),a=vector(f.rpm),p=vector(f.drive_pwm);let rows=['FL','FR','RL','RR'].map((w,i)=>`<tr><td>${w}</td><td>${esc(t[i])}</td><td>${esc(a[i])}</td><td>${esc(p[i])}</td></tr>`).join('');let imu=f.imu_ok!==undefined?`<div class="metric"><div class="label">IMU</div><div class="value">${f.imu_ok==='1'?'OK':'ERROR'}</div></div><div class="metric"><div class="label">자세 R:P:Y</div><div class="value">${esc(f.att_deg)}</div></div><div class="metric"><div class="label">Gyro</div><div class="value">${esc(f.gyro_dps)}</div></div>`:'';return `<div class="card"><div class="row between"><h2>Robot ${r.number}</h2><span class="${r.online?'online':'offline'} value">${r.online?'ONLINE':'OFFLINE'}</span></div><div class="label">${esc(r.ip)} · ${esc(f.fw)}</div><div class="distance">${valid?dist.toFixed(1):'--'} <small style="font-size:16px;color:var(--muted)">cm</small></div><div class="bar"><div style="width:${pct}%;background:${color}"></div></div><div class="meta"><div class="metric"><div class="label">센서 역할</div><div class="value">${esc(f.sensor_role)}</div></div><div class="metric"><div class="label">정지 기준</div><div class="value">${esc(f.stop_cm)} cm</div></div><div class="metric"><div class="label">상태</div><div class="value">${esc(f.state)}</div></div><div class="metric"><div class="label">RSSI</div><div class="value">${esc(f.rssi)} dBm</div></div><div class="metric"><div class="label">안전 래치</div><div class="value" style="color:${f.obstacle==='1'?'var(--bad)':'var(--good)'}">${f.obstacle==='1'?'STOP':'CLEAR'}</div></div>${imu}</div><table><thead><tr><th>Wheel</th><th>Target</th><th>RPM</th><th>PWM</th></tr></thead><tbody>${rows}</tbody></table></div>`}
async function poll(){try{const d=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());if(!initialized){for(const id of ['linear','rotation']){const el=$(id);el.min=d.defaults.minimum;el.max=d.defaults.maximum;el.value=d.defaults[id];$(id+'Value').textContent=Number(el.value).toFixed(2)}initialized=true}$('robots').innerHTML=d.robots.map(robotCard).join('');$('motion').textContent=d.motion;const online=d.robots.filter(r=>r.online).length;$('fleet').textContent=`ROBOTS ${online}/2`;$('fleet').className='pill '+(online===2?'good':'bad');$('alert').textContent=d.safety;$('alert').className='alert '+(d.safety==='READY'?'':'bad');const o=d.orbit||{};const rpm=o.yaw_rpm_error!==undefined?` · 회전RPM R1 ${o.yaw_rpm_robot1} / R2 ${o.yaw_rpm_robot2} · 보정 ${o.yaw_correction}`:'';$('orbitInfo').textContent=o.target_gap_cm!==undefined?`${o.active?'공전 제어 중':'마지막 공전'} · 실측 간격 ${o.gap_cm}cm / 목표 ${o.target_gap_cm}cm · 추정 중심 반지름 ${o.center_radius_cm}cm · 횡이동 ${o.tangent_command} · 전후보정 ${o.radial_command} · 회전 ${o.rotation_command}${rpm}`:`공전 대기 · K=${o.rotation_radius_cm||'-'}cm · 시작 시 현재 초음파 간격을 목표로 자동 저장합니다.`}catch(e){$('fleet').textContent='GUI 연결 끊김';$('fleet').className='pill bad'}}
setInterval(poll,250);poll();
for(const id of ['linear','rotation'])$(id).addEventListener('input',e=>$(id+'Value').textContent=Number(e.target.value).toFixed(2));
document.querySelectorAll('[data-select]').forEach(b=>b.onclick=()=>{selected=b.dataset.select.split(',').map(Number);document.querySelectorAll('[data-select]').forEach(x=>x.classList.toggle('selected',x===b));stopMotion()});
async function sendMove(key){await fetch('/api/move',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key,selected,linear:Number($('linear').value),rotation:Number($('rotation').value)})})}
async function stopMotion(){if(repeat){clearInterval(repeat);repeat=null}document.querySelectorAll('.pressed').forEach(x=>x.classList.remove('pressed'));try{await fetch('/api/stop',{method:'POST'})}catch(e){}}
function startMotion(button){stopMotion().then(()=>{button.classList.add('pressed');sendMove(button.dataset.motion);repeat=setInterval(()=>sendMove(button.dataset.motion),120)})}
document.querySelectorAll('[data-motion]').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();startMotion(b)});for(const ev of ['pointerup','pointercancel','pointerleave'])b.addEventListener(ev,stopMotion)});
document.querySelectorAll('[data-stop]').forEach(b=>b.addEventListener('click',stopMotion));window.addEventListener('blur',stopMotion);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopMotion()});
</script></body></html>"""


def make_handler(core: RobotCore):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, status: int, value: Any) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:
            if self.path == "/":
                payload = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
            elif self.path == "/api/status":
                self._send_json(200, core.status())
            else:
                self._send_json(404, {"error": "Not found"})

        def do_POST(self) -> None:
            if self.path == "/api/stop":
                core.stop()
                self._send_json(200, {"ok": True})
                return
            if self.path != "/api/move":
                self._send_json(404, {"error": "Not found"})
                return
            try:
                length = min(int(self.headers.get("Content-Length", "0")), 4096)
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                selected = tuple(int(value) for value in body.get("selected", []))
                ok, message = core.move(
                    str(body.get("key", "")),
                    selected,
                    float(body.get("linear", core.default_linear)),
                    float(body.get("rotation", core.default_rotation)),
                )
                self._send_json(200 if ok else 409, {"ok": ok, "message": message})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self._send_json(400, {"ok": False, "message": str(exc)})

        def log_message(self, format: str, *args: object) -> None:
            return

    return Handler


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing config value: {key}")
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    port = int(args.port if args.port is not None else config.get("web_port", 8080))
    core = RobotCore(config)
    server = ThreadingHTTPServer((args.host, port), make_handler(core))
    server.daemon_threads = True
    try:
        core.start()
        print(f"Robot GUI ready: http://<Raspberry-Pi-IP>:{port}")
        print("Keep this terminal open. Exit with Ctrl+C.")
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nSTOP: GUI closed; both robots stopped.")
    finally:
        server.server_close()
        core.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
