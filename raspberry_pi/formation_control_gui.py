#!/usr/bin/env python3
"""Browser GUI for manual formation driving and Robot 2 orbit control."""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from uwb_directional_model import DirectionalUwbModel


STATUS_TIMEOUT_SEC = 1.2
UWB_TIMEOUT_SEC = 0.7
WEB_DEADMAN_SEC = 0.45


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def wrap_radians(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_fields(text: str, prefix: str) -> dict[str, str] | None:
    parts = text.strip().split(",")
    if not parts or parts[0] != prefix:
        return None
    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key.strip()] = value.strip()
    return fields


def vector(fields: dict[str, str], name: str, count: int) -> tuple[float, ...] | None:
    try:
        values = tuple(float(value) for value in fields[name].split(":"))
    except (KeyError, ValueError):
        return None
    return values if len(values) == count else None


@dataclass(frozen=True)
class Motion:
    vx: float
    vy: float
    w: float
    name: str

    def packet(self) -> str:
        return f"V,{self.vx:.3f},{self.vy:.3f},{self.w:.3f},{self.name}"


@dataclass
class TimedFields:
    received_at: float
    fields: dict[str, str]


@dataclass
class Pose:
    x: float
    y: float
    heading: float


class FormationSystem:
    def __init__(self, config: dict[str, Any]):
        robot_ips = [str(value) for value in config["robot_ips"]]
        if len(robot_ips) != 2:
            raise ValueError("config.json robot_ips must contain exactly two addresses")
        self.config = config
        self.robot_ips = robot_ips
        command_port = int(config["command_port"])
        self.addresses = [(ip, command_port) for ip in robot_ips]

        self.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.status_socket.bind(("0.0.0.0", int(config["status_port"])))
        self.status_socket.setblocking(False)

        self.uwb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.uwb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.uwb_socket.bind(("0.0.0.0", int(config.get("uwb_position_port", 4220))))
        self.uwb_socket.setblocking(False)

        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.worker = threading.Thread(target=self._run, name="formation-control", daemon=True)
        self.status: dict[str, TimedFields] = {}
        self.uwb: TimedFields | None = None

        self.gap = float(config.get("formation_gap_m", 0.30))
        self.start_x = float(config.get("formation_start_leader_x_m", 2.0))
        self.start_y = float(config.get("formation_start_leader_y_m", 0.0))
        self.start_heading = math.radians(float(config.get("formation_start_heading_deg", 90.0)))
        self.position_kp = float(config.get("formation_position_kp", 0.8))
        self.yaw_kp = float(config.get("formation_yaw_kp", 0.18))
        self.max_correction = float(config.get("formation_max_correction", 0.12))
        self.max_command = float(config.get("formation_max_command", 0.25))
        self.leader_max_command = float(config.get("formation_leader_max_command", 0.50))
        self.orbit_min_radius = float(config.get("formation_orbit_min_radius_m", 0.30))
        self.orbit_max_radius = float(config.get("formation_orbit_max_radius_m", 0.90))
        self.orbit_radial_kp = float(config.get("formation_orbit_radial_kp", 1.2))
        self.orbit_radial_max = float(config.get("formation_orbit_radial_max", 0.18))
        self.orbit_yaw_kp = float(config.get("formation_orbit_yaw_kp", 0.8))
        self.orbit_yaw_max = float(config.get("formation_orbit_yaw_max", 0.30))
        self.orbit_speed_max = float(config.get("formation_orbit_speed_max", 0.35))
        self.wheel_radius = float(config.get("wheel_radius_m", 0.03))
        self.kinematic_radius = float(config.get("wheelbase_half_m", 0.097)) + float(
            config.get("track_half_m", 0.15)
        )
        self.command_hz = max(10.0, float(config.get("command_hz", 10.0)))
        self.fusion_rmse_max = float(config.get("fusion_uwb_rmse_max_m", 0.10))
        self.fusion_uwb_enabled = bool(config.get("fusion_uwb_enabled", True))
        self.fusion_innovation_max = float(config.get("fusion_uwb_innovation_max_m", 0.20))
        self.fusion_moving_alpha = clamp(float(config.get("fusion_uwb_moving_alpha", 0.04)), 0.0, 0.5)
        self.fusion_stationary_alpha = clamp(float(config.get("fusion_uwb_stationary_alpha", 0.12)), 0.0, 0.5)
        self.fusion_stationary_speed = float(config.get("fusion_stationary_speed_mps", 0.015))

        self.tag_forward = float(config.get("uwb_tag_forward_m", 0.110))
        self.tag_left = float(config.get("uwb_tag_left_m", 0.025))
        self.uwb_bias_x = float(config.get("uwb_bias_x_m", 0.0))
        self.uwb_bias_y = float(config.get("uwb_bias_y_m", 0.0))
        self.directional_model: DirectionalUwbModel | None = None
        self.directional_active = False
        self.directional_correction_m = 0.0
        self.directional_range_rmse: float | None = None
        self.directional_tag: tuple[float, float] | None = None
        self.directional_alpha = clamp(float(config.get("uwb_directional_filter_alpha", 0.35)), 0.05, 1.0)
        self.directional_max_correction = float(config.get("uwb_directional_max_correction_m", 0.45))
        if bool(config.get("uwb_directional_enabled", False)):
            model_path = Path(str(config.get(
                "uwb_directional_model_file", "calibration_results/uwb_directional_model.json"
            )))
            if not model_path.is_absolute():
                model_path = Path(str(config.get("_config_dir", "."))) / model_path
            try:
                self.directional_model = DirectionalUwbModel.load(model_path)
                print(f"UWB directional model loaded: {model_path}")
            except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError) as error:
                print(f"WARNING: directional UWB model disabled: {error}")
        yaw_signs = config.get("imu_yaw_signs", [-1.0, -1.0])
        if not isinstance(yaw_signs, list) or len(yaw_signs) != 2:
            raise ValueError("config.json imu_yaw_signs must contain two values")
        self.yaw_signs = [1.0 if float(value) >= 0.0 else -1.0 for value in yaw_signs]
        self.yaw_bias = [self.start_heading, self.start_heading]
        self.follower = Pose(
            self.start_x - self.gap * math.cos(self.start_heading),
            self.start_y - self.gap * math.sin(self.start_heading),
            self.start_heading,
        )
        self.leader = Pose(self.start_x, self.start_y, self.start_heading)
        self.goal = Pose(self.follower.x, self.follower.y, self.start_heading)
        now = time.monotonic()
        self.last_odometry_time = [now, now]
        self.odometry_speed = [0.0, 0.0]
        self.last_fused_uwb_at = -1.0
        self.fusion_status = "WAIT_RESET"
        self.fusion_innovation_m: float | None = None
        self.fusion_alpha = 0.0
        self.fusion_accept_count = 0
        self.fusion_reject_count = 0
        self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")
        self.follower_motion = Motion(0.0, 0.0, 0.0, "AUTO_STOP")
        self.motion_deadline = 0.0
        self.follow_enabled = False
        self.control_mode = "OFF"
        self.orbit_direction = 1
        self.orbit_radius = max(self.orbit_min_radius, min(self.gap, self.orbit_max_radius))
        self.orbit_speed = 0.15
        self.orbit_distance = self.gap
        self.pose_reset = False
        self.position_error = 0.0
        self.yaw_error_deg = 0.0
        self.safety = "두 로봇을 시작 위치에 놓고 위치 초기화를 누르세요."

    def start(self) -> None:
        self.stop_all()
        self.worker.start()

    def send(self, index: int, message: str) -> None:
        self.status_socket.sendto(message.encode("utf-8"), self.addresses[index])

    def stop_all(self) -> None:
        for _ in range(4):
            self.send(0, "STOP")
            self.send(1, "STOP")
            time.sleep(0.02)

    def close(self) -> None:
        self.shutdown.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        try:
            self.stop_all()
        finally:
            self.status_socket.close()
            self.uwb_socket.close()

    def _receive(self) -> None:
        while True:
            try:
                payload, address = self.status_socket.recvfrom(1400)
            except BlockingIOError:
                break
            except ConnectionResetError:
                # Windows reports ICMP "port unreachable" on UDP receive when
                # a test/robot destination is offline. Treat it as no packet.
                break
            fields = parse_fields(payload.decode("utf-8", errors="replace"), "STATUS")
            if fields is not None:
                self.status[address[0]] = TimedFields(time.monotonic(), fields)
        while True:
            try:
                payload, _ = self.uwb_socket.recvfrom(1400)
            except BlockingIOError:
                break
            except ConnectionResetError:
                break
            fields = parse_fields(payload.decode("utf-8", errors="replace"), "UWB_POS")
            if fields is not None:
                self.uwb = TimedFields(time.monotonic(), fields)

    def _status_fresh(self, index: int, now: float) -> bool:
        packet = self.status.get(self.robot_ips[index])
        return packet is not None and now - packet.received_at <= STATUS_TIMEOUT_SEC

    def _uwb_fresh(self, now: float) -> bool:
        return (
            self.uwb is not None
            and now - self.uwb.received_at <= UWB_TIMEOUT_SEC
            and self.uwb.fields.get("quality") == "OK"
        )

    def _fields(self, index: int) -> dict[str, str]:
        packet = self.status.get(self.robot_ips[index])
        return {} if packet is None else packet.fields

    def _raw_yaw(self, index: int) -> float | None:
        fields = self._fields(index)
        attitudes = vector(fields, "att_deg", 3)
        if (attitudes is None or not all(math.isfinite(v) for v in attitudes)
                or fields.get("imu_ok") != "1"):
            return None
        return math.radians(attitudes[2])

    def _heading(self, index: int) -> float | None:
        raw = self._raw_yaw(index)
        return None if raw is None else wrap_radians(
            self.yaw_signs[index] * raw + self.yaw_bias[index]
        )

    def _uwb_center_measurement(self, heading: float) -> tuple[float, float] | None:
        if heading is None or self.uwb is None:
            return None
        try:
            raw_tag_x = float(self.uwb.fields["tag_x_m"])
            raw_tag_y = float(self.uwb.fields["tag_y_m"])
        except (KeyError, ValueError):
            return None
        tag_x = raw_tag_x - self.uwb_bias_x
        tag_y = raw_tag_y - self.uwb_bias_y
        self.directional_active = False
        self.directional_range_rmse = None
        if self.directional_model is not None:
            ranges = vector(self.uwb.fields, "ranges_m", 4)
            if ranges is not None:
                try:
                    corrected = self.directional_model.correct(ranges, heading, (raw_tag_x, raw_tag_y))
                    correction_m = math.hypot(corrected.tag_x - raw_tag_x, corrected.tag_y - raw_tag_y)
                    inside = -0.5 <= corrected.tag_x <= 4.5 and -0.5 <= corrected.tag_y <= 3.5
                    if inside and correction_m <= self.directional_max_correction and corrected.range_rmse <= 0.25:
                        if self.directional_tag is None:
                            self.directional_tag = (corrected.tag_x, corrected.tag_y)
                        else:
                            self.directional_tag = (
                                self.directional_tag[0] + self.directional_alpha * (corrected.tag_x - self.directional_tag[0]),
                                self.directional_tag[1] + self.directional_alpha * (corrected.tag_y - self.directional_tag[1]),
                            )
                        tag_x, tag_y = self.directional_tag
                        self.directional_active = True
                        self.directional_correction_m = correction_m
                        self.directional_range_rmse = corrected.range_rmse
                except (ArithmeticError, TypeError, ValueError):
                    self.directional_active = False
        return (
            tag_x - self.tag_forward * math.cos(heading) + self.tag_left * math.sin(heading),
            tag_y - self.tag_forward * math.sin(heading) - self.tag_left * math.cos(heading),
        )

    def _body_velocity(self, index: int, rpms: tuple[float, ...]) -> tuple[float, float]:
        """Wheel conversion hook; manual/fusion programs retain unit scaling."""
        scale = 2.0 * math.pi * self.wheel_radius / 60.0
        fl, fr, rl, rr = (rpm * scale for rpm in rpms)
        return (fl + fr + rl + rr) / 4.0, (-fl + fr + rl - rr) / 4.0

    def _update_odometry(self, index: int, pose: Pose, now: float) -> None:
        dt = clamp(now - self.last_odometry_time[index], 0.0, 0.20)
        self.last_odometry_time[index] = now
        heading = self._heading(index)
        rpms = vector(self._fields(index), "rpm", 4)
        if (heading is None or not math.isfinite(heading) or rpms is None
                or not all(math.isfinite(rpm) for rpm in rpms)
                or not self._status_fresh(index, now)):
            self.odometry_speed[index] = 0.0
            return
        vx, vy = self._body_velocity(index, rpms)
        pose.x += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        pose.y += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
        pose.heading = heading
        self.odometry_speed[index] = math.hypot(vx, vy)

    def _fuse_leader_uwb(self, now: float) -> None:
        if not self.fusion_uwb_enabled:
            self.fusion_status = "DISABLED"
            return
        if not self.pose_reset:
            self.fusion_status = "WAIT_RESET"
            return
        if self.uwb is None or self.uwb.received_at == self.last_fused_uwb_at:
            return
        self.last_fused_uwb_at = self.uwb.received_at
        if not self._uwb_fresh(now):
            self.fusion_status = "REJECT_QUALITY"
            self.fusion_reject_count += 1
            return
        try:
            rmse = float(self.uwb.fields["rmse_m"])
        except (KeyError, ValueError):
            self.fusion_status = "REJECT_RMSE"
            self.fusion_reject_count += 1
            return
        if not math.isfinite(rmse) or rmse > self.fusion_rmse_max:
            self.fusion_status = "REJECT_RMSE"
            self.fusion_reject_count += 1
            return
        measurement = self._uwb_center_measurement(self.leader.heading)
        if measurement is None:
            self.fusion_status = "REJECT_PARSE"
            self.fusion_reject_count += 1
            return
        innovation = math.hypot(measurement[0] - self.leader.x, measurement[1] - self.leader.y)
        self.fusion_innovation_m = innovation
        if innovation > self.fusion_innovation_max:
            self.fusion_status = "REJECT_JUMP"
            self.fusion_reject_count += 1
            return
        stationary = self.odometry_speed[0] <= self.fusion_stationary_speed
        alpha = self.fusion_stationary_alpha if stationary else self.fusion_moving_alpha
        self.leader.x += alpha * (measurement[0] - self.leader.x)
        self.leader.y += alpha * (measurement[1] - self.leader.y)
        self.fusion_alpha = alpha
        self.fusion_status = "ACCEPT_STOP" if stationary else "ACCEPT_MOVE"
        self.fusion_accept_count += 1

    def _calculate_follower_motion(self) -> Motion | None:
        follower_heading = self._heading(1)
        if follower_heading is None:
            return None
        self.goal = Pose(
            self.leader.x - self.gap * math.cos(self.leader.heading),
            self.leader.y - self.gap * math.sin(self.leader.heading),
            self.leader.heading,
        )
        error_x = self.goal.x - self.follower.x
        error_y = self.goal.y - self.follower.y
        error_forward = error_x * math.cos(follower_heading) + error_y * math.sin(follower_heading)
        error_left = -error_x * math.sin(follower_heading) + error_y * math.cos(follower_heading)
        yaw_error = wrap_radians(self.leader.heading - follower_heading)
        self.position_error = math.hypot(error_x, error_y)
        self.yaw_error_deg = math.degrees(yaw_error)

        world_vx = self.leader_motion.vx * math.cos(self.leader.heading) - self.leader_motion.vy * math.sin(self.leader.heading)
        world_vy = self.leader_motion.vx * math.sin(self.leader.heading) + self.leader_motion.vy * math.cos(self.leader.heading)
        ff_forward = world_vx * math.cos(follower_heading) + world_vy * math.sin(follower_heading)
        ff_left = -world_vx * math.sin(follower_heading) + world_vy * math.cos(follower_heading)
        return Motion(
            clamp(ff_forward + clamp(self.position_kp * error_forward, -self.max_correction, self.max_correction), -self.max_command, self.max_command),
            clamp(ff_left + clamp(self.position_kp * error_left, -self.max_correction, self.max_correction), -self.max_command, self.max_command),
            clamp(self.leader_motion.w + clamp(self.yaw_kp * yaw_error, -self.max_correction, self.max_correction), -self.max_command, self.max_command),
            "AUTO_FOLLOW",
        )

    def _calculate_orbit_motion(self) -> Motion | None:
        """Orbit Robot 2 while Robot 1 rotates so its rear faces Robot 2."""
        follower_heading = self._heading(1)
        leader_heading = self._heading(0)
        if follower_heading is None or leader_heading is None:
            return None

        dx = self.follower.x - self.leader.x
        dy = self.follower.y - self.leader.y
        distance = math.hypot(dx, dy)
        if distance < 0.05:
            return None

        outward_x = dx / distance
        outward_y = dy / distance
        inward_heading = math.atan2(-outward_y, -outward_x)
        yaw_error = wrap_radians(inward_heading - follower_heading)
        leader_yaw_error = wrap_radians(inward_heading - leader_heading)
        distance_error = distance - self.orbit_radius
        radial_speed = clamp(
            self.orbit_radial_kp * distance_error,
            -self.orbit_radial_max,
            self.orbit_radial_max,
        )

        # Positive direction is CCW in the world coordinate frame.
        tangent_x = -outward_y * self.orbit_direction * self.orbit_speed
        tangent_y = outward_x * self.orbit_direction * self.orbit_speed
        world_vx = -outward_x * radial_speed + tangent_x
        world_vy = -outward_y * radial_speed + tangent_y
        forward = world_vx * math.cos(follower_heading) + world_vy * math.sin(follower_heading)
        left = -world_vx * math.sin(follower_heading) + world_vy * math.cos(follower_heading)

        self.goal = Pose(
            self.leader.x + outward_x * self.orbit_radius,
            self.leader.y + outward_y * self.orbit_radius,
            inward_heading,
        )
        self.orbit_distance = distance
        self.position_error = abs(distance_error)
        self.yaw_error_deg = math.degrees(yaw_error)
        orbit_rotation_ff = (
            self.orbit_direction
            * self.orbit_speed
            * self.kinematic_radius
            / max(self.orbit_radius, 0.05)
        )
        self.leader_motion = Motion(
            0.0,
            0.0,
            clamp(
                orbit_rotation_ff + self.orbit_yaw_kp * leader_yaw_error,
                -self.orbit_yaw_max,
                self.orbit_yaw_max,
            ),
            "ORBIT_CENTER_CCW" if self.orbit_direction > 0 else "ORBIT_CENTER_CW",
        )
        return Motion(
            clamp(forward, -self.max_command, self.max_command),
            clamp(left, -self.max_command, self.max_command),
            clamp(
                orbit_rotation_ff + self.orbit_yaw_kp * yaw_error,
                -self.orbit_yaw_max,
                self.orbit_yaw_max,
            ),
            "AUTO_ORBIT_CCW" if self.orbit_direction > 0 else "AUTO_ORBIT_CW",
        )

    def reset_pose(self) -> tuple[bool, str]:
        with self.lock:
            now = time.monotonic()
            yaw1 = self._raw_yaw(0)
            yaw2 = self._raw_yaw(1)
            if not self._status_fresh(0, now) or not self._status_fresh(1, now):
                return False, "로봇 상태가 오프라인입니다."
            if yaw1 is None or yaw2 is None:
                return False, "두 로봇의 IMU 값을 확인하세요."
            self.follow_enabled = False
            self.control_mode = "OFF"
            self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")
            self.yaw_bias[0] = wrap_radians(
                self.start_heading - self.yaw_signs[0] * yaw1
            )
            self.yaw_bias[1] = wrap_radians(
                self.start_heading - self.yaw_signs[1] * yaw2
            )
            self.leader = Pose(self.start_x, self.start_y, self.start_heading)
            self.follower = Pose(
                self.start_x - self.gap * math.cos(self.start_heading),
                self.start_y - self.gap * math.sin(self.start_heading),
                self.start_heading,
            )
            self.last_odometry_time = [now, now]
            self.odometry_speed = [0.0, 0.0]
            self.last_fused_uwb_at = -1.0
            self.fusion_status = "READY_ODOMETRY"
            self.fusion_innovation_m = None
            self.fusion_alpha = 0.0
            self.fusion_accept_count = 0
            self.fusion_reject_count = 0
            self.goal = Pose(self.follower.x, self.follower.y, self.start_heading)
            self.pose_reset = True
            self.safety = "위치 초기화 완료: Encoder+IMU 주행, 신뢰 가능한 UWB만 보정합니다."
            self.stop_all()
            return True, self.safety

    def set_follow(self, enabled: bool) -> tuple[bool, str]:
        with self.lock:
            now = time.monotonic()
            if not enabled:
                self.follow_enabled = False
                self.control_mode = "OFF"
                self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")
                self.safety = "자동 추종 꺼짐"
                self.stop_all()
                return True, self.safety
            if not self.pose_reset:
                return False, "먼저 위치 초기화를 누르세요."
            if not (self._status_fresh(0, now) and self._status_fresh(1, now)):
                return False, "로봇 상태가 오프라인입니다."
            if self._heading(0) is None or self._heading(1) is None:
                return False, "두 로봇의 IMU 상태를 확인하세요."
            self.follow_enabled = True
            self.control_mode = "FOLLOW"
            self.safety = "자동 추종 켜짐"
            return True, self.safety

    def set_orbit(
        self, enabled: bool, direction: str = "CCW", radius: float = 0.30, speed: float = 0.15
    ) -> tuple[bool, str]:
        with self.lock:
            if not enabled:
                self.follow_enabled = False
                self.control_mode = "OFF"
                self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")
                self.follower_motion = Motion(0.0, 0.0, 0.0, "AUTO_STOP")
                self.safety = "원궤도 주행 꺼짐"
                self.stop_all()
                return True, self.safety

            now = time.monotonic()
            if not self.pose_reset:
                return False, "먼저 위치 초기화를 누르세요."
            if not (self._status_fresh(0, now) and self._status_fresh(1, now)):
                return False, "로봇 상태가 오프라인입니다."
            if self._heading(0) is None or self._heading(1) is None:
                return False, "두 로봇의 IMU 상태를 확인하세요."

            direction = direction.upper()
            if direction not in ("CCW", "CW"):
                return False, "원궤도 방향은 CCW 또는 CW여야 합니다."
            self.orbit_direction = 1 if direction == "CCW" else -1
            self.orbit_radius = clamp(radius, self.orbit_min_radius, self.orbit_max_radius)
            self.orbit_speed = clamp(speed, 0.05, self.orbit_speed_max)
            self.leader_motion = Motion(0.0, 0.0, 0.0, "ORBIT_CENTER_STOP")
            self.motion_deadline = 0.0
            self.follow_enabled = True
            self.control_mode = f"ORBIT_{direction}"
            korean_direction = "반시계" if direction == "CCW" else "시계"
            self.safety = (
                f"{korean_direction} 원궤도 주행: 반지름 {self.orbit_radius:.2f}m, "
                f"속도 {self.orbit_speed:.2f} · Robot 1 제자리 동기 회전"
            )
            return True, self.safety

    def set_motion(self, vx: float, vy: float, w: float, name: str) -> tuple[bool, str]:
        with self.lock:
            if not self.follow_enabled:
                return False, "자동 추종을 먼저 켜세요."
            # Manual input always returns from orbit mode to normal following.
            self.control_mode = "FOLLOW"
            limit = min(self.leader_max_command, 1.0)
            self.leader_motion = Motion(
                clamp(vx, -limit, limit),
                clamp(vy, -limit, limit),
                clamp(w, -limit, limit),
                name,
            )
            self.motion_deadline = time.monotonic() + WEB_DEADMAN_SEC
            return True, ""

    def stop_leader(self) -> None:
        with self.lock:
            self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")

    def emergency_stop(self) -> None:
        with self.lock:
            self.follow_enabled = False
            self.control_mode = "OFF"
            self.pose_reset = False
            self.leader_motion = Motion(0.0, 0.0, 0.0, "EMERGENCY_STOP")
            self.follower_motion = Motion(0.0, 0.0, 0.0, "EMERGENCY_STOP")
            self.safety = "비상 정지: 다시 위치 초기화가 필요합니다."
            self.stop_all()

    def state(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            h1 = self._heading(0)
            h2 = self._heading(1)
            uwb_age = None if self.uwb is None else now - self.uwb.received_at
            return {
                "follow_enabled": self.follow_enabled,
                "control_mode": self.control_mode,
                "pose_reset": self.pose_reset,
                "safety": self.safety,
                "leader_motion": self.leader_motion.__dict__,
                "follower_motion": self.follower_motion.__dict__,
                "leader": {
                    "x": self.leader.x,
                    "y": self.leader.y,
                    "heading_deg": None if h1 is None else math.degrees(h1),
                    "online": self._status_fresh(0, now),
                    "imu_ok": self._fields(0).get("imu_ok") == "1",
                },
                "follower": {
                    "x": self.follower.x,
                    "y": self.follower.y,
                    "heading_deg": None if h2 is None else math.degrees(h2),
                    "online": self._status_fresh(1, now),
                    "imu_ok": self._fields(1).get("imu_ok") == "1",
                    "ultrasonic_cm": self._fields(1).get("distance_cm", "NA"),
                },
                "goal": {"x": self.goal.x, "y": self.goal.y},
                "error_m": self.position_error,
                "yaw_error_deg": self.yaw_error_deg,
                "uwb": {
                    "online": self._uwb_fresh(now),
                    "age_sec": uwb_age,
                    "quality": None if self.uwb is None else self.uwb.fields.get("quality"),
                    "rmse_m": None if self.uwb is None else self.uwb.fields.get("rmse_m"),
                    "directional_model_loaded": self.directional_model is not None,
                    "directional_active": self.directional_active,
                    "directional_correction_m": self.directional_correction_m,
                    "directional_range_rmse_m": self.directional_range_rmse,
                },
                "fusion": {
                    "status": self.fusion_status,
                    "innovation_m": self.fusion_innovation_m,
                    "alpha": self.fusion_alpha,
                    "accepted": self.fusion_accept_count,
                    "rejected": self.fusion_reject_count,
                    "odometry_speed_mps": self.odometry_speed[0],
                },
                "orbit": {
                    "active": self.control_mode.startswith("ORBIT_"),
                    "direction": "CCW" if self.orbit_direction > 0 else "CW",
                    "radius_m": self.orbit_radius,
                    "speed": self.orbit_speed,
                    "distance_m": self.orbit_distance,
                },
                "start": {"leader_x": self.start_x, "leader_y": self.start_y, "gap": self.gap},
            }

    def _run(self) -> None:
        next_send = 0.0
        next_ping = 0.0
        while not self.shutdown.is_set():
            now = time.monotonic()
            with self.lock:
                self._receive()
                self._update_odometry(0, self.leader, now)
                self._update_odometry(1, self.follower, now)
                self._fuse_leader_uwb(now)
                if now > self.motion_deadline:
                    self.leader_motion = Motion(0.0, 0.0, 0.0, "GUI_STOP")

                obstacle = self._fields(0).get("obstacle") == "1" or self._fields(1).get("obstacle") == "1"
                sensors_ready = (
                    self._status_fresh(0, now)
                    and self._status_fresh(1, now)
                    and self._heading(0) is not None
                    and self._heading(1) is not None
                )
                if self.follow_enabled and (not sensors_ready or obstacle):
                    self.follow_enabled = False
                    self.control_mode = "OFF"
                    self.pose_reset = False
                    self.safety = "자동 정지: 센서 연결 또는 장애물을 확인하세요."
                    self.stop_all()

                if now >= next_send:
                    if self.follow_enabled:
                        if self.control_mode.startswith("ORBIT_"):
                            follower = self._calculate_orbit_motion()
                        else:
                            follower = self._calculate_follower_motion()
                        if follower is None:
                            self.follow_enabled = False
                            self.control_mode = "OFF"
                            self.pose_reset = False
                            self.safety = "자동 정지: 위치 계산 실패"
                            self.stop_all()
                        else:
                            self.follower_motion = follower
                            self.send(0, self.leader_motion.packet())
                            self.send(1, follower.packet())
                    else:
                        self.send(0, "STOP")
                        self.send(1, "STOP")
                    next_send = now + 1.0 / self.command_hz

                if now >= next_ping:
                    self.send(0, "PING")
                    self.send(1, "PING")
                    next_ping = now + 0.8
            time.sleep(0.01)


HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>센서 융합 군집주행 제어</title><style>
:root{--bg:#07111f;--panel:#101d30;--line:#28415e;--text:#eaf2ff;--muted:#8da4bf;--blue:#40a9ff;--orange:#ff9f43;--green:#35d07f;--red:#ff5d68}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,"Noto Sans KR",sans-serif}
.wrap{max-width:1400px;margin:auto;padding:18px}.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:14px}.title{font-size:24px;font-weight:750}.sub{color:var(--muted);font-size:13px}
.grid{display:grid;grid-template-columns:minmax(600px,1.55fr) minmax(360px,.85fr);gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px;box-shadow:0 10px 35px #0005}
canvas{width:100%;height:auto;background:#091626;border-radius:10px;display:block}.badges{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:7px 11px;border-radius:20px;background:#172a42;color:var(--muted);font-size:13px}.ok{color:#07150d;background:var(--green)}.bad{color:white;background:var(--red)}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin:12px 0}.stat{background:#0b1727;border:1px solid #213852;border-radius:10px;padding:10px}.stat b{display:block;font-size:19px;margin-top:4px}.label{color:var(--muted);font-size:12px}
.buttons{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:10px 0}.btn{border:1px solid #335579;background:#172a42;color:white;border-radius:10px;padding:12px;cursor:pointer;font-weight:650}.btn:hover{background:#203a59}.primary{background:#1768a7}.orbit{background:#4f3a8a;border-color:#7657c7}.danger{background:#9e2931;border-color:#d34a54}.wide{grid-column:span 3}
.controls{display:grid;grid-template-columns:repeat(3,70px);justify-content:center;gap:7px;margin:14px}.key{height:54px;font-size:18px}.empty{visibility:hidden}input[type=range]{width:100%}.notice{min-height:48px;padding:11px;background:#0b1727;border-radius:9px;color:#bdd0e5;font-size:14px;line-height:1.5}
.legend{display:flex;gap:18px;margin-top:10px;color:var(--muted);font-size:13px}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
@media(max-width:980px){.grid{grid-template-columns:1fr}.wrap{padding:10px}}
</style></head><body><div class="wrap">
<div class="top"><div><div class="title">센서 융합 군집주행 제어</div><div class="sub">Robot 1 수동 조작 · Robot 2 후방 추종 / 두 로봇 동기 회전 원궤도</div></div><div class="badges"><span id="r1b" class="badge">R1</span><span id="r2b" class="badge">R2</span><span id="uwbb" class="badge">UWB</span><span id="fb" class="badge">MODE OFF</span></div></div>
<div class="grid"><section class="card"><canvas id="map" width="900" height="680"></canvas><div class="legend"><span><i class="dot" style="background:#40a9ff"></i>Robot 1 (Encoder+IMU, UWB 조건부 보정)</span><span><i class="dot" style="background:#ff9f43"></i>Robot 2 (Encoder+IMU)</span><span><i class="dot" style="background:#35d07f"></i>추종 목표</span></div></section>
<aside class="card"><div class="buttons"><button class="btn primary" onclick="post('/api/reset')">1. 위치 초기화</button><button class="btn" onclick="follow(true)">2. 추종 켜기</button><button class="btn" onclick="follow(false)">추종 끄기</button></div>
<div class="buttons"><button class="btn orbit" onclick="startOrbit('CCW')">↺ 반시계 공전</button><button class="btn orbit" onclick="startOrbit('CW')">↻ 시계 공전</button><button class="btn" onclick="stopOrbit()">공전 정지</button></div>
<div class="notice" id="notice">데이터 대기 중...</div>
<div class="stats"><div class="stat"><span class="label">R1 융합 좌표</span><b id="p1">-</b></div><div class="stat"><span class="label">R2 추정 좌표</span><b id="p2">-</b></div><div class="stat"><span class="label">위치/반지름 오차</span><b id="err">-</b></div><div class="stat"><span class="label">각도 오차</span><b id="yawerr">-</b></div><div class="stat"><span class="label">R1-R2 거리</span><b id="odist">-</b></div><div class="stat"><span class="label">UWB RMSE</span><b id="rmse">-</b></div><div class="stat"><span class="label">UWB 융합 상태</span><b id="fusion">-</b></div><div class="stat"><span class="label">UWB-오도메트리 차이</span><b id="innovation">-</b></div><div class="stat"><span class="label">초음파 거리</span><b id="us">-</b></div></div>
<label class="label">수동 이동 속도 <span id="sv">0.20</span></label><input id="speed" type="range" min="0.05" max="0.50" value="0.20" step="0.01" oninput="sv.textContent=value">
<label class="label">수동 회전 속도 <span id="rv">0.15</span></label><input id="rot" type="range" min="0.05" max="0.40" value="0.15" step="0.01" oninput="rv.textContent=value">
<label class="label">공전 반지름 <span id="orv">0.30</span> m</label><input id="orbitRadius" type="range" min="0.30" max="0.90" value="0.30" step="0.05" oninput="orv.textContent=Number(value).toFixed(2)">
<label class="label">공전 속도 <span id="osv">0.15</span></label><input id="orbitSpeed" type="range" min="0.05" max="0.35" value="0.15" step="0.01" oninput="osv.textContent=value">
<div class="controls"><button class="btn key" data-key="q">Q ↖</button><button class="btn key" data-key="w">W ↑</button><button class="btn key" data-key="e">E ↗</button><button class="btn key" data-key="a">A ←</button><button class="btn key danger" onclick="emergency()">STOP</button><button class="btn key" data-key="d">D →</button><button class="btn key" data-key="j">J ↺</button><button class="btn key" data-key="s">S ↓</button><button class="btn key" data-key="l">L ↻</button></div>
<button class="btn danger wide" onclick="emergency()">비상 정지 · 다시 초기화 필요</button><p class="sub">수동 조작키를 누르면 원궤도 모드는 자동으로 후방 추종 모드로 전환됩니다. 공전 중에는 Robot 1이 제자리 회전하여 후면이 Robot 2를 향하고, Robot 2는 Robot 1을 바라봅니다.</p></aside></div></div>
<script>
const $=id=>document.getElementById(id),canvas=$('map'),ctx=canvas.getContext('2d');let state=null,tr1=[],tr2=[],active=null,timer=null;
async function post(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok)throw Error(d.message||'요청 실패');return d}
async function follow(v){try{await post('/api/follow',{enabled:v})}catch(e){alert(e.message)}}async function startOrbit(direction){clearActive();try{await post('/api/orbit',{enabled:true,direction,radius:+$('orbitRadius').value,speed:+$('orbitSpeed').value})}catch(e){alert(e.message)}}async function stopOrbit(){try{await post('/api/orbit',{enabled:false})}catch(e){alert(e.message)}}async function emergency(){clearActive();await post('/api/emergency')}
const motions={w:[1,0,0,'FORWARD'],s:[-1,0,0,'BACKWARD'],a:[0,1,0,'LEFT'],d:[0,-1,0,'RIGHT'],q:[.5,.5,0,'DIAG_LEFT'],e:[.5,-.5,0,'DIAG_RIGHT'],j:[0,0,1,'CCW'],l:[0,0,-1,'CW']};
function sendKey(k){let m=motions[k];if(!m)return;let sp=+$('speed').value,ro=+$('rot').value;post('/api/motion',{vx:m[0]*sp,vy:m[1]*sp,w:m[2]*ro,name:'GUI_'+m[3]}).catch(e=>{$('notice').textContent=e.message})}
function startKey(k){if(active===k)return;clearActive();active=k;sendKey(k);timer=setInterval(()=>sendKey(k),120)}function clearActive(){if(timer)clearInterval(timer);timer=null;active=null;post('/api/leader_stop').catch(()=>{})}
document.querySelectorAll('[data-key]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();startKey(b.dataset.key)};b.onpointerup=clearActive;b.onpointerleave=clearActive});
addEventListener('keydown',e=>{let k=e.key.toLowerCase();if(motions[k]&&!e.repeat){e.preventDefault();startKey(k)}if(e.code==='Space'){e.preventDefault();emergency()}});addEventListener('keyup',e=>{if(e.key.toLowerCase()===active)clearActive()});
function badge(id,ok,label){let e=$(id);e.textContent=label;e.className='badge '+(ok?'ok':'bad')}
function mapxy(x,y){let m=60,xmin=-.25,xmax=4.25,ymin=-.60,ymax=3.25;return[m+(x-xmin)/(xmax-xmin)*(canvas.width-2*m),canvas.height-m-(y-ymin)/(ymax-ymin)*(canvas.height-2*m)]}
function robot(p,color,label){if(!p||p.heading_deg==null)return;let [x,y]=mapxy(p.x,p.y),a=-p.heading_deg*Math.PI/180;ctx.save();ctx.translate(x,y);ctx.rotate(a);ctx.fillStyle=color;ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.beginPath();ctx.roundRect(-17,-12,34,24,5);ctx.fill();ctx.stroke();ctx.beginPath();ctx.moveTo(13,0);ctx.lineTo(27,0);ctx.stroke();ctx.restore();ctx.fillStyle='#fff';ctx.font='bold 14px sans-serif';ctx.fillText(label,x+20,y-15)}
function trail(arr,color){if(arr.length<2)return;ctx.strokeStyle=color;ctx.globalAlpha=.5;ctx.lineWidth=2;ctx.beginPath();arr.forEach((p,i)=>{let q=mapxy(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.stroke();ctx.globalAlpha=1}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#091626';ctx.fillRect(0,0,canvas.width,canvas.height);let p0=mapxy(0,0),p1=mapxy(4,3);ctx.strokeStyle='#35516e';ctx.lineWidth=2;ctx.strokeRect(p0[0],p1[1],p1[0]-p0[0],p0[1]-p1[1]);ctx.strokeStyle='#18304a';ctx.lineWidth=1;for(let x=0;x<=4;x++){let a=mapxy(x,0),b=mapxy(x,3);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}for(let y=0;y<=3;y++){let a=mapxy(0,y),b=mapxy(4,y);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}ctx.fillStyle='#8da4bf';ctx.font='13px sans-serif';[['A1',0,0],['A2',4,0],['A3',4,3],['A4',0,3]].forEach(a=>{let p=mapxy(a[1],a[2]);ctx.beginPath();ctx.arc(...p,6,0,7);ctx.fill();ctx.fillText(a[0],p[0]+8,p[1]-8)});if(!state)return;trail(tr1,'#40a9ff');trail(tr2,'#ff9f43');if(state.orbit.active){let c=mapxy(state.leader.x,state.leader.y),ex=mapxy(state.leader.x+state.orbit.radius_m,state.leader.y),ey=mapxy(state.leader.x,state.leader.y+state.orbit.radius_m);ctx.strokeStyle='#9d7cff';ctx.setLineDash([8,7]);ctx.lineWidth=2;ctx.beginPath();ctx.ellipse(c[0],c[1],Math.abs(ex[0]-c[0]),Math.abs(ey[1]-c[1]),0,0,Math.PI*2);ctx.stroke();ctx.setLineDash([])}let g=mapxy(state.goal.x,state.goal.y);ctx.strokeStyle='#35d07f';ctx.lineWidth=3;ctx.beginPath();ctx.arc(...g,10,0,7);ctx.stroke();ctx.beginPath();ctx.moveTo(g[0]-14,g[1]);ctx.lineTo(g[0]+14,g[1]);ctx.moveTo(g[0],g[1]-14);ctx.lineTo(g[0],g[1]+14);ctx.stroke();robot(state.leader,'#40a9ff','R1');robot(state.follower,'#ff9f43','R2')}
async function poll(){try{state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());badge('r1b',state.leader.online&&state.leader.imu_ok,'R1 '+(state.leader.online?'ONLINE':'OFFLINE'));badge('r2b',state.follower.online&&state.follower.imu_ok,'R2 '+(state.follower.online?'ONLINE':'OFFLINE'));badge('uwbb',state.uwb.online,'UWB '+(state.uwb.online?'OK':'OPTIONAL'));badge('fb',state.follow_enabled,'MODE '+state.control_mode);$('notice').textContent=state.safety;$('p1').textContent=`${state.leader.x.toFixed(2)}, ${state.leader.y.toFixed(2)}`;$('p2').textContent=`${state.follower.x.toFixed(2)}, ${state.follower.y.toFixed(2)}`;$('err').textContent=state.error_m.toFixed(3)+' m';$('yawerr').textContent=state.yaw_error_deg.toFixed(1)+'°';$('odist').textContent=state.orbit.distance_m.toFixed(3)+' m';$('rmse').textContent=(state.uwb.rmse_m??'-')+' m';$('fusion').textContent=state.fusion.status;$('innovation').textContent=state.fusion.innovation_m==null?'-':Number(state.fusion.innovation_m).toFixed(3)+' m';$('us').textContent=state.follower.ultrasonic_cm+' cm';let a=tr1.at(-1),b=tr2.at(-1);if(!a||Math.hypot(a[0]-state.leader.x,a[1]-state.leader.y)>.01){tr1.push([state.leader.x,state.leader.y]);if(tr1.length>300)tr1.shift()}if(!b||Math.hypot(b[0]-state.follower.x,b[1]-state.follower.y)>.01){tr2.push([state.follower.x,state.follower.y]);if(tr2.length>300)tr2.shift()}draw()}catch(e){$('notice').textContent='GUI 연결 오류: '+e.message}}
setInterval(poll,100);poll();addEventListener('beforeunload',()=>navigator.sendBeacon('/api/emergency','{}'));
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    system: FormationSystem

    def log_message(self, format: str, *args: object) -> None:
        return

    def _json_body(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            return json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return {}

    def _reply_json(self, document: dict[str, Any], status: int = 200) -> None:
        payload = json.dumps(document, ensure_ascii=False).encode("utf-8")
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
            self.end_headers()
            self.wfile.write(payload)
        elif self.path == "/api/state":
            self._reply_json(self.system.state())
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        body = self._json_body()
        try:
            if self.path == "/api/reset":
                ok, message = self.system.reset_pose()
            elif self.path == "/api/follow":
                ok, message = self.system.set_follow(bool(body.get("enabled")))
            elif self.path == "/api/orbit":
                ok, message = self.system.set_orbit(
                    bool(body.get("enabled")),
                    str(body.get("direction", "CCW")),
                    float(body.get("radius", 0.30)),
                    float(body.get("speed", 0.15)),
                )
            elif self.path == "/api/motion":
                ok, message = self.system.set_motion(
                    float(body.get("vx", 0.0)),
                    float(body.get("vy", 0.0)),
                    float(body.get("w", 0.0)),
                    str(body.get("name", "GUI_MOVE")),
                )
            elif self.path == "/api/leader_stop":
                self.system.stop_leader()
                ok, message = True, ""
            elif self.path == "/api/emergency":
                self.system.emergency_stop()
                ok, message = True, "비상 정지"
            else:
                self.send_error(404)
                return
            self._reply_json({"ok": ok, "message": message}, 200 if ok else 409)
        except (TypeError, ValueError) as error:
            self._reply_json({"ok": False, "message": str(error)}, 400)


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing config value: {key}")
    config["_config_dir"] = str(path.resolve().parent)
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    args = parser.parse_args()
    config = load_config(args.config)
    system = FormationSystem(config)
    Handler.system = system
    server = ThreadingHTTPServer(("0.0.0.0", int(config.get("formation_web_port", 8082))), Handler)
    system.start()
    print(f"Formation GUI: http://0.0.0.0:{server.server_port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
        system.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nFormation GUI stopped.")
        raise SystemExit(0)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
