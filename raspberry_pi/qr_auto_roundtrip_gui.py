#!/usr/bin/env python3
"""QR route: (0,0) -> (0,3), CW 90-degree formation turn, -> (4,3)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

try:
    import cv2
except ImportError:
    cv2 = None

from formation_control_gui import FormationSystem, Motion, clamp, wrap_radians, vector
from robot_control_gui import RobotCore, wrap_degrees
from orbit_distance_control import OrbitDistanceControl, OrbitDistanceFault
from route_precision import alignment_rotation, headings_settled, rear_relative_pose, leg_errors


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    heading: float
    curvature: float
    label: str


def line_points(
    start: tuple[float, float],
    end: tuple[float, float],
    heading: float,
    label: str,
    spacing: float = 0.04,
) -> list[PathPoint]:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, math.ceil(distance / spacing))
    return [
        PathPoint(
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
            heading,
            0.0,
            label,
        )
        for index in range(count + 1)
    ]


OUTBOUND_Y_PATH = line_points((0.0, 0.0), (0.0, 3.0), math.pi / 2, "OUT_STRAIGHT_Y")
OUTBOUND_X_PATH = line_points((0.0, 3.0), (4.0, 3.0), 0.0, "OUT_STRAIGHT_X")
RETURN_X_PATH = line_points((4.0, 3.0), (0.0, 3.0), math.pi, "BACK_STRAIGHT_X")
RETURN_Y_PATH = line_points((0.0, 3.0), (0.0, 0.0), -math.pi / 2, "BACK_STRAIGHT_Y")


def build_paths() -> tuple[list[PathPoint], list[PathPoint]]:
    # For display/export only. The worker executes each leg separately, with a
    # stationary-leader turn between legs; never follow this concatenation.
    return OUTBOUND_Y_PATH + OUTBOUND_X_PATH, RETURN_X_PATH + RETURN_Y_PATH


OUTBOUND_PATH, RETURN_PATH = build_paths()


class RouteFormationSystem(FormationSystem):
    """One UDP transport; reuse the experiment's calibrated orbit calculator."""
    route_deadline = 0.0

    def __init__(self, config):
        self.forward_odometry_scales = tuple(float(v) for v in config.get(
            'auto_route_forward_odometry_scales', [1.0, 1.0]))
        if (len(self.forward_odometry_scales) != 2 or not all(
                math.isfinite(v) and .5 <= v <= 1.5 for v in self.forward_odometry_scales)):
            raise ValueError('auto_route_forward_odometry_scales: two finite values in [0.5,1.5] required')
        super().__init__(config)
        self.orbit_controller = RobotCore(config, compute_only=True)
        # Separate state, not a change to the experimentally calibrated orbit.
        self.follow_distance = OrbitDistanceControl(dict(config,
            orbit_distance_kp=.006, orbit_radial_max_speed=.04,
            orbit_radial_slew_per_sec=.10))
        self.follow_target_gap_cm = None
        self.follow_telemetry = {}
        self.orbit_phase_scale = 1.0
        self.alignment_target = None
        self.straight_heading_target = None
        self.last_follow_time = 0.0
        self.last_follow_vx = 0.0

    def _body_velocity(self, index, rpms):
        vx, vy = super()._body_velocity(index, rpms)
        # Measurement covers forward travel only, not strafe/reverse or yaw.
        return (vx * self.forward_odometry_scales[index] if vx > 0 else vx), vy

    def set_follow(self, enabled):
        with self.lock:
            ok, message = super().set_follow(enabled)
            if ok and enabled:
                now = time.monotonic()
                # Keep the departure gap through every leg; never adopt a bad
                # post-turn gap as the new desired spacing.
                target = self.follow_target_gap_cm
                if target is None:
                    self.follow_enabled = False
                    self.control_mode = 'OFF'
                    self.stop_all()
                    return False, '직진 목표 초음파 간격이 초기화되지 않았습니다.'
                self.follow_distance.reset(target, now)
                self.last_follow_time, self.last_follow_vx = now, 0.0
                self.follow_telemetry = {}
            return ok, message

    def _follow_range_correction(self, now):
        packet = self.status.get(self.robot_ips[1])
        return self.follow_distance.update(self._fields(1).get('distance_cm'),
            packet.received_at if packet else None, now)

    _alignment_rotation = staticmethod(alignment_rotation)

    def _yaw_rate(self, index):
        gyro = vector(self._fields(index), 'gyro_dps', 3)
        if gyro is None or not all(math.isfinite(v) for v in gyro):
            return None
        return math.radians(gyro[2]) * self.yaw_signs[index]

    def _relative_display(self):
        now = time.monotonic()
        if not self.pose_reset or any(not self._status_fresh(i, now) for i in (0, 1)):
            return None, '초기화/센서 연결 필요'
        headings = [self._heading(i) for i in (0, 1)]
        if any(h is None or not math.isfinite(h) for h in headings):
            return None, 'IMU 확인 필요'
        if abs(wrap_radians(headings[0]-headings[1])) > math.radians(15):
            return None, '방향 차이가 커 후면 조준 가정 불확실'
        packet = self.status.get(self.robot_ips[1])
        if packet is None or now-packet.received_at > .75:
            return None, '초음파 상태 오래됨'
        try:
            gap = float(packet.fields['distance_cm'])
        except (ValueError, TypeError, KeyError):
            return None, '초음파 측정 없음'
        guard = (self.orbit_controller.orbit_distance if self.control_mode.startswith('ORBIT_')
                 else self.follow_distance)
        target = self.follow_target_gap_cm
        if (not math.isfinite(gap) or not guard.minimum < gap <= guard.maximum
                or target is None or abs(gap-target) > guard.max_error):
            return None, '초음파 간격 범위 이탈'
        if guard.fault_reason or guard.state == 'CHECK_JUMP':
            return None, '초음파 이상값 확인 중'
        # Only use a confirmed filter when it has sampled this leg. No synthetic
        # "measured coordinate" is written back to either odometry pose.
        if guard.last_sample is not None:
            if abs(gap-guard.filtered) > guard.jump:
                return None, '초음파 급변 확인 중'
            gap = guard.filtered
        p = rear_relative_pose(self.leader.x, self.leader.y, *headings, gap,
            float(self.config.get('orbit_center_offset_cm', 19.4)))
        p['gap_cm'] = gap
        return p, 'R2가 R1 후면 중심을 본다는 가정 · 절대좌표 아님'

    def _sync_orbit_status(self):
        self.orbit_controller.snapshots = {
            ip: (packet.received_at, dict(packet.fields))
            for ip, packet in self.status.items()
        }

    def set_orbit(self, enabled, direction="CCW", radius=.30, speed=.15):
        with self.lock:
            if not enabled:
                self.alignment_target = None
                self.straight_heading_target = None
                self.orbit_controller.orbit_direction = 0
                return super().set_orbit(False)
            now = time.monotonic()
            self._sync_orbit_status()
            core = self.orbit_controller
            gap = core._robot2_gap(now)
            if core.orbit_distance.continue_on_target_loss and self.follow_target_gap_cm is not None:
                # Cargo-only: retain the departure spacing as the orbit target,
                # even if R2 temporarily has no echo or sees a distant wall.
                gap = self.follow_target_gap_cm
            if gap is None:
                return False, "공전 시작 실패: R2 초음파 유효 거리 없음"
            ok, message = super().set_orbit(True, direction, radius, speed)
            if not ok:
                return ok, message
            core.orbit_direction = self.orbit_direction
            core.orbit_target_gap_cm = gap
            core.orbit_distance.reset(gap, now)
            core.orbit_yaw_error_filtered = 0.0
            core.orbit_encoder_heading_error_rad = 0.0
            core.orbit_heading_updated_at = now
            yaw1, yaw2 = core._imu_yaw_deg(0, now), core._imu_yaw_deg(1, now)
            core.orbit_yaw_delta_target_deg = None if yaw1 is None or yaw2 is None else wrap_degrees(yaw2 - yaw1)
            core.orbit_linear_limit = self.orbit_speed
            core.orbit_rotation_request = clamp(float(self.config.get("auto_route_orbit_rotation", .10)), .05, .12)
            core.orbit_telemetry = {}
            self.orbit_phase_scale = 1.0
            return True, message

    def _calculate_follower_motion(self):
        now = time.monotonic()
        if now > self.route_deadline:
            return None
        headings = [self._heading(i) for i in (0, 1)]
        if any(h is None or not math.isfinite(h) for h in headings):
            return None
        try:
            correction = self._follow_range_correction(now)
        except OrbitDistanceFault as error:
            self.follow_telemetry = dict(self.follow_distance.telemetry(), fault=str(error))
            return None
        h1, h2 = headings
        self.goal.x = self.leader.x - self.gap * math.cos(h1)
        self.goal.y = self.leader.y - self.gap * math.sin(h1)
        self.goal.heading = h1
        # Diagnostic only: UWB-free orbit odometry is NOT a measured offset.
        self.position_error = math.hypot(self.goal.x-self.follower.x, self.goal.y-self.follower.y)
        yaw_error = wrap_radians(h1 - h2)
        self.yaw_error_deg = math.degrees(yaw_error)
        self.follow_telemetry = dict(self.follow_distance.telemetry(),
            controller='RANGE_IMU', pose_error_control=False,
            yaw_error_deg=self.yaw_error_deg)
        if self.alignment_target is not None:
            errors = [wrap_radians(self.alignment_target - h) for h in headings]
            if any(abs(e) > math.radians(20) for e in errors):
                self.follow_telemetry['fault'] = 'ALIGNMENT_TOO_LARGE'
                return None
            rates = [self._yaw_rate(i) for i in (0, 1)]
            if any(r is None for r in rates):
                self.follow_telemetry['fault'] = 'ALIGN_GYRO_MISSING'
                return None
            self.leader_motion = Motion(0, 0, self._alignment_rotation(errors[0], rates[0]), 'AUTO_ALIGN_R1')
            return Motion(0, 0, self._alignment_rotation(errors[1], rates[1]), 'AUTO_ALIGN_R2')
        if abs(yaw_error) > math.radians(15):
            self.follow_telemetry['fault'] = 'FOLLOW_HEADING_LOST'
            return None
        leader = self.leader_motion
        # A stopped leader must never cause a follower pose-error chase.
        if max(abs(leader.vx), abs(leader.vy), abs(leader.w)) < 1e-6:
            self.last_follow_time, self.last_follow_vx = now, 0.0
            return Motion(0, 0, 0, 'AUTO_RANGE_HOLD')
        # Match commanded translation/heading, then trim longitudinal spacing
        # by <=0.04 using confirmed ultrasound. Do not invent lateral position.
        ff_forward = leader.vx * math.cos(yaw_error) - leader.vy * math.sin(yaw_error)
        ff_left = leader.vx * math.sin(yaw_error) + leader.vy * math.cos(yaw_error)
        forward = clamp(ff_forward + correction, 0, self.max_command)
        minimum = self.follow_distance.minimum
        # Raw range always wins over filtering: begin braking 6cm before STOP.
        guard = (1.0 if self.follow_distance.state == 'TARGET_LOST' and self.follow_distance.raw is None
                 else clamp((self.follow_distance.raw - (minimum + 6)) / 4.0, 0, 1))
        forward *= guard
        dt = clamp(now - self.last_follow_time, 0, .20)
        forward = min(forward, self.last_follow_vx + .60 * dt)
        self.last_follow_time, self.last_follow_vx = now, forward
        self.follow_telemetry.update(forward_trim=correction, forward_guard=guard,
                                     forward_command=forward)
        # Align R2 with the straight leg itself, not only a slightly skewed R1.
        target_error = (yaw_error if self.straight_heading_target is None else
                        wrap_radians(self.straight_heading_target-h2))
        yaw_command = clamp(.40 * target_error + .15 * yaw_error, -.10, .10)
        return Motion(forward, clamp(ff_left, -.12, .12) * guard,
            yaw_command, 'AUTO_RANGE_FOLLOW')

    def _calculate_orbit_motion(self):
        if time.monotonic() > self.route_deadline:
            return None
        self._sync_orbit_status()
        core = self.orbit_controller
        if not core._update_orbit_commands(time.monotonic()):
            return None
        leader, follower = core.active_commands[0], core.active_commands[1]
        scale = self.orbit_phase_scale
        h1, h2 = self._heading(0), self._heading(1)
        if h1 is None or h2 is None or not all(math.isfinite(h) for h in (h1, h2)):
            return None
        yaw_error = wrap_radians(h1-h2)
        if abs(yaw_error) > math.radians(15):
            core.orbit_telemetry['route_fault'] = 'ORBIT_IMU_HEADING_LOST'
            return None
        # Retain wheel/RPM calibration; add bounded IMU synchronization.
        imu_trim = clamp(.25*yaw_error, -.025, .025)
        self.leader_motion = Motion(leader.vx * scale, leader.vy * scale, leader.w * scale, leader.name)
        core.orbit_telemetry['phase_scale'] = scale
        core.orbit_telemetry['route_imu_trim'] = imu_trim
        self.goal.x = self.leader.x - self.gap * math.cos(self.leader.heading)
        self.goal.y = self.leader.y - self.gap * math.sin(self.leader.heading)
        self.goal.heading = self.leader.heading
        self.position_error = math.hypot(self.goal.x - self.follower.x, self.goal.y - self.follower.y)
        self.yaw_error_deg = math.degrees(wrap_radians(self.leader.heading - self.follower.heading))
        self.orbit_distance = core.orbit_telemetry['center_radius_cm'] / 100.0
        return Motion(follower.vx * scale, follower.vy * scale,
                      clamp(follower.w * scale + imu_trim, -.16, .16), follower.name)

    def state(self):
        with self.lock:
            result = super().state()
            result['calibrated_orbit'] = dict(self.orbit_controller.orbit_telemetry)
            result['range_follow'] = dict(self.follow_telemetry)
            result['follower_relative'], result['relative_notice'] = self._relative_display()
            result['calibration'] = {
                'route_revision': '2026-08-30-precision-v2',
                'alignment_absolute_deg': 1.0, 'alignment_relative_deg': 1.0,
                'r2_position_display': 'rear-beam assumption; raw odometry retained',
                'forward_odometry_scales': self.forward_odometry_scales,
                'straight_follower': 'RANGE_IMU (no pose-error chase)',
                'wheel_radius_m': self.wheel_radius, 'kinematic_radius_m': self.kinematic_radius,
                'imu_yaw_signs': self.yaw_signs,
                'orbit_tangent_scale': self.orbit_controller.orbit_tangent_scale,
                'orbit_radial_max_speed': self.orbit_controller.orbit_radial_max,
                'orbit_gap_deadband_cm': self.orbit_controller.orbit_distance.deadband,
                'orbit_distance_kp': self.orbit_controller.orbit_distance.kp,
                'orbit_radial_slew_per_sec': self.orbit_controller.orbit_distance.slew,
                'orbit_yaw_rpm_kp': self.orbit_controller.orbit_yaw_rpm_kp,
                'orbit_encoder_heading_kp': self.orbit_controller.orbit_encoder_heading_kp,
                'orbit_encoder_heading_max': self.orbit_controller.orbit_encoder_heading_max,
                'firmware_versions': [self._fields(i).get('fw', 'OFFLINE') for i in (0, 1)],
            }
            return result


class AutoFormationApp:
    def __init__(self, config: dict[str, Any], system_factory=None):
        autonomous_config = dict(config)
        autonomous_config.update(
            {
                "formation_start_leader_x_m": 0.0,
                "formation_start_leader_y_m": 0.0,
                "formation_start_heading_deg": 90.0,
                "formation_gap_m": (30.0 + float(config.get("orbit_center_offset_cm", 19.4))) / 100.0,
                "fusion_uwb_enabled": False,
            }
        )
        self.system = (system_factory or RouteFormationSystem)(autonomous_config)
        self.config = autonomous_config
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.abort = threading.Event()
        self.route_thread: threading.Thread | None = None
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)

        self.speed = clamp(float(config.get("auto_route_default_speed", 0.30)), 0.20, 0.50)
        self.lookahead = float(config.get("auto_route_lookahead_m", 0.16))
        self.position_kp = float(config.get("auto_route_position_kp", 0.9))
        self.heading_kp = float(config.get("auto_route_heading_kp", 0.18))
        self.max_rotation = float(config.get("auto_route_max_rotation", 0.16))
        self.max_position_correction = float(config.get("auto_route_max_correction", 0.16))
        self.return_enabled = bool(config.get("auto_route_return_enabled", False))
        self.orbit_speed = clamp(float(config.get("auto_route_orbit_speed", 0.22)), 0.10, 0.22)
        self.turn_degrees = 0.0
        self.turn_target_degrees = 0.0
        self.start_gap_cm: float | None = None
        self.path_index = 0
        self.path_count = len(OUTBOUND_PATH) + len(RETURN_PATH)
        self.route_state = "WAITING_QR"
        self.route_label = "대기"
        self.route_progress = 0.0
        self.last_error = ""
        self.running = False
        self.last_qr = ""
        self.qr_candidate = ""
        self.qr_stable_count = 0
        self.qr_absent_count = 0
        self.qr_armed = True
        self.camera_ok = False
        self.camera_message = "카메라 시작 전"
        self.latest_jpeg: bytes | None = None
        self.log_path = ""
        self.log_file: Any | None = None
        self.log_writer: csv.writer | None = None

    def start(self) -> None:
        self.system.start()
        self.camera_thread.start()

    def close(self) -> None:
        self.abort.set()
        self.shutdown.set()
        if self.route_thread and self.route_thread.is_alive():
            self.route_thread.join(timeout=2.0)
        if self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)
        self.system.close()

    def set_speed(self, speed: float) -> float:
        if not math.isfinite(speed):
            raise ValueError("속도는 유한한 숫자여야 합니다.")
        with self.lock:
            self.speed = clamp(speed, 0.20, 0.50)
            return self.speed

    def set_return(self, enabled: bool) -> None:
        with self.lock:
            if self.running or (self.route_thread and self.route_thread.is_alive()):
                raise ValueError("복귀 여부는 출발 전에만 변경할 수 있습니다.")
            self.return_enabled = enabled

    def trigger_route(self, source: str) -> tuple[bool, str]:
        with self.lock:
            if self.running or (self.route_thread and self.route_thread.is_alive()):
                return False, "이미 자동 주행 중입니다."
            self.abort.clear()
            self.running = True
            self.route_state = "INITIALIZING"
            self.route_label = f"시작 신호: {source}"
            self.route_progress = 0.0
            self.last_error = ""
            self.qr_armed = False
            self.route_thread = threading.Thread(
                target=self._route_worker, name="auto-roundtrip", daemon=True
            )
            self.route_thread.start()
            return True, "ㄱ자 자동 주행을 시작합니다."

    def stop_route(self) -> None:
        self.abort.set()
        self.system.emergency_stop()
        with self.lock:
            self.running = False
            self.route_state = "EMERGENCY_STOP"
            self.route_label = "사용자 비상 정지"

    def _wait_for_reset(self, timeout: float = 6.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout
        message = "로봇 상태 대기 중"
        while time.monotonic() < deadline and not self.abort.is_set():
            # Ultrasonic gap is surface-to-sensor distance, not center spacing.
            with self.system.lock:
                if not self.system._status_fresh(1, time.monotonic()):
                    gap = None
                else:
                    try:
                        gap = self._range_cm()
                    except RuntimeError as error:
                        return False, str(error)
                if gap is not None:
                    center_gap = (gap + float(self.config.get("orbit_center_offset_cm", 19.4))) / 100.0
                    if not self.system.orbit_min_radius <= center_gap <= self.system.orbit_max_radius:
                        return False, "시작 간격이 공전 반지름 범위를 벗어났습니다. 초음파 약 30cm로 배치하세요."
                    self.system.gap = center_gap
                    self.system.follow_target_gap_cm = gap
                    self.start_gap_cm = gap
            if gap is None:
                time.sleep(0.10)
                continue
            ok, message = self.system.reset_pose()
            if ok:
                return True, message
            time.sleep(0.25)
        return False, message

    def _route_worker(self) -> None:
        try:
            self._open_log()
            ok, message = self._wait_for_reset()
            if not ok:
                raise RuntimeError(f"위치 초기화 실패: {message}")
            # Every leg ends with BOTH robots stopped. No curved shortcut.
            out_end = 0.46 if self.return_enabled else 1.0
            self._follow_path(OUTBOUND_Y_PATH, "+y 직진 · (0,3)으로", 0.0, .40 * out_end)
            self._dwell("(0,3) 정지 · 시계 90° 전환 준비", 0.5)
            self._rotate_formation(math.pi / 2, -1, "(0,3) 시계 90° 공전", target_heading=0.0)
            self._follow_path(OUTBOUND_X_PATH, "+x 직진 · (4,3)으로", .45 * out_end, out_end)
            if self.return_enabled:
                self._dwell("(4,3) 도착 · 선택된 자동 복귀 준비", 1.0)
                self._rotate_formation(math.pi, 1, "복귀를 위한 180° 전환", target_heading=math.pi)
                self._follow_path(RETURN_X_PATH, "-x 직진 · (0,3)으로", .50, .73)
                self._dwell("(0,3) 정지 · 반시계 90° 전환 준비", 0.5)
                self._rotate_formation(math.pi / 2, 1, "(0,3) 반시계 90° 공전", target_heading=-math.pi / 2)
                self._follow_path(RETURN_Y_PATH, "-y 직진 · (0,0)으로", .77, .97)
                self._rotate_formation(math.pi, 1, "초기 +y 방향 복원", target_heading=math.pi / 2)
            self._check_abort()
            self._stop_formation()
            with self.lock:
                self.route_state = "COMPLETE"
                destination = "(0,0) 복귀" if self.return_enabled else "(4,3) 도착"
                self.route_label = f"{destination} 완료 · 두 로봇 정지 · 재출발 전 시작 위치로 옮기세요."
                self.route_progress = 1.0
        except InterruptedError:
            self.system.emergency_stop()
            with self.lock:
                self.route_state = "EMERGENCY_STOP"
                self.route_label = "자동 주행 중지"
        except Exception as error:
            self.system.emergency_stop()
            with self.lock:
                self.route_state = "ERROR"
                self.route_label = "자동 주행 오류"
                self.last_error = str(error)
            self._write_log("ERROR")
        finally:
            self._stop_formation()
            self._close_log()
            with self.lock:
                self.running = False

    def _open_log(self) -> None:
        log_directory = Path(str(self.config.get("_config_dir", "."))) / "experiment_results"
        log_directory.mkdir(parents=True, exist_ok=True)
        prefix = getattr(self, 'log_prefix', 'auto_roundtrip')
        path = log_directory / f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = path.open("w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            [
                "elapsed_sec", "stage", "route_state", "speed_command",
                "r1_x_m", "r1_y_m", "r1_heading_deg",
                "r2_x_m", "r2_y_m", "r2_heading_deg",
                "r1_vx", "r1_vy", "r1_w", "r2_vx", "r2_vy", "r2_w",
                "formation_error_m", "formation_yaw_error_deg",
                "r1_target_rpm", "r1_actual_rpm", "r1_drive_pwm",
                "r2_target_rpm", "r2_actual_rpm", "r2_drive_pwm",
                "r1_ultrasonic_cm", "r2_ultrasonic_cm", "start_gap_cm", "center_gap_m",
                "turn_degrees", "turn_target_degrees", "safety", "last_error",
                "orbit_gap_filtered_cm", "orbit_radial_command", "orbit_tangent_scale",
                "orbit_encoder_heading_error_deg", "orbit_gap_filter_state",
                "r1_forward_odometry_scale", "r2_forward_odometry_scale",
                "follower_controller", "follow_gap_filtered_cm", "follow_forward_trim",
                "follow_forward_guard", "follow_fault", "orbit_phase_scale",
                "r1_gyro_dps", "r2_gyro_dps", "fault_gap_cm", "orbit_imu_trim",
                "r2_relative_x_m", "r2_relative_y_m", "relative_position_notice",
            ]
        )
        self.log_started_at = time.monotonic()
        with self.lock:
            self.log_path = str(path)

    def _write_log(self, stage: str) -> None:
        if self.log_writer is None:
            return
        state = self.system.state()
        leader = state["leader"]
        follower = state["follower"]
        leader_motion = state["leader_motion"]
        follower_motion = state["follower_motion"]
        with self.system.lock:
            robot1_fields = dict(self.system._fields(0))
            robot2_fields = dict(self.system._fields(1))
        self.log_writer.writerow(
            [
                f"{time.monotonic() - self.log_started_at:.3f}", stage, self.route_state,
                f"{self.speed:.3f}", f"{leader['x']:.5f}", f"{leader['y']:.5f}",
                leader["heading_deg"], f"{follower['x']:.5f}", f"{follower['y']:.5f}",
                follower["heading_deg"], leader_motion["vx"], leader_motion["vy"],
                leader_motion["w"], follower_motion["vx"], follower_motion["vy"],
                follower_motion["w"], state["error_m"], state["yaw_error_deg"],
                robot1_fields.get("target_rpm", ""), robot1_fields.get("rpm", ""),
                robot1_fields.get("drive_pwm", ""), robot2_fields.get("target_rpm", ""),
                robot2_fields.get("rpm", ""), robot2_fields.get("drive_pwm", ""),
                robot1_fields.get("distance_cm", ""), robot2_fields.get("distance_cm", ""),
                self.start_gap_cm, self.system.gap, self.turn_degrees, self.turn_target_degrees,
                state["safety"], self.last_error,
                *(state.get("calibrated_orbit", {}).get(key, "") for key in (
                    "gap_filtered_cm", "radial_command", "tangent_scale", "encoder_heading_error_deg", "gap_filter_state")),
                *self.system.forward_odometry_scales,
                *(state.get("range_follow", {}).get(key, "") for key in (
                    "controller", "gap_filtered_cm", "forward_trim", "forward_guard", "fault")),
                state.get("calibrated_orbit", {}).get("phase_scale", ""),
                robot1_fields.get('gyro_dps', ''), robot2_fields.get('gyro_dps', ''),
                state.get('range_follow', {}).get('gap_cm', ''),
                state.get('calibrated_orbit', {}).get('route_imu_trim', ''),
                (state.get('follower_relative') or {}).get('x', ''),
                (state.get('follower_relative') or {}).get('y', ''),
                state.get('relative_notice', ''),
            ]
        )

    def _close_log(self) -> None:
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    def _check_abort(self) -> None:
        if self.abort.is_set() or self.shutdown.is_set():
            raise InterruptedError

    def _stop_formation(self) -> None:
        # stop_leader() alone does NOT leave ORBIT mode in FormationSystem.
        self.system.set_orbit(False)

    def _enable_follow(self) -> None:
        self._check_abort()
        with self.system.lock:
            self.system.route_deadline = time.monotonic() + 0.6
            ok, message = self.system.set_follow(True)
        if not ok:
            raise RuntimeError(message)

    def _range_cm(self) -> float:
        try:
            gap = float(self.system._fields(1)["distance_cm"])
        except (KeyError, TypeError, ValueError):
            raise RuntimeError("R2 초음파 측정값 없음") from None
        minimum = max(18.0, float(self.config.get("orbit_min_gap_cm", 18.0)))
        if not math.isfinite(gap) or gap <= minimum or gap > 150:
            raise RuntimeError(f"R2 초음파 안전정지: {gap}cm (최소 {minimum:g}cm)")
        return gap

    def _check_active(self, *, turning: bool = False) -> float:
        self._check_abort()
        with self.system.lock:
            now = time.monotonic()
            if not self.system.follow_enabled:
                detail = getattr(self.system, 'follow_telemetry', {}).get('fault', '')
                raise RuntimeError("센서 또는 장애물 안전정지: " + self.system.safety + " " + detail)
            for index in (0, 1):
                heading = self.system._heading(index)
                if (not self.system._status_fresh(index, now) or heading is None
                        or not math.isfinite(heading)):
                    raise RuntimeError(f"Robot {index + 1} 상태 또는 IMU 연결 끊김")
                if self.system._fields(index).get("obstacle") == "1":
                    raise RuntimeError(f"Robot {index + 1} 장애물 안전정지")
            gap = self._range_cm()
            if turning and abs(gap - self._turn_gap_cm) > float(self.config.get("orbit_max_gap_error_cm", 15.0)):
                raise RuntimeError("공전 거리 이탈: RANGE_TARGET_LOST")
            self.system.route_deadline = now + 0.6
            return gap

    def _dwell(self, label: str, seconds: float) -> None:
        self._stop_formation()
        with self.lock:
            self.route_state = "DWELL"
            self.route_label = label
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._check_abort()
            time.sleep(0.05)

    def _pose(self) -> tuple[float, float, float]:
        with self.system.lock:
            return self.system.leader.x, self.system.leader.y, self.system.leader.heading

    def _follow_path(self, points: list[PathPoint], phase_label: str,
                     progress_start: float = 0.0, progress_end: float = 1.0) -> None:
        with self.system.lock:
            self.system.straight_heading_target = points[-1].heading
        self._enable_follow()
        index = 0
        start_time = time.monotonic()
        while True:
            self._check_active()
            if time.monotonic() - start_time > 90.0:
                raise RuntimeError(f"{phase_label} 제한시간 초과")
            if not self.system.follow_enabled:
                raise RuntimeError("센서 또는 장애물 안전 정지 발생")

            x, y, heading = self._pose()
            search_end = min(len(points), index + 30)
            index = min(
                range(index, search_end),
                key=lambda candidate: math.hypot(
                    points[candidate].x - x, points[candidate].y - y
                ),
            )
            final = points[-1]
            final_distance = math.hypot(final.x - x, final.y - y)
            final_heading_error = wrap_radians(final.heading - heading)
            remaining, cross_error = leg_errors(x, y, final.x, final.y, final.heading)
            if remaining <= .03:
                self._stop_formation()
                if remaining < -.08 or abs(cross_error) > .06 or abs(final_heading_error) > math.radians(3):
                    raise RuntimeError('직진 종점 오차 초과: 자동 후진하지 않고 정지합니다.')
                self._dwell('직진 종점 정지 확인', .35)
                self._write_log(final.label + '_STOPPED')
                with self.lock:
                    self.route_progress = progress_end
                return

            target_index = index
            accumulated = 0.0
            while target_index + 1 < len(points) and accumulated < self.lookahead:
                current = points[target_index]
                following = points[target_index + 1]
                accumulated += math.hypot(following.x - current.x, following.y - current.y)
                target_index += 1
            target = points[target_index]

            with self.lock:
                commanded_speed = self.speed
                self.path_index = index
                self.route_progress = progress_start + (progress_end - progress_start) * index / max(1, len(points) - 1)
                self.route_state = "DRIVING"
                self.route_label = f"{phase_label} · {target.label}"

            if final_distance < 0.30:
                # Retain 0.20 minimum for traction, but brake earlier and reduce
                # the old 6.5cm early-arrival allowance to 3cm.
                commanded_speed = min(commanded_speed, .20)
                world_vx = self.position_kp * (final.x - x)
                world_vy = self.position_kp * (final.y - y)
                magnitude = math.hypot(world_vx, world_vy)
                if magnitude > commanded_speed:
                    world_vx *= commanded_speed / magnitude
                    world_vy *= commanded_speed / magnitude
                elif magnitude > 0.0 and magnitude < min(0.20, commanded_speed):
                    scale = min(0.20, commanded_speed) / magnitude
                    world_vx *= scale
                    world_vy *= scale
                reference_heading = final.heading
                curvature = 0.0
            else:
                aim_x = target.x - x
                aim_y = target.y - y
                aim_length = max(0.001, math.hypot(aim_x, aim_y))
                world_vx = commanded_speed * aim_x / aim_length
                world_vy = commanded_speed * aim_y / aim_length
                nearest = points[index]
                world_vx += clamp(
                    self.position_kp * (nearest.x - x),
                    -self.max_position_correction,
                    self.max_position_correction,
                )
                world_vy += clamp(
                    self.position_kp * (nearest.y - y),
                    -self.max_position_correction,
                    self.max_position_correction,
                )
                reference_heading = target.heading
                curvature = target.curvature

            forward = world_vx * math.cos(heading) + world_vy * math.sin(heading)
            left = -world_vx * math.sin(heading) + world_vy * math.cos(heading)
            # Both robots leave a stopped corner together, over ~0.5s at 0.3.
            ramp_limit = .60 * (time.monotonic() - start_time)
            magnitude = math.hypot(forward, left)
            if magnitude > ramp_limit and magnitude > 0:
                forward *= ramp_limit / magnitude
                left *= ramp_limit / magnitude
            heading_error = wrap_radians(reference_heading - heading)
            rotation_feedforward = commanded_speed * self.system.kinematic_radius * curvature
            rotation = clamp(
                rotation_feedforward + self.heading_kp * heading_error,
                -self.max_rotation,
                self.max_rotation,
            )
            ok, message = self.system.set_motion(
                forward, left, rotation, f"AUTO_{target.label}"
            )
            if not ok:
                raise RuntimeError(message)
            self._write_log(target.label)
            time.sleep(0.07)

    def _rotate_formation(self, angle: float, direction: int, label: str,
                          *, target_heading: float | None = None) -> None:
        self._check_abort()
        direction_name = "CCW" if direction > 0 else "CW"
        _, _, previous = self._pose()
        target_heading = wrap_radians(previous + direction * angle) if target_heading is None else target_heading
        required_angle = (direction * wrap_radians(target_heading - previous)) % (2 * math.pi)
        if abs(required_angle - angle) > math.radians(15):
            raise RuntimeError("공전 시작 방향이 경로 방향과 맞지 않습니다.")
        with self.lock:
            self.route_state = "TURNING"
            self.route_label = label
            self.turn_degrees = 0.0
            self.turn_target_degrees = math.degrees(required_angle)
        with self.system.lock:
            self._turn_gap_cm = self._range_cm()
            self.system.route_deadline = time.monotonic() + 0.6
            ok, message = self.system.set_orbit(True, direction_name, self.system.gap, self.orbit_speed)
        if not ok:
            raise RuntimeError(message)
        accumulated = 0.0
        deadline = time.monotonic() + 18.0
        try:
            while accumulated < required_angle - math.radians(3.0):
                self._check_active(turning=True)
                if time.monotonic() > deadline:
                    raise RuntimeError(f"{label} 제한시간 초과")
                _, _, current = self._pose()
                delta = direction * wrap_radians(current - previous)
                if abs(delta) > math.radians(25):
                    raise RuntimeError("공전 IMU 각도 급변")
                accumulated += delta  # Signed: oscillation must not count as progress.
                if accumulated < -math.radians(10):
                    raise RuntimeError("공전 방향이 반대입니다. IMU 부호/배선을 확인하세요.")
                previous = current
                remaining = max(0.0, required_angle - accumulated)
                with self.system.lock:
                    # Preserve calibrated command ratios; slow BOTH near exit.
                    self.system.orbit_phase_scale = clamp(
                        remaining / math.radians(30), .55, 1.0)
                with self.lock:
                    self.turn_degrees = math.degrees(accumulated)
                self._write_log(label)
                time.sleep(0.04)
        finally:
            self._stop_formation()  # Also stops on abort, timeout and sensor loss.
        self._dwell("90° 전환 정지 확인" if angle < math.pi else "방향 전환 정지 확인", 0.35)
        _, _, stopped_heading = self._pose()
        if abs(wrap_radians(target_heading - stopped_heading)) > math.radians(8):
            raise RuntimeError("공전 종료 각도 오차가 8°를 초과했습니다. 두 로봇 정지.")
        self._align_headings(target_heading)

    def _align_headings(self, target_heading: float) -> None:
        """Only in-place rotation; neither robot translates to chase drift."""
        self._check_abort()
        with self.system.lock:
            self.system.alignment_target = target_heading
            self._enable_follow()
        with self.lock:
            self.route_state = 'ALIGNING'
            self.route_label = '공전 종료 · 두 로봇 방향 정렬 중 (전진 금지)'
        deadline = time.monotonic() + 12.0
        stable_since = None
        try:
            while True:
                self._check_active(turning=True)
                now = time.monotonic()
                with self.system.lock:
                    headings = [self.system._heading(i) for i in (0, 1)]
                    rates = [self.system._yaw_rate(i) for i in (0, 1)]
                errors = [abs(wrap_radians(target_heading - h)) for h in headings]
                if max(errors) > math.radians(20):
                    raise RuntimeError('공전 종료 방향 오차 20° 초과: 배치 확인 필요')
                if any(r is None for r in rates):
                    raise RuntimeError('방향 정렬 중 자이로 회전속도 데이터 없음')
                aligned = headings_settled(headings, rates, target_heading)
                stable_since = (now if stable_since is None else stable_since) if aligned else None
                self._write_log('TURN_ALIGN')
                if stable_since is not None and now - stable_since >= 1.0:
                    break
                if now > deadline:
                    raise RuntimeError('공전 후 방향 정렬 제한시간: R1/R2가 맞지 않아 직진 차단')
                time.sleep(.05)
        finally:
            self._stop_formation()
        self._dwell('방향 정렬 완료 · 직진 준비', .20)
        # Check coast after STOP as well, without re-zeroing either IMU/pose.
        with self.system.lock:
            now = time.monotonic()
            for index in (0, 1):
                h = self.system._heading(index)
                if (not self.system._status_fresh(index, now) or h is None or not math.isfinite(h)
                        or abs(wrap_radians(target_heading - h)) > math.radians(1)):
                    raise RuntimeError(f'R{index+1} 정렬 후 방향 확인 실패: 직진 차단')
            if not headings_settled([self.system._heading(i) for i in (0, 1)],
                                    [self.system._yaw_rate(i) for i in (0, 1)], target_heading):
                raise RuntimeError('정렬 후 회전 잔류/상대방향 오차: 직진 차단')
            self._range_cm()

    def _camera_loop(self) -> None:
        if cv2 is None:
            with self.lock:
                self.camera_message = "OpenCV가 설치되어 있지 않습니다."
            return
        camera_index = int(self.config.get("camera_index", 0))
        capture = cv2.VideoCapture(camera_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.config.get("frame_width", 1280)))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("frame_height", 720)))
        if not capture.isOpened():
            with self.lock:
                self.camera_message = f"카메라 {camera_index} 열기 실패"
            return
        detector = cv2.QRCodeDetector()
        with self.lock:
            self.camera_ok = True
            self.camera_message = "QR 대기 중"
        try:
            while not self.shutdown.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    with self.lock:
                        self.camera_ok = False
                        self.camera_message = "카메라 프레임 읽기 실패"
                    time.sleep(0.05)
                    continue
                value = ""
                try:
                    value, _, _ = detector.detectAndDecode(frame)
                except cv2.error:
                    detector = cv2.QRCodeDetector()

                preview = frame
                if frame.shape[1] > 720:
                    scale = 720.0 / frame.shape[1]
                    preview = cv2.resize(frame, None, fx=scale, fy=scale)
                encoded, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 72])
                if encoded:
                    with self.lock:
                        self.latest_jpeg = jpeg.tobytes()
                self._handle_qr(value.strip())
                time.sleep(0.015)
        finally:
            capture.release()

    def _handle_qr(self, value: str) -> None:
        trigger = False
        display_value = value[:120]
        with self.lock:
            if value:
                self.qr_absent_count = 0
                if value == self.qr_candidate:
                    self.qr_stable_count += 1
                else:
                    self.qr_candidate = value
                    self.qr_stable_count = 1
                self.last_qr = display_value
                self.camera_message = f"QR 감지: {display_value}"
                if self.qr_armed and self.qr_stable_count >= 3 and not self.running:
                    self.qr_armed = False
                    trigger = True
            else:
                self.qr_candidate = ""
                self.qr_stable_count = 0
                if not self.running:
                    self.qr_absent_count += 1
                    if self.qr_absent_count >= 5:
                        self.qr_armed = True
                        self.camera_message = "QR 대기 중"
        if trigger:
            self.trigger_route(f"QR: {display_value}")

    def state(self) -> dict[str, Any]:
        base = self.system.state()
        with self.lock:
            base["auto"] = {
                "running": self.running,
                "state": self.route_state,
                "label": self.route_label,
                "progress": self.route_progress,
                "speed": self.speed,
                "return_enabled": self.return_enabled,
                "orbit_speed": self.orbit_speed,
                "turn_degrees": self.turn_degrees,
                "turn_target_degrees": self.turn_target_degrees,
                "start_gap_cm": self.start_gap_cm,
                "waypoints": [[0, 0], [0, 3], [4, 3]],
                "last_error": self.last_error,
                "qr": self.last_qr,
                "qr_armed": self.qr_armed,
                "camera_ok": self.camera_ok,
                "camera_message": self.camera_message,
                "uwb_enabled": False,
                "log_path": self.log_path,
            }
        return base


HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>QR ㄱ자 군집주행 · 90° 공전</title><style>
:root{--bg:#07111f;--panel:#101d30;--line:#28415e;--text:#eaf2ff;--muted:#8da4bf;--blue:#40a9ff;--orange:#ff9f43;--green:#35d07f;--red:#ff5d68}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,"Noto Sans KR",sans-serif}.wrap{max-width:1450px;margin:auto;padding:18px}.top{display:flex;justify-content:space-between;gap:15px;align-items:center}.title{font-size:25px;font-weight:800}.sub,.label{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:minmax(620px,1.5fr) minmax(380px,.8fr);gap:14px;margin-top:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}.badges{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:7px 10px;border-radius:999px;background:#27384d;font-size:12px}.ok{background:#124c36;color:#8ff0be}.bad{background:#54252a;color:#ffadb3}canvas{width:100%;height:auto;background:#091626;border-radius:10px}.camera{width:100%;max-height:280px;object-fit:contain;background:#050b12;border-radius:10px;margin-top:10px}.notice{padding:12px;background:#0b1727;border-radius:10px;line-height:1.5;margin:10px 0}.stats{display:grid;grid-template-columns:1fr 1fr;gap:8px}.stat{padding:10px;background:#0b1727;border-radius:9px}.stat b{display:block;margin-top:4px}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:12px 0}.btn{padding:13px;border:1px solid #335579;border-radius:10px;background:#1768a7;color:white;font-weight:700;cursor:pointer}.danger{background:#9e2931;border-color:#d34a54}input[type=range]{width:100%}.progress{height:13px;background:#07111f;border-radius:8px;overflow:hidden;margin:9px 0}.bar{height:100%;width:0;background:linear-gradient(90deg,#40a9ff,#35d07f)}@media(max-width:980px){.grid{grid-template-columns:1fr}}
</style></head><body><div class="wrap"><div class="top"><div><div class="title">QR ㄱ자 군집주행 · 90° 공전</div><div class="sub">로봇1 중심: (0,0) → (0,3) 정지 → 시계 90° → (4,3) 정지 · Encoder + IMU / UWB OFF</div></div><div class="badges"><span id="r1b" class="badge">R1</span><span id="r2b" class="badge">R2</span><span id="cb" class="badge">CAMERA</span><span class="badge ok">UWB OFF</span></div></div>
<div class="grid"><section class="card"><canvas id="map" width="920" height="690"></canvas><p class="sub">파랑 R1: Encoder+IMU 추정 · 주황 R2: 후면 조준을 가정한 초음파·IMU 편대 추정 (절대좌표 아님)</p><label class="label"><input id="showOdom" type="checkbox" onchange="draw()"> R2 엔코더 누적 좌표도 표시 (회색 십자)</label><div id="positionNotice" class="notice"></div><div id="rawPose" class="sub"></div></section><aside class="card"><div id="notice" class="notice">데이터 대기 중...</div><div class="progress"><div id="bar" class="bar"></div></div><div class="stats"><div class="stat"><span class="label">주행 상태</span><b id="mode">-</b></div><div class="stat"><span class="label">진행률</span><b id="progress">0%</b></div><div class="stat"><span class="label">Robot 1 · 보정된 Encoder+IMU</span><b id="p1">-</b></div><div class="stat"><span class="label">Robot 2 · 조건부 편대 추정</span><b id="p2">-</b></div><div class="stat"><span class="label">QR 상태</span><b id="qr">-</b></div><div class="stat"><span class="label">오류</span><b id="error">-</b></div><div class="stat" style="grid-column:1/-1"><span class="label">CSV 기록</span><b id="log">주행 시작 시 생성</b></div></div>
<div id="turnInfo" class="notice">공전 대기 · 시작 시 두 로봇 +y / 초음파 간격 약 30cm</div><details><summary>적용 중 보정값</summary><pre id="calibration" style="white-space:pre-wrap;font-size:12px"></pre></details><label class="label"><input id="returnOption" type="checkbox" onchange="setReturn(this.checked)"> 도착 후 자동 복귀 (추가 180° 전환 포함, 기본 꺼짐)</label><br><label class="label">직진 속도 <span id="sv">0.30</span></label><input id="speed" type="range" min="0.20" max="0.50" value="0.30" step="0.01" oninput="setSpeed(value)"><div class="buttons"><button class="btn" onclick="manualStart()">QR 없이 시험 시작</button><button class="btn danger" onclick="stopRoute()">비상 정지</button></div><img id="camera" class="camera" src="/api/frame.jpg"><div class="sub">네 종류 QR 중 어느 것이든 3프레임 연속 인식하면 같은 ㄱ자 경로를 시작합니다. 기본은 (4,3) 도착 후 정지입니다. 재출발은 반드시 두 로봇을 원래 시작 위치·방향으로 옮긴 뒤 QR을 치웠다가 다시 보여주세요. 코너에서 로봇2가 x&lt;0 영역으로 공전하므로 여유 공간을 확보하세요.</div></aside></div></div>
<script>
const $=id=>document.getElementById(id),canvas=$('map'),ctx=canvas.getContext('2d');let state=null;
async function post(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok)throw Error(d.message||'요청 실패');return d}
async function setSpeed(v){$('sv').textContent=Number(v).toFixed(2);try{await post('/api/speed',{speed:+v})}catch(e){alert(e.message)}}async function manualStart(){try{await post('/api/start')}catch(e){alert(e.message)}}async function stopRoute(){await post('/api/stop')}async function setReturn(v){try{await post('/api/return',{enabled:v})}catch(e){alert(e.message)}}
function mapxy(x,y){const xmin=-.85,xmax=4.85,ymin=-.85,ymax=3.85,m=65,s=Math.min((canvas.width-2*m)/(xmax-xmin),(canvas.height-2*m)/(ymax-ymin)),ox=(canvas.width-(xmax-xmin)*s)/2,oy=(canvas.height-(ymax-ymin)*s)/2;return[ox+(x-xmin)*s,canvas.height-oy-(y-ymin)*s]}
function path(){const pts=[[0,0],[0,3],[4,3]];ctx.strokeStyle='#35d07f';ctx.setLineDash([8,6]);ctx.lineWidth=3;ctx.beginPath();pts.forEach((v,i)=>{let p=mapxy(...v);i?ctx.lineTo(...p):ctx.moveTo(...p)});ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#eaf2ff';ctx.font='bold 14px sans-serif';pts.forEach((v,i)=>{let p=mapxy(...v);ctx.beginPath();ctx.arc(...p,5,0,Math.PI*2);ctx.fill();ctx.fillText(['시작 (0,0)','정지 + CW 90° (0,3)','도착·정지 (4,3)'][i],p[0]+8,p[1]-12)});const r=state?.start?.gap??.494;ctx.strokeStyle='#ff9f43';ctx.beginPath();for(let i=0;i<=30;i++){let a=-Math.PI/2-i*Math.PI/60,p=mapxy(r*Math.cos(a),3+r*Math.sin(a));i?ctx.lineTo(...p):ctx.moveTo(...p)}ctx.stroke();}
function robot(p,color,label){if(!p||p.heading_deg==null)return;let q=mapxy(p.x,p.y),a=-p.heading_deg*Math.PI/180;ctx.save();ctx.translate(...q);ctx.rotate(a);ctx.fillStyle=color;ctx.strokeStyle='white';ctx.lineWidth=2;ctx.beginPath();ctx.roundRect(-17,-12,34,24,5);ctx.fill();ctx.stroke();ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(28,0);ctx.stroke();ctx.restore();ctx.fillStyle='white';ctx.font='bold 14px sans-serif';ctx.fillText(label,q[0]+20,q[1]-15)}
function draw(){ctx.fillStyle='#091626';ctx.fillRect(0,0,canvas.width,canvas.height);let a=mapxy(0,0),b=mapxy(4,3);ctx.strokeStyle='#35516e';ctx.lineWidth=2;ctx.strokeRect(a[0],b[1],b[0]-a[0],a[1]-b[1]);ctx.strokeStyle='#18304a';for(let x=0;x<=4;x++){let p=mapxy(x,0),q=mapxy(x,3);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke()}for(let y=0;y<=3;y++){let p=mapxy(0,y),q=mapxy(4,y);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke()}path();if(state){robot(state.leader,'#40a9ff','R1');robot(state.follower_relative,'#ff9f43','R2 편대추정');if($('showOdom').checked){let p=mapxy(state.follower.x,state.follower.y);ctx.strokeStyle='#a9b3c1';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(p[0]-9,p[1]-9);ctx.lineTo(p[0]+9,p[1]+9);ctx.moveTo(p[0]-9,p[1]+9);ctx.lineTo(p[0]+9,p[1]-9);ctx.stroke();ctx.fillStyle='#a9b3c1';ctx.fillText('R2 엔코더 누적',p[0]+13,p[1]-10)}}}
function badge(id,ok,text){let e=$(id);e.textContent=text;e.className='badge '+(ok?'ok':'bad')}async function poll(){try{state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());badge('r1b',state.leader.online,'R1 '+(state.leader.online?'ONLINE':'OFFLINE'));badge('r2b',state.follower.online,'R2 '+(state.follower.online?'ONLINE':'OFFLINE'));badge('cb',state.auto.camera_ok,'CAMERA '+(state.auto.camera_ok?'OK':'ERROR'));$('notice').textContent=state.auto.label;$('mode').textContent=state.auto.state;$('progress').textContent=(state.auto.progress*100).toFixed(1)+'%';$('bar').style.width=(state.auto.progress*100)+'%';$('p1').textContent=`${state.leader.x.toFixed(2)}, ${state.leader.y.toFixed(2)} / ${state.leader.heading_deg?.toFixed(1)??'-'}°`;const rp=state.follower_relative;$('p2').textContent=rp?`${rp.x.toFixed(2)}, ${rp.y.toFixed(2)} / ${rp.heading_deg.toFixed(1)}°`:'추정 불가 (센서/가정 확인)';$('positionNotice').textContent=state.relative_notice||'좌표 추정 대기';$('rawPose').textContent=`R2 엔코더 누적: (${state.follower.x.toFixed(2)}, ${state.follower.y.toFixed(2)}) · 상대 각도 ${(state.yaw_error_deg??0).toFixed(1)}° · 초음파 ${state.follower.ultrasonic_cm??'-'}cm`;$('qr').textContent=state.auto.camera_message;$('error').textContent=state.auto.last_error||'-';$('log').textContent=state.auto.log_path||'주행 시작 시 생성';$('calibration').textContent=JSON.stringify(state.calibration,null,2);$('returnOption').checked=state.auto.return_enabled;$('returnOption').disabled=state.auto.running;$('speed').value=state.auto.speed;$('sv').textContent=state.auto.speed.toFixed(2);$('turnInfo').textContent=`공전 ${state.auto.turn_degrees.toFixed(1)}° / 목표 ${state.auto.turn_target_degrees.toFixed(1)}° · 공전속도 ${state.auto.orbit_speed.toFixed(2)} · 중심 간격 ${state.start.gap.toFixed(3)}m`;draw()}catch(e){$('notice').textContent='GUI 연결 오류: '+e.message}}setInterval(poll,120);setInterval(()=>{$('camera').src='/api/frame.jpg?t='+Date.now()},250);poll();draw();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    app: AutoFormationApp

    def log_message(self, format: str, *args: object) -> None:
        return

    def _body(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            return json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return {}

    def _json(self, document: dict[str, Any], status: int = 200) -> None:
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
            self._json(self.app.state())
        elif self.path.startswith("/api/frame.jpg"):
            with self.app.lock:
                payload = self.app.latest_jpeg
            if payload is None:
                self.send_response(204)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        body = self._body()
        try:
            if self.path == "/api/start":
                ok, message = self.app.trigger_route("GUI 수동 시작")
                self._json({"ok": ok, "message": message}, 200 if ok else 409)
            elif self.path == "/api/stop":
                self.app.stop_route()
                self._json({"ok": True, "message": "비상 정지"})
            elif self.path == "/api/speed":
                speed = self.app.set_speed(float(body.get("speed", 0.30)))
                self._json({"ok": True, "speed": speed})
            elif self.path == "/api/return":
                enabled = body.get("enabled")
                if not isinstance(enabled, bool):
                    raise ValueError("enabled must be boolean")
                self.app.set_return(enabled)
                self._json({"ok": True, "return_enabled": enabled})
            else:
                self.send_error(404)
        except (TypeError, ValueError) as error:
            self._json({"ok": False, "message": str(error)}, 400)


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    config["_config_dir"] = str(path.resolve().parent)
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--headless", action="store_true", help="Accepted for SSH compatibility")
    args = parser.parse_args()
    config = load_config(args.config)
    app = AutoFormationApp(config)
    Handler.app = app
    server = ThreadingHTTPServer(
        ("0.0.0.0", int(config.get("auto_route_web_port", 8085))), Handler
    )
    app.start()
    print(f"QR auto round-trip GUI: http://0.0.0.0:{server.server_port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
        app.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nQR auto round-trip stopped.")
        raise SystemExit(0)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
