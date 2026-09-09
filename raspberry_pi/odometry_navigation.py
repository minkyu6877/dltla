#!/usr/bin/env python3
"""UWB-less mecanum odometry and waypoint guidance.

The estimator integrates measured wheel RPM and uses the MPU6050 yaw as the
short-mission heading reference.  Coordinates are reset from surveyed floor
marks immediately before a QR mission starts; there is no claim that this is
an absolute-position replacement for UWB over long operating periods.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def parse_vector(raw: str | None, size: int) -> tuple[float, ...] | None:
    if raw is None:
        return None
    try:
        values = tuple(float(value) for value in raw.split(":"))
    except (TypeError, ValueError):
        return None
    return values if len(values) == size and all(math.isfinite(v) for v in values) else None


@dataclass
class Pose2D:
    x: float
    y: float
    heading: float


class MecanumOdometry:
    """Integrate FL/FR/RL/RR RPM in a map frame."""

    def __init__(
        self,
        wheel_radius_m: float,
        wheelbase_m: float,
        track_m: float,
        initial_pose: Pose2D,
        use_imu: bool,
    ):
        if min(wheel_radius_m, wheelbase_m, track_m) <= 0.0:
            raise ValueError("Mecanum dimensions must be positive")
        self.wheel_radius_m = wheel_radius_m
        self.rotation_radius_m = (wheelbase_m + track_m) / 2.0
        self.use_imu = use_imu
        self.pose = Pose2D(initial_pose.x, initial_pose.y, initial_pose.heading)
        self.initial_heading = initial_pose.heading
        self.last_timestamp: float | None = None
        self.imu_yaw_zero_deg: float | None = None

    def reset(self, pose: Pose2D) -> None:
        self.pose = Pose2D(pose.x, pose.y, pose.heading)
        self.initial_heading = pose.heading
        self.last_timestamp = None
        self.imu_yaw_zero_deg = None

    def _imu_heading(self, fields: dict[str, str]) -> float | None:
        if not self.use_imu or fields.get("imu_ok") != "1":
            return None
        attitude = parse_vector(fields.get("att_deg"), 3)
        if attitude is None:
            return None
        yaw_deg = attitude[2]
        if self.imu_yaw_zero_deg is None:
            self.imu_yaw_zero_deg = yaw_deg
        delta_deg = (yaw_deg - self.imu_yaw_zero_deg + 180.0) % 360.0 - 180.0
        return normalize_angle(self.initial_heading + math.radians(delta_deg))

    def update(
        self,
        fields: dict[str, str],
        timestamp: float,
        shared_heading: float | None = None,
    ) -> bool:
        """Consume one new status packet. Return False if RPM is unavailable."""
        rpm = parse_vector(fields.get("rpm"), 4)
        if rpm is None:
            return False

        imu_heading = self._imu_heading(fields)
        measured_heading = shared_heading if shared_heading is not None else imu_heading
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            if measured_heading is not None:
                self.pose.heading = measured_heading
            return True
        if timestamp <= self.last_timestamp:
            return True

        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        # Never integrate a long communications outage as if the last RPM had
        # remained valid. The caller treats stale packets as an emergency stop.
        if dt > 1.0:
            if measured_heading is not None:
                self.pose.heading = measured_heading
            return True

        fl, fr, rl, rr = (
            value * 2.0 * math.pi / 60.0 for value in rpm
        )
        radius = self.wheel_radius_m
        body_vx = radius * (fl + fr + rl + rr) / 4.0
        body_vy = radius * (-fl + fr + rl - rr) / 4.0
        encoder_yaw_rate = radius * (-fl + fr - rl + rr) / (
            4.0 * self.rotation_radius_m
        )

        old_heading = self.pose.heading
        new_heading = measured_heading
        if new_heading is None:
            new_heading = normalize_angle(old_heading + encoder_yaw_rate * dt)
        midpoint_heading = old_heading + normalize_angle(new_heading - old_heading) * 0.5
        cos_h, sin_h = math.cos(midpoint_heading), math.sin(midpoint_heading)
        self.pose.x += (cos_h * body_vx - sin_h * body_vy) * dt
        self.pose.y += (sin_h * body_vx + cos_h * body_vy) * dt
        self.pose.heading = normalize_angle(new_heading)
        return True


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    dwell_sec: float = 0.0
    reverse: bool = False

    @classmethod
    def from_config(cls, raw: dict) -> "Waypoint":
        waypoint = cls(
            name=str(raw["name"]),
            x=float(raw["x"]),
            y=float(raw["y"]),
            dwell_sec=float(raw.get("dwell_sec", 0.0)),
            reverse=bool(raw.get("reverse", False)),
        )
        if not all(math.isfinite(v) for v in (waypoint.x, waypoint.y, waypoint.dwell_sec)):
            raise ValueError(f"Invalid coordinate waypoint: {raw}")
        if waypoint.dwell_sec < 0.0:
            raise ValueError(f"Negative waypoint dwell: {raw}")
        return waypoint


@dataclass(frozen=True)
class Guidance:
    mode: str
    name: str
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    heading_error: float = 0.0


class CoordinateMission:
    """Generate body-frame motion toward the simulation's surveyed points."""

    def __init__(
        self,
        route: list[Waypoint],
        tolerance_m: float,
        heading_tolerance_deg: float,
        max_speed_mps: float,
        min_speed_mps: float,
        position_kp: float,
        heading_kp: float,
        max_yaw_rate_rps: float,
    ):
        if not route:
            raise ValueError("Coordinate route is empty")
        if min(tolerance_m, max_speed_mps, min_speed_mps, position_kp, heading_kp) <= 0:
            raise ValueError("Coordinate controller gains and speeds must be positive")
        self.route = route
        self.tolerance_m = tolerance_m
        self.heading_tolerance = math.radians(heading_tolerance_deg)
        self.max_speed_mps = max_speed_mps
        self.min_speed_mps = min(min_speed_mps, max_speed_mps)
        self.position_kp = position_kp
        self.heading_kp = heading_kp
        self.max_yaw_rate_rps = max_yaw_rate_rps
        self.index = -1
        self.dwell_until: float | None = None
        self.completed = False

    @property
    def active(self) -> bool:
        return 0 <= self.index < len(self.route) and not self.completed

    @property
    def target(self) -> Waypoint | None:
        return self.route[self.index] if self.active else None

    def start(self) -> None:
        self.index = 0
        self.dwell_until = None
        self.completed = False

    def stop(self) -> None:
        self.index = -1
        self.dwell_until = None

    def _advance(self) -> None:
        self.index += 1
        self.dwell_until = None
        if self.index >= len(self.route):
            self.completed = True
            self.index = -1

    def guidance(self, pose: Pose2D, now: float) -> Guidance:
        if not self.active:
            return Guidance("COMPLETE" if self.completed else "STOP", "STOP")

        target = self.route[self.index]
        dx, dy = target.x - pose.x, target.y - pose.y
        distance = math.hypot(dx, dy)
        if distance <= self.tolerance_m:
            if target.dwell_sec > 0.0:
                if self.dwell_until is None:
                    self.dwell_until = now + target.dwell_sec
                if now < self.dwell_until:
                    return Guidance("DWELL", target.name)
            self._advance()
            return self.guidance(pose, now)

        travel_heading = math.atan2(dy, dx)
        desired_heading = normalize_angle(
            travel_heading + (math.pi if target.reverse else 0.0)
        )
        heading_error = normalize_angle(desired_heading - pose.heading)
        if abs(heading_error) > self.heading_tolerance:
            yaw_rate = clamp(
                self.heading_kp * heading_error,
                -self.max_yaw_rate_rps,
                self.max_yaw_rate_rps,
            )
            return Guidance("TURN", target.name, yaw_rate_rps=yaw_rate,
                            heading_error=heading_error)

        speed = clamp(
            self.position_kp * distance,
            self.min_speed_mps,
            self.max_speed_mps,
        )
        # Convert the world-frame direction into the robot body frame. A
        # reverse waypoint naturally produces negative body vx.
        cos_h, sin_h = math.cos(pose.heading), math.sin(pose.heading)
        body_x = cos_h * dx + sin_h * dy
        body_y = -sin_h * dx + cos_h * dy
        vx = speed * body_x / distance
        vy = speed * body_y / distance
        yaw_rate = clamp(
            self.heading_kp * heading_error,
            -self.max_yaw_rate_rps,
            self.max_yaw_rate_rps,
        )
        return Guidance("MOVE", target.name, vx, vy, yaw_rate, heading_error)


def command_from_physical(
    guidance: Guidance,
    command_linear_scale_mps: float,
    rotation_radius_m: float,
) -> tuple[float, float, float]:
    """Convert m/s and rad/s into the firmware's normalized wheel request."""
    if command_linear_scale_mps <= 0.0 or rotation_radius_m <= 0.0:
        raise ValueError("Command scales must be positive")
    vx = guidance.vx_mps / command_linear_scale_mps
    vy = guidance.vy_mps / command_linear_scale_mps
    w = guidance.yaw_rate_rps * rotation_radius_m / command_linear_scale_mps
    largest = max(1.0, abs(vx), abs(vy), abs(w))
    return vx / largest, vy / largest, w / largest


def follower_command(
    leader_command: tuple[float, float, float],
    mode: str,
    gap_cm: float,
    target_gap_cm: float,
    gap_kp: float,
    max_gap_correction: float,
    rotation_radius_cm: float,
    center_offset_cm: float,
) -> tuple[float, float, float]:
    """Keep Robot 2 behind the leader, including the large corner sweep."""
    leader_vx, leader_vy, leader_w = leader_command
    radial = clamp(
        gap_kp * (gap_cm - target_gap_cm),
        -max_gap_correction,
        max_gap_correction,
    )
    if mode != "TURN" or abs(leader_w) < 1e-6:
        return (
            clamp(leader_vx + radial, -1.0, 1.0),
            leader_vy,
            leader_w,
        )

    center_radius_cm = max(rotation_radius_cm, gap_cm + center_offset_cm)
    tangent = abs(leader_w) * center_radius_cm / rotation_radius_cm
    tangent = clamp(tangent, 0.0, 1.0)
    if leader_w > 0.0:
        return radial, -tangent, leader_w
    return radial, tangent, leader_w


def limit_turn_for_follower(
    leader_command: tuple[float, float, float],
    gap_cm: float,
    center_offset_cm: float,
    rotation_radius_cm: float,
    max_tangent_command: float,
) -> tuple[float, float, float]:
    """Reduce leader yaw so the outer follower stays under the speed limit."""
    vx, vy, w = leader_command
    center_radius_cm = max(rotation_radius_cm, gap_cm + center_offset_cm)
    max_w = max_tangent_command * rotation_radius_cm / center_radius_cm
    return vx, vy, clamp(w, -max_w, max_w)
