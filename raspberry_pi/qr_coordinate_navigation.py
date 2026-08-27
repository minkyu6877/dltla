#!/usr/bin/env python3
"""Run the two-robot QR mission without UWB.

Robot 1 wheel RPM supplies leader translation. Robot 2's MPU6050 yaw is used
as the shared convoy heading, and both robots' encoders are monitored. Robot 2
uses its ultrasonic measurement to preserve the 30 cm physical gap.

The robots MUST be placed on the configured home marks before scanning a QR.
This program intentionally stops on stale RPM, IMU, ultrasound, Wi-Fi, or an
obstacle report.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import cv2
except ModuleNotFoundError:  # --check-config does not need a camera stack.
    cv2 = None

from odometry_navigation import (
    CoordinateMission,
    MecanumOdometry,
    Pose2D,
    Waypoint,
    command_from_physical,
    follower_command,
    limit_turn_for_follower,
)
from qr_dual_robot import (
    CameraSource,
    RobotFleet,
    StableQrTrigger,
    decode_qr,
    draw_qr_boxes,
    load_config,
)


@dataclass(frozen=True)
class DeliveryRequest:
    cargo_type: str
    weight_kg: float
    destination: tuple[float, float]

    def label(self) -> str:
        x, y = self.destination
        return f"{self.cargo_type}/{self.weight_kg:g}kg -> ({x:.2f}, {y:.2f})"


def parse_delivery_qr(text: str) -> DeliveryRequest | None:
    try:
        payload = json.loads(text)
        if not isinstance(payload, dict) or payload.get("command") != "DELIVERY":
            return None
        cargo_type = str(payload["cargo_type"]).strip().upper()
        weight_kg = float(payload["weight_kg"])
        destination = (float(payload["dest_x"]), float(payload["dest_y"]))
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if not cargo_type or not 0.0 < weight_kg <= 1000.0:
        return None
    if not all(math.isfinite(value) for value in destination):
        return None
    return DeliveryRequest(cargo_type, weight_kg, destination)


def motion_packet(command: tuple[float, float, float], state: str) -> str:
    vx, vy, w = command
    safe_state = state.replace(",", "_")
    return f"V,{vx:.3f},{vy:.3f},{w:.3f},{safe_state}"


class UwbLessCoordinator:
    def __init__(self, config: dict):
        nav = config.get("coordinate_navigation")
        if not isinstance(nav, dict):
            raise ValueError("Missing coordinate_navigation in config.json")

        home = nav.get("home_positions", {})
        robot1_home = self._pose(home.get("robot1"), "robot1")
        robot2_home = self._pose(home.get("robot2"), "robot2")
        wheel_radius = float(nav.get("wheel_radius_m", 0.03))
        wheelbase = float(nav.get("wheelbase_m", 0.194))
        track = float(nav.get("track_m", 0.30))
        self.rotation_radius_m = (wheelbase + track) / 2.0
        self.robot1 = MecanumOdometry(
            wheel_radius, wheelbase, track, robot1_home, use_imu=False
        )
        self.robot2 = MecanumOdometry(
            wheel_radius, wheelbase, track, robot2_home, use_imu=True
        )
        self.robot1_home = robot1_home
        self.robot2_home = robot2_home

        route_raw = nav.get("route")
        if not isinstance(route_raw, list):
            raise ValueError("coordinate_navigation.route must be a list")
        route = [Waypoint.from_config(raw) for raw in route_raw]
        self.mission = CoordinateMission(
            route=route,
            tolerance_m=float(nav.get("position_tolerance_m", 0.05)),
            heading_tolerance_deg=float(nav.get("heading_tolerance_deg", 4.0)),
            max_speed_mps=float(nav.get("max_speed_mps", 0.20)),
            min_speed_mps=float(nav.get("min_speed_mps", 0.04)),
            position_kp=float(nav.get("position_kp", 0.8)),
            heading_kp=float(nav.get("heading_kp", 1.6)),
            max_yaw_rate_rps=math.radians(
                float(nav.get("max_yaw_rate_deg_s", 45.0))
            ),
        )
        self.command_linear_scale_mps = float(
            nav.get("command_linear_scale_mps", 0.942)
        )
        self.target_gap_cm = float(nav.get("target_gap_cm", 30.0))
        self.gap_kp = float(nav.get("gap_command_kp", 0.012))
        self.max_gap_correction = float(nav.get("max_gap_correction", 0.08))
        self.center_offset_cm = float(nav.get("center_offset_cm", 23.0))
        self.min_gap_cm = float(nav.get("min_valid_gap_cm", 10.0))
        self.max_gap_cm = float(nav.get("max_valid_gap_cm", 150.0))
        self.destination_tolerance_m = float(
            nav.get("qr_destination_tolerance_m", 0.10)
        )
        destination_name = str(
            nav.get("destination_waypoint", "GREEN_DESTINATION")
        )
        matches = [point for point in route if point.name == destination_name]
        if len(matches) != 1:
            raise ValueError(
                f"Route must contain exactly one {destination_name!r} waypoint"
            )
        self.destination = (matches[0].x, matches[0].y)
        self.last_mode = "STOP"
        self.last_target = "STOP"

    @staticmethod
    def _pose(raw, robot: str) -> Pose2D:
        if not isinstance(raw, list) or len(raw) != 3:
            raise ValueError(
                f"coordinate_navigation.home_positions.{robot} must be [x,y,heading_deg]"
            )
        return Pose2D(float(raw[0]), float(raw[1]), math.radians(float(raw[2])))

    def qr_matches_destination(self, request: DeliveryRequest) -> bool:
        return math.dist(request.destination, self.destination) <= self.destination_tolerance_m

    def reset_at_home(self) -> None:
        self.robot1.reset(self.robot1_home)
        self.robot2.reset(self.robot2_home)
        self.last_mode = "STOP"
        self.last_target = "STOP"

    def update_odometry(
        self,
        robot1_snapshot: tuple[float, dict[str, str]],
        robot2_snapshot: tuple[float, dict[str, str]],
    ) -> tuple[bool, str]:
        timestamp2, fields2 = robot2_snapshot
        if fields2.get("imu_ok") != "1" or "att_deg" not in fields2:
            return False, "Robot 2 MPU6050 yaw unavailable"
        if not self.robot2.update(fields2, timestamp2):
            return False, "Robot 2 wheel RPM unavailable"

        timestamp1, fields1 = robot1_snapshot
        if not self.robot1.update(
            fields1, timestamp1, shared_heading=self.robot2.pose.heading
        ):
            return False, "Robot 1 wheel RPM unavailable"
        return True, "READY"

    def gap(self, fields2: dict[str, str]) -> float | None:
        try:
            gap = float(fields2["distance_cm"])
        except (KeyError, TypeError, ValueError):
            return None
        if fields2.get("us_verified") != "1":
            return None
        return gap if self.min_gap_cm <= gap <= self.max_gap_cm else None

    def commands(
        self, now: float, gap_cm: float
    ) -> tuple[str, str, str] | None:
        guidance = self.mission.guidance(self.robot1.pose, now)
        if guidance.mode == "COMPLETE":
            return None
        leader = command_from_physical(
            guidance, self.command_linear_scale_mps, self.rotation_radius_m
        )
        if guidance.mode == "TURN":
            # Robot 2 travels on the larger 0.53 m formation radius. Slow the
            # leader's in-place yaw so the follower's tangent never exceeds
            # the same 0.20 m/s mission speed limit.
            max_tangent_command = (
                self.mission.max_speed_mps / self.command_linear_scale_mps
            )
            leader = limit_turn_for_follower(
                leader,
                gap_cm,
                self.center_offset_cm,
                self.rotation_radius_m * 100.0,
                max_tangent_command,
            )
        follower = follower_command(
            leader_command=leader,
            mode=guidance.mode,
            gap_cm=gap_cm,
            target_gap_cm=self.target_gap_cm,
            gap_kp=self.gap_kp,
            max_gap_correction=self.max_gap_correction,
            rotation_radius_cm=self.rotation_radius_m * 100.0,
            center_offset_cm=self.center_offset_cm,
        )
        self.last_mode = guidance.mode
        self.last_target = guidance.name
        state = f"ODOM_{guidance.mode}_{guidance.name}"
        return motion_packet(leader, state), motion_packet(follower, state), state


def validation_summary(config: dict, coordinator: UwbLessCoordinator) -> str:
    nav = config["coordinate_navigation"]
    route = " -> ".join(point.name for point in coordinator.mission.route)
    return (
        "UWB-less configuration OK\n"
        f"  Robot 1 home: {nav['home_positions']['robot1']}\n"
        f"  Robot 2 home: {nav['home_positions']['robot2']}\n"
        f"  Destination: {coordinator.destination}\n"
        f"  Route: {route}"
    )


def main() -> int:
    default_config = Path(__file__).with_name("config.json")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--check-config", action="store_true",
        help="Validate configuration without opening the camera or motors",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    coordinator = UwbLessCoordinator(config)
    if args.check_config:
        print(validation_summary(config, coordinator))
        return 0
    if cv2 is None:
        raise RuntimeError(
            "OpenCV is not installed; run sudo apt install python3-opencv"
        )

    fleet = RobotFleet(
        list(config["robot_ips"]),
        int(config["command_port"]),
        int(config["status_port"]),
    )
    camera = None
    active = False
    send_interval = 1.0 / max(2.0, float(config.get("command_hz", 10.0)))
    next_send = 0.0
    next_pose_log = 0.0

    try:
        camera = CameraSource(config)
        detector = cv2.QRCodeDetector()
        trigger = StableQrTrigger(
            int(config.get("stable_frames", 3)),
            int(config.get("rearm_empty_frames", 5)),
        )
        print(validation_summary(config, coordinator))
        print("[READY] Put both robots on their home marks, then show the delivery QR")
        fleet.send("PING")

        while True:
            frame = camera.read()
            now = time.monotonic()
            fleet.receive_status()
            snap1 = fleet.status_snapshot(0, now)
            snap2 = fleet.status_snapshot(1, now)
            online = int(snap1 is not None) + int(snap2 is not None)

            values, points = decode_qr(detector, frame)
            emitted = trigger.update(values)
            if emitted is not None:
                request = parse_delivery_qr(emitted)
                if request is None:
                    print("[IGNORE] QR is not the coordinate DELIVERY format")
                elif active:
                    print("[IGNORE] A coordinate mission is already active")
                elif not coordinator.qr_matches_destination(request):
                    print(
                        f"[SAFETY] QR destination {request.destination} does not match "
                        f"the surveyed green point {coordinator.destination}"
                    )
                    fleet.emergency_stop()
                elif snap1 is None or snap2 is None:
                    print("[SAFETY] Both robots must be online before start")
                    fleet.emergency_stop()
                else:
                    coordinator.reset_at_home()
                    ready, reason = coordinator.update_odometry(snap1, snap2)
                    gap = coordinator.gap(snap2[1])
                    if not ready or gap is None:
                        print(f"[SAFETY] Cannot start: {reason}; ultrasonic gap={gap}")
                        fleet.emergency_stop()
                    else:
                        fleet.emergency_stop()
                        coordinator.mission.start()
                        active = True
                        print(f"[QR] {request.label()}")
                        print("[START] Home coordinates zeroed; two-robot mission started")

            leader_packet = follower_packet = "STOP"
            state = "STOP"
            if active:
                if snap1 is None or snap2 is None:
                    print("[SAFETY] ESP32 status stale; emergency stop")
                    active = False
                    coordinator.mission.stop()
                    fleet.emergency_stop()
                elif (alert := fleet.safety_alert(2, now)) is not None:
                    print(f"[SAFETY] {alert}; emergency stop")
                    active = False
                    coordinator.mission.stop()
                    fleet.emergency_stop()
                else:
                    ready, reason = coordinator.update_odometry(snap1, snap2)
                    gap = coordinator.gap(snap2[1])
                    if not ready or gap is None:
                        print(f"[SAFETY] {reason}; ultrasonic gap unavailable")
                        active = False
                        coordinator.mission.stop()
                        fleet.emergency_stop()
                    else:
                        result = coordinator.commands(now, gap)
                        if result is None:
                            active = False
                            fleet.emergency_stop()
                            print("[COMPLETE] Green destination visited; robots returned home")
                        else:
                            leader_packet, follower_packet, state = result

            if now >= next_send:
                if active:
                    fleet.send_to(0, leader_packet)
                    fleet.send_to(1, follower_packet)
                else:
                    fleet.send("STOP", 2)
                next_send = now + send_interval

            if active and now >= next_pose_log:
                p1, p2 = coordinator.robot1.pose, coordinator.robot2.pose
                print(
                    f"[POSE] R1=({p1.x:.2f},{p1.y:.2f},{math.degrees(p1.heading):.1f}deg) "
                    f"R2=({p2.x:.2f},{p2.y:.2f}) {state}"
                )
                next_pose_log = now + 1.0

            if not args.headless:
                draw_qr_boxes(frame, points)
                text = f"{state}  ONLINE {online}/2"
                cv2.putText(
                    frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0) if online == 2 else (0, 165, 255), 2,
                )
                cv2.imshow("QR UWB-less Coordinate Navigation", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        print("[SAFETY] Sending emergency STOP to both robots")
        try:
            fleet.emergency_stop()
        finally:
            fleet.close()
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
