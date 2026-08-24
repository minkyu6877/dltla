#!/usr/bin/env python3
"""Recognize a QR code and command two ESP32 robots over UDP."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2


@dataclass(frozen=True)
class Motion:
    vx: float
    vy: float
    w: float
    state: str

    def packet(self) -> str:
        return f"V,{self.vx:.3f},{self.vy:.3f},{self.w:.3f},{self.state}"


@dataclass(frozen=True)
class CargoInfo:
    destination: str
    cargo_type: str
    weight_kg: float

    def label(self) -> str:
        return f"{self.destination}/{self.cargo_type}/{self.weight_kg:g}kg"


class DriveSequence:
    """Raspberry Pi-side state machine. QR data never contains motor commands."""

    def __init__(self):
        self.steps: list[tuple[Motion, float]] = []
        self.step_index = -1
        self.step_deadline = 0.0
        self.cargo: Optional[CargoInfo] = None

    @property
    def active(self) -> bool:
        return 0 <= self.step_index < len(self.steps)

    @property
    def label(self) -> str:
        return self.steps[self.step_index][0].state if self.active else "STOP"

    def packet(self) -> str:
        return self.steps[self.step_index][0].packet() if self.active else "STOP"

    def start(self, cargo: CargoInfo, steps: list[tuple[Motion, float]], now: float) -> None:
        if not steps:
            raise ValueError(f"Route {cargo.destination} has no drive steps")
        self.cargo = cargo
        self.steps = steps
        self.step_index = 0
        self.step_deadline = now + steps[0][1]

    def update(self, now: float) -> bool:
        """Advance expired steps. Return True when the output command changed."""
        changed = False
        while self.active and now >= self.step_deadline:
            self.step_index += 1
            changed = True
            if self.active:
                self.step_deadline += self.steps[self.step_index][1]
        return changed

    def stop(self) -> None:
        self.steps = []
        self.step_index = -1
        self.step_deadline = 0.0


class CameraSource:
    """Use Picamera2 for a CSI camera, or OpenCV for a USB camera."""

    def __init__(self, config: dict):
        self._picamera = None
        self._capture = None
        backend = str(config.get("camera_backend", "auto")).lower()
        width = int(config.get("frame_width", 1280))
        height = int(config.get("frame_height", 720))

        if backend in ("auto", "picamera2"):
            try:
                from picamera2 import Picamera2

                camera = Picamera2()
                camera_config = camera.create_preview_configuration(
                    main={"size": (width, height), "format": "RGB888"},
                    buffer_count=4,
                )
                camera.configure(camera_config)
                camera.start()
                time.sleep(1.0)
                self._picamera = camera
                print("[CAMERA] Picamera2 (CSI camera)")
                return
            except Exception as exc:
                if backend == "picamera2":
                    raise RuntimeError(f"Picamera2 camera initialization failed: {exc}") from exc
                print(f"[CAMERA] Picamera2 unavailable, trying USB camera: {exc}")

        index = int(config.get("camera_index", 0))
        capture = cv2.VideoCapture(index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open USB camera index {index}")
        self._capture = capture
        print(f"[CAMERA] OpenCV USB camera index {index}")

    def read(self):
        if self._picamera is not None:
            return self._picamera.capture_array("main")
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Camera frame capture failed")
        return frame

    def close(self) -> None:
        if self._picamera is not None:
            self._picamera.stop()
            self._picamera.close()
        if self._capture is not None:
            self._capture.release()


class RobotFleet:
    def __init__(self, robot_ips: list[str], command_port: int, status_port: int):
        if len(robot_ips) != 2:
            raise ValueError("config.json robot_ips must contain exactly two ESP32 IP addresses")
        self.addresses = [(ip, command_port) for ip in robot_ips]
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", status_port))
        self.socket.setblocking(False)
        self.last_status: dict[str, tuple[float, str]] = {}

    def send(self, message: str, robot_count: int = 2) -> None:
        if robot_count not in (1, 2):
            raise ValueError(f"robot_count must be 1 or 2, got {robot_count}")
        payload = message.encode("utf-8")
        # config.json order is significant: index 0 is robot 1, index 1 is robot 2.
        for address in self.addresses[:robot_count]:
            self.socket.sendto(payload, address)

    def receive_status(self) -> None:
        while True:
            try:
                payload, address = self.socket.recvfrom(512)
            except BlockingIOError:
                break
            text = payload.decode("utf-8", errors="replace").strip()
            self.last_status[address[0]] = (time.monotonic(), text)

    def online_count(self, now: float, robot_count: int = 2, timeout: float = 1.5) -> int:
        if robot_count not in (1, 2):
            return 0
        expected = {address[0] for address in self.addresses[:robot_count]}
        return sum(
            1
            for ip, (timestamp, _) in self.last_status.items()
            if ip in expected and now - timestamp <= timeout
        )

    def status_fields(
        self, robot_index: int, now: float, timeout: float = 1.5
    ) -> dict[str, str]:
        ip = self.addresses[robot_index][0]
        status = self.last_status.get(ip)
        if status is None or now - status[0] > timeout:
            return {}
        fields: dict[str, str] = {}
        for part in status[1].split(",")[1:]:
            if "=" in part:
                key, value = part.split("=", 1)
                fields[key.strip()] = value.strip()
        return fields

    def safety_alert(self, robot_count: int, now: float) -> str | None:
        for index in range(robot_count):
            fields = self.status_fields(index, now)
            if fields.get("obstacle") != "1":
                continue
            distance = fields.get("distance_cm", "?")
            if index == 0:
                return f"Robot 1 leader obstacle at {distance} cm"
            return f"Robot 2 collision guard at {distance} cm"
        return None

    def emergency_stop(self) -> None:
        for _ in range(5):
            self.send("STOP", 2)
            time.sleep(0.03)

    def close(self) -> None:
        self.socket.close()


class StableQrTrigger:
    """Require repeated frames and emit each shown QR only once."""

    def __init__(self, stable_frames: int, rearm_empty_frames: int):
        self.stable_frames = max(1, stable_frames)
        self.rearm_empty_frames = max(1, rearm_empty_frames)
        self.candidate: Optional[str] = None
        self.candidate_frames = 0
        self.empty_frames = 0
        self.last_emitted: Optional[str] = None

    def update(self, values: list[str]) -> Optional[str]:
        value = values[0].strip() if values else ""
        if not value:
            self.empty_frames += 1
            self.candidate = None
            self.candidate_frames = 0
            if self.empty_frames >= self.rearm_empty_frames:
                self.last_emitted = None
            return None

        self.empty_frames = 0
        if value != self.candidate:
            self.candidate = value
            self.candidate_frames = 1
        else:
            self.candidate_frames += 1

        if self.candidate_frames >= self.stable_frames and value != self.last_emitted:
            self.last_emitted = value
            return value
        return None


def decode_qr(detector, frame) -> tuple[list[str], object]:
    """Read one QR at a time; the four supplied QRs are four alternatives."""
    # Some OpenCV builds occasionally throw an internal convexHull assertion
    # on a malformed camera frame. Treat that frame as unreadable so the
    # controller keeps running and the ESP32 command timeout remains serviced.
    try:
        value, points, _ = detector.detectAndDecode(frame)
    except cv2.error:
        return [], None
    value = value.strip()
    return ([value] if value else []), points


def draw_qr_boxes(frame, points) -> None:
    if points is None:
        return
    polygons = points
    if len(points.shape) == 2:
        polygons = [points]
    for polygon in polygons:
        corners = polygon.astype(int).reshape(-1, 2)
        for index in range(len(corners)):
            start = tuple(corners[index])
            end = tuple(corners[(index + 1) % len(corners)])
            cv2.line(frame, start, end, (0, 255, 0), 3)


def parse_cargo_qr(text: str) -> Optional[CargoInfo]:
    """Parse the actual JSON schema stored in the four supplied QR codes."""
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None

    try:
        destination = str(payload["destination"]).strip().upper()
        cargo_type = str(payload["cargo_type"]).strip().upper()
        weight_kg = float(payload["weight_kg"])
    except (KeyError, TypeError, ValueError):
        return None

    if destination not in {"A", "B", "C", "D"}:
        return None
    if not cargo_type or not 0.0 < weight_kg <= 1000.0:
        return None
    return CargoInfo(destination, cargo_type, weight_kg)


def robot_count_for_cargo(config: dict, cargo_type: str) -> Optional[int]:
    """Return how many robots the Raspberry Pi should command for this cargo."""
    mapping = config.get("cargo_robot_counts", {})
    if not isinstance(mapping, dict):
        return None

    raw_count = mapping.get(cargo_type.strip().upper())
    try:
        count = int(raw_count)
    except (TypeError, ValueError):
        return None
    return count if count in (1, 2) else None


def load_route(config: dict, destination: str) -> list[tuple[Motion, float]]:
    routes = config.get("routes")
    if not isinstance(routes, dict) or destination not in routes:
        raise ValueError(f"No Raspberry Pi drive route configured for destination {destination}")

    steps: list[tuple[Motion, float]] = []
    for index, raw_step in enumerate(routes[destination]):
        try:
            vx = float(raw_step["vx"])
            vy = float(raw_step["vy"])
            w = float(raw_step["w"])
            duration = float(raw_step["duration_sec"])
            state = str(raw_step.get("state", f"{destination}_STEP_{index + 1}"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid route {destination} step {index + 1}") from exc
        if not all(-1.0 <= value <= 1.0 for value in (vx, vy, w)):
            raise ValueError(f"Route {destination} step {index + 1}: vx/vy/w must be -1.0..1.0")
        if not 0.0 < duration <= 300.0:
            raise ValueError(f"Route {destination} step {index + 1}: invalid duration_sec")
        steps.append((Motion(vx, vy, w, state.replace(",", "_")), duration))
    return steps


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = json.load(file)
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing required config value: {key}")
    return config


def ascii_overlay(text: str) -> str:
    return text.encode("ascii", errors="replace").decode("ascii")


def main() -> int:
    default_config = Path(__file__).with_name("config.json")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--headless", action="store_true", help="Do not open an OpenCV window")
    args = parser.parse_args()

    config = load_config(args.config)
    fleet = RobotFleet(
        list(config["robot_ips"]),
        int(config["command_port"]),
        int(config["status_port"]),
    )
    camera = None

    sequence = DriveSequence()
    send_interval = 1.0 / max(1.0, float(config.get("command_hz", 10.0)))
    next_send = 0.0
    active_robot_count = 0

    try:
        camera = CameraSource(config)
        detector = cv2.QRCodeDetector()
        trigger = StableQrTrigger(
            int(config.get("stable_frames", 3)),
            int(config.get("rearm_empty_frames", 5)),
        )

        print(f"[NETWORK] Robots: {', '.join(config['robot_ips'])}")
        print("[READY] Show one cargo QR. Press q or Ctrl+C for emergency stop.")
        fleet.send("PING")

        while True:
            frame = camera.read()
            now = time.monotonic()
            fleet.receive_status()
            online = fleet.online_count(now)

            if sequence.active:
                alert = fleet.safety_alert(active_robot_count, now)
                if alert is not None:
                    print(f"[SAFETY] {alert}; stopping both robots")
                    sequence.stop()
                    active_robot_count = 0
                    fleet.emergency_stop()

            values, points = decode_qr(detector, frame)
            emitted = trigger.update(values)

            if emitted is not None:
                print(f"[QR] {emitted}")
                cargo = parse_cargo_qr(emitted)
                if cargo is None:
                    print("[IGNORE] QR is not a valid cargo JSON payload")
                elif sequence.active:
                    print(f"[IGNORE] Route already running for {sequence.cargo.label()}")
                else:
                    robot_count = robot_count_for_cargo(config, cargo.cargo_type)
                    if robot_count is None:
                        print(f"[IGNORE] Unknown cargo_type: {cargo.cargo_type}")
                        fleet.emergency_stop()
                    elif fleet.online_count(now, robot_count) < robot_count:
                        print(f"[SAFETY] Required robot(s) offline: need {robot_count}")
                        fleet.emergency_stop()
                    elif (alert := fleet.safety_alert(robot_count, now)) is not None:
                        print(f"[SAFETY] Cannot start: {alert}")
                        fleet.emergency_stop()
                    else:
                        active_robot_count = robot_count

                        # Always stop both robots before a new route starts.
                        fleet.send("STOP", 2)
                        route = load_route(config, cargo.destination)
                        sequence.start(cargo, route, now)

                        # Only the selected robot(s) receive motion commands.
                        fleet.send(sequence.packet(), active_robot_count)
                        print(f"[CARGO] {cargo.label()}")
                        print(f"[SELECT] Robots: {active_robot_count}")
                        print(f"[ROUTE] Destination {cargo.destination}: {sequence.label}")

            if sequence.update(now):
                if sequence.active:
                    print(f"[ROUTE] Next step: {sequence.label}")
                else:
                    fleet.emergency_stop()
                    active_robot_count = 0
                    print("[ROUTE] Complete; both robots stopped")

            if now >= next_send:
                if sequence.active:
                    if fleet.online_count(now, active_robot_count) < active_robot_count:
                        print("[SAFETY] Selected robot communication lost; stopping both robots")
                        sequence.stop()
                        active_robot_count = 0
                        fleet.emergency_stop()
                    else:
                        fleet.send(sequence.packet(), active_robot_count)
                else:
                    fleet.send("STOP", 2)
                next_send = now + send_interval

            if not args.headless:
                draw_qr_boxes(frame, points)
                cv2.putText(
                    frame,
                    f"STATE: {ascii_overlay(sequence.label)}  ONLINE: {online}/2  SELECTED: {active_robot_count}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0) if online == 2 else (0, 165, 255),
                    2,
                )
                cv2.imshow("QR Dual Robot Controller", frame)
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
