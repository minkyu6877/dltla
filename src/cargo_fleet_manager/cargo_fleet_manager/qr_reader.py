#!/usr/bin/env python3

import json
import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# 설정
# ============================================================

# 같은 QR 반복 실행 방지
COOLDOWN_SEC = 4.0
RESET_SEC = 1.0

# 5kg 이상이면 2대
WEIGHT_THRESHOLD_KG = 5.0

# 현재 시뮬레이션용 기본 화물 적재 위치
DEFAULT_CARGO_X = 2.0
DEFAULT_CARGO_Y = 0.0

# 기존 QR의 destination A/B/C/D를 좌표로 변환
# 나중에 UWB 실제 좌표에 맞게 숫자만 변경하면 됨
DESTINATION_COORDS = {
    "A": (4.0, 0.0),
    "B": (4.0, 2.0),
    "C": (4.0, -2.0),
    "D": (1.0, 0.0),
}


# ============================================================
# QR decoder
# ============================================================

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    HAS_PYZBAR = True
except Exception:
    pyzbar_decode = None
    HAS_PYZBAR = False


@dataclass
class QrDetection:
    text: str
    points: Optional[List[Tuple[int, int]]]


def detect_qr_with_pyzbar(frame):
    if not HAS_PYZBAR:
        return []

    results = []

    for qr in pyzbar_decode(frame):
        try:
            text = qr.data.decode("utf-8")
        except UnicodeDecodeError:
            text = qr.data.decode(
                "utf-8",
                errors="replace"
            )

        points = None

        if qr.polygon:
            points = [
                (p.x, p.y)
                for p in qr.polygon
            ]

        results.append(
            QrDetection(
                text=text,
                points=points
            )
        )

    return results


def detect_qr_with_opencv(frame, detector):
    detections = []

    # 카메라 프레임 자체가 이상하면 그냥 무시
    if frame is None or frame.size == 0:
        return detections

    # 여러 QR 인식
    try:
        retval, decoded_info, points, _ = detector.detectAndDecodeMulti(frame)

        if retval and points is not None:
            for text, pts in zip(decoded_info, points):
                if not text or pts is None:
                    continue

                try:
                    pt_list = [
                        (int(x), int(y))
                        for x, y in pts.reshape(-1, 2)
                    ]
                except Exception:
                    pt_list = None

                detections.append(
                    QrDetection(
                        text=text,
                        points=pt_list
                    )
                )

            if detections:
                return detections

    except cv2.error as e:
        print(f"[QR WARN] Multi QR frame skip: {e}")
    except Exception as e:
        print(f"[QR WARN] Multi QR error: {e}")

    # 단일 QR 인식
    try:
        text, points, _ = detector.detectAndDecode(frame)

        if text:
            pt_list = None

            if points is not None:
                try:
                    pt_list = [
                        (int(x), int(y))
                        for x, y in points.reshape(-1, 2)
                    ]
                except Exception:
                    pt_list = None

            detections.append(
                QrDetection(
                    text=text,
                    points=pt_list
                )
            )

    except cv2.error as e:
        # OpenCV 내부 convexHull 오류가 나도 노드를 죽이지 않음
        print(f"[QR WARN] QR frame skip: {e}")

    except Exception as e:
        print(f"[QR WARN] QR error: {e}")

    return detections

def detect_qr(frame, detector):

    detections = detect_qr_with_pyzbar(
        frame
    )

    if detections:
        return detections

    return detect_qr_with_opencv(
        frame,
        detector
    )


def draw_qr_box(frame, points):

    if not points:
        return

    for i in range(len(points)):

        p1 = points[i]
        p2 = points[
            (i + 1) % len(points)
        ]

        cv2.line(
            frame,
            p1,
            p2,
            (0, 255, 0),
            3
        )


# ============================================================
# QR 데이터 처리
# ============================================================

def parse_qr_payload(raw_text):

    try:
        data = json.loads(raw_text)

        if isinstance(data, dict):
            return data

    except json.JSONDecodeError:
        pass

    # 단순 문자열 QR도 지원
    text = raw_text.strip().upper()

    return {
        "destination": text,
        "cargo_type": "SMALL_BOX",
        "weight_kg": None,
    }


def decide_cargo_mode(cargo_info):

    cargo_type = str(
        cargo_info.get(
            "cargo_type",
            "SMALL_BOX"
        )
    ).upper()

    weight_raw = cargo_info.get(
        "weight_kg",
        0
    )

    try:
        weight = (
            float(weight_raw)
            if weight_raw is not None
            else 0.0
        )
    except (TypeError, ValueError):
        weight = 0.0

    # Mission Manager가 이해하는
    # small / long 으로 변환
    if cargo_type in (
        "LONG_BOX",
        "WIDE_BOX",
        "HEAVY_BOX",
    ):
        return "long"

    if weight >= WEIGHT_THRESHOLD_KG:
        return "long"

    return "small"


def build_mission(cargo_info):

    cargo_mode = decide_cargo_mode(
        cargo_info
    )

    # QR 안에 cargo_x/y가 있으면 사용
    # 없으면 기본 적재 위치 사용
    cargo_x = float(
        cargo_info.get(
            "cargo_x",
            DEFAULT_CARGO_X
        )
    )

    cargo_y = float(
        cargo_info.get(
            "cargo_y",
            DEFAULT_CARGO_Y
        )
    )

    # QR에 목적지 좌표가 직접 있으면 우선 사용
    if (
        "dest_x" in cargo_info
        and "dest_y" in cargo_info
    ):

        dest_x = float(
            cargo_info["dest_x"]
        )

        dest_y = float(
            cargo_info["dest_y"]
        )

    else:

        destination = str(
            cargo_info.get(
                "destination",
                "A"
            )
        ).upper()

        if destination not in DESTINATION_COORDS:
            raise ValueError(
                f"알 수 없는 목적지: "
                f"{destination}"
            )

        dest_x, dest_y = \
            DESTINATION_COORDS[destination]

    return {
        "cargo_type": cargo_mode,
        "cargo_x": cargo_x,
        "cargo_y": cargo_y,
        "dest_x": dest_x,
        "dest_y": dest_y,
    }


# ============================================================
# ROS 2 QR Reader
# ============================================================

class QrReader(Node):

    def __init__(self):

        super().__init__("qr_reader")

        default_config = os.path.join(
            get_package_share_directory("cargo_fleet_manager"),
            "config", "missions.yaml"
        )
        self.declare_parameter("mission_config", default_config)
        config_path = str(self.get_parameter("mission_config").value)
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        self.valid_mission_ids = {
            str(mission_id).strip().upper()
            for mission_id in config.get("missions", {})
        }
        if not self.valid_mission_ids:
            raise RuntimeError(f"No missions configured in {config_path}")

        self.declare_parameter(
            "camera_index",
            0
        )

        camera_index = int(
            self.get_parameter(
                "camera_index"
            ).value
        )

        self.mission_pub = \
            self.create_publisher(
                String,
                "/fleet/mission",
                10
            )

        # Mission Manager 상태 확인
        self.create_subscription(
            String,
            "/fleet/state",
            self.state_callback,
            10
        )

        self.fleet_state = "UNKNOWN"

        self.detector = \
            cv2.QRCodeDetector()

        self.cap = cv2.VideoCapture(
            camera_index
        )

        if not self.cap.isOpened():
            raise RuntimeError(
                f"카메라 {camera_index}번을 "
                "열 수 없습니다."
            )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            640
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            480
        )

        self.active_text = None
        self.last_detected_time = 0.0
        self.last_action_time = 0.0

        # 카메라 미리보기
        self.gui_enabled = True

        self.timer = self.create_timer(
            0.03,
            self.camera_loop
        )

        self.get_logger().info(
            "QR Reader 시작"
        )

        self.get_logger().info(
            f"Camera index = {camera_index}"
        )

        self.get_logger().info(
            "QR 인식 후 /fleet/mission 으로 전송"
        )

    def state_callback(self, msg):
        self.fleet_state = msg.data

    def camera_loop(self):

        ok, frame = self.cap.read()

        if not ok:
            return

        now = time.monotonic()

        qr_results = detect_qr(
            frame,
            self.detector
        )

        if qr_results:

            self.last_detected_time = now

        elif (
            self.active_text is not None
            and
            now - self.last_detected_time
            > RESET_SEC
        ):

            self.active_text = None

        for detection in qr_results:

            raw_text = detection.text

            draw_qr_box(
                frame,
                detection.points
            )

            cooldown_ok = (
                raw_text != self.active_text
                or
                now - self.last_action_time
                >= COOLDOWN_SEC
            )

            if not cooldown_ok:
                continue

            # 로봇이 준비 위치에서 대기할 때만
            # 새 QR 미션 시작
            if self.fleet_state != "STANDBY":

                cv2.putText(
                    frame,
                    f"BUSY: {self.fleet_state}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                continue

            mission_id = raw_text.strip().upper()
            if mission_id in self.valid_mission_ids:
                command_text = mission_id
            else:
                try:
                    mission = build_mission(parse_qr_payload(raw_text))
                    command_text = json.dumps({
                        "command": "DELIVERY",
                        **mission,
                    })
                except (TypeError, ValueError) as error:
                    self.get_logger().warning(f"잘못된 QR 데이터 {raw_text!r}: {error}")
                    continue

            self.active_text = raw_text
            self.last_action_time = now

            msg = String()

            msg.data = command_text

            self.mission_pub.publish(msg)

            self.get_logger().info(
                "=============================="
            )

            self.get_logger().info(
                f"QR 인식 → {raw_text}"
            )


            self.get_logger().info(
                f"배송 명령 → /fleet/mission 전송 완료"
            )

        # 카메라 미리보기
        if self.gui_enabled:
            try:
                cv2.imshow(
                    "Cargo QR Reader",
                    frame
                )

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    self.get_logger().info(
                        "QR 카메라 창 종료"
                    )
                    self.gui_enabled = False
                    cv2.destroyAllWindows()

            except cv2.error as e:
                self.get_logger().warn(
                    f"카메라 미리보기 비활성화: {e}"
                )
                self.gui_enabled = False

    def destroy_node(self):

        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = QrReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
