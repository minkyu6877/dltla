#!/usr/bin/env python3
"""Browser GUI for Robot 1 UWB tag-to-center calibration."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any


STATUS_TIMEOUT_SEC = 1.2
UWB_TIMEOUT_SEC = 0.8
REQUIRED_HEADINGS = (0, 90, 180, -90)


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


def number(fields: dict[str, str], key: str) -> float | None:
    try:
        value = float(fields[key])
    except (KeyError, ValueError):
        return None
    return value if math.isfinite(value) else None


def vector(fields: dict[str, str], key: str, count: int) -> tuple[float, ...] | None:
    try:
        values = tuple(float(value) for value in fields[key].split(":"))
    except (KeyError, ValueError):
        return None
    if len(values) != count or not all(math.isfinite(value) for value in values):
        return None
    return values


def normalized_heading(value: float) -> int:
    heading = int(round(value))
    while heading > 180:
        heading -= 360
    while heading <= -180:
        heading += 360
    return heading


def circular_mean_deg(values: list[float]) -> float | None:
    if not values:
        return None
    sine = fmean(math.sin(math.radians(value)) for value in values)
    cosine = fmean(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sine, cosine))


def solve_linear(matrix: list[list[float]], values: list[float]) -> list[float]:
    size = len(values)
    augmented = [matrix[row][:] + [values[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            raise ValueError("캘리브레이션 행렬을 풀 수 없습니다. 서로 다른 방향을 측정하세요.")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][index] - factor * augmented[column][index]
                for index in range(size + 1)
            ]
    return [augmented[row][-1] for row in range(size)]


@dataclass(frozen=True)
class Sample:
    timestamp: float
    wall_time: str
    tag_x: float
    tag_y: float
    raw_x: float
    raw_y: float
    rmse: float
    ranges: tuple[float, float, float, float]
    imu_yaw_deg: float | None
    wifi_rssi: float | None


@dataclass
class Capture:
    heading_deg: int
    center_x: float
    center_y: float
    samples: list[Sample]

    def summary(self) -> dict[str, Any]:
        xs = [sample.tag_x for sample in self.samples]
        ys = [sample.tag_y for sample in self.samples]
        rmses = [sample.rmse for sample in self.samples]
        yaws = [sample.imu_yaw_deg for sample in self.samples if sample.imu_yaw_deg is not None]
        return {
            "heading_deg": self.heading_deg,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "sample_count": len(self.samples),
            "tag_x_mean": fmean(xs),
            "tag_y_mean": fmean(ys),
            "tag_x_std": pstdev(xs) if len(xs) > 1 else 0.0,
            "tag_y_std": pstdev(ys) if len(ys) > 1 else 0.0,
            "rmse_mean": fmean(rmses),
            "imu_yaw_mean_deg": circular_mean_deg(yaws),
        }


class CalibrationSystem:
    def __init__(self, config: dict[str, Any], config_path: Path):
        self.config = config
        self.config_path = config_path
        self.robot_ip = str(config["robot_ips"][0])
        self.command_address = (self.robot_ip, int(config["command_port"]))

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
        self.worker = threading.Thread(target=self._run, name="uwb-calibration", daemon=True)
        self.samples: deque[Sample] = deque(maxlen=6000)
        self.captures: dict[int, Capture] = {}
        self.latest_status: tuple[float, dict[str, str]] | None = None
        self.latest_uwb: tuple[float, dict[str, str]] | None = None
        self.capturing_heading: int | None = None
        self.session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_dir = config_path.parent / "calibration_results"
        self.last_saved: dict[str, str] = {}

    def start(self) -> None:
        self._send("STOP")
        self.worker.start()

    def close(self) -> None:
        self.shutdown.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        for _ in range(3):
            self._send("STOP")
            time.sleep(0.02)
        self.status_socket.close()
        self.uwb_socket.close()

    def _send(self, message: str) -> None:
        try:
            self.status_socket.sendto(message.encode("utf-8"), self.command_address)
        except OSError:
            pass

    def _receive(self) -> None:
        while True:
            try:
                payload, address = self.status_socket.recvfrom(1600)
            except (BlockingIOError, ConnectionResetError):
                break
            fields = parse_fields(payload.decode("utf-8", errors="replace"), "STATUS")
            if fields is not None and address[0] == self.robot_ip:
                self.latest_status = (time.monotonic(), fields)

        while True:
            try:
                payload, _ = self.uwb_socket.recvfrom(1600)
            except (BlockingIOError, ConnectionResetError):
                break
            fields = parse_fields(payload.decode("utf-8", errors="replace"), "UWB_POS")
            if fields is None:
                continue
            now = time.monotonic()
            self.latest_uwb = (now, fields)
            tag_x = number(fields, "tag_x_m")
            tag_y = number(fields, "tag_y_m")
            raw_x = number(fields, "raw_x_m")
            raw_y = number(fields, "raw_y_m")
            rmse = number(fields, "rmse_m")
            ranges = vector(fields, "ranges_m", 4)
            if (
                fields.get("quality") != "OK"
                or tag_x is None
                or tag_y is None
                or raw_x is None
                or raw_y is None
                or rmse is None
                or ranges is None
            ):
                continue
            imu_yaw = None
            if self.latest_status is not None and now - self.latest_status[0] <= STATUS_TIMEOUT_SEC:
                attitudes = vector(self.latest_status[1], "att_deg", 3)
                if attitudes is not None and self.latest_status[1].get("imu_ok") == "1":
                    imu_yaw = attitudes[2]
            self.samples.append(
                Sample(
                    now,
                    datetime.now().isoformat(timespec="milliseconds"),
                    tag_x,
                    tag_y,
                    raw_x,
                    raw_y,
                    rmse,
                    ranges,
                    imu_yaw,
                    number(fields, "wifi_rssi"),
                )
            )

    def _run(self) -> None:
        next_keepalive = 0.0
        while not self.shutdown.is_set():
            with self.lock:
                self._receive()
                now = time.monotonic()
                if now >= next_keepalive:
                    self._send("STOP")
                    self._send("PING")
                    next_keepalive = now + 0.5
            time.sleep(0.01)

    def _ready(self) -> tuple[bool, str]:
        now = time.monotonic()
        if self.latest_status is None or now - self.latest_status[0] > STATUS_TIMEOUT_SEC:
            return False, "Robot 1 STATUS가 없습니다. 로봇 전원과 IP를 확인하세요."
        if self.latest_status[1].get("imu_ok") != "1":
            return False, "Robot 1 IMU가 준비되지 않았습니다."
        if self.latest_uwb is None or now - self.latest_uwb[0] > UWB_TIMEOUT_SEC:
            return False, "UWB 데이터가 없습니다. 태그와 앵커 전원을 확인하세요."
        if self.latest_uwb[1].get("quality") != "OK":
            return False, "UWB quality가 CHECK입니다. 앵커 가시선을 확인하세요."
        return True, ""

    def capture(self, heading_deg: float, center_x: float, center_y: float, duration: float) -> dict[str, Any]:
        heading = normalized_heading(heading_deg)
        if heading not in REQUIRED_HEADINGS:
            raise ValueError("방향은 0, 90, 180, -90 중 하나여야 합니다.")
        if not (0.0 <= center_x <= 4.0 and 0.0 <= center_y <= 3.0):
            raise ValueError("중심 좌표는 4m × 3m 앵커 영역 안이어야 합니다.")
        duration = max(3.0, min(15.0, duration))
        with self.lock:
            if self.capturing_heading is not None:
                raise RuntimeError("다른 방향을 측정 중입니다.")
            ready, message = self._ready()
            if not ready:
                raise RuntimeError(message)
            self.capturing_heading = heading
            start = time.monotonic()
        try:
            while time.monotonic() - start < duration:
                if self.shutdown.wait(0.05):
                    raise RuntimeError("프로그램이 종료 중입니다.")
            end = time.monotonic()
            with self.lock:
                selected = [sample for sample in self.samples if start <= sample.timestamp <= end]
                minimum = max(12, int(duration * 5))
                if len(selected) < minimum:
                    raise RuntimeError(f"유효 샘플이 부족합니다: {len(selected)}/{minimum}")
                self.captures[heading] = Capture(heading, center_x, center_y, selected)
                result = self.analysis()
                self._save_results(result)
                return result
        finally:
            with self.lock:
                self.capturing_heading = None

    def reset(self) -> None:
        with self.lock:
            if self.capturing_heading is not None:
                raise RuntimeError("측정 중에는 초기화할 수 없습니다.")
            self.captures.clear()
            self.session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.last_saved = {}

    def analysis(self) -> dict[str, Any]:
        summaries = [self.captures[key].summary() for key in REQUIRED_HEADINGS if key in self.captures]
        document: dict[str, Any] = {
            "capture_count": len(summaries),
            "required_count": len(REQUIRED_HEADINGS),
            "captures": summaries,
            "complete": all(key in self.captures for key in REQUIRED_HEADINGS),
            "fit": None,
        }
        if len(self.captures) < 2:
            return document

        normal = [[0.0] * 4 for _ in range(4)]
        right = [0.0] * 4
        all_rows: list[tuple[list[float], float]] = []
        for capture in self.captures.values():
            angle = math.radians(capture.heading_deg)
            cosine, sine = math.cos(angle), math.sin(angle)
            for sample in capture.samples:
                rows = (
                    ([cosine, -sine, 1.0, 0.0], sample.tag_x - capture.center_x),
                    ([sine, cosine, 0.0, 1.0], sample.tag_y - capture.center_y),
                )
                all_rows.extend(rows)
                for row, observed in rows:
                    for i in range(4):
                        right[i] += row[i] * observed
                        for j in range(4):
                            normal[i][j] += row[i] * row[j]
        forward, left, bias_x, bias_y = solve_linear(normal, right)

        direction_errors = []
        all_error_m = []
        for key in REQUIRED_HEADINGS:
            if key not in self.captures:
                continue
            capture = self.captures[key]
            angle = math.radians(capture.heading_deg)
            cosine, sine = math.cos(angle), math.sin(angle)
            errors_x: list[float] = []
            errors_y: list[float] = []
            for sample in capture.samples:
                corrected_tag_x = sample.tag_x - bias_x
                corrected_tag_y = sample.tag_y - bias_y
                center_est_x = corrected_tag_x - forward * cosine + left * sine
                center_est_y = corrected_tag_y - forward * sine - left * cosine
                error_x = center_est_x - capture.center_x
                error_y = center_est_y - capture.center_y
                errors_x.append(error_x)
                errors_y.append(error_y)
                all_error_m.append(math.hypot(error_x, error_y))
            direction_errors.append(
                {
                    "heading_deg": capture.heading_deg,
                    "error_x_mean": fmean(errors_x),
                    "error_y_mean": fmean(errors_y),
                    "error_m_mean": fmean(math.hypot(x, y) for x, y in zip(errors_x, errors_y)),
                }
            )
        fit = {
            "tag_forward_m": forward,
            "tag_left_m": left,
            "uwb_bias_x_m": bias_x,
            "uwb_bias_y_m": bias_y,
            "center_error_mean_m": fmean(all_error_m),
            "center_error_max_m": max(all_error_m),
            "direction_errors": direction_errors,
            "recommended_config": {
                "uwb_tag_forward_m": round(forward, 5),
                "uwb_tag_left_m": round(left, 5),
                "uwb_bias_x_m": round(bias_x, 5),
                "uwb_bias_y_m": round(bias_y, 5),
            },
        }
        document["fit"] = fit
        return document

    def _save_results(self, result: dict[str, Any]) -> None:
        self.results_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.results_dir / f"uwb_center_{self.session_stamp}_samples.csv"
        json_path = self.results_dir / f"uwb_center_{self.session_stamp}_result.json"
        with csv_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "time", "heading_deg", "center_x_m", "center_y_m", "tag_x_m", "tag_y_m",
                    "raw_x_m", "raw_y_m", "rmse_m", "range_a1_m", "range_a2_m",
                    "range_a3_m", "range_a4_m", "imu_raw_yaw_deg", "wifi_rssi",
                ]
            )
            for key in REQUIRED_HEADINGS:
                capture = self.captures.get(key)
                if capture is None:
                    continue
                for sample in capture.samples:
                    writer.writerow(
                        [
                            sample.wall_time, capture.heading_deg, capture.center_x, capture.center_y,
                            sample.tag_x, sample.tag_y, sample.raw_x, sample.raw_y, sample.rmse,
                            *sample.ranges, sample.imu_yaw_deg, sample.wifi_rssi,
                        ]
                    )
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.last_saved = {"csv": str(csv_path), "json": str(json_path)}

    def apply_to_config(self) -> dict[str, Any]:
        with self.lock:
            result = self.analysis()
            if not result["complete"] or result["fit"] is None:
                raise RuntimeError("네 방향 측정을 모두 완료해야 적용할 수 있습니다.")
            recommended = result["fit"]["recommended_config"]
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup = self.config_path.with_name(f"config.json.bak_uwb_center_{stamp}")
            shutil.copy2(self.config_path, backup)
            config = json.loads(self.config_path.read_text(encoding="utf-8"))
            config.update(recommended)
            self.config_path.write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            return {"backup": str(backup), "applied": recommended}

    def state(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            status_online = self.latest_status is not None and now - self.latest_status[0] <= STATUS_TIMEOUT_SEC
            uwb_online = self.latest_uwb is not None and now - self.latest_uwb[0] <= UWB_TIMEOUT_SEC
            uwb_fields = {} if self.latest_uwb is None else self.latest_uwb[1]
            status_fields = {} if self.latest_status is None else self.latest_status[1]
            tag_x = number(uwb_fields, "tag_x_m")
            tag_y = number(uwb_fields, "tag_y_m")
            attitudes = vector(status_fields, "att_deg", 3)
            return {
                "robot_online": status_online,
                "imu_ok": status_online and status_fields.get("imu_ok") == "1",
                "uwb_online": uwb_online,
                "uwb_quality": uwb_fields.get("quality", "-"),
                "tag_x": tag_x,
                "tag_y": tag_y,
                "rmse": number(uwb_fields, "rmse_m"),
                "imu_raw_yaw_deg": None if attitudes is None else attitudes[2],
                "capturing_heading": self.capturing_heading,
                "analysis": self.analysis(),
                "saved": self.last_saved,
            }


HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UWB 중심점 캘리브레이션</title><style>
:root{--bg:#07111f;--panel:#101d30;--line:#29415d;--text:#edf5ff;--muted:#90a7c0;--blue:#40a9ff;--green:#35d07f;--red:#ff5d68;--yellow:#ffc857}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,"Noto Sans KR",sans-serif}.wrap{max-width:1320px;margin:auto;padding:18px}.top{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:14px}.title{font-size:25px;font-weight:800}.sub,.label{color:var(--muted);font-size:13px}.badges{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:7px 11px;border-radius:18px;background:#172a42;color:var(--muted);font-size:13px}.ok{background:var(--green);color:#062012}.bad{background:var(--red);color:#fff}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:15px}.wide{grid-column:1/-1}.inputs{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin:12px 0}input{width:100%;margin-top:5px;padding:10px;background:#091626;border:1px solid #35516e;border-radius:8px;color:#fff}.headings{display:grid;grid-template-columns:repeat(4,1fr);gap:9px}.heading{padding:14px 8px;border:1px solid #376087;border-radius:10px;background:#172a42;color:#fff;font-weight:750;cursor:pointer}.heading.done{background:#176746;border-color:#35d07f}.heading:disabled{opacity:.45}.notice{margin:12px 0;padding:11px;background:#091626;border-radius:9px;min-height:44px}.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:9px}.stat{padding:10px;background:#091626;border-radius:9px}.stat b{display:block;font-size:19px;margin-top:4px}table{width:100%;border-collapse:collapse;margin-top:10px;font-size:13px}th,td{border-bottom:1px solid #263e58;padding:8px;text-align:right}th:first-child,td:first-child{text-align:left}.actions{display:flex;gap:9px;flex-wrap:wrap;margin-top:12px}.btn{padding:11px 14px;border:1px solid #376087;border-radius:9px;background:#172a42;color:#fff;cursor:pointer;font-weight:700}.primary{background:#1768a7}.danger{background:#8d2b33}.apply{background:#176746}.result{font-family:ui-monospace,monospace;white-space:pre-wrap;background:#091626;border-radius:9px;padding:12px;min-height:160px;line-height:1.55}canvas{width:100%;height:auto;background:#091626;border-radius:10px}.warn{color:var(--yellow)}@media(max-width:900px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.headings,.stats{grid-template-columns:repeat(2,1fr)}}
</style></head><body><div class="wrap"><div class="top"><div><div class="title">UWB 중심점 캘리브레이션</div><div class="sub">Robot 1은 항상 STOP · 네 방향의 태그 위치로 장착 오프셋과 좌표 바이어스 계산</div></div><div class="badges"><span id="rb" class="badge">R1</span><span id="ib" class="badge">IMU</span><span id="ub" class="badge">UWB</span></div></div>
<div class="grid"><section class="card"><h3>1. 기준 위치와 측정</h3><div class="inputs"><label class="label">로봇 중심 X (m)<input id="cx" type="number" value="2.0" min="0" max="4" step="0.01"></label><label class="label">로봇 중심 Y (m)<input id="cy" type="number" value="1.5" min="0" max="3" step="0.01"></label><label class="label">측정 시간 (초)<input id="duration" type="number" value="5" min="3" max="15" step="1"></label></div><div class="headings"><button class="heading" data-h="0">0°<br><span class="sub">+X 방향</span></button><button class="heading" data-h="90">90°<br><span class="sub">+Y 방향</span></button><button class="heading" data-h="180">180°<br><span class="sub">−X 방향</span></button><button class="heading" data-h="-90">−90°<br><span class="sub">−Y 방향</span></button></div><div id="notice" class="notice">로봇 중심을 (2.0, 1.5)에 고정하고 첫 방향을 맞추세요.</div><div class="stats"><div class="stat"><span class="label">태그 좌표</span><b id="tag">-</b></div><div class="stat"><span class="label">UWB RMSE</span><b id="rmse">-</b></div><div class="stat"><span class="label">IMU raw yaw</span><b id="yaw">-</b></div><div class="stat"><span class="label">완료 방향</span><b id="count">0 / 4</b></div></div><div class="actions"><button class="btn danger" onclick="resetSession()">측정 초기화</button></div></section>
<section class="card"><canvas id="map" width="620" height="470"></canvas><p class="sub">십자: 입력한 로봇 중심 · 점: 각 방향에서 측정된 UWB 태그 평균</p></section>
<section class="card wide"><h3>2. 계산 결과</h3><table><thead><tr><th>방향</th><th>샘플</th><th>태그 X 평균</th><th>태그 Y 평균</th><th>X 표준편차</th><th>Y 표준편차</th><th>RMSE 평균</th></tr></thead><tbody id="rows"></tbody></table><div class="grid" style="margin-top:14px"><div id="result" class="result">네 방향 중 두 방향 이상을 측정하면 중간 계산값이 표시됩니다.</div><div><p class="sub">네 방향이 모두 끝난 뒤 결과를 설정에 적용할 수 있습니다. 적용 전 config.json은 자동 백업됩니다.</p><button id="apply" class="btn apply" onclick="applyConfig()" disabled>config.json에 적용</button><div id="saved" class="notice">결과 파일 대기 중</div><p class="sub warn">방향별 잔여 오차가 5cm 이상이면 숫자 보정보다 UWB 안테나 높이·차폐를 먼저 개선하세요.</p></div></div></section></div></div>
<script>
const $=id=>document.getElementById(id),canvas=$('map'),ctx=canvas.getContext('2d');let state=null,busy=false;function badge(id,ok,text){let e=$(id);e.textContent=text;e.className='badge '+(ok?'ok':'bad')}async function post(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok)throw Error(d.message||'요청 실패');return d}function fmt(v,n=3){return v==null?'-':Number(v).toFixed(n)}
document.querySelectorAll('.heading').forEach(b=>b.onclick=async()=>{if(busy)return;busy=true;disable(true);$('notice').textContent=`${b.dataset.h}° 방향을 측정 중입니다. 로봇을 움직이지 마세요.`;try{await post('/api/capture',{heading_deg:+b.dataset.h,center_x:+$('cx').value,center_y:+$('cy').value,duration_sec:+$('duration').value});$('notice').textContent=`${b.dataset.h}° 측정 완료`;}catch(e){$('notice').textContent=e.message}finally{busy=false;disable(false);poll()}});function disable(v){document.querySelectorAll('.heading').forEach(b=>b.disabled=v)}
async function resetSession(){if(!confirm('현재 측정값을 모두 지울까요?'))return;try{await post('/api/reset');$('notice').textContent='측정값을 초기화했습니다.'}catch(e){$('notice').textContent=e.message}}
async function applyConfig(){if(!confirm('계산값을 config.json에 적용할까요? 기존 파일은 백업됩니다.'))return;try{let d=await post('/api/apply');$('notice').textContent='설정 적용 완료. formation GUI를 재시작하세요.';$('saved').textContent='백업: '+d.backup}catch(e){$('notice').textContent=e.message}}
function draw(){ctx.fillStyle='#091626';ctx.fillRect(0,0,canvas.width,canvas.height);let m=42,map=(x,y)=>[m+x/4*(canvas.width-2*m),canvas.height-m-y/3*(canvas.height-2*m)];ctx.strokeStyle='#233e59';for(let x=0;x<=4;x++){let a=map(x,0),b=map(x,3);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}for(let y=0;y<=3;y++){let a=map(0,y),b=map(4,y);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke()}let c=map(+$('cx').value,+$('cy').value);ctx.strokeStyle='#35d07f';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(c[0]-10,c[1]);ctx.lineTo(c[0]+10,c[1]);ctx.moveTo(c[0],c[1]-10);ctx.lineTo(c[0],c[1]+10);ctx.stroke();if(!state)return;let colors={'0':'#40a9ff','90':'#35d07f','180':'#ff9f43','-90':'#d26cff'};state.analysis.captures.forEach(x=>{let p=map(x.tag_x_mean,x.tag_y_mean);ctx.fillStyle=colors[x.heading_deg]||'#fff';ctx.beginPath();ctx.arc(...p,7,0,7);ctx.fill();ctx.fillStyle='#fff';ctx.font='13px sans-serif';ctx.fillText(x.heading_deg+'°',p[0]+9,p[1]-8)})}
function render(){let a=state.analysis;badge('rb',state.robot_online,'R1 '+(state.robot_online?'ONLINE':'OFFLINE'));badge('ib',state.imu_ok,'IMU '+(state.imu_ok?'OK':'ERROR'));badge('ub',state.uwb_online&&state.uwb_quality==='OK','UWB '+state.uwb_quality);$('tag').textContent=state.tag_x==null?'-':fmt(state.tag_x)+', '+fmt(state.tag_y);$('rmse').textContent=fmt(state.rmse)+' m';$('yaw').textContent=fmt(state.imu_raw_yaw_deg,1)+'°';$('count').textContent=a.capture_count+' / 4';let done=new Set(a.captures.map(x=>String(x.heading_deg)));document.querySelectorAll('.heading').forEach(b=>b.classList.toggle('done',done.has(b.dataset.h)));$('rows').innerHTML=a.captures.map(x=>`<tr><td>${x.heading_deg}°</td><td>${x.sample_count}</td><td>${fmt(x.tag_x_mean)}</td><td>${fmt(x.tag_y_mean)}</td><td>${fmt(x.tag_x_std)}</td><td>${fmt(x.tag_y_std)}</td><td>${fmt(x.rmse_mean)}</td></tr>`).join('');if(a.fit){let f=a.fit;$('result').textContent=`태그 앞쪽 오프셋 : ${fmt(f.tag_forward_m)} m\n태그 왼쪽 오프셋 : ${fmt(f.tag_left_m)} m\nUWB X 바이어스    : ${fmt(f.uwb_bias_x_m)} m\nUWB Y 바이어스    : ${fmt(f.uwb_bias_y_m)} m\n중심 평균 오차    : ${fmt(f.center_error_mean_m)} m\n중심 최대 오차    : ${fmt(f.center_error_max_m)} m`}$('apply').disabled=!a.complete;$('saved').textContent=state.saved.json?`CSV: ${state.saved.csv}\nJSON: ${state.saved.json}`:'결과 파일 대기 중';draw()}
async function poll(){try{state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());render()}catch(e){$('notice').textContent='GUI 연결 오류: '+e.message}}setInterval(poll,250);['cx','cy'].forEach(id=>$(id).oninput=draw);poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    system: CalibrationSystem

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
            if self.path == "/api/capture":
                result = self.system.capture(
                    float(body.get("heading_deg", 0.0)),
                    float(body.get("center_x", 2.0)),
                    float(body.get("center_y", 1.5)),
                    float(body.get("duration_sec", 5.0)),
                )
                self._reply_json({"ok": True, "analysis": result})
            elif self.path == "/api/reset":
                self.system.reset()
                self._reply_json({"ok": True})
            elif self.path == "/api/apply":
                self._reply_json({"ok": True, **self.system.apply_to_config()})
            else:
                self.send_error(404)
        except (OSError, RuntimeError, ValueError) as error:
            self._reply_json({"ok": False, "message": str(error)}, 409)


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    for key in ("robot_ips", "command_port", "status_port"):
        if key not in config:
            raise ValueError(f"Missing config value: {key}")
    if not isinstance(config["robot_ips"], list) or not config["robot_ips"]:
        raise ValueError("config.json robot_ips must contain Robot 1 address")
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    args = parser.parse_args()
    config = load_config(args.config)
    system = CalibrationSystem(config, args.config)
    Handler.system = system
    port = int(config.get("uwb_calibration_web_port", 8083))
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    system.start()
    print(f"UWB center calibration GUI: http://0.0.0.0:{server.server_port}")
    print("Robot 1 is held in STOP while this program is running.")
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
        print("\nUWB center calibration GUI stopped.")
        raise SystemExit(0)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
