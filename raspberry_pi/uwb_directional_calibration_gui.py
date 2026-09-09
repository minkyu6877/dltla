#!/usr/bin/env python3
"""Nine-point, four-heading UWB directional range calibration GUI."""

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
from dataclasses import asdict, dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any

from uwb_directional_model import (
    DEFAULT_ANCHORS,
    DirectionalUwbModel,
    build_directional_model,
    center_from_tag,
)


POINTS = (
    ("P1", 1.0, 0.75), ("P2", 2.0, 0.75), ("P3", 3.0, 0.75),
    ("P4", 1.0, 1.50), ("P5", 2.0, 1.50), ("P6", 3.0, 1.50),
    ("P7", 1.0, 2.25), ("P8", 2.0, 2.25), ("P9", 3.0, 2.25),
)
HEADINGS = (0, 90, 180, -90)
STATUS_TIMEOUT_SEC = 1.2
UWB_TIMEOUT_SEC = 0.8


def parse_fields(text: str, prefix: str) -> dict[str, str] | None:
    parts = text.strip().split(",")
    if not parts or parts[0] != prefix:
        return None
    result = {}
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def number(fields: dict[str, str], key: str) -> float | None:
    try:
        result = float(fields[key])
    except (KeyError, ValueError):
        return None
    return result if math.isfinite(result) else None


def vector(fields: dict[str, str], key: str, count: int) -> tuple[float, ...] | None:
    try:
        values = tuple(float(value) for value in fields[key].split(":"))
    except (KeyError, ValueError):
        return None
    if len(values) != count or not all(math.isfinite(value) for value in values):
        return None
    return values


@dataclass(frozen=True)
class Sample:
    monotonic_time: float
    wall_time: str
    tag_x_m: float
    tag_y_m: float
    raw_x_m: float
    raw_y_m: float
    rmse_m: float
    ranges_m: tuple[float, float, float, float]
    imu_raw_yaw_deg: float | None


@dataclass
class Capture:
    point_id: str
    center_x: float
    center_y: float
    heading_deg: int
    samples: list[Sample]

    @property
    def key(self) -> str:
        return f"{self.point_id}:{self.heading_deg}"

    def summary(self) -> dict[str, Any]:
        ranges = [tuple(sample.ranges_m[index] for sample in self.samples) for index in range(4)]
        return {
            "key": self.key,
            "point_id": self.point_id,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "heading_deg": self.heading_deg,
            "sample_count": len(self.samples),
            "tag_x_mean": fmean(sample.tag_x_m for sample in self.samples),
            "tag_y_mean": fmean(sample.tag_y_m for sample in self.samples),
            "rmse_mean": fmean(sample.rmse_m for sample in self.samples),
            "range_means_m": [fmean(values) for values in ranges],
            "range_std_m": [pstdev(values) if len(values) > 1 else 0.0 for values in ranges],
        }

    def to_document(self) -> dict[str, Any]:
        return {
            "point_id": self.point_id,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "heading_deg": self.heading_deg,
            "samples": [asdict(sample) for sample in self.samples],
        }

    @classmethod
    def from_document(cls, document: dict[str, Any]) -> "Capture":
        samples = []
        for item in document["samples"]:
            item = dict(item)
            item["ranges_m"] = tuple(item["ranges_m"])
            samples.append(Sample(**item))
        return cls(
            str(document["point_id"]), float(document["center_x"]), float(document["center_y"]),
            int(document["heading_deg"]), samples,
        )


class DirectionalCalibrationSystem:
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
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.samples: deque[Sample] = deque(maxlen=8000)
        self.captures: dict[str, Capture] = {}
        self.latest_status: tuple[float, dict[str, str]] | None = None
        self.latest_uwb: tuple[float, dict[str, str]] | None = None
        self.capturing_key: str | None = None
        self.results_dir = config_path.parent / "calibration_results"
        self.checkpoint_path = self.results_dir / "uwb_directional_session.json"
        self.model_path = self.results_dir / "uwb_directional_model.json"
        self.last_model: dict[str, Any] | None = None
        anchors = config.get("uwb_anchor_positions_m", DEFAULT_ANCHORS)
        self.anchors = tuple(tuple(float(value) for value in pair) for pair in anchors)
        self.tag_forward = float(config.get("uwb_tag_forward_m", 0.110))
        self.tag_left = float(config.get("uwb_tag_left_m", 0.025))
        self._load_checkpoint()

    def start(self) -> None:
        self._send("STOP")
        self.worker.start()

    def close(self) -> None:
        self.shutdown.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        for _ in range(3):
            self._send("STOP")
        self.status_socket.close()
        self.uwb_socket.close()

    def _send(self, message: str) -> None:
        try:
            self.status_socket.sendto(message.encode(), self.command_address)
        except OSError:
            pass

    def _receive(self) -> None:
        while True:
            try:
                payload, address = self.status_socket.recvfrom(1600)
            except (BlockingIOError, ConnectionResetError):
                break
            fields = parse_fields(payload.decode(errors="replace"), "STATUS")
            if fields is not None and address[0] == self.robot_ip:
                self.latest_status = (time.monotonic(), fields)
        while True:
            try:
                payload, _ = self.uwb_socket.recvfrom(1600)
            except (BlockingIOError, ConnectionResetError):
                break
            fields = parse_fields(payload.decode(errors="replace"), "UWB_POS")
            if fields is None:
                continue
            now = time.monotonic()
            self.latest_uwb = (now, fields)
            tag_x, tag_y = number(fields, "tag_x_m"), number(fields, "tag_y_m")
            raw_x, raw_y = number(fields, "raw_x_m"), number(fields, "raw_y_m")
            rmse, ranges = number(fields, "rmse_m"), vector(fields, "ranges_m", 4)
            if fields.get("quality") != "OK" or None in (tag_x, tag_y, raw_x, raw_y, rmse) or ranges is None:
                continue
            yaw = None
            if self.latest_status is not None and now - self.latest_status[0] <= STATUS_TIMEOUT_SEC:
                attitude = vector(self.latest_status[1], "att_deg", 3)
                if attitude is not None and self.latest_status[1].get("imu_ok") == "1":
                    yaw = attitude[2]
            self.samples.append(
                Sample(now, datetime.now().isoformat(timespec="milliseconds"), float(tag_x), float(tag_y),
                       float(raw_x), float(raw_y), float(rmse), ranges, yaw)
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
            return False, "Robot 1 STATUS가 없습니다."
        if self.latest_status[1].get("imu_ok") != "1":
            return False, "Robot 1 IMU가 준비되지 않았습니다."
        if self.latest_uwb is None or now - self.latest_uwb[0] > UWB_TIMEOUT_SEC:
            return False, "UWB 데이터가 없습니다."
        if self.latest_uwb[1].get("quality") != "OK":
            return False, "UWB quality가 CHECK입니다."
        return True, ""

    def capture(self, point_id: str, heading_deg: int, settle_sec: float, sample_sec: float) -> dict[str, Any]:
        point = next((item for item in POINTS if item[0] == point_id), None)
        if point is None or heading_deg not in HEADINGS:
            raise ValueError("측정 위치 또는 방향이 잘못되었습니다.")
        settle_sec = max(1.0, min(8.0, settle_sec))
        sample_sec = max(5.0, min(20.0, sample_sec))
        key = f"{point_id}:{heading_deg}"
        with self.lock:
            if self.capturing_key is not None:
                raise RuntimeError("다른 위치를 측정 중입니다.")
            ready, message = self._ready()
            if not ready:
                raise RuntimeError(message)
            self.capturing_key = key
        try:
            if self.shutdown.wait(settle_sec):
                raise RuntimeError("프로그램이 종료 중입니다.")
            start = time.monotonic()
            if self.shutdown.wait(sample_sec):
                raise RuntimeError("프로그램이 종료 중입니다.")
            end = time.monotonic()
            with self.lock:
                selected = [sample for sample in self.samples if start <= sample.monotonic_time <= end]
                minimum = max(25, int(sample_sec * 4))
                if len(selected) < minimum:
                    raise RuntimeError(f"유효 샘플 부족: {len(selected)}/{minimum}")
                self.captures[key] = Capture(point_id, point[1], point[2], heading_deg, selected)
                self._save_checkpoint()
                self._save_csv()
                return self.state()
        finally:
            with self.lock:
                self.capturing_key = None

    def _load_checkpoint(self) -> None:
        if not self.checkpoint_path.exists():
            return
        try:
            document = json.loads(self.checkpoint_path.read_text(encoding="utf-8"))
            self.captures = {
                capture.key: capture
                for capture in (Capture.from_document(item) for item in document.get("captures", []))
            }
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.captures = {}

    def _save_checkpoint(self) -> None:
        self.results_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.checkpoint_path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps({"version": 1, "captures": [value.to_document() for value in self.captures.values()]},
                       ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.checkpoint_path)

    def _save_csv(self) -> None:
        path = self.results_dir / "uwb_directional_samples.csv"
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["time", "point_id", "center_x_m", "center_y_m", "heading_deg", "tag_x_m",
                             "tag_y_m", "raw_x_m", "raw_y_m", "rmse_m", "a1_m", "a2_m", "a3_m",
                             "a4_m", "imu_raw_yaw_deg"])
            for capture in self.captures.values():
                for sample in capture.samples:
                    writer.writerow([sample.wall_time, capture.point_id, capture.center_x, capture.center_y,
                                     capture.heading_deg, sample.tag_x_m, sample.tag_y_m, sample.raw_x_m,
                                     sample.raw_y_m, sample.rmse_m, *sample.ranges_m, sample.imu_raw_yaw_deg])

    def reset(self) -> None:
        with self.lock:
            if self.capturing_key is not None:
                raise RuntimeError("측정 중에는 초기화할 수 없습니다.")
            if self.checkpoint_path.exists():
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.checkpoint_path.replace(self.results_dir / f"uwb_directional_session_{stamp}.bak.json")
            self.captures.clear()
            self.last_model = None

    def build_and_apply(self) -> dict[str, Any]:
        with self.lock:
            required = {f"{point[0]}:{heading}" for point in POINTS for heading in HEADINGS}
            missing = sorted(required - set(self.captures))
            if missing:
                raise RuntimeError(f"36개 측정을 완료하세요. 남은 측정: {len(missing)}개")
            summaries = [capture.summary() for capture in self.captures.values()]
            model = build_directional_model(
                summaries, self.anchors,
                self.tag_forward, self.tag_left, 30.0,
            )
            cv_raw_errors: list[float] = []
            cv_corrected_errors: list[float] = []
            for point_id, _, _ in POINTS:
                training = [item for item in summaries if item["point_id"] != point_id]
                validation = [item for item in summaries if item["point_id"] == point_id]
                fold_model = DirectionalUwbModel(build_directional_model(
                    training, self.anchors, self.tag_forward, self.tag_left, 30.0,
                ))
                for item in validation:
                    heading = math.radians(float(item["heading_deg"]))
                    initial_tag = (float(item["tag_x_mean"]), float(item["tag_y_mean"]))
                    raw_center = center_from_tag(
                        initial_tag[0], initial_tag[1], heading, self.tag_forward, self.tag_left
                    )
                    corrected = fold_model.correct(item["range_means_m"], heading, initial_tag)
                    corrected_center = center_from_tag(
                        corrected.tag_x, corrected.tag_y, heading, self.tag_forward, self.tag_left
                    )
                    cv_raw_errors.append(math.hypot(
                        raw_center[0] - float(item["center_x"]),
                        raw_center[1] - float(item["center_y"]),
                    ))
                    cv_corrected_errors.append(math.hypot(
                        corrected_center[0] - float(item["center_x"]),
                        corrected_center[1] - float(item["center_y"]),
                    ))
            cross_validation = {
                "method": "leave_one_position_out",
                "raw_error_mean_m": fmean(cv_raw_errors),
                "raw_error_max_m": max(cv_raw_errors),
                "corrected_error_mean_m": fmean(cv_corrected_errors),
                "corrected_error_max_m": max(cv_corrected_errors),
            }
            safe_to_apply = (
                cross_validation["corrected_error_mean_m"]
                < cross_validation["raw_error_mean_m"] * 0.85
                and cross_validation["corrected_error_mean_m"] <= 0.12
                and cross_validation["corrected_error_max_m"] <= 0.25
            )
            cross_validation["safe_to_apply"] = safe_to_apply
            model["cross_validation_stats"] = cross_validation
            self.results_dir.mkdir(parents=True, exist_ok=True)
            self.model_path.write_text(json.dumps(model, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            self.last_model = model
            if not safe_to_apply:
                return {
                    "model_path": str(self.model_path),
                    "backup": None,
                    "applied": False,
                    "stats": model["training_stats"],
                    "cross_validation": cross_validation,
                    "message": "교차검증 기준을 통과하지 못해 config.json 적용을 차단했습니다.",
                }
            backup = self.config_path.with_name(
                f"config.json.bak_directional_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            )
            shutil.copy2(self.config_path, backup)
            config = json.loads(self.config_path.read_text(encoding="utf-8"))
            config["uwb_directional_enabled"] = True
            config["uwb_directional_model_file"] = str(self.model_path.relative_to(self.config_path.parent))
            config["uwb_directional_max_correction_m"] = 0.45
            self.config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            return {
                "model_path": str(self.model_path), "backup": str(backup), "applied": True,
                "stats": model["training_stats"], "cross_validation": cross_validation,
                "message": "교차검증 통과: config.json에 방향 보정 모델을 적용했습니다.",
            }

    def state(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            status_online = self.latest_status is not None and now - self.latest_status[0] <= STATUS_TIMEOUT_SEC
            uwb_online = self.latest_uwb is not None and now - self.latest_uwb[0] <= UWB_TIMEOUT_SEC
            status = {} if self.latest_status is None else self.latest_status[1]
            uwb = {} if self.latest_uwb is None else self.latest_uwb[1]
            attitude = vector(status, "att_deg", 3)
            summaries = sorted((capture.summary() for capture in self.captures.values()),
                               key=lambda item: (item["point_id"], HEADINGS.index(item["heading_deg"])))
            return {
                "robot_online": status_online,
                "imu_ok": status_online and status.get("imu_ok") == "1",
                "uwb_online": uwb_online,
                "uwb_quality": uwb.get("quality", "-"),
                "tag_x": number(uwb, "tag_x_m"), "tag_y": number(uwb, "tag_y_m"),
                "rmse": number(uwb, "rmse_m"),
                "imu_yaw": None if attitude is None else attitude[2],
                "capturing_key": self.capturing_key,
                "capture_count": len(summaries), "required_count": len(POINTS) * len(HEADINGS),
                "captures": summaries,
                "complete": len(summaries) == len(POINTS) * len(HEADINGS),
                "points": [{"id": p[0], "x": p[1], "y": p[2]} for p in POINTS],
                "model_stats": None if self.last_model is None else self.last_model["training_stats"],
            }


HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>UWB 방향 보정</title><style>
:root{--bg:#07111f;--p:#101d30;--line:#29415d;--text:#eef6ff;--muted:#91a7bf;--green:#35d07f;--red:#ff5d68;--blue:#40a9ff;--orange:#ff9f43}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,"Noto Sans KR",sans-serif}.wrap{max-width:1350px;margin:auto;padding:18px}.top{display:flex;justify-content:space-between;align-items:center;gap:12px}.title{font-size:25px;font-weight:800}.sub{color:var(--muted);font-size:13px}.badges,.actions{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:7px 11px;border-radius:18px;background:#172a42;color:var(--muted)}.ok{background:var(--green);color:#052013}.bad{background:var(--red);color:#fff}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}.card{background:var(--p);border:1px solid var(--line);border-radius:14px;padding:15px}.wide{grid-column:1/-1}.points{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.point,.heading,.btn{border:1px solid #376087;background:#172a42;color:#fff;border-radius:9px;padding:12px;cursor:pointer;font-weight:700}.point.active,.heading.active{background:#1768a7;border-color:#5bb9ff}.point.done{box-shadow:inset 0 0 0 2px var(--green)}.headings{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:12px 0}.heading.done{background:#176746}.measure{width:100%;background:#1768a7;font-size:16px}.danger{background:#8d2b33}.apply{background:#176746}.notice{background:#091626;border-radius:9px;padding:12px;min-height:45px;margin:10px 0}.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.stat{background:#091626;padding:10px;border-radius:9px}.stat b{display:block;font-size:18px;margin-top:4px}.label{color:var(--muted);font-size:12px}.matrix{width:100%;border-collapse:collapse}.matrix th,.matrix td{padding:8px;border-bottom:1px solid #29415d;text-align:center}.cell{display:inline-block;min-width:50px;padding:6px;border-radius:7px;background:#172a42}.cell.done{background:#176746}.progress{height:12px;border-radius:9px;background:#091626;overflow:hidden;margin:10px 0}.bar{height:100%;background:var(--green);width:0}.result{white-space:pre-wrap;font-family:ui-monospace,monospace;background:#091626;padding:12px;border-radius:9px;min-height:100px}@media(max-width:900px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.stats{grid-template-columns:repeat(2,1fr)}}
</style></head><body><div class="wrap"><div class="top"><div><div class="title">UWB 앵커별 방향 보정</div><div class="sub">9개 위치 × 4방향 · Robot 1은 측정 내내 STOP · 중단 후 재실행해도 자동 재개</div></div><div class="badges"><span id="rb" class="badge">R1</span><span id="ib" class="badge">IMU</span><span id="ub" class="badge">UWB</span></div></div>
<div class="grid"><section class="card"><h3>1. 로봇 중심 위치 선택</h3><div id="points" class="points"></div><p class="sub">선택한 좌표에 로봇 중심을 정확히 맞추세요. 태그 위치가 아니라 섀시 중심입니다.</p><h3>2. 로봇 방향 선택</h3><div id="headings" class="headings"></div><div id="selection" class="notice"></div><button id="measure" class="btn measure" onclick="captureNow()">3초 안정화 + 8초 측정</button><div class="stats" style="margin-top:10px"><div class="stat"><span class="label">태그 좌표</span><b id="tag">-</b></div><div class="stat"><span class="label">RMSE</span><b id="rmse">-</b></div><div class="stat"><span class="label">IMU raw yaw</span><b id="yaw">-</b></div><div class="stat"><span class="label">진행률</span><b id="count">0/36</b></div></div></section>
<section class="card"><h3>측정 진행표</h3><div class="progress"><div id="bar" class="bar"></div></div><table class="matrix"><thead><tr><th>위치</th><th>0°</th><th>90°</th><th>180°</th><th>-90°</th></tr></thead><tbody id="matrix"></tbody></table><div class="actions" style="margin-top:12px"><button class="btn danger" onclick="resetAll()">전체 측정 초기화</button></div></section>
<section class="card wide"><h3>모델 생성 및 적용</h3><p class="sub">36개 측정 완료 후 각 앵커의 로봇 상대각별 거리 바이어스를 생성합니다. 추종 GUI는 이 모델이 불안정하면 기존 UWB 좌표로 자동 복귀합니다.</p><button id="apply" class="btn apply" onclick="buildModel()" disabled>방향 보정 모델 생성·config.json 적용</button><div id="notice" class="notice">첫 측정 위치와 방향을 선택하세요.</div><div id="result" class="result">모델 결과 대기 중</div></section></div></div>
<script>
const $=id=>document.getElementById(id);let state=null,point='P1',heading=0,busy=false;const hs=[0,90,180,-90];async function post(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok)throw Error(d.message||'실패');return d}function badge(id,ok,t){let e=$(id);e.textContent=t;e.className='badge '+(ok?'ok':'bad')}function fmt(v,n=3){return v==null?'-':Number(v).toFixed(n)}
function selectPoint(id){point=id;render()}function selectHeading(h){heading=h;render()}async function captureNow(){if(busy)return;busy=true;$('measure').disabled=true;$('notice').textContent=`${point}, ${heading}°: 3초 안정화 후 8초 측정 중. 움직이지 마세요.`;try{await post('/api/capture',{point_id:point,heading_deg:heading,settle_sec:3,sample_sec:8});$('notice').textContent=`${point}, ${heading}° 측정 완료`;await poll();nextMissing()}catch(e){$('notice').textContent=e.message}finally{busy=false;$('measure').disabled=false}}
function nextMissing(){if(!state)return;let done=new Set(state.captures.map(x=>x.key));for(let p of state.points)for(let h of hs)if(!done.has(`${p.id}:${h}`)){point=p.id;heading=h;render();return}}
async function resetAll(){if(!confirm('36개 측정값을 모두 초기화할까요? 기존 체크포인트는 백업됩니다.'))return;try{await post('/api/reset');$('notice').textContent='초기화 완료';point='P1';heading=0;poll()}catch(e){$('notice').textContent=e.message}}
async function buildModel(){if(!confirm('방향 보정 모델을 생성하고 교차검증 후 안전할 때만 적용할까요?'))return;try{let d=await post('/api/model'),s=d.stats,c=d.cross_validation;$('result').textContent=`모델: ${d.model_path}\n학습 기존 평균: ${fmt(s.raw_error_mean_m)} m\n학습 보정 평균: ${fmt(s.corrected_error_mean_m)} m\n교차검증 기존 평균: ${fmt(c.raw_error_mean_m)} m\n교차검증 보정 평균: ${fmt(c.corrected_error_mean_m)} m\n교차검증 보정 최대: ${fmt(c.corrected_error_max_m)} m\n적용 여부: ${d.applied?'적용됨':'차단됨'}\n설정 백업: ${d.backup??'-'}`;$('notice').textContent=d.message}catch(e){$('notice').textContent=e.message}}
function render(){if(!state)return;let done=new Set(state.captures.map(x=>x.key));badge('rb',state.robot_online,'R1 '+(state.robot_online?'ONLINE':'OFFLINE'));badge('ib',state.imu_ok,'IMU '+(state.imu_ok?'OK':'ERROR'));badge('ub',state.uwb_online&&state.uwb_quality==='OK','UWB '+state.uwb_quality);$('points').innerHTML=state.points.map(p=>`<button class="point ${point===p.id?'active':''} ${hs.every(h=>done.has(`${p.id}:${h}`))?'done':''}" onclick="selectPoint('${p.id}')">${p.id}<br><span class="sub">(${p.x}, ${p.y})</span></button>`).join('');$('headings').innerHTML=hs.map(h=>`<button class="heading ${heading===h?'active':''} ${done.has(`${point}:${h}`)?'done':''}" onclick="selectHeading(${h})">${h}°</button>`).join('');let p=state.points.find(x=>x.id===point);$('selection').textContent=`${point}: 중심 (${p.x}, ${p.y}) · 로봇 앞 방향 ${heading}°`;$('tag').textContent=state.tag_x==null?'-':`${fmt(state.tag_x)}, ${fmt(state.tag_y)}`;$('rmse').textContent=fmt(state.rmse)+' m';$('yaw').textContent=fmt(state.imu_yaw,1)+'°';$('count').textContent=`${state.capture_count}/36`;$('bar').style.width=(state.capture_count/36*100)+'%';$('matrix').innerHTML=state.points.map(p=>`<tr><th>${p.id}<br><span class="sub">${p.x},${p.y}</span></th>${hs.map(h=>`<td><span class="cell ${done.has(`${p.id}:${h}`)?'done':''}">${done.has(`${p.id}:${h}`)?'완료':'-'}</span></td>`).join('')}</tr>`).join('');$('apply').disabled=!state.complete;}
async function poll(){try{state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());render()}catch(e){$('notice').textContent='GUI 연결 오류: '+e.message}}setInterval(poll,300);poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    system: DirectionalCalibrationSystem
    def log_message(self, format: str, *args: object) -> None: return
    def body(self) -> dict[str, Any]:
        try: return json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
        except (ValueError, json.JSONDecodeError): return {}
    def reply(self, value: dict[str, Any], status: int = 200) -> None:
        payload = json.dumps(value, ensure_ascii=False).encode(); self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8"); self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(payload)
    def do_GET(self) -> None:
        if self.path == "/":
            payload=HTML.encode(); self.send_response(200); self.send_header("Content-Type","text/html; charset=utf-8")
            self.send_header("Content-Length",str(len(payload))); self.end_headers(); self.wfile.write(payload)
        elif self.path == "/api/state": self.reply(self.system.state())
        else: self.send_error(404)
    def do_POST(self) -> None:
        body=self.body()
        try:
            if self.path == "/api/capture":
                self.reply(self.system.capture(str(body.get("point_id")),int(body.get("heading_deg")),
                                               float(body.get("settle_sec",3)),float(body.get("sample_sec",8))))
            elif self.path == "/api/reset": self.system.reset(); self.reply({"ok":True})
            elif self.path == "/api/model": self.reply({"ok":True,**self.system.build_and_apply()})
            else: self.send_error(404)
        except (OSError,RuntimeError,ValueError) as error: self.reply({"ok":False,"message":str(error)},409)


def load_config(path: Path) -> dict[str, Any]:
    config=json.loads(path.read_text(encoding="utf-8"))
    for key in ("robot_ips","command_port","status_port"):
        if key not in config: raise ValueError(f"Missing config value: {key}")
    return config


def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("--config",type=Path,default=Path(__file__).with_name("config.json")); args=parser.parse_args()
    config=load_config(args.config); system=DirectionalCalibrationSystem(config,args.config); Handler.system=system
    server=ThreadingHTTPServer(("0.0.0.0",int(config.get("uwb_directional_calibration_web_port",8084))),Handler); system.start()
    print(f"UWB directional calibration GUI: http://0.0.0.0:{server.server_port}"); print("Robot 1 is held in STOP while this program is running.")
    try: server.serve_forever()
    finally: server.server_close(); system.close()
    return 0


if __name__ == "__main__":
    try: raise SystemExit(main())
    except KeyboardInterrupt: print("\nUWB directional calibration GUI stopped."); raise SystemExit(0)
    except (OSError,RuntimeError,ValueError,json.JSONDecodeError) as error: print(f"ERROR: {error}"); raise SystemExit(1)
