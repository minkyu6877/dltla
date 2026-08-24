#!/usr/bin/env python3
"""Guided RPM, straight-line, and orbit experiments with automatic logging."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

from robot_control_gui import RobotCore, load_config


WHEELS = ("FL", "FR", "RL", "RR")


def parse_vector(fields: dict[str, str], key: str) -> tuple[float, float, float, float] | None:
    try:
        values = tuple(float(value) for value in fields[key].split(":"))
    except (KeyError, ValueError):
        return None
    return values if len(values) == 4 else None  # type: ignore[return-value]


def average(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


class ExperimentManager:
    def __init__(self, core: RobotCore, config: dict, result_dir: Path):
        self.core = core
        self.config = config
        self.result_dir = result_dir
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.wheelbase_cm = float(config.get("mecanum_wheelbase_cm", 19.4))
        if self.wheelbase_cm <= 0.0:
            raise ValueError("mecanum_wheelbase_cm must be positive")
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.finish_requested = threading.Event()
        self.worker: threading.Thread | None = None
        self.mode = "IDLE"
        self.message = "실험을 선택하세요."
        self.progress = 0.0
        self.started_at = 0.0
        self.last_heartbeat = time.monotonic()
        self.last_result: dict[str, Any] | None = None
        self.json_path: Path | None = None
        self.csv_path: Path | None = None

    def heartbeat(self) -> None:
        with self.lock:
            self.last_heartbeat = time.monotonic()

    def busy(self) -> bool:
        return self.worker is not None and self.worker.is_alive()

    def set_state(self, mode: str, message: str, progress: float | None = None) -> None:
        with self.lock:
            self.mode = mode
            self.message = message
            if progress is not None:
                self.progress = max(0.0, min(1.0, progress))

    def launch(self, mode: str, function, *args) -> tuple[bool, str]:
        with self.lock:
            if self.busy():
                return False, "다른 실험이 진행 중입니다."
            self.cancel.clear()
            self.finish_requested.clear()
            self.mode = mode
            self.message = "실험 준비 중"
            self.progress = 0.0
            self.started_at = time.monotonic()
            self.last_heartbeat = time.monotonic()
            self.last_result = None
            self.json_path = None
            self.csv_path = None
            self.worker = threading.Thread(target=self.guarded, args=(function, *args), daemon=True)
            self.worker.start()
        return True, "실험을 시작했습니다."

    def guarded(self, function, *args) -> None:
        try:
            function(*args)
        except Exception as exc:
            self.core.stop("EXPERIMENT ERROR")
            self.set_state("ERROR", f"실험 오류: {exc}")

    def can_continue(self) -> bool:
        if self.cancel.is_set():
            return False
        with self.lock:
            browser_lost = time.monotonic() - self.last_heartbeat > 2.0
        if browser_lost:
            self.cancel.set()
            self.core.stop("BROWSER LOST")
            self.set_state("STOPPED", "브라우저 연결이 끊겨 자동정지했습니다.")
            return False
        return True

    def start_rpm(self, robot: int) -> tuple[bool, str]:
        return (False, "로봇 번호가 잘못되었습니다.") if robot not in (0, 1) else self.launch("RPM", self.run_rpm, robot)

    def start_straight(self, robot: int, speed: float) -> tuple[bool, str]:
        if robot not in (0, 1):
            return False, "로봇 번호가 잘못되었습니다."
        return self.launch("STRAIGHT", self.run_straight, robot, max(0.05, min(0.25, speed)))

    def start_orbit(self, direction: str, linear: float, rotation: float) -> tuple[bool, str]:
        if direction not in ("u", "i"):
            return False, "공전 방향이 잘못되었습니다."
        return self.launch(
            "ORBIT", self.run_orbit, direction,
            max(0.05, min(0.25, linear)), max(0.05, min(0.25, rotation)),
        )

    def finish(self) -> tuple[bool, str]:
        if not self.busy() or self.mode not in ("STRAIGHT", "ORBIT"):
            return False, "종료할 직진/공전 실험이 없습니다."
        self.finish_requested.set()
        self.set_state(self.mode, "측정을 종료하고 결과를 계산 중입니다.")
        return True, "측정을 종료합니다."

    def emergency_stop(self) -> None:
        self.cancel.set()
        self.finish_requested.set()
        self.core.stop("EXPERIMENT EMERGENCY STOP")
        self.set_state("STOPPED", "비상정지했습니다.")

    def new_fields(self, robot: int, last_timestamp: float) -> tuple[float, dict[str, str]] | None:
        with self.core.lock:
            snapshot = self.core.snapshots.get(self.core.robot_ips[robot])
            if snapshot is None or snapshot[0] <= last_timestamp:
                return None
            return snapshot[0], dict(snapshot[1])

    def keep_moving(self, key: str, selected: tuple[int, ...], linear: float, rotation: float) -> None:
        ok, reason = self.core.move(key, selected, linear, rotation)
        if not ok:
            raise RuntimeError(reason)

    def run_rpm(self, robot: int) -> None:
        speeds = [float(value) for value in self.config.get("experiment_rpm_speeds", [0.10, 0.15, 0.20])]
        warmup = float(self.config.get("experiment_rpm_warmup_sec", 1.0))
        sample_time = float(self.config.get("experiment_rpm_sample_sec", 3.0))
        rest = float(self.config.get("experiment_rest_sec", 1.0))
        rows: list[dict[str, Any]] = []

        for step, speed in enumerate(speeds):
            self.core.stop("RPM STEP")
            phase_start = time.monotonic()
            while time.monotonic() - phase_start < warmup:
                if not self.can_continue():
                    return
                self.keep_moving("w", (robot,), speed, 0.10)
                self.set_state("RPM", f"Robot {robot + 1} · 속도 {speed:.2f} 워밍업")
                time.sleep(0.10)

            samples: list[tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]] = []
            last_timestamp = 0.0
            phase_start = time.monotonic()
            while time.monotonic() - phase_start < sample_time:
                if not self.can_continue():
                    return
                self.keep_moving("w", (robot,), speed, 0.10)
                fresh = self.new_fields(robot, last_timestamp)
                if fresh is not None:
                    last_timestamp, fields = fresh
                    target = parse_vector(fields, "target_rpm")
                    actual = parse_vector(fields, "rpm")
                    pwm = parse_vector(fields, "drive_pwm")
                    if target and actual and pwm:
                        samples.append((target, actual, pwm))
                elapsed = time.monotonic() - phase_start
                self.set_state("RPM", f"Robot {robot + 1} · 속도 {speed:.2f} 측정 {elapsed:.1f}/{sample_time:.1f}초", (step + elapsed / sample_time) / len(speeds))
                time.sleep(0.10)

            self.core.stop("RPM STEP COMPLETE")
            for wheel_index, wheel in enumerate(WHEELS):
                targets = [sample[0][wheel_index] for sample in samples]
                actuals = [sample[1][wheel_index] for sample in samples]
                pwms = [sample[2][wheel_index] for sample in samples]
                rows.append({
                    "robot": robot + 1, "speed_command": speed, "wheel": wheel,
                    "sample_count": len(actuals), "target_rpm_mean": average(targets),
                    "actual_rpm_mean": average(actuals),
                    "actual_rpm_stdev": statistics.pstdev(actuals) if len(actuals) > 1 else 0.0 if actuals else None,
                    "rpm_error_mean": average([target - actual for target, actual in zip(targets, actuals)]),
                    "pwm_mean": average(pwms),
                })
            rest_deadline = time.monotonic() + rest
            while time.monotonic() < rest_deadline:
                if not self.can_continue():
                    return
                time.sleep(0.10)

        result = {"type": "rpm", "robot": robot + 1, "rows": rows}
        self.save_result(result, rows)
        self.set_state("COMPLETE", f"Robot {robot + 1} RPM 시험 완료", 1.0)

    def run_straight(self, robot: int, speed: float) -> None:
        max_time = float(self.config.get("experiment_max_straight_sec", 30.0))
        start = time.monotonic()
        last_timestamp = 0.0
        samples: list[dict[str, str]] = []
        while not self.finish_requested.is_set() and time.monotonic() - start < max_time:
            if not self.can_continue():
                return
            self.keep_moving("w", (robot,), speed, 0.10)
            fresh = self.new_fields(robot, last_timestamp)
            if fresh is not None:
                last_timestamp, fields = fresh
                samples.append(fields)
            elapsed = time.monotonic() - start
            self.set_state("STRAIGHT", f"Robot {robot + 1} 직진 중 · 1m 선에서 종료 버튼을 누르세요 · {elapsed:.1f}초", min(0.99, elapsed / max_time))
            time.sleep(0.10)
        self.core.stop("STRAIGHT COMPLETE")
        result = self.summarize_straight(robot, speed, time.monotonic() - start, samples)
        self.save_result(result, self.metric_rows(result))
        self.set_state("COMPLETE", "직진 시험 완료 · 실측 오차를 아래에 입력하세요.", 1.0)

    def summarize_straight(self, robot: int, speed: float, duration: float, samples: list[dict[str, str]]) -> dict[str, Any]:
        result: dict[str, Any] = {
            "type": "straight", "robot": robot + 1, "speed_command": speed,
            "duration_sec": duration, "sample_count": len(samples), "manual_measurements": {},
        }
        for field_name in ("target_rpm", "rpm", "drive_pwm"):
            vectors = [value for fields in samples if (value := parse_vector(fields, field_name)) is not None]
            for index, wheel in enumerate(WHEELS):
                values = [value[index] for value in vectors]
                result[f"{field_name}_{wheel}_mean"] = average(values)
                result[f"{field_name}_{wheel}_stdev"] = statistics.pstdev(values) if len(values) > 1 else 0.0 if values else None
        yaw_values = []
        for fields in samples:
            try:
                yaw_values.append(float(fields["att_deg"].split(":")[2]))
            except (KeyError, ValueError, IndexError):
                pass
        result["imu_yaw_start_deg"] = yaw_values[0] if yaw_values else None
        result["imu_yaw_end_deg"] = yaw_values[-1] if yaw_values else None
        result["imu_yaw_change_deg"] = (
            (yaw_values[-1] - yaw_values[0] + 180.0) % 360.0 - 180.0
            if len(yaw_values) > 1 else None
        )
        return result

    def run_orbit(self, direction: str, linear: float, rotation: float) -> None:
        max_time = float(self.config.get("experiment_max_orbit_sec", 60.0))
        start = time.monotonic()
        samples: list[dict[str, Any]] = []
        while not self.finish_requested.is_set() and time.monotonic() - start < max_time:
            if not self.can_continue():
                return
            self.keep_moving(direction, (0, 1), linear, rotation)
            orbit = self.core.status().get("orbit", {})
            if orbit.get("gap_cm") is not None:
                samples.append(dict(orbit))
            elapsed = time.monotonic() - start
            self.set_state("ORBIT", f"공전 중 · 한 바퀴 후 종료 버튼을 누르세요 · {elapsed:.1f}초", min(0.99, elapsed / max_time))
            time.sleep(0.10)
        self.core.stop("ORBIT COMPLETE")
        result: dict[str, Any] = {
            "type": "orbit", "direction": "CCW" if direction == "u" else "CW",
            "linear_limit": linear, "rotation_request": rotation,
            "duration_sec": time.monotonic() - start, "sample_count": len(samples),
            "manual_measurements": {},
        }
        metrics = ("gap_cm", "target_gap_cm", "center_radius_cm", "yaw_rpm_robot1", "yaw_rpm_robot2", "yaw_rpm_error", "yaw_correction", "radial_command", "rotation_command")
        for metric in metrics:
            values = [float(sample[metric]) for sample in samples if sample.get(metric) is not None]
            result[f"{metric}_min"] = min(values) if values else None
            result[f"{metric}_mean"] = average(values)
            result[f"{metric}_max"] = max(values) if values else None
        self.save_result(result, self.metric_rows(result))
        self.set_state("COMPLETE", "공전 시험 완료 · 최종 위치 오차를 아래에 입력하세요.", 1.0)

    @staticmethod
    def metric_rows(result: dict[str, Any]) -> list[dict[str, Any]]:
        return [{"metric": key, "value": value} for key, value in result.items() if not isinstance(value, (dict, list))]

    def save_result(self, result: dict[str, Any], rows: list[dict[str, Any]]) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = f"{stamp}_{result['type']}"
        json_path = self.result_dir / f"{stem}.json"
        csv_path = self.result_dir / f"{stem}.csv"
        result["created_at"] = datetime.now().isoformat(timespec="seconds")
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        if rows:
            keys = list(dict.fromkeys(key for row in rows for key in row))
            with csv_path.open("w", newline="", encoding="utf-8-sig") as file:
                writer = csv.DictWriter(file, fieldnames=keys)
                writer.writeheader()
                writer.writerows(rows)
        with self.lock:
            self.last_result = result
            self.json_path = json_path
            self.csv_path = csv_path

    def annotate(self, body: dict[str, Any]) -> tuple[bool, str]:
        with self.lock:
            if self.last_result is None or self.json_path is None:
                return False, "최근 실험 결과가 없습니다."
            allowed = (
                "actual_distance_cm", "lateral_error_cm", "heading_error_deg",
                "front_offset_cm", "rear_offset_cm", "final_gap_error_cm", "notes",
            )
            manual = self.last_result.setdefault("manual_measurements", {})
            for key in allowed:
                if body.get(key) is not None:
                    manual[key] = body[key]
            try:
                front = float(manual["front_offset_cm"])
                rear = float(manual["rear_offset_cm"])
            except (KeyError, TypeError, ValueError):
                pass
            else:
                manual["lateral_error_cm"] = (front + rear) / 2.0
                manual["heading_error_deg"] = math.degrees(
                    math.atan2(front - rear, self.wheelbase_cm)
                )
                manual["heading_formula"] = (
                    f"atan2(front-rear, {self.wheelbase_cm:g}cm)"
                )
            self.json_path.write_text(json.dumps(self.last_result, ensure_ascii=False, indent=2), encoding="utf-8")
        return True, "실측값을 저장했습니다."

    def state(self) -> dict[str, Any]:
        with self.lock:
            return {
                "mode": self.mode, "busy": self.busy(), "message": self.message,
                "progress": self.progress, "last_result": self.last_result,
                "json_file": self.json_path.name if self.json_path else None,
                "csv_file": self.csv_path.name if self.csv_path else None,
                "robot": self.core.status(),
            }


HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot Experiment Lab</title><style>
:root{--bg:#08111e;--card:#111e2f;--line:#293b52;--text:#edf4fc;--muted:#95a9c0;--green:#3bd6a0;--red:#ff6374}*{box-sizing:border-box}body{margin:0;background:linear-gradient(145deg,#13253d,var(--bg) 42%);color:var(--text);font-family:system-ui,"Noto Sans KR",sans-serif}.wrap{max-width:1250px;margin:auto;padding:22px}.top,.row{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}h1{font-size:25px;margin:0}h2{font-size:18px;margin:0 0 12px}.muted{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-top:15px}.card{background:rgba(17,30,47,.97);border:1px solid var(--line);border-radius:15px;padding:17px}.robots{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}.rpms{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:10px}.metric{background:#091625;border-radius:9px;padding:8px}.label{font-size:11px;color:var(--muted)}.value{font-weight:750}.online{color:var(--green)}.offline{color:var(--red)}button{border:1px solid #3a5879;background:#17304c;color:var(--text);border-radius:10px;padding:11px 13px;font-weight:750;cursor:pointer}.danger{background:#8a2635;border-color:#c24b5c;font-size:17px}.finish{background:#765316}input,select,textarea{width:100%;border:1px solid #334a64;background:#091625;color:var(--text);border-radius:9px;padding:10px;margin:5px 0 10px}.progress{height:11px;background:#07101c;border-radius:20px;overflow:hidden;margin:10px 0}.progress div{height:100%;background:linear-gradient(90deg,#4ca6ff,var(--green));width:0}.status{padding:13px;border-radius:11px;background:#091625;font-weight:700}.wide{margin-top:14px}.files a{color:#78bdff;margin-right:14px}pre{max-height:260px;overflow:auto;background:#07111d;border-radius:10px;padding:12px;color:#bdd0e5}@media(max-width:900px){.grid{grid-template-columns:1fr}.robots{grid-template-columns:1fr}.wrap{padding:13px}}</style></head><body><div class="wrap">
<div class="top"><div><h1>Robot Experiment Lab</h1><div class="muted">RPM · 1m 직진 · 공전 자동 기록</div></div><button class="danger" onclick="stopAll()">두 로봇 비상정지</button></div><div id="robots" class="robots"></div><div class="grid">
<div class="card"><h2>3. RPM 자동 시험</h2><p class="muted">바퀴를 띄우세요. config.json에 지정한 속도를 순서대로 자동 측정합니다.</p><div class="row"><button onclick="start({type:'rpm',robot:0})">Robot 1</button><button onclick="start({type:'rpm',robot:1})">Robot 2</button></div></div>
<div class="card"><h2>4. 1m 직진 시험</h2><select id="sr"><option value="0">Robot 1</option><option value="1">Robot 2</option></select><input id="ss" type="number" value="0.10" min="0.05" max="0.25" step="0.01"><button onclick="startStraight()">직진 시작</button> <button class="finish" onclick="finish()">1m 도착·종료</button></div>
<div class="card"><h2>5. 공전 시험</h2><select id="od"><option value="u">반시계</option><option value="i">시계</option></select><div class="row"><input id="ol" type="number" value="0.10" min="0.05" max="0.25" step="0.01"><input id="or" type="number" value="0.10" min="0.05" max="0.25" step="0.01"></div><button onclick="startOrbit()">공전 시작</button> <button class="finish" onclick="finish()">한 바퀴·종료</button></div></div>
<div class="card wide"><div class="row"><h2>진행 상태</h2><strong id="mode">IDLE</strong></div><div class="progress"><div id="bar"></div></div><div id="msg" class="status">준비 중</div></div>
<div class="card wide"><h2>실측값 추가 · 각도기 불필요</h2><p class="muted">기준선 오른쪽은 +, 왼쪽은 -로 입력하세요. 앞·뒤 차축 오차로 중심 좌우 오차와 각도를 자동 계산합니다.</p><div class="grid"><div><label>실제 이동거리 cm</label><input id="ad" type="number" step="0.1"><label>앞차축 중심 좌우 오차 cm</label><input id="fo" type="number" step="0.1"></div><div><label>뒤차축 중심 좌우 오차 cm</label><input id="ro" type="number" step="0.1"><label>최종 초음파 간격 오차 cm</label><input id="ge" type="number" step="0.1"></div><div><label>메모</label><textarea id="notes" rows="4"></textarea><button onclick="saveNotes()">각도 자동 계산·저장</button></div></div></div>
<div class="card wide"><div class="row"><h2>최근 결과</h2><div id="files" class="files"></div></div><pre id="result">아직 결과가 없습니다.</pre></div></div><script>
const $=id=>document.getElementById(id);function vec(v){let a=String(v||'').split(':');return a.length===4?a:['-','-','-','-']}function card(r){let f=r.fields||{},p=vec(f.rpm);return `<div class="card"><div class="row"><h2>Robot ${r.number}</h2><strong class="${r.online?'online':'offline'}">${r.online?'ONLINE':'OFFLINE'}</strong></div><div class="muted">${r.ip} · ${f.state||'-'}</div><div class="rpms">${['FL','FR','RL','RR'].map((w,i)=>`<div class="metric"><div class="label">${w} RPM</div><div class="value">${p[i]}</div></div>`).join('')}</div><div class="row" style="margin-top:9px"><span>초음파 ${f.distance_cm||'-'}cm</span><span>장애물 ${f.obstacle==='1'?'STOP':'CLEAR'}</span></div></div>`}async function api(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok)throw Error(d.message);return d}async function start(b){try{await api('/api/start',b)}catch(e){alert(e.message)}}function startStraight(){start({type:'straight',robot:Number($('sr').value),speed:Number($('ss').value)})}function startOrbit(){start({type:'orbit',direction:$('od').value,linear:Number($('ol').value),rotation:Number($('or').value)})}async function finish(){try{await api('/api/finish')}catch(e){alert(e.message)}}async function stopAll(){await api('/api/stop')}function num(id){return $(id).value===''?null:Number($(id).value)}async function saveNotes(){try{await api('/api/annotate',{actual_distance_cm:num('ad'),lateral_error_cm:num('le'),heading_error_deg:num('he'),final_gap_error_cm:num('ge'),notes:$('notes').value});alert('저장 완료')}catch(e){alert(e.message)}}async function poll(){try{let d=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());$('robots').innerHTML=d.robot.robots.map(card).join('');$('mode').textContent=d.mode;$('msg').textContent=d.message;$('bar').style.width=(d.progress*100)+'%';$('result').textContent=d.last_result?JSON.stringify(d.last_result,null,2):'아직 결과가 없습니다.';let f='';if(d.json_file)f+=`<a href="/results/${encodeURIComponent(d.json_file)}">JSON 다운로드</a>`;if(d.csv_file)f+=`<a href="/results/${encodeURIComponent(d.csv_file)}">CSV 다운로드</a>`;$('files').innerHTML=f}catch(e){$('msg').textContent='연결 오류'}}setInterval(poll,300);poll();window.addEventListener('beforeunload',()=>navigator.sendBeacon('/api/stop','{}'));
</script><script>
async function saveNotes(){try{await api('/api/annotate',{actual_distance_cm:num('ad'),front_offset_cm:num('fo'),rear_offset_cm:num('ro'),final_gap_error_cm:num('ge'),notes:$('notes').value});alert('중심 오차와 각도를 계산해 저장했습니다.')}catch(e){alert(e.message)}}
</script></body></html>"""


def make_handler(manager: ExperimentManager):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, status: int, value: Any) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def body(self) -> dict[str, Any]:
            length = min(int(self.headers.get("Content-Length", "0")), 8192)
            raw = self.rfile.read(length)
            return json.loads(raw.decode("utf-8")) if raw else {}

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                payload = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            elif path == "/api/status":
                manager.heartbeat()
                self.send_json(200, manager.state())
            elif path.startswith("/results/"):
                name = Path(unquote(path[len("/results/"):])).name
                candidate = manager.result_dir / name
                if not candidate.is_file() or candidate.parent.resolve() != manager.result_dir.resolve():
                    self.send_json(404, {"message": "파일을 찾을 수 없습니다."})
                    return
                payload = candidate.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/json" if candidate.suffix == ".json" else "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="{candidate.name}"')
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            else:
                self.send_json(404, {"message": "Not found"})

        def do_POST(self) -> None:
            try:
                body = self.body()
                if self.path == "/api/start":
                    kind = body.get("type")
                    if kind == "rpm":
                        ok, message = manager.start_rpm(int(body.get("robot", -1)))
                    elif kind == "straight":
                        ok, message = manager.start_straight(int(body.get("robot", -1)), float(body.get("speed", 0.10)))
                    elif kind == "orbit":
                        ok, message = manager.start_orbit(str(body.get("direction", "")), float(body.get("linear", 0.10)), float(body.get("rotation", 0.10)))
                    else:
                        ok, message = False, "알 수 없는 실험입니다."
                    self.send_json(200 if ok else 409, {"ok": ok, "message": message})
                elif self.path == "/api/finish":
                    ok, message = manager.finish()
                    self.send_json(200 if ok else 409, {"ok": ok, "message": message})
                elif self.path == "/api/stop":
                    manager.emergency_stop()
                    self.send_json(200, {"ok": True})
                elif self.path == "/api/annotate":
                    ok, message = manager.annotate(body)
                    self.send_json(200 if ok else 409, {"ok": ok, "message": message})
                else:
                    self.send_json(404, {"message": "Not found"})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.send_json(400, {"message": str(exc)})

        def log_message(self, format: str, *args: object) -> None:
            return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    port = int(args.port if args.port is not None else config.get("experiment_web_port", 8081))
    core = RobotCore(config)
    manager = ExperimentManager(core, config, Path(__file__).with_name("experiment_results"))
    server = ThreadingHTTPServer((args.host, port), make_handler(manager))
    server.daemon_threads = True
    try:
        core.start()
        print(f"Experiment GUI ready: http://<Raspberry-Pi-IP>:{port}")
        print("Keep this terminal open. Exit with Ctrl+C.")
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nSTOP: both robots stopped.")
    finally:
        manager.emergency_stop()
        server.server_close()
        core.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
