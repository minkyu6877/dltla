#!/usr/bin/env python3
"""Cargo demonstration: approach -> 5-second loading wait -> L route -> left exit."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time
from http.server import ThreadingHTTPServer

import qr_auto_roundtrip_gui as base
from formation_control_gui import Motion, Pose, clamp, wrap_radians
from route_precision import leg_errors, headings_settled
from cargo_mission_geometry import ANCHORS, display_uwb

START_X, START_Y = .6, 0.0
CENTER_GAP = .53
NOMINAL_ULTRASONIC_GAP_CM = 30.0  # Physical setup guide; keep measured-start gap control.
LOADING_GAP_TOLERANCE_CM = 10.0
MISSION_REVISION = '2026-09-02-cargo-qr-count-gui-v12'
LOADING_WAIT_SECONDS = 5.0
TURN_COAST_SECONDS = 0.0
TURN_CAPTURE_DEG = 0.0  # Stop when R1's measured IMU yaw reaches/passes the goal.
TURN_COAST_MAX_DEG = 0.0
TURN_ALIGNMENT_ENTRY_MAX_DEG = 12.0
R2_POST_ORBIT_RIGHT_SPEED = 0.20
R2_POST_ORBIT_RIGHT_SECONDS = 1.00  # About 20 cm; open-loop because the wheels slip.
APPROACH = base.line_points((START_X,START_Y), (0,0), math.pi/2, 'APPROACH_LOAD')
TRANSPORT_Y = base.line_points((0,0), (0,3), math.pi/2, 'CARGO_Y')
TRANSPORT_X = base.line_points((0,3), (3.6,3), 0, 'CARGO_X')

# Unloaded APPROACH_LOAD only. User measured R1 x=.40, R2 x=.35;
# 20260830_202228.csv APPROACH_LOAD_STOPPED recorded x=.00849 / -.03991.
# Correct displacement units, not the goal coordinates or the wheel RPM PID.
APPROACH_ODOMETRY_SCALES = (1.60 / (2-.00849), 1.65 / (2+.03991))
APPROACH_R2_COMMAND_SCALE = 1.60 / 1.65
STRAFE_PHASES = frozenset(('APPROACH_LOAD','EXIT_LEFT'))


def required_robots_for_qr(config, text):
    """Return the configured 1/2 robot display for one cargo QR."""
    try:
        payload = json.loads(text)
        cargo_type = str(payload['cargo_type']).strip().upper()
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    normalized = cargo_type.replace(' ', '_').replace('-', '_')
    mapping = config.get('cargo_robot_counts', {})
    if isinstance(mapping, dict):
        normalized_mapping = {
            str(key).strip().upper().replace(' ', '_').replace('-', '_'): value
            for key, value in mapping.items()
        }
        try:
            count = int(normalized_mapping.get(normalized))
        except (TypeError, ValueError):
            count = None
        if count in (1, 2):
            return count
    if 'SMALL' in normalized:
        return 1
    if any(name in normalized for name in ('LONG', 'WIDE', 'HEAVY', 'LARGE', 'BIG')):
        return 2
    return None


def turn_exit_probe(headings, rates, target, direction):
    """R1 measured yaw ends the orbit; no encoder arc or coast prediction.

    Keep rates in the call signature for callers, but completion uses yaw only.
    R2 finishes residual chassis yaw with the existing stationary alignment.
    """
    if direction not in (-1,1):
        raise ValueError('공전 방향은 -1 또는 +1이어야 합니다.')
    if (len(headings)!=2 or
            any(v is None or not math.isfinite(v) for v in (*headings,target))):
        raise RuntimeError('공전 종료 판단에 필요한 두 로봇 IMU 방향 없음')
    remaining=[math.degrees(direction*wrap_radians(target-h)) for h in headings]
    return dict(controller='R1_IMU_YAW_ONLY',remaining_deg=remaining,
                target_heading_deg=math.degrees(target),
                brake_robot=1 if remaining[0]<=TURN_CAPTURE_DEG else None)


class MissionSystem(base.RouteFormationSystem):
    """Opt-in holonomic follower. Old forward-only GUI remains unchanged."""

    def __init__(self, config):
        self.approach_strafe_scales = tuple(float(v) for v in config.get(
            'cargo_approach_strafe_odometry_scales', APPROACH_ODOMETRY_SCALES))
        if (len(self.approach_strafe_scales)!=2 or not all(
                math.isfinite(v) and .5<=v<=1.5 for v in self.approach_strafe_scales)):
            raise ValueError('cargo_approach_strafe_odometry_scales: two values in [0.5,1.5] required')
        self.approach_r2_command_scale = float(config.get(
            'cargo_approach_r2_strafe_command_scale', APPROACH_R2_COMMAND_SCALE))
        if not math.isfinite(self.approach_r2_command_scale) or not .8<=self.approach_r2_command_scale<=1.2:
            raise ValueError('cargo_approach_r2_strafe_command_scale: value in [0.8,1.2] required')
        self.translation_phase = None
        self.orbit_progress_telemetry = {}
        self.robot2_nudge_motion = None
        super().__init__(config)
        self.follow_distance.continue_on_target_loss = True
        self.orbit_controller.orbit_distance.continue_on_target_loss = True

    def set_translation_phase(self, label):
        with self.lock:
            self.translation_phase = label

    def _body_velocity(self, index, rpms):
        vx,vy = super()._body_velocity(index,rpms)
        # Restore the measured v2 profile only where it was measured: unloaded
        # approach. The later loaded exit has no measured lateral multiplier.
        if self.translation_phase == 'APPROACH_LOAD':
            vy *= self.approach_strafe_scales[index]
        return vx,vy

    def _fuse_leader_uwb(self, now):
        self.fusion_status = 'DISPLAY_ONLY'  # Hard separation, even if config says true.

    def set_orbit(self, enabled, direction='CCW', radius=.30, speed=.15):
        with self.lock:
            ok,message=super().set_orbit(enabled,direction,radius,speed)
            if ok and enabled:
                self.orbit_progress_telemetry={}
            return ok,message

    # Inherit the calibrated distance/yaw/RPM orbit controller unchanged.
    # Do not integrate wheel travel into orbital phase or hold R2 on that
    # estimate: wheel slip makes that position estimate unreliable.

    def _calculate_follower_motion(self):
        if self.robot2_nudge_motion is not None:
            if time.monotonic() > self.route_deadline:
                return None
            self.follow_telemetry.update(
                controller='R2_POST_ORBIT_RIGHT_NUDGE',
                forward_command=self.robot2_nudge_motion.vx,
                lateral_command=self.robot2_nudge_motion.vy,
                rotation_command=self.robot2_nudge_motion.w)
            return self.robot2_nudge_motion
        follower = super()._calculate_follower_motion()
        if follower is None or self.alignment_target is not None:
            return follower
        h1, h2 = self._heading(0), self._heading(1)
        yaw_error = wrap_radians(h1-h2)
        raw = self.follow_distance.raw
        soft_guard = (1.0 if self.follow_distance.state == 'TARGET_LOST' and raw is None
                      else clamp((raw-(self.follow_distance.minimum+6))/4, 0, 1))
        leader = self.leader_motion
        if max(abs(leader.vx),abs(leader.vy),abs(leader.w)) < 1e-6:
            return follower
        forward = leader.vx*math.cos(yaw_error)-leader.vy*math.sin(yaw_error)
        left = leader.vx*math.sin(yaw_error)+leader.vy*math.cos(yaw_error)
        trim = self.follow_distance.command
        forward = clamp(forward+trim,-self.max_command,self.max_command)
        command_scale = self.approach_r2_command_scale if self.translation_phase=='APPROACH_LOAD' else 1.0
        left = clamp(left*command_scale,-self.max_command,self.max_command)
        # Reverse/strafe must not leave R1 moving toward a stalled R2. Slow
        # BOTH translations on raw near range; <=18cm still stops both.
        self.leader_motion = Motion(leader.vx*soft_guard,leader.vy*soft_guard,
                                     leader.w,leader.name)
        self.follow_telemetry.update(controller='MISSION_HOLONOMIC',
                                    strafe_command_scale=command_scale,
                                    compensation_applied=True,
                                    forward_command=forward*soft_guard,
                                    lateral_command=left*soft_guard,
                                    rotation_command=follower.w)
        return Motion(forward*soft_guard,left*soft_guard,follower.w,'MISSION_FOLLOW')

    def state(self):
        with self.lock:
            result = super().state()
            heading = self._heading(0) if self._status_fresh(0,time.monotonic()) else None
            result['uwb_display'] = display_uwb(self.uwb,time.monotonic(),heading,
                                               self.tag_forward,self.tag_left)
            result['calibration'].update(mission_revision=MISSION_REVISION,
                turn_exit_controller='R1_IMU_YAW_ONLY',
                turn_capture_deg=TURN_CAPTURE_DEG,turn_coast_seconds=TURN_COAST_SECONDS,
                turn_alignment_entry_max_deg=TURN_ALIGNMENT_ENTRY_MAX_DEG,
                orbit_encoder_progress_enabled=False,
                r2_post_orbit_right_speed=R2_POST_ORBIT_RIGHT_SPEED,
                r2_post_orbit_right_seconds=R2_POST_ORBIT_RIGHT_SECONDS,
                r2_range_loss_policy='CONTINUE_WITHOUT_DISTANCE_CORRECTION',
                strafe_compensation_enabled=True,
                strafe_odometry_scales=self.approach_strafe_scales,
                strafe_r2_command_scale=self.approach_r2_command_scale,
                strafe_calibration_scope='Measured lateral multipliers: APPROACH_LOAD only. Path/heading/range correction restored for APPROACH_LOAD + EXIT_LEFT; transport/orbit unchanged.',
                translation_phase=self.translation_phase)
            result['orbit_progress']=dict(self.orbit_progress_telemetry)
            return result


class CargoMissionApp(base.AutoFormationApp):
    def __init__(self, config, system_factory=None):
        super().__init__(config, system_factory=system_factory or MissionSystem)
        self.return_enabled = False
        self.log_prefix = 'cargo_mission'
        self.exit_distance = config.get('cargo_exit_distance_m')
        if self.exit_distance is not None:
            self.exit_distance = self._exit_value(self.exit_distance)
        self.loading_gap_cm = None  # Capture ultrasound at measured 53cm center spacing.
        self.loading_token = None
        self.loading_deadline = None
        self.required_robot_count = None
        self.turn_exit = {}
        self.orbit_log_file = None
        self.system.start_x, self.system.start_y = START_X,START_Y
        self.system.start_heading = math.pi/2
        self.system.gap = CENTER_GAP
        self.system.leader = Pose(START_X,START_Y,math.pi/2)
        self.system.follower = Pose(START_X,START_Y-CENTER_GAP,math.pi/2)
        self.route_label = f'시작 R1 ({START_X:g},0), R2 ({START_X:g},-{CENTER_GAP:g}) / +y · 퇴장 거리를 설정하세요.'

    def _open_log(self):
        super()._open_log()
        # Keep the existing CSV schema/tools compatible; save the exact profile
        # alongside each run so the next real-distance correction is repeatable.
        metadata = dict(calibration=self.system.state()['calibration'],
            start_positions_m=[[START_X,START_Y],[START_X,START_Y-CENTER_GAP]],
            approach_distance_m=START_X,
            center_gap_m=CENTER_GAP,nominal_ultrasonic_gap_cm=NOMINAL_ULTRASONIC_GAP_CM,
            loading_gap_tolerance_cm=LOADING_GAP_TOLERANCE_CM,
            loading_wait_seconds=LOADING_WAIT_SECONDS,
            scope='Measured strafe multipliers: APPROACH_LOAD only. APPROACH_LOAD + EXIT_LEFT use path/heading/range correction. Transport/orbit retain calibration.',
            calibration_reference=dict(log='cargo_mission_20260830_202228.csv',
                actual_displacement_m=[1.60,1.65],
                odometry_displacement_m=[2-.00849,2+.03991]))
        Path(self.log_path).with_suffix('.calibration.json').write_text(
            json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        self.orbit_log_file=Path(self.log_path).with_suffix('.orbit.jsonl').open(
            'w',encoding='utf-8',buffering=1)

    def _write_log(self, stage):
        super()._write_log(stage)
        if self.orbit_log_file is not None:
            with self.system.lock:
                progress=dict(getattr(self.system,'orbit_progress_telemetry',{}))
            self.orbit_log_file.write(json.dumps(dict(
                elapsed_sec=time.monotonic()-self.log_started_at,stage=stage,
                progress=progress,turn_exit=self.turn_exit),ensure_ascii=False)+'\n')

    def _close_log(self):
        try:
            super()._close_log()
        finally:
            if self.orbit_log_file is not None:
                self.orbit_log_file.close()
                self.orbit_log_file=None

    @staticmethod
    def _exit_value(value):
        value = float(value)
        if not math.isfinite(value) or not .2 <= value <= 2.0:
            raise ValueError('퇴장 거리는 0.2~2.0m로 입력하세요.')
        return value

    def set_setup(self, exit_distance):
        distance = self._exit_value(exit_distance)
        with self.lock:
            if self.running or (self.route_thread and self.route_thread.is_alive()):
                raise ValueError('주행 중에는 시나리오를 변경할 수 없습니다.')
            self.exit_distance = distance
            self.route_label = '설정 완료 · QR 또는 시나리오 시작 버튼을 누르세요.'

    def set_return(self, enabled):
        raise ValueError('이 시나리오는 퇴장 후 정지하며 자동 복귀하지 않습니다.')

    def trigger_route(self, source):
        with self.lock:
            if self.exit_distance is None:
                return False,'먼저 퇴장 거리 설정을 적용하세요.'
            if self.running or (self.route_thread and self.route_thread.is_alive()):
                return False,'이미 시나리오가 진행 중입니다.'
            self.loading_token = None
            self.loading_deadline = None
            return super().trigger_route(source)

    def confirm_loaded(self, token):
        # Old browser tabs must not bypass the countdown via the retired API.
        return False,'적재 버튼은 사용하지 않습니다. 적재 위치에서 5초 후 자동 출발합니다.'

    def _handle_qr(self, value):
        if value:
            count = required_robots_for_qr(self.config, value)
            with self.lock:
                self.required_robot_count = count
        super()._handle_qr(value)

    def _wait_for_reset(self, timeout=6):
        with self.system.lock:
            self.system.follow_target_gap_cm = None  # Require a fresh start gap each run.
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self._check_abort()
            with self.system.lock:
                if all(self.system._status_fresh(i,time.monotonic()) for i in (0,1)):
                    gap=self._range_cm()
                    offset=CENTER_GAP*100-gap
                    if not 8 <= offset <= 35:
                        return False,f'중심 간격 {CENTER_GAP*100:g}cm 배치 및 초음파 조준을 확인하세요.'
                    self.loading_gap_cm=self.start_gap_cm=gap
                    self.system.gap=CENTER_GAP
                    self.system.follow_target_gap_cm=gap
                    # Measured center spacing calibrates the range-to-center
                    # offset for THIS run only. Wheel geometry/RPM unchanged.
                    self.config['orbit_center_offset_cm']=offset
                    self.system.orbit_controller.orbit_center_offset_cm=offset
                    return self.system.reset_pose()
            time.sleep(.1)
        return False,'두 로봇 센서 상태 대기 시간 초과'

    def _range_cm(self):
        # Initial placement still requires a real measurement. Once started,
        # a lost R2 echo must not abort loading/turn/straight transitions.
        target = self.system.follow_target_gap_cm
        if target is None:
            return super()._range_cm()
        try:
            gap = float(self.system._fields(1).get('distance_cm'))
        except (TypeError, ValueError):
            gap = None
        guard = self.system.follow_distance
        if gap is not None and math.isfinite(gap) and gap <= guard.minimum:
            raise RuntimeError(f'R2 초음파 근접 안전정지: {gap:g}cm')
        if (gap is None or not math.isfinite(gap) or gap > guard.maximum
                or abs(gap-target) > guard.max_error):
            # A control reference only: raw sensor/GUI data is not overwritten.
            return target
        return gap

    def _check_active(self, *, turning=False):
        # Only skip the R2 target-distance excursion abort. Sensor freshness,
        # IMU, near obstacles and manual STOP remain checked by the base class.
        return super()._check_active(turning=False)

    def _check_stopped_ready(self):
        self._check_abort()
        with self.system.lock:
            now=time.monotonic()
            if self.system.follow_enabled or not self.system.pose_reset:
                raise RuntimeError('적재 대기 중 제어 상태 변경: 재시작 필요')
            for i in (0,1):
                h=self.system._heading(i)
                if not self.system._status_fresh(i,now) or h is None or not math.isfinite(h):
                    raise RuntimeError('자동 출발 중단: 두 로봇 센서 연결을 확인한 뒤 재시작하세요.')
                if self.system._fields(i).get('obstacle')=='1':
                    raise RuntimeError('자동 출발 중단: 주변 사람/장애물을 비운 뒤 재시작하세요.')
            gap=self._range_cm()
            if abs(gap-self.loading_gap_cm)>LOADING_GAP_TOLERANCE_CM+1e-9:
                raise RuntimeError(f'적재 출발 간격 초과: 시작 {self.loading_gap_cm:.1f}cm / 현재 {gap:.1f}cm '
                                   f'(허용 변화 ±{LOADING_GAP_TOLERANCE_CM:g}cm). 재배치 후 재시작 필요')
            hs=[self.system._heading(i) for i in (0,1)]
            if any(abs(wrap_radians(h-math.pi/2))>math.radians(3) for h in hs):
                raise RuntimeError('적재 중 방향 변경: 안전을 위해 출발하지 않습니다.')
            rates=[self.system._yaw_rate(i) for i in (0,1)]
            if any(r is None or not math.isfinite(r) or abs(r)>math.radians(2) for r in rates):
                raise RuntimeError('자동 출발 중단: 로봇이 안정적으로 정지한 뒤 재시작하세요.')

    def _wait_loading(self):
        self._stop_formation()
        deadline = time.monotonic() + LOADING_WAIT_SECONDS
        with self.lock:
            self.loading_token = None
            self.loading_deadline = deadline
            self.route_state='WAIT_LOADING'
            self.route_label='적재 대기 · 5초 후 자동 출발 · 준비가 안 됐으면 STOP'
        next_log=0
        while time.monotonic() < deadline:
            self._check_abort()
            if self.system.follow_enabled:
                raise RuntimeError('적재 대기 중 제어 모드 변경')
            now=time.monotonic()
            with self.lock:
                self.route_label=f'적재 대기 · {max(0,math.ceil(deadline-now))}초 후 자동 출발 · 준비가 안 됐으면 STOP'
            if now>=next_log:
                self._write_log('WAIT_LOADING')
                next_log=now+1
            time.sleep(.1)
        with self.lock:
            self.loading_deadline = None
        self._check_stopped_ready()
        # Don't re-zero position/yaw after a person handles the cargo.
        with self.system.lock:
            hs=[self.system._heading(i) for i in (0,1)]
            if any(abs(wrap_radians(h-math.pi/2))>math.radians(3) for h in hs):
                raise RuntimeError('적재 중 방향 변경: 안전을 위해 출발하지 않습니다.')

    def _follow_path(self, points, phase_label, progress_start=0, progress_end=1):
        # Path tangent and chassis heading are DIFFERENT for strafe/diagonal.
        first,final=points[0],points[-1]
        tangent=math.atan2(final.y-first.y,final.x-first.x)
        length=math.hypot(final.x-first.x,final.y-first.y)
        if final.label in STRAFE_PHASES and abs(wrap_radians(tangent-final.heading-math.pi/2))>1e-6:
            raise RuntimeError('횡이동 경로 방향 설정 오류')
        self.system.set_translation_phase(final.label)
        self.system.straight_heading_target=final.heading
        started=time.monotonic()
        try:
            self._enable_follow()
            while True:
                self._check_active()
                elapsed=time.monotonic()-started
                if elapsed>max(45,length/.1+15):
                    raise RuntimeError(phase_label+' 제한시간 초과')
                x,y,h=self._pose()
                remaining,cross=leg_errors(x,y,final.x,final.y,tangent)
                yaw=wrap_radians(final.heading-h)
                if remaining<=.03:
                    self._stop_formation()
                    if remaining<-.08 or abs(cross)>.08 or abs(yaw)>math.radians(3):
                        raise RuntimeError('구간 종점 오차 초과: 자동으로 되돌리지 않고 정지합니다.')
                    self._dwell(phase_label+' · 정지 확인',.35)
                    self._write_log(final.label+'_STOPPED')
                    return
                with self.lock:
                    speed=self.speed if remaining>=.30 else min(self.speed,.20)
                    self.route_state='DRIVING'
                    self.route_label=phase_label
                    self.route_progress=progress_start+(progress_end-progress_start)*clamp(1-remaining/length,0,1)
                limit=min(speed,.60*elapsed)
                cross_cmd=clamp(.9*cross,-.10,.10)
                wx=speed*math.cos(tangent)-cross_cmd*math.sin(tangent)
                wy=speed*math.sin(tangent)+cross_cmd*math.cos(tangent)
                magnitude=math.hypot(wx,wy)
                if magnitude>limit:
                    wx*=limit/magnitude
                    wy*=limit/magnitude
                vx,vy=wx*math.cos(h)+wy*math.sin(h),-wx*math.sin(h)+wy*math.cos(h)
                w=clamp(self.heading_kp*yaw,-.12,.12)
                ok,msg=self.system.set_motion(vx,vy,w,'MISSION_'+final.label)
                if not ok:
                    raise RuntimeError(msg)
                self._write_log(final.label)
                time.sleep(.07)
        finally:
            self._stop_formation()
            # Includes the stopped/coasting dwell above, but not the following
            # heading alignment, loaded forward leg or calibrated orbit.
            self.system.set_translation_phase(None)

    def _route_worker(self):
        try:
            self._open_log()
            ok,msg=self._wait_for_reset()
            if not ok:
                raise RuntimeError(msg)
            self._follow_path(APPROACH,f'왼쪽 60cm 횡이동 · ({START_X:g},0) → 적재 (0,0)',0,.25)
            self._turn_gap_cm=self._range_cm()
            self._align_headings(math.pi/2)
            self._wait_loading()
            self._follow_path(TRANSPORT_Y,'적재 운송 · (0,0) → (0,3)',.25,.55)
            self._dwell('(0,3) 정지 · 시계 90° 공전 준비',.5)
            self._rotate_formation(math.pi/2,-1,'시계 90° 공전',target_heading=0)
            self._follow_path(TRANSPORT_X,'적재 운송 · (0,3) → (3.6,3)',.60,.90)
            self._dwell('운송 도착 · 왼쪽 횡이동 퇴장 준비',.5)
            exit_path=base.line_points((3.6,3),(3.6,3+self.exit_distance),0,'EXIT_LEFT')
            self._follow_path(exit_path,'두 로봇 왼쪽 횡이동 퇴장 (+y)',.90,1)
            self._check_abort()
            with self.lock:
                self.route_state='COMPLETE'
                self.route_label=f'퇴장 완료 · 두 로봇 정지 · 재시작 전 ({START_X:g},0) 시작 배치로 옮기세요.'
                self.route_progress=1
        except InterruptedError:
            self.system.emergency_stop()
            self.route_state='EMERGENCY_STOP'
            self.route_label='사용자 중지 · 자동 재개하지 않습니다.'
        except Exception as error:
            self.system.emergency_stop()
            self.route_state='ERROR'
            self.route_label='시나리오 안전정지'
            self.last_error=str(error)
            self._write_log('ERROR')
        finally:
            self._stop_formation()
            self._close_log()
            with self.lock:
                self.loading_token=None
                self.loading_deadline=None
                self.running=False

    def _rotate_formation(self, angle, direction, label, *, target_heading=None):
        """End at R1's measured IMU goal; retain orbit commands and alignment."""
        self._check_abort()
        with self.system.lock:
            previous=[self.system._heading(i) for i in (0,1)]
        if any(h is None or not math.isfinite(h) for h in previous):
            raise RuntimeError('공전 시작 IMU 연결 확인 필요')
        target_heading=wrap_radians(previous[0]+direction*angle) if target_heading is None else target_heading
        required=(direction*wrap_radians(target_heading-previous[0]))%(2*math.pi)
        if abs(required-angle)>math.radians(15):
            raise RuntimeError('공전 시작 방향이 경로 방향과 맞지 않습니다.')
        with self.lock:
            self.route_state='TURNING'
            self.route_label=label+' · R1 IMU 목표 방향 도달 시 공전 종료'
            self.turn_degrees=0.0
            self.turn_target_degrees=math.degrees(required)
            self.turn_exit={}
        accumulated=[0.0,0.0]
        deadline=time.monotonic()+18
        try:
            with self.system.lock:
                self._turn_gap_cm=self._range_cm()
                self.system.route_deadline=time.monotonic()+.6
                ok,message=self.system.set_orbit(True,'CCW' if direction>0 else 'CW',
                                                 self.system.gap,self.orbit_speed)
            if not ok:
                raise RuntimeError(message)
            while True:
                self._check_active(turning=True)
                if time.monotonic()>deadline:
                    raise RuntimeError(label+' 제한시간 초과')
                with self.system.lock:
                    headings=[self.system._heading(i) for i in (0,1)]
                    rates=[self.system._yaw_rate(i) for i in (0,1)]
                probe=turn_exit_probe(headings,rates,target_heading,direction)
                for i in (0,1):
                    delta=direction*wrap_radians(headings[i]-previous[i])
                    if abs(delta)>math.radians(25):
                        raise RuntimeError(f'R{i+1} 공전 IMU 각도 급변')
                    accumulated[i]+=delta
                    if accumulated[i]<-math.radians(10):
                        raise RuntimeError(f'R{i+1} 공전 방향 반대: IMU 부호/배선 확인')
                previous=headings
                probe['imu_turned_deg']=[math.degrees(v) for v in accumulated]
                with self.lock:
                    self.turn_degrees=math.degrees(accumulated[0])
                    self.turn_exit=probe
                if probe['brake_robot'] is not None:
                    self._stop_formation()
                    self._write_log(f"TURN_BRAKE_R{probe['brake_robot']}")
                    break
                with self.system.lock:
                    self.system.orbit_phase_scale=clamp(
                        max(0,required-accumulated[0])/math.radians(30),.55,1.0)
                self._write_log(label)
                time.sleep(.04)
        finally:
            self._stop_formation()
        self._dwell('공전 정지 · 잔류 회전 확인',.35)
        self._check_abort()
        with self.system.lock:
            stopped=[self.system._heading(i) for i in (0,1)]
        if (any(h is None or not math.isfinite(h) for h in stopped) or
                max(abs(wrap_radians(target_heading-h)) for h in stopped)>
                math.radians(TURN_ALIGNMENT_ENTRY_MAX_DEG)):
            raise RuntimeError('공전 정지 후 방향 오차 12° 초과: 두 로봇 정지, 배치 확인 필요')
        self._write_log('TURN_STOPPED')
        # Existing 1-degree / 2 deg/s / 1-second gate, with post-STOP recheck.
        # Never relabel an 85-degree pose as 90 or re-zero either IMU.
        self._align_headings(target_heading)
        self._nudge_robot2_right()

    def _nudge_robot2_right(self):
        """Hold R1 and move only R2 body-right for about 20 cm, open-loop."""
        self._check_abort()
        with self.system.lock:
            self.system.alignment_target = None
            self.system.leader_motion = Motion(0, 0, 0, 'R1_HOLD_FOR_R2_NUDGE')
            # Body +vy is left, therefore negative vy is right.
            self.system.robot2_nudge_motion = Motion(
                0, -R2_POST_ORBIT_RIGHT_SPEED, 0, 'R2_POST_ORBIT_RIGHT_20CM')
        try:
            self._enable_follow()
            with self.lock:
                self.route_state = 'R2_POSITION_CORRECTION'
                self.route_label = '공전 후 R1 정지 · R2 오른쪽 약 20cm 횡이동 보정'
            deadline = time.monotonic() + R2_POST_ORBIT_RIGHT_SECONDS
            while time.monotonic() < deadline:
                self._check_active()
                self._write_log('R2_POST_ORBIT_RIGHT_20CM')
                time.sleep(.04)
        finally:
            with self.system.lock:
                self.system.robot2_nudge_motion = None
            self._stop_formation()
        self._dwell('R2 오른쪽 위치 보정 완료', .25)

    def state(self):
        result=super().state()
        result['auto'].update(mission=True,exit_distance_m=self.exit_distance,
            turn_exit=dict(self.turn_exit),
            center_gap_m=CENTER_GAP,nominal_ultrasonic_gap_cm=NOMINAL_ULTRASONIC_GAP_CM,
            loading_gap_tolerance_cm=LOADING_GAP_TOLERANCE_CM,
            loading_gap_cm=self.loading_gap_cm,loading_token=self.loading_token,
            loading_wait_seconds=LOADING_WAIT_SECONDS,
            loading_seconds_remaining=(max(0,self.loading_deadline-time.monotonic())
                                       if self.loading_deadline is not None else None),
            loading_wait=self.route_state=='WAIT_LOADING',setup_ready=self.exit_distance is not None,
            required_robot_count=self.required_robot_count,
            waypoints=[[START_X,START_Y],[0,0],[0,3],[3.6,3],[3.6,3+(self.exit_distance or 0)]],
            uwb_enabled=False)
        return result


HTML = r'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>화물 적재·운송·퇴장 시나리오</title><style>
:root{color-scheme:dark;--bg:#091421;--panel:#122237;--text:#eaf2ff;--muted:#a3b7cf;--line:#29435e}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,sans-serif}
main{max-width:1500px;margin:auto;padding:18px}h1{font-size:24px;margin:0 0 6px}.muted{color:var(--muted);font-size:13px;line-height:1.6}
.layout{display:grid;grid-template-columns:minmax(500px,1.6fr) minmax(360px,1fr);gap:16px;margin-top:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:15px}
canvas{width:100%;height:auto;background:#091626;border-radius:10px}.notice{background:#081626;padding:12px;border-radius:9px;line-height:1.6;margin:10px 0;overflow-wrap:anywhere}
.row{display:flex;gap:8px;flex-wrap:wrap}.badge{padding:6px 10px;border-radius:20px;background:#273d57;font-size:12px}.ok{background:#15543e}.bad{background:#662c35}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:8px}.stats>div{background:#091a2d;padding:10px;border-radius:8px}.stats b{display:block;margin-top:6px}
button,input{font:inherit}button{padding:12px;border-radius:9px;border:1px solid #3e6890;background:#175c98;color:white;cursor:pointer}button:disabled{opacity:.4;cursor:not-allowed}.danger{background:#9d303e}.load{background:#14724c;width:100%;font-weight:700;margin-top:10px}
input[type=number]{width:105px;padding:8px;background:#091626;color:white;border:1px solid #456482;border-radius:7px}input[type=range]{width:100%}.camera{width:100%;max-height:210px;object-fit:contain;background:#060d17;margin-top:12px}
pre{white-space:pre-wrap;font-size:12px}.error{color:#ffb0b0}.progress{height:10px;background:#07101b;margin:10px 0;border-radius:8px;overflow:hidden}.progress>div{height:100%;background:#32ce94;width:0}
@media(max-width:1000px){.layout{grid-template-columns:1fr}}
</style><main><h1>화물 적재 → 운송 → 횡이동 퇴장</h1>
<div class="muted">시작 R1 (__START_X__,0), R2 (__START_X__,−__CENTER_GAP__), 둘 다 +y · 초음파 배치 __ULTRASONIC_GAP_CM__cm · UWB는 화면 전용 · 기본 속도 0.30 · 60cm 접근 · 보정 ON v11 · 적재 5초 후 자동 출발 · IMU 90° 공전 종료 · R2 오른쪽 약 20cm 횡이동 · R2 초음파 대상 유실 시 주행 유지</div>
<div class="notice">첫 횡이동 60cm → 적재 대기. 기존 거리 배율·R2 횡속도 보정은 첫 구간에 복구했습니다. 처음·마지막 횡이동의 위치/방향/초음파 간격 보정도 복구했습니다. 직진·공전·안전정지는 유지됩니다. 바닥 미끄러짐에 따른 실측 오차는 남을 수 있습니다.</div>
<div class="row" style="margin-top:10px"><span id="r1badge" class="badge">R1</span><span id="r2badge" class="badge">R2</span><span id="uwbbadge" class="badge">UWB</span><span class="badge">UWB 주행 반영 없음</span></div>
<div class="layout"><section class="card"><canvas id="map" width="920" height="760"></canvas>
<div class="muted">파랑: R1 엔코더+IMU · 주황: R2 조건부 편대 추정 · 보라 ◇: UWB 태그 · 보라 +: UWB 로봇1 중심 환산</div>
<label class="muted"><input id="rawToggle" type="checkbox" onchange="draw()"> R2 엔코더 누적 좌표 표시 (회색 ×)</label>
<div id="relativeNotice" class="notice">편대 추정은 R2 센서가 R1 후면을 본다는 가정이며 실제 절대좌표가 아닙니다.</div>
<div class="muted">A1 (−0.6,0) · A2 (−0.6,3) · A3 (4.2,3) · A4 (4.2,0). 앵커 번호를 이 순서에 맞춰 배치하세요.</div></section>
<aside class="card"><div id="notice" class="notice">연결 대기</div><div class="progress"><div id="bar"></div></div>
<div class="stats"><div>단계<b id="phase">-</b></div><div>진행률<b id="progress">-</b></div><div>필요 로봇<b id="requiredRobots">QR 대기</b></div><div>R1 엔코더+IMU<b id="p1">-</b></div><div>R2 편대 추정<b id="p2">-</b></div><div>초음파 간격<b id="gap">-</b></div><div>중심 간격 기준<b>__CENTER_GAP__ m</b></div></div>
<div class="notice"><label>마지막 왼쪽 퇴장 거리 <input id="exitDistance" type="number" min="0.2" max="2" step="0.1" placeholder="예: 0.5"> m</label>
<button id="setup" onclick="setup()">설정 적용</button><div id="exitInfo" class="muted">+x를 바라보는 상태의 왼쪽은 지도 +y입니다. 설정 전에는 출발하지 않습니다.</div></div>
<label>속도 <span id="speedText">0.30</span></label><input id="speed" type="range" min="0.2" max="0.5" step="0.01" value="0.3" oninput="speedChanged(this.value)">
<div class="row"><button id="start" onclick="start()" disabled>QR 없이 시나리오 시작</button><button class="danger" onclick="stop()">비상 정지 (Space)</button></div>
<div class="notice">적재 위치에서 방향 정렬 후 5초가 지나면 자동 출발합니다. 카운트다운 중 적재를 마치고 이동 경로를 비우세요. 준비가 안 됐으면 STOP을 누르세요.</div>
<div id="uwbInfo" class="notice">UWB 대기</div><div id="error" class="notice error">-</div>
<div id="rangeLoss" class="notice">R2 초음파 대기 · 대상 유실 시 거리 보정만 중단</div>
<div id="orbitSync" class="notice">공전 종료 기준: R1 IMU 목표 방향 · 엔코더 공전 이동량 판정 사용 안 함 · R2는 IMU 방향 정렬</div>
<details><summary>적용 보정값 / 원시 좌표 / CSV</summary><pre id="details"></pre></details>
<img id="camera" class="camera" src="/api/frame.jpg"></aside></div></main>
<script>
const $=id=>document.getElementById(id),canvas=$('map'),ctx=canvas.getContext('2d');let state=null;
async function post(path,body={}){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),d=await r.json();if(!r.ok||d.ok===false)throw Error(d.message||'요청 실패');return d}
async function action(fn){try{await fn()}catch(e){$('error').textContent=e.message}}
async function setup(){await action(()=>post('/api/setup',{exit_distance_m:Number($('exitDistance').value)}))}
async function start(){await action(()=>post('/api/start'))}
async function stop(){await action(()=>post('/api/stop'))}
async function speedChanged(v){$('speedText').textContent=Number(v).toFixed(2);await action(()=>post('/api/speed',{speed:Number(v)}))}
addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','TEXTAREA'].includes(e.target.tagName)){e.preventDefault();stop()}});
function mapxy(x,y){const xmin=-1,xmax=4.7,ymin=-1,ymax=Math.max(3.8,3+(state?.auto.exit_distance_m||0)+.5),m=55,s=Math.min((canvas.width-2*m)/(xmax-xmin),(canvas.height-2*m)/(ymax-ymin)),ox=(canvas.width-(xmax-xmin)*s)/2,oy=(canvas.height-(ymax-ymin)*s)/2;return[ox+(x-xmin)*s,canvas.height-oy-(y-ymin)*s]}
function line(points,color,dash=[]){ctx.strokeStyle=color;ctx.lineWidth=3;ctx.setLineDash(dash);ctx.beginPath();points.forEach((p,i)=>{const q=mapxy(...p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.stroke();ctx.setLineDash([])}
function robot(p,color,label){if(!p||p.heading_deg==null)return;const [x,y]=mapxy(p.x,p.y);ctx.save();ctx.translate(x,y);ctx.rotate(-p.heading_deg*Math.PI/180);ctx.fillStyle=color;ctx.fillRect(-16,-11,32,22);ctx.strokeStyle='white';ctx.beginPath();ctx.moveTo(10,0);ctx.lineTo(25,0);ctx.stroke();ctx.restore();ctx.fillStyle=color;ctx.font='13px sans-serif';ctx.fillText(label,x+19,y-13)}
function cross(p,color,label,size=7){if(!p)return;const [x,y]=mapxy(p.x,p.y);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x-size,y);ctx.lineTo(x+size,y);ctx.moveTo(x,y-size);ctx.lineTo(x,y+size);ctx.stroke();ctx.fillStyle=color;ctx.fillText(label,x+10,y-9)}
function draw(){ctx.fillStyle='#091626';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.font='12px sans-serif';for(let x=0;x<=4;x++)line([[x,-.8],[x,3]],'#173047');for(let y=0;y<=3;y++)line([[-.6,y],[4.2,y]],'#173047');
const pts=state?.auto.waypoints||[[__START_X__,0],[0,0],[0,3],[3.6,3]];line(pts,'#31cf98',[7,5]);pts.forEach((p,i)=>{const q=mapxy(...p);ctx.fillStyle='#d9eaff';ctx.fillText(['시작','적재 · 5초 대기','90° 공전','운송 도착','퇴장'][i]+' ('+p.join(',')+')',q[0]+7,q[1]-14)});
const anchors=state?.uwb_display?.anchors||[[-.6,0],[-.6,3],[4.2,3],[4.2,0]];anchors.forEach((p,i)=>{const [x,y]=mapxy(...p);ctx.fillStyle='#b099ee';ctx.fillRect(x-5,y-5,10,10);ctx.fillText('A'+(i+1),x+9,y+6)});
const r=__CENTER_GAP__,arc=[];for(let i=0;i<=24;i++){const a=-Math.PI/2-i*Math.PI/48;arc.push([r*Math.cos(a),3+r*Math.sin(a)])}line(arc,'#ff9f43');
if(!state)return;robot(state.leader,'#40a9ff','R1');robot(state.follower_relative,'#ff9f43','R2 편대추정');if($('rawToggle').checked)cross(state.follower,'#99aabb','R2 엔코더');
const u=state.uwb_display;if(u?.tag){const [x,y]=mapxy(u.tag.x,u.tag.y);ctx.strokeStyle=u.status==='OK'?'#c18cff':'#8c739e';ctx.beginPath();ctx.moveTo(x,y-8);ctx.lineTo(x+8,y);ctx.lineTo(x,y+8);ctx.lineTo(x-8,y);ctx.closePath();ctx.stroke();cross(u.center,ctx.strokeStyle,'UWB R1 중심')}}
function badge(id,ok,label){$(id).textContent=label;$(id).className='badge '+(ok?'ok':'bad')}
function pose(p){return p?`${p.x.toFixed(2)}, ${p.y.toFixed(2)} / ${p.heading_deg?.toFixed(1)??'-'}°`:'추정 불가'}
async function poll(){try{state=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());const a=state.auto,u=state.uwb_display;
$('notice').textContent=a.label;$('phase').textContent=a.state;$('progress').textContent=(a.progress*100).toFixed(0)+'%';$('bar').style.width=(a.progress*100)+'%';$('p1').textContent=pose(state.leader);$('p2').textContent=pose(state.follower_relative);$('gap').textContent=(state.follower.ultrasonic_cm??'-')+' cm';$('relativeNotice').textContent=state.relative_notice;
badge('r1badge',state.leader.online,'R1 '+(state.leader.online?'ONLINE':'OFFLINE'));badge('r2badge',state.follower.online,'R2 '+(state.follower.online?'ONLINE':'OFFLINE'));badge('uwbbadge',true,'UWB ONLINE');$('requiredRobots').textContent=a.required_robot_count?`${a.required_robot_count}대`:(a.qr?'판단 불가':'QR 대기');
$('start').disabled=a.running||!a.setup_ready;$('setup').disabled=a.running;$('exitDistance').disabled=a.running;
if(document.activeElement!==$('exitDistance')&&a.exit_distance_m!=null)$('exitDistance').value=a.exit_distance_m;
$('exitInfo').textContent=a.exit_distance_m==null?'퇴장 거리를 입력해 적용하세요.':`퇴장: R1 (3.6, ${(3+a.exit_distance_m).toFixed(2)}) · R2는 중심 간격 __CENTER_GAP__m 유지`;
if(document.activeElement!==$('speed'))$('speed').value=a.speed;$('speedText').textContent=a.speed.toFixed(2);
$('uwbInfo').textContent=`UWB ONLINE · 화면 전용 / RMSE ${u?.rmse_m?.toFixed(3)??'-'}m · 태그 좌표 ${u?.tag?u.tag.x.toFixed(2)+', '+u.tag.y.toFixed(2):'-'} · 기존 태그 좌표 대신 A1~A4 거리로 재계산`;
const g=(state.control_mode||'').startsWith('ORBIT_')?state.calibrated_orbit:state.range_follow;$('rangeLoss').textContent=g?.gap_target_lost?`R2 초음파 대상 유실: ${g.gap_loss_reason} · 거리 보정 OFF / 주행 유지`:'R2 초음파: 대상 유실 시 거리 보정만 중단 · 근접 안전정지는 유지';
const t=a.turn_exit;$('orbitSync').textContent=t?.imu_turned_deg?`공전 IMU 회전: R1 ${t.imu_turned_deg[0].toFixed(1)}° / R2 ${t.imu_turned_deg[1].toFixed(1)}° · 목표 방향 ${t.target_heading_deg.toFixed(1)}° · 엔코더 이동량 판정 OFF`:'공전 종료 기준: R1 IMU 목표 방향 · 엔코더 공전 이동량 판정 사용 안 함 · R2는 IMU 방향 정렬';
if(a.last_error)$('error').textContent=a.last_error;
$('details').textContent=JSON.stringify({calibration:state.calibration,orbit_progress:state.orbit_progress,turn_exit:a.turn_exit,r2_odometry:state.follower,log:a.log_path,uwb:u},null,2);draw()}catch(e){$('notice').textContent='GUI 연결 오류: '+e.message}}
setInterval(poll,150);setInterval(()=>{$('camera').src='/api/frame.jpg?t='+Date.now()},300);poll();draw();
</script></html>'''.replace('__START_X__',f'{START_X:g}').replace('__CENTER_GAP__',f'{CENTER_GAP:g}').replace('__ULTRASONIC_GAP_CM__',f'{NOMINAL_ULTRASONIC_GAP_CM:g}').replace('__LOADING_GAP_TOLERANCE_CM__',f'{LOADING_GAP_TOLERANCE_CM:g}')


class Handler(base.Handler):
    def do_GET(self):
        if self.path=='/':
            payload=HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type','text/html; charset=utf-8')
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(payload)
        else:
            super().do_GET()

    def do_POST(self):
        if self.path not in ('/api/setup','/api/loaded'):
            return super().do_POST()
        body=self._body()
        try:
            if self.path=='/api/setup':
                self.app.set_setup(body.get('exit_distance_m'))
                self._json({'ok':True})
            else:
                ok,msg=self.app.confirm_loaded(body.get('token'))
                self._json({'ok':ok,'message':msg},200 if ok else 409)
        except (TypeError,ValueError) as error:
            self._json({'ok':False,'message':str(error)},400)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.json'))
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args()
    config=base.load_config(args.config)
    app=CargoMissionApp(config)
    Handler.app=app
    server=ThreadingHTTPServer(('0.0.0.0',int(config.get('auto_route_web_port',8085))),Handler)
    app.start()
    print('Cargo mission GUI: port',server.server_port)
    print(f'Start: R1 ({START_X:g},0), R2 ({START_X:g},-{CENTER_GAP:g}), both +y. UWB DISPLAY ONLY.')
    print('Revision:',MISSION_REVISION,'| approach 0.60m left | calibration ON')
    print(f'Setup ultrasound: {NOMINAL_ULTRASONIC_GAP_CM:g}cm; center gap: {CENTER_GAP:g}m. Existing orbit controller unchanged.')
    try:
        server.serve_forever()
    finally:
        server.server_close()
        app.close()
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Cargo mission stopped.')
    except (OSError,RuntimeError,ValueError) as error:
        print('ERROR:',error)
        raise SystemExit(1)
