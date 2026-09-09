"""Stateful orbit range control. Raw safety checks always precede filtering."""
from collections import deque
import math
import statistics


class OrbitDistanceFault(RuntimeError):
    pass


class OrbitDistanceControl:
    continue_on_target_loss = False  # Enabled only by CargoMission.

    def __init__(self, config):
        self.kp = float(config.get('orbit_distance_kp', 0.012))
        self.limit = float(config.get('orbit_radial_max_speed', 0.04))
        self.deadband = float(config.get('orbit_gap_deadband_cm', 2.0))
        self.slew = float(config.get('orbit_radial_slew_per_sec', 0.06))
        self.jump = float(config.get('orbit_gap_jump_cm', 4.0))
        self.minimum = float(config.get('orbit_min_gap_cm', 18.0))
        self.maximum = float(config.get('orbit_max_gap_cm', 150.0))
        self.max_error = float(config.get('orbit_max_gap_error_cm', 15.0))
        self.stale_timeout = 0.75
        self.untrusted_timeout = 0.80
        self.confirm_samples = 3
        self.confirm_span = 0.30
        self.confirm_tolerance = 2.0
        values = (self.kp, self.limit, self.deadband, self.slew, self.jump,
                  self.minimum, self.maximum, self.max_error)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError('Invalid orbit distance settings')
        if not (self.limit > 0 and self.slew > 0 and self.jump > 0
                and self.minimum >= 18 and self.maximum > self.minimum
                and self.max_error > self.deadband):
            raise ValueError('Unsafe orbit distance settings')
        self.reset(30.0, 0.0)

    def reset(self, target, now):
        self.target = float(target)
        self.history = deque([self.target], maxlen=3)
        self.filtered = self.target
        self.last_accepted = self.target
        self.last_sample = None
        self.last_evidence = None
        self.last_update = now
        self.pending = []
        self.pending_since = None
        self.rejected = 0
        self.command = 0.0
        self.desired = 0.0
        self.state = 'TRACKING'
        self.fault_reason = ''
        self.loss_reason = ''
        self.raw = self.target

    def fail(self, reason):
        self.command = self.desired = 0.0
        self.state = 'STOP'
        self.fault_reason = reason
        raise OrbitDistanceFault(reason)

    def update(self, raw, sample_time, now):
        try:
            result = self._update_range(raw, sample_time, now)
        except OrbitDistanceFault as error:
            reason = str(error)
            if not self.continue_on_target_loss or reason not in (
                    'RANGE_INVALID', 'RANGE_OUT_OF_BOUNDS', 'RANGE_TARGET_LOST',
                    'RANGE_STALE', 'RANGE_UNCONFIRMED'):
                raise  # In particular, RANGE_NEAR still stops the robots.
            # Keep the last confirmed radius, not a wall/no-echo reading.
            # Disable only radial/spacing correction, not nominal travel.
            self.command = self.desired = 0.0
            self.fault_reason = ''
            self.loss_reason = reason
            self.state = 'TARGET_LOST'
            self.last_update = now
            self.last_sample = self.last_evidence = None
            self.pending.clear()
            self.pending_since = None
            return 0.0
        if self.state == 'TRACKING':
            self.loss_reason = ''
        return result

    def _update_range(self, raw, sample_time, now):
        if self.fault_reason:
            raise OrbitDistanceFault(self.fault_reason)
        try:
            self.raw = float(raw)
        except (TypeError, ValueError):
            self.raw = None
            self.fail('RANGE_INVALID')
        if not math.isfinite(self.raw):
            self.raw = None
            self.fail('RANGE_INVALID')
        # Never hide a near obstacle, target loss, or invalid value in a median.
        if self.raw <= self.minimum:
            self.fail('RANGE_NEAR')
        if self.raw > self.maximum:
            self.fail('RANGE_OUT_OF_BOUNDS')
        if abs(self.raw - self.target) > self.max_error:
            self.fail('RANGE_TARGET_LOST')
        if (sample_time is None or not math.isfinite(sample_time)
                or sample_time > now + 0.01 or now - sample_time > self.stale_timeout):
            self.fail('RANGE_STALE')
        dt = max(0.0, min(0.10, now - self.last_update))
        self.last_update = now
        new_sample = self.last_sample is None or sample_time > self.last_sample
        if new_sample:
            self.last_sample = sample_time
            # PING acknowledgments can repeat a measurement. Require spaced
            # reports as well as distinct timestamps for confirmation/filtering.
            evidence = self.last_evidence is None or sample_time - self.last_evidence >= 0.10 - 1e-9
            if evidence:
                self.last_evidence = sample_time
                if abs(self.raw - self.last_accepted) > self.jump:
                    self.rejected += 1
                    if self.pending_since is None:
                        self.pending_since = now
                    if self.pending and abs(self.raw - statistics.median(v for _, v in self.pending)) > self.confirm_tolerance:
                        self.pending.clear()
                    self.pending.append((sample_time, self.raw))
                    self.state = 'CHECK_JUMP'
                    if (len(self.pending) >= self.confirm_samples
                            and sample_time - self.pending[0][0] >= self.confirm_span):
                        accepted = statistics.median(v for _, v in self.pending)
                        self.history.clear()
                        self.history.append(accepted)
                        self._accept(accepted, now)
                else:
                    self.history.append(self.raw)
                    self._accept(self.raw, now)
        if self.pending_since is not None and now - self.pending_since >= self.untrusted_timeout:
            self.fail('RANGE_UNCONFIRMED')
        error = self.filtered - self.target
        active_error = math.copysign(max(0.0, abs(error) - self.deadband), error)
        self.desired = max(-self.limit, min(self.limit, self.kp * active_error))
        if self.state == 'CHECK_JUMP':
            self.desired = 0.0  # Do not chase a yet-unverified echo.
        # Safety override: do not keep moving in the wrong radial direction
        # when the current raw range contradicts the filtered range.
        if ((self.raw < self.target - self.deadband and self.command > 0)
                or (self.raw > self.target + self.deadband and self.command < 0)):
            self.command = 0.0
        if ((self.raw < self.target - self.deadband and self.desired > 0)
                or (self.raw > self.target + self.deadband and self.desired < 0)):
            self.desired = 0.0
        goal = 0.0 if self.command * self.desired < 0 else self.desired
        max_step = self.slew * dt
        self.command += max(-max_step, min(max_step, goal - self.command))
        return self.command

    def _accept(self, value, now):
        self.last_accepted = value
        self.filtered = statistics.median(self.history)
        self.pending.clear()
        self.pending_since = None
        self.state = 'TRACKING'

    def telemetry(self):
        return {
            'gap_cm': self.raw,
            'gap_filtered_cm': self.filtered,
            'gap_error_cm': None if self.raw is None else self.raw - self.target,
            'target_gap_cm': self.target,
            'gap_filter_state': self.state,
            'gap_pending_samples': len(self.pending),
            'gap_rejected_total': self.rejected,
            'gap_fault': int(bool(self.fault_reason)),
            'gap_fault_reason': self.fault_reason,
            'gap_target_lost': self.state == 'TARGET_LOST',
            'gap_loss_reason': self.loss_reason,
            'radial_requested': self.desired,
            'radial_command': self.command,
        }
