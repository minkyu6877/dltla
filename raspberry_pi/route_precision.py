"""Pure geometry/control helpers. No I/O and no fabricated absolute position."""
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def alignment_rotation(error, yaw_rate=0.0):
    """Wheel-equivalent yaw command; anticipate coast using rad/s gyro rate."""
    if abs(error) <= math.radians(.8):
        return 0.0
    predicted = error - .20 * yaw_rate
    if error * yaw_rate > 0 and (predicted * error <= 0 or abs(predicted) < math.radians(.5)):
        return 0.0
    # 0.035 previously left wheels stationary for seconds in the recorded run.
    return math.copysign(min(.075, max(.05, .45 * abs(error))), error)


def headings_settled(headings, rates, target):
    if any(v is None or not math.isfinite(v) for v in (*headings, *rates)):
        return False
    return (max(abs(wrap(target-h)) for h in headings) <= math.radians(1)
            and abs(wrap(headings[0]-headings[1])) <= math.radians(1)
            and max(abs(v) for v in rates) <= math.radians(2))


def rear_relative_pose(x1, y1, h1, h2, gap_cm, center_offset_cm):
    """CONDITIONAL estimate: R2 beam hits R1 rear-center, symmetric offsets.

    Ultrasound alone cannot observe lateral displacement/target identity.
    Keep this estimate separate from raw odometry and navigation control.
    """
    rear = center_offset_cm / 200
    beam = gap_cm / 100 + rear
    return {'x': x1-rear*math.cos(h1)-beam*math.cos(h2),
            'y': y1-rear*math.sin(h1)-beam*math.sin(h2),
            'heading_deg': math.degrees(h2), 'source': 'REAR_BEAM_ASSUMPTION'}


def leg_errors(x, y, final_x, final_y, heading):
    dx, dy = final_x-x, final_y-y
    return dx*math.cos(heading)+dy*math.sin(heading), -dx*math.sin(heading)+dy*math.cos(heading)
