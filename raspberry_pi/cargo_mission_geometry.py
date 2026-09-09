"""Mission geometry and DISPLAY-ONLY UWB multilateration (no control imports)."""
import math

ANCHORS = ((-.6, 0.0), (-.6, 3.0), (4.2, 3.0), (4.2, 0.0))  # A1..A4


def solve_ranges(ranges, anchors=ANCHORS):
    if len(ranges) != 4 or len(anchors) != 4:
        raise ValueError('Four anchor ranges required')
    if any(not math.isfinite(v) or v <= 0 or v > 30 for v in ranges):
        raise ValueError('Invalid range')
    if any(not math.isfinite(v) for a in anchors for v in a):
        raise ValueError('Invalid anchor')
    x0, y0 = anchors[0]
    aa = ab = bb = ac = bc = 0.0
    for (x, y), d in zip(anchors[1:], ranges[1:]):
        a, b = 2*(x-x0), 2*(y-y0)
        c = ranges[0]**2-d*d+x*x+y*y-x0*x0-y0*y0
        aa += a*a
        ab += a*b
        bb += b*b
        ac += a*c
        bc += b*c
    det = aa*bb-ab*ab
    if det <= 1e-9:
        raise ValueError('Degenerate anchors')
    x, y = (bb*ac-ab*bc)/det, (aa*bc-ab*ac)/det
    rmse = math.sqrt(sum((math.hypot(x-ax,y-ay)-d)**2
                          for (ax,ay),d in zip(anchors,ranges))/4)
    if not all(math.isfinite(v) for v in (x,y,rmse)):
        raise ValueError('Nonfinite solution')
    return x, y, rmse


def display_uwb(packet, now, heading, tag_forward=.11, tag_left=.025, anchors=ANCHORS):
    """Never use tag's old-layout x/y; solve received A1..A4 ranges locally.

    Equal tag/anchor height (10cm) assumed. No old-layout directional model.
    Invalid UWB is a display state, never a reason to stop or steer robots.
    """
    result = {'enabled_for_control': False, 'status': 'OFFLINE',
              'tag': None, 'center': None, 'rmse_m': None, 'anchors': anchors,
              'source': 'Pi ranges solve; A1..A4 order', 'age_sec': None}
    if packet is None:
        return result
    result['age_sec'] = now-packet.received_at
    if not 0 <= result['age_sec'] <= 1.0:
        return result
    try:
        ranges = tuple(float(v) for v in packet.fields['ranges_m'].split(':'))
        x, y, rmse = solve_ranges(ranges, anchors)
    except (KeyError, TypeError, ValueError, ArithmeticError):
        result['status'] = 'INVALID_RANGES'
        return result
    result.update(rmse_m=rmse, tag={'x':x,'y':y},
                  status='OK' if rmse <= .35 else 'CHECK_QUALITY')
    if heading is not None and math.isfinite(heading):
        result['center'] = {'x': x-tag_forward*math.cos(heading)+tag_left*math.sin(heading),
                            'y': y-tag_forward*math.sin(heading)-tag_left*math.cos(heading)}
    return result
