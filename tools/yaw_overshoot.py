"""yaw_overshoot.py RUN...: gaze overshoots from mppi_track.jsonl: the heading passing its gaze target and swinging on
past it by more than 45 degrees; the rate at the crossing, the swing and the time they take."""
import json, math, sys
for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    rows = [r for r in rows if r['gaze_target_yaw'] is not None and r['planning_state'] == 'planned']
    events, swing, t_cross, rate_cross, sign = [], 0.0, None, 0.0, 0
    for a, b in zip(rows, rows[1:]):
        ea = math.remainder(a['yaw'] - a['gaze_target_yaw'], 2 * math.pi)
        eb = math.remainder(b['yaw'] - b['gaze_target_yaw'], 2 * math.pi)
        if t_cross is None:
            if abs(ea) < 0.5 and abs(eb) < 0.5 and ea * eb < 0 and abs(b['yaw_rate']) > 0.5:
                t_cross, rate_cross, sign, swing = b['stamp_ns'], b['yaw_rate'], math.copysign(1, eb), 0.0
            continue
        past = math.remainder(b['yaw'] - b['gaze_target_yaw'], 2 * math.pi) * sign
        if past < 0 or b['yaw_rate'] * sign <= 0:
            if swing > math.radians(45):
                events.append((swing, rate_cross, (b['stamp_ns'] - t_cross) * 1e-9))
            t_cross = None
            continue
        swing = max(swing, past)
    span = (rows[-1]['stamp_ns'] - rows[0]['stamp_ns']) * 1e-9
    big = [e for e in events if e[0] > math.radians(90)]
    print(f"{run}: {span:.0f} s gazing; overshoots > 45 deg: {len(events)}, > 90 deg: {len(big)}; "
          f"time swinging past {sum(e[2] for e in events):.0f} s; rate at crossing p50 "
          f"{sorted(abs(e[1]) for e in events)[len(events)//2] if events else 0:.2f} rad/s")
