#!/usr/bin/env python3
"""void_episodes.py RUN...: the void of a camera flight (roadmap item 31, K46): the void and dead-reckoning episodes
from the estimator's events, the share of the stereo node's frames judged void, the speed and the distance through
each void, the estimate's error against the truth at its start and end with the published sigma and the drift rate,
the map registrations, the descents and holds, the mission verdicts. Host python, repository root (the root goal of
2026-10-24, part 2; `log/tools/void/part23_episodes.txt`)."""
import csv, json, re, sys
import numpy as np
for run in sys.argv[1:]:
    log = open(f"log/runs/{run}/ros_drone_nav.log", errors="ignore").read()
    events = [(float(m.group(1)), m.group(2) == "true") for m in re.finditer(r"\[(\d+\.\d+)\] \[visual_inertial_odometry_node\]: VISUAL_INERTIAL_ODOMETRY_VOID started=(true|false)", log)]
    dr = [(float(m.group(1)), m.group(2) == "true") for m in re.finditer(r"\[(\d+\.\d+)\] \[visual_inertial_odometry_node\]: VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started=(true|false)", log)]
    void_flags = re.findall(r"STEREO_DEPTH .*? void=(true|false)", log)
    t0 = float(re.search(r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_READINESS ready=true", log).group(1)) if re.search(r"MISSION_READINESS ready=true", log) else None
    g = np.array([[float(v) for v in r[:9]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 9])  # t, x, y, z, ..., wall reception
    est = np.array([[float(v) for v in r[:6]] for r in csv.reader(open(f"log/runs/{run}/vio_estimate.csv")) if len(r) >= 6 and r[0][:1].isdigit()])
    # the truth's sim time against the wall reception time (column 9) maps the log's wall stamps
    wall_to_sim = lambda w: np.interp(w, g[:, 8], g[:, 0])
    episodes = []; start = None
    for t, on in events:
        if on and start is None: start = t
        elif not on and start is not None: episodes.append((start, t)); start = None
    if start is not None: episodes.append((start, g[-1, 8]))
    sig = [(float(m.group(1)), np.hypot(float(m.group(2)), float(m.group(3)))) for m in re.finditer(r"\[(\d+\.\d+)\] \[visual_inertial_odometry_node\]: VISUAL_INERTIAL_ODOMETRY [^\n]*? position_sigma_m=\(([-\d.eE+]+),([-\d.eE+]+),", log)]
    sigma_at = lambda wall: next((v for t, v in reversed(sig) if t <= wall), float('nan'))
    rows = [json.loads(l) for l in open(f"log/runs/{run}/mppi/mppi_track.jsonl")]
    rows = [r for r in rows if r.get("planning_state") == "planned"]
    tt = np.array([r["stamp_ns"] * 1e-9 for r in rows]); spd = np.array([r["actual_speed_mps"] for r in rows])
    ex = np.interp(est[:, 0], g[:, 0], g[:, 1]); ey = np.interp(est[:, 0], g[:, 0], g[:, 2]); ez = np.interp(est[:, 0], g[:, 0], g[:, 3])
    err = np.hypot(est[:, 1] - ex, est[:, 2] - ey); errz = est[:, 3] - ez
    print(f"== {run}: void episodes {len(episodes)}, dead-reckoning events {len(dr)}, stereo frames void {sum(f == 'true' for f in void_flags)} of {len(void_flags)}; truth z {g[:,3].min():.1f}..{g[:,3].max():.1f}; estimate error at the end {err[-1]:.2f} m (z {errz[-1]:+.2f}), largest {err.max():.2f}")
    for a, b in episodes:
        sa, sb = wall_to_sim(a), wall_to_sim(b)
        m = (tt >= sa) & (tt <= sb); me = (est[:, 0] >= sa) & (est[:, 0] <= sb)
        dist = 0.0
        if m.sum() > 1:
            xs = np.interp(tt[m], g[:, 0], g[:, 1]); ys = np.interp(tt[m], g[:, 0], g[:, 2]); dist = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
        print(f"   void {sa:.1f}..{sb:.1f} s ({sb-sa:.1f} s): distance {dist:.1f} m, speed mean {spd[m].mean() if m.any() else float('nan'):.2f} max {spd[m].max() if m.any() else float('nan'):.2f}; error at the start {err[me][0] if me.any() else float('nan'):.2f} at the end {err[me][-1] if me.any() else float('nan'):.2f} (z {errz[me][-1] if me.any() else float('nan'):+.2f}); truth z {np.interp(sa, g[:,0], g[:,3]):.1f}..{np.interp(sb, g[:,0], g[:,3]):.1f}; published sigma at the end {sigma_at(b):.2f} (error/sigma {(err[me][-1] / sigma_at(b)) if me.any() and sigma_at(b) > 0 else float('nan'):.2f}); drift rate {(err[me][-1] - err[me][0]) / max(sb - sa, 1e-6) if me.any() else float('nan'):.3f} m/s, {100 * (err[me][-1] - err[me][0]) / max(dist, 1e-6) if me.any() else float('nan'):.1f} % of the distance")
    for t, on in dr: print(f"   dead reckoning started={on} at {wall_to_sim(t):.1f} s")
    reg = re.findall(r"VIO_MAP_REGISTRATION outcome=(\w+)", log); print(f"   map registrations: " + ", ".join(f"{o} {reg.count(o)}" for o in sorted(set(reg))))
    for key in ("BLIND_LANDING", "DEAD_RECKONING_HOLD", "POSITION_LOST", "MISSION_RESULT", "unreliable_light", "MISSION_WAYPOINT_ACKNOWLEDGED"):
        n = log.count(key); print(f"   {key}: {n}") if n else None
    val = open(f"log/tools/run_{run}.log", errors="ignore").read()
    for l in val.splitlines():
        if l.startswith("FAIL") or ("mean flight speed" in l) or ("true position is" in l): print("   " + l[:150])
