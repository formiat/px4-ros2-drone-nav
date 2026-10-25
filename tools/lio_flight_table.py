#!/usr/bin/env python3
"""lio_flight_table.py RUN...: one line a lidar-profile flight for a series table: the mean speed, the true distance to
B, the real-time factor, the lidar-inertial estimate's vertical error range and horizontal error (end, largest) against
the truth, the time under 1 m/s, the mid-flight seconds of the modelled body under 0.10 m of true clearance (the pad
and the landing left out), the registration time, and the mission check's FAIL lines. Host python, repository root
(the root goal of 2026-10-24, part 1)."""
import csv, re, subprocess, sys
sys.path.insert(0, "tools"); sys.path.insert(0, "scripts")
import numpy as np
import body_truth_clearance as btc

def vertical(run):
    est = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/lio_estimate.csv")) if len(r) >= 4 and r[0][:1].isdigit()])
    g = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 4])
    t = est[:, 0]; tx, ty, tz = (np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3))
    rest = np.median((est[:, 3] - tz)[t < t[0] + 5]); z = est[:, 3] - tz - rest; h = np.hypot(est[:, 1] - tx, est[:, 2] - ty)
    mv = t > t[0] + 10
    return z[mv].min(), z[mv].max(), h[-1], h.max(), g

def midflight_under(g, threshold=0.10):
    t = g[:, 0] - g[0, 0]
    c = np.array([btc.clearance(x, y, z + btc.CENTRE) for _, x, y, z in g[:, :4]])
    # the pad: the first 10 s; the landing: the last 12 s
    mid = (t > 10.0) & (t < t[-1] - 12.0)
    dt = np.median(np.diff(g[:, 0]))
    return float(((c < threshold) & mid).sum() * dt)

for run in sys.argv[1:]:
    val = open(f"log/tools/run_{run}.log", errors="ignore").read()
    speed = re.search(r"mean flight speed (?:is |exceeds [\d.]+ m/s \()([\d.]+) m/s", val)
    b = re.search(r"the true position is ([\d.]+) m from the goal", val)
    rtf = re.search(r"real-time factor is ([\d.]+) at p50", val)
    crash = "CRASH" if "FAIL: no crash" in val or "collided" in val and "FAIL" in val.split("collided")[0][-80:] else "no crash"
    fails = [l for l in val.splitlines() if l.startswith("FAIL")]
    zmin, zmax, hend, hmax, g = vertical(run)
    lim = subprocess.run(["python3", "tools/speed_limiters.py", run], capture_output=True, text=True).stdout
    under1 = re.search(r"under 1 m/s (\d+)%", lim)
    cost = subprocess.run(["python3", "tools/lio/lio_cost.py", run], capture_output=True, text=True).stdout
    scan = re.search(r"scan_ms p50 ([\d.]+) p95 ([\d.]+)", cost)
    print(f"{run}: speed {speed.group(1) if speed else '?'} B {b.group(1) if b else '?'} rtf {rtf.group(1) if rtf else '?'} "
          f"z [{zmin:+.2f},{zmax:+.2f}] h end {hend:.2f} max {hmax:.2f} under1m/s {under1.group(1) if under1 else '?'}% "
          f"mid<0.10m {midflight_under(g):.1f}s scan_ms {scan.group(1) if scan else '?'}/{scan.group(2) if scan else '?'} "
          f"{'FAIL: ' + '; '.join(f[6:60] for f in fails) if fails else 'no FAIL'}")
