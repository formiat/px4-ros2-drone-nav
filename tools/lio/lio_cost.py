"""lio_cost.py RUN...: the lidar-inertial estimator's cost in flight from its once-a-second line: scan_ms p50/p95/max,
the scans let go by K23 (dropped_scans of all), the pose age's 95th percentile (pose_age_max_ms)."""
import re, sys
import numpy as np
allms = []
for run in sys.argv[1:]:
    ms, ages, last = [], [], None
    for line in open(f'log/runs/{run}/ros_drone_nav.log', errors='replace'):
        if 'LIDAR_INERTIAL_ODOMETRY healthy=' not in line:
            continue
        d = dict(re.findall(r'(\w+)=([-0-9.]+)', line))
        ms.append(float(d['scan_ms'])); ages.append(float(d['pose_age_max_ms'])); last = d
    allms += ms
    print(f"{run}: scan_ms p50 {np.median(ms):.1f} p95 {np.percentile(ms, 95):.1f} max {max(ms):.1f}; "
          f"dropped {last['dropped_scans']} of {last['scans']}; pose age p95 {np.percentile(ages, 95):.0f} ms")
print(f"all: scan_ms p50 {np.median(allms):.1f} p95 {np.percentile(allms, 95):.1f}")
