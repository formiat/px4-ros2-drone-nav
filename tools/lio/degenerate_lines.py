#!/usr/bin/env python3
"""degenerate_lines.py RUN...: what a lidar flight's estimator said of its unobserved axis (specification K70), from
the once-a-second LIDAR_INERTIAL_ODOMETRY lines of log/runs/RUN/ros_drone_nav.log and the estimate against the
truth (lio_estimate.csv, gz_pose.csv): the lines with a degenerate axis by the weakest axis's information per
matched point (in thousandths: a bare tunnel reads 0 to 3, a sloped one the barometer holds 8 to 14), the runs of
such lines of 3 s or more and whether the estimate stayed healthy through them, the lines read at 0.010 to 0.020,
the accelerometer bias held (the median and the largest) and the estimate's worst horizontal error. Host,
repository root (the goal of 2026-11-17: r3914, r3929)."""
import bisect, collections, csv, re, statistics, sys
for name in sys.argv[1:]:
    run = f"log/runs/{name}"
    runs = []; current = None; bias = []; flagged = []; marginal = 0; lines = 0
    for line in open(f"{run}/ros_drone_nav.log", errors="replace"):
        if "LIDAR_INERTIAL_ODOMETRY healthy=" not in line: continue
        lines += 1
        t = float(re.search(r"\] \[(\d+\.\d+)\] \[", line).group(1))
        m = re.search(r"healthy=(\w+).*information=(\S+) degenerate_axes=(\d)", line)
        healthy = m.group(1) == "true"; information = float(m.group(2)); degenerate = m.group(3) != "0"
        b = re.search(r"accelerometer_bias_mps2=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", line)
        if b: bias.append(tuple(float(v) for v in b.groups()))
        marginal += 0.010 <= information < 0.020
        if degenerate:
            flagged.append(information)
            if current is None: current = [t, t, healthy]
            else: current[1] = t; current[2] = current[2] and healthy
        elif current is not None:
            runs.append(current); current = None
    if current: runs.append(current)
    truth = [(float(r[0]), float(r[1]), float(r[2])) for r in csv.reader(open(f"{run}/gz_pose.csv")) if len(r) >= 9]
    stamps = [t[0] for t in truth]; worst = 0.0
    for r in list(csv.reader(open(f"{run}/lio_estimate.csv")))[::10]:
        try: e = (float(r[0]), float(r[1]), float(r[2]))
        except (ValueError, IndexError): continue
        g = truth[min(bisect.bisect_left(stamps, e[0]), len(truth) - 1)]
        worst = max(worst, ((e[1] - g[1]) ** 2 + (e[2] - g[2]) ** 2) ** 0.5)
    by = dict(sorted(collections.Counter(int(v * 1000) for v in flagged).items()))
    long = [(round(b - a, 1), h) for a, b, h in runs if b - a >= 3]
    print(f"{name}: {lines} lines, {len(flagged)} with a degenerate axis, by information in thousandths {by}; "
          f"runs of 3 s or more (s, healthy throughout) {long}; lines at 0.010 to 0.020: {marginal}; "
          f"the estimate's worst horizontal error {worst:.2f} m"
          + (f"; bias median ({statistics.median(b[0] for b in bias):+.3f},{statistics.median(b[1] for b in bias):+.3f},"
             f"{statistics.median(b[2] for b in bias):+.3f}), largest {max((b[0] ** 2 + b[1] ** 2 + b[2] ** 2) ** 0.5 for b in bias):.3f} m/s2"
             if bias else ""))
