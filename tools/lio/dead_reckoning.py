#!/usr/bin/env python3
"""dead_reckoning.py RECORD RUN REPLAY_CSV [WINDOW_S]: how far the lidar-inertial estimator's IMU propagation carries
a horizontal axis the scans do not observe, measured on a flight's record without a tunnel: from the replay's own
state at a window's start (tools/lio/replay_lio: position, velocity, attitude and the accelerometer bias it holds),
the record's IMU (RECORD/imu.csv) is integrated for WINDOW_S (10, the bound of specification K70) with the attitude
the replay holds at each scan, the gyroscope between scans and the bias frozen at the window's start, and the
displacement is compared with the truth's (log/runs/RUN/gz_pose.csv), window by window every half window. Prints
the error along one axis (both horizontal axes pooled) and the horizontal norm, position and velocity. Host python
(NumPy), repository root (the goal of 2026-11-17: r3914's slip in a straight tunnel)."""
import csv, sys
import numpy as np
record, run, replay = sys.argv[1], sys.argv[2], sys.argv[3]
T = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0
imu = np.loadtxt(f"{record}/imu.csv", delimiter=",")
est = np.loadtxt(replay, delimiter=","); est = est[np.concatenate([[True], np.diff(est[:, 0]) > 0])]
tru = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 9])
ti = imu[:, 0] * 1e-9; te = est[:, 0] * 1e-9
def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])
def small(w):
    a = np.linalg.norm(w)
    if a < 1e-12: return np.eye(3)
    k = w / a; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
# The attitude at every IMU sample: the replay's at each scan, the gyroscope's turn between scans.
scan_of = np.searchsorted(te, ti, side="right") - 1
dt = np.diff(ti, append=ti[-1])
R = np.zeros((len(ti), 3, 3)); current = np.eye(3); last_scan = -1
for i in range(len(ti)):
    if scan_of[i] != last_scan and scan_of[i] >= 0:
        last_scan = scan_of[i]; current = rotation(est[last_scan, 13], est[last_scan, 14], est[last_scan, 15])
    R[i] = current
    current = current @ small(imu[i, 1:4] * dt[i])
A = np.einsum("nij,nj->ni", R, imu[:, 4:7]) + np.array([0.0, 0.0, 9.80665])
# Running integrals: of the acceleration and of the attitude (the bias enters through it), once and twice.
def integrals(v):
    first = np.concatenate([np.zeros((1,) + v.shape[1:]), np.cumsum(v[:-1] * dt[:-1].reshape((-1,) + (1,) * (v.ndim - 1)), axis=0)])
    second = np.concatenate([np.zeros((1,) + v.shape[1:]), np.cumsum(first[:-1] * dt[:-1].reshape((-1,) + (1,) * (v.ndim - 1)), axis=0)])
    return first, second
A1, A2 = integrals(A); R1, R2 = integrals(R)
tt = tru[:, 0]
def truth_ned(t): return np.array([np.interp(t, tt, tru[:, 2]), np.interp(t, tt, tru[:, 1])])   # north = map y, east = map x
def truth_velocity(t): return (truth_ned(t + 0.1) - truth_ned(t - 0.1)) / 0.2
healthy = est[:, 4] == 1
rows = []
for t0 in np.arange(te[0] + 40.0, min(te[-1], tt[-1], ti[-1]) - T - 1.0, T / 2):
    k = int(np.searchsorted(te, t0)); t0 = te[k]
    if not healthy[k]: continue
    i0, i1 = int(np.searchsorted(ti, t0)), int(np.searchsorted(ti, t0 + T))
    bias = est[k, 7:10]; v0 = est[k, 10:13]
    span = ti[i1] - ti[i0]
    dv = (A1[i1] - A1[i0]) - (R1[i1] - R1[i0]) @ bias
    dp = v0 * span + (A2[i1] - A2[i0] - A1[i0] * span) - (R2[i1] - R2[i0] - R1[i0] * span) @ bias
    ep = dp[:2] - (truth_ned(ti[i1]) - truth_ned(ti[i0]))
    ev = (v0 + dv)[:2] - truth_velocity(ti[i1])
    rows.append((t0 - te[0], *ep, *ev, *bias, np.linalg.norm(truth_velocity(t0))))
r = np.array(rows)
def stats(v): return f"p50 {np.percentile(v, 50):.2f} p95 {np.percentile(v, 95):.2f} max {v.max():.2f}"
axis_p = np.abs(r[:, 1:3]).ravel(); axis_v = np.abs(r[:, 3:5]).ravel()
print(f"{replay}: {len(r)} windows of {T:.0f} s")
print("  position error along an axis, m:  ", stats(axis_p), "| horizontal norm:", stats(np.hypot(r[:, 1], r[:, 2])))
print("  velocity error along an axis, m/s:", stats(axis_v), "| horizontal norm:", stats(np.hypot(r[:, 3], r[:, 4])))
print(f"  the bias held (body x, y, z), m/s2: median ({np.median(r[:, 5]):+.3f},{np.median(r[:, 6]):+.3f},{np.median(r[:, 7]):+.3f})"
      f" range x [{r[:, 5].min():+.3f},{r[:, 5].max():+.3f}] y [{r[:, 6].min():+.3f},{r[:, 6].max():+.3f}] z [{r[:, 7].min():+.3f},{r[:, 7].max():+.3f}]")
