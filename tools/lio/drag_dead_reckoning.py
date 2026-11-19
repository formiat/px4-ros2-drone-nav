#!/usr/bin/env python3
"""drag_dead_reckoning.py RECORD RUN REPLAY_CSV [K]: on a lidar flight's record, how far a horizontal axis the scans
do not observe runs over 10, 20 and 40 s when it rides (i) the IMU with the accelerometer bias the replay holds
(tools/lio/dead_reckoning.py's measure) and (ii) the same with the rotors' drag fused as the axis's velocity, as the
estimator fuses it (specification K70): the body's velocity across the rotor axis = -(the accelerometer's mean
reading there over a scan interval - offset) / K (0.104 1/s), the offset learned at a gain of 0.005 a scan against
the replay's velocity up to the window's start, a scalar Kalman step a scan with a deviation of 1.4 m/s and as much
again for each metre a second read. Windows every 5 s from the replay's own state; the errors against the truth
(log/runs/RUN/gz_pose.csv) along one axis, both horizontal axes pooled, and for the windows where the body moves
under 1 m/s on the average (a stand's case). Host python (NumPy), repository root (the goal of 2026-11-17: r3914 and
r3929, the stand in a tunnel)."""
import csv, sys
import numpy as np
record, run, replay = sys.argv[1], sys.argv[2], sys.argv[3]
K = float(sys.argv[4]) if len(sys.argv) > 4 else 0.104
SIGMA, GAIN, MAXIMUM = 1.4, 0.005, 1.2
imu = np.loadtxt(f"{record}/imu.csv", delimiter=",")
est = np.loadtxt(replay, delimiter=","); est = est[np.concatenate([[True], np.diff(est[:, 0]) > 0])]
tru = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 9])
ti = imu[:, 0] * 1e-9; te = est[:, 0] * 1e-9; tt = tru[:, 0]
def truth_ned(t): return np.array([np.interp(t, tt, tru[:, 2]), np.interp(t, tt, tru[:, 1])])   # north = map y, east = map x
n = len(te)
edges = np.searchsorted(ti, te)
fmean = np.zeros((n, 3))
for k in range(1, n):
    a, b = edges[k - 1], edges[k]
    fmean[k] = imu[a:b, 4:7].mean(axis=0) if b > a else fmean[k - 1]
roll, pitch, yaw = est[:, 13], est[:, 14], est[:, 15]
cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
Rk = np.stack([np.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], 1),
               np.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], 1),
               np.stack([-sp, cp * sr, cp * cr], 1)], 1)
vel = est[:, 10:13]
vbody = np.einsum("nji,nj->ni", Rk, vel)
# The offset as the estimator learns it: every healthy scan with no degenerate axis.
offsets = np.zeros((n, 2)); offset = np.zeros(2)
for k in range(1, n):
    if est[k, 4] == 1 and est[k, 6] == 0:
        offset = offset + GAIN * (fmean[k, :2] - offset + K * vbody[k, :2])
    offsets[k] = offset
dts = np.diff(te, prepend=te[0])
print(f"{replay}: the drag {K} 1/s; the offset the scans taught, body x and y by the minute:",
      " ".join(f"({offsets[k, 0]:+.3f},{offsets[k, 1]:+.3f})" for k in range(600, n, 600)))
for T in (10.0, 20.0, 40.0):
    rows = []; slow = []
    for t0 in np.arange(te[0] + 100.0, te[-1] - T - 1.0, 5.0):
        k0 = int(np.searchsorted(te, t0)); k1 = int(np.searchsorted(te, t0 + T))
        if est[k0, 4] != 1: continue
        bias = est[k0, 7:10]; c = offsets[k0]
        v_i = vel[k0, :2].copy(); p_i = np.zeros(2); v_d = vel[k0, :2].copy(); p_d = np.zeros(2); P = 0.01
        for k in range(k0 + 1, k1 + 1):
            dt = dts[k]
            acc = (Rk[k] @ (fmean[k] - bias) + np.array([0, 0, 9.80665]))[:2]
            p_i += v_i * dt + 0.5 * acc * dt * dt; v_i += acc * dt
            p_d += v_d * dt + 0.5 * acc * dt * dt; v_d += acc * dt; P += (0.5 * dt) ** 2
            across = fmean[k, :2] - c
            if np.all(np.abs(across) < MAXIMUM):
                body = -across / K
                z = (Rk[k] @ np.array([body[0], body[1], 0.0]))[:2]
                variance = (SIGMA * (1.0 + np.linalg.norm(body))) ** 2
                gain = P / (P + variance); v_d += gain * (z - v_d); P *= (1 - gain)
        dtruth = truth_ned(te[k1]) - truth_ned(te[k0])
        vtruth = (truth_ned(te[k1] + 0.1) - truth_ned(te[k1] - 0.1)) / 0.2
        rows.append((*(p_i - dtruth), *(v_i - vtruth), *(p_d - dtruth), *(v_d - vtruth)))
        slow.append(np.linalg.norm(vbody[k0:k1, :2], axis=1).mean() < 1.0)
    r = np.abs(np.array(rows)); s = np.array(slow)
    def stats(v): return f"p50 {np.percentile(v, 50):5.2f} p95 {np.percentile(v, 95):5.2f} max {v.max():5.2f}"
    print(f"{T:.0f} s, {len(r)} windows, the IMU with the bias: position {stats(r[:, 0:2].ravel())} m, velocity {stats(r[:, 2:4].ravel())} m/s")
    print(f"                    with the drag:          position {stats(r[:, 4:6].ravel())} m, velocity {stats(r[:, 6:8].ravel())} m/s")
    if s.sum() > 3:
        print(f"      the {int(s.sum())} windows under 1 m/s: the IMU's velocity {stats(r[s][:, 2:4].ravel())} | the drag's {stats(r[s][:, 6:8].ravel())} m/s")
