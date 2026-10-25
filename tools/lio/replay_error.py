#!/usr/bin/env python3
"""replay_error.py RUN REPLAY_CSV [LABEL]: a replay's (or the flight's own lio_estimate.csv's) vertical error, horizontal
error and tilt against the flight's truth (log/runs/RUN/gz_pose.csv), the rest's offset taken as zero, with marks every
15 s. Host python, repository root (the root goal of 2026-10-24, part 1)."""
import csv, sys
import numpy as np
run, path = sys.argv[1], sys.argv[2]; label = sys.argv[3] if len(sys.argv) > 3 else path
g = np.array([[float(v) for v in r[:9]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 9])
r = np.loadtxt(path, delimiter=","); r = r[r[:, 4] == 1]; r = r[np.concatenate([[True], np.diff(r[:, 0]) > 0])]
spawn = g[0]
t = r[:, 0] * 1e-9; x = r[:, 2] + spawn[1]; y = r[:, 1] + spawn[2]; z = -r[:, 3] + spawn[3]
tx, ty, tz = (np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3))
rest = np.median((z - tz)[t < t[0] + 5]); ze = z - tz - rest
he = np.hypot(x - tx, y - ty)
# truth roll/pitch from the quaternion (w, x, y, z)
qw, qx, qy, qz = (np.interp(t, g[:, 0], g[:, k]) for k in (4, 5, 6, 7))
t_roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
t_pitch = np.degrees(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1)))
have_tilt = r.shape[1] >= 15
if have_tilt:
    # the replay's roll/pitch are in NED (body FRD to NED); the truth's in ENU (FLU to ENU): roll_ned = roll_enu, pitch_ned = -pitch_enu
    e_roll = np.degrees(r[:, 13]); e_pitch = -np.degrees(r[:, 14])
    d_roll = e_roll - t_roll; d_pitch = e_pitch - t_pitch
moving = t > t[0] + 10
print(f"{label}: vertical error min {ze[moving].min():+.2f} median {np.median(ze[moving]):+.2f} max {ze[moving].max():+.2f} m; horizontal end {he[-1]:.2f} max {he.max():.2f} m" +
      (f"; tilt error (estimate - truth) roll median {np.median(d_roll[moving]):+.2f} [{d_roll[moving].min():+.2f},{d_roll[moving].max():+.2f}] pitch median {np.median(d_pitch[moving]):+.2f} [{d_pitch[moving].min():+.2f},{d_pitch[moving].max():+.2f}] deg" if have_tilt else ""))
marks = []
for mark in np.arange(0, t[-1] - t[0], 15.0):
    i = min(int(np.searchsorted(t, t[0] + mark)), len(t) - 1)
    marks.append(f"{mark:3.0f}s ({tx[i]:5.1f},{ty[i]:5.1f}) z{ze[i]:+.2f}" + (f" r{d_roll[i]:+.2f} p{d_pitch[i]:+.2f}" if have_tilt else ""))
print("  " + " | ".join(marks))
