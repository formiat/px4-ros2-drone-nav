#!/usr/bin/env python3
"""record_report.py TRUTH_CSV ESTIMATE kind=lio|fastlio out=CSV: a lidar-inertial estimate over a real recording
(roadmap item 25: NTU VIRAL through tools/lio/rosbag1_to_record.py) against the recording's ground truth, as the
visual-inertial report reads it (tools/vio/euroc_report.py: ATE after a 4-DOF alignment, RTE over 10 m, the end
error as a share of the path). TRUTH_CSV is the record's gz_pose.csv (stamp_s,x,y,z,qw,qx,qy,qz,wall_s: the
Leica prism, the world z up). ESTIMATE is DIR/replay.csv of tools/lio/replay_lio.cpp (kind=lio: stamp_ns, x, y, z
north-east-down from the first scan, healthy …; the stamps the record's, so the truth's are on the same clock) or
the CSV of FAST-LIO's /Odometry as `ros2 topic echo --csv` prints it (kind=fastlio: sec, nanosec, frame, child, px,
py, pz, qx, qy, qz, qw, …; its frame camera_init is the first body frame, forward-right-down here). Both estimates are the
body's origin and the truth the prism's, 0.4 m from it in the body (leica_prism.yaml): a floor of that size on the
errors, the same for both. The estimate is turned z up (north-east-down to east-north-up for the lidar-inertial
replay), the truth interpolated at every estimate stamp with a reference sample within 0.1 s, and the 32-column
replay CSV written to `out` for euroc_report.py, which then prints the figures."""
import subprocess, sys
import numpy as np
from pathlib import Path

truth_path, estimate_path = sys.argv[1], sys.argv[2]
keys = dict(arg.split("=", 1) for arg in sys.argv[3:])
kind, out = keys.get("kind", "lio"), keys["out"]
truth = np.loadtxt(truth_path, delimiter=",")
first_imu_ns = int(keys["first_imu_ns"]) if "first_imu_ns" in keys else None
if kind == "lio":
    r = np.loadtxt(estimate_path, delimiter=",")
    r = r[r[:, 4] == 1]
    stamp_s = r[:, 0] * 1e-9 - (first_imu_ns * 1e-9 if first_imu_ns else 0.0)
    # North-east-down from the first scan to east-north-up.
    p = np.stack([r[:, 2], r[:, 1], -r[:, 3]], 1)
    q = np.tile([1.0, 0.0, 0.0, 0.0], (len(r), 1))
else:
    rows = []
    for line in open(estimate_path):
        parts = line.strip().split(",")
        try:
            sec, nsec = int(parts[0]), int(parts[1])
            px, py, pz, qx, qy, qz, qw = (float(v) for v in parts[4:11])
        except (ValueError, IndexError):
            continue
        rows.append((sec + nsec * 1e-9 - (first_imu_ns * 1e-9 if first_imu_ns else 0.0), px, py, pz, qw, qx, qy, qz))
    r = np.array(rows)
    # FAST-LIO's camera_init is the first body frame, forward-right-down as the record's: to z up as ours.
    stamp_s, p, q = r[:, 0], np.stack([r[:, 2], r[:, 1], -r[:, 3]], 1), r[:, 4:8]
t_truth = truth[:, 0]
inside = (stamp_s >= t_truth[0]) & (stamp_s <= t_truth[-1])
stamp_s, p, q = stamp_s[inside], p[inside], q[inside]
index = np.searchsorted(t_truth, stamp_s)
near = np.abs(t_truth[np.clip(index, 0, len(t_truth) - 1)] - stamp_s) <= 0.1
tp = np.stack([np.interp(stamp_s, t_truth, truth[:, k]) for k in (1, 2, 3)], 1)
stamp_s, p, q, tp = stamp_s[near], p[near], q[near], tp[near]
with open(out, "w") as f:
    f.write("stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,untriangulated,rms,ms_track,ms_filter,bgx,bgy,bgz,bax,bay,baz\n")
    for i in range(len(stamp_s)):
        f.write("%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,0,0,0,%.4f,%.4f,%.4f,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0\n"
                % (stamp_s[i], p[i, 0], p[i, 1], p[i, 2], q[i, 0], q[i, 1], q[i, 2], q[i, 3], tp[i, 0], tp[i, 1], tp[i, 2]))
print(f"{kind}: {len(stamp_s)} estimate samples with a reference, {stamp_s[0]:.1f} to {stamp_s[-1]:.1f} s; path of the reference "
      f"{np.sum(np.linalg.norm(np.diff(tp, axis=0), axis=1)):.1f} m", flush=True)
subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "vio" / "euroc_report.py"), out], check=False)
