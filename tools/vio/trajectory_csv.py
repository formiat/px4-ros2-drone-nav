#!/usr/bin/env python3
"""trajectory_csv.py FORMAT TRAJECTORY OUT_CSV [--truth=ASL_GROUNDTRUTH_CSV] [--lever=x,y,z]: a reference estimator's
trajectory over a real recording as a replay CSV of tools/vio/replay_vio.cpp, so that tools/vio/euroc_report.py
measures it as it measures ours (roadmap item 41 stage 3: the references that close loops beside ours).

  FORMAT orbslam3  ORB-SLAM3's SaveTrajectoryEuRoC file: stamp_ns x y z qx qy qz qw, the body (IMU) in its world
         kimera    Kimera-VIO's traj_vio.csv or traj_pgo.csv: #timestamp,x,y,z,qw,qx,qy,qz,...
         replay    a replay CSV of ours (stamp_s,px,py,pz,qw,qx,qy,qz,...,tx,ty,tz,...), kept as it is but for the lever

--truth is the sequence's reference in the ASL format (mav0/state_groundtruth_estimate0/data.csv: stamp_ns, the
position, ...), interpolated at the trajectory's stamps; poses outside its span are left out (not needed for
`replay`, which carries its own). A reference estimator starts in a frame of its own (ORB-SLAM3's is its first body
frame, not even level): its trajectory is first laid on the reference by the rigid fit of the positions (a rotation
and a shift, no scale), so that the report's figure for it is the absolute trajectory error after a six-degree
alignment; ours starts on the reference's pose and is aligned about the vertical alone. --lever is a point of the body, in the body frame of that estimator, whose
position is written in place of the body origin's: a reference measured at a prism off the body (NTU VIRAL) is
compared with where the estimator puts the prism. Host.

  python3 tools/vio/trajectory_csv.py orbslam3 f_V101.txt orbslam3_V101.csv \\
      --truth=/data/gazebo/datasets/euroc/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv
"""
from __future__ import annotations

import sys

import numpy as np

HEADER = ("stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,"
          "untriangulated,rms,ms_track,ms_filter")


def rotate(quaternion_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz / np.linalg.norm(quaternion_wxyz)
    matrix = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return matrix @ vector


def main() -> int:
    arguments = [a for a in sys.argv[1:] if not a.startswith("--")]
    options = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    if len(arguments) != 3 or arguments[0] not in ("orbslam3", "kimera", "replay"):
        print(__doc__.split("\n\n")[0], file=sys.stderr)
        return 2
    kind, source, out = arguments
    lever = np.array([float(v) for v in options["lever"].split(",")]) if "lever" in options else np.zeros(3)
    if kind == "replay":
        lines = open(source).read().splitlines()
        rows = [line.split(",") for line in lines[1:] if line]
        with open(out, "w") as stream:
            stream.write(lines[0] + "\n")
            for row in rows:
                position = np.array([float(v) for v in row[1:4]]) + rotate(np.array([float(v) for v in row[4:8]]), lever)
                stream.write(",".join([row[0], *(f"{v:.4f}" for v in position), *row[4:]]) + "\n")
        print(f"{out}: {len(rows)} poses")
        return 0
    if kind == "orbslam3":
        table = np.loadtxt(source)
        stamps_ns, positions = table[:, 0], table[:, 1:4]
        quaternions = table[:, [7, 4, 5, 6]]
    else:
        table = np.loadtxt(source, delimiter=",", comments="#", usecols=range(8))
        stamps_ns, positions, quaternions = table[:, 0], table[:, 1:4], table[:, 4:8]
    truth = np.loadtxt(options["truth"], delimiter=",", comments="#", usecols=range(4))
    inside = (stamps_ns >= truth[0, 0]) & (stamps_ns <= truth[-1, 0])
    points = np.array([position + rotate(quaternion, lever)
                       for position, quaternion in zip(positions[inside], quaternions[inside])])
    references = np.column_stack([np.interp(stamps_ns[inside], truth[:, 0], truth[:, axis]) for axis in (1, 2, 3)])
    # The rigid fit of the estimator's frame to the reference's (Kabsch, no scale).
    centre, reference_centre = points.mean(axis=0), references.mean(axis=0)
    u, _, vt = np.linalg.svd((references - reference_centre).T @ (points - centre))
    turn = u @ np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    points = (points - centre) @ turn.T + reference_centre
    written = 0
    with open(out, "w") as stream:
        stream.write(HEADER + "\n")
        for stamp, point, reference, quaternion in zip(stamps_ns[inside], points, references, quaternions[inside]):
            stream.write(f"{(stamp - stamps_ns[inside][0]) * 1.0e-9:.6f}," + ",".join(f"{v:.4f}" for v in point) + ","
                         + ",".join(f"{v:.6f}" for v in quaternion) + ",0,0,0,"
                         + ",".join(f"{v:.4f}" for v in reference) + ",1,0,0,0,0,0,1,0,0,0,0,0\n")
            written += 1
    print(f"{out}: {written} poses of {len(stamps_ns)} within the reference's span")
    return 0


if __name__ == "__main__":
    sys.exit(main())
