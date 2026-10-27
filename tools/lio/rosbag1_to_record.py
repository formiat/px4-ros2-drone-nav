#!/usr/bin/env python3
"""rosbag1_to_record.py BAG OUT_DIR [key=value ...]: a real lidar-inertial recording in a ROS 1 bag (roadmap item 25:
NTU VIRAL, an Ouster OS1-16 and a VectorNav VN-100) as a record of tools/lio/replay_lio.cpp, in the format of
tools/lio/extract_bag.py: OUT_DIR/scans.bin (repeated [int64 stamp_ns][uint32 n][n*3 float32], the points in the
body frame, forward-right-down, the lidar's extrinsic applied), OUT_DIR/imu.csv (stamp_ns,gx,gy,gz,ax,ay,az in the
body frame FRD, as the autopilot reports them), and, with truth=CSV (the dataset's ground truth, a Leica prism's
position at 20 Hz: %time,seq,stamp,x,y,z,qx,qy,qz,qw), OUT_DIR/gz_pose.csv in the flight record's shape
(stamp_s,x,y,z,qw,qx,qy,qz,wall_s; the world z up; the stamps seconds from the first IMU sample, as the replay's).
Keys: lidar=TOPIC (/os1_cloud_node1/points), imu=TOPIC (/imu/imu), extrinsic=r11,r12,...,r33,tx,ty,tz (the lidar
in the IMU frame, the rows of the rotation then the translation; NTU VIRAL's horizontal lidar: identity and
-0.050,0.000,0.055), flu=1 (the IMU frame forward-left-up, turned to forward-right-down: the VN-100's as mounted;
the mean specific force at rest is printed, +9.8 on z up is the sign of a left-up frame; NTU VIRAL's body is
forward-right-down already, flu=0), stampshift=S (every scan's stamp moved by S seconds: the Ouster stamps a sweep
at its start, and a sweep that moves is registered at one instant — 0.05 puts it at the sweep's middle, the one
cheap probe of roadmap item 25). Reads the bag with the
rosbags library (no ROS 1 on the host); logs its progress every 500 scans."""
import struct, sys, time
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

bag, out = Path(sys.argv[1]), Path(sys.argv[2])
keys = dict(arg.split("=", 1) for arg in sys.argv[3:])
lidar_topic = keys.get("lidar", "/os1_cloud_node1/points"); imu_topic = keys.get("imu", "/imu/imu")
ext = [float(v) for v in keys.get("extrinsic", "1,0,0,0,1,0,0,0,1,-0.050,0.000,0.055").split(",")]
R_li = np.array(ext[:9], dtype=np.float64).reshape(3, 3); t_li = np.array(ext[9:12], dtype=np.float64)
flip = np.diag([1.0, -1.0, -1.0]) if keys.get("flu", "1") == "1" else np.eye(3)
stampshift_ns = int(float(keys.get("stampshift", "0")) * 1e9)
out.mkdir(parents=True, exist_ok=True)
store = get_typestore(Stores.ROS1_NOETIC)
scans = open(out / "scans.bin", "wb"); imu = open(out / "imu.csv", "w")
counts = {"scans": 0, "imu": 0, "points": 0}; accel_sum = np.zeros(3); first_imu_ns = None; started = time.time()
print(f"record start {time.strftime('%FT%TZ', time.gmtime())} bag {bag}", flush=True)
with Reader(bag) as reader:
    connections = [c for c in reader.connections if c.topic in (lidar_topic, imu_topic)]
    total = sum(c.msgcount for c in connections)
    print(f"messages {total} on {[c.topic for c in connections]}", flush=True)
    for connection, stamp_ns, raw in reader.messages(connections=connections):
        msg = store.deserialize_ros1(raw, connection.msgtype)
        header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if connection.topic == imu_topic:
            if first_imu_ns is None: first_imu_ns = header_ns
            g = flip @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            a = flip @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            accel_sum += a
            imu.write("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n" % (header_ns, *g, *a)); counts["imu"] += 1
        else:
            offs = {f.name: (f.offset, f.datatype) for f in msg.fields}
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
            xyz = np.stack([data[:, offs[k][0]:offs[k][0] + 4].copy().view(np.float32)[:, 0] for k in ("x", "y", "z")], 1).astype(np.float64)
            keep = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.0)
            body = (flip @ (R_li @ xyz[keep].T + t_li[:, None])).T.astype(np.float32)
            scans.write(struct.pack("<qI", header_ns + stampshift_ns, len(body))); scans.write(body.tobytes())
            counts["scans"] += 1; counts["points"] += len(body)
            if counts["scans"] % 500 == 0:
                print(f"{counts['scans']} scans, {counts['imu']} imu, {time.time() - started:.0f} s", flush=True)
scans.close(); imu.close()
mean_a = accel_sum / max(counts["imu"], 1)
print(f"record end: {counts}; mean specific force in the body frame {mean_a.round(3).tolist()}", flush=True)
if "truth" in keys and first_imu_ns is not None:
    rows = []
    for line in open(keys["truth"]):
        if line.startswith("%") or not line.strip(): continue
        v = [float(x) for x in line.split(",")]
        rows.append((v[2], v[3], v[4], v[5], v[9], v[6], v[7], v[8], v[0]))
    with open(out / "gz_pose.csv", "w") as truth:
        for stamp_ns, x, y, z, qw, qx, qy, qz, wall_ns in rows:
            truth.write("%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.6f\n" % ((stamp_ns - first_imu_ns) * 1e-9, x, y, z, qw, qx, qy, qz, wall_ns * 1e-9))
    print(f"truth rows {len(rows)} from {keys['truth']}", flush=True)
