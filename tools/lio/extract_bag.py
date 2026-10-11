"""extract_bag.py MCAP_FILE OUT_DIR: lidar scans (body FRD, extrinsic applied) and PX4 IMU from a rosbag2
mcap file, read record by record so a file cut off without its footer still yields its data.
scans.bin: repeated [int64 stamp_ns][uint32 n][n*3 float32]; imu.csv: stamp_ns,gx,gy,gz,ax,ay,az.
IMU stamps are PX4 microseconds; in lockstep SITL they equal the simulation clock (tool-only assumption)."""
import sys, struct, os
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from px4_msgs.msg import SensorCombined, TimesyncStatus
path, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
scans = open(os.path.join(out, "scans.bin"), "wb"); imu = open(os.path.join(out, "imu.csv"), "w")
t_body = np.array([0.12, 0.0, -0.315], dtype=np.float32)
channels = {}
counts = {"scans": 0, "imu": 0, "compressed_chunks": 0, "unsynchronised_imu": 0}
offset_us = [None]
def handle(op, body):
    if op == 0x04:  # Channel
        cid, sid = struct.unpack_from("<HH", body, 0); n = struct.unpack_from("<I", body, 4)[0]
        channels[cid] = body[8:8 + n].decode()
    elif op == 0x05:  # Message
        cid = struct.unpack_from("<H", body, 0)[0]; data = body[22:]
        topic = channels.get(cid)
        if topic == "/lidar_3d/points":
            m = deserialize_message(data, PointCloud2)
            offs = {f.name: f.offset for f in m.fields}
            raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
            xyz = np.stack([raw[:, offs[k]:offs[k] + 4].copy().view(np.float32)[:, 0] for k in ("x", "y", "z")], 1)
            b = np.stack([xyz[:, 0], -xyz[:, 1], -xyz[:, 2]], 1) + t_body
            b = b[np.isfinite(b).all(1)].astype(np.float32)
            stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            scans.write(struct.pack("<qI", stamp, len(b))); scans.write(b.tobytes()); counts["scans"] += 1
        elif topic == "/fmu/out/timesync_status":
            offset_us[0] = deserialize_message(data, TimesyncStatus).estimated_offset
        elif topic == "/fmu/out/sensor_combined":
            m = deserialize_message(data, SensorCombined)
            if offset_us[0] is None:
                # UXRCE_DDS_SYNCT=0: no synchronisation, the stamps are the simulation clock already.
                counts["unsynchronised_imu"] += 1; offset_us[0] = 0
            # The transport stamps the autopilot's clock plus the synchronisation offset;
            # the autopilot's own clock is the simulation's in lockstep.
            local_ns = (m.timestamp + offset_us[0]) * 1000
            imu.write("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n" % (local_ns, *m.gyro_rad, *m.accelerometer_m_s2)); counts["imu"] += 1
    elif op == 0x06:  # Chunk
        n = struct.unpack_from("<I", body, 28)[0]; comp = body[32:32 + n].decode()
        size = struct.unpack_from("<Q", body, 32 + n)[0]; records = body[40 + n:40 + n + size]
        if comp:
            counts["compressed_chunks"] += 1; return
        walk(records)
def walk(buf):
    i = 0
    while i + 9 <= len(buf):
        op = buf[i]; ln = struct.unpack_from("<Q", buf, i + 1)[0]
        if i + 9 + ln > len(buf): break
        handle(op, memoryview(buf)[i + 9:i + 9 + ln].tobytes()); i += 9 + ln
with open(path, "rb") as f:
    f.seek(8)
    while True:
        head = f.read(9)
        if len(head) < 9: break
        op = head[0]; ln = struct.unpack("<Q", head[1:])[0]
        body = f.read(ln)
        if len(body) < ln: break
        handle(op, body)
print(counts)
