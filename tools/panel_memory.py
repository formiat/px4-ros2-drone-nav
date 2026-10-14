#!/usr/bin/env python3
"""panel_memory.py RUN X Y Z YAW W H: what the memory holds of a blank panel (roadmap item 18 stage 0b) (BLANK_PANELS geometry) from the run's
render bag at the end of the flight and when the vehicle first came within 6 m of it: the memory's occupied voxels
(raw_memory_obstacle_points_3d) and darkness voxels (raw_memory_darkness_points_3d) inside the panel's slab
(0.25 m thick), the share of the panel's face voxels covered, and the vehicle's nearest true approach to the face.
Inside the container."""
import sys, math, csv
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
run = sys.argv[1]; x, y, z, yaw, w, h = (float(v) for v in sys.argv[2:8])
n = np.array([math.cos(yaw), math.sin(yaw), 0.0]); t = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
c = np.array([x, y, z])
def in_slab(p, thick=0.3, margin=0.15):
    d = p - c
    return (abs(d @ n) <= thick) & (abs(d @ t) <= w / 2 + margin) & (abs(d[:, 2]) <= h / 2 + margin)
def face_cover(p):
    if len(p) == 0: return 0.0
    d = p - c; u = np.floor((d @ t + w / 2) / 0.25).astype(int); v = np.floor((d[:, 2] + h / 2) / 0.25).astype(int)
    cells = set(zip(u.tolist(), v.tolist())); total = int(math.ceil(w / 0.25)) * int(math.ceil(h / 0.25))
    return len(cells) / total
truth = [(float(a[0]), float(a[1]), float(a[2]), float(a[3])) for a in csv.reader(open(f'/workspace/log/runs/{run}/gz_pose.csv')) if a and a[0][0].isdigit()]
P = np.array([[a[1], a[2], a[3]] for a in truth]); T = np.array([a[0] for a in truth])
dist = np.abs((P - c) @ n); lateral = np.abs((P - c) @ t); vertical = np.abs(P[:, 2] - c[2])
near_face = (lateral <= w / 2) & (vertical <= h / 2)
print(f'{run}: nearest true approach to the panel plane within its face outline: {dist[near_face].min() if near_face.any() else float("nan"):.2f} m; nearest approach to the panel body (any side): {np.min(np.sqrt(np.maximum(dist - 0.025, 0)**2 + np.maximum(lateral - w/2, 0)**2 + np.maximum(vertical - h/2, 0)**2)):.2f} m')
first6 = T[np.linalg.norm(P - c, axis=1) <= 6.0]; t6 = first6.min() if len(first6) else None
print(f'  first within 6 m at sim {t6:.1f} s' if t6 is not None else '  never within 6 m')
import sqlite3
topics = {'/drone_city_nav/raw_memory_obstacle_points_3d': 'occupied', '/drone_city_nav/raw_memory_darkness_points_3d': 'darkness'}
db = sqlite3.connect(f'file:/workspace/log/runs/{run}/render_bag/render_bag_0.db3?mode=ro', uri=True)
ids = {name: topics[name] for name, in db.execute('SELECT name FROM topics') if name in topics}
id_of = {row[0]: topics[row[1]] for row in db.execute('SELECT id, name FROM topics') if row[1] in topics}
latest = {}; at6 = {}
for topic_id, stamp_ns, data in db.execute('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp' % ','.join(str(i) for i in id_of)):
    m = deserialize_message(bytes(data), PointCloud2); st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)]).reshape(-1, 3)
    latest[id_of[topic_id]] = (st, pts)
    if t6 is not None and st <= t6 + 8.0: at6[id_of[topic_id]] = (st, pts)
for label, store in (('when the vehicle has been within 6 m for 8 s', at6), ('at the end', latest)):
    for name, (st, pts) in store.items():
        sel = pts[in_slab(pts)] if len(pts) else pts
        print(f'  {label} ({st:.0f} s) {name}: {len(sel)} voxels in the panel slab, face covered {100*face_cover(sel):.0f} %')
