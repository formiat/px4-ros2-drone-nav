#!/usr/bin/env python3
"""record_vio.py OUT_DIR [--half]: the visual-inertial record of one flight, every stream on the simulation clock.

  left.bin / right.bin   every frame of the pair, grey uint8, raw, frame after frame (--half: 2x2 binned)
  frames.csv             side,stamp_s,wall_s,width,height,byte_offset
  imu_gz.csv             stamp_s,gx,gy,gz,ax,ay,az            Gazebo's IMU sensor (FLU body), its own stamps
  imu_px4.csv            stamp_s,gx,gy,gz,ax,ay,az,ros_now_s  /fmu/out/sensor_combined (FRD body); stamp = PX4 boot clock,
                                                              the simulation clock in lockstep (UXRCE_DDS_SYNCT=0)
  truth.csv              stamp_s,x,y,z,qw,qx,qy,qz            world pose of the model (pose/info)
Run inside the simulation container. Tool only, nothing of it is production code."""
import argparse, mmap, os, queue, threading, time
import numpy as np
import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from px4_msgs.msg import SensorCombined
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V

ap = argparse.ArgumentParser()
ap.add_argument('out'); ap.add_argument('--frames', action='store_true', help='also take the frames here (slow: prefer record_frames)')
ap.add_argument('--world', default='urban_circuit_practice_01_collisions'); ap.add_argument('--model', default='x500_lidar_3d_0')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
base = f'/world/{a.world}/model/{a.model}/link'
stamp_of = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nsec
writes = queue.Queue(maxsize=600)  # 600 frames = 0.7 GB: the disk is never that far behind
counts = {}; lock = threading.Lock()
def count(k):
    with lock: counts[k] = counts.get(k, 0) + 1

def image_cb(side):
    def cb(m: Image):
        count(side)
        writes.put((side, stamp_of(m), time.time(), m.width, m.height, bytes(m.data)))
    return cb

def writer():
    # Direct I/O into preallocated files: 18 MB/s through the page cache came back as journal commits that stalled every process
    # of the flight for up to a second (r547: PX4's IMU lost 0.5 s, the ROS nodes fell silent together). trim_record.py cuts the
    # files to their used length afterwards.
    files = {}; offsets = {'left': 0, 'right': 0}; block = None
    for side in ('left', 'right'):
        fd = os.open(os.path.join(a.out, f'{side}.bin'), os.O_WRONLY | os.O_CREAT | os.O_DIRECT, 0o644)
        os.posix_fallocate(fd, 0, 7 << 30); files[side] = fd
    index = open(os.path.join(a.out, 'frames.csv'), 'w'); index.write('side,stamp_s,wall_s,width,height,byte_offset\n')
    while True:
        side, stamp, wall, w, h, data = writes.get()
        img = np.frombuffer(data, np.uint8)
        if len(data) == 3 * w * h:  # ITU-R BT.601 luma in integers, as the production image source computes it
            rgb = img.reshape(h, w, 3).astype(np.uint32)
            img = ((4899 * rgb[..., 0] + 9617 * rgb[..., 1] + 1868 * rgb[..., 2] + 8192) >> 14).astype(np.uint8)
        else:
            img = img.reshape(h, w)
        raw = img.tobytes(); padded = -(-len(raw) // 4096) * 4096
        if block is None or len(block) != padded: block = mmap.mmap(-1, padded)
        block[:len(raw)] = raw
        index.write(f'{side},{stamp:.9f},{wall:.6f},{img.shape[1]},{img.shape[0]},{offsets[side]}\n')
        os.pwrite(files[side], block, offsets[side]); offsets[side] += padded; index.flush()

imu_gz = open(os.path.join(a.out, 'imu_gz.csv'), 'w'); imu_gz.write('stamp_s,gx,gy,gz,ax,ay,az\n')
def imu_cb(m: IMU):
    count('imu_gz'); g, l = m.angular_velocity, m.linear_acceleration
    imu_gz.write(f'{stamp_of(m):.9f},{g.x:.9g},{g.y:.9g},{g.z:.9g},{l.x:.9g},{l.y:.9g},{l.z:.9g}\n')
truth = open(os.path.join(a.out, 'truth.csv'), 'w'); truth.write('stamp_s,x,y,z,qw,qx,qy,qz\n')
def pose_cb(m: Pose_V):
    for p in m.pose:
        if p.name == a.model:
            count('truth'); q = p.orientation
            truth.write(f'{stamp_of(m):.9f},{p.position.x:.6f},{p.position.y:.6f},{p.position.z:.6f},{q.w:.9f},{q.x:.9f},{q.y:.9f},{q.z:.9f}\n')
            return

node = Node()
if a.frames:
    threading.Thread(target=writer, daemon=True).start()
    for side in ('left', 'right'):
        print(side, node.subscribe(Image, f'{base}/stereo_tof_link/sensor/stereo_{side}/image', image_cb(side)), flush=True)
print('imu', node.subscribe(IMU, f'{base}/base_link/sensor/imu_sensor/imu', imu_cb), flush=True)
print('truth', node.subscribe(Pose_V, f'/world/{a.world}/pose/info', pose_cb), flush=True)

rclpy.init()
ros = RosNode('vio_recorder', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
imu_px4 = open(os.path.join(a.out, 'imu_px4.csv'), 'w'); imu_px4.write('stamp_s,gx,gy,gz,ax,ay,az,ros_now_s\n')
def px4_cb(m: SensorCombined):
    count('imu_px4'); g, l = m.gyro_rad, m.accelerometer_m_s2
    imu_px4.write(f'{m.timestamp * 1e-6:.6f},{g[0]:.9g},{g[1]:.9g},{g[2]:.9g},{l[0]:.9g},{l[1]:.9g},{l[2]:.9g},{ros.get_clock().now().nanoseconds * 1e-9:.6f}\n')
ros.create_subscription(SensorCombined, '/fmu/out/sensor_combined', px4_cb, qos_profile_sensor_data)
def report():
    with lock: print(f'{time.time():.0f}', dict(counts), 'queue', writes.qsize(), flush=True)
    for f in (imu_gz, truth, imu_px4): f.flush()
ros.create_timer(10.0, report, clock=rclpy.clock.Clock())
rclpy.spin(ros)
