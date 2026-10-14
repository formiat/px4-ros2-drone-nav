#!/usr/bin/env python3
"""stereo_record_frames.py OUT_DIR [KEEP_EVERY]: the stereo pair and the evaluation depth camera of a flight, recorded
over gz-transport for specification K7's measurement (tools/stereo_depth_accuracy.sh): every KEEP_EVERY-th frame of
each stream by the frame number of the pair's 15 Hz clock, the same frames of every stream (left and right as grey
uint8, the depth as float16 metres), listed in OUT_DIR/index.csv. Run inside the simulation container while a camera
flight on Urban flies; RECORD_BASE names the sensors' topic prefix."""
import os
import sys
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

out = sys.argv[1]
keep = int(sys.argv[2]) if len(sys.argv) > 2 else 8
os.makedirs(out, exist_ok=True)
base = os.environ.get(
    'RECORD_BASE',
    '/world/urban_circuit_practice_01_collisions/model/x500_lidar_3d_0/link/stereo_tof_link/sensor')
lock = threading.Lock()
counts = {}
index = open(os.path.join(out, 'index.csv'), 'a')


def kept(stamp):
    return int(round(stamp * 15.0)) % keep == 0


def image_cb(name):
    def cb(msg: Image):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
        with lock:
            counts[name] = counts.get(name, 0) + 1
        if not kept(stamp):
            return
        if name.startswith('evaluation'):
            a = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            a = np.where(np.isfinite(a), a, 0).astype(np.float16)
        else:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            a = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)
        path = f'{name}_{stamp:010.3f}.npy'
        np.save(os.path.join(out, path), a)
        with lock:
            index.write(f'{name},{stamp:.6f},{msg.width},{msg.height},{path}\n')
            index.flush()
    return cb


node = Node()
for name in ('stereo_left', 'stereo_right', 'evaluation_depth_left'):
    topic = f'{base}/{name}/' + ('depth_image' if name.startswith('evaluation') else 'image')
    print(name, topic, node.subscribe(Image, topic, image_cb(name)), flush=True)
started = time.time()
while True:
    time.sleep(10)
    with lock:
        print(f'{time.time() - started:6.0f}s', dict(counts), flush=True)
