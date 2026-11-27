#!/usr/bin/env python3
"""extract_stack_bag.py MCAP_FILE OUT_DIR: the lidar scans of a flight's stack bag (RECORD_STACK_BAG=true, zstd
chunks, which tools/lio/extract_bag.py does not open) as scans.bin of a record of tools/lio/extract_bag.py:
repeated [int64 stamp_ns][uint32 n][n*3 float32], the points in the body frame (forward-right-down, the lidar's
extrinsic applied). Read with the mcap library and its ROS 2 decoder (host: /data/gazebo/tools/bagpy/bin/python -I);
logs the scans done every thousand."""
import os
import struct
import sys
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

path, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
t_body = np.array([0.12, 0.0, -0.315], dtype=np.float32)
started = time.time()
count = 0
with open(path, "rb") as stream, open(os.path.join(out, "scans.bin"), "wb") as scans:
    reader = make_reader(stream, decoder_factories=[DecoderFactory()])
    total = next((c.message_count for c in [reader.get_summary().statistics] if c), 0)
    print(f"start {time.strftime('%F %T')} {path}: {total} messages in the bag", flush=True)
    for _, _, _, m in reader.iter_decoded_messages(topics=["/lidar_3d/points"]):
        offsets = {f.name: f.offset for f in m.fields}
        raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
        xyz = np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0] for k in ("x", "y", "z")], 1)
        body = np.stack([xyz[:, 0], -xyz[:, 1], -xyz[:, 2]], 1) + t_body
        body = body[np.isfinite(body).all(1)].astype(np.float32)
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        scans.write(struct.pack("<qI", stamp, len(body)))
        scans.write(body.tobytes())
        count += 1
        if count % 1000 == 0:
            print(f"  done {count} scans, elapsed {time.time() - started:.0f} s", flush=True)
print(f"end {time.strftime('%F %T')}: {count} scans, elapsed {time.time() - started:.0f} s", flush=True)
