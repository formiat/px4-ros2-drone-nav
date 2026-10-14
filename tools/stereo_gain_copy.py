#!/usr/bin/env python3
"""stereo_gain_copy.py SRC DST: a copy of a record of tools/stereo_record_frames.py with the camera driver's automatic
gain applied to each pair, as the vehicle's pair arrives at the depth node: the left image's 95th percentile lifted to
200, the gain from 1 to 8, the same on both sides, per pair and without the driver's smoothing."""
import os
import shutil
import sys

import numpy as np

src, dst = sys.argv[1], sys.argv[2]
os.makedirs(dst, exist_ok=True)
rows = [line.strip().split(',') for line in open(os.path.join(src, 'index.csv')) if line.strip()]
by_stamp = {}
for row in rows:
    by_stamp.setdefault(row[1], {})[row[0]] = row
gains = []
with open(os.path.join(dst, 'index.csv'), 'w') as out:
    for stamp, frame in sorted(by_stamp.items()):
        if not all(name in frame for name in ('stereo_left', 'stereo_right', 'evaluation_depth_left')):
            continue
        left = np.load(os.path.join(src, frame['stereo_left'][4]))
        right = np.load(os.path.join(src, frame['stereo_right'][4]))
        gain = float(np.clip(200.0 / max(np.percentile(left, 95), 1.0), 1.0, 8.0))
        gains.append(gain)
        for name, image in (('stereo_left', left), ('stereo_right', right)):
            np.save(os.path.join(dst, frame[name][4]),
                    np.clip(image.astype(np.float32) * gain, 0, 255).astype(np.uint8))
            out.write(','.join(frame[name]) + '\n')
        shutil.copy(os.path.join(src, frame['evaluation_depth_left'][4]), dst)
        out.write(','.join(frame['evaluation_depth_left']) + '\n')
print('pairs', len(gains), 'gain p50 %.2f p90 %.2f' % (np.median(gains), np.percentile(gains, 90)))
