#!/usr/bin/env python3
"""down_recognition.py FRAMES_DIR GZ_POSE [--radius M] [--height M] [--ground Z]: is the ground recognised from
above? (roadmap item 39 stage 3's question, asked of a recording before a camera looking down is built into the
stack). FRAMES_DIR is a record of the evaluation camera that looks down (tools/stereo_record_frames.py with
RECORD_SENSORS=evaluation_down: index.csv and grey frames, 640 x 480 over 90 degrees), GZ_POSE the flight's true
poses. Keyframes a metre of true path apart; ORB with orientation (a place is come back to on another heading) away
from the vehicle's own legs at the frame's edges; Hamming matches with the ratio test at 0.75; a similarity (turn,
shift, scale) by RANSAC at 3 px. Printed: the pairs of the same place (the true positions within the radius, 3 m
unless given, their heights within --height, 2 m unless given, 30 s and 30 m of path apart) and as many pairs of
different places (20 m and more apart, drawn with a fixed seed), the matches and the inliers of each kind, the share
accepted at 12, 20, 30 and 50 inliers, how many of the keyframes that came back are recognised, and the pairs of
different places accepted though they stand farther apart than both frames see (a frame sees its height to each
side; the height is the true one over --ground, the level of the ground, 0 unless given). Host (OpenCV).

  python3 tools/down_recognition.py log/tools/sheets/down/r4348 log/runs/r4348/gz_pose.csv --radius 10 --height 20 --ground 10.7
"""
from __future__ import annotations

import bisect
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RATIO = 0.75
REPROJECTION_PX = 3.0
LEGS_PX = 70  # the vehicle's legs stand in the frame's first and last columns and match themselves in every pair


def option(name: str, default: float) -> float:
    return float(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    frames_dir, truth_path = Path(sys.argv[1]), Path(sys.argv[2])
    radius, height, ground = option("--radius", 3.0), option("--height", 2.0), option("--ground", 0.0)
    truth = np.array([[float(v) for v in row[:4]] for row in csv.reader(open(truth_path)) if row and row[0][0].isdigit()])
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1))])
    index = sorted((float(row[1]), row[4]) for row in csv.reader(open(frames_dir / "index.csv"))
                   if row[0] == "evaluation_down")
    keys, last = [], -math.inf
    for stamp, name in index:
        i = min(len(truth) - 1, bisect.bisect_left(truth[:, 0], stamp))
        if path[i] - last >= 1.0:
            keys.append((stamp, name, truth[i, 1:4].copy(), path[i]))
            last = path[i]
    position = np.array([key[2] for key in keys])
    stamps = np.array([key[0] for key in keys])
    along = np.array([key[3] for key in keys])
    print(f"frames {len(index)}, keyframes {len(keys)}, path {path[-1]:.0f} m, true height "
          f"p10 {np.percentile(position[:, 2], 10):.1f} p50 {np.median(position[:, 2]):.1f} "
          f"p90 {np.percentile(position[:, 2], 90):.1f} m", flush=True)
    orb = cv2.ORB_create(nfeatures=1500)
    mask = np.zeros((480, 640), np.uint8)
    mask[:, LEGS_PX:640 - LEGS_PX] = 255
    started = time.time()
    features = []
    for n, key in enumerate(keys):
        points, descriptors = orb.detectAndCompute(np.load(frames_dir / key[1]), mask)
        features.append((np.array([p.pt for p in points], np.float32).reshape(-1, 2), descriptors))
        if n % 300 == 0:
            print(f"  features of {n} of {len(keys)} keyframes, {time.time() - started:.0f} s", flush=True)
    counts = [len(f[0]) for f in features]
    print(f"features a keyframe p10 {np.percentile(counts, 10):.0f} p50 {np.median(counts):.0f} "
          f"p90 {np.percentile(counts, 90):.0f}", flush=True)
    same = [(i, j) for j in range(len(keys)) for i in range(j)
            if stamps[j] - stamps[i] >= 30.0 and along[j] - along[i] >= 30.0
            and np.linalg.norm(position[j, :2] - position[i, :2]) <= radius
            and abs(position[j, 2] - position[i, 2]) <= height]
    came_back = len({j for _, j in same})
    random = np.random.default_rng(1)
    other: list[tuple[int, int]] = []
    while len(other) < max(200, min(len(same), 2000)):
        i, j = sorted(int(v) for v in random.integers(0, len(keys), 2))
        if i != j and np.linalg.norm(position[j, :2] - position[i, :2]) >= 20.0:
            other.append((i, j))
    if len(same) > 3000:
        same = [same[k] for k in random.choice(len(same), 3000, replace=False)]
    print(f"pairs of the same place (within {radius:g} m and {height:g} m of height) {len(same)} over {came_back} "
          f"keyframes that came back; of different places {len(other)}", flush=True)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def read(i: int, j: int) -> tuple[int, int]:
        (points_i, descriptors_i), (points_j, descriptors_j) = features[i], features[j]
        if descriptors_i is None or descriptors_j is None or len(points_i) < 8 or len(points_j) < 8:
            return 0, 0
        good = [pair[0] for pair in matcher.knnMatch(descriptors_i, descriptors_j, k=2)
                if len(pair) == 2 and pair[0].distance < RATIO * pair[1].distance]
        if len(good) < 6:
            return len(good), 0
        _, inliers = cv2.estimateAffinePartial2D(points_i[[m.queryIdx for m in good]],
                                                 points_j[[m.trainIdx for m in good]], method=cv2.RANSAC,
                                                 ransacReprojThreshold=REPROJECTION_PX)
        return len(good), int(inliers.sum()) if inliers is not None else 0

    results = {}
    for label, pairs in (("same place", same), ("different places", other)):
        results[label] = np.array([read(i, j) for i, j in pairs]).reshape(-1, 2)
        matches, inliers = results[label][:, 0], results[label][:, 1]
        if len(pairs):
            print(f"{label}: matches p50 {np.median(matches):.0f} p90 {np.percentile(matches, 90):.0f}; inliers p50 "
                  f"{np.median(inliers):.0f} p90 {np.percentile(inliers, 90):.0f} max {inliers.max()}; accepted "
                  + ", ".join(f"at {t}: {100 * (inliers >= t).mean():.1f} %" for t in (12, 20, 30, 50)), flush=True)
    # Different places that no frame of the pair can both see: a frame sees its height over the ground to each side.
    reach = np.array([np.linalg.norm(position[j, :2] - position[i, :2])
                      > 2.5 * (max(1.0, position[i, 2] - ground) + max(1.0, position[j, 2] - ground))
                      for i, j in other])
    for threshold in (12, 20, 30):
        recognised = len({j for (_, j), inliers in zip(same, results["same place"][:, 1]) if inliers >= threshold})
        false = int((results["different places"][reach, 1] >= threshold).sum())
        print(f"at {threshold} inliers: {recognised} of {came_back} keyframes that came back recognised; "
              f"{false} of {int(reach.sum())} pairs of places out of each other's sight accepted")
    return 0


if __name__ == "__main__":
    sys.exit(main())
