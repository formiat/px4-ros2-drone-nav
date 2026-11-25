#!/usr/bin/env python3
"""loop_closures.py RUN | ROS_LOG GZ_POSE_CSV [--list]: the stereo estimate's loop closures against the truth
(roadmap item 41, stages 1 and 2).

Every `VIO_LOOP_CLOSURE` line of the log (one a keyframe that had a candidate: `outcome`, the two keyframes'
stamps, the closure's measurement `shift_body` and `turn_body`, the new keyframe's body in the old one's body
frame, forward-right-down) is set against the true relative pose of the vehicle between the two stamps
(`gz_pose.csv`: the true pose on the simulation clock, east-north-up, the body forward-left-up). A closure is right
within 0.3 m and 3 degrees of the truth. An accepted closure that is not right is a wrong closure, which the item
counts as a crash: the verdict is FAIL with one.

The returns are read from the truth alone: the vehicle is at a return while a pose it held thirty seconds and
thirty metres of path before lies within 5 m and 45 degrees of heading of where it is; samples less than 10 s apart
are one return. A return is covered when a right closure was accepted in it; an uncovered one is printed with the
outcomes its keyframes read.

RUN reads log/runs/RUN/ros_drone_nav.log and log/runs/RUN/gz_pose.csv; a replay's log (log/tools/replay/RUN/
stack/ros_replay.log) is given with the flight's gz_pose.csv. --list prints every closure tried. Host.

  python3 tools/loop_closures.py r4234
  python3 tools/loop_closures.py log/tools/replay/r4234/stack/ros_replay.log log/tools/gpscl/part5/runs/r4234/gz_pose.csv
"""
from __future__ import annotations

import bisect
import csv
import math
import re
import sys
from collections import Counter
from pathlib import Path

import numpy as np

RIGHT_M = 0.3
RIGHT_DEG = 3.0
RETURN_AGE_S = 30.0
RETURN_PATH_M = 30.0
RETURN_RADIUS_M = 5.0
RETURN_HEADING_RAD = math.radians(45.0)
RETURN_GAP_S = 10.0
# Forward-right-down to forward-left-up and back.
FLIP = np.diag([1.0, -1.0, -1.0])
LINE = re.compile(r"VIO_LOOP_CLOSURE outcome=(\w+) stamp_s=([\d.]+) matched_stamp_s=([\d.]+) .*?"
                  r"inliers=(\d+) off_plane=(\d+) .*?path_between_m=([\d.]+) innovation_m=([\d.eE+-]+|nan|inf) .*?"
                  r"shift_body=\(([^)]*)\) turn_body=\(([^)]*)\) .*?recognition_ms=([\d.]+)")


def rotation(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Truth:
    def __init__(self, path: Path) -> None:
        rows = [[float(v) for v in row[:8]] for row in csv.reader(path.open()) if row and row[0][:1].isdigit()]
        self.stamps = [row[0] for row in rows]
        self.rows = rows
        self.path = [0.0]
        for before, after in zip(rows, rows[1:]):
            self.path.append(self.path[-1] + math.dist(before[1:4], after[1:4]))

    def at(self, stamp: float) -> tuple[np.ndarray, np.ndarray, float]:
        """The true position, the body's rotation into the world and the path flown, at the stamp."""
        i = min(max(bisect.bisect_left(self.stamps, stamp), 1), len(self.rows) - 1)
        before, after = self.rows[i - 1], self.rows[i]
        span = after[0] - before[0]
        share = min(max((stamp - before[0]) / span, 0.0), 1.0) if span > 0 else 0.0
        position = np.array(before[1:4]) + share * (np.array(after[1:4]) - np.array(before[1:4]))
        row = after if share >= 0.5 else before
        return position, rotation(*row[4:8]), self.path[i - 1] + share * (self.path[i] - self.path[i - 1])


def returns(truth: Truth) -> list[tuple[float, float]]:
    """The returns as (first stamp, last stamp), from the truth sampled every second."""
    samples = []
    stamp = truth.stamps[0]
    while stamp <= truth.stamps[-1]:
        position, body, path = truth.at(stamp)
        samples.append((stamp, position, math.atan2(body[1, 0], body[0, 0]), path))
        stamp += 1.0
    episodes: list[list[float]] = []
    for index, (stamp, position, heading, path) in enumerate(samples):
        again = False
        for old_stamp, old_position, old_heading, old_path in samples[:index]:
            if stamp - old_stamp < RETURN_AGE_S or path - old_path < RETURN_PATH_M:
                break
            if (np.linalg.norm(old_position - position) <= RETURN_RADIUS_M and
                    abs(math.remainder(old_heading - heading, math.tau)) <= RETURN_HEADING_RAD):
                again = True
                break
        if not again:
            continue
        if episodes and stamp - episodes[-1][1] <= RETURN_GAP_S:
            episodes[-1][1] = stamp
        else:
            episodes.append([stamp, stamp])
    return [(first, last) for first, last in episodes]


def main() -> int:
    arguments = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(arguments) == 1:
        base = Path("log/runs") / arguments[0]
        log, pose = base / "ros_drone_nav.log", base / "gz_pose.csv"
    elif len(arguments) == 2:
        log, pose = Path(arguments[0]), Path(arguments[1])
    else:
        print(__doc__.split("\n\n")[0])
        return 2
    truth = Truth(pose)
    closures = []
    for line in log.open(errors="replace"):
        match = LINE.search(line)
        if match is None:
            continue
        outcome, stamp, old_stamp = match.group(1), float(match.group(2)), float(match.group(3))
        record = {"outcome": outcome, "stamp": stamp, "old_stamp": old_stamp, "inliers": int(match.group(4)),
                  "off_plane": int(match.group(5)), "path_between": float(match.group(6)),
                  "innovation": float(match.group(7)), "ms": float(match.group(10)), "error_m": None,
                  "error_deg": None}
        # A reading with a pose: every outcome past the inliers' count carries its measurement.
        if outcome not in ("few_matches", "few_inliers"):
            shift = FLIP @ np.array([float(v) for v in match.group(8).split(",")])
            w, x, y, z = (float(v) for v in match.group(9).split(","))
            turn = FLIP @ rotation(w, x, y, z) @ FLIP
            position, body, _ = truth.at(stamp)
            old_position, old_body, _ = truth.at(old_stamp)
            true_shift = old_body.T @ (position - old_position)
            true_turn = old_body.T @ body
            record["error_m"] = float(np.linalg.norm(shift - true_shift))
            record["error_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(turn.T @ true_turn) - 1) / 2))))
        closures.append(record)

    def right(record: dict) -> bool:
        return record["error_m"] is not None and record["error_m"] <= RIGHT_M and record["error_deg"] <= RIGHT_DEG

    if "--list" in sys.argv:
        for record in closures:
            measured = "" if record["error_m"] is None else (
                f" error {record['error_m']:.2f} m {record['error_deg']:.2f} deg" + ("" if right(record) else " NOT RIGHT"))
            print(f"{record['stamp']:9.2f} from {record['old_stamp']:9.2f} {record['outcome']:11s} inliers "
                  f"{record['inliers']:3d} off plane {record['off_plane']:3d} innovation {record['innovation']:.2f} m"
                  f"{measured}")
    accepted = [r for r in closures if r["outcome"] == "accepted"]
    wrong = [r for r in accepted if not right(r)]
    print(f"{log}: keyframes with a candidate {len(closures)}; accepted {len(accepted)}, right {len(accepted) - len(wrong)}"
          f", wrong {len(wrong)}")
    if accepted:
        print(f"  accepted: error p50 {np.percentile([r['error_m'] for r in accepted], 50):.2f} max "
              f"{max(r['error_m'] for r in accepted):.2f} m, {max(r['error_deg'] for r in accepted):.2f} deg; innovation "
              f"p50 {np.percentile([r['innovation'] for r in accepted], 50):.2f} max "
              f"{max(r['innovation'] for r in accepted):.2f} m")
    for record in wrong:
        print(f"  WRONG {record['stamp']:.2f} from {record['old_stamp']:.2f}: {record['error_m']:.2f} m "
              f"{record['error_deg']:.2f} deg, inliers {record['inliers']}, off plane {record['off_plane']}, innovation "
              f"{record['innovation']:.2f} m")
    dropped = Counter(r["outcome"] for r in closures if r["outcome"] != "accepted")
    dropped_right = Counter(r["outcome"] for r in closures if r["outcome"] != "accepted" and right(r))
    dropped_wrong = Counter(r["outcome"] for r in closures
                            if r["outcome"] != "accepted" and r["error_m"] is not None and not right(r))
    print("  dropped: " + (", ".join(f"{name} {count} (right by the truth {dropped_right[name]}, not right "
                                     f"{dropped_wrong[name]})" for name, count in sorted(dropped.items())) or "none"))
    if closures:
        times = [r["ms"] for r in closures]
        print(f"  recognition of a keyframe with a candidate: p50 {np.percentile(times, 50):.1f} p95 "
              f"{np.percentile(times, 95):.1f} max {max(times):.1f} ms")
    episodes = returns(truth)
    covered = 0
    for first, last in episodes:
        inside = [r for r in closures if first - 1.0 <= r["stamp"] <= last + 1.0]
        if any(r["outcome"] == "accepted" and right(r) for r in inside):
            covered += 1
            continue
        read = Counter(r["outcome"] for r in inside)
        print(f"  return {first:.0f} to {last:.0f} s not covered: " +
              (", ".join(f"{name} {count}" for name, count in sorted(read.items())) or "no keyframe had a candidate"))
    print(f"  returns (again within {RETURN_RADIUS_M:.0f} m and 45 deg of a pose held {RETURN_AGE_S:.0f} s and "
          f"{RETURN_PATH_M:.0f} m before): {len(episodes)}, covered by a right closure {covered}")
    print("LOOP CLOSURES " + ("FAIL" if wrong else "PASS"))
    return 1 if wrong else 0


if __name__ == "__main__":
    sys.exit(main())
