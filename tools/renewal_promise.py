#!/usr/bin/env python3
"""The renewal promise of a flight (specification K89), read from its record.

For every run: how long the planned horizons the offboard accepted stood before the next came, while the
vehicle moved; the promise the planner measured and planned under (the track's `renewal_promise_s`); how
often the offboard braked a horizon held past its promise; and the intervals flown past the promise with no
brake, which the law leaves none of.

    python3 tools/renewal_promise.py r4183 r4190
    python3 tools/renewal_promise.py --estimate 0.10 r4170    # a record flown before the law, or another floor

`--estimate FLOOR` reads a record that carries no promise (or asks what another floor would have done): the
promise is rebuilt from the accepted intervals by the law's own rule over that floor, and the sensor braking
limit the track logged is solved again with the promise's excess as reaction latency (the stopping distance
at the stack's 5 m/s^2; the jerk's ramp left out, so the fall is an estimate, not a flight).
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FLOOR_S = 0.20
LIMIT_S = 1.00
FACTOR = 1.5
WINDOW = 25
# One offboard tick, the clock's grain and the log's own delay.
ENFORCEMENT_SLACK_S = 0.06
MOVING_MPS = 0.5
DECELERATION_MPS2 = 5.0
ACCEPTED = re.compile(r"\[(\d+\.\d+)\] \[mppi_offboard_node\]: EXECUTION_HORIZON accepted=true .* mode=(\w+)")
BRAKED = re.compile(r"\[(\d+\.\d+)\] \[mppi_offboard_node\]: "
                    r"(?:HORIZON_RENEWAL overdue=true .* speed=([0-9.]+)|FINITE_EXECUTION_PATH unavailable=true)")


def read(run):
    directory = ROOT / "log" / "runs" / run
    pose = np.array([[float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[8])]
                     for row in csv.reader(open(directory / "gz_pose.csv", encoding="utf-8"))])
    accepted, modes, braked, overdue_speeds = [], [], [], []
    for line in open(directory / "ros_drone_nav.log", encoding="utf-8", errors="replace"):
        match = ACCEPTED.search(line)
        if match:
            accepted.append(float(match.group(1)))
            modes.append(match.group(2))
            continue
        match = BRAKED.search(line)
        if match:
            braked.append(float(match.group(1)))
            if match.group(2) is not None:
                overdue_speeds.append(float(match.group(2)))
    to_sim = lambda wall: np.interp(np.array(wall, dtype=float), pose[:, 4], pose[:, 0])  # noqa: E731
    step = 12
    speed = np.linalg.norm(pose[step:, 1:4] - pose[:-step, 1:4], axis=1) / np.maximum(
        1.0e-3, pose[step:, 0] - pose[:-step, 0])
    track = [json.loads(line) for line in open(directory / "mppi" / "mppi_track.jsonl", encoding="utf-8")]
    return {
        "accepted": to_sim(accepted), "planned": np.array(modes) == "planned", "braked": to_sim(braked),
        "overdue_speeds": overdue_speeds, "speed_time": pose[step:, 0], "speed": speed, "track": track,
    }


def rebuilt_promise(interval, floor):
    promise = np.full(len(interval), floor)
    for index in range(1, len(interval)):
        promise[index] = max(floor, FACTOR * interval[max(0, index - WINDOW):index].max())
    return promise


def report(run, floor):
    record = read(run)
    accepted, planned = record["accepted"], record["planned"]
    track = record["track"]
    stamp = np.array([row["stamp_ns"] * 1.0e-9 for row in track])
    actual = np.array([row["actual_speed_mps"] for row in track])
    moving_tick = actual > MOVING_MPS
    interval = np.diff(accepted)
    older_speed = np.interp(accepted[:-1], record["speed_time"], record["speed"])
    flown = planned[:-1] & (older_speed > MOVING_MPS)
    measured = floor is None and "renewal_promise_s" in track[len(track) // 2]
    if measured:
        tick_promise = np.array([row["renewal_promise_s"] for row in track])
        index = np.clip(np.searchsorted(stamp, accepted[:-1], side="right") - 1, 0, len(track) - 1)
        promise = np.maximum(FLOOR_S, tick_promise[index])
        used_floor = FLOOR_S
    else:
        used_floor = FLOOR_S if floor is None else floor
        promise = rebuilt_promise(interval, used_floor)
        tick_promise = np.interp(stamp, accepted[1:], promise)
    late = flown & (interval > promise + ENFORCEMENT_SLACK_S)
    unbraked = sum(1 for k in np.where(late)[0]
                   if not np.any((record["braked"] > accepted[k]) & (record["braked"] <= accepted[k + 1])))
    in_motion = interval[flown]
    print(f"{run}: planned horizons flown in motion {len(in_motion)}; the next came after p50 "
          f"{np.percentile(in_motion, 50) * 1e3:.0f} p95 {np.percentile(in_motion, 95) * 1e3:.0f} p99 "
          f"{np.percentile(in_motion, 99) * 1e3:.0f} max {in_motion.max() * 1e3:.0f} ms")
    print(f"   promise ({'measured by the planner' if measured else f'rebuilt over a floor of {used_floor:.2f} s'}): "
          f"p50 {np.percentile(tick_promise[moving_tick], 50) * 1e3:.0f} p95 "
          f"{np.percentile(tick_promise[moving_tick], 95) * 1e3:.0f} ms; over the floor on "
          f"{100.0 * (tick_promise[moving_tick] > used_floor + 1e-6).mean():.1f} percent of the ticks in motion, "
          f"past {LIMIT_S:.1f} s (rest) on {100.0 * (tick_promise > LIMIT_S).mean():.2f} percent of all ticks")
    print(f"   horizons held past their promise in motion: {int(late.sum())}; of them with no brake: "
          f"{unbraked if measured else 'not judged (no law in this record)'}; the offboard's brakes logged: "
          f"{len(record['overdue_speeds'])}"
          + (f", at {min(record['overdue_speeds']):.2f} to {max(record['overdue_speeds']):.2f} m/s"
             if record["overdue_speeds"] else ""))
    print(f"   speed in motion: mean {actual[moving_tick].mean():.2f} p95 "
          f"{np.percentile(actual[moving_tick], 95):.2f} m/s; ticks {len(track)} over {stamp[-1] - stamp[0]:.0f} s")
    if not measured:
        limit = np.array([row["limits"]["sensor_braking"] for row in track], dtype=float)
        age = np.array([row.get("sensor_evidence_age_s") or 0.1 for row in track], dtype=float)
        limiter = np.array([row["limiter"] for row in track])
        extra = np.maximum(0.0, tick_promise - used_floor)
        latency = age + 0.10
        budget = limit * latency + limit ** 2 / (2.0 * DECELERATION_MPS2)
        charged = -(latency + extra) * DECELERATION_MPS2 + np.sqrt(
            ((latency + extra) * DECELERATION_MPS2) ** 2 + 2.0 * DECELERATION_MPS2 * budget)
        fall = limit - charged
        bound = moving_tick & ((limiter == "sensor_braking") | (limiter == "unobserved_frontier"))
        print(f"   estimate: the sensor braking limit falls p50 {np.percentile(fall[moving_tick], 50):.2f} p95 "
              f"{np.percentile(fall[moving_tick], 95):.2f} m/s; the speed is bound by it or by the frontier on "
              f"{100.0 * bound.sum() / max(1, moving_tick.sum()):.0f} percent of the ticks in motion, mean fall there "
              f"{fall[bound].mean() if bound.any() else 0.0:.2f} m/s")
    return unbraked if measured else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimate", type=float, metavar="FLOOR",
                        help="rebuild the promise over this floor, seconds, and estimate what it would charge")
    parser.add_argument("runs", nargs="+")
    arguments = parser.parse_args()
    unbraked = sum(report(run, arguments.estimate) for run in arguments.runs)
    print(f"runs {len(arguments.runs)}; horizons flown past their promise with no brake: {unbraked}")
    return 1 if unbraked else 0


if __name__ == "__main__":
    sys.exit(main())
