#!/usr/bin/env python3
"""euroc_report.py REPLAY_CSV [REPLAY_CSV ...]: a replay of a real recording (tools/vio/replay_vio.cpp on a record of
tools/vio/euroc_to_replay.py) against its reference, as the visual-inertial literature reports it (roadmap item 25):

  ATE   the absolute trajectory error, RMSE of the position after the 4-DOF alignment a visual-inertial odometry
        is observable up to (a translation and a turn about the vertical), and without any alignment (the filter
        starts on the reference's pose, so the raw error is a fair figure too)
  RTE   the relative translation error over segments of 10 m of path: the RMSE of the error each segment adds
  drift the error at the end over the path flown, in percent

The first row (before the filter's start) is left out."""
import sys

import numpy as np


def yaw_alignment(estimate: np.ndarray, reference: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The turn about z and the translation that best map the estimate onto the reference."""
    e_mean, r_mean = estimate.mean(0), reference.mean(0)
    e, r = estimate - e_mean, reference - r_mean
    yaw = np.arctan2(np.sum(e[:, 0] * r[:, 1] - e[:, 1] * r[:, 0]),
                     np.sum(e[:, 0] * r[:, 0] + e[:, 1] * r[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return rotation, r_mean - rotation @ e_mean


def report(path: str) -> None:
    rows = np.loadtxt(path, delimiter=",", skiprows=1, usecols=range(26))[1:]
    estimate, reference = rows[:, 1:4], rows[:, 11:14]
    rotation, translation = yaw_alignment(estimate, reference)
    aligned = estimate @ rotation.T + translation
    raw_error = np.linalg.norm(estimate - reference, axis=1)
    ate = np.sqrt(np.mean(np.sum((aligned - reference) ** 2, axis=1)))
    ate_raw = np.sqrt(np.mean(raw_error ** 2))
    steps = np.linalg.norm(np.diff(reference, axis=0), axis=1)
    station = np.concatenate([[0.0], np.cumsum(steps)])
    # The relative error over 10 m on the yaw-aligned estimate: an estimator that starts in
    # a frame of its own (OpenVINS, roadmap item 25's reference) would otherwise charge the
    # frames' turn to every segment; ours starts on the reference's pose, unchanged by it.
    segment_errors = []
    for start in range(len(rows)):
        end = int(np.searchsorted(station, station[start] + 10.0))
        if end >= len(rows):
            break
        added = (aligned[end] - aligned[start]) - (reference[end] - reference[start])
        segment_errors.append(np.linalg.norm(added))
    rte = np.sqrt(np.mean(np.square(segment_errors))) if segment_errors else float("nan")
    # The end's drift after the start's alignment alone (the first five seconds, 4-DOF): what
    # the flight sees of a drift, the same number as without alignment for an estimator that
    # starts on the reference's pose.
    first = rows[:, 0] <= rows[0, 0] + 5.0
    start_rotation, start_translation = yaw_alignment(estimate[first], reference[first])
    from_start = np.linalg.norm(estimate @ start_rotation.T + start_translation - reference, axis=1)
    print(f"{path}: {rows[0, 0]:.1f}..{rows[-1, 0]:.1f} s, path {station[-1]:.1f} m; "
          f"ATE {ate:.3f} m (no alignment {ate_raw:.3f} m, max {raw_error.max():.2f} m); "
          f"RTE(10 m) {rte:.3f} m; drift at the end {from_start[-1]:.2f} m = "
          f"{100.0 * from_start[-1] / station[-1]:.2f} % of the path (the start aligned; "
          f"{raw_error[-1]:.2f} m raw); "
          f"frames with features used {100.0 * np.mean(rows[:, 20] > 0):.0f} %")


if __name__ == "__main__":
    for argument in sys.argv[1:]:
        report(argument)
