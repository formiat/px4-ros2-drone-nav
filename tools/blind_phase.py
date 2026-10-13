#!/usr/bin/env python3
"""blind_phase.py RUN...: the blind phase of a flight that lost its light (light-lost, a loss beside a wall), from
the last dead-reckoning start before the landing (DR) to the landing, by the truth.

- To the light's verdict (unreliable_light; three seconds after DR when there is none): the offboard's planned
  setpoints and their largest horizontal speed, the truth's path and its displacement.
- The descent, due three seconds into the dead reckoning (specification K19): the true height above the landing at
  DR and at the descent, the true horizontal speed and the estimate's velocity error at the descent, the drift in
  the descent and from DR to the ground.
- Clearances up to one second before the touchdown: the body's least, a floor included (which is what the last
  second brings), and the walls' least, the voxels at the body's height only (up to 1.5 m).

The body and the truth grid are tools/body_truth_clearance.py's; evaluation only."""
from __future__ import annotations

import csv
import sys

import numpy as np

from body_truth_clearance import B, CENTRE, LO, R, RES, UP, clearance, occupied
from log_window import events

LATERAL_REACH_M = 1.5


def wall_clearance(x: float, y: float, z: float) -> float:
    """The body's horizontal clearance to the truly occupied voxels at its height."""
    best = LATERAL_REACH_M
    i0 = int(np.floor((x - R - LATERAL_REACH_M - B.origin_x_m) / RES))
    i1 = int(np.floor((x + R + LATERAL_REACH_M - B.origin_x_m) / RES))
    j0 = int(np.floor((y - R - LATERAL_REACH_M - B.origin_y_m) / RES))
    j1 = int(np.floor((y + R + LATERAL_REACH_M - B.origin_y_m) / RES))
    k0 = int(np.floor((z - LO + 0.05 - B.origin_z_m) / RES))
    k1 = int(np.floor((z + UP - B.origin_z_m) / RES))
    for i in range(i0, i1 + 1):
        dx = max(B.origin_x_m + i * RES - x, 0.0, x - (B.origin_x_m + (i + 1) * RES))
        for j in range(j0, j1 + 1):
            dy = max(B.origin_y_m + j * RES - y, 0.0, y - (B.origin_y_m + (j + 1) * RES))
            radial = max(0.0, np.hypot(dx, dy) - R)
            if radial >= best:
                continue
            if any(occupied(i, j, k) for k in range(k0, k1 + 1)):
                best = radial
    return best


def report(run: str) -> str:
    # The event times to the hundredth of a second, as the log window prints them: the figures the
    # documents cite were taken that way.
    found = [(round(s, 2), text) for s, text in
             events(run, ("DEAD_RECKONING started=true", "VEHICLE_LANDED",
                          "GOAL_UNREACHABLE trigger=unreliable_light"))]
    landed = [s for s, text in found if "VEHICLE_LANDED" in text]
    if not landed:
        return f"{run}: no landing"
    landing = landed[0]
    dead_reckoning = [s for s, text in found if "started=true" in text and s < landing][-1]
    verdicts = [s for s, text in found if "unreliable_light" in text and s >= dead_reckoning]
    verdict = verdicts[0] if verdicts else dead_reckoning + 3.0
    descent = dead_reckoning + 3.0
    pose = np.array([[float(v) for v in r[:9]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv"))])
    truth = pose[:, :4]
    estimate = np.array([[float(v) for v in r[:4]]
                         for r in csv.reader(open(f"log/runs/{run}/vio_estimate.csv"))])

    def at(track, s):
        return track[min(np.searchsorted(track[:, 0], s), len(track) - 1)][1:4]

    def velocity(track, s):
        return (at(track, s + 0.25) - at(track, s - 0.25)) / 0.5

    # The plan flown on the dead-reckoned estimate, to the verdict.
    tracking = np.load(f"log/runs/{run}/tracking.npz")
    received = np.interp(tracking["sp_received_s"], pose[:, 8], pose[:, 0])
    setpoints = tracking["sp"]
    planned = (received >= dead_reckoning) & (received < verdict) & np.isfinite(setpoints[:, 1])
    horizontal = np.hypot(setpoints[planned, 4], setpoints[planned, 5])
    segment = truth[(truth[:, 0] >= dead_reckoning) & (truth[:, 0] <= verdict)]
    path = np.sum(np.linalg.norm(np.diff(segment[:, 1:3], axis=0), axis=1)) if len(segment) > 1 else 0.0
    before = np.linalg.norm(at(truth, verdict)[:2] - at(truth, dead_reckoning)[:2])
    # The descent.
    floor_z = at(truth, landing)[2]
    speed = np.linalg.norm(velocity(truth, descent)[:2])
    error = np.linalg.norm((velocity(estimate, descent) - velocity(truth, descent))[:2])
    drift = np.linalg.norm(at(truth, landing)[:2] - at(truth, descent)[:2])
    whole = np.linalg.norm(at(truth, landing)[:2] - at(truth, dead_reckoning)[:2])
    # Clearances to a second before the touchdown.
    window = truth[(truth[:, 0] >= dead_reckoning) & (truth[:, 0] <= landing - 1.0)][::3]
    least = min((clearance(x, y, z + CENTRE), t) for t, x, y, z in window)
    walls = min((wall_clearance(x, y, z + CENTRE), t) for t, x, y, z in window)
    return (f"{run}: DR {dead_reckoning:.1f}, verdict {verdict:.1f} (+{verdict - dead_reckoning:.1f}), descent "
            f"{descent:.1f}, landed {landing:.1f}; to the verdict {int(planned.sum())} planned setpoints up to "
            f"{horizontal.max() if planned.any() else 0.0:.2f} m/s, truth path {path:.2f} m, displacement "
            f"{before:.2f} m; height {at(truth, dead_reckoning)[2] - floor_z:.2f} -> "
            f"{at(truth, descent)[2] - floor_z:.2f} m, at the descent {speed:.2f} m/s true, error {error:.2f} m/s; "
            f"descent {drift:.2f} m in {landing - descent:.1f} s ({drift / (landing - descent):.2f} m/s); "
            f"whole {whole:.2f} m; least clearance {least[0]:.2f} m at {least[1]:.1f} s, walls "
            f"{walls[0]:.2f} m at {walls[1] - dead_reckoning:.1f} s after DR")


if __name__ == "__main__":
    for run in sys.argv[1:]:
        print(report(run), flush=True)
