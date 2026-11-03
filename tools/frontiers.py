#!/usr/bin/env python3
"""frontiers.py RUN [RUN...] [--volume X0,Y0,Z0,X1,Y1,Z1] [--truth GRID] [--reach M] [--body M] [--cluster-min N]
[--rules]: the frontiers of a flight's obstacle memory and the coverage of the truth (roadmap item 34 step 1).

The last raw snapshot of each run (log/runs/RUN/raw_snapshot_3d_revision_*.json) is read on its own lattice
(0.25 m). A frontier is an observed free cell inside the volume with a 6-neighbour unknown inside the volume
(the volume's face is not a frontier: the volume is the mission's order). Clusters are 26-connected components
of at least --cluster-min cells; a cluster's goal is its cell nearest its centroid with the body's clearance
(no occupied cell within --body) that lies in the reachable component (the flood of K16 from the vehicle's
last position through the cells the body fits in, unknown included). Coverage: of the truth's flyable cells
inside the volume within --reach of the flown true path (gz_pose.csv), the share the memory observed. With
--rules the three value rules are walked greedily over the cluster goals from the vehicle's last position
(the route's length through the body-fitting cells; a cluster within --reach of the one visited counts as
seen): nearest by route, largest unknown within --reach of the goal, nearest by the straight line. The
volume defaults to the snapshot's selection (the recorded flights hold the route volume alone). Host."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from survey_coverage import FACES, shifted, survey_figures  # noqa: E402

URBAN_TRUTH = "external/environment-candidates/work/urban_practice_01_r050.occupancy3d"


def route_lengths(fits, source, res):
    """The route's length from the source cell to every cell through the body-fitting cells: a 6-connected
    breadth-first search, one lattice step a level (the whole array a level: a Python queue took minutes)."""
    distance = np.full(fits.shape, np.inf)
    if not fits[source]:
        return distance
    visited = np.zeros(fits.shape, dtype=bool)
    visited[source] = True
    level = visited.copy()
    distance[source] = 0.0
    step = 0
    while level.any():
        step += 1
        grown = np.zeros_like(level)
        for dx, dy, dz in FACES:
            grown |= shifted(level, dx, dy, dz)
        level = grown & fits & ~visited
        visited |= level
        distance[level] = step * res
    return distance


def greedy_tours(cluster_list, fits, start, res, reach_cells, observed):
    goals = [c for c in cluster_list if c["goal"] is not None]
    unknown = ~observed
    behind = [int(unknown[max(0, g["goal"][0] - reach_cells):g["goal"][0] + reach_cells + 1,
                          max(0, g["goal"][1] - reach_cells):g["goal"][1] + reach_cells + 1,
                          max(0, g["goal"][2] - reach_cells):g["goal"][2] + reach_cells + 1].sum()) for g in goals]
    results = {}
    for rule in ("route", "unknown", "euclid"):
        left = list(range(len(goals)))
        position = start
        metres = 0.0
        visits = 0
        while left:
            lengths = route_lengths(fits, position, res)
            reachable_left = [i for i in left if np.isfinite(lengths[tuple(goals[i]["goal"])])]
            if not reachable_left:
                break
            if rule == "route":
                pick = min(reachable_left, key=lambda i: lengths[tuple(goals[i]["goal"])])
            elif rule == "unknown":
                pick = max(reachable_left, key=lambda i: behind[i] / max(lengths[tuple(goals[i]["goal"])], res))
            else:
                pick = min(reachable_left, key=lambda i: float(np.linalg.norm(goals[i]["goal"] - np.array(position))))
            goal = tuple(int(v) for v in goals[pick]["goal"])
            metres += float(lengths[goal])
            visits += 1
            position = goal
            left = [i for i in left if float(np.linalg.norm(goals[i]["goal"] - np.array(goal))) > reach_cells]
        results[rule] = {"metres": round(metres, 1), "visits": visits, "left": len(left)}
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--volume", type=lambda s: [float(v) for v in s.split(",")])
    parser.add_argument("--truth", type=Path, default=Path(URBAN_TRUTH))
    parser.add_argument("--reach", type=float, default=6.4)
    parser.add_argument("--body", type=float, default=0.55)
    parser.add_argument("--cluster-min", type=int, default=8)
    parser.add_argument("--rules", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    for run in args.runs:
        directory = Path("log/runs") / run
        try:
            report = survey_figures(directory, args.truth, args.volume, args.reach, args.body, args.cluster_min)
        except FileNotFoundError as error:
            print(error)
            continue
        observed, occupied, fits, reachable, frontier, cluster_list, vehicle = report.pop("observed_arrays")
        res = report["resolution_m"]
        if args.rules and vehicle is not None:
            report["tours"] = greedy_tours(cluster_list, fits, vehicle, res, int(round(args.reach / res)), observed)
        if args.json:
            print(json.dumps(report, default=lambda v: v.tolist() if hasattr(v, "tolist") else str(v)))
        else:
            print(f"{run} rev {report['revision']}: volume {tuple(report['shape'])} observed {report['observed']} occupied {report['occupied']} "
                  f"frontier cells {report['frontier_cells']} clusters {report['clusters']} "
                  f"(with goal {report['clusters_with_goal']}) sizes {report['cluster_sizes']}"
                  + (f" | truth target {report['truth_target_cells']} coverage {report['coverage']}"
                     f" occupied agreeing {report['occupied_agreeing_with_truth']}" if "coverage" in report else "")
                  + (f" | tours {report['tours']}" if "tours" in report else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
