#!/usr/bin/env python3
"""survey_frontier_parts.py RUN [SNAPSHOT] [--scenario S] [--truth GRID] [--reach M]: why the frontier clusters of a
survey's memory hold no vantage, read off a raw snapshot (the run's last unless given) on the host after the flight
(roadmap item 11, K59). For every 26-connected cluster of frontier cells inside the scenario's volume, and again for
its parts by the cube of the sensor's reach (K59's division): the observed free cells within 6 m of the centroid that
fit the body, of them the ones reached through what was seen from the flown path, of those the ones open round (the
vantage's cylinder), and whether one of them sees the cluster's frontier cell nearest the centroid (the vantage rule
replicated; the node's own counts differ by the cells it has looked at). Then the pockets: the clusters, by the
node's rule of unknown the body fits in, whose unknown is a face-connected piece smaller than the body's ball of
cells (K59 reads no frontier beside one), and the truth's reachable cells of the volume the memory missed, by height.
r3169 read so: one cluster of 32 thousand cells through the hall, its one vantage seeing nothing, the lower level never
looked into; r3225: 55 of 97 clusters beside pockets, 51 of them under four cells. Host (numpy, scipy)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import survey_coverage as sc  # noqa: E402

URBAN_SCENARIO = REPOSITORY / "drone_city_nav/config/urban_circuit_practice_01_survey_scenario.json"
URBAN_TRUTH = REPOSITORY / "log/tools/truth25/urban_r025.occupancy3d"
VANTAGE_REACH_M = 6.0
OPEN_SIDE_M, OPEN_BELOW_M, OPEN_ABOVE_M = 0.8, 1.5, 0.8


def sees(free, res, a, b):
    """The node's line of sight: the first metre skipped, then observed free cells to the target's."""
    a = (np.array(a) + 0.5) * res
    b_c = (np.array(b) + 0.5) * res
    length = float(np.linalg.norm(b_c - a))
    if length < 1e-6:
        return True
    step = 0.5 * res
    for i in range(int(np.ceil((length - 1.0) / step))):
        share = (1.0 + i * step) / length
        if share >= 1.0:
            return True
        cell = tuple(np.floor((a + share * (b_c - a)) / res).astype(int))
        if cell == tuple(b):
            return True
        if not free[cell]:
            return False
    return True


def classify(part, free, passable, known, open_round, res, reach):
    centroid = part.mean(axis=0)
    sight = part[np.argmin(((part - centroid) ** 2).sum(axis=1))]
    near = np.rint(centroid).astype(int)
    lo = np.maximum(near - reach, 0)
    hi = np.minimum(near + reach + 1, free.shape)
    win = tuple(slice(lo[i], hi[i]) for i in range(3))
    grid = np.argwhere(np.ones(tuple(hi - lo), dtype=bool)) + lo
    within = (((grid - centroid) * res) ** 2).sum(axis=1) <= VANTAGE_REACH_M ** 2
    mask = np.zeros(tuple(hi - lo), dtype=bool)
    mask[tuple((grid[within] - lo).T)] = True
    if not (passable[win] & mask).any():
        return "no_passable_near"
    if not (known[win] & mask).any():
        return "none_known"
    candidates = np.argwhere(open_round[win] & known[win] & mask) + lo
    if len(candidates) == 0:
        return "none_open"
    order = np.argsort((((candidates - centroid) * res) ** 2).sum(axis=1))
    return "vantage" if any(sees(free, res, candidates[i], sight) for i in order[:3000]) else "no_sight"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run")
    parser.add_argument("snapshot", nargs="?", type=Path)
    parser.add_argument("--scenario", type=Path, default=URBAN_SCENARIO)
    parser.add_argument("--truth", type=Path, default=URBAN_TRUTH)
    parser.add_argument("--reach", type=float, default=6.4, help="the sensor's reach, the cube of the division")
    parser.add_argument("--body", type=float, default=0.55)
    parser.add_argument("--cluster-min", type=int, default=8)
    args = parser.parse_args()
    run = REPOSITORY / "log/runs" / args.run
    snapshot = args.snapshot or sc.last_snapshot(run)
    volume = np.array(json.loads(args.scenario.read_text())["survey_volume_m"], dtype=float)
    box = np.concatenate([volume[:3] - 8.0, volume[3:] + 8.0])
    box[2] = max(box[2], -1.0)
    revision, res, origin, observed, occupied = sc.load_memory(snapshot, list(box))
    free = observed & ~occupied
    shape = free.shape
    fits = sc.body_fits(occupied, res, args.body)
    lo = np.round((volume[:3] - origin) / res).astype(int)
    hi = np.round((volume[3:] - origin) / res).astype(int)
    inside = np.zeros_like(free)
    inside[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] = True
    unknown_in = ~observed & inside
    frontier = np.zeros_like(free)
    for d in sc.FACES:
        frontier |= free & inside & sc.shifted(unknown_in, *d)
    path = sc.true_path(run)
    seeds = {tuple(np.floor((p - origin) / res).astype(int)) for p in path}
    seeds = [c for c in seeds if all(0 <= c[i] < shape[i] for i in range(3))]
    passable = free & fits
    seed_mask = np.zeros_like(free)
    for c in seeds:
        seed_mask[c] = True
    # The path's own cells are often unobserved (the body's own place): the passable cells within a metre seed.
    known = sc.flood(passable, [tuple(c) for c in np.argwhere(sc.dilate(seed_mask, sc.ball_offsets(1.0 / res)) & passable)])
    side = OPEN_SIDE_M / res
    below, above = int(np.ceil(OPEN_BELOW_M / res - 1e-9)), int(np.ceil(OPEN_ABOVE_M / res - 1e-9))
    cylinder = [(x, y, z) for z in range(-below, above + 1) for y in range(-int(np.ceil(side)), int(np.ceil(side)) + 1)
                for x in range(-int(np.ceil(side)), int(np.ceil(side)) + 1) if x * x + y * y <= side * side + 1e-9]
    open_round = ~sc.dilate(~free, [(-x, -y, -z) for x, y, z in cylinder])
    print(f"{run.name} {snapshot.name}: free {int(free.sum())} passable {int(passable.sum())} known {int(known.sum())} "
          f"open and known {int((open_round & known).sum())} frontier cells in the volume {int(frontier.sum())}")
    reach = int(np.ceil(VANTAGE_REACH_M / res))
    side_cells = int(np.ceil(args.reach / res))
    for title, divide in (("whole clusters", False), (f"parts by the cube of {args.reach} m", True)):
        classes: dict[str, list[int]] = {}
        largest = []
        for cells in sc.label_clusters(frontier):
            if len(cells) < args.cluster_min:
                continue
            parts = [cells]
            if divide and (cells.max(axis=0) - cells.min(axis=0) + 1).max() * res > args.reach:
                keys = cells // side_cells
                parts = [cells[(keys == k).all(axis=1)] for k in np.unique(keys, axis=0)]
            for part in parts:
                kind = "small" if len(part) < args.cluster_min else classify(part, free, passable, known, open_round, res, reach)
                classes.setdefault(kind, []).append(len(part))
                largest.append((len(part), kind, np.round(origin + (part.mean(axis=0) + 0.5) * res, 1).tolist()))
        print(title + ":")
        for kind, sizes in sorted(classes.items(), key=lambda item: -sum(item[1])):
            print(f"  {kind:18s} clusters {len(sizes):4d} cells {sum(sizes):6d} median {int(np.median(sizes))}")
        for size, kind, centroid in sorted(largest, reverse=True)[:6]:
            print(f"    {size:6d} cells {kind:10s} centroid {centroid}")
    # The pockets: by the node's rule (unknown the body fits in), the largest piece of unknown each cluster borders.
    from scipy import ndimage
    labels, _ = ndimage.label(~observed)
    sizes = np.bincount(labels.ravel())
    node_frontier = np.zeros_like(free)
    for d in sc.FACES:
        node_frontier |= free & inside & sc.shifted(unknown_in & fits, *d)
    body_cells = len(sc.ball_offsets(args.body / res))
    pieces = []
    for cells in sc.label_clusters(node_frontier):
        if len(cells) < args.cluster_min:
            continue
        beside: set[int] = set()
        for d in sc.FACES:
            n = cells + np.array(d)
            n = n[((n >= 0) & (n < np.array(shape))).all(axis=1)]
            n = n[fits[tuple(n.T)]]
            found = labels[tuple(n.T)]
            beside.update(int(v) for v in found[found > 0])
        pieces.append(max((int(sizes[b]) for b in beside), default=0))
    print(f"clusters by the node's rule {len(pieces)}: beside a pocket of unknown under the body's {body_cells} cells "
          f"{sum(1 for v in pieces if v < body_cells)}, under 4 cells {sum(1 for v in pieces if v < 4)}")
    if args.truth.is_file() and seeds:
        truth = sc.load_truth(args.truth, res, origin, shape)
        truth_fits = sc.body_fits(truth, res, args.body)
        reachable = sc.flood(truth_fits, [tuple(c) for c in np.argwhere(seed_mask & truth_fits)])
        target = reachable & inside
        missed = target & ~observed
        print(f"truth reachable in the volume {int(target.sum())}, observed {int((target & observed).sum())} "
              f"({(target & observed).sum() / max(target.sum(), 1):.3f}), missed {int(missed.sum())}")
        bins = np.arange(np.floor(volume[2]), np.ceil(volume[5]) + 2.0, 2.0)
        tz = origin[2] + (np.argwhere(target)[:, 2] + 0.5) * res
        mz = origin[2] + (np.argwhere(missed)[:, 2] + 0.5) * res
        print("  z bins     ", bins.tolist())
        print("  target by z", np.histogram(tz, bins=bins)[0].tolist())
        print("  missed by z", np.histogram(mz, bins=bins)[0].tolist())
    return 0


if __name__ == "__main__":
    sys.exit(main())
