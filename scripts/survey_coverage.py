#!/usr/bin/env python3
"""The survey's figures read from a flight's records (roadmap item 34): the frontiers left in the memory's
last snapshot inside the volume, and the coverage of the truth. Shared by the mission check and
tools/frontiers.py. NumPy alone: the container's scipy.ndimage does not import beside its numpy.

The memory's last raw snapshot (log/runs/RUN/raw_snapshot_3d_revision_*.json) is read on its own lattice
(0.25 m). A frontier is an observed free cell inside the volume with a 6-neighbour unknown inside the volume
the body fits in (the volume's face is not a frontier: the volume is the mission's order). Clusters are 26-connected
components of at least `cluster_minimum` cells; a cluster's goal is its vantage, the cell of the reachable
component (the flood of K16 from the vehicle's last position through the cells the body fits in, no occupied
cell within `body` of them, unknown included) nearest the cluster's centroid within `reach` of it. Two
coverages: of the truth's flyable cells inside the volume within `reach` of the flown true path
(gz_pose.csv), the share the memory observed (the path's coverage: what the sensor took in where it went);
and of the truth's flyable cells inside the volume reachable from the path, the share observed (the
volume's coverage: how much of the ordered volume the survey took in)."""
from __future__ import annotations

import csv
import json
from collections import deque
from pathlib import Path

import numpy as np

from validate_static_cooperative_scenario import Occupancy3D

CHUNK = 16
FACES = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))


def dense_words(words):
    bits = np.unpackbits(np.frombuffer(np.array(words, dtype="<u8").tobytes(), dtype=np.uint8), bitorder="little")
    return bits.reshape(CHUNK, CHUNK, CHUNK).transpose(2, 1, 0).astype(bool)


def load_memory(path: Path, volume):
    """The snapshot's observed and occupied cells inside the volume, on the memory's lattice."""
    document = json.loads(path.read_text())
    grid = document["grid"]
    res = float(grid["resolution_m"])
    origin = np.array(grid["origin_m"], dtype=float)
    if volume is None:
        volume = document["selection"]["bounds_m"]
    if volume is None:
        raise ValueError("the snapshot holds no selection: a volume is needed")
    lo = np.floor((np.array(volume[:3]) - origin) / res + 1e-6).astype(int)
    hi = np.ceil((np.array(volume[3:]) - origin) / res - 1e-6).astype(int)
    shape = tuple(hi - lo)
    observed = np.zeros(shape, dtype=bool)
    occupied = np.zeros(shape, dtype=bool)
    for chunk in document["chunks"]:
        c0 = np.array(chunk["index"]) * CHUNK
        a = np.maximum(c0, lo)
        b = np.minimum(c0 + CHUNK, hi)
        if np.any(b <= a):
            continue
        for key, target in (("observed_words_hex", observed), ("occupied_words_hex", occupied)):
            block = dense_words([int(w, 16) for w in chunk[key]])
            target[a[0] - lo[0]:b[0] - lo[0], a[1] - lo[1]:b[1] - lo[1], a[2] - lo[2]:b[2] - lo[2]] |= \
                block[a[0] - c0[0]:b[0] - c0[0], a[1] - c0[1]:b[1] - c0[1], a[2] - c0[2]:b[2] - c0[2]]
    return document["revision"], res, origin + lo * res, observed, occupied


def load_truth(path: Path, res, origin, shape):
    """The truth's occupied cells sampled on the memory's lattice (the truth is coarser)."""
    grid = Occupancy3D.load(path)
    bounds = grid.bounds
    occupied = np.zeros(shape, dtype=bool)
    xs = origin[0] + (np.arange(shape[0]) + 0.5) * res
    ys = origin[1] + (np.arange(shape[1]) + 0.5) * res
    zs = origin[2] + (np.arange(shape[2]) + 0.5) * res
    ix = np.floor((xs - bounds.origin_x_m) / bounds.resolution_m).astype(int)
    iy = np.floor((ys - bounds.origin_y_m) / bounds.resolution_m).astype(int)
    iz = np.floor((zs - bounds.origin_z_m) / bounds.resolution_m).astype(int)
    for (cx, cy, cz), words in grid.chunks.items():
        block = dense_words(words)
        sx = np.nonzero((ix >= cx * CHUNK) & (ix < (cx + 1) * CHUNK))[0]
        sy = np.nonzero((iy >= cy * CHUNK) & (iy < (cy + 1) * CHUNK))[0]
        sz = np.nonzero((iz >= cz * CHUNK) & (iz < (cz + 1) * CHUNK))[0]
        if not (len(sx) and len(sy) and len(sz)):
            continue
        sub = block[np.ix_(ix[sx] - cx * CHUNK, iy[sy] - cy * CHUNK, iz[sz] - cz * CHUNK)]
        occupied[np.ix_(sx, sy, sz)] = sub
    return occupied


def ball_offsets(radius_cells: float):
    reach = int(np.ceil(radius_cells - 1e-9))
    grid = np.mgrid[-reach:reach + 1, -reach:reach + 1, -reach:reach + 1].reshape(3, -1).T
    return grid[(grid ** 2).sum(axis=1) <= radius_cells ** 2 + 1e-9]


def shifted(mask, dx, dy, dz):
    """The mask moved by the offset, what moves out lost, what moves in false."""
    result = np.zeros_like(mask)
    nx, ny, nz = mask.shape
    sx = slice(max(dx, 0), nx + min(dx, 0))
    sy = slice(max(dy, 0), ny + min(dy, 0))
    sz = slice(max(dz, 0), nz + min(dz, 0))
    tx = slice(max(-dx, 0), nx + min(-dx, 0))
    ty = slice(max(-dy, 0), ny + min(-dy, 0))
    tz = slice(max(-dz, 0), nz + min(-dz, 0))
    result[sx, sy, sz] = mask[tx, ty, tz]
    return result


def dilate(mask, offsets):
    result = np.zeros_like(mask)
    for dx, dy, dz in offsets:
        result |= shifted(mask, int(dx), int(dy), int(dz))
    return result


def body_fits(occupied, res, body):
    """No occupied cell within the body's radius (K16's rule)."""
    return ~dilate(occupied, ball_offsets(body / res))


def flood(fits, seeds):
    """The connected component of the body-fitting cells holding any seed (6-connected), a level a step."""
    reachable = np.zeros_like(fits)
    for seed in seeds:
        if fits[seed] and not reachable[seed]:
            reachable[seed] = True
    level = reachable.copy()
    while level.any():
        grown = np.zeros_like(fits)
        for dx, dy, dz in FACES:
            grown |= shifted(level, dx, dy, dz)
        level = grown & fits & ~reachable
        reachable |= level
    return reachable


def frontier_cells(observed, occupied, fits):
    """Observed free cells beside unknown the body fits in (K16's rule, the survey node's): a hole a ray left
    in a wall or a floor is no frontier."""
    free = observed & ~occupied
    open_unknown = ~observed & fits
    frontier = np.zeros_like(free)
    for dx, dy, dz in FACES:
        # The volume's face is not a frontier: nothing moves in from outside.
        frontier |= free & shifted(open_unknown, dx, dy, dz)
    return frontier


def label_clusters(frontier):
    """The 26-connected clusters of the frontier cells, as lists of cells."""
    cells = {tuple(c) for c in np.argwhere(frontier)}
    clusters = []
    while cells:
        seed = cells.pop()
        members = [seed]
        queue = deque([seed])
        while queue:
            x, y, z = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        n = (x + dx, y + dy, z + dz)
                        if n in cells:
                            cells.remove(n)
                            members.append(n)
                            queue.append(n)
        clusters.append(np.array(members))
    return clusters


def clusters(frontier, reachable, minimum, res, reach):
    """The clusters and their vantages: the reachable cell nearest the cluster's centroid, within the sensor's
    reach of it (a frontier cell itself lies at a wall more often than not, r2758: clearance 0.25 m on 20 of
    22 clusters; the body looks at it from where it fits)."""
    reach_cells = int(np.ceil(reach / res))
    shape = reachable.shape
    result = []
    for cells in label_clusters(frontier):
        if len(cells) < minimum:
            continue
        centroid = cells.mean(axis=0)
        near = np.rint(centroid).astype(int)
        lo = np.maximum(near - reach_cells, 0)
        hi = np.minimum(near + reach_cells + 1, shape)
        window = reachable[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
        goal = None
        vantage_m = None
        if window.any():
            candidates = np.argwhere(window) + lo
            distances = np.linalg.norm((candidates - centroid) * res, axis=1)
            best = int(np.argmin(distances))
            vantage_m = round(float(distances[best]), 2)
            if distances[best] <= reach:
                goal = candidates[best]
        result.append({"size": int(len(cells)), "centroid": centroid, "goal": goal, "vantage_m": vantage_m})
    return result


def true_path(run_directory: Path):
    path = []
    pose = run_directory / "gz_pose.csv"
    if pose.is_file():
        with pose.open(newline="") as stream:
            for row in csv.reader(stream):
                if row and row[0][:1].isdigit():
                    path.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(path) if path else np.zeros((0, 3))


def last_snapshot(run_directory: Path) -> Path | None:
    snapshots = sorted(run_directory.glob("raw_snapshot_3d_revision_*.json"),
                       key=lambda p: int(p.stem.rsplit("_", 1)[1]))
    return snapshots[-1] if snapshots else None


def near_path_mask(shape, res, path_cells, reach):
    """The cells within the reach of the path: the distance to the path's samples on a 1 m lattice, the
    lattice's verdict given to the cells it holds."""
    samples = np.array(sorted({tuple(c) for c in path_cells}), dtype=float)
    step = max(1, int(round(1.0 / res)))
    coarse_shape = tuple((s + step - 1) // step for s in shape)
    centres = np.mgrid[0:coarse_shape[0], 0:coarse_shape[1], 0:coarse_shape[2]].reshape(3, -1).T * step
    near_coarse = np.zeros(len(centres), dtype=bool)
    reach_cells = reach / res + step
    for start in range(0, len(samples), 64):
        batch = samples[start:start + 64]
        d = np.sqrt(((centres[:, None, :] - batch[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
        near_coarse |= d <= reach_cells
    near = np.repeat(np.repeat(np.repeat(near_coarse.reshape(coarse_shape), step, axis=0), step, axis=1),
                     step, axis=2)
    return near[:shape[0], :shape[1], :shape[2]]


def survey_figures(run_directory: Path, truth: Path | None, volume, reach: float, body: float = 0.55,
                   cluster_minimum: int = 8) -> dict:
    """The frontiers left and the coverages of one flight; `volume` None reads the snapshot's selection."""
    snapshot = last_snapshot(run_directory)
    if snapshot is None:
        raise FileNotFoundError(f"{run_directory}: no raw snapshot")
    revision, res, origin, observed, occupied = load_memory(snapshot, volume)
    shape = observed.shape
    path = true_path(run_directory)
    cells = [tuple(np.floor((p - origin) / res).astype(int)) for p in path]
    inside = [c for c in cells if all(0 <= c[i] < shape[i] for i in range(3))]
    fits = body_fits(occupied, res, body)
    # The vehicle: the last sample inside the volume the body fits at (a landed vehicle sits on the pad).
    airborne = [c for c in inside if fits[c]]
    vehicle = airborne[-1] if airborne else None
    reachable = flood(fits, [vehicle]) if vehicle is not None else fits
    frontier = frontier_cells(observed, occupied, fits)
    cluster_list = clusters(frontier, reachable, cluster_minimum, res, reach)
    with_goal = [c for c in cluster_list if c["goal"] is not None]
    figures = {"run": run_directory.name, "revision": revision, "resolution_m": res,
               "origin_m": origin.tolist(), "shape": list(shape), "volume_cells": int(np.prod(shape)),
               "observed": int(observed.sum()), "occupied": int(occupied.sum()),
               "frontier_cells": int(frontier.sum()), "clusters": len(cluster_list),
               "clusters_with_goal": len(with_goal), "reachable_cells": int(reachable.sum()),
               "frontier_in_reachable": int((frontier & reachable).sum()),
               "cluster_sizes": sorted((c["size"] for c in cluster_list), reverse=True)[:12],
               "path_samples_inside": len(inside),
               "path_m": float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0,
               "observed_arrays": (observed, occupied, fits, reachable, frontier, cluster_list, vehicle)}
    if truth is not None and truth.is_file() and inside:
        truth_occupied = load_truth(truth, res, origin, shape)
        truth_fits = body_fits(truth_occupied, res, body)
        truth_reachable = flood(truth_fits, [c for c in inside if truth_fits[c]])
        target = truth_reachable & near_path_mask(shape, res, inside, reach)
        figures["truth_target_cells"] = int(target.sum())
        figures["coverage"] = round(float((observed & target).sum() / max(target.sum(), 1)), 4)
        figures["truth_reachable_cells"] = int(truth_reachable.sum())
        figures["volume_coverage"] = round(
            float((observed & truth_reachable).sum() / max(truth_reachable.sum(), 1)), 4)
        # The memory's occupied cells against the truth: a check of the frames.
        figures["occupied_agreeing_with_truth"] = round(float(
            (occupied & dilate(truth_occupied, ball_offsets(0.5 / res))).sum() / max(occupied.sum(), 1)), 3)
    return figures
