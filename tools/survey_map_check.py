#!/usr/bin/env python3
"""survey_map_check.py MAP_DIR --truth GRID [--run RUN] [--body M] [--path-margin M]: a survey map (tools/survey_map.py)
against the world's truth voxelization (roadmap item 11's validation: coverage, alignment, raw collision).

The map (occupancy.occupancy3d, observed.occupancy3d, map_manifest.json) and the truth are read inside the map's
volume on the coarser of the two lattices (the urban truth is 0.25 m like the memory; the held-out worlds' truths are
0.5 m: the map is then pooled, a coarse cell occupied or observed when any of its cells is), the truth in the SDF frame
by the manifest's offset.

  coverage    the truth's flyable cells inside the volume the body reaches from the flown path (the flood of
              scripts/survey_coverage.py), the share the map observed
  alignment   from every occupied cell of the map, the distance to the nearest occupied cell of the truth: the median,
              p95 and the share beyond one cell; the frame shift as the mean vector to those nearest cells
  collision   the truth's occupied cells inside the volume that bound the reachable space (within the body's radius
              and a cell of a reachable cell): how many the map holds occupied, observed free ("false free": told
              sliver from missing surface by its depth off the map's own surface: the voxelization marks a cell at
              any touch of a mesh, the sensor a cell a ray stopped in), unobserved.
              Beside them the map's own flyable cells (no occupied cell of the map within the body's radius) that the
              truth holds occupied, "false flyable": where a flight by the map would put the body into a surface; the
              ones within the body's radius of the flown path (plus --path-margin, none by default) must be zero: the
              path flown was clear. The count within half a metre more is reported beside it.

The verdict is PASS when no false-flyable cell lies by the flown path; the other figures are reported for the K row
that sets their thresholds from the first measurement. Host (SciPy).

  python3 tools/survey_map_check.py log/tools/wbsm/part2/maps/urban_lidar_r3168 \\
      --truth log/tools/truth25/urban_r025.occupancy3d --run r3168
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))
import survey_coverage  # noqa: E402
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D  # noqa: E402


def dense_in_box(grid: Occupancy3D, lo: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    """The grid's bits inside a box of cells (lo, shape) on its own lattice."""
    out = np.zeros(shape, dtype=bool)
    hi = lo + np.array(shape)
    for (cx, cy, cz), words in grid.chunks.items():
        c0 = np.array([cx, cy, cz]) * CHUNK_SIZE
        a = np.maximum(c0, lo)
        b = np.minimum(c0 + CHUNK_SIZE, hi)
        if np.any(b <= a):
            continue
        block = survey_coverage.dense_words(list(words))
        out[a[0] - lo[0]:b[0] - lo[0], a[1] - lo[1]:b[1] - lo[1], a[2] - lo[2]:b[2] - lo[2]] |= \
            block[a[0] - c0[0]:b[0] - c0[0], a[1] - c0[1]:b[1] - c0[1], a[2] - c0[2]:b[2] - c0[2]]
    return out


def pool(mask: np.ndarray, factor: int) -> np.ndarray:
    """A coarse cell holds the mask when any of its fine cells does."""
    if factor == 1:
        return mask
    shape = tuple((s + factor - 1) // factor * factor for s in mask.shape)
    padded = np.zeros(shape, dtype=bool)
    padded[:mask.shape[0], :mask.shape[1], :mask.shape[2]] = mask
    return padded.reshape(shape[0] // factor, factor, shape[1] // factor, factor,
                          shape[2] // factor, factor).any(axis=(1, 3, 5))


def check(map_directory: Path, truth_path: Path, run_directory: Path | None, body: float = 0.55,
          path_margin: float = 0.0, near_margin: float = 0.5) -> dict:
    from scipy import ndimage

    manifest = json.loads((map_directory / "map_manifest.json").read_text())
    volume = np.array(manifest["volume_m"], dtype=float)
    offset = np.array(manifest["frame"]["map_to_sdf_offset_m"], dtype=float)
    occupancy = Occupancy3D.load(map_directory / "occupancy.occupancy3d")
    observed_grid = Occupancy3D.load(map_directory / "observed.occupancy3d")
    truth = Occupancy3D.load(truth_path)
    map_res = occupancy.bounds.resolution_m
    truth_res = truth.bounds.resolution_m
    if truth_res + 1e-9 < map_res or abs(truth_res / map_res - round(truth_res / map_res)) > 1e-6:
        raise ValueError(f"the truth's resolution {truth_res} is not a multiple of the map's {map_res}")
    factor = int(round(truth_res / map_res))
    res = truth_res
    # The volume on the truth's lattice (SDF frame), and the same box on the map's (map frame).
    truth_origin = np.array([truth.bounds.origin_x_m, truth.bounds.origin_y_m, truth.bounds.origin_z_m])
    t_lo = np.floor((volume[:3] + offset - truth_origin) / res + 1e-6).astype(int)
    t_hi = np.ceil((volume[3:] + offset - truth_origin) / res - 1e-6).astype(int)
    shape = tuple(int(v) for v in (t_hi - t_lo))
    map_origin = np.array([occupancy.bounds.origin_x_m, occupancy.bounds.origin_y_m, occupancy.bounds.origin_z_m])
    box_origin_map = truth_origin + t_lo * res - offset
    m_lo = np.round((box_origin_map - map_origin) / map_res).astype(int)
    fine_shape = tuple(s * factor for s in shape)
    map_occupied = pool(dense_in_box(occupancy, m_lo, fine_shape), factor)
    map_observed = pool(dense_in_box(observed_grid, m_lo, fine_shape), factor) | map_occupied
    truth_occupied = dense_in_box(truth, t_lo, shape)
    truth_free = ~truth_occupied
    result: dict = {"resolution_m": res, "volume_cells": int(np.prod(shape)),
                    "truth_occupied": int(truth_occupied.sum()), "map_occupied": int(map_occupied.sum()),
                    "map_observed": int(map_observed.sum())}
    # Coverage: the truth's flyable cells the body reaches from the flown path, the share observed.
    fits = survey_coverage.body_fits(truth_occupied, res, body)
    # The flown path's cells inside the volume: every one of them for what lies by the path, those the body fits
    # at for the flood's seeds (the model's 0.55 m overlaps the voxelization's slivers where the airframe passed).
    path_cells: list[tuple[int, int, int]] = []
    flown_cells: list[tuple[int, int, int]] = []
    if run_directory is not None:
        path = survey_coverage.true_path(run_directory)
        cells = np.floor((path + offset - (truth_origin + t_lo * res)) / res).astype(int)
        inside = cells[np.all((cells >= 0) & (cells < np.array(shape)), axis=1)]
        flown_cells = list(dict.fromkeys(tuple(int(v) for v in c) for c in inside))
        path_cells = [c for c in flown_cells if fits[c]]
    if path_cells:
        reachable = survey_coverage.flood(fits, list(dict.fromkeys(path_cells)))
        result["truth_reachable_cells"] = int(reachable.sum())
        result["coverage"] = float((reachable & map_observed).sum() / max(1, reachable.sum()))
    else:
        reachable = fits
        result["truth_reachable_cells"] = int(fits.sum())
        result["coverage"] = float((fits & map_observed).sum() / max(1, fits.sum()))
        result["coverage_note"] = "no flown path inside the volume: the whole flyable truth counted"
    # Alignment: the map's occupied cells against the truth's nearest occupied ones.
    if truth_occupied.any() and map_occupied.any():
        distance, indices = ndimage.distance_transform_edt(~truth_occupied, return_indices=True)
        at = distance[map_occupied] * res
        nearest = np.stack([indices[axis][map_occupied] for axis in range(3)], axis=1)
        own = np.argwhere(map_occupied)
        shift = (nearest - own).mean(axis=0) * res
        result["alignment"] = {"median_m": float(np.median(at)), "p95_m": float(np.percentile(at, 95)),
                               "beyond_one_cell_share": float((at > res + 1e-9).mean()),
                               "mean_shift_m": [float(v) for v in shift]}
    # Raw collision: the truth's occupied cells that bound the reachable space (within the body's radius and a
    # cell of a reachable cell: the body flying there can meet them), and what the map says of each.
    free_reachable = reachable & truth_free
    grown = survey_coverage.dilate(free_reachable, survey_coverage.ball_offsets(body / res + 1.0))
    bounding = truth_occupied & grown
    held = bounding & map_occupied
    false_free = bounding & map_observed & ~map_occupied
    unseen = bounding & ~map_observed
    # How deep a false-free cell lies off the map's own surface: within a cell of a map-occupied cell it is a
    # sliver of the mesh the voxelization marks and the sensor does not; farther, a surface the map lacks.
    if false_free.any() and map_occupied.any():
        off_map = ndimage.distance_transform_edt(~map_occupied)[false_free] * res
        depth = {"within_one_cell": int((off_map <= res + 1e-9).sum()),
                 "beyond_one_cell": int((off_map > res + 1e-9).sum()), "p95_m": float(np.percentile(off_map, 95))}
    else:
        depth = {"within_one_cell": 0, "beyond_one_cell": 0, "p95_m": 0.0}
    # The map's own flyable cells (no map-occupied cell within the body's radius) that the truth holds occupied:
    # where a flight by the map would put the body into a surface. The risk of the map, and the figure that must be
    # zero by the flown path.
    map_fits = survey_coverage.body_fits(map_occupied, res, body)
    false_flyable = map_fits & truth_occupied
    collision = {"bounding_cells": int(bounding.sum()), "held_occupied": int(held.sum()),
                 "false_free": int(false_free.sum()), "false_free_depth": depth, "unobserved": int(unseen.sum()),
                 "false_flyable": int(false_flyable.sum()),
                 "false_flyable_observed": int((false_flyable & map_observed).sum())}
    if path_cells:
        near = np.zeros(shape, dtype=bool)
        for c in flown_cells:
            near[c] = True
        by_path = survey_coverage.dilate(near, survey_coverage.ball_offsets((body + path_margin) / res))
        collision["false_free_by_the_path"] = int((false_free & by_path).sum())
        collision["false_flyable_by_the_path"] = int((false_flyable & by_path).sum())
        collision["unobserved_by_the_path"] = int((unseen & by_path).sum())
        collision["path_margin_m"] = body + path_margin
        # For the record, the same within a wider margin of the path (the body's radius and half a metre).
        wider = survey_coverage.dilate(near, survey_coverage.ball_offsets((body + near_margin) / res))
        collision["false_flyable_near_the_path"] = int((false_flyable & wider).sum())
        collision["near_margin_m"] = body + near_margin
    result["collision"] = collision
    if not path_cells:
        result["verdict"] = "NO PATH"
    else:
        result["verdict"] = "PASS" if collision["false_flyable_by_the_path"] == 0 else "FAIL"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_directory", type=Path)
    parser.add_argument("--truth", required=True, type=Path)
    parser.add_argument("--run")
    parser.add_argument("--body", type=float, default=0.55)
    parser.add_argument("--path-margin", type=float, default=0.0)
    args = parser.parse_args()
    run_directory = REPOSITORY / "log/runs" / args.run if args.run else None
    result = check(args.map_directory, args.truth, run_directory, args.body, args.path_margin)
    print(json.dumps(result, indent=2))
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
