#!/usr/bin/env python3
"""Whether a point lies inside a world, read from the world's truth voxelization by rays (specification K63).

A world of tunnels, caverns or rooms is a set of shells in an empty grid. The void round the shells is free space
too, roofed by the tunnels above it and floored by the ones below: a start drawn from "the largest free space under a
ceiling" stood on the outside of a tunnel's shell on both held-out worlds until 2026-11-05 (roadmap item 33). What
tells the two apart at a point is what the point sees: from inside a world every direction meets a surface near by;
from the void most directions run off.

The measure: rays cast from the point, evenly over the sphere, and the share of them that meet an occupied voxel
within the reach. Measured on the finals world at a reach of 40 m over 1500 points each, clear of the walls and within
6 m of one: inside, 0.99 at the first percentile; outside, 0.63 at the 99th; the start flown until 2026-11-05 read
0.62 and its goal 0.49. The Urban world's inside is joined to the grid's faces (its staging area is open), so no
flood tells its inside from the void, and its start and goal read 1.00. A point is inside at 0.90 or more.

Used by tools/scenario_inside.py (the scenario's proof, on the host) and by the flight's check
(validate_drone_nav_headless.py: a scenario that carries the proof is measured again on the truth grid it flew in).
NumPy alone: the dev container's Python loads no SciPy.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

RAYS = 128
REACH_M = 40.0
ENCLOSED_SHARE = 0.90
TAKEOFF_CLIMB_M = 2.0


def ray_directions(count: int = RAYS):
    """Unit vectors spread evenly over the sphere (a Fibonacci lattice)."""
    import numpy as np  # noqa: PLC0415

    index = np.arange(count) + 0.5
    polar = np.arccos(1.0 - 2.0 * index / count)
    azimuth = math.pi * (1.0 + 5.0 ** 0.5) * index
    return np.stack([np.cos(azimuth) * np.sin(polar), np.sin(azimuth) * np.sin(polar), np.cos(polar)], axis=1)


def ray_enclosure(occupied, cells, reach_cells: int, directions=None):
    """For each cell, the share of the rays that meet an occupied voxel within the reach; a ray that leaves the
    grid first has met nothing."""
    import numpy as np  # noqa: PLC0415

    if directions is None:
        directions = ray_directions()
    cells = np.asarray(cells, dtype=int).reshape(-1, 3)
    points = cells.astype(float) + 0.5
    shape = np.array(occupied.shape)
    met = np.zeros((len(cells), len(directions)), dtype=bool)
    alive = np.ones_like(met)
    for step in range(1, reach_cells + 1):
        voxels = np.floor(points[:, None, :] + directions[None, :, :] * step).astype(int)
        alive &= ((voxels >= 0) & (voxels < shape)).all(axis=2)
        index = voxels[alive]
        struck = np.zeros_like(met)
        struck[alive] = occupied[index[:, 0], index[:, 1], index[:, 2]]
        met |= struck
        alive &= ~struck
        if not alive.any():
            break
    return met.mean(axis=1)


def dense_occupancy(path: Path):
    """The truth grid's bounds and its occupied voxels as a dense array."""
    import numpy as np  # noqa: PLC0415
    from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D  # noqa: PLC0415

    grid = Occupancy3D.load(path)
    bounds = grid.bounds
    occupied = np.zeros((bounds.width, bounds.height, bounds.depth), dtype=bool)
    for (chunk_x, chunk_y, chunk_z), words in grid.chunks.items():
        bits = np.unpackbits(np.frombuffer(np.array(words, dtype="<u8").tobytes(), dtype=np.uint8),
                             bitorder="little")
        block = bits.reshape(CHUNK_SIZE, CHUNK_SIZE, CHUNK_SIZE).transpose(2, 1, 0)
        x0, y0, z0 = chunk_x * CHUNK_SIZE, chunk_y * CHUNK_SIZE, chunk_z * CHUNK_SIZE
        x1 = min(x0 + CHUNK_SIZE, bounds.width)
        y1 = min(y0 + CHUNK_SIZE, bounds.height)
        z1 = min(z0 + CHUNK_SIZE, bounds.depth)
        occupied[x0:x1, y0:y1, z0:z1] |= block[:x1 - x0, :y1 - y0, :z1 - z0].astype(bool)
    return bounds, occupied


def scenario_points(scenario_path: Path) -> list[tuple[str, tuple[float, ...], tuple[float, ...]]]:
    """The scenario's points to prove: the start after its takeoff climb and every goal, each by its name, its map
    coordinates and its coordinates in the SDF frame the truth grid is in (a frame moved, not turned or scaled)."""
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    world = json.loads((scenario_path.parent / scenario["canonical_world"]).read_text(encoding="utf-8"))
    transform = world["map_to_sdf"]
    if (transform.get("sdf_x_from") != "map_x" or transform.get("sdf_y_from") != "map_y"
            or any(transform.get(f"sdf_{axis}_scale", 1.0) != 1.0 for axis in "xyz")):
        raise ValueError(f"{scenario_path}: a turned or scaled map frame")
    offset = tuple(float(transform.get(f"sdf_{axis}_offset_m", 0.0)) for axis in "xyz")
    start = scenario["vehicle"]["map_start_m"]
    points = [("start", (start[0], start[1], start[2] + TAKEOFF_CLIMB_M))]
    points += [(f"goal {index + 1}", tuple(goal)) for index, goal in enumerate(scenario["mission_goal_sequence_m"])]
    return [(name, tuple(float(v) for v in point), tuple(float(v) + o for v, o in zip(point, offset)))
            for name, point in points]


def cell_of(bounds, point_sdf) -> tuple[int, int, int]:
    origin = (bounds.origin_x_m, bounds.origin_y_m, bounds.origin_z_m)
    return tuple(int(math.floor((point_sdf[axis] - origin[axis]) / bounds.resolution_m)) for axis in range(3))


def enclosure_of_scenario(scenario_path: Path, bounds, occupied) -> list[tuple[str, tuple[float, ...], float]]:
    """Each point of the scenario with its map coordinates and its ray enclosure (0 for a point off the grid or in
    an occupied voxel)."""
    reach_cells = int(round(REACH_M / bounds.resolution_m))
    findings = []
    for name, point_map, point_sdf in scenario_points(scenario_path):
        cell = cell_of(bounds, point_sdf)
        if not all(0 <= cell[axis] < occupied.shape[axis] for axis in range(3)) or occupied[cell]:
            findings.append((name, point_map, 0.0))
            continue
        findings.append((name, point_map, float(ray_enclosure(occupied, [cell], reach_cells)[0])))
    return findings


def validate_scenario_inside(scenario_path: Path, truth_occupancy_path: Path, errors: list[str]) -> None:
    """The flight's check: a scenario that carries the proof of its start and goals (`inside`, written by
    tools/scenario_inside.py) is measured again on the truth grid the flight was judged by. A scenario without the
    proof is not judged here: the open sky has no inside."""
    if not scenario_path.is_file() or not truth_occupancy_path.is_file():
        return
    if "inside" not in json.loads(scenario_path.read_text(encoding="utf-8")):
        return
    bounds, occupied = dense_occupancy(truth_occupancy_path)
    outside = [(name, point, share) for name, point, share in enclosure_of_scenario(scenario_path, bounds, occupied)
               if share < ENCLOSED_SHARE]
    if outside:
        for name, point, share in outside:
            errors.append(f"FAIL: the scenario's {name} lies inside the world (at {point} only {share:.2f} of the rays "
                          f"meet a surface within {REACH_M:.0f} m, {ENCLOSED_SHARE:.2f} asked: the void round the "
                          "world's shells)")
    else:
        print(f"OK: the scenario's start and goals lie inside the world ({ENCLOSED_SHARE:.2f} of the rays or more "
              f"meet a surface within {REACH_M:.0f} m)")
