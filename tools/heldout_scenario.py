#!/usr/bin/env python3
"""heldout_scenario.py: the point-to-point scenario of a held-out world (roadmap item 33), the start and the goal
drawn with a recorded seed from the reachable free component of the world's truth voxelization, nothing adapted to
the world.

  python3 tools/heldout_scenario.py --world-id cave_circuit_practice_01 \\
      --grid external/environment-candidates/work/cave_practice_01_r050.occupancy3d \\
      --gazebo-world-name cave_circuit_practice_01_collisions --seed 1 \\
      --scenario drone_city_nav/config/cave_circuit_practice_01_point_to_point_scenario.json \\
      --world drone_city_nav/worlds/cave_circuit_practice_01.world3d.json

The frame. The stack's obstacle memory is a box of fixed size at fixed map coordinates (urban_mvp.yaml,
obstacle_memory_3d_node: grid_origin_*, grid_*_m) and the planner's lattice never leaves it (specification K16). The
world's canonical frame lays that box over the world by one rule: on each axis, the window of the box's length that
holds the most occupied voxels of the truth grid (the box aligned with the grid's origin where the grid is shorter).
The rule's result is the canonical world's map_to_sdf offset; the stack's box is not touched.

The space. A voxel is flyable when no occupied voxel lies within the body's radius 0.55 m plus 0.5 m of clearance
(1.05 m: the flood of K16 with item 33's clearance), it lies inside the box by that same clearance (the memory holds
nothing beyond its faces), something occupied stands above it in its column (a ceiling: the sky over a terrain is not
the world), and it is enclosed by the world: not joined to the grid's faces through free voxels. A world of tunnels
and caverns is a set of shells in an empty grid, and the void round the shells is free, roofed by the tunnels above
it, floored by the ones below and far larger than the world's inside: until 2026-11-05 the largest flyable component
was that void, and the start and the goal of both held-out worlds stood on the outside of a tunnel's shell (the
flights r2561 to r2610, r2788 to r2790, r2840 and r2841). The start's component is the largest connected set of
flyable voxels. Everything is measured on the truth grid: the result is reproducible from the grid and the seed.

The start: a flat 3 x 3 m patch of floor (one floor voxel top under its nine cells, nothing occupied between it and
the seed cell), the launch pad 3 x 3 x 1 m with its top 0.5 m over the floor voxel's top (the dark hall's pad stands
0.5 m over its floor), base_link 0.23 m over the pad (the legs), and the seed cell of the flood 2 m over base_link
(the takeoff climb, urban_mvp.yaml takeoff_climb_m). The goal: a flyable cell 300 to 600 m from the seed cell along
the grid (a 26-neighbour path through flyable voxels with Euclidean steps: the urban mission's band of route lengths),
1.5 to 3 m over its floor voxel's top (the floor inside the box: a floor under the memory's bottom is never mapped),
a ceiling within 15 m, the plain free space four cells wide either way (not a shaft), and a landing patch under it:
across 4.5 x 4.5 m the floor within a metre of the goal's own floor, up and down (no wall, slope or pit the landing
can touch: the landing after the goal descends on the estimate, a metre or two off, specification K30). The start's
pad has the same patch around it. On the finals world a goal under a flat ceiling over a flat floor (five by five cells each: the built part of
the world) is preferred when one is in the band. Start and goal are drawn uniformly with the seed; a seed whose start
reaches no goal in the band is counted and the next seed drawn, up to --seeds; when none finds one, the longest pair
found is written (the part's rule). The yaw faces the path's point 10 m along it. The seed, the attempts and the
lengths are written into the scenario under "selection".
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage, sparse
from scipy.sparse import csgraph

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D  # noqa: E402

BODY_RADIUS_M = 0.55  # the stack's body model (K16)
CLEARANCE_M = 0.5  # item 33's clearance over the body
PAD_SIZE_M = (3.0, 3.0, 1.0)
PAD_TOP_OVER_FLOOR_M = 0.5  # the dark hall's pad over its floor
LEGS_M = 0.23  # base_link over the pad's top (the dark hall's start, 0.73 over the floor)
ROUTE_BAND_M = (300.0, 600.0)
GOAL_OVER_FLOOR_M = (1.5, 3.0)
CEILING_WITHIN_M = 15.0
WIDTH_CELLS = 4  # plain free cells either way of the goal, horizontally
LANDING_CELLS = 4  # the landing patch: this many cells either way of the goal (4.5 m across)
LANDING_STEP_CELLS = 2  # the floor within this many cells (1 m) of the goal's own floor across the patch, up and down
YAW_ALONG_M = 10.0
NEIGHBOURS = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
              if (dx, dy, dz) > (0, 0, 0)]  # one half of the 26: the graph is undirected


def dense_occupancy(path: Path):
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


def outside_of_the_world(occupied):
    """The free voxels joined to the grid's faces through free voxels (six neighbours): the void round the world's
    shells, which the vehicle never enters. What a shell encloses is the world; a voxel's size closes the meshes'
    seams (both held-out worlds are tight at 0.5 m)."""
    labels, _ = ndimage.label(~np.pad(occupied, 1, constant_values=False))
    return (labels == labels[0, 0, 0])[1:-1, 1:-1, 1:-1]


def memory_box():
    """The obstacle memory's box in map coordinates, and the takeoff climb, from the node configuration."""
    configuration = yaml.safe_load((REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text())
    memory = configuration["obstacle_memory_3d_node"]["ros__parameters"]
    origin = np.array([memory["grid_origin_x"], memory["grid_origin_y"], memory["grid_origin_z"]], dtype=float)
    size = np.array([memory["grid_width_m"], memory["grid_height_m"], memory["grid_depth_m"]], dtype=float)
    climb = float(next(node["ros__parameters"]["takeoff_climb_m"] for node in configuration.values()
                       if "takeoff_climb_m" in node.get("ros__parameters", {})))
    return origin, size, climb


def frame_offset(occupied, bounds, box_origin, box_size):
    """map_to_sdf offset: on each axis the box window holding the most occupied voxels."""
    resolution = bounds.resolution_m
    grid_origin = (bounds.origin_x_m, bounds.origin_y_m, bounds.origin_z_m)
    offset = []
    for axis in range(3):
        profile = occupied.sum(axis=tuple(other for other in range(3) if other != axis))
        window = int(round(box_size[axis] / resolution))
        if window >= len(profile):
            start = 0
        else:
            sums = np.concatenate([[0], np.cumsum(profile)])
            start = int(np.argmax(sums[window:] - sums[:-window]))
        offset.append(grid_origin[axis] + start * resolution - box_origin[axis])
    return np.array(offset)


def select(args):
    box_origin, box_size, climb = memory_box()
    bounds, occupied = dense_occupancy(args.grid)
    resolution = bounds.resolution_m
    grid_origin = np.array([bounds.origin_x_m, bounds.origin_y_m, bounds.origin_z_m])
    offset = frame_offset(occupied, bounds, box_origin, box_size)
    box_low_sdf = box_origin + offset
    box_high_sdf = box_low_sdf + box_size
    print(f"frame: map_to_sdf offset {offset.round(3).tolist()}, the memory box in the world "
          f"{box_low_sdf.round(1).tolist()} .. {box_high_sdf.round(1).tolist()}")

    # The sub-volume: the box with margins (the erosion sees walls just outside, the floor and the ceiling checks
    # look 4 m under and 16 m over it).
    margin = np.array([3, 3, 32])
    low = np.maximum(np.floor((box_low_sdf - grid_origin) / resolution).astype(int) - margin, 0)
    high = np.minimum(np.ceil((box_high_sdf - grid_origin) / resolution).astype(int) + margin,
                      np.array(occupied.shape))
    sub = occupied[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
    shape = sub.shape
    # Something occupied above, anywhere in the column of the whole grid: a ceiling.
    above = np.cumsum(occupied[low[0]:high[0], low[1]:high[1], ::-1], axis=2)[:, :, ::-1]
    covered_full = (above - occupied[low[0]:high[0], low[1]:high[1], :]) > 0
    covered = covered_full[:, :, low[2]:high[2]]
    del above, covered_full
    distance = ndimage.distance_transform_edt(~sub) * resolution
    inside = np.zeros(shape, dtype=bool)
    box_low_cell = np.floor((box_low_sdf - grid_origin) / resolution).astype(int) - low
    box_high_cell = np.ceil((box_high_sdf - grid_origin) / resolution).astype(int) - low
    inside[max(box_low_cell[0], 0):box_high_cell[0], max(box_low_cell[1], 0):box_high_cell[1],
           max(box_low_cell[2], 0):box_high_cell[2]] = True
    # The body's clearance inside the box too: the memory holds nothing beyond its faces.
    inside_clear = ndimage.binary_erosion(inside, iterations=int(math.ceil((BODY_RADIUS_M + CLEARANCE_M) / resolution)))
    enclosed = ~outside_of_the_world(occupied)[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
    flyable = (distance >= BODY_RADIUS_M + CLEARANCE_M) & covered & inside_clear & enclosed
    print(f"enclosed free voxels in the sub-volume {int((enclosed & ~sub).sum())}")
    del distance, enclosed
    labels, count = ndimage.label(flyable)
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    component_label = int(np.argmax(sizes))
    component = labels == component_label
    print(f"flyable voxels {int(flyable.sum())} in {count} components; the largest {int(sizes[component_label])} "
          f"voxels ({sizes[component_label] * resolution ** 3:.0f} m^3)")
    del labels

    # The floor under and the ceiling over every cell, as indices along z (-1 / shape[2] when none).
    k_index = np.arange(shape[2])[None, None, :]
    floor = np.maximum.accumulate(np.where(sub, k_index, -1), axis=2)
    ceiling = np.minimum.accumulate(np.where(sub, k_index, shape[2])[:, :, ::-1], axis=2)[:, :, ::-1]
    height_cells = k_index - floor  # the cell's index over the floor voxel's (0.5 m over it = 1)

    # The landing patch: under a cell, across 4.5 x 4.5 m, the floor within a metre of the cell's own
    # floor, up and down: a column standing higher is a wall, one lying lower is a slope or a pit (the
    # cave goal of 4a1268eb sat on a funnel's slope, the floor 7 m lower across 8.5 m, and r2556, r2557 and
    # r2559 touched it on the way down; r2554 fell into a pit 2 m from a flat 3 x 3 start). The landing
    # after the goal descends on the estimate, a metre or two off (specification K30).
    landing = np.ones(shape, dtype=bool)
    for dx in range(-LANDING_CELLS, LANDING_CELLS + 1):
        for dy in range(-LANDING_CELLS, LANDING_CELLS + 1):
            landing &= np.abs(np.roll(np.roll(floor, -dx, axis=0), -dy, axis=1) - floor) <= LANDING_STEP_CELLS
    landing[:LANDING_CELLS, :, :] = False
    landing[-LANDING_CELLS:, :, :] = False
    landing[:, :LANDING_CELLS, :] = False
    landing[:, -LANDING_CELLS:, :] = False

    # The start cells: the seed cell of the flood over a flat 3 x 3 patch.
    seed_cells_over_floor = int(round((PAD_TOP_OVER_FLOOR_M + LEGS_M + climb) / resolution)) + 1
    flat = np.ones(shape, dtype=bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            shifted = np.roll(np.roll(floor, -dx, axis=0), -dy, axis=1)
            flat &= shifted == floor
    flat[[0, -1], :, :] = False
    flat[:, [0, -1], :] = False
    start_ok = component & flat & (floor >= 0) & (height_cells == seed_cells_over_floor)
    # The landing patch's floor around the start too: the first step off the pad goes where the pair has
    # not looked (r2554: a slope and a 7 m pit 2 m from a flat 3 x 3 start).
    start_ok &= landing
    # The pad and the vehicle inside the box too.
    start_ok &= (floor + 1) >= max(box_low_cell[2], 0)
    start_cells = np.argwhere(start_ok)
    print(f"start cells {len(start_cells)}")
    if not len(start_cells):
        raise SystemExit("no flat start patch in the component")

    # The goal cells, before the distance band.
    free_x = ndimage.uniform_filter1d(sub.astype(np.float32), 2 * WIDTH_CELLS + 1, axis=0, mode="constant") == 0
    free_y = ndimage.uniform_filter1d(sub.astype(np.float32), 2 * WIDTH_CELLS + 1, axis=1, mode="constant") == 0
    height_m = (height_cells - 0.5) * resolution  # the cell's centre over the floor voxel's top
    goal_ok = component & (floor >= 0) & (height_m >= GOAL_OVER_FLOOR_M[0]) & (height_m <= GOAL_OVER_FLOOR_M[1])
    goal_ok &= (ceiling < shape[2]) & ((ceiling - k_index) * resolution <= CEILING_WITHIN_M) & free_x & free_y
    # The goal's floor in the box: a floor under the memory's bottom is never mapped.
    goal_ok &= (floor + 1) >= max(box_low_cell[2], 0)
    # The landing patch under the goal (r2549 to r2552: four lidar flights reached the cave goal and touched
    # the corner wall 2.4 m below it on the way down).
    goal_ok &= landing
    built = np.ones(shape, dtype=bool)
    for dx in (-2, -1, 0, 1, 2):
        for dy in (-2, -1, 0, 1, 2):
            built &= np.roll(np.roll(floor, -dx, axis=0), -dy, axis=1) == floor
            built &= np.roll(np.roll(ceiling, -dx, axis=0), -dy, axis=1) == ceiling
    print(f"goal cells before the distance band {int(goal_ok.sum())}, under a flat ceiling over a flat floor "
          f"{int((goal_ok & built).sum())}")

    # The graph of the component: 26 neighbours, Euclidean steps.
    cells = np.argwhere(component)
    index = np.full(shape, -1, dtype=np.int64)
    index[tuple(cells.T)] = np.arange(len(cells))
    rows, cols, weights = [], [], []
    for dx, dy, dz in NEIGHBOURS:
        moved = cells + np.array([dx, dy, dz])
        ok = np.all((moved >= 0) & (moved < np.array(shape)), axis=1)
        target = index[tuple(moved[ok].T)]
        linked = target >= 0
        rows.append(np.nonzero(ok)[0][linked])
        cols.append(target[linked])
        weights.append(np.full(int(linked.sum()), math.sqrt(dx * dx + dy * dy + dz * dz) * resolution))
    graph = sparse.csr_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
                              shape=(len(cells), len(cells)))
    del rows, cols, weights
    goal_indices = index[tuple(np.argwhere(goal_ok).T)]
    goal_indices = goal_indices[goal_indices >= 0]
    built_flat = built[tuple(cells[goal_indices].T)]

    attempts = []
    longest = None
    chosen = None
    for seed in range(args.seed, args.seed + args.seeds):
        rng = random.Random(seed)
        start_cell = start_cells[rng.randrange(len(start_cells))]
        start_index = int(index[tuple(start_cell)])
        distances, predecessors = csgraph.dijkstra(graph, directed=False, indices=start_index,
                                                   return_predecessors=True)
        reach = distances[goal_indices]
        finite = np.isfinite(reach)
        farthest = float(reach[finite].max()) if finite.any() else 0.0
        in_band = finite & (reach >= ROUTE_BAND_M[0]) & (reach <= ROUTE_BAND_M[1])
        preferred = in_band & built_flat if args.prefer_built else in_band
        candidates = goal_indices[preferred] if preferred.any() else goal_indices[in_band]
        attempts.append({"seed": seed, "start_cell": start_cell.tolist(), "goals_in_band": int(in_band.sum()),
                         "farthest_goal_m": round(farthest, 1)})
        print(f"seed {seed}: start cell {start_cell.tolist()}, {int(in_band.sum())} goals in the band, "
              f"the farthest goal {farthest:.1f} m")
        if finite.any() and (longest is None or farthest > longest[0]):
            farthest_index = int(goal_indices[finite][np.argmax(reach[finite])])
            longest = (farthest, seed, start_index, farthest_index, distances, predecessors)
        if len(candidates):
            goal_index = int(candidates[rng.randrange(len(candidates))])
            chosen = (seed, start_index, goal_index, distances, predecessors)
            break
    if chosen is None:
        if longest is None:
            raise SystemExit("no goal reachable from any start")
        farthest, seed, start_index, goal_index, distances, predecessors = longest
        print(f"no pair in the band over {args.seeds} seeds: the longest pair, {farthest:.1f} m, seed {seed}")
        chosen = (seed, start_index, goal_index, distances, predecessors)
    seed, start_index, goal_index, distances, predecessors = chosen

    # The path, for the yaw and the record.
    path = [goal_index]
    while path[-1] != start_index:
        path.append(int(predecessors[path[-1]]))
    path.reverse()
    path_cells = cells[path]
    path_sdf = grid_origin + (path_cells + low + 0.5) * resolution
    along = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path_sdf, axis=0), axis=1))])
    ahead = path_sdf[min(int(np.searchsorted(along, YAW_ALONG_M)), len(path_sdf) - 1)]
    yaw = math.atan2(ahead[1] - path_sdf[0][1], ahead[0] - path_sdf[0][0])

    start_cell = cells[start_index]
    goal_cell = cells[goal_index]
    floor_top_sdf = grid_origin[2] + (floor[tuple(start_cell)] + low[2] + 1) * resolution
    pad_top_sdf = floor_top_sdf + PAD_TOP_OVER_FLOOR_M
    start_sdf = np.array([grid_origin[0] + (start_cell[0] + low[0] + 0.5) * resolution,
                          grid_origin[1] + (start_cell[1] + low[1] + 0.5) * resolution,
                          pad_top_sdf + LEGS_M])
    goal_sdf = grid_origin + (goal_cell + low + 0.5) * resolution
    goal_floor_top_sdf = grid_origin[2] + (floor[tuple(goal_cell)] + low[2] + 1) * resolution
    goal_ceiling_sdf = grid_origin[2] + (ceiling[tuple(goal_cell)] + low[2]) * resolution
    start_map = start_sdf - offset
    goal_map = goal_sdf - offset
    route_m = float(distances[goal_index])
    straight_m = float(np.linalg.norm(goal_sdf - start_sdf))
    print(f"start (map) {start_map.round(3).tolist()} yaw {yaw:.3f}, the pad's top {pad_top_sdf - offset[2]:.3f}; "
          f"goal (map) {goal_map.round(3).tolist()}, {goal_sdf[2] - goal_floor_top_sdf:.2f} m over its floor, "
          f"the ceiling {goal_ceiling_sdf - goal_sdf[2]:.2f} m over it; route {route_m:.1f} m along the grid, "
          f"{straight_m:.1f} m straight; seed {seed} after {len(attempts)} attempts")

    world = {
        "schema": "drone_city_nav_canonical_world_v1",
        "frame_id": "map",
        "gazebo_world_name": args.gazebo_world_name,
        "map_to_sdf": {
            "sdf_x_from": "map_x", "sdf_x_scale": 1.0, "sdf_x_offset_m": round(float(offset[0]), 3),
            "sdf_y_from": "map_y", "sdf_y_scale": 1.0, "sdf_y_offset_m": round(float(offset[1]), 3),
            "sdf_z_scale": 1.0, "sdf_z_offset_m": round(float(offset[2]), 3),
        },
        "navigation": {
            "initial_altitude_m": round(float(start_map[2] + climb), 3),
            "minimum_target_z_m": round(float(box_origin[2] + 1.0), 3),
            "maximum_target_z_m": round(float(box_origin[2] + box_size[2] - 1.0), 3),
        },
        "environment": {"manifest_id": args.world_id},
    }
    scenario = {
        "schema": "drone_city_nav_point_to_point_scenario_v2",
        "canonical_world": f"../worlds/{args.world.name}",
        "vehicle": {
            "px4_model_target": "gz_x500_lidar_3d",
            "gazebo_model_name": "x500_lidar_3d_0",
            "map_start_m": [round(float(value), 3) for value in start_map],
            "yaw_rad": round(yaw, 6),
        },
        "mission_goal_sequence_m": [[round(float(value), 3) for value in goal_map]],
        "launch_platforms": [{
            "id": "pad",
            "vehicle_ids": ["point_to_point_vehicle"],
            "size_m": list(PAD_SIZE_M),
            "top_z_m": round(float(pad_top_sdf - offset[2]), 3),
        }],
        "selection": {
            "tool": "tools/heldout_scenario.py",
            "grid": str(args.grid),
            "seed": seed,
            "attempts": attempts,
            "route_length_m": round(route_m, 1),
            "straight_distance_m": round(straight_m, 1),
            "goal_over_floor_m": round(float(goal_sdf[2] - goal_floor_top_sdf), 2),
            "goal_under_ceiling_m": round(float(goal_ceiling_sdf - goal_sdf[2]), 2),
            "landing_patch_m": (2 * LANDING_CELLS + 1) * resolution,
            "component_voxels": int(sizes[component_label]),
            "space": "enclosed by the world's shells",
            "memory_box_in_world_m": [box_low_sdf.round(1).tolist(), box_high_sdf.round(1).tolist()],
        },
    }
    args.world.write_text(json.dumps(world, indent=2) + "\n", encoding="utf-8")
    args.scenario.write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    print(f"written {args.scenario} and {args.world}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world-id", required=True, help="the manifest id of the world")
    parser.add_argument("--grid", type=Path, required=True, help="the world's truth Occupancy3D at 0.5 m")
    parser.add_argument("--gazebo-world-name", required=True, help="the materialized world's name")
    parser.add_argument("--seed", type=int, default=1, help="the first seed drawn")
    parser.add_argument("--seeds", type=int, default=200, help="seeds tried before the longest pair is taken")
    parser.add_argument("--prefer-built", action="store_true",
                        help="prefer a goal under a flat ceiling over a flat floor (the finals world)")
    parser.add_argument("--scenario", type=Path, required=True, help="the scenario file written")
    parser.add_argument("--world", type=Path, required=True, help="the canonical world file written")
    select(parser.parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
