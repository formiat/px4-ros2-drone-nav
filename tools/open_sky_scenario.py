#!/usr/bin/env python3
"""open_sky_scenario.py: the point-to-point scenario of the open-sky world (roadmap item 39, stage 0), the start and
a sequence of goals drawn with a recorded seed from the truth voxelization, low over the ground.

  python3 tools/open_sky_scenario.py --world-id open_sky_baylands \\
      --grid external/environment-candidates/work/open_sky_baylands_r050.occupancy3d \\
      --gazebo-world-name open_sky_baylands_collisions --seed 1 \\
      --scenario drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json \\
      --world drone_city_nav/worlds/open_sky_baylands.world3d.json

The frame and the memory box are placed as tools/heldout_scenario.py places them (the window of the box holding the
most of the world). The ground under a column is its lowest occupied voxel's top (the terrain; trees and buildings
stand on it); a column with nothing occupied (the water, which carries no collision) has no ground and is not drawn
from. A cell is flyable when it stands 4 to 8 m over its ground, clear of everything occupied by the body's radius
plus 0.5 m (trees, walls), and inside the box by that clearance. The start: a flat 3 x 3 m patch of ground in the
outer fifth of the box (the edge), a pad 3 x 3 x 1 m with its top 0.5 m over the ground voxel's top, base_link 0.23 m
over the pad, the flood's seed cell 2 m over base_link, and a landing patch around it (the ground within a metre across
4.5 x 4.5 m, nothing occupied more than a metre over it). The goals: a sequence of --goals cells, each 500 to 700 m
from the previous along the grid (26 neighbours, Euclidean steps), 6 m over its ground, over a landing patch; the
mission is the sequence, 1.5 to 2.1 km with three goals. Start and goals are drawn uniformly with the seed; a seed
whose draw fails at any goal is counted and the next seed drawn. The yaw faces the path's point 10 m along it.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "tools"))
from heldout_scenario import (  # noqa: E402
    BODY_RADIUS_M, CLEARANCE_M, LANDING_CELLS, LANDING_STEP_CELLS, LEGS_M, NEIGHBOURS, PAD_SIZE_M,
    PAD_TOP_OVER_FLOOR_M, YAW_ALONG_M, dense_occupancy, frame_offset, memory_box)

FLIGHT_OVER_GROUND_M = (4.0, 8.0)  # the band the open sky is flown in (the pair sees the ground and the trees)
GOAL_OVER_GROUND_M = 6.0
LEG_BAND_M = (500.0, 700.0)  # each goal from the previous, along the grid
EDGE_SHARE = 0.2  # the start in the outer fifth of the box


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
    margin = np.array([3, 3, 8])
    low = np.maximum(np.floor((box_low_sdf - grid_origin) / resolution).astype(int) - margin, 0)
    high = np.minimum(np.ceil((box_high_sdf - grid_origin) / resolution).astype(int) + margin,
                      np.array(occupied.shape))
    sub = occupied[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
    shape = sub.shape
    k_index = np.arange(shape[2])[None, None, :]
    # The ground: the lowest occupied voxel of the column (the whole grid's column, the terrain may lie under the box).
    column = occupied[low[0]:high[0], low[1]:high[1], :]
    lowest = np.where(column, np.arange(occupied.shape[2])[None, None, :], 10 ** 6).min(axis=2)
    has_ground = lowest < 10 ** 6
    ground = lowest + 1 - low[2]  # the index, in the sub-volume, of the first cell over the ground voxel
    highest = np.where(column, np.arange(occupied.shape[2])[None, None, :], -1).max(axis=2) + 1 - low[2]
    del column
    height_m = (k_index - ground[:, :, None] + 0.5) * resolution  # the cell's centre over the ground voxel's top
    distance = ndimage.distance_transform_edt(~sub) * resolution
    inside = np.zeros(shape, dtype=bool)
    box_low_cell = np.floor((box_low_sdf - grid_origin) / resolution).astype(int) - low
    box_high_cell = np.ceil((box_high_sdf - grid_origin) / resolution).astype(int) - low
    inside[max(box_low_cell[0], 0):box_high_cell[0], max(box_low_cell[1], 0):box_high_cell[1],
           max(box_low_cell[2], 0):box_high_cell[2]] = True
    inside_clear = ndimage.binary_erosion(
        inside, iterations=int(math.ceil((BODY_RADIUS_M + CLEARANCE_M) / resolution)))
    clear = (distance >= BODY_RADIUS_M + CLEARANCE_M) & inside_clear & has_ground[:, :, None]
    del distance
    flyable = clear & (height_m >= FLIGHT_OVER_GROUND_M[0]) & (height_m <= FLIGHT_OVER_GROUND_M[1])
    labels, count = ndimage.label(flyable)
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    component_label = int(np.argmax(sizes))
    component = labels == component_label
    print(f"flyable voxels {int(flyable.sum())} in {count} components; the largest {int(sizes[component_label])} "
          f"voxels")
    del labels

    # The landing patch (as the held-out worlds': the ground within a metre across 4.5 m, nothing occupied more
    # than a metre over it: no tree, wall or ledge the descent can touch).
    patch = has_ground.copy()
    for dx in range(-LANDING_CELLS, LANDING_CELLS + 1):
        for dy in range(-LANDING_CELLS, LANDING_CELLS + 1):
            shifted_ground = np.roll(np.roll(ground, -dx, axis=0), -dy, axis=1)
            shifted_top = np.roll(np.roll(highest, -dx, axis=0), -dy, axis=1)
            patch &= np.abs(shifted_ground - ground) <= LANDING_STEP_CELLS
            patch &= (shifted_top - ground) <= LANDING_STEP_CELLS
            patch &= np.roll(np.roll(has_ground, -dx, axis=0), -dy, axis=1)
    patch[:LANDING_CELLS, :] = False
    patch[-LANDING_CELLS:, :] = False
    patch[:, :LANDING_CELLS] = False
    patch[:, -LANDING_CELLS:] = False
    # The start: a flat 3 x 3 patch, the seed cell 2 m over base_link, in the outer fifth of the box.
    seed_cells_over_ground = int(round((PAD_TOP_OVER_FLOOR_M + LEGS_M + climb) / resolution))
    flat = has_ground.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            flat &= np.roll(np.roll(ground, -dx, axis=0), -dy, axis=1) == ground
    edge = np.zeros(shape[:2], dtype=bool)
    x0, x1 = max(box_low_cell[0], 0), min(box_high_cell[0], shape[0])
    y0, y1 = max(box_low_cell[1], 0), min(box_high_cell[1], shape[1])
    band_x = int((x1 - x0) * EDGE_SHARE)
    band_y = int((y1 - y0) * EDGE_SHARE)
    edge[x0:x0 + band_x, y0:y1] = True
    edge[x1 - band_x:x1, y0:y1] = True
    edge[x0:x1, y0:y0 + band_y] = True
    edge[x0:x1, y1 - band_y:y1] = True
    # The seed cell stands under the band (2.7 m over the ground): clear itself, with the band's first cell
    # over it in the component, so that the climb enters the flown space.
    band_cells_over_ground = int(math.ceil(FLIGHT_OVER_GROUND_M[0] / resolution))
    column_in_component = np.zeros(shape[:2], dtype=bool)
    ii, jj = np.nonzero(has_ground)
    kk = ground[ii, jj] + band_cells_over_ground
    valid = (kk >= 0) & (kk < shape[2])
    column_in_component[ii[valid], jj[valid]] = component[ii[valid], jj[valid], kk[valid]]
    start_ok = (clear & (flat & patch & edge & column_in_component)[:, :, None]
                & (k_index - ground[:, :, None] == seed_cells_over_ground))
    start_cells = np.argwhere(start_ok)
    print(f"start cells {len(start_cells)}")
    if not len(start_cells):
        raise SystemExit("no start patch at the box's edge")
    goal_cells_over_ground = int(round(GOAL_OVER_GROUND_M / resolution))
    goal_ok = component & patch[:, :, None] & (k_index - ground[:, :, None] == goal_cells_over_ground)
    print(f"goal cells {int(goal_ok.sum())}")

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

    attempts = []
    chosen = None
    for seed in range(args.seed, args.seed + args.seeds):
        rng = random.Random(seed)
        start_cell = start_cells[rng.randrange(len(start_cells))]
        climb_cell = start_cell.copy()
        climb_cell[2] = ground[start_cell[0], start_cell[1]] + band_cells_over_ground
        sequence = [int(index[tuple(climb_cell)])]
        legs = []
        paths = []
        for _ in range(args.goals):
            distances, predecessors = csgraph.dijkstra(graph, directed=False, indices=sequence[-1],
                                                       return_predecessors=True)
            reach = distances[goal_indices]
            in_band = np.isfinite(reach) & (reach >= LEG_BAND_M[0]) & (reach <= LEG_BAND_M[1])
            if not in_band.any():
                break
            goal_index = int(goal_indices[in_band][rng.randrange(int(in_band.sum()))])
            path = [goal_index]
            while path[-1] != sequence[-1]:
                path.append(int(predecessors[path[-1]]))
            path.reverse()
            paths.append(path)
            legs.append(float(distances[goal_index]))
            sequence.append(goal_index)
        attempts.append({"seed": seed, "start_cell": start_cell.tolist(), "legs_m": [round(leg, 1) for leg in legs]})
        print(f"seed {seed}: start cell {start_cell.tolist()}, legs {[round(leg, 1) for leg in legs]}")
        if len(legs) == args.goals:
            chosen = (seed, sequence, legs, paths)
            break
    if chosen is None:
        raise SystemExit(f"no sequence of {args.goals} goals over {args.seeds} seeds")
    seed, sequence, legs, paths = chosen

    path_sdf = grid_origin + (cells[paths[0]] + low + 0.5) * resolution
    along = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path_sdf, axis=0), axis=1))])
    ahead = path_sdf[min(int(np.searchsorted(along, YAW_ALONG_M)), len(path_sdf) - 1)]
    yaw = math.atan2(ahead[1] - path_sdf[0][1], ahead[0] - path_sdf[0][0])
    start_cell = cells[sequence[0]]  # the band's first cell over the pad; the start's x, y are its own
    ground_top_sdf = grid_origin[2] + (ground[start_cell[0], start_cell[1]] + low[2]) * resolution
    pad_top_sdf = ground_top_sdf + PAD_TOP_OVER_FLOOR_M
    start_sdf = np.array([grid_origin[0] + (start_cell[0] + low[0] + 0.5) * resolution,
                          grid_origin[1] + (start_cell[1] + low[1] + 0.5) * resolution,
                          pad_top_sdf + LEGS_M])
    goals_sdf = [grid_origin + (cells[goal] + low + 0.5) * resolution for goal in sequence[1:]]
    start_map = start_sdf - offset
    goals_map = [goal - offset for goal in goals_sdf]
    straight = [float(np.linalg.norm(b - a)) for a, b in zip([start_sdf] + goals_sdf[:-1], goals_sdf)]
    print(f"start (map) {start_map.round(3).tolist()} yaw {yaw:.3f}; goals (map) "
          f"{[goal.round(2).tolist() for goal in goals_map]}; legs {[round(leg, 1) for leg in legs]} m along the "
          f"grid ({[round(value, 1) for value in straight]} straight), {sum(legs):.0f} m in all; seed {seed} after "
          f"{len(attempts)} attempts")

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
        "mission_goal_sequence_m": [[round(float(value), 3) for value in goal] for goal in goals_map],
        "launch_platforms": [{
            "id": "pad",
            "vehicle_ids": ["point_to_point_vehicle"],
            "size_m": list(PAD_SIZE_M),
            "top_z_m": round(float(pad_top_sdf - offset[2]), 3),
        }],
        "selection": {
            "tool": "tools/open_sky_scenario.py",
            "grid": str(args.grid),
            "seed": seed,
            "attempts": attempts,
            "legs_m": [round(leg, 1) for leg in legs],
            "route_length_m": round(sum(legs), 1),
            "straight_distances_m": [round(value, 1) for value in straight],
            "flight_over_ground_m": list(FLIGHT_OVER_GROUND_M),
            "goal_over_ground_m": GOAL_OVER_GROUND_M,
            "landing_patch_m": (2 * LANDING_CELLS + 1) * resolution,
            "component_voxels": int(sizes[component_label]),
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
    parser.add_argument("--seeds", type=int, default=50, help="seeds tried")
    parser.add_argument("--goals", type=int, default=3, help="goals in the sequence")
    parser.add_argument("--scenario", type=Path, required=True, help="the scenario file written")
    parser.add_argument("--world", type=Path, required=True, help="the canonical world file written")
    select(parser.parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
