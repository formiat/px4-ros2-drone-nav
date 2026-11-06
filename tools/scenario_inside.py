#!/usr/bin/env python3
"""scenario_inside.py --scenario S --grid G [--world-sdf W] [--write] [--pictures DIR]: the proof that a scenario's
start and goals lie inside its world, by tests that do not share a way of failing (specification K63). Until
2026-11-05 both held-out scenarios stood in the void round their worlds' shells: the scenario tool took the largest
free space under a ceiling, and the void has ceilings (the tunnels above it), floors (the ones below) and a hundred
times the world's volume (roadmap item 33).

The tests, each on the world's truth voxelization (the start read 2 m over its pad, after the takeoff climb):

  rays      what the point sees (scripts/world_enclosure.py): 128 rays over the sphere, the share that meets a
            surface within 40 m; 0.90 or more inside. The one test that holds on a world whose inside is joined to
            the grid's faces (Urban: an open staging area).
  tiles     what the world is made of: the world's own parts in its SDF (the staging area and every tile, each with
            its pose) vote for the free component that holds the point 1.5 m over the most of their origins; the
            point is in that component. A part's origin is not always on its floor (36 of 117 on the finals world
            read the void), so the parts vote, they do not decide alone.
  shells    where the tiles' component is sealed from the grid's faces (a tight world), the point is sealed too.
  floor     a floor under the point within 8 m and a ceiling over it within 25 m, straight down and straight up.

Every test must pass for every point. --write puts the proof into the scenario file (`inside`: the grid's hash, the
world's figures, each point's readings); the flight's check measures the rays again on a scenario that carries it.
--pictures writes, for each point, the horizontal slice of the grid at its height: occupied black, the void grey,
the world white, the point red, for the eye that found the fault. Host (SciPy; the dev container's Python loads none).

  python3 tools/scenario_inside.py --write --pictures log/tools/scenario_inside \\
      --scenario drone_city_nav/config/finals_prize_round_world_07_point_to_point_scenario.json \\
      --grid external/environment-candidates/work/final_event_07_r050.occupancy3d
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import world_enclosure  # noqa: E402

ANCHOR_OVER_ORIGIN_M = 1.5
FLOOR_WITHIN_M = 8.0
CEILING_WITHIN_M = 25.0
TILE = re.compile(r'<model name="include_((?:tile_\d+|staging_area)[^"]*)">\s*<static>true</static>\s*'
                  r'<pose>([^<]*)</pose>')


def world_parts(sdf_text: str) -> dict[str, tuple[float, float, float]]:
    """Each part of the world (the staging area, every tile) with the origin of its first collision that is not a
    ground sheet: the part's own pose."""
    parts: dict[str, tuple[float, float, float]] = {}
    for match in TILE.finditer(sdf_text):
        name = match.group(1)
        key = re.match(r"(tile_\d+|staging_area)", name).group(1)
        if "ground" in name or key in parts:
            continue
        x, y, z = (float(value) for value in match.group(2).split()[:3])
        parts[key] = (x, y, z)
    return parts


def free_components(occupied):
    """The free voxels' components (six neighbours) and the label of the one joined to the grid's faces."""
    labels, _ = ndimage.label(~np.pad(occupied, 1, constant_values=False))
    return labels[1:-1, 1:-1, 1:-1], int(labels[0, 0, 0])


def tiles_component(labels, bounds, parts) -> tuple[int, int, int]:
    """The free component the world's parts vote for, its votes and the parts whose anchor is free."""
    votes: dict[int, int] = {}
    for origin in parts.values():
        cell = world_enclosure.cell_of(bounds, (origin[0], origin[1], origin[2] + ANCHOR_OVER_ORIGIN_M))
        if all(0 <= cell[axis] < labels.shape[axis] for axis in range(3)) and labels[cell] != 0:
            votes[int(labels[cell])] = votes.get(int(labels[cell]), 0) + 1
    if not votes:
        return 0, 0, 0
    winner = max(votes, key=votes.get)
    return winner, votes[winner], sum(votes.values())


def straight(occupied, cell, step: int, within_cells: int):
    """The distance in cells to the first occupied voxel straight down (step -1) or up (+1), or None."""
    x, y, z = cell
    for distance in range(1, within_cells + 1):
        k = z + step * distance
        if not 0 <= k < occupied.shape[2]:
            return None
        if occupied[x, y, k]:
            return distance
    return None


def prove(scenario_path: Path, grid_path: Path, sdf_path: Path | None) -> tuple[bool, dict, tuple]:
    bounds, occupied = world_enclosure.dense_occupancy(grid_path)
    resolution = bounds.resolution_m
    labels, outside = free_components(occupied)
    parts = world_parts(sdf_path.read_text(encoding="utf-8")) if sdf_path is not None and sdf_path.is_file() else {}
    component, votes, anchors_free = tiles_component(labels, bounds, parts)
    tight = component != 0 and component != outside
    sizes = np.bincount(labels.ravel())
    world = {"parts": len(parts), "anchors_free": anchors_free, "anchors_in_the_tiles_component": votes,
             "tiles_component_voxels": int(sizes[component]) if component else 0,
             "void_voxels": int(sizes[outside]), "sealed_from_the_grids_faces": bool(tight)}
    points = []
    passed = True
    for (name, point_map, point_sdf), (_, _, share) in zip(
            world_enclosure.scenario_points(scenario_path),
            world_enclosure.enclosure_of_scenario(scenario_path, bounds, occupied)):
        cell = world_enclosure.cell_of(bounds, point_sdf)
        on_grid = all(0 <= cell[axis] < occupied.shape[axis] for axis in range(3))
        label = int(labels[cell]) if on_grid else 0
        floor = straight(occupied, cell, -1, int(FLOOR_WITHIN_M / resolution)) if on_grid else None
        ceiling = straight(occupied, cell, 1, int(CEILING_WITHIN_M / resolution)) if on_grid else None
        tests = {
            "rays": share >= world_enclosure.ENCLOSED_SHARE,
            "tiles": component == 0 or label == component,
            "shells": not tight or (label != 0 and label != outside),
            "floor": floor is not None and ceiling is not None,
        }
        passed = passed and all(tests.values())
        points.append({"name": name, "map_m": [round(v, 3) for v in point_map], "rays_meeting_a_surface": round(share, 3),
                       "in_the_tiles_component": bool(label == component) if component else None,
                       "joined_to_the_grids_faces": bool(label == outside),
                       "floor_under_m": None if floor is None else floor * resolution,
                       "ceiling_over_m": None if ceiling is None else ceiling * resolution,
                       "passed": {key: bool(value) for key, value in tests.items()}})
    proof = {"tool": "tools/scenario_inside.py", "grid": str(grid_path),
             "grid_sha256": hashlib.sha256(grid_path.read_bytes()).hexdigest(),
             "rays": world_enclosure.RAYS, "reach_m": world_enclosure.REACH_M,
             "enclosed_share": world_enclosure.ENCLOSED_SHARE, "world": world, "points": points}
    return passed, proof, (bounds, occupied, labels, outside, component)


def pictures(directory: Path, scenario_path: Path, proof: dict, state) -> None:
    """For each point the grid's slice at its height: occupied black, the void grey, the world white, the point red."""
    import matplotlib  # noqa: PLC0415
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    bounds, occupied, labels, outside, _ = state
    directory.mkdir(parents=True, exist_ok=True)
    for (name, _, point_sdf), reading in zip(world_enclosure.scenario_points(scenario_path), proof["points"]):
        cell = world_enclosure.cell_of(bounds, point_sdf)
        if not 0 <= cell[2] < occupied.shape[2]:
            continue
        layer = np.where(occupied[:, :, cell[2]], 0.0, np.where(labels[:, :, cell[2]] == outside, 0.55, 1.0))
        figure, axes = plt.subplots(figsize=(10, 10 * occupied.shape[1] / occupied.shape[0]))
        extent = (bounds.origin_x_m, bounds.origin_x_m + occupied.shape[0] * bounds.resolution_m,
                  bounds.origin_y_m, bounds.origin_y_m + occupied.shape[1] * bounds.resolution_m)
        axes.imshow(layer.T, origin="lower", cmap="gray", vmin=0.0, vmax=1.0, extent=extent)
        axes.plot(point_sdf[0], point_sdf[1], "o", color="red", markersize=9, markerfacecolor="none", markeredgewidth=2)
        axes.set_title(f"{scenario_path.stem}: {name} at z {point_sdf[2]:.1f} m (SDF), rays "
                       f"{reading['rays_meeting_a_surface']:.2f}; the void grey, the world white")
        figure.savefig(directory / f"{scenario_path.stem}_{name.replace(' ', '_')}.png", dpi=110, bbox_inches="tight")
        plt.close(figure)


def default_world_sdf(scenario_path: Path) -> Path:
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    world_id = Path(scenario["canonical_world"]).name.replace(".world3d.json", "")
    return REPOSITORY / "external/environment-artifacts/derived" / world_id / "runtime/world_collision.sdf"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--grid", type=Path, required=True, help="the world's truth Occupancy3D")
    parser.add_argument("--world-sdf", type=Path, help="the world's collision SDF (the derived runtime's unless given)")
    parser.add_argument("--write", action="store_true", help="write the proof into the scenario file")
    parser.add_argument("--pictures", type=Path, help="a directory for the slices")
    args = parser.parse_args()
    sdf_path = args.world_sdf if args.world_sdf is not None else default_world_sdf(args.scenario)
    passed, proof, state = prove(args.scenario, args.grid, sdf_path)
    world = proof["world"]
    print(f"world: {world['parts']} parts, {world['anchors_in_the_tiles_component']} of {world['anchors_free']} free "
          f"anchors in the tiles' component of {world['tiles_component_voxels']} voxels; the void "
          f"{world['void_voxels']}; sealed from the grid's faces: {world['sealed_from_the_grids_faces']}")
    for point in proof["points"]:
        failed = [key for key, value in point["passed"].items() if not value]
        print(f"{'OK' if not failed else 'FAIL'}: {point['name']} at {point['map_m']}: rays "
              f"{point['rays_meeting_a_surface']:.2f}, in the tiles' component {point['in_the_tiles_component']}, "
              f"joined to the faces {point['joined_to_the_grids_faces']}, floor {point['floor_under_m']} m under, "
              f"ceiling {point['ceiling_over_m']} m over" + (f"; failed: {', '.join(failed)}" if failed else ""))
    if args.pictures is not None:
        pictures(args.pictures, args.scenario, proof, state)
    print("INSIDE " + ("PASS" if passed else "FAIL"))
    if args.write and passed:
        document = json.loads(args.scenario.read_text(encoding="utf-8"))
        document["inside"] = proof
        args.scenario.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(f"written into {args.scenario}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
