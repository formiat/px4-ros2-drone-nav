#!/usr/bin/env python3
"""Validate cooperative starts and routes against an Occupancy3D artifact."""

from __future__ import annotations

import argparse
import collections
import functools
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_PLANNER_CONFIG = REPOSITORY / "drone_city_nav/config/urban_mvp.yaml"
OCCUPANCY_MAGIC = b"DCNOCC3D"
SUPPORTED_VERSIONS = {4, 5}
CHUNK_SIZE = 16
WORDS_PER_CHUNK = 64


class ScenarioValidationError(RuntimeError):
    """Raised when the scenario cannot be proven executable."""


@dataclass(frozen=True)
class GridBounds:
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_z_m: float
    width: int
    height: int
    depth: int


@dataclass(frozen=True)
class Footprint:
    radius_m: float
    lower_extent_m: float
    upper_extent_m: float


@dataclass(frozen=True)
class LaunchPlatform:
    id: str
    vehicle_ids: tuple[str, ...]
    center_x_m: float
    center_y_m: float
    size_x_m: float
    size_y_m: float
    bottom_z_m: float
    top_z_m: float


class Occupancy3D:
    """Minimal read-only decoder for sparse project Occupancy3D artifacts."""

    def __init__(
        self,
        bounds: GridBounds,
        chunks: dict[tuple[int, int, int], tuple[int, ...]],
    ) -> None:
        self.bounds = bounds
        self.chunks = chunks

    @classmethod
    def load(cls, path: Path) -> "Occupancy3D":
        data = path.read_bytes()
        if data[: len(OCCUPANCY_MAGIC)] != OCCUPANCY_MAGIC:
            raise ScenarioValidationError(f"invalid Occupancy3D magic: {path}")
        offset = len(OCCUPANCY_MAGIC)
        version, chunk_size = struct.unpack_from("<II", data, offset)
        offset += 8
        if version not in SUPPORTED_VERSIONS or chunk_size != CHUNK_SIZE:
            raise ScenarioValidationError(
                f"unsupported Occupancy3D version={version} chunk_size={chunk_size}"
            )
        resolution, origin_x, origin_y, origin_z = struct.unpack_from(
            "<ffff", data, offset
        )
        offset += 16
        width, height, depth = struct.unpack_from("<III", data, offset)
        offset += 12
        offset += 8  # Occupancy fingerprint.
        (chunk_count,) = struct.unpack_from("<I", data, offset)
        offset += 4
        chunks: dict[tuple[int, int, int], tuple[int, ...]] = {}
        for _ in range(chunk_count):
            index = struct.unpack_from("<iii", data, offset)
            offset += 12
            chunks[index] = struct.unpack_from(
                f"<{WORDS_PER_CHUNK}Q", data, offset
            )
            offset += WORDS_PER_CHUNK * 8
        if version == 4:
            embedded_regions, embedded_traversals = struct.unpack_from(
                "<II", data, offset
            )
            offset += 8
            if embedded_regions or embedded_traversals:
                raise ScenarioValidationError(
                    "legacy Occupancy3D contains embedded topology"
                )
        if offset != len(data):
            raise ScenarioValidationError(f"trailing Occupancy3D data: {path}")
        return cls(
            GridBounds(
                resolution_m=resolution,
                origin_x_m=origin_x,
                origin_y_m=origin_y,
                origin_z_m=origin_z,
                width=width,
                height=height,
                depth=depth,
            ),
            chunks,
        )

    def world_to_cell(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        bounds = self.bounds
        result = (
            math.floor((point[0] - bounds.origin_x_m) / bounds.resolution_m),
            math.floor((point[1] - bounds.origin_y_m) / bounds.resolution_m),
            math.floor((point[2] - bounds.origin_z_m) / bounds.resolution_m),
        )
        if not (
            0 <= result[0] < bounds.width
            and 0 <= result[1] < bounds.height
            and 0 <= result[2] < bounds.depth
        ):
            raise ScenarioValidationError(f"point outside Occupancy3D: {point}")
        return result

    def occupied(self, cell: tuple[int, int, int]) -> bool:
        x, y, z = cell
        bounds = self.bounds
        if not (
            0 <= x < bounds.width
            and 0 <= y < bounds.height
            and 0 <= z < bounds.depth
        ):
            return True
        chunk = self.chunks.get((x // CHUNK_SIZE, y // CHUNK_SIZE, z // CHUNK_SIZE))
        if chunk is None:
            return False
        bit = ((z % CHUNK_SIZE) * CHUNK_SIZE + y % CHUNK_SIZE) * CHUNK_SIZE + (
            x % CHUNK_SIZE
        )
        return bool(chunk[bit // 64] & (1 << (bit % 64)))

    def spawn_has_support(
        self,
        point: tuple[float, float, float],
        footprint: Footprint,
        maximum_gap_m: float = 0.25,
    ) -> bool:
        """Return whether a near-planar surface supports the complete footprint."""
        center_x, center_y, _ = self.world_to_cell(point)
        bounds = self.bounds
        footprint_bottom_m = point[2] - footprint.lower_extent_m
        minimum_surface_m = footprint_bottom_m - maximum_gap_m
        minimum_z = max(
            0,
            math.floor(
                (minimum_surface_m - bounds.origin_z_m) / bounds.resolution_m
            ),
        )
        maximum_z = min(
            bounds.depth - 1,
            math.floor(
                (footprint_bottom_m - bounds.origin_z_m) / bounds.resolution_m
            ),
        )
        conservative_radius_m = footprint.radius_m + bounds.resolution_m / math.sqrt(2)
        cell_radius = math.ceil(conservative_radius_m / bounds.resolution_m)
        support_heights_m: list[float] = []
        for delta_y in range(-cell_radius, cell_radius + 1):
            for delta_x in range(-cell_radius, cell_radius + 1):
                if (
                    math.hypot(delta_x, delta_y) * bounds.resolution_m
                    > conservative_radius_m
                ):
                    continue
                support_height_m = None
                for z in range(maximum_z, minimum_z - 1, -1):
                    voxel_top_m = bounds.origin_z_m + (z + 1) * bounds.resolution_m
                    if voxel_top_m <= footprint_bottom_m + 1.0e-6 and self.occupied(
                        (center_x + delta_x, center_y + delta_y, z)
                    ):
                        support_height_m = voxel_top_m
                        break
                if support_height_m is None:
                    return False
                support_heights_m.append(support_height_m)
        return bool(support_heights_m) and (
            max(support_heights_m) - min(support_heights_m)
            <= bounds.resolution_m + 1.0e-6
        )

    def vertical_sweep_is_clear(
        self,
        start: tuple[float, float, float],
        end_z_m: float,
        footprint: Footprint,
    ) -> bool:
        """Return whether the complete vertical takeoff cylinder is collision-free."""
        if end_z_m < start[2]:
            return False
        center_x, center_y, _ = self.world_to_cell(start)
        bounds = self.bounds
        minimum_z = math.floor(
            (start[2] - footprint.lower_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        maximum_z = math.floor(
            (end_z_m + footprint.upper_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        conservative_radius_m = footprint.radius_m + bounds.resolution_m / math.sqrt(2)
        cell_radius = math.ceil(conservative_radius_m / bounds.resolution_m)
        for z in range(minimum_z, maximum_z + 1):
            for delta_y in range(-cell_radius, cell_radius + 1):
                for delta_x in range(-cell_radius, cell_radius + 1):
                    if (
                        math.hypot(delta_x, delta_y) * bounds.resolution_m
                        > conservative_radius_m
                    ):
                        continue
                    if self.occupied((center_x + delta_x, center_y + delta_y, z)):
                        return False
        return True

    def box_is_clear(
        self,
        center: tuple[float, float, float],
        size: tuple[float, float, float],
    ) -> bool:
        """Return whether an axis-aligned fixture box avoids source occupancy."""
        bounds = self.bounds
        minimum = tuple(center[axis] - size[axis] / 2.0 for axis in range(3))
        maximum = tuple(center[axis] + size[axis] / 2.0 for axis in range(3))
        minimum_cell = self.world_to_cell(minimum)
        maximum_cell = self.world_to_cell(
            tuple(maximum[axis] - 1.0e-6 for axis in range(3))
        )
        for z in range(minimum_cell[2], maximum_cell[2] + 1):
            for y in range(minimum_cell[1], maximum_cell[1] + 1):
                for x in range(minimum_cell[0], maximum_cell[0] + 1):
                    if self.occupied((x, y, z)):
                        return False
        return True

    @functools.cache
    def center_is_clear(
        self, point: tuple[float, float, float], footprint: Footprint
    ) -> bool:
        center_x, center_y, _ = self.world_to_cell(point)
        bounds = self.bounds
        minimum_z = math.floor(
            (point[2] - footprint.lower_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        maximum_z = math.floor(
            (point[2] + footprint.upper_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        # Account for the finite voxel face rather than testing voxel centers only.
        conservative_radius_m = footprint.radius_m + bounds.resolution_m / math.sqrt(2)
        cell_radius = math.ceil(conservative_radius_m / bounds.resolution_m)
        for z in range(minimum_z, maximum_z + 1):
            for delta_y in range(-cell_radius, cell_radius + 1):
                for delta_x in range(-cell_radius, cell_radius + 1):
                    if (
                        math.hypot(delta_x, delta_y) * bounds.resolution_m
                        > conservative_radius_m
                    ):
                        continue
                    if self.occupied((center_x + delta_x, center_y + delta_y, z)):
                        return False
        return True

    @functools.cache
    def planar_clearance_mask(
        self, altitude_m: float, footprint: Footprint
    ) -> bytearray:
        """Return one byte per XY cell, set when the full footprint is clear."""
        bounds = self.bounds
        minimum_z = math.floor(
            (altitude_m - footprint.lower_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        maximum_z = math.floor(
            (altitude_m + footprint.upper_extent_m - bounds.origin_z_m)
            / bounds.resolution_m
        )
        raw_blocked = bytearray(bounds.width * bounds.height)
        for y in range(bounds.height):
            row_offset = y * bounds.width
            for x in range(bounds.width):
                if any(
                    self.occupied((x, y, z))
                    for z in range(minimum_z, maximum_z + 1)
                ):
                    raw_blocked[row_offset + x] = 1

        conservative_radius_m = footprint.radius_m + bounds.resolution_m / math.sqrt(2)
        cell_radius = math.ceil(conservative_radius_m / bounds.resolution_m)
        offsets = [
            (delta_x, delta_y)
            for delta_y in range(-cell_radius, cell_radius + 1)
            for delta_x in range(-cell_radius, cell_radius + 1)
            if math.hypot(delta_x, delta_y) * bounds.resolution_m
            <= conservative_radius_m
        ]
        clear = bytearray(b"\x01") * (bounds.width * bounds.height)
        for y in range(bounds.height):
            row_offset = y * bounds.width
            for x in range(bounds.width):
                if not raw_blocked[row_offset + x]:
                    continue
                for delta_x, delta_y in offsets:
                    blocked_x = x + delta_x
                    blocked_y = y + delta_y
                    if 0 <= blocked_x < bounds.width and 0 <= blocked_y < bounds.height:
                        clear[blocked_y * bounds.width + blocked_x] = 0
        for y in range(bounds.height):
            for x in range(bounds.width):
                if (
                    x < cell_radius
                    or x >= bounds.width - cell_radius
                    or y < cell_radius
                    or y >= bounds.height - cell_radius
                ):
                    clear[y * bounds.width + x] = 0
        return clear


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--planner-config", type=Path, default=DEFAULT_PLANNER_CONFIG)
    parser.add_argument("--static-route-tracking-margin-m", type=float)
    parser.add_argument("--minimum-route-length-m", type=float, default=150.0)
    return parser.parse_args()


def load_footprints(
    config_path: Path, tracking_margin_override_m: float | None = None
) -> tuple[Footprint, Footprint]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = config["production_mppi_node"]["ros__parameters"]
    physical = Footprint(
        radius_m=float(parameters["physical_footprint_radius_m"]),
        lower_extent_m=float(parameters["physical_footprint_lower_extent_m"]),
        upper_extent_m=float(parameters["physical_footprint_upper_extent_m"]),
    )
    margin = (
        float(parameters["static_route_tracking_margin_m"])
        if tracking_margin_override_m is None
        else tracking_margin_override_m
    )
    if not math.isfinite(margin) or margin < 0.0:
        raise ScenarioValidationError("static route tracking margin must be non-negative")
    route = Footprint(
        radius_m=physical.radius_m + margin,
        lower_extent_m=physical.lower_extent_m + margin,
        upper_extent_m=physical.upper_extent_m + margin,
    )
    return physical, route


def canonical_world_path(scenario_path: Path, scenario: dict) -> Path:
    path = Path(scenario["canonical_world"])
    return path if path.is_absolute() else (scenario_path.parent / path).resolve()


def load_launch_platforms(scenario: dict) -> dict[str, LaunchPlatform]:
    vehicles = scenario["vehicles"]
    starts = {
        vehicle["id"]: tuple(float(value) for value in vehicle["map_start_m"])
        for vehicle in vehicles
    }
    platforms = scenario.get("launch_platforms", [])
    if not isinstance(platforms, list):
        raise ScenarioValidationError("launch_platforms must be an array")
    result: dict[str, LaunchPlatform] = {}
    platform_ids: set[str] = set()
    for index, source in enumerate(platforms):
        if not isinstance(source, dict):
            raise ScenarioValidationError(
                f"launch_platforms[{index}] must be an object"
            )
        platform_id = source.get("id")
        if (
            not isinstance(platform_id, str)
            or not platform_id
            or platform_id in platform_ids
        ):
            raise ScenarioValidationError("launch platform id is invalid or duplicate")
        platform_ids.add(platform_id)
        vehicle_ids = source.get("vehicle_ids")
        if not isinstance(vehicle_ids, list) or not vehicle_ids:
            raise ScenarioValidationError(
                f"launch platform {platform_id} must list vehicle_ids"
            )
        if any(vehicle_id not in starts for vehicle_id in vehicle_ids):
            raise ScenarioValidationError(
                f"launch platform {platform_id} references an unknown vehicle"
            )
        if any(vehicle_id in result for vehicle_id in vehicle_ids):
            raise ScenarioValidationError("vehicle belongs to multiple launch platforms")
        size = tuple(float(value) for value in source.get("size_m", []))
        if len(size) != 3 or not all(
            math.isfinite(value) and value > 0.0 for value in size
        ):
            raise ScenarioValidationError(
                f"launch platform {platform_id} size_m is invalid"
            )
        top_z = float(source.get("top_z_m"))
        if not math.isfinite(top_z):
            raise ScenarioValidationError(
                f"launch platform {platform_id} top_z_m is invalid"
            )
        platform = LaunchPlatform(
            id=platform_id,
            vehicle_ids=tuple(vehicle_ids),
            center_x_m=sum(starts[vehicle_id][0] for vehicle_id in vehicle_ids)
            / len(vehicle_ids),
            center_y_m=sum(starts[vehicle_id][1] for vehicle_id in vehicle_ids)
            / len(vehicle_ids),
            size_x_m=size[0],
            size_y_m=size[1],
            bottom_z_m=top_z - size[2],
            top_z_m=top_z,
        )
        for vehicle_id in vehicle_ids:
            result[vehicle_id] = platform
    if platforms and set(result) != set(starts):
        raise ScenarioValidationError("every vehicle must have one launch platform")
    return result


def platform_supports_spawn(
    platform: LaunchPlatform,
    start: tuple[float, float, float],
    footprint: Footprint,
    maximum_gap_m: float = 0.25,
) -> bool:
    footprint_bottom_m = start[2] - footprint.lower_extent_m
    return (
        abs(start[0] - platform.center_x_m) + footprint.radius_m
        <= platform.size_x_m / 2.0
        and abs(start[1] - platform.center_y_m) + footprint.radius_m
        <= platform.size_y_m / 2.0
        and -1.0e-6 <= footprint_bottom_m - platform.top_z_m <= maximum_gap_m
    )


def shortest_planar_route_m(
    occupancy: Occupancy3D,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    footprint: Footprint,
) -> float | None:
    if not math.isclose(start[2], goal[2], abs_tol=1e-6):
        raise ScenarioValidationError("fixed-height proof requires matching route Z")
    start_cell = occupancy.world_to_cell(start)[:2]
    goal_cell = occupancy.world_to_cell(goal)[:2]
    clear = occupancy.planar_clearance_mask(start[2], footprint)
    bounds = occupancy.bounds
    for name, cell in (("start", start_cell), ("goal", goal_cell)):
        if not clear[cell[1] * bounds.width + cell[0]]:
            raise ScenarioValidationError(f"route footprint is blocked at {name}")
    queue = collections.deque([start_cell])
    distance = {start_cell: 0}
    while queue:
        cell = queue.popleft()
        if cell == goal_cell:
            return distance[cell] * occupancy.bounds.resolution_m
        for neighbour in (
            (cell[0] - 1, cell[1]),
            (cell[0] + 1, cell[1]),
            (cell[0], cell[1] - 1),
            (cell[0], cell[1] + 1),
        ):
            if neighbour in distance:
                continue
            if not (
                0 <= neighbour[0] < bounds.width
                and 0 <= neighbour[1] < bounds.height
            ):
                continue
            if clear[neighbour[1] * bounds.width + neighbour[0]]:
                distance[neighbour] = distance[cell] + 1
                queue.append(neighbour)
    return None


def planar_segment_is_clear(
    occupancy: Occupancy3D,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    footprint: Footprint,
) -> bool:
    """Check a segment against the same conservative planar route mask."""
    if not math.isclose(start[2], goal[2], abs_tol=1.0e-6):
        return False
    clear = occupancy.planar_clearance_mask(start[2], footprint)
    bounds = occupancy.bounds
    length_m = math.dist(start[:2], goal[:2])
    samples = max(1, math.ceil(length_m / (bounds.resolution_m / 2.0)))
    for index in range(samples + 1):
        fraction = index / samples
        point = (
            start[0] + fraction * (goal[0] - start[0]),
            start[1] + fraction * (goal[1] - start[1]),
            start[2],
        )
        x, y, _ = occupancy.world_to_cell(point)
        if not clear[y * bounds.width + x]:
            return False
    return True


def validate(args: argparse.Namespace) -> None:
    scenario_path = args.scenario.resolve()
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    world = json.loads(
        canonical_world_path(scenario_path, scenario).read_text(encoding="utf-8")
    )
    initial_altitude_m = float(world["navigation"]["initial_altitude_m"])
    occupancy = Occupancy3D.load(args.occupancy.resolve())
    physical_footprint, route_footprint = load_footprints(
        args.planner_config.resolve(), args.static_route_tracking_margin_m
    )
    vehicles = scenario["vehicles"]
    if len(vehicles) < 2 or len(vehicles) % 2 != 0:
        raise ScenarioValidationError("cooperative crossing scenario needs two equal groups")

    start_positions = {
        vehicle["id"]: tuple(float(value) for value in vehicle["map_start_m"])
        for vehicle in vehicles
    }
    launch_platforms = load_launch_platforms(scenario)
    checked_platforms: set[str] = set()
    for vehicle in vehicles:
        vehicle_id = vehicle["id"]
        start = start_positions[vehicle_id]
        goal = tuple(float(value) for value in vehicle["goal_m"])
        takeoff = (start[0], start[1], initial_altitude_m)
        if not occupancy.center_is_clear(start, physical_footprint):
            raise ScenarioValidationError(
                f"{vehicle_id} physical spawn footprint intersects Occupancy3D"
            )
        platform = launch_platforms.get(vehicle_id)
        if platform is None:
            if not occupancy.spawn_has_support(start, physical_footprint):
                raise ScenarioValidationError(
                    f"{vehicle_id} physical spawn has no support surface"
                )
        else:
            if not platform_supports_spawn(platform, start, physical_footprint):
                raise ScenarioValidationError(
                    f"{vehicle_id} is not physically supported by launch platform"
                )
            if platform.id not in checked_platforms:
                fixture_center = (
                    platform.center_x_m,
                    platform.center_y_m,
                    (platform.bottom_z_m + platform.top_z_m) / 2.0,
                )
                fixture_size = (
                    platform.size_x_m,
                    platform.size_y_m,
                    platform.top_z_m - platform.bottom_z_m,
                )
                if not occupancy.box_is_clear(fixture_center, fixture_size):
                    raise ScenarioValidationError(
                        f"launch platform {platform.id} intersects source Occupancy3D"
                    )
                checked_platforms.add(platform.id)
                print(
                    "STATIC_SCENARIO_LAUNCH_PLATFORM"
                    f" id={platform.id} vehicles={','.join(platform.vehicle_ids)}"
                    f" center=({platform.center_x_m:.3f},{platform.center_y_m:.3f})"
                    f" top_z_m={platform.top_z_m:.3f} status=valid"
                )
        if not occupancy.vertical_sweep_is_clear(
            start, initial_altitude_m, physical_footprint
        ):
            raise ScenarioValidationError(
                f"{vehicle_id} vertical takeoff footprint intersects Occupancy3D"
            )
        route_clearance = occupancy.planar_clearance_mask(
            initial_altitude_m, route_footprint
        )
        takeoff_cell = occupancy.world_to_cell(takeoff)
        if not route_clearance[
            takeoff_cell[1] * occupancy.bounds.width + takeoff_cell[0]
        ]:
            raise ScenarioValidationError(
                f"{vehicle_id} route footprint is blocked at takeoff"
            )
        if not math.isclose(goal[2], initial_altitude_m, abs_tol=1e-6):
            raise ScenarioValidationError(
                f"{vehicle_id} goal Z does not match initial altitude"
            )
        goal_cell = occupancy.world_to_cell(goal)
        if not route_clearance[goal_cell[1] * occupancy.bounds.width + goal_cell[0]]:
            raise ScenarioValidationError(
                f"{vehicle_id} route footprint is blocked at goal"
            )
        route_length_m = shortest_planar_route_m(
            occupancy, takeoff, goal, route_footprint
        )
        if route_length_m is None:
            raise ScenarioValidationError(
                f"{vehicle_id} start and goal are not in one route-safe component"
            )
        if route_length_m < args.minimum_route_length_m:
            raise ScenarioValidationError(
                f"{vehicle_id} route is too short: {route_length_m:.1f} m"
            )
        if not planar_segment_is_clear(
            occupancy, takeoff, goal, route_footprint
        ):
            raise ScenarioValidationError(
                f"{vehicle_id} has no direct swept-footprint route"
            )
        direct_length_m = math.dist(takeoff[:2], goal[:2])
        print(
            "STATIC_SCENARIO_ROUTE"
            f" vehicle={vehicle_id} direct_m={direct_length_m:.1f}"
            f" geodesic_m={route_length_m:.1f}"
            f" altitude_m={initial_altitude_m:.1f} direct_clear=true status=valid"
        )


def main() -> int:
    args = parse_args()
    try:
        validate(args)
    except (KeyError, OSError, ValueError, ScenarioValidationError) as error:
        print(f"STATIC_SCENARIO_VALIDATION status=failed reason={error}")
        return 1
    print("STATIC_SCENARIO_VALIDATION status=passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
