#!/usr/bin/env python3
"""Load and validate one point-to-point navigation scenario."""

from __future__ import annotations

import argparse
import json
import math
import re
import runpy
from pathlib import Path
from typing import Any


SCHEMA = "drone_city_nav_point_to_point_scenario_v2"
_GAZEBO_NAME_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_LIDAR_PROFILE_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("lidar_profile.py"))
)
_DEFAULT_LIDAR_PROFILE = _LIDAR_PROFILE_SUPPORT["DEFAULT_LIDAR_PROFILE"]
_resolve_lidar_model_identity = _LIDAR_PROFILE_SUPPORT["resolve_model_identity"]
_PX4_MAP_FRAME_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("px4_map_frame.py"))
)
_px4_to_map_matrix = _PX4_MAP_FRAME_SUPPORT["px4_to_map_matrix"]
_map_to_sdf_swaps_axes = _PX4_MAP_FRAME_SUPPORT["map_to_sdf_swaps_axes"]


def _finite_vector(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must contain three numeric values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{label} must contain finite values")
    return result


def _waypoint_sequence(value: Any) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, list) or not value:
        raise ValueError("mission_goal_sequence_m must be a non-empty array")
    return tuple(
        _finite_vector(waypoint, f"mission_goal_sequence_m[{index}]")
        for index, waypoint in enumerate(value)
    )


def _required_string(document: dict[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _navigation_profile(
    world: dict[str, Any], scenario: dict[str, Any]
) -> dict[str, float]:
    world_source = world.get("navigation", {})
    scenario_source = scenario.get("navigation", {})
    if not isinstance(world_source, dict):
        raise ValueError("canonical world navigation must be an object")
    if not isinstance(scenario_source, dict):
        raise ValueError("scenario navigation must be an object")
    supported = {
        "initial_altitude_m",
        "minimum_target_z_m",
        "maximum_target_z_m",
    }
    unknown = set(scenario_source) - supported
    if unknown:
        raise ValueError(f"unsupported scenario navigation fields: {sorted(unknown)}")
    defaults = {
        "initial_altitude_m": 18.0,
        "minimum_target_z_m": 1.0,
        "maximum_target_z_m": 32.0,
    }
    navigation = {
        key: float(scenario_source.get(key, world_source.get(key, default)))
        for key, default in defaults.items()
    }
    if not all(math.isfinite(value) for value in navigation.values()):
        raise ValueError("navigation values must be finite")
    if not (
        navigation["minimum_target_z_m"]
        <= navigation["initial_altitude_m"]
        < navigation["maximum_target_z_m"]
    ):
        raise ValueError("scenario initial altitude is outside its flight envelope")
    return navigation


def _map_to_sdf(
    point: tuple[float, float, float], transform: dict[str, Any]
) -> tuple[float, float, float]:
    source_axes = (transform.get("sdf_x_from"), transform.get("sdf_y_from"))
    if set(source_axes) != {"map_x", "map_y"}:
        raise ValueError("map_to_sdf must map distinct map axes to SDF X/Y")
    map_axes = {"map_x": point[0], "map_y": point[1]}
    scales = (
        float(transform.get("sdf_x_scale", 1.0)),
        float(transform.get("sdf_y_scale", 1.0)),
        float(transform.get("sdf_z_scale", 1.0)),
    )
    offsets = (
        float(transform["sdf_x_offset_m"]),
        float(transform["sdf_y_offset_m"]),
        float(transform.get("sdf_z_offset_m", 0.0)),
    )
    if not all(math.isfinite(value) for value in (*scales, *offsets)):
        raise ValueError("map_to_sdf scales and offsets must be finite")
    return (
        scales[0] * map_axes[source_axes[0]] + offsets[0],
        scales[1] * map_axes[source_axes[1]] + offsets[1],
        scales[2] * point[2] + offsets[2],
    )


def _launch_platforms(document: dict[str, Any]) -> tuple[dict[str, Any], ...]:
    platforms = document.get("launch_platforms", [])
    if not isinstance(platforms, list):
        raise ValueError("launch_platforms must be an array")
    result: list[dict[str, Any]] = []
    identifiers: set[str] = set()
    for index, platform in enumerate(platforms):
        if not isinstance(platform, dict):
            raise ValueError(f"launch_platforms[{index}] must be an object")
        identifier = _required_string(platform, "id")
        if identifier in identifiers:
            raise ValueError("launch platform ids must be unique")
        identifiers.add(identifier)
        vehicle_ids = platform.get("vehicle_ids")
        if vehicle_ids != ["point_to_point_vehicle"]:
            raise ValueError(
                "point-to-point launch platform must target point_to_point_vehicle"
            )
        size = _finite_vector(
            platform.get("size_m"), f"launch platform {identifier} size_m"
        )
        if any(component <= 0.0 for component in size):
            raise ValueError(f"launch platform {identifier} size must be positive")
        top_z_m = float(platform.get("top_z_m"))
        if not math.isfinite(top_z_m):
            raise ValueError(f"launch platform {identifier} top_z_m must be finite")
        result.append(
            {
                "id": identifier,
                "size_m": size,
                "top_z_m": top_z_m,
            }
        )
    if len(result) > 1:
        raise ValueError("point-to-point scenario supports at most one launch platform")
    return tuple(result)
def load_point_to_point_scenario(
    path: str | Path, lidar_profile: str = _DEFAULT_LIDAR_PROFILE
) -> dict[str, Any]:
    """Return a validated point-to-point scenario with its Gazebo spawn pose."""
    scenario_path = Path(path).resolve()
    document = json.loads(scenario_path.read_text(encoding="utf-8"))
    if document.get("schema") != SCHEMA:
        raise ValueError("unsupported point-to-point scenario schema")
    world_path = Path(_required_string(document, "canonical_world"))
    if not world_path.is_absolute():
        world_path = scenario_path.parent / world_path
    world_path = world_path.resolve()
    world = json.loads(world_path.read_text(encoding="utf-8"))
    world_name = world.get("gazebo_world_name")
    if not isinstance(world_name, str) or _GAZEBO_NAME_PATTERN.fullmatch(world_name) is None:
        raise ValueError("canonical world gazebo_world_name is invalid")
    transform = world.get("map_to_sdf")
    if not isinstance(transform, dict):
        raise ValueError("canonical world is missing map_to_sdf")
    navigation = _navigation_profile(world, document)

    vehicle = document.get("vehicle")
    if not isinstance(vehicle, dict):
        raise ValueError("vehicle must be an object")
    start = _finite_vector(vehicle.get("map_start_m"), "vehicle.map_start_m")
    mission_goal_sequence = _waypoint_sequence(
        document.get("mission_goal_sequence_m")
    )
    yaw_rad = float(vehicle.get("yaw_rad", 0.0))
    if not math.isfinite(yaw_rad):
        raise ValueError("vehicle.yaw_rad must be finite")
    initial_altitude = navigation["initial_altitude_m"]
    minimum_target_z = navigation["minimum_target_z_m"]
    maximum_target_z = navigation["maximum_target_z_m"]
    for waypoint in mission_goal_sequence:
        if not minimum_target_z <= waypoint[2] < maximum_target_z:
            raise ValueError("mission waypoint is outside the canonical flight envelope")

    px4_model_target, gazebo_model_name = _resolve_lidar_model_identity(
        _required_string(vehicle, "px4_model_target"),
        _required_string(vehicle, "gazebo_model_name"),
        lidar_profile,
    )
    return {
        "path": scenario_path,
        "canonical_world_path": world_path,
        "gazebo_world_name": world_name,
        "px4_model_target": px4_model_target,
        "gazebo_model_name": gazebo_model_name,
        "map_start_m": start,
        "gazebo_spawn_m": _map_to_sdf(start, transform),
        "px4_to_map_matrix": _px4_to_map_matrix(transform),
        "map_to_sdf": transform,
        "gazebo_axes_swapped": _map_to_sdf_swaps_axes(transform),
        "yaw_rad": yaw_rad,
        "mission_goal_sequence_m": mission_goal_sequence,
        "initial_altitude_m": initial_altitude,
        "minimum_target_z_m": minimum_target_z,
        "maximum_target_z_m": maximum_target_z,
        "launch_platforms": _launch_platforms(document),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--format", choices=("runtime-tsv",), required=True)
    parser.add_argument(
        "--lidar-profile",
        choices=("none", "3d"),
        default=_DEFAULT_LIDAR_PROFILE,
    )
    args = parser.parse_args()
    try:
        scenario = load_point_to_point_scenario(args.scenario, args.lidar_profile)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"POINT_TO_POINT_SCENARIO status=failed reason={error}")
        return 1
    start = scenario["map_start_m"]
    spawn = scenario["gazebo_spawn_m"]
    print(
        "\t".join(
            str(value)
            for value in (
                scenario["gazebo_world_name"],
                scenario["px4_model_target"],
                scenario["gazebo_model_name"],
                *start,
                *spawn,
                scenario["yaw_rad"],
            )
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
