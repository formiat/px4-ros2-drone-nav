#!/usr/bin/env python3
"""Load and validate a canonical finite multi-vehicle scenario."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any


EXPECTED_VEHICLE_IDS = (
    "interceptor_0",
    "interceptor_1",
    "interceptor_2",
    "evader",
)

SUPPORTED_SCHEMAS = {
    "drone_city_nav_intercept_scenario_v1",
    "drone_city_nav_intercept_scenario_v2",
    "drone_city_nav_cooperative_traffic_scenario_v1",
}

_GAZEBO_NAME_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


def _finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} must contain {length} numeric values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def _required_string(document: dict[str, Any], key: str, label: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label}.{key} must be a non-empty string")
    return value


def _resolve_world_path(scenario_path: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("canonical_world must be a non-empty path")
    path = Path(value)
    if not path.is_absolute():
        path = scenario_path.parent / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"canonical world does not exist: {path}")
    return path


def _map_to_sdf(
    map_position: tuple[float, float, float], transform: dict[str, Any]
) -> tuple[float, float, float]:
    source_axes = (transform.get("sdf_x_from"), transform.get("sdf_y_from"))
    if set(source_axes) != {"map_x", "map_y"}:
        raise ValueError(
            "map_to_sdf must map distinct map_x/map_y axes to SDF X/Y"
        )
    offset_x = float(transform["sdf_x_offset_m"])
    offset_y = float(transform["sdf_y_offset_m"])
    offset_z = float(transform.get("sdf_z_offset_m", 0.0))
    scale_x = float(transform.get("sdf_x_scale", 1.0))
    scale_y = float(transform.get("sdf_y_scale", 1.0))
    scale_z = float(transform.get("sdf_z_scale", 1.0))
    if not all(
        math.isfinite(value)
        for value in (offset_x, offset_y, offset_z, scale_x, scale_y, scale_z)
    ):
        raise ValueError("map_to_sdf offsets and scales must be finite")
    if any(abs(abs(scale) - 1.0) > 1.0e-9 for scale in (scale_x, scale_y, scale_z)):
        raise ValueError("map_to_sdf scales must be +1 or -1")
    map_axes = {"map_x": map_position[0], "map_y": map_position[1]}
    return (
        scale_x * map_axes[source_axes[0]] + offset_x,
        scale_y * map_axes[source_axes[1]] + offset_y,
        scale_z * map_position[2] + offset_z,
    )


def _world_navigation(world: dict[str, Any]) -> dict[str, float]:
    source = world.get("navigation", {})
    if not isinstance(source, dict):
        raise ValueError("canonical world navigation must be an object")
    navigation = {
        "initial_altitude_m": float(source.get("initial_altitude_m", 18.0)),
        "minimum_target_z_m": float(source.get("minimum_target_z_m", 1.0)),
        "maximum_target_z_m": float(source.get("maximum_target_z_m", 32.0)),
    }
    if not all(math.isfinite(value) for value in navigation.values()):
        raise ValueError("canonical world navigation values must be finite")
    if not (
        navigation["minimum_target_z_m"]
        <= navigation["initial_altitude_m"]
        < navigation["maximum_target_z_m"]
    ):
        raise ValueError("canonical world initial altitude is outside its envelope")
    return navigation


def _px4_to_map_matrix(transform: dict[str, Any]) -> tuple[float, ...]:
    source_x = transform["sdf_x_from"]
    source_y = transform["sdf_y_from"]
    scale_x = float(transform.get("sdf_x_scale", 1.0))
    scale_y = float(transform.get("sdf_y_scale", 1.0))
    sdf_x_row = (
        scale_x if source_x == "map_x" else 0.0,
        scale_x if source_x == "map_y" else 0.0,
    )
    sdf_y_row = (
        scale_y if source_y == "map_x" else 0.0,
        scale_y if source_y == "map_y" else 0.0,
    )
    return (sdf_y_row[0], sdf_x_row[0], sdf_y_row[1], sdf_x_row[1])


def load_multi_vehicle_scenario(path: str | Path) -> dict[str, Any]:
    """Return a validated scenario with derived Gazebo spawn poses."""
    scenario_path = Path(path).resolve()
    with scenario_path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    schema = document.get("schema")
    if schema not in SUPPORTED_SCHEMAS:
        raise ValueError("unsupported multi-vehicle scenario schema")

    world_path = _resolve_world_path(scenario_path, document.get("canonical_world"))
    with world_path.open(encoding="utf-8") as stream:
        world = json.load(stream)
    gazebo_world_name = world.get("gazebo_world_name", "generated_city")
    if (
        not isinstance(gazebo_world_name, str)
        or _GAZEBO_NAME_PATTERN.fullmatch(gazebo_world_name) is None
    ):
        raise ValueError("canonical world gazebo_world_name is invalid")
    transform = world.get("map_to_sdf")
    if not isinstance(transform, dict):
        raise ValueError("canonical world is missing map_to_sdf")
    navigation = _world_navigation(world)

    source_vehicles = document.get("vehicles")
    if not isinstance(source_vehicles, list):
        raise ValueError("vehicles must be an array")
    vehicles: list[dict[str, Any]] = []
    for index, source in enumerate(source_vehicles):
        if not isinstance(source, dict):
            raise ValueError(f"vehicles[{index}] must be an object")
        vehicle_id = _required_string(source, "id", f"vehicles[{index}]")
        role = _required_string(source, "role", f"vehicles[{index}]")
        if role not in {"interceptor", "evader", "civilian"}:
            raise ValueError(f"invalid role for {vehicle_id}: {role}")
        map_start = _finite_vector(
            source.get("map_start_m"), 3, f"vehicles[{index}].map_start_m"
        )
        yaw_rad = float(source.get("yaw_rad", 0.0))
        if not math.isfinite(yaw_rad):
            raise ValueError(f"invalid yaw for {vehicle_id}")
        vehicles.append(
            {
                "id": vehicle_id,
                "role": role,
                "px4_namespace": _required_string(
                    source, "px4_namespace", f"vehicles[{index}]"
                ),
                "px4_model_target": _required_string(
                    source, "px4_model_target", f"vehicles[{index}]"
                ),
                "gazebo_model_name": _required_string(
                    source, "gazebo_model_name", f"vehicles[{index}]"
                ),
                "map_start_m": map_start,
                "gazebo_spawn_m": _map_to_sdf(map_start, transform),
                "yaw_rad": yaw_rad,
            }
        )

    actual_ids = tuple(vehicle["id"] for vehicle in vehicles)
    if len(set(actual_ids)) != len(actual_ids):
        raise ValueError("vehicle ids must be unique")
    namespaces = tuple(vehicle["px4_namespace"] for vehicle in vehicles)
    if len(set(namespaces)) != len(namespaces):
        raise ValueError("PX4 namespaces must be unique")
    models = tuple(vehicle["gazebo_model_name"] for vehicle in vehicles)
    if len(set(models)) != len(models):
        raise ValueError("Gazebo model names must be unique")

    interceptor_count = sum(
        vehicle["role"] == "interceptor" for vehicle in vehicles
    )
    evader_count = sum(vehicle["role"] == "evader" for vehicle in vehicles)
    civilian_count = sum(vehicle["role"] == "civilian" for vehicle in vehicles)
    cooperative_schema = schema == "drone_city_nav_cooperative_traffic_scenario_v1"
    if cooperative_schema:
        if civilian_count < 2 or civilian_count != len(vehicles):
            raise ValueError(
                "cooperative traffic scenario must contain only civilian vehicles"
            )
    elif interceptor_count == 0 or evader_count == 0 or civilian_count != 0:
        raise ValueError("intercept scenario must contain interceptors and evaders")
    if schema == "drone_city_nav_intercept_scenario_v1":
        if actual_ids != EXPECTED_VEHICLE_IDS:
            raise ValueError(
                f"vehicle order must be {EXPECTED_VEHICLE_IDS}, got {actual_ids}"
            )
        if evader_count != 1:
            raise ValueError("legacy scenario must contain exactly one evader")

    default_evader_goal = document.get("evader_goal_m")
    if default_evader_goal is not None:
        default_evader_goal = _finite_vector(
            default_evader_goal, 3, "evader_goal_m"
        )
    evaders = []
    vehicle_goals = []
    detection_ids: set[int] = set()
    for vehicle, source_vehicle in zip(vehicles, source_vehicles, strict=True):
        if vehicle["role"] == "civilian":
            source_goal = source_vehicle.get("goal_m")
            if source_goal is None:
                raise ValueError(f"civilian {vehicle['id']} is missing goal_m")
            vehicle_goals.append(
                {
                    "id": vehicle["id"],
                    "goal_m": _finite_vector(
                        source_goal, 3, f"vehicle {vehicle['id']} goal_m"
                    ),
                }
            )
            continue
        if vehicle["role"] != "evader":
            continue
        source_detection_id = source_vehicle.get("detection_id")
        if source_detection_id is None:
            if schema == "drone_city_nav_intercept_scenario_v2":
                raise ValueError(
                    f"evader {vehicle['id']} is missing detection_id"
                )
            detection_id = len(detection_ids) + 1
        elif not isinstance(source_detection_id, int) or isinstance(
            source_detection_id, bool
        ):
            raise ValueError(f"invalid detection_id for evader {vehicle['id']}")
        else:
            detection_id = source_detection_id
        if detection_id <= 0 or detection_id in detection_ids:
            raise ValueError(
                f"detection_id for evader {vehicle['id']} must be positive and unique"
            )
        detection_ids.add(detection_id)
        source_goal = source_vehicle.get("goal_m", default_evader_goal)
        if source_goal is None:
            raise ValueError(f"evader {vehicle['id']} is missing goal_m")
        evaders.append(
            {
                "id": vehicle["id"],
                "goal_m": _finite_vector(
                    source_goal, 3, f"vehicle {vehicle['id']} goal_m"
                ),
                "detection_id": detection_id,
            }
        )
        vehicle_goals.append({"id": vehicle["id"], "goal_m": evaders[-1]["goal_m"]})

    mission_name = document.get("mission_name", "intercept")
    if not isinstance(mission_name, str) or not mission_name:
        raise ValueError("mission_name must be a non-empty string")
    if cooperative_schema:
        if mission_name != "cooperative_traffic":
            raise ValueError(
                "cooperative traffic scenario mission_name must be cooperative_traffic"
            )
        goal_altitudes = {goal["goal_m"][2] for goal in vehicle_goals}
        start_altitudes = {vehicle["map_start_m"][2] for vehicle in vehicles}
        if len(goal_altitudes) != 1 or len(start_altitudes) != 1:
            raise ValueError(
                "cooperative traffic vehicles must share start and cruise altitudes"
            )

    return {
        "schema": schema,
        "mission_name": mission_name,
        "path": scenario_path,
        "canonical_world_path": world_path,
        "gazebo_world_name": gazebo_world_name,
        "map_to_sdf": transform,
        "px4_to_map_matrix": _px4_to_map_matrix(transform),
        "navigation": navigation,
        "vehicles": vehicles,
        "interceptor_ids": [
            vehicle["id"]
            for vehicle in vehicles
            if vehicle["role"] == "interceptor"
        ],
        "civilian_ids": [
            vehicle["id"] for vehicle in vehicles if vehicle["role"] == "civilian"
        ],
        "vehicle_goals": vehicle_goals,
        "evaders": evaders,
        "evader_goal_m": evaders[0]["goal_m"] if evaders else None,
    }


def load_intercept_scenario(path: str | Path) -> dict[str, Any]:
    """Return an intercept scenario while preserving the legacy public loader."""
    scenario = load_multi_vehicle_scenario(path)
    if not scenario["evaders"] or scenario["civilian_ids"]:
        raise ValueError("scenario is not an intercept mission")
    return scenario


def _print_tsv(scenario: dict[str, Any]) -> None:
    for vehicle in scenario["vehicles"]:
        map_x, map_y, map_z = vehicle["map_start_m"]
        sdf_x, sdf_y, sdf_z = vehicle["gazebo_spawn_m"]
        print(
            "\t".join(
                (
                    vehicle["id"],
                    vehicle["role"],
                    vehicle["px4_namespace"],
                    vehicle["px4_model_target"],
                    vehicle["gazebo_model_name"],
                    f"{map_x:.6f}",
                    f"{map_y:.6f}",
                    f"{map_z:.6f}",
                    f"{sdf_x:.6f}",
                    f"{sdf_y:.6f}",
                    f"{sdf_z:.6f}",
                    f"{vehicle['yaw_rad']:.9f}",
                )
            )
        )


def _print_metadata_tsv(scenario: dict[str, Any]) -> None:
    navigation = scenario["navigation"]
    print(
        "\t".join(
            (
                scenario["gazebo_world_name"],
                f"{navigation['initial_altitude_m']:.6f}",
                f"{navigation['minimum_target_z_m']:.6f}",
                f"{navigation['maximum_target_z_m']:.6f}",
            )
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument(
        "--format", choices=("tsv", "metadata-tsv"), default="tsv"
    )
    args = parser.parse_args()
    scenario = load_multi_vehicle_scenario(args.scenario)
    if args.format == "metadata-tsv":
        _print_metadata_tsv(scenario)
    else:
        _print_tsv(scenario)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
