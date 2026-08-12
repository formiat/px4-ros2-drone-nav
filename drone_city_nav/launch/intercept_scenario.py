#!/usr/bin/env python3
"""Load and validate a canonical finite multi-vehicle scenario."""

from __future__ import annotations

import argparse
import json
import math
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
    if transform.get("sdf_x_from") != "map_y":
        raise ValueError("canonical world must derive SDF X from map Y")
    if transform.get("sdf_y_from") != "map_x":
        raise ValueError("canonical world must derive SDF Y from map X")
    offset_x = float(transform["sdf_x_offset_m"])
    offset_y = float(transform["sdf_y_offset_m"])
    if not math.isfinite(offset_x) or not math.isfinite(offset_y):
        raise ValueError("map_to_sdf offsets must be finite")
    return (
        map_position[1] + offset_x,
        map_position[0] + offset_y,
        map_position[2],
    )


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
    transform = world.get("map_to_sdf")
    if not isinstance(transform, dict):
        raise ValueError("canonical world is missing map_to_sdf")

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
        "map_to_sdf": transform,
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--format", choices=("tsv",), default="tsv")
    args = parser.parse_args()
    scenario = load_multi_vehicle_scenario(args.scenario)
    _print_tsv(scenario)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
