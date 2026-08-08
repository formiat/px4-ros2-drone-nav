#!/usr/bin/env python3
"""Load and validate the canonical finite intercept scenario."""

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


def _finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
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


def load_intercept_scenario(path: str | Path) -> dict[str, Any]:
    """Return a validated scenario with derived Gazebo spawn poses."""
    scenario_path = Path(path).resolve()
    with scenario_path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("schema") != "drone_city_nav_intercept_scenario_v1":
        raise ValueError("unsupported intercept scenario schema")

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
        if role not in {"interceptor", "evader"}:
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
    if actual_ids != EXPECTED_VEHICLE_IDS:
        raise ValueError(
            f"vehicle order must be {EXPECTED_VEHICLE_IDS}, got {actual_ids}"
        )
    if sum(vehicle["role"] == "evader" for vehicle in vehicles) != 1:
        raise ValueError("scenario must contain exactly one evader")

    return {
        "path": scenario_path,
        "canonical_world_path": world_path,
        "map_to_sdf": transform,
        "vehicles": vehicles,
        "evader_goal_m": _finite_vector(
            document.get("evader_goal_m"), 3, "evader_goal_m"
        ),
    }


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
    scenario = load_intercept_scenario(args.scenario)
    _print_tsv(scenario)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
