#!/usr/bin/env python3
"""Validate the production MPPI headless navigation contract."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from headless_topology_validation import (
    parse_route_volume_bounds,
    validate_observed_3d_route_volume,
)
from controller_dynamics_evidence import validate_controller_dynamics  # noqa: E402
from resource_budget_evidence import validate_resource_budget  # noqa: E402
from headless_runtime_evidence import (
    validate_localization_profile,
    validate_mean_flight_speed,
    validate_persistent_3d_acceptance_metrics,
    validate_runtime_manifest,
)


CRITICAL_PX4_PATTERN = re.compile(
    r"(?:ERROR \[|Critical failure|Segmentation fault)",
    re.IGNORECASE,
)


def parse_bool(value: str) -> bool | None:
    normalized = value.strip().lower()
    if not normalized:
        return None
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value}")


def require(label: str, text: str, pattern: str, errors: list[str]) -> None:
    if re.search(pattern, text):
        print(f"OK: {label}")
    else:
        errors.append(f"FAIL: {label}")


def require_count(
    label: str, text: str, pattern: str, minimum: int, errors: list[str]
) -> None:
    count = len(re.findall(pattern, text))
    if count >= minimum:
        print(f"OK: {label} ({count})")
    else:
        errors.append(f"FAIL: {label} ({count} < {minimum})")


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def validate_execution_chain(ros_log: str, errors: list[str]) -> None:
    """Require a mission route, publication, offboard admission, and application."""
    require(
        "mission-reaching persistent route is active",
        ros_log,
        r"PRODUCTION_MPPI_TICK .*target_source=persistent_route_3d.*"
        r"route_reaches_mission_goal=true|"
        r"PRODUCTION_MPPI_TICK .*route_reaches_mission_goal=true.*"
        r"target_source=persistent_route_3d",
        errors,
    )
    require(
        "published collision-free planned horizon",
        ros_log,
        r"PRODUCTION_MPPI_TICK .*execution_mode=planned execution_reason=none "
        r"execution_published=true .*finite_path_validation_status=valid .*"
        r"raw_invalidation_active=false",
        errors,
    )
    published = set(re.findall(
        r"EXECUTION_HORIZON published=true producer=([1-9][0-9]*) sequence=([1-9][0-9]*)",
        ros_log,
    ))
    accepted = set(re.findall(
        r"EXECUTION_HORIZON accepted=true producer=([1-9][0-9]*) sequence=([1-9][0-9]*) mode=planned",
        ros_log,
    ))
    applied = set(re.findall(
        r"OFFBOARD_PLANNED_HORIZON_APPLIED producer=([1-9][0-9]*) sequence=([1-9][0-9]*)",
        ros_log,
    ))
    chain = published & accepted & applied
    if chain:
        print(f"OK: exact planner horizon is accepted and applied ({len(chain)})")
    else:
        errors.append("FAIL: exact planner horizon is accepted and applied")
    require(
        "measured execution route progress is positive",
        ros_log,
        r"PRODUCTION_MPPI_TICK .*"
        r"liveness_actual_route_progress_m=(?:0\.0*[1-9][0-9]*|[1-9][0-9]*(?:\.[0-9]+)?) "
        r"liveness_route_progress_used=true",
        errors,
    )


def validate_mapping_pipeline(
    ros_log: str,
    lidar_profile: str,
    expected_memory: bool | None,
    enable_lidar_debug: bool,
    errors: list[str],
) -> None:
    if lidar_profile == "3d":
        scan_pattern = (
            r"LIDAR3D_MEMORY accepted=true stamp_ns=[1-9][0-9]* .*"
            r"hits=[1-9][0-9]* "
            r"misses=[1-9][0-9]*"
        )
        update_pattern = (
            r"ONLINE_OCCUPANCY3D_UPDATE .*revision=[1-9][0-9]* .*"
            r"(?:snapshot=true|delta=true)"
        )
        memory_activity_pattern = (
            r"LIDAR3D_(?:CURRENT_SCAN|MEMORY) accepted=true|"
            r"ONLINE_OCCUPANCY3D_UPDATE"
        )
        scan_label = "3D obstacle memory receives timestamped hit/miss scans"
        update_label = "revisioned Occupancy3D snapshots or deltas are published"
    else:
        scan_pattern = r"First lidar scan|Obstacle memory update:"
        update_pattern = r"Raw obstacle snapshot|raw obstacle snapshot|raw_revision="
        memory_activity_pattern = scan_pattern
        scan_label = "obstacle memory receives lidar"
        update_label = "raw obstacle snapshots are published"

    if expected_memory is not False:
        require(scan_label, ros_log, scan_pattern, errors)
        require(update_label, ros_log, update_pattern, errors)
    elif re.search(memory_activity_pattern, ros_log):
        errors.append("FAIL: obstacle memory is disabled")
    else:
        print("OK: obstacle memory is disabled")

    if not enable_lidar_debug:
        return
    if lidar_profile == "3d":
        require(
            "current 3D lidar cloud is published",
            ros_log,
            r"LIDAR3D_CURRENT_SCAN accepted=true .*debug=true",
            errors,
        )
        require(
            "selected-spectator accumulated 3D memory cloud is published",
            ros_log,
            r"ONLINE_OCCUPANCY3D_UPDATE .*debug_cloud=true",
            errors,
        )
    else:
        require(
            "lidar debug snapshots are written",
            ros_log,
            r"LIDAR_DEBUG snapshot=",
            errors,
        )


def validate_cooperative_traffic(
    ros_log: str,
    expected_vehicles: int,
    expected_memory: bool | None,
    errors: list[str],
    lidar_profile: str,
) -> None:
    require(
        "cooperative ground truth is restricted to the referee",
        ros_log,
        r"COOPERATIVE_GROUND_TRUTH_BOUNDARY verified=true",
        errors,
    )
    require(
        "cooperative navigation and physical coordinates are aligned",
        ros_log,
        r"SIMULATION_TRUTH_ALIGNMENT ready=true failure_confirmed=false "
        r"reason=aligned",
        errors,
    )
    require_count(
        "cooperative agents publish independent flight intents",
        ros_log,
        r"COOPERATIVE_AGENT_READY vehicle_id='civilian_[0-9]+'",
        expected_vehicles,
        errors,
    )
    require(
        "cooperative readiness barrier observes all intents",
        ros_log,
        rf"COOPERATIVE_TRAFFIC_MISSION state=running "
        rf"vehicle_count={expected_vehicles} .*all_intents_ready=true",
        errors,
    )
    require_count(
        "all cooperative vehicles physically settle at their own goals",
        ros_log,
        r"COOPERATIVE_GOAL_HOLD_CONFIRMED vehicle_id='civilian_[0-9]+'",
        expected_vehicles,
        errors,
    )
    result = re.search(
        r"MISSION_RESULT success=true mission=cooperative_traffic "
        r"outcome=all_goals_reached vehicle_count=([0-9]+) "
        r"minimum_physical_separation_m=([0-9.]+) .*"
        r"desired_separation_m=([0-9.]+) "
        r"desired_separation_violation_events=([0-9]+) .*"
        r"physical_collisions=0 building_collisions=0",
        ros_log,
    )
    if result is None:
        errors.append("FAIL: cooperative traffic reports a complete physical result")
    elif int(result.group(1)) != expected_vehicles:
        errors.append("FAIL: cooperative result vehicle count matches the scenario")
    else:
        print(
            "OK: cooperative traffic reaches all goals "
            f"(minimum separation {float(result.group(2)):.3f} m, "
            f"soft-target violations {int(result.group(4))})"
        )
    if re.search(
        r"COOPERATIVE_VEHICLE_DESTROYED referee_observed=true|"
        r"MISSION_RESULT success=false mission=cooperative_traffic",
        ros_log,
    ):
        errors.append("FAIL: cooperative traffic contains a physical loss")
    else:
        print("OK: cooperative traffic has no vehicle destruction")
    if expected_memory is True:
        peer_filter_pattern = (
            r"COOPERATIVE_PEER_LIDAR_FILTER3D filtered_beams=[0-9]+ "
            r"known_peers=[1-9][0-9]* forgotten_voxels=[0-9]+"
            if lidar_profile == "3d"
            else r"COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=[0-9]+ "
            r"matched_peers=[0-9]+ known_peers=[1-9][0-9]*"
        )
        require(
            "cooperative peer memory filtering is active",
            ros_log,
            peer_filter_pattern,
            errors,
        )


def validate_building_collisions(ros_log: str, errors: list[str]) -> None:
    collisions = sorted(
        set(
            re.findall(
                r"VEHICLE_DESTROYED role=[0-9]+ vehicle_id='([^']*)' "
                r"cause=physical_collision .*?obstacle_collision='([^']*"
                r"(?:building|passage_structure_)[^']*)'",
                ros_log,
            )
        )
    )
    if not collisions:
        print("OK: no vehicle collided with a static world obstacle")
        return
    for vehicle_id, obstacle in collisions:
        vehicle_label = vehicle_id or "default_vehicle"
        errors.append(
            f"FAIL: {vehicle_label} collided with static world obstacle '{obstacle}'"
        )


def validate_point_to_point_waypoints(ros_log: str, errors: list[str]) -> None:
    result = re.search(
        r"MISSION_RESULT success=true .*waypoint_count=([0-9]+) "
        r"completed_waypoints=([0-9]+)",
        ros_log,
    )
    if result is None:
        errors.append("FAIL: point-to-point mission reports waypoint completion")
        return
    waypoint_count = int(result.group(1))
    completed_waypoints = int(result.group(2))
    reached_events = len(
        re.findall(
            r"MISSION_WAYPOINT_REACHED completed_index=[0-9]+ "
            r"waypoint_count=[0-9]+",
            ros_log,
        )
    )
    if waypoint_count < 1:
        errors.append("FAIL: point-to-point mission has at least one waypoint")
    elif completed_waypoints != waypoint_count:
        errors.append(
            "FAIL: point-to-point mission completes every configured waypoint "
            f"({completed_waypoints} != {waypoint_count})"
        )
    elif reached_events < waypoint_count:
        errors.append(
            "FAIL: point-to-point mission logs every waypoint arrival "
            f"({reached_events} < {waypoint_count})"
        )
    else:
        print(
            "OK: point-to-point mission completes every waypoint "
            f"({completed_waypoints}/{waypoint_count})"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate production MPPI headless run logs."
    )
    parser.add_argument("--ros-log", required=True, type=Path)
    parser.add_argument("--px4-log", required=True, action="append", type=Path)
    parser.add_argument(
        "--mission-type",
        choices=("point_to_point", "cooperative_traffic"),
        default="point_to_point",
    )
    parser.add_argument("--expected-vehicles", type=int, default=0)
    parser.add_argument("--expected-static", default="")
    parser.add_argument("--expected-memory", default="")
    parser.add_argument(
        "--lidar-profile", choices=("none", "3d"), default="3d"
    )
    parser.add_argument(
        "--require-observed-3d-route-volume-crossing", action="store_true"
    )
    parser.add_argument(
        "--observed-3d-route-volume-bounds-m",
        type=parse_route_volume_bounds,
    )
    parser.add_argument("--runtime-manifest", type=Path)
    parser.add_argument(
        "--require-persistent-3d-acceptance", action="store_true"
    )
    parser.add_argument("--enable-lidar-debug", default="true")
    parser.add_argument("--mission-check", action="store_true")
    parser.add_argument("--allow-mission-failure", action="store_true")
    args = parser.parse_args()
    if (
        args.require_observed_3d_route_volume_crossing
        and args.observed_3d_route_volume_bounds_m is None
    ):
        parser.error(
            "--require-observed-3d-route-volume-crossing requires "
            "--observed-3d-route-volume-bounds-m"
        )
    if args.require_persistent_3d_acceptance and args.runtime_manifest is None:
        parser.error(
            "--require-persistent-3d-acceptance requires --runtime-manifest"
        )

    ros_log = read_text(args.ros_log)
    px4_logs = [read_text(path) for path in args.px4_log]
    px4_log = "\n".join(px4_logs)
    expected_static = parse_bool(args.expected_static)
    expected_memory = parse_bool(args.expected_memory)
    enable_lidar_debug = parse_bool(args.enable_lidar_debug) is not False
    errors: list[str] = []
    validate_building_collisions(ros_log, errors)

    expected_vehicles = args.expected_vehicles
    if expected_vehicles <= 0:
        expected_vehicles = (
            4 if args.mission_type == "cooperative_traffic" else 1
        )
    require_count(
        "PX4 instances report Gazebo ready",
        px4_log,
        r"Gazebo world is ready",
        expected_vehicles,
        errors,
    )
    validate_mapping_pipeline(
        ros_log,
        args.lidar_profile,
        expected_memory,
        enable_lidar_debug,
        errors,
    )
    require(
        "production MPPI is ready",
        ros_log,
        r"Production MPPI ready:",
        errors,
    )
    require(
        "ESDF is available",
        ros_log,
        r"PRODUCTION_MPPI_ESDF(?:3D(?:_ONLINE)?)? .*revision=",
        errors,
    )
    validate_execution_chain(ros_log, errors)
    if args.runtime_manifest is not None:
        validate_runtime_manifest(
            args.runtime_manifest,
            args.observed_3d_route_volume_bounds_m,
            args.require_persistent_3d_acceptance,
            errors,
            expected_mission_type=args.mission_type,
            expected_lidar_profile=args.lidar_profile,
            expected_static_map=expected_static,
        )
    if args.require_persistent_3d_acceptance:
        validate_persistent_3d_acceptance_metrics(ros_log, errors)
        manifest_overrides = json.loads(
            args.runtime_manifest.read_text(encoding="utf-8")
        ).get("effective_overrides", {})
        validate_mean_flight_speed(
            ros_log, errors,
            manifest_overrides.get("NAVIGATION_SENSOR_PROFILE", "lidar"),
        )
        validate_controller_dynamics(args.runtime_manifest.parent, ros_log, errors)
    if args.runtime_manifest is not None:
        validate_resource_budget(args.runtime_manifest.parent, ros_log, errors)
        validate_localization_profile(args.runtime_manifest, ros_log, px4_log, errors)
    require(
        "production offboard is ready",
        ros_log,
        r"Production MPPI offboard ready:",
        errors,
    )
    require_count(
        "vehicles are armed by production offboard",
        px4_log,
        r"Armed by external command",
        expected_vehicles,
        errors,
    )
    require_count(
        "vehicles take off under production MPPI",
        px4_log,
        r"Takeoff detected",
        expected_vehicles,
        errors,
    )

    if expected_static is True:
        require(
            "static map contributes to raw occupancy",
            ros_log,
            r"STATIC_WORLD_3D .*\.occupancy3d|"
            r"Published static world visualization:|use_static_map=true",
            errors,
        )
    elif expected_static is False and re.search(r"use_static_map=true", ros_log):
        errors.append("FAIL: static map is disabled")
    else:
        print("OK: static map source contract")

    if expected_static is False:
        if re.search(
            r"STATIC_WORLD_3D|STATIC_ESDF3D_READY|STATIC_ESDF_CACHE_READY",
            ros_log,
        ):
            errors.append("FAIL: no static occupancy, ESDF, or topology is loaded")
        else:
            print("OK: no static occupancy, ESDF, or topology is loaded")

    if args.require_observed_3d_route_volume_crossing:
        if args.lidar_profile != "3d" or expected_static is not False:
            errors.append(
                "FAIL: observed 3D route validation requires no-static 3D lidar"
            )
        else:
            validate_observed_3d_route_volume(
                ros_log,
                args.observed_3d_route_volume_bounds_m,
                errors,
            )
    if re.search(r"CRASH_EVENT|cause=physical_collision", ros_log):
        errors.append("FAIL: crash was reported")
    else:
        print("OK: no crash was reported")

    mission_failed = re.search(r"MISSION_RESULT success=false", ros_log) is not None
    if args.mission_check and not args.allow_mission_failure:
        require(
            "mission monitor verifies complete flight",
            ros_log,
            r"MISSION_RESULT success=true",
            errors,
        )
        if args.mission_type == "cooperative_traffic":
            validate_cooperative_traffic(
                ros_log,
                expected_vehicles,
                expected_memory,
                errors,
                args.lidar_profile,
            )
        else:
            validate_point_to_point_waypoints(ros_log, errors)
    elif mission_failed:
        print("WARN: mission failure was allowed")

    if CRITICAL_PX4_PATTERN.search(px4_log):
        errors.append("FAIL: PX4 log contains critical simulator errors")
    else:
        print("OK: no critical PX4 simulator errors found")

    for error in errors:
        print(error, file=sys.stderr)
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
