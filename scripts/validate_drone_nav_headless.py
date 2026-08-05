#!/usr/bin/env python3
"""Validate the production MPPI headless navigation contract."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


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


def safety_relevant_ros_log(ros_log: str, mission_type: str) -> str:
    if mission_type != "intercept":
        return ros_log
    terminal_result = re.search(
        r"MISSION_RESULT success=true mission=intercept "
        r"outcome=(?:intercepted|evader_reached_goal).*",
        ros_log,
    )
    return ros_log[: terminal_result.end()] if terminal_result else ros_log


def validate_intercept_settlement(ros_log: str, errors: list[str]) -> None:
    result = re.search(
        r"MISSION_RESULT success=true mission=intercept "
        r"outcome=(intercepted|evader_reached_goal).*",
        ros_log,
    )
    if result is None:
        validate_intercept_physical_destruction_settlement(ros_log, errors)
        return
    outcome = result.group(1)
    settled_log = ros_log[: result.end()]
    require(
        "intercept first terminal event matches the result",
        settled_log,
        rf"INTERCEPT_OUTCOME outcome={outcome} first_terminal_event=true",
        errors,
    )
    if outcome == "intercepted":
        for role in ("interceptor", "evader"):
            require(
                f"intercept disarm is confirmed for {role}",
                ros_log,
                rf"\[vehicles\.{role}\.mppi_offboard_node\].*"
                rf"VEHICLE_DESTROYED disarm_confirmed=true role={role} "
                r"cause=proximity_intercept .*detail='intercepted'",
                errors,
            )
        return

    require(
        "interceptor hold is requested after evader goal arrival",
        settled_log,
        r"INTERCEPTOR_HOLD requested=true",
        errors,
    )
    if "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal" in settled_log:
        require(
            "late capture aborts interceptor hold without changing the outcome",
            settled_log,
            r"INTERCEPTOR_HOLD_ABORTED reason=late_capture",
            errors,
        )
        for role in ("interceptor", "evader"):
            require(
                f"late capture disarm is confirmed for {role}",
                settled_log,
                rf"\[vehicles\.{role}\.mppi_offboard_node\].*"
                rf"VEHICLE_DESTROYED disarm_confirmed=true role={role} "
                r"cause=proximity_intercept "
                r".*detail='late_intercept_after_evader_goal'",
                errors,
            )
    elif re.search(r"VEHICLE_DESTROYED .*cause=proximity_intercept", settled_log):
        errors.append(
            "FAIL: unreported late proximity intercept changed goal settlement"
        )
    else:
        require(
            "interceptor hold is physically confirmed",
            settled_log,
            r"INTERCEPTOR_HOLD_CONFIRMED position_error_m=.* speed_mps=",
            errors,
        )
        print("OK: evader goal settlement keeps both vehicles armed")


def validate_intercept_physical_destruction_settlement(
    ros_log: str, errors: list[str]
) -> None:
    result = re.search(
        r"MISSION_RESULT success=false mission=intercept .*"
        r"reason='physical_collision_(interceptor|evader)' .*",
        ros_log,
    )
    if result is None:
        return
    role = result.group(1)
    settled_log = ros_log[: result.end()]
    require(
        f"physical destruction is observed for {role}",
        settled_log,
        rf"VEHICLE_DESTROYED referee_observed=true role={role} "
        r"cause=physical_collision",
        errors,
    )
    require(
        f"physical destruction disarm is confirmed for {role}",
        settled_log,
        rf"\[vehicles\.{role}\.mppi_offboard_node\].*"
        rf"VEHICLE_DESTROYED disarm_confirmed=true role={role} "
        r"cause=physical_collision",
        errors,
    )
    if role == "evader":
        require(
            "interceptor hold is requested after evader destruction",
            settled_log,
            r"INTERCEPTOR_HOLD requested=true",
            errors,
        )
        require(
            "interceptor hold is confirmed after evader destruction",
            settled_log,
            r"INTERCEPTOR_HOLD_CONFIRMED position_error_m=.* speed_mps=",
            errors,
        )


def validate_intercept_radar_pipeline(ros_log: str, errors: list[str]) -> None:
    require(
        "evader ground truth is restricted to the referee and radar simulator",
        ros_log,
        r"RADAR_DATA_BOUNDARY verified=true",
        errors,
    )
    require(
        "radar simulator publishes relative measurements",
        ros_log,
        r"RADAR_SCAN published=true .*source=ideal_truth_adapter",
        errors,
    )
    require(
        "radar tracker reaches velocity-tracking state",
        ros_log,
        r"RADAR_TRACK status=tracking .*velocity_valid=true",
        errors,
    )
    require(
        "interceptor guidance consumes radar-derived tracks",
        ros_log,
        r"INTERCEPT_GUIDANCE source=radar_track",
        errors,
    )
    if "ground_truth_boundary_violation" in ros_log:
        errors.append("FAIL: interceptor data path accessed evader ground truth")
    else:
        print("OK: no evader ground-truth boundary violation")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate production MPPI headless run logs."
    )
    parser.add_argument("--ros-log", required=True, type=Path)
    parser.add_argument("--px4-log", required=True, action="append", type=Path)
    parser.add_argument(
        "--mission-type",
        choices=("point_to_point", "intercept"),
        default="point_to_point",
    )
    parser.add_argument("--expected-static", default="")
    parser.add_argument("--expected-memory", default="")
    parser.add_argument("--expected-current-lidar", default="")
    parser.add_argument("--enable-lidar-debug", default="true")
    parser.add_argument("--mission-check", action="store_true")
    parser.add_argument("--allow-mission-failure", action="store_true")
    args = parser.parse_args()

    ros_log = read_text(args.ros_log)
    px4_logs = [read_text(path) for path in args.px4_log]
    px4_log = "\n".join(px4_logs)
    expected_static = parse_bool(args.expected_static)
    enable_lidar_debug = parse_bool(args.enable_lidar_debug) is not False
    errors: list[str] = []
    safety_ros_log = safety_relevant_ros_log(ros_log, args.mission_type)

    expected_vehicles = 2 if args.mission_type == "intercept" else 1
    require_count(
        "PX4 instances report Gazebo ready",
        px4_log,
        r"Gazebo world is ready",
        expected_vehicles,
        errors,
    )
    require(
        "obstacle memory receives lidar",
        ros_log,
        r"First lidar scan|Obstacle memory update:",
        errors,
    )
    require(
        "raw obstacle snapshots are published",
        ros_log,
        r"Raw obstacle snapshot|raw obstacle snapshot|raw_revision=",
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
        r"PRODUCTION_MPPI_ESDF(?:3D)? .*revision=",
        errors,
    )
    require(
        "global guide is available",
        ros_log,
        r"PRODUCTION_MPPI_GUIDE .*guide_valid=true|target_source=global_route_3d",
        errors,
    )
    require(
        "production MPPI publishes collision-free horizons",
        ros_log,
        r"PRODUCTION_MPPI_TICK .*raw_collision=false "
        r".*known_solid_collision=false",
        errors,
    )
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

    if enable_lidar_debug:
        require(
            "lidar debug snapshots are written",
            ros_log,
            r"LIDAR_DEBUG snapshot=",
            errors,
        )

    if re.search(r"CRASH_EVENT|cause=physical_collision", safety_ros_log):
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
        if args.mission_type == "intercept":
            validate_intercept_radar_pipeline(ros_log, errors)
            require(
                "intercept mission reports a technical outcome",
                ros_log,
                r"MISSION_RESULT success=true mission=intercept "
                r"outcome=(?:intercepted|evader_reached_goal)",
                errors,
            )
            validate_intercept_settlement(ros_log, errors)
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
