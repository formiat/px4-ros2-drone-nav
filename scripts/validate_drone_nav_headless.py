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
    if mission_type not in {"intercept", "multi_intercept"}:
        return ros_log
    terminal_result = re.search(
        rf"MISSION_RESULT success=true mission={mission_type} outcome=[a-z_]+.*",
        ros_log,
    )
    return ros_log[: terminal_result.end()] if terminal_result else ros_log


def validate_cooperative_traffic(
    ros_log: str,
    expected_vehicles: int,
    expected_memory: bool | None,
    errors: list[str],
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
        require(
            "cooperative peer memory filtering is active without weakening latest lidar safety",
            ros_log,
            r"COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=[0-9]+ "
            r"matched_peers=[0-9]+ known_peers=[1-9][0-9]* "
            r"latest_safety_excluded=false",
            errors,
        )


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
        validate_physical_proximity_intercept(settled_log, errors)
        capturing = re.search(r"capturing_interceptor_id='(interceptor_[0-9]+)'", result.group(0))
        if capturing is None:
            errors.append("FAIL: intercept result identifies the capturing interceptor")
            return
        capturing_id = capturing.group(1)
        require(
            f"intercept disarm is confirmed for {capturing_id}",
            ros_log,
            rf"\[vehicles\.{capturing_id}\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=interceptor "
            r"cause=proximity_intercept .*detail='intercepted'",
            errors,
        )
        require(
            "intercept disarm is confirmed for evader",
            ros_log,
            r"\[vehicles\.evader\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=evader "
            r"cause=proximity_intercept .*detail='intercepted'",
            errors,
        )
        outcome_record = re.search(
            r"INTERCEPT_OUTCOME outcome=intercepted .*live_interceptors=([0-9]+)",
            settled_log,
        )
        expected_holds = (
            max(0, int(outcome_record.group(1)) - 1) if outcome_record else 0
        )
        if expected_holds > 0:
            validate_intercept_survivor_settlement(
                ros_log, settled_log, expected_holds, errors
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
            r"INTERCEPTOR_HOLD_ABORTED .*reason=late_capture",
            errors,
        )
        require(
            "late capture disarm is confirmed for an interceptor",
            settled_log,
            r"\[vehicles\.interceptor_[0-9]+\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=interceptor "
            r"cause=proximity_intercept "
            r".*detail='late_intercept_after_evader_goal'",
            errors,
        )
        require(
            "late capture disarm is confirmed for evader",
            settled_log,
            r"\[vehicles\.evader\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=evader "
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
            r"INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_[0-9]+' "
            r"position_error_m=.* speed_mps=",
            errors,
        )
        print("OK: evader goal settlement keeps both vehicles armed")


def validate_intercept_survivor_settlement(
    ros_log: str,
    settled_log: str,
    expected_survivors: int,
    errors: list[str],
) -> None:
    requested_ids = sorted(
        set(
            re.findall(
                r"INTERCEPTOR_HOLD requested=true "
                r"vehicle_id='(interceptor_[0-9]+)'",
                settled_log,
            )
        )
    )
    if len(requested_ids) < expected_survivors:
        errors.append(
            "FAIL: post-capture hold is requested for every surviving interceptor "
            f"({len(requested_ids)} < {expected_survivors})"
        )
        return

    for interceptor_id in requested_ids:
        if re.search(
            rf"INTERCEPTOR_HOLD_CONFIRMED "
            rf"vehicle_id='{re.escape(interceptor_id)}'",
            settled_log,
        ):
            print(f"OK: {interceptor_id} confirms post-capture hold")
            continue
        destroyed = re.search(
            r"VEHICLE_DESTROYED referee_observed=true role=interceptor "
            rf"vehicle_id='{re.escape(interceptor_id)}' "
            r"cause=proximity_collision",
            settled_log,
        )
        disarmed = re.search(
            rf"\[vehicles\.{re.escape(interceptor_id)}\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=interceptor "
            r"cause=proximity_collision",
            ros_log,
        )
        if destroyed and disarmed:
            print(
                f"OK: {interceptor_id} proximity death supersedes "
                "post-capture hold"
            )
            continue
        errors.append(
            f"FAIL: {interceptor_id} confirms post-capture hold or proximity death"
        )


def validate_physical_proximity_intercept(
    ros_log: str, errors: list[str]
) -> None:
    event = re.search(
        r"PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true .*"
        r"measured_swept_separation_m=([0-9.]+) .*"
        r"separation_threshold_m=([0-9.]+) .*"
        r"interpolation_fraction=([0-9.]+) .*"
        r"interceptor_position=\([^)]+\) evader_position=\([^)]+\)",
        ros_log,
    )
    if event is None:
        errors.append("FAIL: intercept has physical Gazebo proximity evidence")
        return
    separation_m = float(event.group(1))
    threshold_m = float(event.group(2))
    fraction = float(event.group(3))
    if separation_m > threshold_m + 1.0e-6:
        errors.append(
            "FAIL: physical intercept separation exceeds the capture threshold"
        )
    elif not 0.0 <= fraction <= 1.0:
        errors.append("FAIL: physical intercept interpolation fraction is invalid")
    else:
        print(
            "OK: physical Gazebo proximity confirms intercept "
            f"({separation_m:.3f} <= {threshold_m:.3f} m)"
        )


def validate_intercept_physical_destruction_settlement(
    ros_log: str, errors: list[str]
) -> None:
    result = re.search(
        r"MISSION_RESULT success=false mission=intercept "
        r"outcome=(evader_crashed|no_interceptors_remaining).*",
        ros_log,
    )
    if result is None:
        return
    role = "evader" if result.group(1) == "evader_crashed" else "interceptor"
    settled_log = ros_log[: result.end()]
    require(
        f"physical destruction is observed for {role}",
        settled_log,
        rf"VEHICLE_DESTROYED referee_observed=true role={role} .*"
        r"cause=physical_collision",
        errors,
    )
    require(
        f"physical destruction disarm is confirmed for {role}",
        settled_log,
        rf"\[vehicles\.{role}(?:_[0-9]+)?\.mppi_offboard_node\].*"
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
            r"INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_[0-9]+' "
            r"position_error_m=.* speed_mps=",
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
        "navigation and Gazebo physical coordinates are aligned",
        ros_log,
        r"SIMULATION_TRUTH_ALIGNMENT ready=true failure_confirmed=false "
        r"reason=aligned",
        errors,
    )
    require(
        "radar simulator publishes relative measurements",
        ros_log,
        r"RADAR_SCAN published=true .*source=gazebo_physical_truth",
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


def validate_multi_intercept_settlement(ros_log: str, errors: list[str]) -> None:
    result = re.search(
        r"MISSION_RESULT success=true mission=multi_intercept "
        r"outcome=(all_intercepted|all_reached_goal|mixed) .*"
        r"intercepted_targets=([0-9]+) reached_goal_targets=([0-9]+) "
        r"destroyed_targets=([0-9]+) target_count=([0-9]+)",
        ros_log,
    )
    if result is None:
        errors.append("FAIL: multi-intercept mission reports a technical outcome")
        return
    settled_log = ros_log[: result.end()]
    result_counts = tuple(int(result.group(index)) for index in range(2, 6))
    intercepted_count, reached_goal_count, destroyed_count, expected_targets = (
        result_counts
    )
    if sum(result_counts[:3]) != expected_targets:
        errors.append("FAIL: multi-intercept result target counts are consistent")

    target_outcomes = re.findall(
        r"INTERCEPT_TARGET_OUTCOME target_id='([^']+)' detection_id=[0-9]+ "
        r"outcome=([a-z_]+) first_target_terminal_event=true "
        r"capturing_interceptor_id='([^']+)'",
        settled_log,
    )
    outcome_ids = [target_id for target_id, _, _ in target_outcomes]
    if len(target_outcomes) != expected_targets or len(set(outcome_ids)) != len(
        outcome_ids
    ):
        errors.append(
            "FAIL: every target has exactly one terminal outcome "
            f"({len(target_outcomes)} != {expected_targets})"
        )
    else:
        print(f"OK: all target outcomes are recorded ({expected_targets})")

    startup = re.search(
        r"INTERCEPT_MISSION state=running mission='multi_intercept' .*"
        r"interceptor_count=([0-9]+) target_count=([0-9]+)",
        settled_log,
    )
    expected_interceptors = int(startup.group(1)) if startup else 0
    if startup is None or int(startup.group(2)) != expected_targets:
        errors.append("FAIL: multi-intercept startup counts match the result")
    assignment_ids = set(
        re.findall(
            r"TARGET_ASSIGNMENT interceptor_id='([^']+)' detection_id=[0-9]+",
            settled_log,
        )
    )
    if len(assignment_ids) < expected_interceptors:
        errors.append(
            "FAIL: adaptive assignments are published for all interceptors "
            f"({len(assignment_ids)} < {expected_interceptors})"
        )
    else:
        print(
            "OK: adaptive assignments are published for all interceptors "
            f"({expected_interceptors})"
        )

    captured_interceptors: set[str] = set()
    for target_id, outcome, capturing_id in target_outcomes:
        if outcome != "intercepted":
            continue
        if capturing_id == "none":
            errors.append(f"FAIL: captured target {target_id} identifies its interceptor")
            continue
        captured_interceptors.add(capturing_id)
        proximity = re.search(
            r"PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            rf"interceptor_id='{re.escape(capturing_id)}' "
            rf"target_id='{re.escape(target_id)}' .*"
            r"measured_swept_separation_m=([0-9.]+) .*"
            r"separation_threshold_m=([0-9.]+)",
            settled_log,
        )
        if proximity is None or float(proximity.group(1)) > float(proximity.group(2)):
            errors.append(
                f"FAIL: {capturing_id} physically intercepts {target_id} within threshold"
            )
        else:
            print(f"OK: {capturing_id} physically intercepts {target_id}")
        for vehicle_id, role in (
            (capturing_id, "interceptor"),
            (target_id, "evader"),
        ):
            require(
                f"multi-intercept disarm is confirmed for {vehicle_id}",
                ros_log,
                rf"\[vehicles\.{re.escape(vehicle_id)}\.mppi_offboard_node\].*"
                rf"VEHICLE_DESTROYED disarm_confirmed=true role={role} "
                r"cause=proximity_intercept",
                errors,
            )

    destroyed_interceptors = captured_interceptors | set(
        re.findall(
            r"VEHICLE_DESTROYED referee_observed=true role=interceptor "
            r"vehicle_id='([^']+)'",
            settled_log,
        )
    )
    surviving_interceptors = assignment_ids - destroyed_interceptors
    for interceptor_id in sorted(surviving_interceptors):
        require(
            f"surviving interceptor {interceptor_id} confirms hold",
            settled_log,
            rf"INTERCEPTOR_HOLD_CONFIRMED vehicle_id='{re.escape(interceptor_id)}'",
            errors,
        )

    if len([item for item in target_outcomes if item[1] == "intercepted"]) != (
        intercepted_count
    ) or len([item for item in target_outcomes if item[1] == "reached_goal"]) != (
        reached_goal_count
    ) or len([item for item in target_outcomes if item[1] == "destroyed"]) != (
        destroyed_count
    ):
        errors.append("FAIL: target outcomes match multi-intercept result counts")


def validate_intercept_physical_losses(ros_log: str, errors: list[str]) -> None:
    if "CRASH_EVENT" in ros_log:
        errors.append("FAIL: untyped legacy crash was reported")
    if re.search(
        r"VEHICLE_DESTROYED referee_observed=true role=evader .*"
        r"cause=physical_collision",
        ros_log,
    ):
        errors.append("FAIL: evader physical crash was reported")

    interceptor_ids = sorted(
        set(
            re.findall(
                r"VEHICLE_DESTROYED referee_observed=true role=interceptor "
                r"vehicle_id='(interceptor_[0-9]+)' cause=physical_collision",
                ros_log,
            )
        )
    )
    for interceptor_id in interceptor_ids:
        require(
            f"physical loss is disarm-confirmed for {interceptor_id}",
            ros_log,
            rf"\[vehicles\.{interceptor_id}\.mppi_offboard_node\].*"
            r"VEHICLE_DESTROYED disarm_confirmed=true role=interceptor "
            r"cause=physical_collision",
            errors,
        )
    unresolved = re.search(r"cause=physical_collision", ros_log) and not (
        interceptor_ids
        or re.search(
            r"VEHICLE_DESTROYED referee_observed=true role=evader .*"
            r"cause=physical_collision",
            ros_log,
        )
    )
    if unresolved:
        errors.append("FAIL: physical collision lacks typed referee settlement")
    elif interceptor_ids:
        print(f"OK: settled interceptor physical losses ({len(interceptor_ids)})")
    else:
        print("OK: no crash was reported")


def validate_building_collisions(ros_log: str, errors: list[str]) -> None:
    collisions = sorted(
        set(
            re.findall(
                r"VEHICLE_DESTROYED role=[0-9]+ vehicle_id='([^']+)' "
                r"cause=physical_collision .*?obstacle_collision='([^']*building[^']*)'",
                ros_log,
            )
        )
    )
    if not collisions:
        print("OK: no vehicle collided with a building")
        return
    for vehicle_id, obstacle in collisions:
        errors.append(
            f"FAIL: {vehicle_id} collided with building obstacle '{obstacle}'"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate production MPPI headless run logs."
    )
    parser.add_argument("--ros-log", required=True, type=Path)
    parser.add_argument("--px4-log", required=True, action="append", type=Path)
    parser.add_argument(
        "--mission-type",
        choices=(
            "point_to_point",
            "intercept",
            "multi_intercept",
            "cooperative_traffic",
        ),
        default="point_to_point",
    )
    parser.add_argument("--expected-vehicles", type=int, default=0)
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
    expected_memory = parse_bool(args.expected_memory)
    expected_current_lidar = parse_bool(args.expected_current_lidar)
    enable_lidar_debug = parse_bool(args.enable_lidar_debug) is not False
    errors: list[str] = []
    safety_ros_log = safety_relevant_ros_log(ros_log, args.mission_type)
    validate_building_collisions(ros_log, errors)

    expected_vehicles = args.expected_vehicles
    if expected_vehicles <= 0:
        expected_vehicles = 4 if args.mission_type in {
            "intercept",
            "multi_intercept",
            "cooperative_traffic",
        } else 1
    require_count(
        "PX4 instances report Gazebo ready",
        px4_log,
        r"Gazebo world is ready",
        expected_vehicles,
        errors,
    )
    if expected_memory is not False:
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
    elif re.search(r"First lidar scan|Obstacle memory update:", ros_log):
        errors.append("FAIL: obstacle memory is disabled")
    else:
        print("OK: obstacle memory is disabled")
    if expected_current_lidar is True:
        require(
            "latest lidar safety source is active",
            ros_log,
            r"LATEST_LIDAR_SAFETY_SCAN published=true",
            errors,
        )
    elif expected_current_lidar is False and re.search(
        r"LATEST_LIDAR_SAFETY_SCAN published=true", ros_log
    ):
        errors.append("FAIL: latest lidar safety source is disabled")
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

    if args.mission_type in {"intercept", "multi_intercept"}:
        validate_intercept_physical_losses(safety_ros_log, errors)
    elif re.search(r"CRASH_EVENT|cause=physical_collision", safety_ros_log):
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
        if args.mission_type in {"intercept", "multi_intercept"}:
            validate_intercept_radar_pipeline(ros_log, errors)
        if args.mission_type == "intercept":
            require(
                "intercept mission reports a technical outcome",
                ros_log,
                r"MISSION_RESULT success=true mission=intercept "
                r"outcome=(?:intercepted|evader_reached_goal)",
                errors,
            )
            validate_intercept_settlement(ros_log, errors)
        elif args.mission_type == "multi_intercept":
            validate_multi_intercept_settlement(ros_log, errors)
        elif args.mission_type == "cooperative_traffic":
            validate_cooperative_traffic(
                ros_log, expected_vehicles, expected_memory, errors
            )
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
