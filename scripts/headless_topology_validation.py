#!/usr/bin/env python3
"""Validate incremental 3D topology evidence in mission logs."""

from __future__ import annotations

import argparse
import math
import re


FLOAT_PATTERN = r"[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?"
RouteVolumeBounds = tuple[float, float, float, float, float, float]


def require_pattern(
    label: str, text: str, pattern: str, errors: list[str]
) -> None:
    if re.search(pattern, text):
        print(f"OK: {label}")
    else:
        errors.append(f"FAIL: {label}")


def parse_route_volume_bounds(value: str) -> RouteVolumeBounds:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "route volume must contain min_x,min_y,min_z,max_x,max_y,max_z"
        )
    try:
        values = tuple(float(part) for part in parts)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "route volume bounds must be numeric"
        ) from error
    if not all(math.isfinite(bound) for bound in values):
        raise argparse.ArgumentTypeError("route volume bounds must be finite")
    if any(values[axis] >= values[axis + 3] for axis in range(3)):
        raise argparse.ArgumentTypeError(
            "route volume minimum bounds must be below maximum bounds"
        )
    return (
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
    )


def validate_observed_3d_route_volume(
    ros_log: str,
    bounds: RouteVolumeBounds,
    errors: list[str],
) -> None:
    require_pattern(
        "an observed-occupancy generic 3D route is activated",
        ros_log,
        r"PRODUCTION_MPPI_GUIDE3D .*activated=true .*"
        r"route_space=observed_occupancy_3d .*"
        r"topology_acceleration=incremental_topological_graph",
        errors,
    )
    require_pattern(
        "an incremental topological route is committed after raw-safe activation",
        ros_log,
        r"INCREMENTAL_TOPOLOGICAL_PLAN3D .*directive_available=true .*"
        r"activated=true .*commit_accepted=true",
        errors,
    )
    if re.search(r"ONLINE_FREE_SPACE_TOPOLOGY3D", ros_log):
        errors.append("FAIL: no online free-space partition is used")
    else:
        print("OK: no online free-space partition is used")

    logger_pattern = re.compile(r"\[([^\]]*production_mppi_node)\]:")
    position_pattern = re.compile(
        rf"PRODUCTION_MPPI_TICK .*?state_position=\("
        rf"({FLOAT_PATTERN}),({FLOAT_PATTERN}),({FLOAT_PATTERN})\)"
    )
    positions_by_logger: dict[str, list[tuple[float, float, float]]] = {}
    for line in ros_log.splitlines():
        position_match = position_pattern.search(line)
        if position_match is None:
            continue
        logger_match = logger_pattern.search(line)
        logger = logger_match.group(1) if logger_match is not None else "unscoped"
        positions_by_logger.setdefault(logger, []).append(
            tuple(float(value) for value in position_match.groups())
        )
    if not any(len(positions) >= 3 for positions in positions_by_logger.values()):
        errors.append("FAIL: enough vehicle state samples exist for route validation")
        return

    minimum = bounds[:3]
    maximum = bounds[3:]
    extents = tuple(maximum[axis] - minimum[axis] for axis in range(3))
    dominant_axis = max(range(3), key=extents.__getitem__)

    def inside(position: tuple[float, float, float]) -> bool:
        return all(
            minimum[axis] <= position[axis] <= maximum[axis]
            for axis in range(3)
        )

    for logger, positions in positions_by_logger.items():
        inside_flags = [inside(position) for position in positions]
        block_start = 0
        while block_start < len(inside_flags):
            if not inside_flags[block_start]:
                block_start += 1
                continue
            block_end = block_start
            while (
                block_end + 1 < len(inside_flags) and inside_flags[block_end + 1]
            ):
                block_end += 1
            sample_count = block_end - block_start + 1
            if (
                block_start > 0
                and block_end + 1 < len(positions)
                and sample_count >= 2
            ):
                before = positions[block_start - 1][dominant_axis]
                after = positions[block_end + 1][dominant_axis]
                low_to_high = (
                    before < minimum[dominant_axis]
                    and after > maximum[dominant_axis]
                )
                high_to_low = (
                    before > maximum[dominant_axis]
                    and after < minimum[dominant_axis]
                )
                if low_to_high or high_to_low:
                    direction = "low_to_high" if low_to_high else "high_to_low"
                    print(
                        "OK: vehicle physically crosses the observed 3D route volume "
                        f"(logger={logger}, axis={'xyz'[dominant_axis]}, "
                        f"direction={direction}, inside_samples={sample_count})"
                    )
                    return
            block_start = block_end + 1

    errors.append("FAIL: vehicle physically crosses the observed 3D route volume")


def validate_incremental_topology_evidence(
    ros_log: str,
    maximum_no_executable_route_age_ms: float,
    errors: list[str],
) -> None:
    update_pattern = re.compile(
        rf"INCREMENTAL_TOPOLOGY3D_UPDATE .*?full_reset=(true|false) "
        rf".*?refined_blocks=([0-9]+) .*?base_resolution_m=({FLOAT_PATTERN}) "
        rf"coarse_resolution_m=({FLOAT_PATTERN}) "
        rf"refined_resolution_m=({FLOAT_PATTERN}) .*?"
        rf"retained_nodes=([0-9]+) .*?nodes=([0-9]+) edges=([0-9]+)"
    )
    updates = [
        (
            full_reset == "true",
            int(refined_blocks),
            float(base_resolution_m),
            float(coarse_resolution_m),
            float(refined_resolution_m),
            int(retained_nodes),
            int(nodes),
            int(edges),
        )
        for (
            full_reset,
            refined_blocks,
            base_resolution_m,
            coarse_resolution_m,
            refined_resolution_m,
            retained_nodes,
            nodes,
            edges,
        ) in update_pattern.findall(ros_log)
    ]
    adaptive_updates = [
        update
        for update in updates
        if update[1] > 0
        and update[2] > 0.0
        and math.isclose(update[2], update[4], rel_tol=0.0, abs_tol=1.0e-9)
        and update[3] > update[4]
        and update[6] > 0
        and update[7] > 0
    ]
    if adaptive_updates:
        base_resolution_m = min(update[2] for update in adaptive_updates)
        coarse_resolution_m = max(update[3] for update in adaptive_updates)
        refined_resolution_m = min(update[4] for update in adaptive_updates)
        print(
            "OK: incremental topology uses adaptive spatial resolution "
            f"(base={base_resolution_m:.3f} m, "
            f"coarse={coarse_resolution_m:.3f} m, "
            f"refined={refined_resolution_m:.3f} m)"
        )
    else:
        errors.append("FAIL: incremental topology uses adaptive spatial resolution")

    stable_updates = [
        update for update in updates if not update[0] and update[5] > 0
    ]
    if stable_updates:
        print(
            "OK: dirty topology updates retain stable node identities "
            f"({max(update[5] for update in stable_updates)} retained nodes)"
        )
    else:
        errors.append("FAIL: dirty topology updates retain stable node identities")

    require_pattern(
        "topological planning commits a reachable strategic route",
        ros_log,
        r"INCREMENTAL_TOPOLOGICAL_PLAN3D "
        r".*status=(?:mission_route|mission_continuation_route|frontier_route) "
        r".*route_edges=[1-9][0-9]* "
        r".*activated=true .*commit_accepted=true",
        errors,
    )
    require_pattern(
        "topological route logs typed source-edge history",
        ros_log,
        r"INCREMENTAL_TOPOLOGICAL_SOURCE_EDGE3D .*edge_id=[1-9][0-9]* "
        r"from=[1-9][0-9]* to=[1-9][0-9]*",
        errors,
    )
    require_pattern(
        "vehicle traversal advances through the incremental graph",
        ros_log,
        r"INCREMENTAL_TOPOLOGY3D_OBSERVATION .*traversed_edges=[1-9][0-9]* "
        r"coverage_cells=[1-9][0-9]*",
        errors,
    )

    route_ages_ms = [
        float(value)
        for value in re.findall(
            rf"INCREMENTAL_TOPOLOGICAL_PLAN3D .*?"
            rf"no_executable_route_age_ms=({FLOAT_PATTERN})",
            ros_log,
        )
    ]
    if not route_ages_ms:
        errors.append("FAIL: topology logs time without an executable route")
    elif max(route_ages_ms) > maximum_no_executable_route_age_ms:
        errors.append(
            "FAIL: incremental topology route continuity "
            f"({max(route_ages_ms):.2f} ms > "
            f"{maximum_no_executable_route_age_ms:.2f} ms)"
        )
    else:
        print(
            "OK: incremental topology route continuity "
            f"(maximum gap {max(route_ages_ms):.2f} ms)"
        )

    clearance_tokens = re.findall(
        rf"lattice_3d_minimum_clearance_m=({FLOAT_PATTERN}|inf|nan)",
        ros_log,
        flags=re.IGNORECASE,
    )
    clearances_m = [float(value) for value in clearance_tokens]
    if not clearances_m or any(
        math.isnan(value) or value < 0.0 for value in clearances_m
    ):
        errors.append("FAIL: incremental 3D routes report valid clearance samples")
    else:
        finite_clearances_m = [
            value for value in clearances_m if math.isfinite(value)
        ]
        minimum_clearance = (
            min(finite_clearances_m) if finite_clearances_m else math.inf
        )
        print(
            "OK: incremental 3D routes report valid clearance samples "
            f"(minimum {minimum_clearance:.3f} m)"
        )
