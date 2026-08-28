#!/usr/bin/env python3
"""Validate physical 3D route-volume evidence in mission logs."""

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
        "a raw-safe persistent 3D route is certified",
        ros_log,
        r"PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite .*"
        r"certified_pending=true .*validation=valid",
        errors,
    )
    if re.search(r"INCREMENTAL_TOPOLOG|ONLINE_FREE_SPACE_TOPOLOGY3D", ros_log):
        errors.append("FAIL: no online topology route producer is used")
    else:
        print("OK: no online topology route producer is used")

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
