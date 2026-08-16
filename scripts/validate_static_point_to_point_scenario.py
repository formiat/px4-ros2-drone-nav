#!/usr/bin/env python3
"""Validate one static point-to-point route against an Occupancy3D artifact."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_PLANNER_CONFIG = REPOSITORY / "drone_city_nav/config/urban_mvp.yaml"
sys.path.insert(0, str(REPOSITORY / "drone_city_nav/launch"))

from point_to_point_scenario import load_point_to_point_scenario  # noqa: E402
from validate_static_cooperative_scenario import (  # noqa: E402
    LaunchPlatform,
    Occupancy3D,
    ScenarioValidationError,
    load_footprints,
    planar_segment_is_clear,
    platform_supports_spawn,
    shortest_planar_route_m,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--planner-config", type=Path, default=DEFAULT_PLANNER_CONFIG)
    parser.add_argument("--static-route-tracking-margin-m", type=float)
    parser.add_argument("--minimum-route-length-m", type=float, default=60.0)
    parser.add_argument(
        "--route-contract",
        choices=("direct", "connected"),
        default="connected",
        help="Require a direct segment or only a route-safe connected component.",
    )
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    scenario = load_point_to_point_scenario(args.scenario)
    occupancy = Occupancy3D.load(args.occupancy.resolve())
    physical_footprint, route_footprint = load_footprints(
        args.planner_config.resolve(), args.static_route_tracking_margin_m
    )
    start = scenario["map_start_m"]
    goal = scenario["goal_m"]
    initial_altitude_m = scenario["initial_altitude_m"]
    takeoff = (start[0], start[1], initial_altitude_m)
    launch_platforms = scenario["launch_platforms"]
    platform = None
    if launch_platforms:
        source = launch_platforms[0]
        size = source["size_m"]
        platform = LaunchPlatform(
            id=source["id"],
            vehicle_ids=("point_to_point_vehicle",),
            center_x_m=start[0],
            center_y_m=start[1],
            size_x_m=size[0],
            size_y_m=size[1],
            bottom_z_m=source["top_z_m"] - size[2],
            top_z_m=source["top_z_m"],
        )

    if args.minimum_route_length_m <= 0.0:
        raise ScenarioValidationError("minimum route length must be positive")
    if not occupancy.center_is_clear(start, physical_footprint):
        raise ScenarioValidationError(
            "physical spawn footprint intersects Occupancy3D"
        )
    if platform is None:
        if not occupancy.spawn_has_support(start, physical_footprint):
            raise ScenarioValidationError("physical spawn has no support surface")
    else:
        if not platform_supports_spawn(platform, start, physical_footprint):
            raise ScenarioValidationError(
                "physical spawn is not supported by its launch platform"
            )
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
        print(
            "STATIC_POINT_TO_POINT_LAUNCH_PLATFORM"
            f" id={platform.id} center=({platform.center_x_m:.3f},"
            f"{platform.center_y_m:.3f}) top_z_m={platform.top_z_m:.3f}"
            " status=valid"
        )
    if not occupancy.vertical_sweep_is_clear(
        start, initial_altitude_m, physical_footprint
    ):
        raise ScenarioValidationError(
            "vertical takeoff footprint intersects Occupancy3D"
        )
    if not math.isclose(goal[2], initial_altitude_m, abs_tol=1.0e-6):
        raise ScenarioValidationError("goal Z does not match initial altitude")

    route_length_m = shortest_planar_route_m(
        occupancy, takeoff, goal, route_footprint
    )
    if route_length_m is None:
        raise ScenarioValidationError(
            "start and goal are not in one route-safe component"
        )
    if route_length_m < args.minimum_route_length_m:
        raise ScenarioValidationError(
            f"route is too short: {route_length_m:.1f} m"
        )
    direct_clear = planar_segment_is_clear(
        occupancy, takeoff, goal, route_footprint
    )
    if args.route_contract == "direct" and not direct_clear:
        raise ScenarioValidationError("no direct swept-footprint route")
    print(
        "STATIC_POINT_TO_POINT_ROUTE"
        f" start=({start[0]:.3f},{start[1]:.3f},{start[2]:.3f})"
        f" goal=({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f})"
        f" direct_m={math.dist(takeoff[:2], goal[:2]):.1f}"
        f" geodesic_m={route_length_m:.1f}"
        f" altitude_m={initial_altitude_m:.1f}"
        f" route_contract={args.route_contract}"
        f" direct_clear={str(direct_clear).lower()} status=valid"
    )


def main() -> int:
    args = parse_args()
    try:
        validate(args)
    except (OSError, ValueError, ScenarioValidationError) as error:
        print(f"STATIC_POINT_TO_POINT_VALIDATION status=failed reason={error}")
        return 1
    print("STATIC_POINT_TO_POINT_VALIDATION status=passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
