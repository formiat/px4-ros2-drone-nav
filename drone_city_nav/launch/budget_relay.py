"""The budget relay's wiring (roadmap item 36, specification K53): with `budget_relay:=true` the consumers of the
onboard loop read the `/budgeted` topics that `budget_relay_node` publishes, and the estimators publish their
odometry to the relay instead of the autopilot; no node learns anything else. Off, every topic is as it was."""
from __future__ import annotations

BUDGETED = "/budgeted"
UNBUDGETED_ODOMETRY = "/drone_city_nav/vehicle_visual_odometry/unbudgeted"
PLANNER_INPUTS = (
    ("raw_obstacle_delta_3d_topic", "/drone_city_nav/raw_obstacle_delta_3d"),
    ("raw_obstacle_snapshot_3d_topic", "/drone_city_nav/raw_obstacle_snapshot_3d"),
    ("latest_sensor_obstacle_scan_topic", "/drone_city_nav/latest_sensor_obstacle_scan"),
)
HORIZON = "/drone_city_nav/mppi/execution_horizon"
DEPTH_POINTS = "/stereo_depth/points"


def apply_budget_relay(
    enabled: str,
    navigation_sensors: str,
    production_mppi_parameters: list,
    offboard_overrides: dict,
    visual_inertial_overrides: dict | None,
    lidar_inertial_overrides: dict | None,
    obstacle_memory_parameters: list,
) -> list[str]:
    """Rewires the loop's consumers; returns the ROS arguments of the process that hosts the camera estimator
    (a node-specific remap: the depth node beside it keeps publishing the unbudgeted returns)."""
    if enabled.strip().lower() not in ("true", "1", "on"):
        return []
    production_mppi_parameters.append({name: topic + BUDGETED for name, topic in PLANNER_INPUTS})
    offboard_overrides["mppi_execution_horizon_topic"] = HORIZON + BUDGETED
    for overrides in (visual_inertial_overrides, lidar_inertial_overrides):
        if overrides is not None:
            overrides["px4_visual_odometry_topic"] = UNBUDGETED_ODOMETRY
    if navigation_sensors != "stereo_tof":
        return []
    obstacle_memory_parameters.append({"lidar_3d_topic": DEPTH_POINTS + BUDGETED})
    return ["--ros-args", "-r", f"visual_inertial_odometry_node:{DEPTH_POINTS}:={DEPTH_POINTS}{BUDGETED}"]
