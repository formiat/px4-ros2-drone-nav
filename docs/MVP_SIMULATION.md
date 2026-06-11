# Drone City Navigation MVP

This repository contains a first ROS 2/PX4/Gazebo MVP for single-drone urban
flight at a fixed altitude.

## Scope

- Gazebo provides a generated city block world with static building obstacles.
- PX4 SITL provides stabilization and accepts offboard trajectory setpoints.
- ROS 2 runs a lidar-driven planner and a PX4 offboard control node.
- The planner starts without a prior map. It incrementally marks a 2D occupancy
  grid from `sensor_msgs/LaserScan`, inflates occupied cells, runs A*, smooths
  the result with line-of-sight checks, and republishes a waypoint path.
- If the planner cannot find a path after a new obstacle update, it publishes an
  empty path; the offboard node then holds the current local position.

This is simulation code only. It is not a certified collision-avoidance system
and must not be used on real hardware.

## Frames And Units

- Planner grid frame: local horizontal `map`, meters.
- PX4 local position: NED, using `VehicleLocalPosition.x` as north/meters and
  `VehicleLocalPosition.y` as east/meters.
- Published `nav_msgs/Path` uses positive altitude for visualization.
- PX4 `TrajectorySetpoint` uses NED altitude, so the offboard node sends
  `z = -cruise_altitude_m`.
- Lidar rays are interpreted in the local horizontal frame with a configurable
  `scan_yaw_offset_rad`.

## Main Files

- `drone_city_nav/worlds/generated_city.sdf` - generated static city world.
- `drone_city_nav/src/planner_node.cpp` - lidar mapping and replanning node.
- `drone_city_nav/src/px4_offboard_node.cpp` - PX4 offboard waypoint follower.
- `drone_city_nav/config/urban_mvp.yaml` - default MVP parameters.
- `drone_city_nav/tests/planner_core_test.cpp` - deterministic algorithm tests.

## Quick Start

Build the dev image once from the host:

```bash
./scripts/build_dev_image.sh
```

Enter the container:

```bash
./scripts/dev_shell.sh
```

Inside the container, clone PX4-Autopilot into the ignored `external/` folder:

```bash
./scripts/setup_px4_autopilot.sh
export PX4_AUTOPILOT_DIR=/workspace/external/PX4-Autopilot
```

Run the MVP stack:

```bash
./scripts/run_city_mvp.sh
```

If Gazebo GUI cannot open from Docker, allow local X11 access on the host before
starting the dev shell:

```bash
xhost +local:docker
```

## Development Checks

Inside the dev container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_msgs_ws/install/setup.bash
colcon build --packages-select drone_city_nav --symlink-install
colcon test --packages-select drone_city_nav
colcon test-result --verbose
```

## Current Limitations

- The generated city is intentionally small and synthetic.
- Only static building obstacles are modeled.
- The planner treats unknown grid cells as traversable and replans as lidar data
  arrives.
- The offboard node assumes the planner path and PX4 local position share the
  same horizontal origin.
- The launch file bridges `/scan`; if the PX4 lidar model publishes a different
  Gazebo topic, update `city_nav.launch.py` or add a remap.
