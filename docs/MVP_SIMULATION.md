# Drone City Navigation MVP

This repository contains a first ROS 2/PX4/Gazebo MVP for single-drone urban
flight at a fixed altitude.

## Scope

- Gazebo provides a generated city block world with 28 static buildings. Half
  of them are above the 12 m cruise altitude and half are below it.
- PX4 SITL provides stabilization and accepts offboard trajectory setpoints.
- ROS 2 runs a lidar-driven planner and a PX4 offboard control node.
- The planner starts without a prior map. It incrementally marks a 2D occupancy
  grid from `sensor_msgs/LaserScan`, inflates occupied cells, runs A*, smooths
  the result with line-of-sight checks, and republishes a waypoint path.
- If the planner cannot find a path after a new obstacle update, fallback
  behavior is parameterized. The simulation MVP enables direct-goal fallback to
  keep the synthetic scenario moving; the real-drone template disables it.

This repository is still an MVP, not a certified collision-avoidance system. The
planner and PX4 offboard nodes are kept independent from Gazebo, but any real
hardware use needs a separate safety review, real sensor calibration, geofence,
RC override, failsafe behavior, and staged tethered/low-risk tests.

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

- `drone_city_nav/worlds/generated_city.sdf` - generated static city world with
  visual point A at `(-75, -45)` and visual point B at `(75, 45)`.
- `drone_city_nav/src/planner_node.cpp` - lidar mapping and replanning node.
- `drone_city_nav/src/px4_offboard_node.cpp` - PX4 offboard waypoint follower.
- `drone_city_nav/src/mission_monitor_node.cpp` - simulation-only mission
  verification node for headless runs.
- `drone_city_nav/config/urban_mvp.yaml` - default MVP parameters.
- `drone_city_nav/config/real_drone_template.yaml` - conservative template for
  running the planner/offboard nodes without Gazebo-specific helpers.
- `drone_city_nav/tests/planner_core_test.cpp` - deterministic algorithm tests.

## Runtime Profiles

The core runtime nodes are `planner_node` and `px4_offboard_node`. They consume
ROS/PX4 topics and do not depend on Gazebo APIs.

Simulation launch starts two extra helpers:

- `ros_gz_bridge` for the Gazebo lidar topic.
- `mission_monitor_node` for headless test assertions against the generated
  city footprints.

Launch only the portable ROS/PX4 stack with a hardware-specific parameter file:

```bash
ros2 launch drone_city_nav city_nav.launch.py \
  params_file:=/workspace/drone_city_nav/config/real_drone_template.yaml \
  enable_gazebo_bridge:=false \
  enable_mission_monitor:=false
```

Before using the real-drone template, update the lidar topic, PX4 topic version,
frame alignment, grid origin, goal, altitude, and safety limits for the actual
vehicle and test area.

In the simulation, PX4 local position starts at `(0, 0)` after the vehicle is
spawned at visual point A. Therefore the planner/monitor use local point A
`(0, 0)` and local point B `(150, 90)`.

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

The script launches `city_nav.launch.py` with the simulation parameter file,
Gazebo bridge enabled, and mission monitor enabled.

PX4 SITL output is written to `log/px4_city_mvp.log` by default. The
MicroXRCEAgent log is written to `log/uxrce_agent_city_mvp.log`. Override them
with `PX4_LOG_FILE=/path/to/px4.log` and
`UXRCE_AGENT_LOG_FILE=/path/to/agent.log` when needed.

For a full headless validation run:

```bash
HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
```

This mode starts Gazebo server-only, PX4 SITL, MicroXRCEAgent, and the ROS 2
planner/offboard launch. When the timeout is reached, the script checks the logs
for a ready Gazebo world, valid PX4 local position, lidar scans, planner
waypoints, offboard and arm commands, armed offboard state, and critical PX4
preflight failures.

During startup the script sends SITL-only PX4 parameters through the PX4 shell:
`CBRK_SUPPLY_CHK=894281` disables the unavailable power-supply check and
`NAV_DLL_ACT=0` allows a no-GCS headless run. These parameters are not saved.

Headless logs are written to:

- `log/gz_city_mvp.log`
- `log/px4_city_mvp.log`
- `log/uxrce_agent_city_mvp.log`
- `log/ros_city_mvp.log`

The script prepares Gazebo runtime resources under `build/gazebo_city_mvp` and
does not modify the PX4 checkout under `external/`.

For a full diagonal A-to-B mission validation run:

```bash
HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh
```

`MISSION_CHECK=1` requires the mission monitor to verify that the drone spawned
near point A, moved away from A, kept the configured clearance from every
building footprint, reached point B, and held position there with low speed.
The validation footprints list the tall buildings that intersect the cruise
altitude; lower buildings are visual city geometry below the flight plane.

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
- Runtime logs include distance-to-start and distance-to-goal values in
  `planner_node`, `px4_offboard_node`, and `mission_monitor_node`.
- The launch file bridges `/scan`; if the PX4 lidar model publishes a different
  Gazebo topic, update `city_nav.launch.py` or add a remap.
