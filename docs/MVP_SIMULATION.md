# Drone City Navigation MVP

This repository contains a first ROS 2/PX4/Gazebo MVP for single-drone urban
flight at a fixed altitude.

## Scope

- Gazebo provides a generated Manhattan-style city grid with 40 static
  buildings. The default MVP world uses uniform 28 m buildings so the
  horizontal 2D lidar can observe every obstacle at the configured cruise
  altitude. The original mixed-height world is preserved as
  `drone_city_nav/worlds/generated_city_mixed_heights.sdf` for later
  experiments.
- PX4 SITL provides stabilization and accepts offboard trajectory setpoints.
- ROS 2 runs an obstacle-memory mapper, a planner, and a PX4 offboard control
  node.
- The stack starts without a prior map. `obstacle_memory_node` integrates
  `sensor_msgs/LaserScan` with navigation pose into a persistent 2D memory grid.
  `planner_node` subscribes to that raw memory grid, inflates occupied cells,
  runs A*, smooths the result with line-of-sight checks, and republishes a
  waypoint path.
- If the planner cannot find a path or has no valid map/pose, it publishes an
  empty path so the offboard node holds position instead of moving without a
  target.

This repository is still an MVP, not a certified collision-avoidance system. The
planner and PX4 offboard nodes are kept independent from Gazebo, but any real
hardware use needs a separate safety review, real sensor calibration, geofence,
RC override, failsafe behavior, and staged tethered/low-risk tests.

## Frames And Units

- Planner grid frame: local horizontal `map`, meters.
- PX4 local position: NED, using `VehicleLocalPosition.x` as north/meters and
  `VehicleLocalPosition.y` as east/meters.
- Gazebo visual coordinates are ENU-like in this simulation. The PX4 local
  mission frame maps Gazebo north/Y to local X and Gazebo east/X to local Y.
- Published `nav_msgs/Path` uses positive altitude for visualization.
- PX4 `TrajectorySetpoint` uses NED altitude, so the offboard node sends
  `z = -cruise_altitude_m`.
- Lidar rays are interpreted in the local horizontal frame with a configurable
  `scan_yaw_offset_rad`.

## Main Files

- `drone_city_nav/worlds/generated_city.sdf` - generated Manhattan-style static
  city world with uniform-height buildings, visual point A at `(-75, -45)`,
  and visual point B at `(75, 45)`.
- `drone_city_nav/worlds/generated_city_mixed_heights.sdf` - preserved
  mixed-height version of the same city layout.
- `drone_city_nav/src/obstacle_memory_node.cpp` - lidar + pose obstacle-memory
  mapper.
- `drone_city_nav/src/planner_node.cpp` - memory-grid replanning node.
- `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` - persistent
  obstacle-memory core with ray clipping and hit/miss score updates.
- `drone_city_nav/include/drone_city_nav/navigation_pose.hpp` - portable
  navigation pose and GPS/compass helpers.
- `drone_city_nav/src/px4_offboard_node.cpp` - PX4 offboard waypoint follower.
- `drone_city_nav/src/mission_monitor_node.cpp` - simulation-only mission
  verification node for headless runs.
- `drone_city_nav/config/urban_mvp.yaml` - default MVP parameters.
- `drone_city_nav/config/real_drone_template.yaml` - conservative template for
  running the planner/offboard nodes without Gazebo-specific helpers.
- `drone_city_nav/tests/planner_core_test.cpp` - deterministic planner/grid
  tests.
- `drone_city_nav/tests/obstacle_memory_test.cpp` - deterministic obstacle
  memory and GPS/compass adapter tests.

## Runtime Profiles

The core runtime nodes are `obstacle_memory_node`, `planner_node`, and
`px4_offboard_node`. They consume ROS/PX4 topics and do not depend on Gazebo
APIs or preloaded building coordinates.

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

Before using the real-drone template, update the lidar topic, GPS/compass
topics or PX4 local position topic version, frame alignment, grid origin, goal,
altitude, and safety limits for the actual vehicle and test area. The template
supports `pose_source: gps_compass` through `sensor_msgs/NavSatFix` and
`sensor_msgs/Imu`, or `pose_source: px4_local_position` when PX4 estimator local
position is available. In `gps_compass` mode, both GPS and compass yaw must stay
fresh: `max_gps_staleness_s` bounds GPS fixes, and `max_compass_staleness_s`
bounds compass yaw. If either source is missing, invalid, or stale, obstacle
memory skips lidar integration instead of reusing cached heading data.

In the simulation, PX4 local position starts at `(0, 0)` after the vehicle is
spawned at visual point A. Therefore the planner/monitor use local point A
`(0, 0)` and local point B `(90, 150)`.

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

## Lidar Debugging

The simulation launch starts `lidar_debug_node` by default. It records periodic
snapshots under `log/lidar_debug`:

- `snapshots.jsonl` - one JSON record per snapshot with pose, scan statistics,
  obstacle-memory grid statistics, path size, file paths, and a capped list of
  hit points.
- `snapshot_000001_scan.csv` - per-beam scan data with raw range, interpreted
  hit flag, and map-frame endpoint.
- `snapshot_000001.ppm` - a full-map top-down debug image when the memory grid
  is available. Red dots/rays are current lidar hits, yellow cells are occupied
  obstacle-memory cells, amber cells are inflated safety cells, cyan/green lines
  are the current path, and the blue marker is the drone.

Override the debug directory or disable recording from the run script:

```bash
LIDAR_DEBUG_DIR=/workspace/log/lidar_debug_run_01 ./scripts/run_city_mvp.sh
ENABLE_LIDAR_DEBUG=false ./scripts/run_city_mvp.sh
```

The regular GUI launch starts Gazebo and RViz so the same data can be inspected
live:

```bash
./scripts/run_city_mvp.sh
```

Force RViz on or off with:

```bash
ENABLE_RVIZ=true ./scripts/run_city_mvp.sh
ENABLE_RVIZ=false ./scripts/run_city_mvp.sh
```

The RViz config shows obstacle-memory cells from
`/drone_city_nav/obstacle_memory_markers`, `/drone_city_nav/path`, and red-only
lidar hit points from `/drone_city_nav/lidar_debug_points`. The standard RViz
`Map` display for `/drone_city_nav/obstacle_memory_inflated_grid` is kept
disabled by default because its fixed black/gray color scheme hides the intended
yellow obstacle-memory view. All debug overlays are published in the `map`
frame, so no Gazebo lidar TF tree is required.

The main obstacle-memory topics are:

- `/drone_city_nav/obstacle_memory_grid` - full raw persistent memory grid.
- `/drone_city_nav/obstacle_memory_inflated_grid` - full memory grid after
  safety inflation for debugging clearance.
- `/drone_city_nav/obstacle_memory_markers` - RViz marker overlay for the
  obstacle-memory grid. Occupied cells are yellow, inflated safety cells are
  amber, and non-hit lidar helper markers are not published by default.
- `/drone_city_nav/occupancy_grid` - planner output grid after planner-side
  inflation, kept for compatibility with existing debug tooling.

For replay-oriented debugging, record the relevant topics in a second shell
while the simulation is running:

```bash
./scripts/record_debug_bag.sh
```

For a full headless validation run:

```bash
HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
```

This mode starts Gazebo server-only, PX4 SITL, MicroXRCEAgent, and the ROS 2
planner/offboard launch. When the timeout is reached, the script checks the logs
for a ready Gazebo world, valid PX4 local position, lidar scans, obstacle-memory
updates, planner waypoints, offboard and arm commands, armed offboard state,
and critical PX4 preflight failures.

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
The default monitor config applies `uniform_building_height_m=28.0`, matching
the default uniform-height world used by the MVP.
The default offboard tuning advances setpoints about three times faster than the
initial conservative MVP tuning.
On a mission-monitor failure, `/drone_city_nav/emergency_stop` is published and
the offboard node stops trajectory setpoints and sends PX4 disarm commands, so a
crashed vehicle is not commanded to recover and continue the mission.

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
- The planner treats unknown memory cells as traversable and replans as
  obstacle-memory data arrives.
- `obstacle_memory_node` stores bounded hit/miss scores per cell. A single free
  miss does not immediately erase a remembered obstacle, but repeated free
  evidence can clear stale cells.
- Lidar hit endpoints are extended by the configurable
  `hit_obstacle_depth_m` before inflation. This conservative 2D mapping
  heuristic prevents A* from treating the unseen volume immediately behind a
  detected wall as a free corridor.
- The offboard node assumes the planner path and PX4 local position share the
  same horizontal origin.
- Runtime logs include obstacle-memory update statistics and distance-to-start
  and distance-to-goal values in `planner_node`,
  `px4_offboard_node`, and `mission_monitor_node`.
- The simulation offboard follower uses a short lookahead so obstacle-avoidance
  waypoints are followed instead of being skipped by a direct-to-goal setpoint.
- The offboard follower also applies path continuity hysteresis so frequent
  replanning updates do not immediately force large lateral target jumps when a
  nearby waypoint from the new path still matches the previous local target.
  This continuity is disabled for far-away targets such as the final goal.
- The simulation obstacle-memory node ignores lidar map updates below
  `min_mapping_altitude_m` so takeoff-time ground returns do not pollute the
  cruise-altitude 2D occupancy grid.
- `obstacle_memory_node` and `planner_node` both fail closed when PX4 local
  position becomes invalid or stale for longer than `max_pose_staleness_s`.
  Obstacle memory skips lidar integration, and the planner publishes an empty
  hold path instead of using cached pose data.
- The simulation parameter file keeps `use_px4_heading_for_scan=false` in
  `obstacle_memory_node` because the bridged Gazebo lidar scan used by this MVP
  is already aligned with the local horizontal map frame. Enabling PX4 heading
  rotation misplaces obstacle hits in the occupancy grid. In this map-aligned
  mode, `initial_heading_rad` is treated as the valid scan yaw. When
  `use_px4_heading_for_scan=true`, obstacle memory requires a finite PX4 heading
  with `heading_good_for_control=true`; otherwise scans are skipped instead of
  being integrated with a stale or default yaw.
- The launch file bridges `/scan`; if the PX4 lidar model publishes a different
  Gazebo topic, update `city_nav.launch.py` or add a remap.
