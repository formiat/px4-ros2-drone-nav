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
- The stack can use a static 2D city map as a conservative prior source.
  `obstacle_memory_node` integrates `sensor_msgs/LaserScan` with navigation pose
  into a persistent 2D memory grid, and `planner_node` overlays static map,
  obstacle memory, and current lidar hits before inflation in the default MVP
  profile.
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
  city world with uniform-height buildings, visual point A at `(-57, -27)`,
  and visual point B at `(51, 27)`.
- `drone_city_nav/worlds/generated_city.map2d` - static 2D obstacle map for the
  same local city layout used by the planner and mission monitor.
- `drone_city_nav/worlds/generated_city_mixed_heights.sdf` - preserved
  mixed-height version of the same city layout.
- `drone_city_nav/src/obstacle_memory_node.cpp` - lidar + pose obstacle-memory
  mapper.
- `drone_city_nav/src/planner_node.cpp` - static/memory/lidar grid replanning
  node.
- `drone_city_nav/include/drone_city_nav/static_city_map.hpp` - static map2d
  loader and rasterizer.
- `drone_city_nav/include/drone_city_nav/grid_overlay.hpp` - occupied-wins grid
  overlay helpers for planner sources.
- `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` - persistent
  obstacle-memory core with ray clipping and hit/miss score updates.
- `drone_city_nav/include/drone_city_nav/lidar_projection.hpp` - shared lidar
  ray projection, PX4 attitude compensation, and projected-altitude filtering.
- `drone_city_nav/include/drone_city_nav/navigation_pose.hpp` - portable
  navigation pose and GPS/compass helpers.
- `drone_city_nav/src/px4_offboard_node.cpp` - PX4 offboard waypoint follower.
- `drone_city_nav/include/drone_city_nav/offboard_speed_controller.hpp` -
  portable speed-profile logic used by the offboard follower.
- `drone_city_nav/src/mission_monitor_node.cpp` - simulation-only mission
  verification node for headless runs.
- `drone_city_nav/config/urban_mvp.yaml` - default MVP parameters.
- `drone_city_nav/config/real_drone_template.yaml` - conservative template for
  running the planner/offboard nodes without Gazebo-specific helpers.
- `drone_city_nav/tests/planner_core_test.cpp` - deterministic planner/grid
  tests.
- `drone_city_nav/tests/static_city_map_test.cpp` - static map parser and
  rasterization tests.
- `drone_city_nav/tests/grid_overlay_test.cpp` - source overlay precedence tests.
- `drone_city_nav/tests/obstacle_memory_test.cpp` - deterministic obstacle
  memory and GPS/compass adapter tests.
- `drone_city_nav/tests/offboard_speed_controller_test.cpp` - deterministic
  speed-profile tests.

## Runtime Profiles

The core runtime nodes are `obstacle_memory_node`, `planner_node`, and
`px4_offboard_node`. They consume ROS/PX4 topics and do not depend on Gazebo
APIs. `planner_node` can optionally consume a static `*.map2d` obstacle file;
for non-simulation runs, provide a site-specific map or disable
`use_static_map`.

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
spawned at visual point A. The default mission sets
`px4_local_origin=(18, 18)`, so planner/monitor map-frame point A is `(18, 18)`
and point B is `(72, 126)`.

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

Override the ROS parameter file used by the run script with:

```bash
CITY_NAV_PARAMS_FILE=/workspace/build/some_params.yaml ./scripts/run_city_mvp.sh
```

Obstacle sources are controlled by the selected params file. In the default
simulation params file all three are enabled. The run script environment
variables below are explicit launch overrides, so unset variables leave the
selected params file in control:

```bash
ENABLE_STATIC_MAP=false ./scripts/run_city_mvp.sh
ENABLE_OBSTACLE_MEMORY=false ./scripts/run_city_mvp.sh
ENABLE_CURRENT_LIDAR=false ./scripts/run_city_mvp.sh
STATIC_CITY_MAP_PATH=/workspace/drone_city_nav/worlds/generated_city.map2d ./scripts/run_city_mvp.sh
```

The same launch arguments can be passed manually. Leave them empty or omit them
to use `params_file`; pass a value only when an explicit launch-time override is
needed:

```bash
ros2 launch drone_city_nav city_nav.launch.py use_static_map:=false
```

## Lidar Debugging

The simulation launch starts `lidar_debug_node` by default. It records periodic
snapshots under `log/lidar_debug`:

- `snapshots.jsonl` - one JSON record per snapshot with pose, horizontal speed,
  PX4 attitude diagnostics (`roll_rad`, `pitch_rad`, `tilt_rad`), scan
  statistics including projected-altitude rejection counts, obstacle-memory grid
  statistics, path size, file paths, projection config/stats, and a capped list
  of hit points.
- `snapshot_000001_scan.csv` - per-beam scan data with raw range, interpreted
  hit flag, projection status, map-frame endpoint, depth endpoint, projected
  endpoint altitude, lidar-frame direction, body-FRD direction, and NED/map
  direction.
- `snapshot_000001.ppm` - a full-map top-down debug image when the memory grid
  is available. Red dots are current lidar hits, yellow dots are accumulated
  remembered lidar hits, cyan/green lines are the current path, and the blue
  marker is the drone. Occupancy-grid cells are counted in JSON but are not
  drawn in this image. Remembered lidar hits are altitude-gated and require
  repeated confirmations before they are displayed. Lidar hit endpoints are
  projected with PX4 roll/pitch compensation and a projected-altitude filter, so
  tilted takeoff or acceleration scans do not permanently store ground returns as
  obstacle outlines.

Override the debug directory or disable recording from the run script:

```bash
LIDAR_DEBUG_DIR=/workspace/log/lidar_debug_run_01 ./scripts/run_city_mvp.sh
ENABLE_LIDAR_DEBUG=false ./scripts/run_city_mvp.sh
```

Validate a headless run without opening Gazebo or RViz:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

The analyzer fails on missing snapshots, missing cruise-altitude current hits,
dominant projected-altitude rejection at cruise altitude, missing final
remembered hits, failed snapshot images, or inconsistent projection config.

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

The RViz config shows red current lidar hit points from
`/drone_city_nav/lidar_debug_points`, yellow accumulated lidar hit points from
`/drone_city_nav/remembered_lidar_points`, and `/drone_city_nav/path`. The
standard RViz `Map` display for `/drone_city_nav/obstacle_memory_inflated_grid`
is kept disabled by default because this GUI is intended to show remembered
lidar wall hits, not filled occupancy-grid cells. All debug overlays are
published in the `map` frame, so no Gazebo lidar TF tree is required.
The RViz config also includes a disabled `Static City Map` display for
`/drone_city_nav/static_map_grid`; enable it when you need to inspect the raw
occupancy-grid encoding. The green `Static City Map Points` display is enabled
by default and shows occupied static-map cells from
`/drone_city_nav/static_map_points`.

The main obstacle-memory topics are:

- `/drone_city_nav/obstacle_memory_grid` - full raw persistent memory grid.
- `/drone_city_nav/obstacle_memory_inflated_grid` - full memory grid after
  safety inflation for debugging clearance.
- `/drone_city_nav/lidar_debug_points` - current lidar hit endpoints, shown red
  in RViz.
- `/drone_city_nav/remembered_lidar_points` - accumulated lidar hit endpoints,
  shown yellow in RViz. The MVP config uses `min_remember_altitude_m=10.0`,
  `remembered_hit_min_confirmations=3`, and `hit_memory_resolution_m=0.25` for
  this visual debug memory. These points use the same attitude-compensated
  projected-altitude filter as obstacle memory.
- `/drone_city_nav/lidar_radar_markers` - optional lidar helper markers
  controlled by `publish_lidar_radar_markers`; disabled by default.
- `/drone_city_nav/static_map_grid` - static city map layer only. It is
  published with transient-local QoS after the map2d file is loaded.
- `/drone_city_nav/static_map_points` - occupied static city map cells as a
  point cloud for RViz. The default debug view shows this layer in green. The
  planner republishes this static debug layer periodically so RViz receives it
  even when RViz starts after the planner.
- `/drone_city_nav/occupancy_grid` - planner output grid after planner-side
  source overlay and inflation, kept for compatibility with existing debug
  tooling.

The planner builds its A* grid from three obstacle sources:

- Static map from `drone_city_nav/worlds/generated_city.map2d`. The map uses the
  planner/mission local frame, not raw Gazebo visual coordinates. Its format is
  line-oriented and versioned:

  ```text
  drone_city_nav_static_map_v1
  frame_id map
  bounds -10.0 -10.0 0.5 115.0 175.0
  rect building_001 9.0 9.0 8.0 8.0 28.0
  ```

- Persistent obstacle memory from `/drone_city_nav/obstacle_memory_grid`.
- A temporary overlay of the latest fresh `/scan` hit endpoints. This overlay is
  applied only to the planner's working grid before inflation, so it can help at
  lower altitude without permanently storing takeoff-time artifacts.

The merge rule is conservative: if any enabled source marks a cell occupied, the
planner treats it as occupied. Free cells from obstacle memory cannot clear a
static-map or current-lidar occupied cell. Inflation runs after all enabled
source overlays are applied.

In the MVP config, obstacle-memory integration starts at
`min_mapping_altitude_m=5.0`. The yellow visual remembered-hit layer is more
conservative and starts at `min_remember_altitude_m=10.0`, because it is only a
GUI/debug memory and should avoid clutter from takeoff transients.

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
for a ready Gazebo world, valid PX4 local position, source-appropriate lidar
markers, obstacle-memory updates or disabled-state markers, static map loading
when enabled, planner source configuration, planner waypoints, offboard and arm
commands, armed offboard state, and critical PX4 preflight failures.

During startup the script sends SITL-only PX4 parameters through the PX4 shell:
`CBRK_SUPPLY_CHK=894281` disables the unavailable power-supply check and
`NAV_DLL_ACT=0` allows a no-GCS headless run. These parameters are not saved.

Headless logs are written to:

- `log/gz_city_mvp.log`
- `log/px4_city_mvp.log`
- `log/uxrce_agent_city_mvp.log`
- `log/ros_city_mvp.log`

Useful ROS log markers for obstacle-source debugging:

- `Planner obstacle sources: static=true memory=true current_lidar=true`
- `Static city map loaded:`
- `Published static map grid:`
- `Planning summary: ... static[enabled=true loaded=true used=true ...]`
- `memory[enabled=true seen=true used=true ...]`
- `current_lidar[enabled=true used=true fresh=true ...]`
- `altitude_rejected=...`

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

For source-toggle smoke checks where a non-default source combination is known
to be unsafe for a complete mission, set `ALLOW_MISSION_FAILURE=true`. This
keeps the log-marker checks useful while explicitly allowing mission monitor and
PX4 attitude failure markers. Do not set it for full A-to-B validation.

## Offboard Speed Control

The offboard follower uses position setpoints by default, but speed is now an
explicit requested profile instead of only an emergent result of target motion.
The main simulation parameters are:

- `desired_speed_mps` - requested cruise speed for normal path following.
- `max_accel_mps2` - acceleration/deceleration limit for requested setpoint
  progression.
- `goal_slowdown_radius_m` and `braking_safety_margin_m` - reduce requested
  speed before point B.
- `turn_slowdown_angle_rad` and `turn_slowdown_min_speed_mps` - reduce speed
  before sharp path turns.
- `narrow_clearance_slowdown_radius_m` and
  `narrow_clearance_min_speed_mps` - reduce speed near occupied or inflated
  planner grid cells.
- `max_commanded_target_step_m` - hard per-tick safety cap that still bounds
  target motion at the 10 Hz controller rate.
- `velocity_feedforward_enabled` - experimental feed-forward flag. It is
  disabled by default; the MVP remains position-setpoint controlled unless this
  is explicitly enabled for SITL validation.

Runtime logs from `px4_offboard_node` include `requested_speed`,
`actual_speed`, `speed_limit_reason`, `allowed_speed`, `braking_distance`,
`target_step`, `turn_angle`, and `local_clearance`. Mission-monitor results
include final speed plus `max_observed_speed` and `mean_observed_speed`, so
headless runs can prove that the drone moved at the expected scale and stopped
at the goal.

Run a simple simulation speed sweep without editing tracked YAML files:

```bash
./scripts/run_speed_sweep.sh 3 5 7
```

The sweep writes temporary parameter files under `build/speed_sweep` and logs
each run under `log/speed_sweep_<speed>`.

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
- The planner treats unknown planning-grid cells as traversable, but occupied
  cells from any enabled source remain blocked through the union overlay.
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
- The simulation offboard follower has explicit requested speed, acceleration,
  goal/turn/clearance slowdown, and a hard per-tick target-step cap. Actual
  speed is still ultimately limited by PX4 position-controller internals.
- The offboard follower also applies path continuity hysteresis so frequent
  replanning updates do not immediately force large lateral target jumps when a
  nearby waypoint from the new path still matches the previous local target.
  This continuity is disabled for far-away targets such as the final goal.
- When obstacle-memory mapping is enabled, the simulation obstacle-memory node
  ignores persistent lidar map updates below `min_mapping_altitude_m`. The
  planner still overlays fresh lidar hits onto its temporary A* grid when
  `use_current_lidar_obstacles=true`.
- `use_static_map`, `use_obstacle_memory`, and `use_current_lidar_obstacles`
  can be toggled before launch. If every obstacle source is disabled or no
  enabled source has usable data, the planner publishes an empty hold path.
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
