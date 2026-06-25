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
  city world with uniform-height buildings, visual point A at `(-171.0, -81.0)`,
  and visual point B at `(153.0, 81.0)`.
- `drone_city_nav/models/x500_lidar_2d/model.sdf` - local PX4 X500 wrapper with
  the 2D lidar, bright yellow Gazebo GUI tracking markers, and a yellow
  ground-projection debug disc.
- `drone_city_nav/worlds/generated_city.map2d` - static 2D obstacle map for the
  same local city layout used by the planner and mission monitor.
- `drone_city_nav/worlds/generated_city_mixed_heights.sdf` - preserved
  mixed-height version of the same city layout.
- `drone_city_nav/src/obstacle_memory_node.cpp` - lidar + pose obstacle-memory
  mapper.
- `drone_city_nav/src/planner_node.cpp` - static/memory/lidar grid replanning
  node.
- `drone_city_nav/include/drone_city_nav/planner_core.hpp` - ROS-free A* path
  computation, path clearance diagnostics, and stable-path reuse decisions.
- `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp` - ROS-free
  union of static map, obstacle-memory, and current-lidar obstacle sources.
- `drone_city_nav/include/drone_city_nav/static_city_map.hpp` - static map2d
  loader and rasterizer.
- `drone_city_nav/include/drone_city_nav/grid_overlay.hpp` - occupied-wins grid
  overlay helpers for planner sources.
- `drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp` - current
  lidar hit projection overlay into a planning grid.
- `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` - persistent
  obstacle-memory core with ray clipping and hit/miss score updates.
- `drone_city_nav/include/drone_city_nav/lidar_projection.hpp` - shared lidar
  ray projection, PX4 attitude compensation, and projected-altitude filtering.
- `drone_city_nav/include/drone_city_nav/navigation_pose.hpp` - portable
  navigation pose helpers for PX4 local position.
- `drone_city_nav/src/px4_offboard_node.cpp` - PX4 offboard waypoint follower.
- `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp` - ROS-free
  waypoint advancement and target continuity.
- `drone_city_nav/include/drone_city_nav/debug_image.hpp` - small PPM debug image
  primitive used by lidar snapshots.
- `drone_city_nav/include/drone_city_nav/lidar_debug_renderer.hpp` - ROS-free
  full-map/fallback-view lidar snapshot renderer.
- `drone_city_nav/include/drone_city_nav/lidar_snapshot_writer.hpp` - ROS-free
  lidar snapshot CSV and JSONL formatting.
- `drone_city_nav/include/drone_city_nav/lidar_radar_markers.hpp` - RViz radar
  marker builder used by `lidar_debug_node`.
- `drone_city_nav/src/mission_monitor_node.cpp` - simulation-only mission
  verification node for headless runs.
- `drone_city_nav/config/urban_mvp.yaml` - default MVP parameters.
- `drone_city_nav/tests/planner_core_test.cpp` - deterministic planner/grid
  tests.
- `drone_city_nav/tests/planning_grid_builder_test.cpp` - deterministic planner
  source selection and overlay tests.
- `drone_city_nav/tests/static_city_map_test.cpp` - static map parser and
  rasterization tests.
- `drone_city_nav/tests/grid_overlay_test.cpp` - source overlay precedence tests.
- `drone_city_nav/tests/obstacle_memory_test.cpp` - deterministic obstacle
  memory and PX4 local pose adapter tests.
- `drone_city_nav/tests/current_lidar_overlay_test.cpp` - current lidar overlay
  tests without ROS messages.
- `drone_city_nav/tests/offboard_path_follower_test.cpp` - deterministic
  waypoint target selection and smoothing tests.
- `drone_city_nav/tests/debug_image_test.cpp` - deterministic PPM/debug image
  primitive tests.
- `drone_city_nav/tests/lidar_debug_renderer_test.cpp` - deterministic snapshot
  coordinate mapping and color-layer tests.
- `drone_city_nav/tests/lidar_snapshot_writer_test.cpp` - deterministic lidar
  CSV/JSONL schema tests.
- `drone_city_nav/tests/lidar_radar_markers_test.cpp` - deterministic RViz marker
  construction tests.

## Architecture Notes

The runtime ROS nodes keep ownership of subscriptions, publishers, parameters,
timestamps, and throttled logs. Domain logic that does not need ROS messages is
kept in `drone_city_nav_core` and covered with deterministic unit tests:

- Planner source union and path reuse live in `planning_grid_builder` and
  `planner_core`.
- Current lidar hit marking lives in `current_lidar_overlay`; the node still
  prepares scan metadata and the attitude-compensated projection pose.
- Offboard waypoint advancement and target continuity live in
  `offboard_path_follower`; the PX4 node still owns arming/offboard commands and
  `TrajectorySetpoint` publication.
- Lidar snapshot low-level drawing lives in `debug_image`; full snapshot
  rendering lives in `lidar_debug_renderer`; CSV/JSONL formatting lives in
  `lidar_snapshot_writer`; RViz helper markers are built by
  `lidar_radar_markers`. The lidar debug node still owns subscriptions,
  snapshot scheduling, point cloud publication, and log emission.

Headless debugging depends on stable log markers such as `Planning summary:`,
`Keeping current path`, `Published path`, `Received path`, `Offboard summary:`,
and `LIDAR_DEBUG snapshot=`. Keep these substrings stable when refactoring
runtime nodes, or update the validation scripts in the same change.

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

The MVP runtime is currently configured for Gazebo/PX4 SITL through
`urban_mvp.yaml`. Hardware-specific parameter files are intentionally not kept in
the active supported path until a real vehicle integration is defined.

In the simulation, PX4 local position starts at `(0, 0)` after the vehicle is
spawned at visual point A. The default mission sets
`px4_local_origin=(27, 27)`, so planner/monitor map-frame point A is `(27, 27)`
and point B is `(108, 189)`.

## Quick Start

The supported workflow for routine development and agent runs is the dev
container workflow. Build the dev image once:

```bash
./scripts/build_dev_image.sh
```

Use the top-level wrapper scripts for common operations:

```bash
./scripts/build.sh
./scripts/sim_gui.sh
./scripts/sim_headless.sh
./scripts/test.sh
```

The wrappers start the dev container and run the requested command inside it.
Use `./scripts/dev_shell.sh` only when you need an interactive container shell.
Inside the container shell, clone PX4-Autopilot into the ignored `external/`
folder if it is missing:

```bash
./scripts/setup_px4_autopilot.sh
export PX4_AUTOPILOT_DIR=/workspace/external/PX4-Autopilot
```

Run the MVP stack from the repository root:

```bash
./scripts/build.sh
./scripts/sim_gui.sh
./scripts/sim_headless.sh
```

Both launch variants run `city_nav.launch.py` with the simulation parameter
file, Gazebo bridge enabled, mission monitor enabled, and Offboard control.

The current-environment runner writes PX4 SITL output to
`log/px4_city_mvp.log` and the MicroXRCEAgent log to
`log/uxrce_agent_city_mvp.log`. Override them with
`PX4_LOG_FILE=/path/to/px4.log` and
`UXRCE_AGENT_LOG_FILE=/path/to/agent.log` when needed.

Override the ROS parameter file used by the run script with:

```bash
CITY_NAV_PARAMS_FILE=build/some_params.yaml ./scripts/sim_gui.sh
```

Obstacle sources are controlled by the selected params file. In the default
simulation params file all three are enabled. The run script environment
variables below are explicit launch overrides, so unset variables leave the
selected params file in control:

```bash
ENABLE_STATIC_MAP=false ./scripts/sim_gui.sh
ENABLE_OBSTACLE_MEMORY=false ./scripts/sim_gui.sh
ENABLE_CURRENT_LIDAR=false ./scripts/sim_gui.sh
ENABLE_EVASIVE_MANEUVERING=true ./scripts/sim_gui.sh
EVASIVE_MANEUVERING_STRAIGHT_COST_WEIGHT=50 ./scripts/sim_gui.sh
STATIC_CITY_MAP_PATH=drone_city_nav/worlds/generated_city.map2d ./scripts/sim_gui.sh
```

The same launch arguments can be passed manually. Leave them empty or omit them
to use `params_file`; pass a value only when an explicit launch-time override is
needed:

```bash
ros2 launch drone_city_nav city_nav.launch.py \
  use_static_map:=false \
  evasive_maneuvering:=true \
  evasive_maneuvering_straight_cost_weight:=50
```

## Lidar Debugging

The simulation launch starts `lidar_debug_node` by default and records periodic
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
LIDAR_DEBUG_DIR=log/lidar_debug_run_01 ./scripts/sim_gui.sh
ENABLE_LIDAR_DEBUG=false ./scripts/sim_gui.sh
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
./scripts/sim_gui.sh
```

Force RViz on or off with:

```bash
ENABLE_RVIZ=true ./scripts/sim_gui.sh
ENABLE_RVIZ=false ./scripts/sim_gui.sh
```

## Gazebo GUI Diagnostics

The run script keeps Gazebo server/world orchestration output and Gazebo GUI
client output in separate files so render/resource issues are not lost:

- `log/gz_city_mvp.log` - server, world-control, follow-camera, stale
  cleanup, and scene-diagnostics summaries.
- `log/gz_gui_city_mvp.log` - stdout/stderr from the `gz sim -g` GUI
  client.
- `log/gazebo_scene_debug/` - bounded `gz topic` captures for
  `/world/generated_city/pose/info`, `/world/generated_city/scene/info`, and
  `/gui/currently_tracked`, plus a `summary.txt` file with key booleans such as
  `target_model_seen`, `target_visual_seen`, `yellow_visual_seen`, and
  `gui_tracking_target_seen`.

Validate the captured launch diagnostics without relying on manual viewing:

```bash
python3 scripts/validate_gazebo_gui_launch_log.py \
  log/gz_city_mvp.log \
  --gui-log log/gz_gui_city_mvp.log \
  --scene-diagnostics-dir log/gazebo_scene_debug
```

Set `GZ_GUI_LOG_FILE=/path/to/gz_gui.log` or
`GZ_SCENE_DIAGNOSTICS_DIR=/path/to/scene_debug` to override the default paths.
Set `ENABLE_GZ_SCENE_DIAGNOSTICS=false` to skip the bounded scene topic capture.

The validator treats common EGL/render-stack messages as warnings unless the
logs contain clear launch/resource failures. Scene diagnostics fail only when
captured data contradicts required runtime facts, such as the spawned target
model being absent from non-empty Gazebo pose/scene data.

The RViz config shows red current lidar hit points from
`/drone_city_nav/lidar_debug_points`, yellow accumulated lidar hit points from
`/drone_city_nav/remembered_lidar_points`, and the final executable trajectory
from `/drone_city_nav/final_trajectory_path`. The rough A* route remains
available as a disabled debug path on `/drone_city_nav/path`. The standard RViz
`Map` display for `/drone_city_nav/prohibited_grid` is kept disabled by default
because this GUI is intended to show remembered lidar wall hits, not filled
occupancy-grid cells. All debug overlays are published in the `map` frame, so
no Gazebo lidar TF tree is required.
The RViz config also includes a disabled `Static City Map` display for
`/drone_city_nav/static_map_grid`; enable it when you need to inspect the raw
occupancy-grid encoding. The green `Static City Map Points` display is enabled
by default and shows occupied static-map cells from
`/drone_city_nav/static_map_points`.

The main obstacle-memory topics are:

- `/drone_city_nav/obstacle_memory_grid` - full raw persistent memory grid.
- `/drone_city_nav/lidar_debug_points` - current lidar hit endpoints, shown red
  in RViz.
- `/drone_city_nav/remembered_lidar_points` - accumulated lidar hit endpoints,
  shown yellow in RViz. The MVP config uses `min_remember_altitude_m=10.0` and
  `hit_memory_resolution_m=0.25` for this visual debug memory. Hits are remembered
  on first observation in each debug-memory cell. These points use the same
  attitude-compensated projected-altitude filter as obstacle memory.
- `/drone_city_nav/lidar_radar_markers` - optional lidar helper markers
  controlled by `publish_lidar_radar_markers`; disabled by default.
- `/drone_city_nav/static_map_grid` - static city map layer only. It is
  published with transient-local QoS after the map2d file is loaded.
- `/drone_city_nav/static_map_points` - occupied static city map cells as a
  point cloud for RViz. The default debug view shows this layer in green. The
  planner republishes this static debug layer periodically so RViz receives it
  even when RViz starts after the planner.
- `/drone_city_nav/prohibited_grid` - planner output grid after planner-side
  raw source overlay and single inflation. A* treats every prohibited cell as
  blocked, regardless of whether the cell started as a direct obstacle or
  planner-side inflation.

The planner builds its A* grid from three obstacle sources:

- Static map from `drone_city_nav/worlds/generated_city.map2d`. The map uses the
  planner/mission local frame, not raw Gazebo visual coordinates. Its format is
  line-oriented and versioned:

  ```text
  drone_city_nav_static_map_v1
  frame_id map
  bounds -30.0 -30.0 0.5 345.0 525.0
  rect building_001 27.0 27.0 24.0 24.0 28.0
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
./scripts/sim_headless.sh
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
- `log/offboard_blackbox.jsonl`

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
MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/sim_headless.sh
```

`MISSION_CHECK=1` requires the mission monitor to verify that the drone spawned
near point A, moved away from A, kept the configured clearance from every
building footprint, reached point B, and held position there with low speed.
The default monitor config applies `uniform_building_height_m=28.0`, matching
the default uniform-height world used by the MVP.
The default offboard tuning advances setpoints about three times faster than the
initial conservative MVP tuning.
On a mission-monitor failure, `/drone_city_nav/emergency_stop` is published.
The Offboard node stops trajectory setpoints and sends PX4 disarm commands. A
crashed vehicle is not commanded to recover and continue the mission.

For source-toggle smoke checks where a non-default source combination is known
to be unsafe for a complete mission, set `ALLOW_MISSION_FAILURE=true`. This
keeps the log-marker checks useful while explicitly allowing mission monitor and
PX4 attitude failure markers. Do not set it for full A-to-B validation.

## Offboard Velocity Cruise Control

The offboard follower uses position setpoints for takeoff, no-path hold, final
goal hold, and other fixed-position states. During normal cruise on a valid path
it switches PX4 Offboard control to velocity setpoints: horizontal velocity is
commanded along the current final trajectory tangent, and vertical velocity
keeps the configured cruise altitude.

The follower does not synthesize intermediate path waypoints between planner
waypoints. Planner waypoints remain the rough route contract, while the offboard
node builds the executable racing trajectory from corridor samples, lateral
racing-line offsets, optional baseline line/arc rounding, and a curvature-based
speed profile. Final-trajectory samples are published on
`/drone_city_nav/final_trajectory_path`; they are not fed back as position
targets. Smoothness comes from the velocity profile over the final trajectory,
acceleration/vector-delta limits, and bounded cross-track correction back toward
the current trajectory projection.

The main simulation parameters are:

- `turn_preview_distance_m` - legacy path-turn preview distance used for
  waypoint/path diagnostics. Active cruise speed limiting is based on the
  trajectory speed profile instead.
- `cruise_speed_mps` - nominal horizontal cruise speed on straight segments.
- `min_turn_speed_mps` - lower bound for the speed allowed by curved trajectory
  segments.
- `max_accel_mps2` - acceleration limit for increasing commanded cruise speed.
- `max_decel_mps2` - longitudinal deceleration limit for reducing the commanded
  velocity setpoint.
- `speed_profile_decel_mps2` - conservative trajectory-profile deceleration
  budget used to begin slowing before arcs and the final goal.
- `max_lateral_accel_mps2` - lateral acceleration budget used to convert arc
  radius/curvature into allowed turn speed, and horizontal velocity-vector
  change limit.
- `speed_profile_sample_step_m` - regular spacing for trajectory speed-profile
  samples. Segment boundaries and arc interior samples are added separately so
  short arcs cannot be skipped by a large regular step.
- `corridor_max_radius_m`, `corridor_sample_step_m`,
  `corridor_safety_margin_m`, and `corridor_rebuild_width_threshold_m` -
  corridor sampling, lateral free-space bounds around the rough route, and the
  minimum material corridor-width change that triggers a final-trajectory
  rebuild after a new prohibited grid arrives.
- `racing_line_max_iterations`, `racing_line_initial_offset_step_m`,
  `racing_line_min_offset_step_m`, and `racing_line_weight_*` - deterministic
  local optimizer controls for lateral offsets inside the corridor.
- `final_trajectory_debug_topic` and `final_trajectory_debug_sample_step_m` -
  debug path topic and visualization sample spacing for the executable final
  trajectory.
- `cross_track_gain`, `cross_track_derivative_gain`,
  `max_lateral_control_angle_deg`, `max_lateral_control_rate_mps2`,
  `curvature_feedforward_time_s`, and `max_curvature_feedforward_angle_deg` -
  the unified lateral-control block. It sums cross-track feedback,
  cross-track derivative damping, and curvature feed-forward before applying one
  common lateral velocity limiter.
- `altitude_hold_kp` and `max_vertical_speed_mps` - vertical velocity hold for
  cruise altitude.
- Final goal completion is latched in the offboard node after the vehicle
  enters the acceptance radius or crosses the final path plane. Once latched,
  cruise velocity is not re-enabled for the same mission goal.

Runtime logs from `px4_offboard_node` include `actual_speed`, current control
mode, velocity setpoint, speed-limit reason, raw and acceleration-limited speed
targets, limiting speed constraint type, limiting constraint distance, final
stop braking distance, trajectory station, segment type, curvature, curve
radius, final trajectory sample count, corridor width, racing-line cost and
offset metrics, `local_clearance`, attitude, path id correlation, cross-track
error, heading error, commanded target delta, and nearest-obstacle bearing. The
same control diagnostics are written as JSON Lines to
`log/offboard_blackbox.jsonl` for machine analysis. Legacy path-turn values, if
emitted, are explicitly named `rough_route_debug_*` and are not active speed or
trajectory-control fields.
Planner logs include smoothing rejection counters and final segment-length
statistics so headless runs can show whether short waypoint segments come from
prohibited line-of-sight failures, grid bounds, or the remaining smoothed
geometry.
Mission-monitor results include final speed plus `max_observed_speed` and
`mean_observed_speed`, so headless runs can prove that the drone moved at the
expected scale and stopped at the goal.

If Gazebo GUI cannot open from Docker, allow local X11 access on the host before
starting the dev shell:

```bash
xhost +local:docker
```

## Development Checks

Default checks:

```bash
make quality
```

## Current Limitations

- The generated city is intentionally small and synthetic.
- Only static building obstacles are modeled.
- The planner treats unknown planning-grid cells as traversable, but occupied
  cells from any enabled source remain prohibited through the union overlay.
- Planner obstacle data uses a raw/prohibited/debug contract: obstacle sources
  publish raw evidence only, the planner overlays all raw sources, and
  `PlanningGridBuilder` performs the only runtime safety inflation before
  publishing `/drone_city_nav/prohibited_grid`.
- `obstacle_memory_node` stores bounded hit/miss scores per cell. A single free
  miss does not immediately erase a remembered obstacle, but repeated free
  evidence can clear stale cells.
- Lidar hit endpoints mark only the measured endpoint cell before planner
  inflation. Raw lidar sources do not add obstacle depth or source-specific
  safety margins.
- A* base edge costs use physical grid distance in meters. `astar_turn_cost_weight`
  prefers smoother paths by penalizing turns. `astar_evasive_maneuvering_enabled`
  switches direction preference to evasive mode, where straight continuations are
  penalized by `astar_evasive_maneuvering_straight_cost_weight` instead.
  `astar_initial_heading_bias_enabled` adds a velocity-aligned soft penalty only
  to the first A* step when current speed is above
  `astar_initial_heading_bias_min_speed_mps`; it discourages left/right path
  flapping without forbidding a necessary turn.
- The offboard node converts PX4 local position into the planner map frame using
  `px4_local_origin_*` parameters, and converts map-frame targets back to PX4
  local setpoints.
- Runtime logs include obstacle-memory update statistics and distance-to-start
  and distance-to-goal values in `planner_node`,
  `px4_offboard_node`, and `mission_monitor_node`.
- The simulation offboard follower commands cruise as velocity setpoints along
  the final trajectory projection; it does not synthesize intermediate position
  targets between waypoints.
- The simulation offboard follower still uses position setpoints for takeoff,
  no-path hold, and final goal hold.
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
