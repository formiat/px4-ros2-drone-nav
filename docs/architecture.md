# Architecture

The project is organized as one ROS 2 package, `drone_city_nav`, plus scripts,
world assets, and Docker tooling around it.

## Main Nodes

`obstacle_memory_node`

- subscribes to lidar scans and PX4 pose/attitude;
- projects scan hits into the map frame;
- maintains an occupancy-style memory grid;
- publishes `/drone_city_nav/obstacle_memory_grid`.

`planner_node`

- consumes static map, current lidar overlay, and obstacle memory;
- builds the prohibited planning grid;
- runs A* rough routing;
- builds a corridor;
- builds optimized executable trajectories;
- publishes `/drone_city_nav/path`, `/drone_city_nav/path_id`,
  `/drone_city_nav/trajectory_diagnostics`, `/drone_city_nav/prohibited_grid`,
  and static-map debug topics.

`px4_offboard_node`

- consumes the accepted executable path and PX4 state;
- rebuilds runtime trajectory samples and speed profile;
- tracks the trajectory with offboard setpoints;
- publishes PX4 offboard control, trajectory setpoints, final trajectory debug
  path, and debug markers;
- writes offboard blackbox telemetry.

`lidar_debug_node`

- records lidar, memory, prohibited-grid, and trajectory debug snapshots;
- publishes point clouds for RViz;
- writes image/JSON/CSV artifacts under `log/lidar_debug`.

`mission_monitor_node`

- simulation-only monitor for mission success, crash detection, and emergency
  stop publication.

`scan_bridge`

- Gazebo-to-ROS bridge for the lidar scan and clock.

## Data Flow

```text
Gazebo lidar
  -> /scan
  -> obstacle_memory_node
  -> /drone_city_nav/obstacle_memory_grid

/scan + memory + static map + PX4 pose
  -> planner_node
  -> prohibited grid
  -> A*
  -> corridor
  -> trajectory optimizer
  -> turn smoothing
  -> executable path + diagnostics

executable path + PX4 pose/attitude/status
  -> px4_offboard_node
  -> trajectory projection
  -> speed policy
  -> velocity command
  -> velocity smoother / terminal state machine
  -> PX4 trajectory setpoint
```

## Planner vs Offboard Responsibilities

Planner responsibilities:

- obstacle source fusion;
- prohibited grid construction;
- route and trajectory generation;
- geometry smoothing;
- planning diagnostics;
- final path publication.

Offboard responsibilities:

- accepting and tracking executable trajectories;
- computing runtime trajectory samples and speed profile;
- managing trajectory continuity updates;
- generating velocity and terminal position setpoints;
- logging runtime telemetry.

The offboard node treats the planner path as an executable artifact, but it
does not blindly trust planner diagnostics. Diagnostics are matched by
`path_stamp_ns`; the accepted planner path id is confirmed from matching
diagnostics.

## Executable Trajectory Lifecycle

1. `planner_node` publishes `path_id`, diagnostics, and path.
2. `px4_offboard_node` receives a path and builds an `OffboardTrajectoryState`.
3. The candidate is validated for freshness and continuity.
4. Invalid or discontinuous candidates can be rejected while the old trajectory
   remains active.
5. Accepted candidates become the executable trajectory.
6. Planner diagnostics are merged only when their `path_stamp_ns` matches the
   accepted trajectory.

This protects the controller from switching to unrelated or stale diagnostics.

## Main Configuration Files

- `drone_city_nav/config/urban_mvp.yaml` - node parameters.
- `drone_city_nav/launch/city_nav.launch.py` - ROS launch graph.
- `drone_city_nav/rviz/city_nav_debug.rviz` - RViz debug layout.
- `docker/Dockerfile` - dev/runtime image.
- `Makefile` and `scripts/` - approved workflow entry points.
