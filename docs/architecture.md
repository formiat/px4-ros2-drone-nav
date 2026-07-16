# Architecture

The project is organized as one ROS 2 package, `drone_city_nav`, plus scripts,
world assets, and Docker tooling around it.

## Main Nodes

`obstacle_memory_node`

- subscribes to lidar scans and PX4 pose/attitude;
- projects scan hits into the map frame;
- maintains an occupancy-style memory grid;
- publishes the raw debug grid and provenance topics;
- publishes `/drone_city_nav/obstacle_memory_snapshot`, an atomic grid/provenance
  pair used by the planner.

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
  -> /drone_city_nav/obstacle_memory_snapshot

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

## Architectural Intent

The project is split into generation, acceptance, and execution layers. The
planner generates candidate geometry. The offboard node decides whether that
geometry is safe to execute now. The PX4 interface executes only the current
accepted setpoint stream. This separation is intentional: planning can be slow
or exploratory, but the controller must always have one clear executable
trajectory and one clear setpoint mode.

The important design rule is that debug data must not become an implicit
control input. RViz markers, diagnostic JSON, route-progress counters, and
timing summaries are useful for analysis, but the drone should not depend on
them unless the dependency is explicitly named and tested. When a value is only
diagnostic, the documentation and the parameter name should say so.

The second design rule is that safety validation and planning preference are
different concepts. The hard prohibited grid answers "must not cross". The
planning clearance grid answers "prefer to stay farther away". Crossing the
planning clearance should not by itself force a replan, because it is a
planning preference rather than a collision condition.

## Layer Boundaries

The normal runtime boundary is:

```text
sensor evidence
  -> obstacle memory and current scan overlay
  -> planner grid builder
  -> executable path artifact
  -> offboard trajectory state
  -> velocity or position setpoint
  -> PX4
```

Each boundary has a different failure policy:

- sensor evidence can be incomplete or noisy, so it is merged and inflated;
- planner output can be rejected, so the previous accepted trajectory remains
  active;
- offboard setpoints must be continuous enough for PX4, so the smoother and
  continuity gates protect the handover;
- PX4 terminal position capture is preserved as the final precision mode at
  the goal.

The planner is allowed to produce a better trajectory later than the first
rough route, but publication policy decides when a trajectory is exposed to the
controller. The current conservative policy is to avoid a visible
baseline-to-refined switch during flight. If asynchronous refinement is enabled
again, it needs a safe handover gate that checks projection, tangent,
curvature, speed-limit, and command discontinuity.

## Ownership Model

The planner owns:

- the grid representation used for global and local route search;
- corridor bounds and clearance diagnostics;
- trajectory geometry before publication;
- planning-only speed-profile diagnostics;
- optimizer and turn-smoothing timing.

The offboard node owns:

- the accepted executable trajectory state;
- runtime speed policy and runtime speed diagnostics;
- current projection and cross-track state;
- terminal state machine state;
- smoother history and continuity decisions;
- the actual velocity or position setpoint sent to PX4.

The same concept must not have two active owners. For example, planner
diagnostics can describe what the planner estimated, but offboard runtime
telemetry is authoritative for what the drone actually tried to fly.

## Topic And Artifact Contracts

`/drone_city_nav/path` is the executable geometry message. It is intentionally
small and ROS-native. The offboard node rebuilds samples from it and does not
assume that diagnostics arrived in the same callback order.

`/drone_city_nav/trajectory_diagnostics` is a companion artifact. It is matched
by path timestamp, not by delivery order. This matters because ROS topics are
not an atomic multi-message transaction. If a path is accepted and diagnostics
arrive later with the same stamp, the diagnostics can still be attached to the
accepted trajectory.

`/drone_city_nav/path_id` is a human-readable correlation id. It is useful for
logs, but it is weaker than the path timestamp because it travels on a separate
topic. The offboard node can update the accepted id from matching diagnostics
when the diagnostic stamp proves the association.

`/drone_city_nav/prohibited_grid` is a hard safety artifact. Consumers should
treat it as the map of space that the drone trajectory must not intersect.
Planning clearance is not published as the same concept because that would mix
hard collision validation with route preference.

## State Ownership During Replanning

When the current trajectory becomes invalid against the latest prohibited grid,
the planner can start a new build. Until a replacement trajectory is accepted,
the offboard node should continue using the previous accepted trajectory if it
is still usable. A failed build must not publish an empty executable path that
deletes the only valid command source.

This rule is important during lidar-triggered replans. Lidar can reveal an
obstacle imperfectly or only partially. The system should respond by building a
better route, but the control layer should not lose its current path merely
because the new candidate has not finished yet.

The future local-repair architecture should preserve the same ownership model:
a local repair is just another candidate executable trajectory. It can be
accepted, rejected, or ignored as stale using the same continuity and validity
checks as a full replan.

## Configuration Ownership

Several parameters are read by both planner and offboard nodes because both
need to understand the same physical limits. The project treats those values as
a contract rather than as two independent truths. Configuration fingerprints
exist to detect when the planner and the runtime controller are no longer
using compatible assumptions.

The fingerprints are split by meaning:

- speed-profile construction describes how trajectory sample limits are built;
- runtime speed policy describes how offboard chooses scalar target speed;
- runtime velocity control describes smoothing and setpoint response.

Only the construction mismatch is a strong planner/offboard compatibility
warning today. Runtime policy fingerprints are still useful context, but some
runtime settings can legitimately be offboard-only.
