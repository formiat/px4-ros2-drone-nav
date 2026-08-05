# Architecture

The project contains one ROS 2 package, `drone_city_nav`, plus Gazebo assets,
PX4 orchestration scripts, and container tooling.

## Runtime Data Flow

```text
Gazebo GPU lidar + PX4 pose
  -> obstacle_memory_node
  -> 2D obstacle memory + atomic raw obstacle snapshot

static:
  canonical Occupancy3D -> local dense ESDF3D -> 3D lattice route

no-static:
  raw obstacle snapshot -> dense ESDF2D -> sticky 2D lattice guide

selected route/guide
  -> GPU MPPI local horizon
  -> post-update classification + braking supervisor
  -> timestamped execution horizon
  -> mppi_offboard_node
  -> PX4 trajectory setpoints
```

Gazebo contacts follow an independent safety path:

```text
Gazebo contact involving the drone
  -> DroneContactSystem
  -> collision_crash_node
  -> latched crash state
  -> offboard force-disarm + mission failure
```

## Node Ownership

### `obstacle_memory_node`

- projects lidar scans into `map`;
- waits for a valid PX4 heading before accepting scan geometry;
- maintains 2D occupancy memory and sparse 3D diagnostic provenance;
- publishes `/drone_city_nav/obstacle_memory_snapshot`;
- publishes `/drone_city_nav/raw_obstacle_snapshot`;
- does not load or merge the canonical static map.

### `production_mppi_node`

- consumes PX4 state and immutable obstacle snapshots;
- loads canonical Occupancy3D directly in static mode;
- builds mode-specific ESDF snapshots asynchronously;
- builds and maintains the active global lattice guide;
- selects local lookahead targets;
- runs the persistent CUDA MPPI engine;
- follows typed 3D route samples and constrained channel spans;
- validates the reconstructed horizon;
- publishes `/drone_city_nav/mppi/execution_horizon`;
- publishes MPPI RViz and diagnostic outputs.

This node has no direct PX4 command publisher.

### `mppi_offboard_node`

- consumes only fresh `MppiTrajectoryHorizon` messages;
- applies timestamp lookahead to the current horizon;
- emits PX4 velocity or position setpoints;
- executes safety and mission position holds when explicitly requested;
- falls back to braking when no fresh executable horizon is available;
- publishes the applied-control feedback used by MPPI continuity logic;
- publishes the RViz drone marker and follow TF.

### Visualization And Observation

`world_visualization_node` publishes downsampled static Occupancy3D points, the
raw 2D world grid, and stale legacy-marker cleanup. The production MPPI markers
include mission start, mission goal, global guide, and local target.
`lidar_debug_node` writes synchronized diagnostic snapshots.
`mission_monitor_node` and `collision_crash_node` observe the mission without
participating in route selection.

## World Representation

Static production planning uses:

- sparse physical 3D occupancy generated from the canonical world;
- a local dense ESDF3D resident on the GPU;
- preferred, planning, critical, and collision risk tiers;
- typed 3D routes with constrained spans inferred from the route envelope.

No-static production planning uses the raw 2D obstacle-memory snapshot and a
2D distance field. It does not load channel metadata or the canonical 3D map.

There are no separately materialized planner/prohibited inflated grids,
hard collision envelopes around raw cells, inflation relaxation, or escape
tunnels. Conservative ESDF clearance is used only to classify the critical and
planning risk bands. A free cell remains executable even when that clearance
falls to zero.

## Global And Local Planning

The risk-aware lattice produces a route from motion primitives. Static mode
searches `(x, y, z)` against the local ESDF3D and samples the accepted result as
`RouteSample3D`. No-static mode produces a 2D guide. Its active guide is sticky:
new snapshots validate the accepted guide instead of replacing it solely
because another route scores slightly better. Blocked, exhausted, or stalled
guides can be replaced.

The lattice guide chooses route direction. GPU MPPI owns the executable local
motion and continuously warm-starts from its previous control sequence.

## Mission Layer

Point-to-point navigation remains the default mission and uses the configured
fixed position objective and terminal goal capture.

The intercept mission runs two complete navigation stacks with separate PX4 DDS
namespaces, lidar memory, planners, offboard nodes, and crash state. Pursuit uses
an explicit radar data boundary:

```text
evader ground truth -> mission referee -> outcome and settlement only
evader ground truth -> radar simulator -> RadarScan
interceptor state + RadarScan -> target tracker -> TargetTrack
interceptor state + TargetTrack -> guidance -> NavigationObjective
```

The mission referee waits until both vehicles are armed and airborne, publishes
the evader's fixed position objective, evaluates the terminal outcome, and owns
hold or disarm settlement. It cannot publish an interceptor navigation
objective. Only the referee and radar simulator subscribe to evader ground
truth; the referee verifies this subscriber set through the ROS graph before
starting the mission.

`RadarScan` exposes only range, azimuth, elevation, and relative radial velocity.
It contains no absolute target state or simulator identity. The ideal simulator
publishes immediately at a deterministic correlated cadence between 0.1 s and
3.0 s. The tracker reconstructs Cartesian position from the interceptor state
at measurement time. Its first measurement has no full velocity estimate; later
variable-dt corrections produce a constant-velocity `TargetTrack` that coasts
between measurements. Interceptor guidance runs at 20 Hz and converts that
track into a typed continuous objective. It uses a smoothed 3 s to 1 s
prediction horizon with hysteresis based on along-track and cross-track
position; vehicle yaw is not used to choose the global route.

Guidance does not read occupancy. The production planner resolves the predicted
segment against its immutable raw world, stopping at the first occupied cell and
retaining the last free sample as the ordinary planning goal. Unknown no-static
space remains traversable, and no inflation or prohibited region is introduced.
Continuous objectives retain the existing 5 m and 0.25 s route-replan gate and
collision validation but disable terminal goal capture. Swept relative-motion
evaluation detects a 5 m intercept between state samples and requests bounded
force-disarm for both vehicles.

The first terminal event is latched and cannot be reclassified by later inertial
motion. An intercept requests force-disarm for both vehicles and records the
result only after both confirmations. If the evader reaches its goal first, the
coordinator freezes the interceptor objective at its current position and
records the result only after position and speed remain inside the configured
hold tolerances. No mission termination or disarm is requested in that branch.
The capture detector remains active until settlement: a late inertial entry into
the capture radius still disarms both vehicles but cannot overwrite the latched
evader-goal outcome.
Headless runs then shut down deterministically. GUI runs keep the terminal world
alive after either result.

Each lidar pipeline filters returns belonging to the other tracked vehicle
before obstacle-memory integration. This prevents a moving agent from becoming
a persistent environmental obstacle; it does not introduce a prohibited zone
or relax collision checks against raw physical occupancy.

## Execution Contract

`MppiTrajectoryHorizon` contains:

- sequence and obstacle/pose revisions;
- `valid_from` and `valid_until`;
- risk and braking state;
- optional constrained-route speed and altitude state;
- time-indexed position, velocity, acceleration, yaw, and yaw rate;
- an explicit stationary position-hold request used by safety or goal capture.

Offboard executes only the current fresh horizon. There is no legacy path-id,
suffix ACK, partial-replan, safe-truncation, or moving/after-hold protocol.

## Safety Boundaries

- Entering a physical occupied cell in the active map is a hard collision
  result.
- Risk-band exposure ranks candidates but is not physical crash detection.
- The braking supervisor evaluates the selected horizon and can publish a
  dynamically generated fallback.
- Gazebo physical contact is the authoritative simulated crash event.

Debug topics, RViz markers, and JSONL files never feed back into control.

## Concurrency

ROS callbacks update short latest-value state. ESDF construction runs outside
the control callback. MPPI uses persistent GPU allocations. Diagnostics are
copied into a bounded latest-value mailbox and written after the execution
horizon has been published.

## Current Architectural Limits

- The lattice is not incremental AD*.
- Static planning is not incremental AD* yet.
- Static mode currently plans only against canonical Occupancy3D; lidar memory
  is not fused into its 3D collision map.
- No-static perception remains 2D and intentionally has no channel semantics.
- Collision validation intentionally uses the drone state point against
  physical occupied cells; vehicle footprint inflation is not part of the
  planning contract.
