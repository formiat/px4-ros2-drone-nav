# Architecture

The project contains one ROS 2 package, `drone_city_nav`, plus Gazebo assets,
PX4 orchestration scripts, and container tooling.

## Runtime Data Flow

```text
Gazebo GPU lidar + PX4 pose
  -> obstacle_memory_node
  -> obstacle memory + atomic raw obstacle snapshot
  -> production_mppi_node ESDF worker
  -> sticky risk-aware lattice guide
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
- maintains occupancy memory and sparse 3D provenance;
- optionally merges the static city map;
- publishes `/drone_city_nav/obstacle_memory_snapshot`;
- publishes `/drone_city_nav/raw_obstacle_snapshot`;
- owns the optional known-static lidar fallback classifier.

### `production_mppi_node`

- consumes PX4 state and immutable obstacle snapshots;
- builds ESDF snapshots asynchronously;
- builds and maintains the active global lattice guide;
- selects local lookahead targets;
- runs the persistent CUDA MPPI engine;
- coordinates known-passage approach, vertical alignment, and traversal;
- validates the reconstructed horizon;
- publishes `/drone_city_nav/mppi/execution_horizon`;
- publishes MPPI RViz and diagnostic outputs.

This node has no direct PX4 command publisher.

### `mppi_offboard_node`

- consumes only fresh `MppiTrajectoryHorizon` messages;
- applies timestamp lookahead to the current horizon;
- emits PX4 velocity or position setpoints;
- executes passage stationary-position hold when explicitly requested;
- falls back to braking when no fresh executable horizon is available;
- publishes the applied-control feedback used by MPPI continuity logic;
- publishes the RViz drone marker and follow TF.

### Visualization And Observation

`world_visualization_node` publishes static-map, raw-world, building, and
known-passage visualization. The production MPPI markers include mission start,
mission goal, global guide, and local target. `lidar_debug_node` writes
synchronized diagnostic snapshots. `mission_monitor_node` and
`collision_crash_node` observe the mission without participating in route
selection.

## World Representation

The production planner uses:

- raw 2D occupied cells;
- an ESDF resident on the GPU;
- preferred, planning, critical, and collision risk tiers;
- known 3D solid volumes for passage buildings;
- passage opening metadata.

There are no separately materialized planner/prohibited inflated grids,
hard collision envelopes around raw cells, inflation relaxation, or escape
tunnels. Conservative ESDF clearance is used only to classify the critical and
planning risk bands. A free cell remains executable even when that clearance
falls to zero.

## Global And Local Planning

The risk-aware lattice produces a guide polyline from motion primitives. The
active guide is sticky: new snapshots validate the accepted guide instead of
replacing it solely because another route scores slightly better. Blocked,
exhausted, or stalled guides can be replaced.

The lattice guide chooses route direction. GPU MPPI owns the executable local
motion and continuously warm-starts from its previous control sequence.

## Execution Contract

`MppiTrajectoryHorizon` contains:

- sequence and obstacle/pose revisions;
- `valid_from` and `valid_until`;
- risk and braking state;
- optional passage constraint state;
- time-indexed position, velocity, acceleration, yaw, and yaw rate;
- an explicit stationary position-hold request used during passage alignment.

Offboard executes only the current fresh horizon. There is no legacy path-id,
suffix ACK, partial-replan, safe-truncation, or moving/after-hold protocol.

## Safety Boundaries

- Raw occupancy and known-solid intersections are hard collision results.
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
- MPPI follows a lookahead target rather than a full GPU-resident guide
  polyline.
- Passage selection is inferred from guide/opening intersection.
- Stationary passage hold shares the execution-horizon message.
- Collision validation intentionally uses the drone state point against raw
  occupied cells and exact known-solid volumes; vehicle footprint inflation is
  not part of the planning contract.
