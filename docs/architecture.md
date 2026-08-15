# Architecture

The project contains one ROS 2 package, `drone_city_nav`, plus Gazebo assets,
PX4 orchestration scripts, and container tooling.

## Runtime Data Flow

```text
Gazebo GPU lidar + PX4 pose
  -> obstacle_memory_node
  -> 2D obstacle memory + atomic raw obstacle snapshot

static:
  canonical Occupancy3D + precomputed chunked ESDF3D
  -> extracted local dense ESDF3D -> 3D lattice route

no-static:
  raw obstacle snapshot -> dense ESDF2D -> sticky 2D lattice guide

selected route/guide
  -> GPU MPPI local horizon
  -> post-update physical validation + route-availability state
  -> timestamped execution horizon
  -> mppi_offboard_node
  -> PX4 trajectory setpoints
```

Gazebo contacts follow an independent safety path:

```text
Gazebo contact involving the drone
  -> DroneContactSystem
  -> collision_crash_node
  -> VehicleDestroyed(role, physical_collision)
  -> latched offboard death lifecycle
  -> bounded force-disarm retries until PX4 confirms disarmed
```

## Node Ownership

### `obstacle_memory_node`

- projects lidar scans into `map`;
- requires a configured number of mutually stable PX4 heading samples before
  accepting scan geometry;
- starts a new pose-history generation at every valid PX4-heading handoff;
- maintains 2D occupancy memory and sparse 3D diagnostic provenance;
- publishes a lightweight `/drone_city_nav/obstacle_memory_status` after every
  accepted update;
- publishes `/drone_city_nav/raw_obstacle_snapshot` on every no-static update and
  at the debug cadence in static mode;
- publishes the full atomic memory/provenance snapshot at the debug cadence;
- publishes timestamp-aligned raw lidar hit endpoints independently of
  persistent-memory integration;
- does not load or merge the canonical static map.

### `production_mppi_node`

- consumes PX4 state, the memory-status heartbeat, and immutable raw obstacle
  snapshots where required;
- loads canonical Occupancy3D directly in static mode;
- extracts the resident static ESDF asynchronously from a fingerprint-validated
  precomputed cache as soon as navigation state and the configured objective are
  ready, independently of lidar snapshots;
- falls back to the exact runtime EDT when the cache is unavailable or invalid;
- publishes latched planner-world readiness after successful ESDF activation;
- builds and maintains the active global lattice guide;
- selects local lookahead targets;
- runs the persistent CUDA MPPI engine;
- follows typed 3D route samples and constrained passage spans;
- converts the reconstructed horizon into one finite path whose speed profile
  reaches a terminal rest state;
- validates that complete path against physical occupancy and fresh direct raw
  lidar evidence where required;
- publishes a typed position hold while no physically executable route exists;
- retains the remaining trajectory of the previous finite path only when both
  its geometry and its remaining controls from the measured vehicle state are
  physically valid; otherwise it may rebuild the
  remaining path from the measured state, shape its arrival profile again, and
  validate the rebuilt path without extending the previous validity
  window;
- publishes a typed position hold only when neither a new path nor the
  remaining previous path is physically executable, and resumes immediately
  when a new finite path validates;
- publishes `/drone_city_nav/mppi/execution_horizon`;
- publishes MPPI RViz and diagnostic outputs.

This node has no direct PX4 command publisher.

The intercept launch loads all four production planners as ROS 2 components in
one multithreaded component container. Each component retains independent
vehicle state, route lifecycle, worker pool, CUDA stream, ESDF, and MPPI nominal
controls. Sharing one process removes redundant ROS/DDS process overhead and
lets all planners use one CUDA primary context; it does not merge vehicle state
or make one vehicle's planner callbacks depend on another vehicle.

Radar target trackers and interceptor guidance nodes run in a separate
multithreaded component container. Tracker-to-guidance `TargetTrack` delivery
uses ROS 2 intra-process transport, while each interceptor retains independent
tracker and guidance state. Radar simulators and the mission referee remain
separate processes because they form the ground-truth data boundary.

The spectator, diagnostics mux, world visualization, and enabled lidar-debug
nodes share a diagnostics-only component container. Intra-process transport
avoids serializing spectator selection and detailed point clouds between these
components. This container remains isolated from planning, mapping, control,
the mission referee, and radar simulators.

### `mppi_offboard_node`

- consumes only fresh `MppiTrajectoryHorizon` messages;
- applies timestamp lookahead to the current horizon;
- tracks finite path positions with velocity and acceleration feed-forward;
- holds the validated terminal path point after its deadline if no replacement
  command arrives;
- emits PX4 trajectory or position setpoints;
- executes safety and mission position holds when explicitly requested;
- holds the current admissible position when no finite path or explicit hold
  command has provided an executable terminal state;
- publishes the applied-control feedback used by MPPI continuity logic;
- accepts typed destruction events for its configured role and mission epoch;
- owns the only force-disarm command path, after a valid destruction event;
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
- an offline-generated chunked global ESDF carrying the same world fingerprint;
- a local dense ESDF3D resident on the GPU;
- preferred, planning, critical, and collision risk tiers;
- typed 3D routes with constrained spans inferred from the route envelope.

No-static production planning uses the raw 2D obstacle snapshot and a
2D distance field. It does not load passage metadata or the canonical 3D map.

There are no separately materialized planner/prohibited inflated grids,
artificial hard collision envelopes around raw cells, inflation relaxation, or
escape tunnels. Conservative ESDF clearance is used only to classify the
critical and planning risk bands and compute soft exposure cost. A rollout
remains executable at low clearance unless the drone's actual swept physical
footprint intersects a raw occupied cell.

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

The finite intercept mission runs four complete navigation stacks: three
interceptors and one evader, each with a separate PX4 DDS namespace, planner,
offboard node, lidar safety input, and destruction state. No-static navigation
maintains independent persistent lidar memory for every vehicle. Static
navigation uses the canonical world for planning and keeps diagnostic persistent
memory only for the current spectator vehicle. Pursuit uses an explicit radar
data boundary:

```text
Gazebo Pose_V -> simulation truth adapter -> typed physical vehicle states
typed evader truth -> mission referee -> outcome and settlement only
typed evader truth -> three radar simulators -> independent RadarScan streams
interceptor[i] state + RadarScan[i] -> tracker[i] -> TargetTrack[i]
interceptor[i] state + TargetTrack[i] -> guidance[i] -> NavigationObjective[i]
```

`intercept_scenario.json` is the only hand-written source for vehicle map starts
and the evader goal. Its Gazebo spawns are derived from the canonical world's
`map_to_sdf` transform by both the runner and launch tooling. The simulation
truth adapter converts physical Gazebo poses back to map coordinates and checks
them against navigation states over consecutive samples. A persistent mismatch
prevents mission start and requests hold for already airborne vehicles.

Visualization remains outside this control boundary. Each planner publishes a
namespaced lightweight path. `intercept_diagnostics_mux_node` subscribes to the
latched `SpectatorTarget`, clears the previous selected layers, and republishes
the selected planner markers, execution horizon, status, memory cloud, and
lidar-debug clouds on stable RViz topics. Lightweight interceptor paths remain
visible concurrently. Selector-gated lidar debug nodes retain pose and
latest-map context for every scenario vehicle, but only the current spectator
projects scans, integrates diagnostic memory, writes a bounded startup snapshot,
and publishes detailed lidar layers. Non-selected vehicles continue publishing
their latest physical lidar returns for finite-path validation, but do not build
static-mode diagnostic memory. These
visualization-only nodes share the diagnostics component container; their
outputs never participate in route selection or vehicle control.

The radar anti-leak graph contract grants the spectator and diagnostics mux a
narrow read-only exception for target navigation-state topics. That exception
exists only to render and follow an attacker. Neither node receives physical
Gazebo truth, publishes tracking objectives, or shares a process with tracker or
guidance components.

The mission referee publishes the evader's fixed position objective, then waits
until all four vehicles are navigation-ready, all planners have activated a
world, all three trackers have produced a valid target position, and physical
and navigation coordinates are aligned. It evaluates the
terminal outcome and owns hold or disarm settlement. It cannot publish an
interceptor navigation objective. The referee verifies both boundaries through
the ROS graph: only the referee and simulation-truth adapter may subscribe to
the evader navigation state, and only the referee and three radar simulators may
subscribe to typed evader physical truth.

`RadarScan` exposes only range, azimuth, elevation, and relative radial velocity.
It contains no absolute target state or simulator identity. The ideal simulator
publishes immediately at a deterministic correlated cadence between 0.1 s and
3.0 s in search mode. The planner publishes a typed mode command containing no
target state: swept raw-clear visibility of the current target estimate requests
an immediate scan and 20 Hz track mode at any range; occlusion restores search
cadence. The tracker reconstructs Cartesian position from the interceptor state
at measurement time. Its first measurement has no full velocity estimate; later
variable-dt corrections produce a constant-velocity `TargetTrack` that coasts
between measurements. Ideal high-rate scans use full velocity innovation gain.
Each interceptor guidance node runs at 20 Hz and converts its track into a typed
continuous objective. All three follow measured target motion by default. When
`intercept_directional_hypotheses_enabled` is enabled, the other two rotate only
their long-range prediction by `-45` and `+45` degrees. Their effective offsets
continuously converge to zero from 120 m to 30 m and their lateral displacement
is capped at 70 m. Radar tracks and measured velocities remain unchanged.
Guidance solves the constant-velocity
intercept equation, caps the result at 15 s, and caps the horizon at 1 s while
ahead inside the target corridor. Vertical coasting applies bounded
deceleration until vertical speed reaches zero and clips altitude to the flight
envelope instead of rejecting the complete tracking objective. Vehicle yaw is
not used to choose the global route.

Guidance does not read occupancy. The production planner resolves the predicted
segment against its immutable raw world, stopping at the first occupied cell and
retaining the last free sample as the ordinary planning goal. Unknown no-static
space remains traversable, and no inflation or prohibited region is introduced.
The planner separately validates swept visibility of the coasted current target
and the path to the full predicted intercept point. Current-target visibility
keeps direct interception active; blockage of only the full prediction shortens
the lead to the farthest directly reachable point, down to the current target.
Current-target occlusion exits direct mode immediately and atomically hands off
to a current-generation global route. MPPI minimizes
closest approach to the target trajectory over its horizon, while raw collision
remains forbidden. Continuous objectives disable terminal goal capture. Swept
relative-motion evaluation over physical Gazebo poses detects a 5 m intercept
between state samples and
publishes one typed `VehicleDestroyed` event for the capturing interceptor and
one for the evader with cause `proximity_intercept`. Every death event includes
a stable `vehicle_id`, so role alone never identifies one of several
interceptors.

The first terminal event is latched and cannot be reclassified by later inertial
motion. Evader goal arrival is latched on the first airborne sample inside the
configured goal radius, without a stop-speed or hold-time delay. An intercept
records the result only after both typed destruction events, both PX4 disarm
confirmations, and confirmed holds from every surviving interceptor. If the
evader reaches its goal first, the coordinator commands every surviving
interceptor to transition to a typed stationary position hold. It records
the result only after a post-command position-hold horizon is active and all
positions and speeds remain inside the configured hold tolerances. No mission
termination or disarm is requested in that branch.
The capture detector remains active until settlement: a late inertial entry into
the capture radius still disarms both vehicles but cannot overwrite the latched
evader-goal outcome.
Headless runs then shut down deterministically. GUI runs keep the terminal world
alive after either result.

Mission failure and vehicle death are independent. Generic system failures only
produce a failed mission result. A physical Gazebo contact publishes cause
`physical_collision`; a 5 m intercept publishes `proximity_intercept`; and a
5 m interceptor-to-interceptor collision publishes `proximity_collision` for
the involved pair. Only these physical death causes can enter the force-disarm
lifecycle. A single interceptor death does not terminate the episode while
another interceptor remains. Physical evader death is settled after its disarm
and confirmed holds of all survivors. If no interceptor remains, the finite
mission ends with `no_interceptors_remaining`.

One spectator node owns the sole RViz `drone_follow` transform. Its initial
vehicle and `first_living` or cyclic `next_living` reselection policy are typed
launch parameters. Death events are matched by vehicle ID, role, and mission
epoch before changing the selection. A visualization-only adapter applies the
same selection to the Gazebo GUI camera. The `3x1` script starts on
`interceptor_0`; the `2x2` script starts on `evader_0` and prefers `evader_1`
after a successful first interception. No attacker respawn or episode reset
exists in this finite mission.

Each interceptor lidar pipeline filters returns belonging to its radar-tracked
evader before obstacle-memory integration. This prevents the moving target from
becoming a persistent environmental obstacle; it does not introduce a
prohibited zone or relax collision checks against raw physical occupancy.

## Execution Contract

`MppiTrajectoryHorizon` contains:

- sequence and obstacle/pose revisions;
- `valid_from` and `valid_until`;
- risk diagnostics plus execution mode and reason;
- optional constrained-route speed and altitude state;
- time-indexed position, velocity, acceleration, yaw, and yaw rate;
- an explicit stationary position-hold request used for mission commands, goal
  capture, cooperative yield, or route unavailability.

Offboard executes only the current fresh horizon. There is no legacy path-id,
suffix ACK, partial-replan, safe-truncation, or moving/after-hold protocol.

## Safety Boundaries

- Entering a physical occupied cell in the active map is a hard collision
  result.
- Intersecting a raw occupied cell with the swept oriented drone footprint is a
  hard physical collision result.
- Route targets and execution horizons must remain in `1.0 <= z < 32.0 m` and
  retain enough vertical stopping room under the configured acceleration and
  jerk limits.
- Risk-band exposure ranks candidates but is not physical crash detection.
- Critical and planning clearance exposure remain strong soft costs and never
  create a hold or reachability gate by themselves.
- A missing executable route produces a typed stationary hold; a newly accepted
  route atomically resumes planned execution.
- Offboard stops only when its timestamped execution horizon expires; it never
  continues an old horizon open loop.
- Gazebo physical contact is the authoritative simulated crash event.

Debug topics, RViz markers, and JSONL files never feed back into control.

## Concurrency

ROS callbacks update short latest-value state. ESDF construction runs outside
the control callback. MPPI uses persistent GPU allocations. Diagnostics are
copied into a bounded latest-value mailbox and written after the execution
horizon has been published.

In the four-vehicle intercept mission, the planner component container has one
executor thread per vehicle. CPU-heavy planner work remains bounded by the
mission-wide planner worker budget. Each MPPI engine currently launches its own
rollout kernels on an independent CUDA stream; vehicle-by-rollout fused kernels
are a separate backend optimization and are not implied by component
composition. Three tracker/guidance pairs share a second component process and
use three executor threads; simulator truth never enters that process.

The intercept launcher applies subsystem CPU affinity when the host exposes at
least four logical CPUs. Control and physics, planning and mapping, and
diagnostics receive overlapping CPU masks so latency-sensitive work retains
reserved capacity without assigning a vehicle to one core. Every mask can be
overridden, and `ENABLE_SUBSYSTEM_CPU_AFFINITY=false` restores unrestricted
scheduling.

## Current Architectural Limits

- The lattice is not incremental AD*.
- Static planning is not incremental AD* yet.
- Static mode currently plans only against canonical Occupancy3D; lidar memory
  is not fused into its 3D collision map.
- No-static perception remains 2D and intentionally has no passage semantics.
- Collision validation uses a swept oriented 3D footprint against physical raw
  occupancy. No additional artificial footprint inflation is part of the
  planning contract.
