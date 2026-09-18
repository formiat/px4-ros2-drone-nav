# Architecture

The project contains one ROS 2 package, `drone_city_nav`, plus Gazebo assets,
PX4 orchestration scripts, and container tooling.

## Runtime Data Flow

```text
Gazebo GPU lidar + PX4 pose
  -> selected 2D or 3D obstacle-memory node
  -> raw snapshot or revisioned Occupancy3D base + dirty chunks

static:
  raw Occupancy3D + precomputed chunked ESDF3D
  -> extracted local distance evidence

no-static 3D:
  revisioned observed Occupancy3D -> local occupied-distance evidence

selected raw world
  -> persistent sparse D* Lite route + certified execution geometry
  -> GPU MPPI local horizon
  -> post-update raw physical validation + atomic ExecutionPlan3D publication
  -> timestamped MppiTrajectoryHorizon
  -> mppi_offboard_node
  -> PX4 trajectory setpoints
```

`MotionState3D`, `MotionControl3D`, and `EsdfGrid3D` are controller-neutral
contracts shared by these stages. The MPPI backend aliases them instead of
owning duplicate motion or world representations.

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
- publishes the compatibility 2D `/drone_city_nav/raw_obstacle_snapshot` for
  diagnostics; production no-static navigation does not consume it;
- publishes the full atomic memory/provenance snapshot at the debug cadence;
- publishes timestamp-aligned raw lidar hit endpoints independently of
  persistent-memory integration;
- does not load or merge the static map.

`obstacle_memory_3d_node` owns the corresponding organized 3D hit/miss beam
pipeline, full-6DoF acquisition pose, sparse observed Occupancy3D, revisioned
snapshot/delta transport, and selected-spectator 3D clouds.

### `production_mppi_node`

- consumes PX4 state, the memory-status heartbeat, and immutable raw obstacle
  snapshots where required;
- terminates raw ROS memory messages at `RawWorldIngressRos3D`, which converts
  them to ROS-free world-ingress values before invoking `WorldPipeline3D`;
- loads static artifacts at composition time and transfers their
  ownership to `WorldPipeline3D`;
- derives no-static soft distance evidence from immutable sparse
  `KnownObstacleDistance3D` chunks and materializes only the controller upload
  projection;
- delegates raw-world producer admission, snapshot/status joining, delta
  reconstruction, latest-wins world scheduling, worker lifetime, and coherent
  resident-world publication to the package-private `WorldPipeline3D` owner;
- publishes latched planner-world readiness after successful ESDF activation;
- delegates static and observed ESDF build, refresh, upload, generation, and
  publication policy to `WorldPipeline3D` and its typed builders;
- delegates persistent D* Lite ownership, request scheduling, planner-worker
  lifetime, and typed validation/result delivery to the package-private
  `RoutePlanningCoordinator3D` and delegates geometric materialization and
  validation to `RouteMaterializer3D`; route compilation and activation are
  delegated to `RouteTrajectoryCompiler3D` and `RouteActivationCoordinator3D`,
  while execution retention, hold, and horizon commits cross the sole
  `ExecutionSupervisor3D` facade;
- delegates the complete planning-cycle decision to
  `PlanningCycleCoordinator3D`, route request/continuation/materialize/activate
  sequencing to `RouteLifecycleCoordinator3D`, resident route/direct/hold
  selection to `RouteExecutionSelector3D`, and controller-cycle/horizon
  assembly to `ExecutionHorizonAssembler3D`;
- delegates diagnostics queuing, worker lifetime, JSONL/error-context files,
  and coherent statistics to the package-private `NavigationDiagnosticsSink`;
- certifies route geometry, tracking-error tube, successor reserve, and suffix
  repair against exact raw-world lineage;
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

The multi-vehicle launch loads every production planner as a ROS 2 component in
one multithreaded component container. Each component retains independent
vehicle state, route lifecycle, worker pool, CUDA stream, ESDF, and MPPI nominal
controls. Sharing one process removes redundant ROS/DDS process overhead and
lets all planners use one CUDA primary context; it does not merge vehicle state
or make one vehicle's planner callbacks depend on another vehicle.

Cooperative traffic agents run in a separate multithreaded component container,
so flight intents are delivered intra-process while each vehicle retains
independent agent state. The mission referee and the simulation truth adapter
remain separate processes because they form the ground-truth data boundary.

The spectator, diagnostics mux, world visualization, and enabled lidar-debug
nodes share a diagnostics-only component container. Intra-process transport
avoids serializing spectator selection and detailed point clouds between these
components. This container remains isolated from planning, mapping, control,
and the mission referee.

## Compile-Time Runtime Boundaries

The layered domain libraries and package-private production services have
separate CMake graphs. The private graph is:

```text
src/runtime/ros  drone_city_nav_production_mppi_component
  -> src/runtime  drone_city_nav_mppi_runtime
  -> src/route_application
                  drone_city_nav_route_runtime
  -> src/world    drone_city_nav_world_runtime
  -> controller-neutral domain libraries
```

`drone_city_nav_world_runtime`, `drone_city_nav_route_runtime`, and
`drone_city_nav_mppi_runtime` are position-independent package-private static
libraries. Each exports only its build-time private include roots to the next
target. None exposes the former flat `src/` root. The world and route runtimes
cannot include MPPI, ROS, generated message, or node headers; the MPPI runtime
remains ROS-free. The production component contains only composition and ROS
adapters, links the MPPI runtime plus ROS adapters, and has no direct dependency
on `drone_city_nav_core`, CUDA, or the lower private runtime targets.

`test_navigation_dependency_contract.py` walks the transitive local header
graph of every private manifest, verifies the source-directory and include-root
layout, checks forbidden dependencies, and keeps the component source list
disjoint from all three service libraries. Every hand-written header is also
compiled as the first and only include in an independent translation unit.

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
raw compatibility grid, and stale legacy-marker cleanup. The production MPPI
markers include mission start, mission goal, persistent route, and local target.
`lidar_debug_node` writes synchronized diagnostic snapshots.
`mission_monitor_node` and `collision_crash_node` observe the mission without
participating in route selection.

## World Representation

Static production planning consumes sparse raw Occupancy3D. Its chunked
global ESDF and compiled `FreeSpaceTopology3D` share the same world fingerprint
and provide derived distance and passage evidence, but neither can override raw
occupied evidence.

No-static production planning requires revisioned observed Occupancy3D produced
from timestamped 3D lidar. A recentered distance resource may accelerate local
queries, while the persistent route graph itself is sparse and survives compatible
world revisions. The raw occupied set plus the drone's swept physical footprint is
the only hard collision boundary in both profiles.

`WorldPipeline3D` is the sole mutable owner of the production raw-world lineage
and resident-world publication. It admits producer epochs, joins memory status
with snapshot/delta payloads, reconstructs an immutable raw world, and schedules
only the newest pending revision. One publication lease linearizes the GPU ESDF
revision, `LocalWorldGeneration`, immutable `WorldSnapshot3D`, build telemetry,
and resident readers. A mixed generation fails closed instead of exposing a CPU
world paired with another GPU upload. In observed mode the service owns local
window selection, recentering, rate policy, exact-source reuse, dense
distance-transform construction, exact-parent admission, controller upload, and
immutable publication. The ROS runtime supplies one immutable pose/evidence
request and consumes typed evidence/update events. A persistent evidence-only
change publishes a new local generation over the exact existing ESDF parent,
forces planner revalidation, and performs no redundant GPU upload. Static ESDF
mode uses `StaticWorldBuilder3D` for aligned ROI selection, fingerprint-bound
cache extraction with runtime-EDT fallback, and immutable occupancy/topology/CPU
artifact assembly. The world service coalesces refreshes, rejects a superseded
base route both before construction and at commit, reuses exact CPU/GPU
resources, and issues a fresh generation for accepted proactive refreshes. An
uploader exception invalidates the resident world because the service cannot
prove whether GPU state changed before the exception.

Unknown space remains traversable without a penalty or gate. There are no
planner/prohibited inflated grids, relaxed inflation modes, escape tunnels, or
location-specific opening rules. Derived clearance can shape speed and tracking
margins, but low clearance remains executable whenever the physical swept
footprint is raw-collision-free.

## Global And Local Planning

`PersistentDStarLitePlanner3D` is the single production strategic route producer.
It searches `(x, y, z)`, incrementally repairs changed occupied evidence, and uses
the shared `FlightTimeModel3D` for anisotropic translation and bounded turn time.
`PlannerUpdate3D` admits a complete feasible incumbent independently from its
`SearchProgress3D`. A publishable incumbent can therefore be activated while a
typed `RoutePlannerSession3D` preserves the exact request and requeues bounded
D* repair and execution-time refinement until convergence or no-route.
`RoutePlanner3D`, rather than the ROS node, owns the one persistent planner and
turns an immutable search transaction plus a controller-neutral vehicle state
into a typed update. It also owns certified-future-stitch selection, route
sampling, and raw-only segment evidence; ROS logging remains an output adapter.
`RoutePlanningCoordinator3D` owns the single pending request slot and planner
worker. World-derived requests use an explicit replace-pending policy so the
worker never processes an older queued world while a newer immutable world is
available; bounded anytime continuations use keep-pending and therefore cannot
displace newer work. The coordinator validates world generation, full-3D depth,
route-generation currency, and vehicle availability and emits typed update or
rejection events without ROS dependencies.
`RouteMaterializer3D` receives one owned request containing the exact planner
transaction, candidate, active certified route, current position, and raw-world
snapshot. It owns constrained-span construction, splice-preserving geometry
optimization, derived risk annotation, optional passage decoration, and final
candidate validation. It returns a typed immutable route result plus telemetry
and fallback information; ROS logging is an outer adapter. Derived-clearance
risk annotation consumes the controller-neutral `EsdfGrid3D` world contract in
`nav_planning` and cannot acquire hard collision authority.
`RouteTrajectoryCompiler3D` receives the materialized route, exact initial
vehicle state, endpoint semantics, and exact observed raw owner as one owned
transaction. It constructs the tracking-world binding and returns the only
sealed `CompiledTrajectory3D` without ROS or resident node-state access.
The compiled trajectory contains only base geometry, canonical time, tracking
tube, and speed constraints. Immutable `RouteDecorations3D` binds optional
passage volumes, traversal identities, and cooperative assignments to that
sealed trajectory only when constructing the execution route.
`RouteActivationCoordinator3D` owns that compiler and receives the complete
planner transaction, materialized route, coherent activation snapshot, and
latency observation as one immutable request. It prepares one typed activation
artifact containing the exact execution base and an unsequenced pending draft.
Its implementation is one transaction owner composed from the pure named stages
`rebaseAndValidate`, `compile`, `assessAdmission`, `assessReplacement`,
`certify`, and `makePendingDraft`; the stages do not introduce independent
mutable services.
Its consume-and-return commit API validates a caller-locked world/objective
context and publishes only through `ExecutionSupervisor3D`; no caller can reuse
a partially committed preparation. The supervisor is the sole production owner
of `RouteExecutionManager3D`, including activation publication. The node adapter
is limited to coherent capture, lock ownership, clock access, and ROS diagnostics.
`MppiController3D` is the sole owner of the stateful CUDA engine, nominal-reseed
lifecycle, and controller-reference cache. Its owned request/result transaction
returns the exact input after assigning the reseed generation and reports
planned, stationary-hold, backend-unavailable, or backend-failure outcomes
without ROS side effects. The node holds the resident-world lease across that
transaction and adapts typed failures to logging, route-release, and fail-closed
execution revocation.

The ownership model is specified in
[`navigation_architecture_remediation.md`](navigation_architecture_remediation.md).
`ExecutionSupervisor3D` owns the only production `RouteExecutionManager3D` and
is the production facade for all pending, lease, revocation, and control-evidence
mutations. Its single typed horizon transaction selects transition,
unchanged-plan, or pending-transition publication without exposing the store.
It captures the exact authority, orders runtime admission, validates current
world/lidar/input/owner/control evidence, revalidates finite command and braking
paths against compatible newer evidence, and performs the final manager CAS.
The ROS adapter owns coherent capture, wire encoding, locks, diagnostics, and
DDS publication; it cannot call a lower-level lease commit. The commit admits
an execution input whose navigation lineage is current and whose applied-control
evidence is authoritative for the resident owner; it does not require the
exact captured tuple and never rebases a captured input after validation. The
horizon sequence is assigned inside the commit and advances only when the
commit succeeds. The manager keeps a valid route sticky and owns the resident plan
and pending successor under one lock. It publishes the resident plan together
with its typed horizon owner, exact immutable versioned input, and matching
applied-control evidence as one atomic `CommittedExecutionAuthority3D` pointer.
Pending activation is also one manager transaction: the manager validates the
captured semantic execution base, assigns the only monotonic pending-publication
sequence, seals the candidate, and occupies the pending slot under that lock.
There is no unchecked or externally sequenced pending-publication API.
Finite-path retention is also supervisor-owned: one evidence request captures
the exact current authority, rebuilds and recertifies the route continuation,
and returns an immutable prepared transition.
Raw or lifecycle invalidation can prepare only a certified emergency-braking
tail bound to that exact owner. The ROS adapter only encodes the prepared finite
horizon, invokes the supervisor transaction, publishes DDS, and reports diagnostics.
Stationary holds use the same ownership boundary. An owned request selects
resident refresh, explicit terminal transfer, or the named stationary-capture
rearm. The supervisor validates exact authority plus current world/lidar
lineage and returns a transition or exact unchanged-plan result without
mutation. The adapter commits that result only against the captured authority;
an intervening lease or control-evidence revision makes the commit stale.
Planning ticks and activation capture that pointer once; feedback, horizon
refresh, revocation, and mission capture use exact-pointer compare-and-swap
transitions, so an old plan cannot be paired with a newer lease or control
witness. Planning starts from a certified future station, splices with measured
latency and braking reserve, and repairs only invalid suffixes.

`MppiController3D` owns executable local motion and its CUDA engine continuously
warm-starts from the previous control sequence. Every integrator of the motion
model (rollouts, the CPU reference, finite path validation) sheds a speed above
the model's caps at least as fast as the maximum deceleration allows, so an
inherited excess above the sensor-braking envelope is braked away instead of
carried along the horizon, and rollouts pay for any excess that remains. While
a followed route is blocked ahead by the persistent raw world and its
replacement is still being searched, the speed policy limits the speed so the
vehicle can stop before the blocked station. Latest raw lidar evidence
validates the finite swept path before publication, independently of strategic
planner reuse.

## Mission Layer

Point-to-point navigation remains the default mission and uses the configured
fixed position objective and terminal goal capture. The goal is captured once
the vehicle rests inside the capture radius (`mission_goal_capture_radius_m`):
the goal hold pins the vehicle's rest position rather than the goal coordinate,
so the mission never waits for the controller to creep onto an exact point,
while the route target that reached the goal still has to match it exactly. A
goal hold may also replace a resident finite execution before its lease ends
once the remaining lease commands nothing but rest at the hold position. The
capture is acknowledged only while the vehicle stays inside the radius, and a
captured goal flies no route, so the capture releases again when the vehicle
leaves the radius: the route takes it back inside, where resting captures the
goal once more.

The finite cooperative traffic mission runs one complete navigation stack per
vehicle: a separate PX4 DDS namespace, planner, offboard node, lidar safety
input, cooperative agent, and destruction state. No-static navigation maintains
independent persistent lidar memory for every vehicle. The mission has no
ground-truth data path into navigation:

```text
Gazebo Pose_V -> simulation truth adapter -> typed physical vehicle states
typed physical truth -> cooperative referee -> outcome and settlement only
vehicle[i] state + horizon[i] -> agent[i] -> FlightIntent[i]
FlightIntent[*] -> agent[i] -> maneuver command and passage state for vehicle[i]
```

`cooperative_traffic_urban_scenario.json` is the only hand-written source for
vehicle map starts and goals. Its Gazebo spawns are derived from the world's
`map_to_sdf` transform by both the runner and launch tooling. The simulation
truth adapter converts physical Gazebo poses back to map coordinates and checks
them against navigation states over consecutive samples. A persistent mismatch
prevents mission start and requests hold for already airborne vehicles.

Vehicles coordinate only through the shared flight-intent channel. Each agent
publishes its own predicted trajectory, reads the intents of its peers, and
issues its own maneuver command and passage state; no node arbitrates for
another vehicle. The referee owns goals, terminal outcome, and hold or disarm
settlement, and it verifies through the ROS graph that it is the only subscriber
of typed physical truth.

Visualization remains outside this control boundary. Each planner publishes a
namespaced lightweight path. `multi_vehicle_diagnostics_mux_node` subscribes to
the latched `SpectatorTarget`, clears the previous selected layers, and
republishes the selected planner markers, execution horizon, status, memory
cloud, and lidar-debug clouds on stable RViz topics. Lightweight per-vehicle
paths remain visible concurrently. Selector-gated lidar debug nodes retain pose
and latest-map context for every scenario vehicle, but only the current
spectator projects scans, integrates diagnostic memory, writes a bounded startup
snapshot, and publishes detailed lidar layers. Non-selected vehicles continue
publishing their latest physical lidar returns for finite-path validation.
These visualization-only nodes share the diagnostics component container; their
outputs never participate in route selection or vehicle control.

One spectator node owns the sole RViz `drone_follow` transform. Its initial
vehicle and `first_living` or cyclic `next_living` reselection policy are typed
launch parameters. Death events are matched by vehicle ID, role, and mission
epoch before changing the selection. A visualization-only adapter applies the
same selection to the Gazebo GUI camera.

Mission failure and vehicle death are independent. Generic system failures only
produce a failed mission result. A physical Gazebo contact publishes cause
`physical_collision`; a 5 m separation between two vehicles publishes
`proximity_collision` for the involved pair. Only these physical death causes
can enter the force-disarm lifecycle. Every death event includes a stable
`vehicle_id`, so role alone never identifies one of several vehicles. The
mission records its result only after every destroyed vehicle is settled and
every survivor confirms a stationary position hold. Headless runs then shut down
deterministically; GUI runs keep the terminal world alive.

Each vehicle's lidar pipeline filters returns belonging to peers whose flight
intents it receives, before obstacle-memory integration. This prevents a moving
peer from becoming a persistent environmental obstacle; it does not introduce a
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

Horizon supersession is monotonic. Offboard acknowledges every accepted horizon
through `MppiControlFeedback` with the horizon sequence it executes. A new plan
replaces the resident owner when that owner is witnessed by applied-control
evidence, when offboard has acknowledged the owner or its predecessor, or when
the configured acknowledgement grace has elapsed since publication; only a
stale acknowledgement after the grace rejects. A rejected commit therefore never
waits for the owner to expire, and a tick that cannot publish keeps the
resident planned owner explicitly as a resident-owner continuation instead of
reporting a hold it did not publish.

## Safety Boundaries

- Entering a physical occupied cell in the active map is a hard collision
  result.
- Intersecting a raw occupied cell with the swept oriented drone footprint is a
  hard physical collision result.
- Route targets and execution horizons must remain in `1.0 <= z < 32.0 m` and
  retain enough vertical stopping room under the configured acceleration and
  jerk limits.
- Sensor-limited motion must fit evidence-age and reaction latency, the shared
  jerk-limited stopping distance along the direction of motion, and physical
  margin inside the guaranteed lidar detection range. The worst direction's
  cap applies to the full translational velocity norm in planning and control;
  the reference speed applies the direction the vehicle moves in. Stale
  required evidence fails closed.
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

The world pipeline owns one stoppable worker. In observed mode its deferred
scheduler retains only the newest immutable raw revision and records skipped
lineage, emits persistent evidence changes before expensive construction, and
then executes the typed `ObservedWorldBuilder3D` transaction. In static mode it
executes the typed `StaticWorldBuilder3D` transaction and coalesces refresh
requests with latest-wins semantics. Producer ingestion, worker scheduling, resident
publication, and build statistics have independent private synchronization.
Publication and resident leases share one mutex, so a planner cannot validate
one generation while another generation is being installed. Reused builds
require the exact immutable parent captured before construction. World builds
run on a dedicated world worker pool sized by `world_worker_count`; the planner
worker pool serves only D* Lite continuations and route work, so neither can
starve the other.
Stop joins outside the lifecycle mutex and processing exceptions are contained
at the service boundary.

In the cooperative traffic mission, the planner component container has one
executor thread per vehicle. CPU-heavy planner work remains bounded by the
mission-wide planner worker budget. Each MPPI engine currently launches its own
rollout kernels on an independent CUDA stream; vehicle-by-rollout fused kernels
are a separate backend optimization and are not implied by component
composition. The cooperative agents share a second component process; simulator
truth never enters that process.

The multi-vehicle launcher applies subsystem CPU affinity when the host exposes at
least four logical CPUs. Control and physics, planning and mapping, and
diagnostics receive overlapping CPU masks so latency-sensitive work retains
reserved capacity without assigning a vehicle to one core. Every mask can be
overridden, and `ENABLE_SUBSYSTEM_CPU_AFFINITY=false` restores unrestricted
scheduling.

## Current Architectural Limits

- The adaptive persistent D* Lite graph uses power-of-two world-aligned
  resolution levels. It is not an octree and deliberately keeps the complete
  minimum-resolution 26-connected lattice as its reachability baseline.
- Static mode currently plans only against raw Occupancy3D; lidar memory
  is not fused into its 3D collision map.
- No-static production navigation requires revisioned 3D-lidar Occupancy3D and
  has no open-space-versus-passage partition.
- Collision validation uses a swept oriented 3D footprint against physical raw
  occupancy. No additional artificial footprint inflation is part of the
  planning contract.
- `WorldPipeline3D` owns raw reconstruction, static and observed ESDF build
  policy, refresh/upload transactions, and publication.
  `RoutePlanningCoordinator3D` owns persistent planning request scheduling and
  worker lifecycle, and `RouteMaterializer3D` owns geometric materialization and
  candidate validation. `RouteTrajectoryCompiler3D` owns exact-state trajectory
  compilation. Activation coordination and controller ownership are extracted;
  finite-path retention, hold preparation, and atomic horizon validation/commit
  are behind `ExecutionSupervisor3D`.
