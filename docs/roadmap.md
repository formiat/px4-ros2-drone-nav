# Gazebo Roadmap

## Dependency Model

Roadmap numbering identifies project milestones; it is not always a strict
execution order. The dependency annotations below use three meanings:

- **hard prerequisite**: implementation cannot begin meaningfully before the
  prerequisite contract exists;
- **validation prerequisite**: the feature can be developed independently, but
  its complete mission-level acceptance requires the prerequisite;
- **independent recurring workstream**: work may run in parallel with any
  milestone and should be repeated as the architecture evolves.

## 1. Interceptor Drone (Completed)

Implement an autonomous interceptor drone capable of pursuing an attacking
drone using external target information.

The current foundation launches three interceptors and one attacking drone in
isolated PX4 and ROS namespaces. The attacker flies from a fixed start to a
fixed goal. Every interceptor receives an independent radar-derived target
track, predicts its motion, and continuously updates a tracking objective
without entering terminal goal hold. A separation of 5 m or less destroys the
capturing pair and records a successful intercept outcome.

The interceptor mission, radar-derived tracking, predictive guidance, and
physical interception lifecycle have been implemented and validated in
repeated mission runs.

## 2. Radar Measurement Simulation (Completed)

Replace direct access to the target's ground-truth coordinates with a more
realistic radar measurement model.

The radar interface will provide only:

- range;
- bearing;
- elevation;
- radial velocity.

The initial implementation uses ideal measurements without noise,
interference, latency, or measurement errors. Measurement cadence follows a
deterministic correlated random walk between 0.1 s and 3.0 s. Guidance continues
at 20 Hz by coasting the latest target track between scans. Swept raw-clear
visibility of the current target estimate commands an immediate scan and 20 Hz
track mode at any range; target occlusion restores variable search cadence.

The physical radar will not be simulated. Only the radar measurement interface
and its integration with the interceptor are implemented.

Each pursuit data path is split into a simulation-truth adapter, mission
referee, radar simulator, target tracker, and interceptor guidance node. Gazebo
model poses are converted to typed physical truth once; only the referee and
radar simulators may subscribe to the target's typed truth. The
interceptor-facing `RadarScan` carries
range, azimuth, elevation, and relative radial velocity, but no absolute target
position, velocity, or simulator entity identity. The first detection supports
direct pursuit; subsequent variable-dt updates estimate Cartesian velocity for
predictive guidance. Runtime graph validation and source-contract tests enforce
this boundary.

## 3. Target Motion Prediction (Completed)

Implement target trajectory prediction. The initial model will intentionally
remain simple:

- use the latest known target position;
- use the latest known velocity vector;
- extrapolate the trajectory several seconds ahead;
- intercept the predicted future position instead of chasing the current
  position.

The implementation solves the constant-velocity intercept equation using the
configured interceptor speed. The solution is capped at 15 s. When the
interceptor is ahead and inside the target corridor, the horizon is capped at
1 s. The transition uses spatial hysteresis and a smoothed prediction horizon.
Slow or invalid target velocity falls back to the observed target position.

Prediction starts at the measurement timestamp, so telemetry age is included
in extrapolation. Interceptor guidance publishes a typed tracking objective from
the radar-derived target track without reading a map. The planner clips the
prediction segment at the first raw occupied cell and uses the last raw-free
sample. It does not search for a nearest free point and does not add inflation
or prohibited regions. Vertical prediction applies bounded deceleration until
the target stops climbing or descending and clamps altitude to the configured
flight envelope instead of rejecting the objective. Swept visibility of the
current target activates direct moving-target MPPI pursuit. If only the full
prediction is blocked, the planner shortens the lead toward the current target;
only current-target occlusion returns execution to ordinary global planning.
RViz and JSONL diagnostics expose observed, coasted current, predicted, and
resolved target points together with visibility, prediction-path clearance,
closing speed, commanded speed, active speed limiter, and radar age.

The initial target-motion prediction scope is complete. It now consumes the
target track produced by the radar pipeline described in section 2.

## 4. Multiple Interceptors Versus One Attacker (Completed)

The finite scenario now runs three interceptor drones against one attacking
drone. They start in three different city corners and own independent PX4,
navigation, radar, tracker, and guidance pipelines. All three use the measured
motion direction by default. Optional long-range directional hypotheses add
`-45` and `+45` degree alternatives; the offsets converge to zero near the
attacker and cannot move the predicted point more than 70 m laterally.

The first interceptor within 5 m of the attacker destroys that pair. Surviving
interceptors receive a typed hold objective and enter confirmed stationary
position hold.
Interceptor-to-interceptor separation within 5 m is accepted as collateral
damage; only that pair is destroyed and the pursuit continues. The spectator
camera starts on `interceptor_0` and uses the configurable living-vehicle
reselection lifecycle after a typed death event.

This stage deliberately contains one attacker and one episode. Attacker
despawn, respawn, and an endless campaign remain future work and are not part of
the current implementation.

## 5. Multiple Interceptors Versus Multiple Attackers (Completed)

The first finite scenario launches two attacking drones from the short city
side farthest from their shared corner destination. One attacker starts at the
corner and the other starts one block inward along that side. Two interceptors
start on the opposite short side, next to the destination.
The existing 3x1 `intercept` entry point remains unchanged; dedicated
`sim_multi_intercept_gui.sh` and `sim_multi_intercept_headless.sh` wrappers load
the 2x2 scenario through the same generic N x M launch pipeline.

Each interceptor receives an independent ideal radar scan containing relative
measurements for every attacker and maintains one radar-derived track per
detection. A typed assignment coordinator minimizes estimated intercept time,
covers distinct active targets where possible, and applies hold time,
improvement threshold, and confirmation hysteresis before changing an existing
assignment. Terminal attackers are removed immediately and the remaining fleet
is reassigned without restarting navigation.

The multi-target referee records one first terminal outcome for every attacker
and preserves physical 5 m proximity, typed death, disarm, and survivor-hold
settlement. Collision avoidance remains disabled, and mid-air collisions are
treated as acceptable collateral damage. The finite mission does not respawn
attackers and does not implement an endless campaign.

The `2x2` spectator starts on `evader_0` and uses cyclic `next_living`
reselection, preferring `evader_1` after the first attacker's destruction.

## 6. Cooperative Multi-Drone Air Traffic (Completed)

The finite cooperative scenario launches four autonomous civilian drones from
the city corners. Each vehicle has an independent point-to-point mission to the
opposite corner, starts at the same altitude, and owns an isolated PX4,
navigation, mapping, and MPPI pipeline. Fixed cruise-altitude layers are not
preassigned.

The drones exchange typed, bounded-validity flight intents at 20 Hz. Each intent
contains the vehicle's physical footprint, current position and velocity,
predicted MPPI trajectory, maneuver state, and constrained-passage use. Every
vehicle independently validates peer freshness, predicts continuous closest
approach over the shared horizon, and selects deterministic complementary
vertical or lateral maneuvers. Latching and release hysteresis prevent rapid
maneuver flapping.

Peer separation is implemented as a strong soft MPPI cost and preferred
acceleration direction. It does not create prohibited grids, inflated obstacles,
or hard exclusion volumes. Static passages expose capacity derived from
raw-validated lane geometry: opposite traffic may use separate lanes when the
physical width permits it, while exclusive or conflicting use is resolved by
deterministic right-of-way and a route-safe hold before entry.

In no-static mode, cooperative peer returns are filtered from persistent lidar
memory and from direct raw obstacle-path validation. A dedicated
referee verifies coordinate readiness, physical minimum separation, goal
arrival, stationary hold, vehicle destruction, and building collisions. The
supported headless contract requires every vehicle to settle at its own goal
without physical loss. Both static-map and no-static-map scenarios have passed
this full mission validation.

## 6.1. Non-Cooperative Collision Avoidance in Interception Missions (Completed)

Every attacker carries an independent high-rate simulated airborne radar. It
reports anonymous relative detections of all aircraft within physical range and
line of sight; it does not expose roles, mission assignments, global routes, or
the intent communication channel used by cooperative civilian traffic. A local variable-time
tracker converts those measurements into anonymous position and velocity tracks.

The attacker evaluates current separation and continuous closest approach for
every fresh track. A strong finite trajectory cost applies below 10 m, with a
lower anticipation cost between 10 m and 20 m and additional time-to-collision
weighting. The cost covers the full MPPI rollout. On entry into a strong threat,
a raw-validated maximin acquisition selects among route-directed, lateral,
vertical, speed-reduction, and reverse candidates; normal route progress breaks
ties.
Lifecycle hysteresis and one-time entry and release reseeds prevent maneuver
flapping.

Physical obstacle validity remains stronger than aircraft separation. Avoidance
cannot select a trajectory that intersects raw occupancy or violates the flight
envelope. Separation is still a soft objective: the implementation does
not create prohibited grids, inflated obstacles, hard exclusion volumes, or an
equivalent mandatory boundary around another drone. Physical interception
therefore remains possible.

Headless validation verifies the radar-only data boundary, fresh independent
tracks for every attacker, observable avoidance activity or cost, physical
mission settlement, and zero building collisions. The `3x1` and `2x2` scenarios
have passed this contract with and without a static map.

## 7. Advanced 3D Passages (Completed)

Static passage planning now uses a separately versioned, map-fingerprint-bound
`FreeSpaceTopology3D`. A chunked C++ compiler consumes raw `Occupancy3D` and the
matching `ESDF3D`, classifies footprint-feasible clearance topology, extracts
arbitrarily oriented portal voxel patches, and skeletonizes constrained free
space into sparse medial passage segments. Roof presence, axis-aligned portal
heuristics, rectangular authoritative openings, and pairwise portal edges are no
longer part of the contract.

Global planning resolves route-specific `PassageTraversal` objects lazily over
the sparse graph and caches them. The lattice uses a spatial index rather than
scanning every passage edge at every state. After route selection, raw occupancy
queries generate a varying 3D cross-section envelope along the traversal, so
sloped, vertical, curved, and changing-height routes do not collapse to one
`min_z/max_z` intersection. MPPI and route activation still perform final raw
swept-footprint validation; derived topology never creates a hard obstacle.

Region, portal, segment, traversal, and cooperative conflict-resource IDs are
distinct strong types. Cooperative passage reservations are scoped to shared
sparse segments instead of locking an entire free-space region. Deterministic
fixtures cover a sloped tunnel, vertical shaft, arch, curved tunnel, T and X
junctions, and a wide-hangar negative case.

The compiler has also produced strict artifacts for the compact fixture and the
Urban, Cave, and Finals release maps. This closes static extraction, planning,
execution, and cooperative use of the optional sparse topology accelerator. It
does not claim production mission integration in those external environments;
that remains item 9. Item 8 does not reproduce this open-versus-passage
classification online: observed navigation uses one continuous free-space
domain.

## 8. 3D Perception And Raw-World Foundation (Completed)

**Type:** ordered implementation stage.

**Hard prerequisite:** item 7.

Provide production 3D perception and a revisioned raw world without requiring a
preloaded static map. A street, room, tunnel, cave, shaft, and continuously
bounded underground network are one physical domain; they do not trigger
separate open-space and passage lifecycles.

The production pipeline is:

1. organized Gazebo 3D lidar hit and miss beams;
2. timestamp alignment and full-6DoF acquisition-pose resolution;
3. ray integration into revisioned `unknown/free/occupied` `Occupancy3D`;
4. base snapshots plus dirty-chunk transport;
5. chunked immutable raw snapshots for planning and exact swept validation;
6. latest-lidar evidence for bounded final execution revalidation;
7. MPPI and finite raw-safe PX4 execution.

Raw occupancy has exactly three evidence labels: `Occupied`, `Free`, and
`Unknown`. Only confirmed `Occupied` geometry is a hard spatial prohibition.
Relabeling any non-occupied voxel between `Free` and `Unknown`, with occupied
geometry unchanged, must not alter traversability or base cost. Clearance,
observability, and derived distance caches are deliberately outside the raw
collision contract and are completed by item 12.

No-static production navigation uses the 3D lidar profile. A static map may be
used without lidar, but there is no 2D-lidar production fallback for autonomous
free-space navigation. RViz displays the selected spectator's latest 3D returns
with queue depth one and its rate-limited accumulated occupied voxels; it does
not render every vehicle's full diagnostic clouds.

This item owns the sensor-to-raw-world boundary, not strategic route selection.
Its deterministic tests prove timestamped hit and miss integration, revisioned
dirty-chunk transport, immutable snapshot identity, exact physical-footprint
queries, latest-lidar admission, and display provenance. End-to-end navigation
acceptance belongs to item 12.

Complex environments introduced after Manhattan use no-static 3D lidar until
item 11 provides validated static 3D maps. After that they support static runs
with no lidar or 3D lidar and no-static runs with 3D lidar only.

## 9. Large-Scale Realistic City And Full-Mission Validation

**Type:** integration and validation milestone.

**Hard prerequisites:** items 8 and 12 for no-static autonomous traversal;
item 11 for full static-map validation.

Find a suitably licensed high-quality city environment or build a new one for
the project. The location should be substantially larger and more visually and
geometrically varied than the current regular test city, with realistic street
layouts, building shapes, heights, materials, and urban topology.

Where practical, include complex physically traversable 3D free-space
structures such as multi-turn tunnels, junctions, shafts, and entrances at
different altitudes. Imported visual assets must have explicit provenance and a
license compatible with the repository. Rendering meshes, collision geometry,
lidar-visible surfaces, static occupancy, and generated planning artifacts must
remain aligned instead of becoming separate hand-maintained versions of the
world.

Use the new location as a full-system validation environment rather than only a
visual showcase. Re-run every supported point-to-point, static-map, no-static,
constrained-3D-traversal, single-target interception, multi-target interception,
and cooperative-traffic mission that exists when this stage begins. Validation
should cover multiple start and goal placements and repeated headless runs, and
must preserve physical outcome checks, zero tolerance for building collisions,
planner and controller diagnostics, real-time-factor monitoring, and measured
CPU/GPU timing.

This stage is complete only when the mission suite succeeds on the new city
without scenario-specific route scripts or geometry exceptions. One successful
3D-lidar exploration flight is integration evidence, not completion: acceptance
requires repeated representative point-to-point and cooperative runs, plus the
other supported mission types claimed by this milestone. Static-map acceptance
is performed after item 11 provides validated maps.

## 10. Architectural Review And Optimization

**Type:** independent recurring workstream.

**Dependencies:** none; this item is not part of the ordered execution sequence.

Perform systematic architecture reviews throughout development and repeat a
full review after the navigation, passage, and large-environment mission
contracts are established. Each review must trace the end-to-end data and
execution paths across sensing, mapping, topology, planning, MPPI, PX4 control,
cooperative coordination, simulation, and diagnostics.

Use repeatable representative missions to measure CPU, GPU, memory, ROS/DDS
transport, simulator real-time factor, planning latency, control deadline
misses, and scaling with vehicle count. Optimize confirmed bottlenecks while
preserving typed contracts, raw-occupancy safety validation, and observable
mission outcomes. Prefer removing duplicated work, stale data transport, and
unnecessary process or synchronization overhead over increasing worker counts
or weakening safety margins.

This stage also records architectural debt, defines ownership and lifetime
boundaries for shared resources, and converts validated optimizations into
regression benchmarks. It is complete when the supported mission suite has
measured performance budgets, reproducible baselines, and documented scaling
limits for both static-map and 3D-sensing configurations.

## 11. Valid 3D Static Maps For New Environments

**Type:** dependent implementation and validation stage.

**Hard prerequisites:** items 8 and 12 for the primary autonomous-survey
acquisition path.

Create a valid static map for every new complex environment. Here, quality
means geometrically correct, physically valid, and aligned with the real
collision environment: every real obstacle relevant to the aircraft footprint
must be represented. It does not require unnecessarily high visual or voxel
detail.

Every new-environment static map must be three-dimensional. Two-dimensional
maps are insufficient for multi-level geometry, tunnels, shafts, windows,
doors, and other traversable 3D passages.

The primary acquisition path uses item 8's production 3D lidar and item 12's
incremental exploration backend to survey every reachable part of an
environment, then persists the resulting validated obstacle memory as the
environment's static-map artifact. Direct generation from collision geometry
may remain as a secondary generation or cross-validation tool. Every artifact
must be versioned with the environment collision geometry, source provenance,
coordinate transform, resolution, coverage evidence, and validation result.

This stage is complete when every supported new environment has a reproducible
3D static-map generation or acquisition path and that map passes coverage,
alignment, and raw-collision validation against its physical world.

## 12. Persistent Full-3D Strategic Navigation

**Type:** dependent implementation stage, in progress.

**Hard prerequisite:** item 8, which is complete.

Finish one unified 3D navigation architecture for static and no-static maps,
open cities, rooms, tunnels, caves, shafts, and labyrinths. There is no
environment-specific planner mode and no location-specific knowledge of world
names, starts, goals, opening coordinates, or opening altitudes. Static maps may
initialize the world completely; no-static maps grow it incrementally from
revisioned 3D-lidar evidence. Both feed the same planner, trajectory compiler,
route owner, MPPI, and PX4 execution contracts.

### Raw World And Distance Evidence

The planner consumes a global, sparse, world-fixed `RawOccupancy3D` snapshot.
Confirmed `Occupied` cells and the physical flight envelope are its only hard
spatial constraints. `Free` and `Unknown` have identical traversability and base
cost. A production strict-known switch, observation-frontier stop policy,
outside-local-grid rejection, inflated hard grid, or information-gain transit
preference would violate this invariant and must not exist.

Maintain a sparse incremental `KnownObstacleDistance3D` cache over confirmed
occupied cells. It supplies optional soft clearance and controller evidence in
both free and unknown volume. Missing or out-of-cache distance evidence is
neutral. The hard swept-footprint query always uses raw occupancy and the
physical vehicle hull. Tracking uncertainty is a speed-dependent tube; it may
reduce speed in a narrow passage but must not enlarge the hard planning hull by
a fixed margin.

Unknown-space safety follows one sensor-and-braking inequality across the whole
world:

```text
speed * total_latency + stopping_distance + physical_margin
  <= guaranteed_lidar_detection_range
```

Stale sensing prevents publication of new motion. Fresh sensing applies the
same speed and admission rules to free and unknown space.

The production speed policy now enforces this inequality directly. Total
latency is the configured maximum age of timestamp-aligned lidar evidence plus
control reaction latency. Its shared jerk-limited stopping model starts from
the worst configured forward 3D acceleration and uses the weaker guaranteed
horizontal or vertical deceleration. Configuration contract tests bind the
guaranteed range to both the 3D lidar model and obstacle-memory range, while
runtime diagnostics publish every distance term and the remaining reserve. The
solved limit is a hard complete-translational-speed bound in host and CUDA
dynamics and the shared strategic/route ETA model. Measured overspeed requests
braking while retaining physically continuous inherited velocity. The
organized lidar covers the complete vertical sphere, including pure climb and
descent directions, instead of leaving polar blind cones outside the range
contract.

### Persistent Strategic Planner

Use one persistent sparse D* Lite planner over an adaptive world-fixed
26-connected 3D lattice. It retains search state across a moving start and raw
occupied updates, repairs only affected vertices, and preserves the incumbent
mission route while bounded repair is incomplete. Lazy exact swept-footprint
validation is authoritative; any-angle shortcutting may reduce lattice artifacts
only after the shortcut passes the same raw validation and the shared complete
path-time profile proves that predicted execution time does not increase.

The planner has one route-producing pipeline with cooperating raw-connectivity,
feasibility, and execution-time search sessions. Persistent D* Lite owns
raw-safe connectivity, incremental repair, and admissible anisotropic
translation-time labels. A resumable direction-labelled refinement uses those
labels inside the same planner to minimize predicted execution time; its
transition cost includes jerk-limited braking and restart plus physically
bounded stationary yaw whenever the compiler's 3D tangent threshold requires a
`StopAndTurn`.

The planner contract must report a publishable incumbent independently from
search progress. A first feasible raw-safe route may be admitted without being
mistaken for convergence, and the same search session must continue until
refinement converges, reports no route, or is invalidated. Goal-altitude-first
queue ordering is forbidden because it can exhaust one horizontal layer before
considering a required climb or descent. An occupied-cell removal disables any
retained D* label that could overestimate a newly opened alternative and falls
back to the geometric admissible heuristic until the next full search lineage.

The strategic objective is predicted 3D execution time to the mission goal.
Preparatory climb, descent, lateral detour, and justified backtracking are valid
motions even when they temporarily reduce Euclidean goal progress. Spatial
planning, curve compilation, and ranking share one 3D station/time model with
horizontal and vertical velocity, acceleration, jerk, and stop-turn effects.
There is no XY-only progress, frontier rank, or controller fallback metric.

Topology and dead-end memory may supply stable macro-edge heuristics to the
persistent planner. They never run as a competing route-producing pipeline,
never own execution, and never reward unknown or unvisited space during normal
mission transit. The legacy direct-versus-topology-versus-frontier arbitration
and the 2D production navigation branch are removed after migration.

### Active Intent, Route Owner, And Execution Plan

One `ActiveIntent3D` owns a mission objective and its persistent strategic route.
World revisions, newer candidates, or a slightly better score cannot replace
it. Ownership ends only for mission-epoch change, completion, exact fresh-raw
invalidation, an external safety constraint, or confirmed sustained physical
inability to follow the route. A continuity-preserving successor may improve the
route only after full certification and hysteresis; an extension does not change
the active intent.

One `ExecutionSupervisor3D` owns the sole production
`RouteExecutionManager3D`, which owns immutable route chunks, monotonic progress,
the pending and active route, an overlapping future-station successor, and
atomic suffix repair. Production mutations cross one typed supervisor lease
transaction; the manager is not exposed to the ROS node. Every admitted
non-terminal route has certified remaining reserve of at least:

```text
stopping_distance + speed * p99_successor_latency + certified_overlap
```

Background planning starts early enough to preserve that reserve. A successor
is built from a future station and appended after the frozen valid suffix. A raw
collision keeps the valid prefix and replaces only the affected suffix. If
repair misses the braking boundary, the certified braking plan becomes the
execution owner; the owner is never cleared merely because repair or a compare-
and-swap attempt failed.

Execution state is a tagged variant for following, direct tracking, braking,
stationary hold, awaiting successor, and revocation. A pure reducer owns its
transitions so conflicting route, direct, braking, and hold combinations cannot
be constructed.

Publish one immutable atomic `CommittedExecutionAuthority3D` containing the
execution plan, owner identity, exact versioned execution input, and
applied-control evidence. The plan contains mission and route identities,
geometry revision, progress, finite nominal horizon, certified braking fallback,
raw-validation certificate, and all required evidence revisions. MPPI may
refresh its short horizon at control rate without changing route ownership. A
progress projection mismatch or snapshot conflict requests a fresh read and
retry; only an exact raw collision result may report `raw_collision`.

### Implementation Order And Cleanup

The detailed contracts and requirement checklist are maintained in
[`navigation_architecture_remediation.md`](navigation_architecture_remediation.md).
The implementation order is:

1. Separate planner incumbent publication from convergence, continue refinement,
   and introduce explicit search/coordinator boundaries.
2. Introduce the single raw-occupied collision oracle and immutable world and
   route-stage artifacts; remove derived-ESDF hard-collision authority and route
   state from the resident world.
3. Move motion, dynamics, and risk types below route and MPPI, then compile once
   from the exact initial vehicle state into a sealed `CompiledTrajectory3D`.
4. Replace phase plus optionals with a tagged execution variant, one pure
   reducer, one pending/active `RouteExecutionManager3D`, and one atomic
   committed execution authority.
5. Extract world, planning, trajectory, execution, control, and diagnostics
   services from `ProductionMppiNode` so the ROS node becomes a composition root.
6. Enforce the resulting dependency graph with internal CMake targets, register
   every production-relevant test source, replace source-text transaction guards
   with executable tests, and remove legacy lifecycle code and terminology.

The current implementation includes the persistent adaptive lattice,
direction-labelled execution-time refinement, immutable sparse
`KnownObstacleDistance3D` cache, immutable world/search/materialization
artifacts, the sealed exact-state `CompiledTrajectory3D`, the tagged execution
variant and reducer, one atomic `CommittedExecutionAuthority3D`, the enforced
eight-layer CMake DAG, the controller-neutral `EsdfGrid3D` world descriptor,
complete registration of production-relevant GTests, split execution
model/certification/transition/store contracts, a package-private hand-written
C++ API, independent compilation of every hand-written header, and an
independently tested `NavigationDiagnosticsSink` that owns its mailbox, worker,
files, error context, and runtime statistics. The package-private
`WorldPipeline3D` now owns raw producer admission and joining, incremental
reconstruction, latest-wins scheduling, worker lifetime, generation issuance,
and coherent immutable resident-world publication. Observed mode additionally
owns local-window/recenter policy, audit and rate decisions, full/incremental/
reused CPU construction, exact-parent admission, GPU upload, and publication;
the node supplies immutable pose/evidence input and consumes typed events.
Static mode owns ROI and refresh policy, cache/runtime-EDT construction,
occupancy/topology/CPU artifact ownership, GPU residency, early and late
base-route supersession, generation issuance, and fail-closed publication; the
node supplies immutable navigation/objective and commit contexts and consumes a
typed result for route-search coordination.
The first planning-service boundary is also concrete: `RoutePlanner3D` owns the
single persistent planner and exact continuation sessions, selects certified
future stitches, and emits typed sampled candidates with raw-only segment
evidence from immutable inputs. `RoutePlanningCoordinator3D` owns the one-slot
request scheduler, explicit newest-world replacement versus keep-pending
policy, worker lifecycle, request validation, failure isolation, and typed
update/rejection delivery. Its direct tests cover overload, displaced lifecycle
transfer, rejection reasons, callback failure, reentrant continuations, stop,
and restart. `RouteMaterializer3D` owns one exact request through geometric
optimization, splice preservation, derived risk annotation, optional passage
decoration, and final candidate validation; typed fallback information is
logged only by the ROS adapter. Its soft risk annotation now lives in
`nav_planning` and consumes the neutral ESDF world descriptor; the legacy MPPI
adapter and controller-layer dependency have been removed.
`RouteTrajectoryCompiler3D` now owns exact-state compilation and
the observed/static tracking-world binding behind one non-ROS request/result
boundary. `RouteActivationCoordinator3D` owns that compiler and the complete
compile/admit/certify/splice pipeline behind one immutable request and prepared
activation result. Its consume-and-return commit takes that preparation by
value plus a caller-locked currentness context and can publish only through the
execution supervisor; the remaining ROS adapter owns only coherent capture,
locks, clock access, and diagnostics. `MppiController3D` now owns the sole CUDA
engine, nominal-reseed lifecycle, and controller-reference cache behind one
owned request/result transaction; the node retains only the resident-world
lease and ROS/fail-closed adaptations. `ExecutionSupervisor3D` now owns the sole
production execution manager; activation, pending recovery, lease publication,
revocation, and control evidence cross its typed facade. Route and
direct-tracking retention now enter through one owned request: the supervisor
captures the resident authority, validates current world/evidence, and returns
one certified transition without mutating the store. Raw invalidation can only
produce an exact-owner emergency-braking tail. Hold preparation also enters
through one owned request: the supervisor captures the exact authority,
validates current raw/lidar lineage, and prepares resident refresh, terminal
transfer, or the named revoked-owner stationary-capture rearm without mutating
the store. Horizon publication now crosses one owned `commitHorizon`
transaction. The supervisor captures the exact authority, orders vehicle/raw/
revocation/objective/navigation/offboard admission, validates the exact input,
derives raw obligation and producer from the plan, validates policy, lidar,
owner, and previous-control witness, revalidates command and
braking paths against compatible newer evidence, and performs the final manager
CAS. The ROS adapter retains only locked runtime capture, optional late rebase,
wire encoding, diagnostics, and DDS publication; the old public low-level lease
commit has been removed. Pending publication is already one manager-owned transaction:
the manager validates the semantic execution base, assigns the sole monotonic
sequence, seals the candidate, and occupies the pending slot under one lock.
Direct tests replace the former raw-world source-order guards with executable
overload, quarantine, full/incremental/reuse, throttling, exact-parent,
publication, upload-rejection/exception fail-closed behavior, and stop
transactions, plus static build/reuse, route supersession, generation failure,
refresh-coalescing, persistent-session transactions, and route-request
scheduling, materialization, and exact-snapshot activation preparation and
commit, plus exact stationary-hold output, controller reseed ownership,
concurrent controller serialization, backend-unavailable classification, and
reference-cache ownership, including invalid and superseded controller-world
currentness. Direct execution-transition tests also carry the raw-invalidation
emergency-brake contract, so the former syntax-specific ternary assertions have
been removed. Direct supervisor tests now cover all lease kinds, exact pending
consumption, stale CAS, control-evidence replacement, and concurrent
single-winner publication; the manager pending-clear source parser has been
removed. Direct supervisor retention tests cover route/direct continuation,
raw-invalidation braking, stale lifecycle ownership, missing evidence, and the
subsequent lease commit; policy no longer lives in the ROS adapter. The direct
hold suite covers terminal transfer, exact unchanged replay, refreshed-input
replacement, stationary-capture rearm, stale evidence, and stale-authority CAS.
Hold certification and reducer calls no longer live in the ROS adapter, and its
former source-order checks have become architectural bans. Direct horizon tests
cover transition, unchanged and pending commits, compatible evidence
revalidation, typed runtime failure, exact navigation/control identity,
stationary-rearm admission, and concurrent single-winner CAS. Node source checks
now enforce the adapter/service boundary instead of parsing the domain
transaction. The obsolete Stage-2 planner publication source-order suite and
the final compatibility execution headers have been removed. Remaining source
checks cover ROS message/QoS/wiring integration or architectural dependency
bans, not domain transaction behavior.
Manager-owned pending identity and atomic base validation are now covered by a
direct executable suite instead of activation source-order parsing.
Item 12 remains in progress until the complete static audit and the unchanged
three-run Manhattan mission gate below are finished.

### Validation

Deterministic generic sensor-to-execution regressions cover unobstructed motion,
lower and upper openings, a lateral opening, a vertical shaft, an inclined
passage, a wall requiring initial motion away from the goal, T and X junctions,
a loop, a cul-de-sac with return, and a route beyond the local distance cache.
They prove:

- arbitrary `Free`/`Unknown` relabeling with occupied cells fixed leaves path,
  cost, rank, speed, and admission unchanged;
- frequent world revisions and better competing candidates do not change the
  valid active route identity;
- a climb-first or drop-first intent remains owner through the passage;
- a longer raw-safe route with fewer mandatory stops defeats a shorter zigzag
  when the shared jerk, acceleration, and yaw model predicts lower execution
  time, while bounded refinement resumes without publishing a greedy frontier;
- successor reserve includes measured p99 latency, stopping distance, and
  overlap, with continuous braking ownership at every commit boundary;
- newer raw occupied evidence repairs or brakes the affected suffix without
  publishing stale or colliding execution;
- planner, compiler, controller, and diagnostics use the same full-3D semantics.

Final mission validation uses the unchanged Manhattan location, spawn, and
waypoints. Run three sequential no-static 3D-lidar headless point-to-point
missions. Each must reach the mission goal within the 120-second hard limit
(approximately 90 seconds is the target), cross the required low-altitude route
volume, remain collision-free, and have no route-ownership gap. After bootstrap,
route availability must exceed 99 percent and ordinary
`no_executable_route_hold` should be effectively absent; planner p95 is targeted
below 200 ms. Each runtime manifest records commit, configuration, and world
hashes, and the run retains the raw snapshot for the constrained section.

## 13. GNSS- And Magnetometer-Denied Lidar-Inertial Navigation

**Type:** dependent localization stage.

**Hard prerequisites:** items 8 and 12.

Item 13 is accepted only in the complex environments introduced after the
original Manhattan world. Manhattan is not a localization acceptance
environment because its repetitive geometry creates severe position and
heading ambiguity. Autonomous test flights in the selected labyrinths, caves,
and tunnel networks require item 12's persistent full-3D route and repair
backend. This roadmap dependency must not create a code dependency between the
localization estimator and the route planner.

Add an optional navigation profile in which the aircraft does not use GNSS or
magnetometer fusion. This stage begins after item 8 provides production 3D
lidar and its timestamped full-6DoF acquisition-pose contract and item 12
provides autonomous routing through partially observed complex environments.
The aircraft retains its IMU and barometric altitude source and estimates
motion from lidar-inertial odometry instead of receiving global position and
heading from simulated navigation satellites and a simulated compass.

Localization must remain a separate subsystem from obstacle memory and route
planning. A dedicated lidar-inertial estimator deskews 3D scans, propagates the
high-rate IMU state, registers scans against dedicated localization submaps,
and publishes a typed pose, velocity, covariance, quality, and frame identity.
Planner obstacle memory consumes that estimate; it must not become the
authoritative localization map, because a map assembled from an erroneous pose
can otherwise reinforce the same localization error.

Feed the estimate to PX4 through its supported external-odometry interface and
configure PX4 to fuse it while GNSS and magnetometer fusion are disabled. The
existing PX4 local-position output remains the stable contract for offboard
control, planning, mapping, and diagnostics. Gazebo ground truth is available
only to evaluation and referee components and must never cross into the
estimator or control data path.

Support two explicit localization configurations:

- with a valid static 3D map, lidar-inertial odometry provides continuous local
  motion while scan-to-map registration corrects accumulated drift and anchors
  the vehicle in the mission map frame;
- without a static map, lidar-inertial SLAM builds revisioned localization
  submaps and uses loop closure to maintain a locally consistent frame.

No-static missions with absolute map-frame goals require a declared initial
map pose or another explicit global reference. Unknown-pose localization in a
known static map is a separate global relocalization capability and must not be
implicitly replaced by a scenario-provided hidden ground-truth transform.

Localization quality and geometric observability must be first-class runtime
signals. Repetitive city blocks, long feature-poor tunnels, and symmetric caves
can leave translation or yaw weakly constrained even with 3D lidar. When the
estimate is stale, divergent, or insufficiently observable, the system must
stop publishing new executable motion and let the current finite path reach
its validated terminal state; it must not continue an invalid path or add a
sticky braking or geometric exclusion lifecycle.

Implement and validate this stage incrementally:

1. replay timestamped 3D lidar and IMU data offline and compare estimated poses
   with evaluation-only Gazebo truth;
2. fly one vehicle from a known initial pose using PX4 external odometry with
   GNSS and magnetometer fusion disabled;
3. add static-map correction, relocalization, and explicit estimator health;
4. add no-static submaps and loop closure;
5. validate multiple cooperative vehicles, each with an independent estimator
   and no shared localization state.

Measure position and attitude drift, velocity error, map alignment, loop
closure consistency, estimator latency, relocalization time, time without a
valid executable path, minimum obstacle clearance, and physical collisions.
This stage is complete when repeated static-map and no-static 3D-lidar missions
run without GNSS, magnetometer data, or control-visible simulator ground truth,
and localization failures produce an explicit safe finite-path outcome instead
of silent frame corruption.
