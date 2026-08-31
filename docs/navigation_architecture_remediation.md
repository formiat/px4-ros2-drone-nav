# Navigation Architecture Remediation

## Status

This is the active architecture-completion plan for roadmap item 12. It replaces
the former mixture of aspirational manager documentation, test-only lifecycle
state, and split production ownership across world, route, and runtime fields.

The migration is incomplete until every checklist item below is implemented,
covered by executable tests, and the unchanged Manhattan acceptance gate in
[`roadmap.md`](roadmap.md) has been evaluated.

## Non-Negotiable Invariants

- Confirmed raw occupied geometry and the physical flight envelope are the only
  hard spatial constraints.
- `Free` and `Unknown` have identical traversability and base cost throughout
  search, compilation, admission, execution, and publication.
- Derived ESDF, clearance, topology, observability, and tracking evidence cannot
  manufacture `raw_collision` or independently reject raw-safe geometry.
- Navigation is genuinely three-dimensional. Search policy, progress, reserve,
  route retention, and successor timing cannot prefer a goal-altitude plane or
  use an XY-only substitute for a 3D mission objective.
- A valid active route remains owned across ordinary world updates and planner
  refinement. Replacement requires a new objective, exact raw invalidation,
  completion, or a certified continuity-preserving successor that clears the
  configured improvement hysteresis.
- Planning, compilation, activation, execution, and diagnostics use the same
  kinematic and time-profile authority.
- A controller-visible execution change is one atomic ownership transition. A
  reader cannot observe a route plan paired with another plan's owner, input, or
  applied-control evidence.

## Target Dependency Graph

The package remains one ROS 2 package and may retain one production ROS
component. Its internal targets must form this directed graph:

```text
nav_model
  -> nav_world
  -> nav_collision
  -> nav_planning
  -> nav_trajectory
  -> nav_execution
  -> nav_control
  -> nav_runtime
```

These boundaries are now enforced by the shared-library targets
`drone_city_nav_model`, `drone_city_nav_world`,
`drone_city_nav_collision`, `drone_city_nav_planning`,
`drone_city_nav_trajectory`, `drone_city_nav_execution`,
`drone_city_nav_control`, and `drone_city_nav_runtime`. Every layer links
only its immediate predecessor and uses `--no-undefined`; configure-time
guards reject any additional internal edge. `drone_city_nav_core` remains an
interface-only compatibility umbrella for in-package targets during API
migration and owns no translation units. The internal layer targets and
hand-written headers are not exported as a downstream CMake API; rosidl-generated
messages and installed ROS nodes/components are the package's supported external
surface.

Execution contracts are physically split into
`execution_route_model_3d.hpp`, `execution_route_certificates_3d.hpp`,
`execution_plan_3d.hpp`, `execution_route_certification_3d.hpp`,
`execution_route_transitions_3d.hpp`, and `execution_route_store_3d.hpp`.
`RouteExecutionManager3D` is the canonical store owner. The former snapshot and
manager headers remain include-only compatibility umbrellas and have no internal
production consumers. Every hand-written public-path and private header is
compiled as an independent translation unit in test builds.

Optional passage and cooperative metadata decorate a compiled route downstream;
they are not mandatory members of the base trajectory and do not produce a
competing strategic route.

### `nav_model`

Owns controller-neutral points, vectors, identities, units, kinematics,
dynamics, and risk classifications. No route or planning header may include an
MPPI header.

### `nav_world`

Owns immutable `WorldSnapshot3D` values: raw occupancy identity, exact occupied
evidence, world revision, and optional derived distance/topology caches. Derived
caches have explicit validity but no hard-collision authority.

### `nav_collision`

Owns the single `OccupiedCollisionOracle3D` used by planning, shortcutting,
materialization, certification, and finite-execution validation:

```text
blocked(segment) = outside_flight_envelope
                OR swept_footprint_intersects_raw_occupied
```

This boundary is implemented. Low-level raw occupancy and point-cloud queries
are confined to the collision implementation and focused primitive tests;
production consumers use the oracle. Unknown and cells outside a finite raw
snapshot are traversable; invalid or missing derived clearance is neutral.
ESDF/MPPI types cannot express `raw_collision`, and the unused controller-local
`KnownSolid` hard-collision path has been removed.

### `nav_planning`

Owns `Lattice3D`, `DStarLiteSession3D`, `FeasiblePathSearch3D`,
`ExecutionTimeRefiner3D`, `PathPostprocessor3D`, and
`AnytimePlannerCoordinator3D`. A planner update has two independent axes:

```cpp
struct PlannerUpdate3D {
  std::optional<SpatialRouteCandidate3D> improved_incumbent;
  SearchProgress3D progress;
  PlannerTelemetry3D telemetry;
};
```

A publishable incumbent never implies convergence. The runtime may admit an
incumbent and must separately requeue the same running search session until it
converges, reports no route, or is invalidated.

### `nav_trajectory`

Owns spatial route geometry and the only executable trajectory compiler. The
compiler receives the exact initial vehicle state and seals canonical stations,
tangents, the time profile, tracking tube, physical fingerprint, and optional
decorators in one immutable `CompiledTrajectory3D`. MPPI references and RViz
projections are derived adapters, not parallel authorities.

### `nav_execution`

Owns one `RouteExecutionManager3D`, including pending and active ownership,
certification, progress, retention, successor hysteresis, braking fallback, and
atomic publication. Execution state is a tagged variant whose alternatives make
route/hold/direct/braking conflicts unrepresentable:

```cpp
using ExecutionPlanState3D = std::variant<
    FollowingPlan3D,
    DirectTrackingPlan3D,
    BrakingPlan3D,
    StationaryHoldPlan3D,
    AwaitingSuccessorPlan3D,
    RevokedPlan3D>;
```

One pure reducer validates transitions. The committed runtime authority is one
immutable object published and captured atomically:

```cpp
class CommittedExecutionAuthority3D {
  std::uint64_t revision;
  std::shared_ptr<const ExecutionPlan3D> plan;
  ExecutionOwnerIdentity3D owner;
  std::shared_ptr<const VersionedExecutionInput3D> input;
  AppliedControlEvidence3D control;
};
```

`ExecutionSupervisor3D` owns the only production `RouteExecutionManager3D`;
the manager remains the only constructor and publisher of this value.
Every mutation is an exact-pointer compare-and-swap transaction against the
captured authority, increments its monotonic revision, and validates that the
lease belongs to the plan owner, the input is the plan's exact immutable input,
and control feedback belongs to that lease. A plan transition clears prior
control evidence. Lease revocation, feedback replacement, pending activation,
and unchanged-plan horizon refresh each publish one complete replacement before
the corresponding DDS message can become visible.

Production code no longer accesses the manager directly. The supervisor exposes
one owned lease commit whose kind is transition, unchanged plan, or pending
transition, plus narrow pending, revocation, and control-evidence operations.
Activation commit and pending recovery use that facade. Direct tests execute all
three lease kinds, exact pending consumption, stale-authority rejection, complete
control-evidence replacement, and concurrent single-winner publication. The
former Python parsing of the manager's pending-clear order has been removed.

Retention preparation now crosses one owned `ExecutionRetentionRequest3D`.
The supervisor captures the exact resident authority, selects route or direct
tracking from that capture, reconstructs the remaining finite path, validates
current raw/lidar evidence, certifies the replacement and braking fallback, and
returns the captured authority with one immutable transition. Preparation does
not mutate the store. Direct tests cover normal route retention and commit,
direct-tracking retention, exact raw-invalidation emergency braking, stale
lifecycle ownership, and missing-evidence rejection. The ROS adapter contains
no finite-path certification or reducer calls.

Stationary-hold preparation now crosses one owned `ExecutionHoldRequest3D`.
The supervisor captures the exact authority, distinguishes resident refresh,
explicit terminal transfer, and the named revoked-owner stationary-capture
rearm, validates current raw/lidar lineage, and returns either one immutable
transition or an exact unchanged-plan result. Preparation never mutates the
store, and the publication commit rejects any intervening authority revision.
The ROS adapter owns only evidence capture, intent mapping, horizon encoding,
and the wire commit; it contains no hold certification or reducer calls. A
refreshed execution input always produces a new immutable hold snapshot, while
`kNoChange` is reserved for the exact resident input and evidence owner.

Pending-route publication no longer allocates identity in the ROS node or
performs a check-then-publish pair. An unsealed candidate carries sequence zero;
`RouteExecutionManager3D` atomically validates its captured semantic execution
base, assigns the next monotonic sequence, validates and seals the route, and
occupies the sole pending slot. Direct state-machine coverage exercises sequence
ownership, occupied-slot rejection, externally numbered candidate rejection,
and stale-base rejection. That executable manager suite replaces the former
Python assertions that parsed activation-source ordering and helper bodies. The
legacy unchecked `publishPending` entry point has been removed, including from
test fixtures; all pending publication now crosses the same production API.

### `nav_control`

Owns controller-neutral local-reference and finite-horizon contracts plus the
MPPI/CUDA adapter. MPPI does not own route-domain types.

### `nav_runtime`

Owns workers, scheduling, mission policy, ROS adapters, parameters, logging, and
diagnostics. `ProductionMppiNode` is the composition root and ROS I/O boundary;
it does not own the mutable internals of world, planning, trajectory, or
execution services.

`NavigationDiagnosticsSink` now owns the bounded latest-value mailbox, worker
lifetime, dropped/failure counters, rate-limited JSONL files, bounded error
context, flushing, and the coherent runtime-statistics snapshot. The node
supplies only the formatting/ROS-publication callback and explicitly stops the
sink before its publishers are destroyed. Worker overload, stop-time draining,
processor failure isolation, file/error-ring behavior, and statistics are
covered by direct executable tests.

`WorldPipeline3D` now owns raw producer admission, status/payload joining,
incremental reconstruction, the immutable latest raw world, latest-wins
scheduling, worker lifetime, the resident world and telemetry, generation
issuance, and publication/build counters. Its public resident lease and private
publication transaction make GPU upload plus CPU world installation one
linearizable transaction.
Executable tests cover overload and dirty lineage, producer-identity conflict
quarantine and recovery, publication/read exclusion, exact transient-evidence
refresh, invalid-generation rejection, exception containment, and lifecycle
reentry during stop. `ObservedWorldBuilder3D` and the world-service transaction
now own observed local-window selection, recentering, full-audit/rate policy,
full/incremental/reused construction, exact-parent admission, GPU upload, and
immutable publication. The node supplies an immutable pose/execution-evidence
request and consumes typed early-evidence and final update events. Persistent
evidence-only changes issue a new exact-parent local generation without a GPU
upload and force planner revalidation. Upload exceptions invalidate the resident
world because a partially changed controller resource cannot remain paired with
the prior CPU artifact. `StaticWorldBuilder3D` now owns static ROI selection,
fingerprint-bound cache extraction with permanent runtime-EDT fallback, immutable
occupancy/topology/CPU-ESDF assembly, and resource reuse. `WorldPipeline3D` owns
refresh sequencing and latest-wins coalescing, early and late base-route
supersession, GPU residency, generation issuance, fail-closed publication, and
typed update events. The node only supplies immutable navigation/objective and
late commit contexts, adapts the uploader, and coordinates a route search after
a successfully published world.

`RoutePlanningCoordinator3D` now owns the single pending request slot, explicit
keep-versus-replace admission policy, planning worker lifetime, request/world/
route-generation validation, the sole `RoutePlanner3D`, and failure isolation.
New world publications replace an older pending world request; bounded anytime
continuations never displace newer work. The node supplies controller-neutral
vehicle and resident-generation ports and consumes typed update or rejection
events. Direct concurrency tests cover pending replacement and lifecycle
transfer, busy retention, validation reasons, callback failure containment,
reentrant continuation submission, stop, and restart.

`RouteMaterializer3D` now owns geometric route materialization, exact active-
route splice prerequisites, raw-only smoothing collision input, derived risk
annotation, optional passage decorators, final candidate validation, and its
telemetry. It receives one self-contained request that owns the exact planner
transaction, candidate, active route, and raw-world snapshot and returns one
typed result. Geometry fallback is an event field consumed by the ROS logging
adapter; the materializer has no node or ROS dependency. Direct tests cover
invalid configuration/request handling, successful immutable materialization,
unknown-neutral invalid derived distances, and exact active-route enforcement.
The soft derived-clearance annotation API is controller-neutral
`route_risk_annotation_3d` in `nav_planning`; the former MPPI adapter and names
have been removed, so route materialization no longer depends on `nav_control`.

`RouteTrajectoryCompiler3D` owns the immutable compiler and passage
configuration and accepts one materialized-route transaction with the exact
initial `VehicleState3D`, endpoint semantics, and observed raw-world owner. It
constructs the tracking-world binding internally and returns the single sealed
`CompiledTrajectory3D` result without reading node state. Direct tests cover
static exact-state sealing, observed-owner binding, and fail-closed missing
observed ownership. The former Python source-order check for this binding has
been removed in favor of that executable transaction test.

`RouteActivationCoordinator3D` now owns the sole trajectory compiler and the
complete compilation, admission, generation/replacement assessment,
certification, splice, and pending-draft pipeline. Its single preparation
request owns the exact planner transaction, materialized route, coherent
world/navigation/objective/execution snapshot, and planning-latency sample.
Preparation reads no node state and emits one `PreparedRouteActivation3D`.
Commit consumes that artifact by value, receives a caller-locked currentness
context and the sole execution manager, then returns a distinct typed result;
it distinguishes snapshot supersession from a manager rejection and cannot
publish around the manager or expose a half-updated preparation. The thin ROS
adapter owns capture locks, clock access, warning output, and the latest telemetry
event; the exact manager publication status remains in that admission event.
Direct tests execute exact-state preparation, successful atomic
publication, resident-world supersession, occupied-slot retention, and invalid
request rejection.

`MppiController3D` now owns the only production `MppiCudaEngine`, the complete
nominal-reseed state machine, and the cached conversion from sealed trajectory
to controller reference. One owned request returns the exact `MppiTickInput`
after reseed assignment together with the result and typed controller status.
Stationary hold and backend failure isolation are controller operations; ROS
logging, route-release requests, fail-closed revocation, and the caller-held
resident-world lease remain in the thin node adapter. GPU world upload and
readiness are narrow controller ports used by `WorldPipeline3D`. Direct tests
cover exact hold output, persistent and concurrently serialized reseed ownership,
unavailable-backend classification, stable status names, cache ownership through
the service, and rejection of missing, incoherent, or superseded resident GPU
worlds. Those executable currentness cases replace the former source-text check
for a particular `sameSnapshot` call location.

## Immutable Stage Pipeline

```text
MissionObjective3D + VehicleState3D
  + WorldSnapshot3D
  -> PlannerUpdate3D
  -> SpatialRouteCandidate3D
  -> MaterializedRoute3D
  -> CompiledTrajectory3D
  -> RouteAdmissionReport3D
  -> RouteExecutionManager3D transition
  -> CommittedExecutionAuthority3D
  -> MppiReferenceAdapter3D
  -> finite raw-safe PX4 horizon
```

Telemetry is emitted as separate events. It is not stored by mutating a world,
route, trajectory, or execution artifact.

## Route Retention And Refinement

- The first feasible raw-safe route may become an incumbent but does not stop
  D* repair or execution-time refinement.
- An admitted active prefix remains frozen while its suffix is valid.
- An improved incumbent first becomes a certified pending successor.
- A normal successor is admitted only when its remaining execution time beats
  the resident route by the configured absolute and relative hysteresis.
- A raw collision repairs the affected future suffix while retaining the valid
  prefix and certified braking owner.
- World revision alone never clears or replaces route ownership.
- Passage and cooperative decorators are rebuilt from the selected route and
  world evidence; losing optional topology metadata cannot erase the resident
  world or invalidate raw-safe base geometry.

## Completion Checklist

The immutable world/search/materialization/admission boundaries are now
implemented. A resident publication swaps one
`shared_ptr<const WorldSnapshot3D>`; topology is an explicitly optional derived
cache on that snapshot; and `MaterializedRoute3D`, compilation candidates,
`RouteAdmissionReport3D`, and pipeline telemetry are distinct values. World
publication cannot copy, clear, or restore route state. Route materialization
geometrically associates matching topology traversals instead of constructing
an unconditionally empty decorator list.

Trajectory compilation now requires one exact revisioned `VehicleState3D` and
one route generation. `TrajectoryCompiler3D` canonicalizes the route once,
derives one tracking tube, parameterizes one speed/time profile from the exact
initial 3D velocity, validates the complete passage-resource graph, and is the
only constructor of `CompiledTrajectory3D`. The sealed class is neither
copyable nor movable; every owned collection is const. MPPI references and
planar RViz views are derived adapters and carry no independent route identity.
The former route compiler, execution-geometry wrappers, stored MPPI route, and
stored 2D projection have been removed.

Planner search now owns an immutable `PlannerSearchTransaction3D` containing
the exact world publication, derived resident planner input or explicit newer
raw overlay, mission objective, typed request identity, release reason, and an
optional certified continuity base. Continuation work retains the same
transaction pointer instead of copying a mutable world/route aggregate. The
world snapshot also carries the raw occupied fingerprint and exact incremental
planner predecessor; a skipped publication forces a safe full planner repair
rather than applying an incomplete dirty-chunk delta. Planner request flags,
parallel planner-world copies, and search objectives no longer reside on the
published world.

Execution publication now owns one immutable
`CommittedExecutionAuthority3D`. `RouteExecutionManager3D` atomically publishes
the plan, typed owner identity, pointer-identical versioned execution input, and
matching applied-control evidence together. Activation and planning ticks load
one authority pointer; mission capture, feedback, lease revocation, and horizon
publication commit only against that exact pointer. The former node-owned
`applied_control_` and `execution_horizon_owner_` fields have been removed, and
executable concurrency tests exercise replacement while readers validate that
no mixed authority revision is observable.

- [x] Split publishable incumbent from search progress and continue anytime
  refinement after the first feasible route.
- [x] Remove goal-altitude-first feasibility ordering and decompose the three
  planner searches behind explicit session interfaces.
- [x] Replace the one-element production candidate set with one typed planner
  update and an explicit coordinator result.
- [x] Introduce immutable world/search/materialization/trajectory/admission
  artifacts and remove route state from the resident world object.
- [x] Introduce the single unknown-neutral occupied-collision oracle and prove
  that invalid or missing derived evidence cannot become `raw_collision`.
- [x] Compile once from the exact initial vehicle state into one sealed,
  controller-neutral `CompiledTrajectory3D`.
- [x] Replace execution phase plus optionals with the tagged variant and one pure
  transition reducer.
- [x] Make `RouteExecutionManager3D` the sole pending/active plan owner and
  linearize pending-to-active replacement under the same mutex.
- [x] Publish plan, owner, input, and applied-control evidence as one atomic
  committed authority.
- [x] Remove the test-only parallel lifecycle state and all competing lifecycle
  ownership terminology.
- [ ] Extract world, planning, trajectory, execution, and control services from
  `ProductionMppiNode`; retain only composition and ROS I/O in the node.
  - [x] Extract `NavigationDiagnosticsSink` as the sole diagnostics worker,
    mailbox, file, error-context, and statistics owner.
  - [x] Extract the complete world pipeline.
    - [x] Move raw ingestion, producer lineage, reconstruction, latest-wins
      scheduling, worker lifecycle, immutable resident publication, generation
      issuance, and coherent world statistics into `WorldPipeline3D`.
    - [x] Move static and observed ESDF construction policy and orchestration
      out of `ProductionMppiNode` callbacks and behind the world-service API.
      - [x] Move observed full/incremental/reused construction, rate/audit/
        recenter policy, exact-parent admission, upload, and publication behind
        typed request/event ports.
      - [x] Move static ESDF/cache/topology construction and refresh policy
        behind the same service boundary.
  - [x] Extract persistent planning and route-pipeline coordination.
    - [x] Move the sole persistent planner, exact continuation session,
      certified-future-stitch selection, route sampling, and raw-only segment
      evidence into `RoutePlanner3D` behind a typed non-ROS API.
    - [x] Move single-pending request scheduling with explicit newest-world
      replacement, worker lifecycle, validation, failure isolation, and typed
      update/rejection events into `RoutePlanningCoordinator3D`.
    - [x] Move geometric materialization and validation behind the non-ROS
      `RouteMaterializer3D` one-request/one-result boundary.
    - [x] Move exact-state trajectory compilation behind the non-ROS
      `RouteTrajectoryCompiler3D` one-request/one-result boundary.
    - [x] Move activation coordination behind the planning-service boundary.
  - [x] Extract trajectory compilation and controller ownership.
    - [x] Extract the sole sealed trajectory-compilation service.
    - [x] Extract controller ownership.
  - [ ] Finish the execution-service facade around the existing sole
    `RouteExecutionManager3D` owner.
    - [x] Move pending sequence allocation and execution-base-checked pending
      publication into one manager transaction.
    - [x] Make `ExecutionSupervisor3D` the sole production manager owner and move
      activation, pending recovery, lease commits, revocation, and control
      evidence through its typed API.
    - [x] Move retention preparation behind the facade.
    - [x] Move hold preparation behind the facade.
    - [ ] Move the remaining horizon validation and commit orchestration behind
      the facade.
- [x] Enforce the internal dependency graph with CMake targets.
- [x] Stop installing private implementation headers as public API.
- [x] Register every production-relevant GTest source and remove the stale test
  for the retired raw-snapshot/risk-field protocol.
- [ ] Replace source-text transaction checks with executable state-machine and
  concurrency tests. Raw-world joining, supersession, quarantine, publication
  linearization, transient refresh, stop behavior, static build/reuse,
  early/late route supersession, upload/generation failure, and refresh
  coalescing now have direct GTests. Persistent-session identity, typed
  candidate/evidence production, and cross-mission continuation rejection also
  have a direct `RoutePlanner3D` suite; the former planner source-contract suite
  has been removed. Queue replacement, lifecycle transfer, typed validation,
  callback failure isolation, reentrant continuation, stop, and restart have a
  direct `RoutePlanningCoordinator3D` suite. Route materialization, invalid
  configuration/request handling, exact active-route ownership, and neutral
  invalid derived-distance behavior have a direct `RouteMaterializer3D` suite.
  Exact initial-state sealing, observed raw-owner binding, and fail-closed
  missing ownership have a direct `RouteTrajectoryCompiler3D` suite; the former
  source-text tracking-world binding check has been removed. Manager-owned
  pending sequence allocation and atomic execution-base validation have a direct
  `RouteExecutionManager3D` suite; the corresponding activation source-text
  checks have also been removed. Exact-snapshot activation preparation,
  supersession, occupied-slot retention, manager publication, and fail-closed
  invalid requests have a direct `RouteActivationCoordinator3D` suite.
  Supervisor lease kinds, exact pending consumption, stale CAS, control-evidence
  replacement, and concurrent publication now have a direct
  `ExecutionSupervisor3D` suite; manager pending-clear source-order parsing has
  been removed. Supervisor retention tests execute route/direct rebuilding,
  raw-invalidation braking, stale-owner rejection, and fail-closed missing
  evidence; the former source-order parsing of that transaction has been
  removed. Supervisor hold tests execute terminal transfer and commit, exact
  resident replay, refreshed-input replacement, named stationary-capture rearm,
  stale source/raw/lidar rejection, and stale-authority commit rejection; node
  source checks now ban certification and reducer calls in the ROS adapter.
- [x] Keep public and private headers self-contained and retain a temporary
  umbrella include only where migration compatibility requires it.
- [ ] Pass formatting, static analysis, C++ tests, and script tests after every
  coherent stage.
- [ ] Evaluate the unchanged three-run Manhattan no-static 3D-lidar gate only
  after all preceding items are complete.

Replacing the persistent D* family with AD*, weighted LPA*, or another algorithm
is not required by this remediation. Such a change requires representative
benchmarks after the planner contract and component boundaries above exist.
