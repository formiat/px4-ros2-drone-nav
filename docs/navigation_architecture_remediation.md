# Navigation Architecture Remediation

## Status

This is the active architecture-completion plan for roadmap item 12. It replaces
the former mixture of aspirational `RouteManager3D` documentation, the test-only
`RouteSupervisor3D`, and production ownership split across
`ProductionMppiPreparedEsdf`, `ExecutionRouteSnapshotStore3D`,
`PendingCertifiedRouteMailbox3D`, and `ProductionMppiNode` fields.

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
struct CommittedExecutionAuthority3D {
  ExecutionPlan3D plan;
  ExecutionOwnerIdentity3D owner;
  VersionedExecutionInput3D input;
  AppliedControlEvidence3D control;
};
```

### `nav_control`

Owns controller-neutral local-reference and finite-horizon contracts plus the
MPPI/CUDA adapter. MPPI does not own route-domain types.

### `nav_runtime`

Owns workers, scheduling, mission policy, ROS adapters, parameters, logging, and
diagnostics. `ProductionMppiNode` is the composition root and ROS I/O boundary;
it does not own the mutable internals of world, planning, trajectory, or
execution services.

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

- [ ] Split publishable incumbent from search progress and continue anytime
  refinement after the first feasible route.
- [ ] Remove goal-altitude-first feasibility ordering and decompose the three
  planner searches behind explicit session interfaces.
- [ ] Replace the one-element production candidate set with one typed planner
  update and an explicit coordinator result.
- [ ] Introduce immutable world/search/materialization/trajectory/admission
  artifacts and remove route state from the resident world object.
- [ ] Introduce the single unknown-neutral occupied-collision oracle and prove
  that invalid or missing derived evidence cannot become `raw_collision`.
- [ ] Compile once from the exact initial vehicle state into one sealed,
  controller-neutral `CompiledTrajectory3D`.
- [ ] Replace execution phase plus optionals with the tagged variant and one pure
  transition reducer.
- [ ] Make `RouteExecutionManager3D` the sole pending/active owner and publish
  plan, owner, input, and applied-control evidence as one atomic authority.
- [ ] Remove the test-only `RouteSupervisor3D` and all competing lifecycle
  ownership terminology.
- [ ] Extract world, planning, trajectory, execution, and control services from
  `ProductionMppiNode`; retain only composition and ROS I/O in the node.
- [ ] Enforce the internal dependency graph with CMake targets and stop
  installing private implementation headers as public API.
- [ ] Register every production-relevant GTest source and replace source-text
  transaction checks with executable state-machine and concurrency tests.
- [ ] Keep public and private headers self-contained and retain a temporary
  umbrella include only where migration compatibility requires it.
- [ ] Pass formatting, static analysis, C++ tests, and script tests after every
  coherent stage.
- [ ] Evaluate the unchanged three-run Manhattan no-static 3D-lidar gate only
  after all preceding items are complete.

Replacing the persistent D* family with AD*, weighted LPA*, or another algorithm
is not required by this remediation. Such a change requires representative
benchmarks after the planner contract and component boundaries above exist.
