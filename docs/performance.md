# Performance

The main runtime costs are ESDF preparation, horizon assembly and the horizon
commit; CUDA MPPI is a smaller share than any of them.

## The Planning Cycle Budget

A planning tick is snapshot preparation, the controller, and publication
(assembly, commit, wire). The controller is the part with a GPU in it and the
smallest part of the cycle: measured at p50 around 7 ms against a 20 ms period,
while the whole cycle sat near 48 ms. Two things dominated it, and both were
re-doing work already done:

- **Horizon assembly.** The arrival-shaping search rebuilds the horizon once
  per shortened nominal prefix, and each rebuild solves an arrival profile by
  damped Newton and then re-swept the *whole* path against occupied evidence.
  Every candidate shares its leading states and controls bit for bit with the
  longer one before it, so a point once proved clear stays clear for the rest of
  the search: `validateCompleteFiniteExecutionPath3D` now takes the number of
  leading points whose sweep the caller has discharged.
  `execution_maximum_assembly_ms` bounds the whole search, so a bad tick
  degrades into a hold rather than into a horizon delivered several periods
  late.
- **The commit revalidation.** A commit whose evidence moved since the
  candidate was prepared revalidates the published horizon, and swept all of
  it. The part the vehicle has already flown will not be flown again, so the
  revalidation now discharges it — which also stops a perfectly executable
  horizon being revoked for evidence that appeared behind the vehicle.
- **The latest lidar scan.** Every swept segment of the horizon asks which of
  the scan's tens of thousands of returns can touch the body, and the answer
  used to be one pass over the whole scan per segment: a hundred segments per
  validation, several validations per assembly, and the commit's revalidation
  on top. The scan is now bucketed into cubic cells once when it is captured
  (`IndexedPointCloud3D`), and a segment visits only the cells its sweep
  overlaps.

`deadline_misses` in `PRODUCTION_MPPI_SUMMARY` measures the whole cycle against
the period, and `controller_deadline_misses` the controller's own share. A run
with thousands of the former and none of the latter is telling you the cost is
around the optimiser, not in it.

## CUDA MPPI

Per-tick GPU stages are:

- noise generation;
- rollout simulation and ESDF queries;
- risk reduction;
- weight calculation;
- control update;
- warm-start shift.

Use CUDA events for GPU stage timing and host steady-clock timing for
end-to-end latency. Report distributions, not one sample:

- p50;
- p95;
- p99;
- maximum;
- deadline misses.

The intercept mission loads all vehicle planners into one ROS 2 component
container. Their MPPI engines share one process and CUDA primary context while
retaining separate streams, buffers, nominal controls, and route state. This
reduces process, DDS, and CUDA-context overhead without changing rollout
selection.

A fused vehicle-by-rollout micro-batch was evaluated with the same mission
contract and rejected. The live four-vehicle workload produced an average batch
size of only about 1.5 vehicles: synchronization increased static GPU p50 from
22.8 ms to 45.9 ms and no-static GPU p50 from 3.45 ms to 6.07 ms. Independent
streams in the shared process are therefore the production backend. Reconsider
fusion only if profiling demonstrates reliably full batches without collection
latency.

The three radar tracker/guidance pairs use one interceptor-side component
container. This replaces six standalone ROS contexts with one shared context;
intra-process delivery also removes the DDS serialization hop between each
`TargetTrack` publisher and its guidance consumer. Radar simulators stay
process-isolated because they consume typed target physical truth. A single
simulation-truth adapter subscribes to Gazebo's dynamic-pose stream, avoiding a
full static-scene pose stream for every radar.

Intercept visualization uses a third component container for the spectator,
diagnostics mux, world visualization, and enabled selector-gated lidar-debug
nodes. Spectator selection and detailed point clouds use intra-process delivery
inside that container. The control, planning, mapping, referee, and radar
ground-truth boundaries remain separate processes.

Intercept simulation uses subsystem affinity rather than per-vehicle pinning.
On a 16-CPU host the default masks are control/physics `0-7`, planning/mapping
`4-15`, and diagnostics `12-15`. The overlap leaves dedicated capacity at both
ends while allowing bursty control and planner work to share the middle CPUs.
Set `ENABLE_SUBSYSTEM_CPU_AFFINITY=false` to run an unrestricted comparison, or
override `CONTROL_CPU_LIST`, `PLANNING_CPU_LIST`, and `DIAGNOSTICS_CPU_LIST`.

Buffers are allocated for the configured maximum rollout count. A confirmed
direct-interception tick may execute a smaller validated prefix through
`direct_tracking_rollouts`; loss of direct tracking, route execution, holds, and
all ordinary navigation ticks retain the full configured budget. Diagnostics
record `active_rollouts` so timing changes can be compared by actual GPU work.

## ESDF

No-static 3D builds `KnownObstacleDistance3D` as an exact capped dense
Euclidean distance transform: occupied sources inside the rectangular influence
halo of the chunk-aligned local window are transformed with three separable
linear-time passes, parallelized over the dedicated world worker pool. The cost
is linear in window volume and independent of the number of occupied voxels,
which replaced the former kd-tree walk and incremental patch repair. An
unchanged occupied source fingerprint reuses both host distances and the
resident GPU texture while rebasing the exact raw generation. Diagnostics report
build mode (`full` or `reused`), source and transform voxel counts, and
collection/transform durations.

High ESDF age is a world-update problem even when MPPI GPU timing is excellent.

## Tick Phases

Every planning tick records snapshot, controller, publication, and total wall
time. The per-tick INFO line and JSONL carry `snapshot_ms`, `controller_ms`,
`publication_ms`, and `tick_total_ms`; the summary reports their p50/p95/p99
together with horizon publications, commit rejections, acknowledgement
deferrals, grace replacements, and resident-owner continuation ticks. A
regression in CPU-side validation or in the execution handshake is therefore
visible without GPU timing.

## Raw Validation Fast Paths

Raw swept-footprint validation first tests the axis-aligned extent of the
whole sweep against the occupied chunks it touches; a sweep whose chunks hold
no occupied bit is accepted without per-pose sampling. Lidar point validation
prefilters points by the sweep extent before the exact per-pose test. Both
paths are pure fast rejections of empty space: every non-empty extent still
runs the exact oriented body test.

## Strategic Planner

D* Lite adjacency is visited without allocation, edge costs are cached and
invalidated only within the affected reach of changed occupied voxels, and a
missing incremental predecessor repairs from an exact full-grid difference
instead of resetting the search. A change forgets only the priced edges it can
move: a voxel that became occupied forgets clear edges whose swept body touches
it, a voxel that became free forgets blocked ones, and adaptive edges are tested
against the exact touch rather than their chunk, so the add/remove flicker of a
persistent raw surface no longer re-validates every edge beside it. Soft
clearance ranking is validated lazily: a change stamps its chunks, a cached
node clearance is re-derived when the search next consults it and a stamped
chunk lies within the ranking reach, and only a re-derivation that moves the
ranking factor by more than a few percent schedules the node for repair. The
scheduler therefore never walks the ranking reach box per changed cell. A
cached clearance below its reach goes stale only when a changed chunk's box
lies within that clearance: evidence added farther away cannot lower a
minimum and evidence removed farther away was not the nearest, so a label
beside a wall ignores the scan-to-scan churn of everything beyond the wall.
The derivation itself reads the observed ESDF's known-obstacle distance field
where the point lies inside its window: the lattice nodes sit on voxel corners,
where the minimum over the eight bracketing cell centres is the exact raw
clearance, and any other point gets a lower bound within half a voxel
diagonal, so a re-derivation costs eight reads instead of a nearest-first
chunk search of the raw grid (50–70 µs per node at a 6 m reach, which made
the repair of a vertex cost about a millisecond). The launch-support contact
cells the field leaves out are counted from the contact boxes, and a point
outside the window, or a world without the field, derives from the raw grid as
before. The field only prices; the raw swept body check stays the authority
on every edge. Repair
and search share every update: with a repair queue pending the search still
receives half the budget, so a world that changes every scan cannot starve it
of expansions, and an anytime route on the current labels reaches the vehicle
while later repairs re-converge the search. The session runs the optimized D* Lite vertex
maintenance: when an expansion lowers g(u) each predecessor's rhs is tightened
through its one edge into u, and when an expansion raises g(u) only the
predecessors whose rhs went through u rescan their successors, so an
expansion costs O(b) ranked edge evaluations rather than O(b^2) over the
26-connected lattice and its adaptive levels. The feasibility-first search runs
only while no incumbent exists, so the resident session's repair and expansion
work improves a route nobody can fly: the session keeps a reserve of the update
(its configured feasibility time, at most a third of the remainder) to absorb
the world, and the search takes the rest. Time to a first route is what the
vehicle waits on at every mission waypoint, and a search bounded to half the
update doubled that wait. Its
labels survive occupied changes: each label carries the world epoch on which
the chain of lattice edges that reached it was last validated, the chain is
re-checked against the lattice edge cache only when the search next touches
the label, and the label behind an edge that stopped surviving is re-parented
through the cheapest adjacent label already validated on that epoch; only a
label no intact neighbour can adopt is dropped, together with the labels below
it on that chain, and re-entered from the labels around it. A scan that moves
one edge near the vehicle therefore costs the search a few re-parented labels
rather than every label beyond the edge, and the re-entry cascade of a dropped
label stops at the planner deadline and resumes on the next call instead of
holding the planning thread. An edge the raw sweep rejects on a candidate is withheld from the search
on that world, so a candidate can never be rebuilt through it. The lattice
edge cache both searches price their edges from forgets every cached edge a
change can touch, whether or not the persistent session labelled its
endpoints, so a region only the feasibility search has priced never keeps an
edge the resident world no longer supports. The raw clearance probe behind the
ranking and the tracking tube visits chunks nearest to the query first and
stops once no unvisited chunk can hold a closer voxel, so a 6 m ranking reach
costs less than the former 3 m scan.

## Tracking Tube

The tracking-error tube derives each segment's admissible body inflation from
an exact capped distance query against raw occupied voxel boxes sampled along
the segment, instead of a bisection over inflated swept footprints. The
physical body remains the hard authority; the tube only shapes the speed
ceiling, and a configured progress floor keeps constrained segments moving
wherever the body itself clears raw occupancy.

## Diagnostics

Execution-horizon publication occurs before RViz, INFO, and JSONL processing.
Diagnostics use a bounded latest-value mailbox. Queue pressure increments a
dropped counter instead of delaying control.

## Benchmark

Run the isolated CUDA benchmark through the container:

```bash
make mppi-benchmark \
  MPPI_BENCHMARK_ARGS="--scenario urban_blocks --rollouts 8192 --steps 80"
```

The benchmark includes real dynamics integration, ESDF texture queries,
clearance-cost evaluation, weighted update, and warm start. It does not include
ROS, Gazebo, or live ESDF rebuild contention.

## Production Profiling

For a real run, compare:

1. sensor and raw-snapshot cadence;
2. ESDF build/upload latency and age;
3. MPPI host/GPU p50/p95/max;
4. deadline misses;
5. dropped ESDF and diagnostic snapshots;
6. offboard horizon receive age;
7. liveness, route-unavailable hold, and unavailable-path hold frequency;
8. planner process/thread count and CUDA process count in multi-vehicle runs.

Do not lower collision checking or weaken finite-path validation solely to
improve timing. First reduce redundant ESDF work, stale revisions, diagnostic
load, or rollout count based on measured bottlenecks.
