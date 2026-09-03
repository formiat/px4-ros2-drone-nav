# Performance

The main runtime costs are ESDF preparation and CUDA MPPI.

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
clearance ranking repairs a label only when its ranking factor moves by more
than a few percent; smaller moves are cached. The raw clearance probe behind
the ranking and the tracking tube visits chunks nearest to the query first and
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
