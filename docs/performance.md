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

Full CPU ESDF construction is substantially more expensive than one resident
MPPI tick. It therefore runs asynchronously by obstacle revision. MPPI uses the
latest complete immutable field and logs ESDF age and raw-to-ready latency.

High ESDF age is a world-update problem even when MPPI GPU timing is excellent.

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
