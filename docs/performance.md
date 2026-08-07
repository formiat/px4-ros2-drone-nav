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
selection. In intercept mode, the dominant simulation stage is fused across the
vehicle dimension when planner ticks reach the bounded two-millisecond batch
window. Per-vehicle reductions and control updates remain independent and
deterministic.

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
categorical risk, weighted update, and warm start. It does not include ROS,
Gazebo, or live ESDF rebuild contention.

## Production Profiling

For a real run, compare:

1. sensor and raw-snapshot cadence;
2. ESDF build/upload latency and age;
3. MPPI host/GPU p50/p95/max;
4. deadline misses;
5. dropped ESDF and diagnostic snapshots;
6. offboard horizon receive age;
7. liveness and braking frequency.
8. planner process/thread count and CUDA process count in multi-vehicle runs.

Do not lower collision checking or shorten braking horizons solely to improve
timing. First reduce redundant ESDF work, stale revisions, diagnostic load, or
rollout count based on measured bottlenecks.
