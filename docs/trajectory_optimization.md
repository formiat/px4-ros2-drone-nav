# GPU MPPI Trajectory Optimization

The production local planner is Model Predictive Path Integral control running
on CUDA. There is no corridor-constrained post-processing optimizer.

## Persistent Engine

`MppiCudaEngine` owns persistent CUDA buffers, the resident ESDF texture,
known-solid geometry, and the nominal control sequence. Per-tick planning does
not recreate CUDA allocations.

## State And Control

The benchmark and production engine use a 2.5D point-mass state:

```text
x, y, z, vx, vy, vz, yaw, yaw_rate
```

Controls are:

```text
ax, ay, az, yaw_accel
```

Dynamics apply acceleration, velocity, vertical-speed, yaw-rate, drag, and jerk
limits from the active static or no-static profile.

## Tick Sequence

```text
shift nominal controls by elapsed time
-> generate counter-based control noise
-> simulate rollouts on CUDA
-> query ESDF and known solids
-> choose eligible categorical risk class
-> compute MPPI weights
-> update controls
-> constrain first control against applied feedback
-> reconstruct nominal horizon
-> classify and validate
```

The warm start is shifted by real elapsed time, including fractional
interpolation. This keeps the nominal sequence aligned with commands already
executed by PX4.

## Risk Hierarchy

Rollout eligibility is hierarchical:

1. raw and known-solid collision;
2. worst risk tier;
3. critical exposure;
4. planning exposure;
5. soft motion cost within the eligible class.

Soft cost includes guide deviation, mission progress, early/head progress,
altitude error, speed tracking, acceleration, jerk, yaw motion, terminal error,
and control effort.

Risk tiers are not a continuously varying clearance penalty. Distance is used
to classify static bands and detect collision.

## Static And No-Static Profiles

Static mode uses a longer horizon, larger target lookahead, larger lattice
window, and higher cruise/cap because the city geometry is known.

No-static mode uses a shorter horizon and lower acceleration, jerk, cruise, and
absolute speed limits. Unknown space remains traversable, but observed-space
and braking constraints bound speed.

Exact defaults live in `config/urban_mvp.yaml`.

## Post-Update Validation

The weighted control update can produce a nominal horizon different from every
individual sampled rollout. The reconstructed horizon is therefore classified
again for raw collision, known-solid collision, and risk exposure.

Collision results activate the braking fallback. The post-update
classification also remains available in diagnostics to expose invalid updates
without hiding them inside aggregate cost.

## Continuity And Liveness

- The first command is bounded relative to applied-control feedback.
- Early predicted progress is measured separately from terminal progress.
- Neighboring horizons are compared through warm-start continuity metrics.
- Persistent prediction without actual displacement triggers reseeding and can
  release a stalled global guide.

## Performance

The supported benchmark is:

```bash
make mppi-benchmark \
  MPPI_BENCHMARK_ARGS="--scenario urban_blocks --rollouts 8192 --steps 80"
```

Production diagnostics separate noise generation, rollout simulation, risk
reduction, control update, horizon reconstruction, and total host/GPU time.
ESDF build latency is measured separately from the control tick.
