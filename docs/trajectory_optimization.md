# GPU MPPI Trajectory Optimization

The production local planner is Model Predictive Path Integral control running
on CUDA. There is no corridor-constrained post-processing optimizer.

## Persistent Engine

`MppiCudaEngine` owns persistent CUDA buffers, the resident ESDF texture, and
the nominal control sequence. Per-tick planning does not recreate CUDA
allocations. A generic known-solid API remains in the engine, but the production
3D-world path does not populate it; canonical physical solids are encoded in
Occupancy3D.

## State And Control

The benchmark and production engine use a 3D translational state with yaw:

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
-> query local ESDF3D against physical Occupancy3D
-> reject only physical collisions and flight-envelope violations
-> compute MPPI weights
-> update controls
-> constrain first control against applied feedback
-> reconstruct nominal horizon
-> classify and validate
-> shape an arrival-to-rest profile within the existing finite horizon
-> validate the complete finite path against raw physical obstacles
```

The arrival-to-rest profile occupies existing path samples and is part of the
published path contract. It is not a second execution phase. If a
later optimization tick fails, the unchanged remaining trajectory continues
only when both its geometry and its remaining controls from the measured state
are physically valid. Otherwise
the remaining path may be rebuilt from the measured state, with its arrival
profile shaped again and the complete rebuilt path physically validated.
The previous validity deadline is never extended.

The warm start is shifted by real elapsed time, including fractional
interpolation. This keeps the nominal sequence aligned with commands already
executed by PX4.

## Feasibility And Soft Clearance Cost

Rollout feasibility has a narrow physical contract:

1. every state remains inside the flight envelope and is dynamically recoverable
   before either altitude boundary;
2. the swept physical footprint does not intersect raw occupancy;
3. the generic known-solid collision contract is not violated.

Critical and planning clearance exposure are strong soft costs. Inside the
critical band, a bounded quadratic proximity term additionally distinguishes a
shallow exposure from a trajectory that nearly touches a wall. The term is
integrated over time, so remaining stationary near a wall does not avoid its
cost. These terms rank safe rollouts together with guide deviation, mission
progress, early/head progress, altitude error, speed tracking, acceleration,
jerk, yaw motion, terminal error, and control effort. Low clearance alone cannot
make a rollout unreachable or force a position hold.

Conservative ESDF distance classifies the critical and planning bands and feeds
the soft proximity term. Hard raw collision is reported only when the swept
oriented physical footprint intersects a raw occupied cell; there is no
additional prohibited inflation layer.

## Static And No-Static Profiles

Static mode uses a longer horizon, larger target lookahead, larger lattice
window, and higher cruise/cap because the city geometry is known.

No-static mode uses a shorter horizon and lower acceleration, jerk, cruise, and
absolute speed limits. Unknown space remains traversable, while sensor range
and physical stopping capability bound speed.

Exact defaults live in `config/urban_mvp.yaml`.

## Post-Update Validation

The weighted control update can produce a nominal horizon different from every
individual sampled rollout. The reconstructed horizon is therefore classified
again for physical occupied-cell collision and risk exposure.

Collision results reject the horizon. Route unavailability is handled by a
separate typed position hold. An expired finite path already ends at rest, and
offboard holds that path's terminal point. Post-update classification remains
available in diagnostics without hiding physical invalidity inside aggregate
cost.

The published planned path always ends at a terminal rest state. Its speed
profile reaches zero inside the configured path duration. If the resulting
physical path is invalid, arrival shaping is retried from an earlier path sample;
an old or colliding path is never
executed as a fallback.

Arrival shaping uses
`finite_path_arrival_maximum_horizontal_deceleration_mps2`, a conservative contract
separate from the higher acceleration available to ordinary manoeuvres. This
prevents a finite path from claiming stopping performance that PX4 cannot track.

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
