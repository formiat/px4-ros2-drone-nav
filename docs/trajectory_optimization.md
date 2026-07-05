# Trajectory Optimization

The trajectory optimizer turns a corridor-constrained centerline into smoother,
more trackable geometry. It runs after A* and corridor construction because A*
is good at finding connectivity, but grid paths are too angular for high-speed
offboard tracking.

## Inputs

The optimizer receives:

- corridor samples;
- route centerline;
- prohibited/planning grid validation data;
- trajectory optimizer configuration;
- speed profile configuration for diagnostics.

## Active Windows

The optimizer does not need to change every sample equally. It builds active
windows around areas that need improvement, for example:

- blocked or near-blocked centerline spans;
- heading changes;
- curvature-heavy regions;
- sharp width changes;
- corridor asymmetry changes;
- areas where the centerline is not safe enough.

Active-window diagnostics are important when optimizer time is high. If nearly
all samples are active, candidate evaluation cost rises.

## Offset Candidates

Candidates are lateral offsets from the centerline inside the corridor. The
optimizer evaluates combinations of offsets and chooses geometry that stays
valid and improves smoothness.

The dynamic-programming pass uses coarse and fine offset steps. Important
parameters include:

- `trajectory_optimizer_dp_offset_step_m`
- `trajectory_optimizer_dp_coarse_offset_step_m`
- `trajectory_optimizer_dp_fine_offset_step_m`
- `trajectory_optimizer_dp_fine_radius_m`

## Current Optimization Criteria

The optimizer primarily values smooth, trackable geometry:

- curvature cost;
- curvature-change cost;
- preferred minimum radius;
- radius-shortfall penalty;
- offset-change penalty;
- offset-second-change penalty;
- offset-slope penalty;
- collision/outside-grid rejection.

Length remains a geometric measurement, but it is not the primary objective for
choosing a good trajectory.

## Candidate Rejection

Candidates can be rejected when they:

- leave the corridor;
- cross prohibited cells;
- go outside the grid;
- worsen important curvature/radius constraints;
- fail traversability validation.

Candidate diagnostics are written in planner logs and trajectory diagnostics.

## Turn Smoothing Role

Turn smoothing is a local repair stage after the optimizer. It is responsible
for residual sharp turns that are easier to fix with a local curve candidate
than by changing the global offset optimization.

It considers entry/exit distances, outer shift, relaxed angles, curvature jump,
minimum radius, and traversability.

## Isolated Spike Smoothing Role

Isolated geometry spike smoothing handles single-point or very local curvature
spikes. It should not change endpoints and should only apply when it improves
the local shape while preserving traversability.

## Baseline And Optimized Trajectories

The code still has baseline/refined concepts internally:

- baseline trajectory: corridor-derived trajectory before full optimization;
- refined/optimized trajectory: trajectory after optimizer and smoothing.

The standard configuration publishes the computed optimized executable
trajectory directly. Async refinement support exists in code, but
`trajectory_optimizer_async_refinement_workers` is `0` by default.

If async refinement is enabled later, handover must remain continuity-aware:
projection, tangent, curvature, speed, and command discontinuity must be
validated before replacing the active trajectory.

## Timing

Optimizer timing is visible in planner summary logs and trajectory diagnostics:

- active-window detection;
- window evaluation;
- DP cost;
- final score;
- candidate evaluation count;
- worker/chunk metrics.

See `performance.md` for optimization guidance.
