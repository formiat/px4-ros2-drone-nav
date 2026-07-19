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

## Local Passage Insertion Role

Local passage insertion is not part of the global smoothness optimizer. It is an
optional geometry repair stage for known annotated passages. If the final XY
trajectory intersects a known structure footprint but misses the opening
corridor, the stage may build a local Hermite/Bezier-style segment through that
opening and stitch it into the trajectory.

The stage is intentionally conservative:

- it is enabled by default;
- it only acts on known-passage validation misses;
- it does not score by length or traversal time;
- it rejects candidates that cross prohibited cells;
- it rejects candidates with excessive tangent or curvature discontinuity at
  stitch points;
- it leaves the original trajectory unchanged when no safe candidate exists.

Diagnostics report candidate counts, rejection reasons, accepted/rejected
opening ids, lateral miss before/after, and join metrics.

## Executable Trajectory Build

The runtime publishes only a complete optimized executable trajectory. It does
not publish a corridor-derived baseline and replace it with a refined path
later.

The planner uses one dedicated latest-wins worker for the complete planning
transaction: grid construction, current-path inspection, A*, smoothing,
corridor construction, passage insertion, vertical profile, and speed profile.
ROS callbacks only update short immutable pose, lidar, and memory snapshots or
enqueue a worker request. The currently accepted trajectory remains active
until the new result is complete.

The worker starts A* at a predicted acceptance station on the currently
executed trajectory. Before publication, it reads a fresh pose and requires the
candidate to be directly compatible or handover-compatible with the active
trajectory on the latest validation grid. A stale, unjoinable candidate is
discarded and replaced by a fresh worker request.

## Timing

Optimizer timing is visible in planner summary logs and trajectory diagnostics:

- active-window detection;
- window evaluation;
- DP cost;
- final score;
- candidate evaluation count;
- worker/chunk metrics.

See `performance.md` for optimization guidance.

## Optimization Philosophy

The optimizer is now a smooth-trajectory optimizer, not a racing-line
optimizer. The practical priority is:

1. valid inside hard safety constraints;
2. smooth enough for the drone to track;
3. large local radius where the corridor permits it;
4. low curvature jump at joins and transitions;
5. reasonable path length as a secondary side effect.

This philosophy matters when tuning weights. A shorter path can be worse if it
creates small-radius turns, abrupt curvature changes, or a path that makes the
velocity follower oscillate. The A* route and corridor already keep the path
globally reasonable. After the corridor, the optimizer should spend its freedom
on smoothness rather than shaving distance.

## Active-Window Interpretation

An active window is a promise that changing offsets in that region is worth the
CPU cost. Activating the whole path should be rare. A long straight street with
constant corridor asymmetry should not be treated the same as a blocked span or
a sharp turn.

Strong active-window reasons:

- centerline blocked by planning or prohibited space;
- bottleneck near the centerline;
- large heading span;
- local curvature or curvature-jump problem;
- sharp corridor-width change;
- detected blocked span from diagnostics.

Weak active-window reasons:

- constant asymmetry over many samples;
- wide corridor with no meaningful heading change;
- small width noise that does not affect available radius;
- regions already smooth and far from obstacles.

Performance analysis should always start with active sample count. If active
samples are nearly all samples, candidate count and scoring cost will be high
even if each candidate is individually cheap.

## Candidate Lifecycle

A candidate usually goes through these conceptual steps:

1. choose the affected sample or window;
2. generate an offset or local curve variation;
3. check basic geometry and corridor bounds;
4. check prohibited/planning-grid validity;
5. compute smoothness metrics;
6. compare score against the current best;
7. record accepted or rejected diagnostics.

Candidate logs should explain both invalid candidates and merely worse
candidates. This is important when RViz shows a sharper turn than expected. The
answer might be that the smoother candidate crossed prohibited space, that it
joined with a large tangent jump, or that weights still preferred a tighter
path.

The useful candidate fields are:

- candidate window or corner index;
- entry and exit distances;
- lateral shift or offset;
- minimum radius before and after;
- maximum curvature jump before and after;
- hard validation result;
- score contribution summary;
- rejection reason.

## Scoring Terms

Radius shortfall penalizes turns below the preferred minimum radius. It is the
most direct way to tell the optimizer that visually tight turns are undesirable
even if they are valid.

Curvature cost penalizes high curvature in general. It discourages tight arcs,
but it does not by itself guarantee smooth transitions.

Curvature-change cost penalizes sudden steering changes between neighboring
samples. This helps avoid shapes that have a large radius in the middle but a
sharp entry or exit.

Offset-change and second-change costs keep the chosen lateral offset from
zigzagging inside the corridor. They are especially useful when the corridor is
wide and many offset choices are valid.

Length is still measured because it is useful geometry, but it should not be a
dominant post-corridor cost. The project values a longer smooth curve over a
shorter angular one.

## Turn Smoothing Diagnostics

Turn smoothing should answer:

- How many problematic corners were detected?
- Which corner was attempted first?
- Did it try the next worst corner if the first one had no acceptable repair?
- How many candidates were built?
- Which candidates were hard-rejected?
- Which candidates were valid but worse?
- Which accepted candidate improved local radius or curvature jump?

If turn smoothing becomes expensive, the first fix is usually not to make it
stricter. A strict rejection rule can make it search many more candidates for a
small visual improvement. Prefer scoring valid candidates by smoothness and
accepting a good-enough improvement over spending seconds searching for a
perfect local curve.

## Safe Performance Work

Exact optimizations are preferred:

- reserve and reuse buffers;
- cache exact collision checks;
- cache exact segment geometry where inputs match;
- reuse clearance fields built from the same grid;
- avoid computing diagnostics that no longer affect scoring.

Behavior-changing optimizations must be introduced with shadow metrics first.
Examples include adaptive candidate spaces, lower-bound pruning, local speed
profile recompute, and top-N full scoring. Previous experiments showed that an
optimization can improve timing but degrade smoothness, so winner mismatch and
trajectory-quality checks are part of the optimization contract.
