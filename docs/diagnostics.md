# Logging And Diagnostics

Diagnostics are a first-class part of the project. Planner and controller
behavior should be debugged from logs and artifacts, not only from RViz.

## Main Log Locations

- `log/offboard_blackbox.jsonl`
- `log/final_trajectory_samples/`
- `log/corridor_samples/`
- `log/lidar_debug/`
- `log/gazebo_scene_debug/`
- `log/gz_drone_nav.log`
- `log/gz_gui_drone_nav.log`

## Offboard Blackbox

`log/offboard_blackbox.jsonl` mirrors runtime telemetry. It includes:

- path ids and accepted path stamp;
- current pose and velocity;
- cross-track error;
- heading error;
- commanded target motion;
- desired and smoothed velocity setpoints;
- attitude and tilt;
- nearest prohibited-cell information;
- terminal state data.

Use this file for control-quality analysis.

## Final Trajectory Dumps

`log/final_trajectory_samples/` contains timestamped trajectory CSV files and a
`latest.csv` shortcut. Summary JSON files sit next to the CSV files.

These files are useful for:

- geometry inspection;
- trajectory altitude inspection through the `z_m` column;
- speed-profile constraints;
- curvature spikes;
- accepted trajectory id and stamp;
- reproducing visual artifacts seen in RViz.

## Corridor Dumps

`log/corridor_samples/` contains corridor samples for each rebuild. Use it to
debug narrow passages, asymmetric corridors, and centerline recovery behavior.

## Trajectory Diagnostics JSON

The planner publishes `/drone_city_nav/trajectory_diagnostics`. Diagnostics are
matched to the accepted path by `path_stamp_ns`, not by assuming topic delivery
order. The accepted planner id is confirmed from matching diagnostics.

Important fields:

- total build duration;
- grid/corridor/optimizer/turn-smoothing timings;
- active windows;
- candidate counts;
- speed-profile top constraints;
- known-passage validation summary and capped per-span diagnostics;
- local passage insertion summary and capped candidate diagnostics;
- fingerprints for speed profile construction and runtime policy/control.

Only construction fingerprint mismatch is a warning. Runtime mismatches are
context because offboard-only control settings can legitimately differ from
planner settings.

Known-passage validation fields include:

- `known_passage_validation_enabled`
- `known_passage_validation_valid`
- `known_passage_structures_checked`
- `known_passage_structures_intersected`
- `known_passage_opening_matches`
- `known_passage_violations`
- `known_passage_validation_reason`
- `known_passage_diag_count`

Each `known_passage_diagN_*` entry reports structure id, opening id, entry/exit
`s`, overlap, clearance, reason, and validity for one capped span.

Mission monitor logs also report `actual_passage_openings_seen`,
`known_passage_openings`, and `min_actual_passage_margin` in `Mission summary`
and `MISSION_RESULT` lines. These fields are runtime diagnostics only: ordinary
building collision volumes still define whether the drone hit a solid part of a
building, while an opening is just free space between those solid volumes.

Local passage insertion fields include:

- `passage_insertion_enabled`
- `passage_insertion_applied`
- `passage_insertion_candidates`
- `passage_insertion_inserted_count`
- `passage_insertion_rejected_join`
- `passage_insertion_rejected_traversability`
- `passage_insertion_rejected_validation`
- `passage_insertion_rejected_geometry`
- `passage_insertion_reason`
- `passage_insertion_duration_ms`
- `passage_insertion_diag_count`

Each `passage_insertion_diagN_*` entry reports structure id, opening id,
anchor/entry/exit/reconnect station, lateral miss before/after, join tangent
and curvature metrics, rejection reason, and whether the candidate was accepted.
These fields are intended for headless debugging when a known-passage XY repair
is possible but not selected.

The obstacle-memory update and planner summary report the always-on 3D known
static lidar classifier:

- `expected_static_hits_ignored` / `ignored`;
- `unexpected_hits_kept` / `unexpected`;
- `ambiguous_hits_kept` / `ambiguous`;
- ignored hit counts for `left`, `right`, `lower`, and `upper` masses;
- the first ignored structure/opening/part identity and range delta.

The classifier is independent of the active trajectory and drone proximity to
an opening. Unexpected and ambiguous hits continue through normal prohibited
grid and replan handling.

## Replan Diagnostics

Planner logs include replan reasons and blockers, for example:

- prohibited intersection;
- path stale/invalid;
- A* failure;
- no valid trajectory;
- diagnostics mismatch;
- invalid or rejected refined trajectory.

## Important Metrics

For flight quality:

- speed;
- tilt;
- roll/pitch;
- cross-track error;
- signed cross-track error;
- normal velocity;
- commanded normal velocity;
- desired-to-setpoint velocity error;
- jerk limiter activity;
- terminal state.

For planner quality:

- total path build wall-time;
- grid build time;
- A* time;
- corridor time;
- trajectory optimizer time;
- turn smoothing time;
- local passage insertion time;
- speed profile time;
- replan count.
- known-passage validation result if the trajectory intersects an annotated
  structure footprint.
- passage traversal sensor policy state when flying through an annotated
  opening.

## Last Run Analysis Checklist

1. Check whether the mission completed or failed.
2. Check replan count and reasons.
3. Check accepted trajectory geometry.
4. Check top speed constraints.
5. Check cross-track and normal velocity around bad segments.
6. Check terminal state separately from cruise flight.
7. Compare RViz colors to trajectory CSV and diagnostics JSON.

## Diagnostic Philosophy

Diagnostics should answer "why" rather than only "what happened". A useful log
does not merely say that a candidate was rejected; it says whether it crossed
prohibited space, had a poor radius, worsened curvature jump, failed a
continuity gate, or arrived with stale pose.

The project has several asynchronous artifacts, so correlation matters. A path,
path id, trajectory diagnostics, offboard blackbox row, and RViz marker may
arrive at different times. Prefer stable keys:

- path timestamp for trajectory diagnostics;
- accepted planner path id for human-readable correlation;
- navigation time for flight metrics;
- sample index and trajectory `s` coordinate for geometry diagnostics.

When a log line has no stable key, it is much less useful for post-run
analysis.

## Planner Timing Summary

Planner timing should be read in wall-time stages:

- grid and inflation;
- clearance field;
- A* rough route;
- corridor construction;
- trajectory optimizer;
- turn smoothing;
- speed profile;
- publication.

Parallel candidate stages can report aggregate CPU time that is much larger
than wall time. That is not a bug. Aggregate time answers how much work was
done across workers; wall time answers how long the mission waited.

For optimization decisions, always ask:

- Is this a wall-time bottleneck or only aggregate CPU?
- Does the stage affect trajectory quality?
- Is the proposed optimization exact or behavior-changing?
- Did a previous shadow-mode test show winner mismatches?

## Trajectory Quality Diagnostics

Important trajectory quality fields include:

- minimum radius and radius-shortfall metrics;
- maximum curvature and curvature jump;
- speed-profile constraints and their sources;
- active optimizer sample count;
- candidate rejection reasons;
- turn-smoothing accepted/rejected candidates;
- blocked spans and centerline blocked diagnostics.

RViz color is useful but not enough. A purple segment can mean a tight radius,
a curvature spike, a speed-profile constraint, or a local artifact. Use the
numeric source of the speed constraint and local curvature metrics to determine
the actual reason.

## Control Diagnostics

Control diagnostics should separate desired command, smoothed setpoint, and
actual vehicle motion. For lateral behavior, inspect:

- signed cross-track error;
- normal velocity relative to the trajectory;
- desired normal velocity;
- smoothed setpoint normal velocity;
- actual normal velocity;
- P factor and effective D factor;
- curvature feedforward;
- projection smoothing mode;
- smoother acceleration and jerk limiting.

For 3D trajectory representation checks, inspect `projection_z_m` in the
blackbox tracking object and the accepted trajectory altitude range in the
`Received executable final trajectory` log. These fields describe trajectory
representation altitude; runtime vertical velocity still follows the cruise
altitude hold controller in this stage.

If desired command is already wrong, fix projection, feedforward, or lateral
controller tuning. If desired command is right but the setpoint is clipped, fix
smoother limits or speed policy. If setpoint is right but actual velocity is
wrong, inspect PX4 response, vehicle dynamics, or simulator state.

## Replan Diagnostics Checklist

A good replan diagnostic should answer:

- What triggered the replan?
- Which grid or source marked the path invalid?
- Where along the trajectory did the invalid span start and end?
- Was the blocked region ahead of the drone?
- Did the new candidate get accepted, reset, or rejected?
- Did the previous trajectory remain active?
- Were diagnostics matched to the accepted path stamp?

These fields are especially important for lidar-triggered replans because
sensor evidence can be partial. A visually harmless scan artifact should not be
confused with a hard collision unless the prohibited-grid intersection proves
it.

## Terminal Diagnostics

Terminal behavior should be read as a separate state machine:

- cruise;
- velocity terminal capture;
- position capture;
- final hold.

Do not use terminal position-capture behavior to judge normal lateral control.
Position capture intentionally changes the setpoint mode. Conversely, a clean
final hold does not prove the high-speed part of the path was tracked well.

## Run Comparison Method

When comparing two runs, use the same route and similar simulator conditions if
possible. Then compare:

- total planning wall time;
- trajectory optimizer wall time;
- turn smoothing wall time;
- final trajectory minimum radius;
- max curvature jump;
- p90 and p95 cross-track error;
- max cross-track error outside terminal capture;
- signed cross-track zero crossings;
- normal velocity zero crossings;
- max tilt and where it occurred;
- replan count and trigger reasons.

Avoid judging a controller change from one number. A change can reduce
oscillation but increase turn miss, or improve geometry but increase planning
time. The docs should record which tradeoff is accepted.
