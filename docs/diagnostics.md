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
`known_passage_openings`, `min_actual_passage_clearance`, and
`min_actual_passage_volume_margin` in `Mission summary` and `MISSION_RESULT`
lines. The clearance metric is the minimum of lateral and vertical clearance
while the actual vehicle position is inside an opening. The volume margin also
includes depth to the entry/exit plane, so it is expected to approach zero for
a normal traversal and must not be interpreted as wall clearance.

The same summary and result lines identify the sample responsible for the
minimum volume margin using:

- `min_actual_passage_volume_opening`
- `min_actual_passage_volume_boundary`
- `min_actual_passage_volume_components=[depth=... lateral=... vertical=...]`
- `min_actual_passage_volume_position=(x, y, z)`

Boundary names distinguish `depth_entry`, `depth_exit`, `lateral_negative`,
`lateral_positive`, `vertical_lower`, and `vertical_upper`. Entry and exit are
defined by the opening normal: entry is the negative-normal plane and exit is
the positive-normal plane. These fields make a small depth-plane margin
distinguishable from a small clearance to the lateral or vertical solid
geometry.

At mission completion, one `actual_passage_opening_metrics` line is emitted per
annotated opening. It contains the opening id, whether it was seen, the number
of actual samples inside it, and independent minimum lateral, vertical, depth,
geometric, and volume margins. It also reports the boundary, world position,
local depth/lateral coordinates, and all three component margins at the sample
that produced that opening's minimum volume margin. These fields are runtime
diagnostics only:
ordinary building collision volumes still define whether the drone hit a solid
part of a building, while an opening is just free space between those solid
volumes.

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
- `closer_side_static_suppressed`, `closer_side_static_pending`, and
  `closer_side_static_confirmed`;
- `detached_obstacles_confirmed` and `ambiguous_expired`;
- `opening_boundary_pending`, `opening_boundary_confirmed_static`,
  `opening_boundary_confirmed_obstacle`, and
  `opening_interior_obstacles_integrated`;
- ignored hit counts for `left`, `right`, `lower`, and `upper` masses;
- the first ignored structure/opening/part identity and range delta.

Bounded opening-boundary samples include endpoint XYZ, opening Z bounds,
opening margin, nearest solid part, signed solid distance, configured boundary
tolerance, evidence count, and the final pending/confirmed decision.

When a raw obstacle-memory cell inside or within 2 m of a known passage
structure first transitions to occupied, `obstacle_memory_node` emits an
unthrottled `PASSAGE_MEMORY_HIT` event. It records the exact memory cell, lidar
beam index, projected endpoint XYZ, measured range, score transition, occupied
threshold, independent trigger scans, vehicle pose and attitude, map-frame ray,
and known-static classifier result. Match the
event's cell with `raw_sources nearest_cell` in a later prohibited-replan log to
identify the original 3D hit even though `/drone_city_nav/obstacle_memory_grid`
itself is intentionally a 2D raw-evidence grid. Repeated hits on an already
occupied cell do not emit another transition event.

Every occupied transition is also written to the bounded JSONL dump configured
by `lidar_memory_hit_dump_*`. The simulator runner assigns a separate file at
`log/lidar_memory_hits/<run-id>.jsonl` for each run. Each row contains the raw
beam range/index/acquisition timestamp, callback-time vehicle pose and
attitude stamps, the map-frame ray origin/direction/endpoint, XY motion
compensation, ground and known-static expected-surface candidates, the selected
ingestion decision, and the active cell's endpoint-Z provenance. Use this dump
to distinguish a range/classification error from pose/attitude timing error.
It is diagnostic output only: it never feeds memory scoring, planner grids, or
flight control.

Persistent correlation no longer depends on finding that earlier event in the
same log. `/drone_city_nav/obstacle_memory_provenance` stores the occupancy
trigger, latest accepted hit, endpoint Z range, and hit count for every active
occupied cell as a standalone diagnostics topic. Runtime planning consumes
`/drone_city_nav/obstacle_memory_snapshot`, which carries that provenance and the
raw grid atomically in one ROS message.

The planner validates the complete pair before replacing its current memory
state. A memory-sourced prohibited-replan log therefore emits
`memory_provenance[status=matched ...]` immediately, including endpoint XYZ,
attitude, measured/expected range, delta, selected surface, trigger score before
and after occupancy, occupied threshold, independent trigger scans, and hit
history. A
subscriber backlog may replace old queued messages under KeepLast(1), but every
delivered grid remains bound to its own provenance. If the nested pair is
malformed or inconsistent, the entire update is ignored and the previous valid
snapshot remains authoritative; planning never accepts an unauditable grid.

The atomic transport is observable independently from mapping behavior. Every
producer publication logs `sequence`, grid stamp, publication interval,
assembly time, occupied/provenance record counts, and whether the standalone
debug topics were also published. The periodic `Obstacle memory snapshot
budget` record reports the full serialized atomic-message size, standalone
provenance size, grid cells, effective publish rate, and configured budget
status.

The planner parses snapshots in a dedicated callback group, independently from
long A*/optimizer work. The callback keeps only the newest complete parsed pair;
the planning timer atomically adopts it immediately before building the next
planning grid. This prevents planner computation from blocking DDS reception
without allowing an active grid to change during one planning cycle.

Every queued and applied snapshot is logged with the same sequence and stamp.
Diagnostics distinguish receive age, apply age, callback time, apply delay,
DDS sequence gaps, and intentional pending replacements. Periodic `Planner
memory snapshot budget` records report receive/apply rates and maximum
age/callback/apply delay. Every prohibited-replan diagnostic includes the
active and pending sequence, so the exact pair used by the current planning
cycle can be distinguished from a newer pair received while that cycle was
running.

Standalone `/obstacle_memory_grid` and `/obstacle_memory_provenance` remain
debug/RViz/bag outputs, but are rate-limited independently. They are not planner
inputs and their lower cadence cannot change runtime planning. The authoritative
atomic snapshot continues to publish on every accepted memory scan update.

`/drone_city_nav/raw_memory_obstacle_points_3d` is published in that same
standalone debug cycle. It contains one exact occupancy-trigger XYZ for every
active provenance record whose endpoint is valid and finite. It deliberately
does not contain inflation, historical removed cells, `last_hit`, or the full Z
range. Compare it with the unchanged ground-plane
`/drone_city_nav/raw_memory_obstacle_points` layer to see both the 2D cell used
by planning and the 3D lidar observation that originally occupied it. This
cloud is visualization-only and must not be used as a planner or control input.

For retained current-lidar evidence, prohibited-intersection logs additionally
include a bounded `known_static_hit` record when it is available. It identifies
the matched structure/opening/part, grid cell, endpoint XYZ, measured range,
expected range, and signed range delta. Obstacle memory logs the equivalent
bounded retained-hit diagnostic for cross-process correlation.

Every occupied current-lidar overlay cell also retains one accepted-hit record
for the lifetime of that scan. If such a cell is the exact or nearest raw source
of a prohibited-path blocker, the replan log emits `current_lidar_hit` with the
cell, ingestion action/reason/selected surface, measured/expected range and
delta, endpoint XYZ, map-frame ray, and source/applied attitude. This record is
independent of known-static matching, so a closer-than-ground blocker remains
auditable even when no known building surface was involved.

Both lidar ingestion paths publish throttled summaries of the shared decision.
Each diagnostic class has an independent bounded sample budget, so a frequent
class cannot hide a rarer closer obstacle, ambiguous beam, unavailable
classifier, or altitude rejection. The memory and planner logs include one
representative sample for every class observed in the reporting interval:

- `expected_ground`: expected ground beams suppressed without hit or free
  updates;
- `closer_retained`: obstacle returns clearly before the nearest expected
  surface;
- `ambiguous_ground`: ambiguous ground or tied-surface beams suppressed;
- `ground_unavailable`: beams for which configured ground classification could
  not run because required geometry/configuration was invalid;
- `ground_disabled`: beams processed while ground rejection was intentionally
  disabled;
- `non_ground_altitude_rejected`: projected-altitude vetoes not explained by a
  ground candidate.
- `ambiguous_known_static`: a near/inside-solid or boundary observation held
  without hit or free-space mutation;
- `opening_obstacle`: an ordinary unknown obstacle measured inside the free
  opening volume and integrated immediately.

The bounded `lidar decision samples` log includes reason, expected-surface kind,
beam index, endpoint XYZ, measured/expected ranges, endpoint relation, signed
solid distance, opening margin, distance before solid, incidence angle,
timestamp-aligned pose status, evidence count, viewpoint translation/direction
delta, resolution, and provider status. Suppressed observations remain
aggregate/sample diagnostics only.

Accepted obstacle-memory hits persist the compact ingestion decision that
admitted the hit: action, reason, selected expected surface, expected range, and
range delta. `ObstacleMemoryProvenance` schema version 2 carries this snapshot
with both the occupancy trigger and last accepted hit. The planner only applies
it after the existing exact stamp, frame, grid geometry, and grid-content match.

The classifier is independent of the active trajectory and drone proximity to
an opening. Opening and detached obstacles continue through normal prohibited
grid and replan handling; unresolved static-attached evidence does not mutate a
grid.

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
- known-static lidar ignored/unexpected/ambiguous counters and first matched
  physical-solid identity.

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
