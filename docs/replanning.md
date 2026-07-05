# Replanning

Replanning is triggered when the current trajectory is no longer safe or when
the planner needs to build a path from updated state.

## Hard Replan Trigger

The main hard trigger is intersection with the current prohibited grid. The
prohibited grid is the hard-safety grid after raw obstacle sources are merged
and inflated.

Planning clearance is different. It is an extra planner margin used to prefer
safer trajectories. Entering the planning-clearance margin is not by itself a
runtime replan reason.

## Runtime Validation

The planner periodically checks the current path/trajectory against updated
obstacle data. Relevant diagnostics include:

- prohibited intersection;
- blocked spans;
- grid bounds;
- current projection on trajectory;
- distance to blocked span;
- path id and path stamp.

## Failed Replan Behavior

If a new candidate trajectory is invalid, stale, or discontinuous, the offboard
node can reject it and keep the previous accepted trajectory. This prevents a
failed update from deleting a still-usable trajectory.

When the planner explicitly publishes an empty hold path after failure, the
offboard node enters hold/fallback behavior instead of treating the empty path
as a normal executable trajectory.

## Trajectory Continuity

Offboard trajectory update continuity checks:

- projection jump;
- tangent jump;
- curvature jump;
- speed-limit jump;
- tangent-speed command jump.

The result can be:

- preserve smoother history;
- reset smoother history;
- reject candidate trajectory.

## Diagnostics Matching

Planner diagnostics are matched to the accepted trajectory by `path_stamp_ns`.
The accepted planner path id is confirmed from matching diagnostics. This
avoids relying on cross-topic delivery ordering between `/path_id`,
`/trajectory_diagnostics`, and `/path`.

## Local Repair Future Work

The current production path uses full replanning. A future optimization could
add local repair:

- find blocked spans on the current executable trajectory;
- replan or repair only a local window around the blocked span;
- stitch prefix + repaired segment + suffix;
- validate continuity and prohibited-grid safety;
- run full global replan in parallel as fallback.

Local repair should be introduced only with strong validation and diagnostics,
because it must not produce unsafe stitching artifacts.
