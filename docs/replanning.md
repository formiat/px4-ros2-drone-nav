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

Known passage geometry is handled before a new lidar hit enters either dynamic
source. A confident range/geometry match to a known physical solid is
suppressed. A hit clearly detached from the solid or inside a free opening is
retained immediately. A boundary, low ground candidate, or projection-uncertain
hit performs no hit or free-space update until independent 3D viewpoints
resolve its geometry.
This is always active when valid 3D pose and known geometry are
available, is independent of the current trajectory, and never suppresses a
static-map cell. Any retained hit that inflates into the current path remains a
normal hard replan trigger.

## Runtime Validation

The planner periodically checks the current path/trajectory against updated
obstacle data. Relevant diagnostics include:

- prohibited intersection;
- blocked spans;
- grid bounds;
- current projection on trajectory;
- distance to blocked span;
- path id and path stamp.
- known-static lidar classification counters and first matched solid identity.

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
- defer a high-speed update until a safe transition is available;
- reject candidate trajectory.

Before deferring an XY-discontinuous update, offboard attempts a continuity
handover. It preserves a speed-dependent prefix of the accepted executable
trajectory, joins it to a future station on the candidate, rebuilds geometry
and speed constraints, and validates the complete stitch against the current
prohibited grid. A failed geometry or grid check never bypasses the continuity
gate.

Every horizontal handover result records whether the builder was attempted and
why it was not applied. Reasons cover unavailable or invalid current
trajectory, invalid candidate, stale local position, an already-compatible
update, an unsafe vertical hard window, missing validation grid, excessive
join distance, geometry limits, and a non-traversable stitch. A stale
trajectory rejection prints this complete result, including projection
stations, projection and tangent jumps, prefix and join distances, curvature,
and the rejected grid segment.

Replans caused by a prohibited blocker also emit `REPLAN_DELIVERY` lifecycle
events. They correlate blocker detection, trajectory-build start, path
publication, offboard receipt, and late diagnostics receipt by trajectory
generation, path id, and path timestamp. The planner records pose and velocity
at detection and build start, predicts the publication position without using
that prediction for planning, and compares it with the actual publication
pose. Offboard reports publish-to-receive and blocker-to-receive latency plus
its actual and independently extrapolated receive positions. These fields are
diagnostic only; they do not change the A* start, preserve a prefix, or accept
a candidate that existing continuity checks reject.

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

## Replan Decision Philosophy

The system should not treat every obstacle update as a reason to throw away the
whole plan. Full replanning is robust but expensive, and it can create visible
trajectory changes even when only a small future span is affected. The current
safe baseline is full replan with previous-trajectory retention. The future
direction is to add local repair as a faster candidate path.

The planner should distinguish:

- current trajectory hard-invalid now;
- current trajectory intersects a future blocked span;
- planning clearance is reduced but hard trajectory remains valid;
- lidar evidence is noisy or incomplete;
- the drone is already beyond the blocked region.

These conditions require different responses. A blocked span 80 m ahead can be
handled differently from a prohibited cell under the current vehicle.

## Local Segment Replan Concept

The first practical local-repair implementation should reuse the existing
planning stack on a shorter window. The sequence is:

1. project the drone onto the accepted executable trajectory;
2. find blocked spans against the latest prohibited grid;
3. choose start and end anchors around the affected span;
4. expand anchors by speed and curvature so joins happen in stable regions;
5. run A*, corridor, optimizer, and turn smoothing between anchors;
6. stitch old prefix, repaired segment, and old suffix;
7. rebuild the speed profile for the stitched full trajectory;
8. validate prohibited-grid safety and continuity;
9. publish only if the stitched candidate passes the same acceptance gates as a
   full replan.

This approach is easier than direct corridor surgery because it reuses known
planner stages. Direct corridor repair can be faster later, but it requires
retaining and modifying more internal planner state.

## Parallel Repair And Full Replan

Local repair and full replan can run concurrently if every result is tagged
with a generation id. The first valid result can be accepted. Later results are
accepted only if they still match the current generation and pass continuity
checks. Otherwise they are ignored as stale.

The implementation does not need unsafe thread termination. Cooperative
cancellation or stale-result rejection is enough for correctness. The
controller should only see accepted executable trajectories, not partial repair
attempts.

## Validation Requirements For Stitched Paths

A stitched path must be treated as a new executable trajectory, not as a small
patch that bypasses validation. It must pass:

- finite geometry checks;
- hard prohibited-grid validation;
- planning-clearance or corridor-validity policy;
- tangent continuity at both joins;
- curvature continuity at both joins;
- maximum segment length checks;
- speed-profile rebuild;
- offboard trajectory continuity acceptance.

If any check fails, the repair candidate is rejected and the full replan
remains the fallback. A local repair that introduces a sharp join can be worse
than the original obstacle-triggered problem.
