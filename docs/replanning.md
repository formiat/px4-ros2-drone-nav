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
why it was not applied. An active passage hard window no longer rejects the
update by itself: the builder preserves the old constrained prefix through the
window exit and settle distance, then reconnects outside the window. Different
active passage ids are rejected as `hard_window_prefix_not_reconnectable`.
Other reasons cover unavailable or invalid trajectories, stale local position,
an already-compatible update, missing validation grid, excessive join distance,
geometry limits, and a non-traversable stitch. A stale
trajectory rejection prints this complete result, including projection
stations, projection and tangent jumps, prefix and join distances, curvature,
and the rejected grid segment.

Replans caused by a prohibited blocker also emit `REPLAN_DELIVERY` lifecycle
events. They correlate blocker detection, trajectory-build start, path
publication, offboard receipt, and late diagnostics receipt by trajectory
generation, path id, and path timestamp. The planner records pose and velocity
at detection and build start and uses the position snapshot taken at the start
of the planning cycle as the A* start. Before publication it compares a
diagnostic constant-velocity publication estimate with the fresh actual pose;
that estimate does not affect planning geometry. Offboard reports
publish-to-receive and blocker-to-receive latency plus its actual and
independently extrapolated receive positions.

## Diagnostics Matching

Planner diagnostics are matched to the accepted trajectory by `path_stamp_ns`.
The accepted planner path id is confirmed from matching diagnostics. This
avoids relying on cross-topic delivery ordering between `/path_id`,
`/trajectory_diagnostics`, and `/path`.

## Local Segment Repair

After safe truncation is confirmed, production replanning races ten local
segment repairs against one full replan. All jobs read one immutable snapshot
containing the accepted trajectory artifact, the absolute blocked span, known
passages, and prepared planning/runtime grids. Local jobs reconnect at fixed
stations 10 through 100 metres after the actual blocked-span exit.

Each local job runs A*, corridor construction, optimization, and smoothing only
from the confirmed truncation point to its reconnect station. It then appends
the unchanged old suffix and globally rebuilds vertical, passage, and speed
metadata. The first completed hard-valid result wins; invalid completions do
not close the race. The full job remains an equal production competitor.

## Replan Decision Philosophy

The system should not treat every obstacle update as a reason to throw away the
whole plan. Full replanning is robust but expensive, and it can create visible
trajectory changes even when only a small future span is affected. Safe
truncation retains the executable prefix while local and full candidates race,
so planning latency does not invalidate the confirmed join point.

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
4. generate configured reconnect stations after the blocked-span exit;
5. run A*, corridor, optimizer, and turn smoothing between anchors;
6. stitch old prefix, repaired segment, and old suffix;
7. rebuild the speed profile for the stitched full trajectory;
8. validate runtime prohibited-grid safety and known solid geometry;
9. publish only if the stitched candidate passes the same acceptance gates as a
   full replan.

This approach is easier than direct corridor surgery because it reuses known
planner stages. Direct corridor repair can be faster later, but it requires
retaining and modifying more internal planner state.

## Parallel Repair And Full Replan

Local repair and full replan run concurrently and every result carries the
truncation generation, blocked path id, prefix fingerprint, and exact planning
grid version. The first hard-valid completion wins. Later results are canceled
cooperatively or ignored as stale.

The implementation does not need unsafe thread termination. Cooperative
cancellation or stale-result rejection is enough for correctness. The
controller should only see accepted executable trajectories, not partial repair
attempts.

## Validation Requirements For Stitched Paths

A stitched path must be treated as a new executable trajectory, not as a small
patch that bypasses validation. It must pass:

- finite geometry checks;
- hard prohibited-grid validation;
- ordered planning-clearance/runtime-prohibited validation;
- mandatory known-solid non-intersection;
- maximum segment length checks;
- speed-profile rebuild;
- offboard trajectory continuity acceptance.

Join tangent and curvature remain quality and speed-profile inputs rather than
automatic hard rejection. Non-finite geometry, runtime-prohibited
intersection, broken endpoints, or known-solid intersection are hard rejects.
