# Receding-Horizon Updates And Guide Replacement

The current stack does not perform legacy full/partial path replanning.

## Local Horizon Updates

GPU MPPI recomputes a local horizon at the configured planning frequency. It
uses:

- current pose and velocity;
- the latest complete ESDF revision;
- a matching CPU/GPU local-world generation;
- the active global lattice guide;
- the previous control sequence as a warm start;
- the latest applied-control feedback.

Only a fresh timestamped horizon is executable. Offboard never continues an
expired horizon as though it were a long accepted route.

## Sticky Global Guide

The global lattice guide is persistent across ordinary ESDF revisions. A newly
calculated guide does not replace it merely because it has a slightly different
score.

The active guide can be released when it is:

- blocked by current raw occupancy;
- exhausted below the mode-specific remaining-distance threshold;
- too far from the current vehicle position;
- stalled according to along-guide progress;
- superseded by mission-goal completion.

The guide is revalidated on each immutable world revision. If its remaining
risk becomes worse than the level at which it was accepted, the guide remains
executable while a background replacement search starts. A replacement is
activated only after validation; risk degradation does not become a movement
prohibition.

An unchanged scene remains live through the obstacle-memory heartbeat without
pretending that ESDF content changed. Raw-map revisions and local-world
generations therefore remain stable until new content or a pose-driven local
window recenter produces a new immutable planning snapshot.

A replacement search prepares both direct and incremental-topology candidates
when available. Geometry optimization, final certification, and dynamic handoff
all precede arbitration, so a preferred candidate rejected at the final check
does not hide an executable fallback. `selected_index == nullopt` means that no
route is eligible.

Accepted incremental-topology strategy is persistent separately from its local
geometry. `StrategicRouteManager3D` owns the complete intent/corridor, a stable
plan identifier, and a monotonic progress cursor. The lattice planner consumes
that state by materializing only the next finite segment. A partial mission
continuation remains strategic even though that segment does not yet reach the
mission target; direct routes compete as independent plan-level candidates.

The selected replacement is activated from one immutable bundle containing the
resident CPU occupancy/ESDF/topology generation, the linked GPU ESDF revision,
current raw occupancy, objective, pose, and applied control. Final validation
starts at the current route projection: it checks the connector and remaining
suffix, then runs dynamic handoff from the same pose/control snapshot. If newer
raw content has not yet produced its matching local-world generation,
activation waits for that generation instead of mixing revisions.

Heading bias for a replacement guide comes from velocity at speed, the previous
accepted-guide tangent at low speed, or mission-goal direction as the final
fallback. Vehicle yaw is not a global-search direction constraint.

## Liveness

The liveness monitor distinguishes predicted progress from actual vehicle
motion. A horizon that repeatedly predicts useful terminal motion while the
vehicle remains stationary is considered ineffective.

Recovery can:

- reseed the local MPPI nominal control sequence;
- release a stalled active guide so the lattice can build another guide;
- hold when no executable route exists while route search continues.

Even with an available route, a failed physical validation of the next local
horizon checks both the unchanged remaining trajectory of the previously
published finite path and its remaining controls from the measured state. If
either continuation became invalid, the planner may
rebuild its unexecuted control intent from the measured state and embed endpoint
deceleration within the remaining duration. Either continuation is raw-world
validated and may continue only up to the previous deadline. If it is expired
or no longer physically executable, a `no_executable_horizon` position hold
supersedes it. A newly validated finite path releases the hold immediately; low
clearance by itself does not activate it.

The current lattice still has limited recovery primitives. The accepted
topological corridor and its progress are persistent, but there is not yet a
region-portal multigraph with split/merge lineage and fair frontier scheduling.
A dead end may therefore end in hold rather than a complete route around the
obstacle.

## Safety During Updates

There is no prefix/suffix stitch or safe truncation. Safety comes from:

- short overlapping horizons;
- collision evaluation against the active raw occupancy/ESDF contract;
- first-control continuity relative to the command actually sent;
- zero terminal speed at an unresolved route frontier;
- horizon validity deadlines.

Every newly published planned path is finite and self-contained. Its existing
control slots include the deceleration required to reach zero speed at the final
point; no continuation is appended after that endpoint. Receding-horizon updates
normally replace the path before its endpoint, but if updates cease the last
valid path still terminates at rest.

Offboard tracks both the finite path geometry and its velocity/acceleration
feed-forward. After the final zero-velocity sample it holds that same terminal
position; path expiry does not create a second motion phase.

If a replacement route is invalid, the system publishes a typed
`no_executable_route` position hold. It does not preserve a physically blocked
old trajectory or fly directly toward the mission goal just because a new plan
failed. A finite path that reaches its deadline is already at rest, after which
offboard holds its terminal point.

## Removed Protocols

The following concepts are not part of production runtime:

- partial A* repair;
- repair margins and parallel repair races;
- blocked-span suffix stitching;
- safe truncation;
- truncation generation or fingerprints;
- moving/after-hold successor negotiation;
- path-id ACK/coalescing lifecycle.
