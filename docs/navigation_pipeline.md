# Navigation Pipeline

The production pipeline converts current vehicle state and obstacle evidence
into a timestamped local trajectory horizon.

## 1. Raw World Snapshot

The selected obstacle-memory node integrates accepted lidar returns into either
a scored 2D memory grid or sparse observed `Occupancy3D`.
`/drone_city_nav/obstacle_memory_status` carries the producer and sequence
heartbeat without copying the grid. The 2D profile publishes
`/drone_city_nav/raw_obstacle_snapshot`; the 3D profile publishes a revisioned
`/drone_city_nav/raw_obstacle_snapshot_3d` base plus
`/drone_city_nav/raw_obstacle_delta_3d` dirty chunks. RViz and provenance
representations are debug-only and are never planner inputs.

In no-static mode the reconstructed raw state is the planning world. The 3D
pipeline publishes the current physical scan first, integrates persistent memory
on a latest-value worker, and transports immutable world revisions at a bounded
rate. Base snapshots are adaptive and intervening updates are cumulative dirty
deltas, so skipped superseded messages do not invalidate the newest state. In
static mode the planner loads canonical Occupancy3D directly and does not merge
the sensor grid into the static 3D map.

Sensor liveness, raw-map content, and local planning state have independent
identities. `LatestObservation` is refreshed by the lightweight heartbeat even
when the scene is unchanged. `RawMapVersion` changes only with reconstructed map
content. A `LocalWorldGeneration` binds one raw version, the pose used to select
the local window, the CPU ESDF, the active GPU ESDF revision, and compatible
topology evidence. A heartbeat never rewrites ESDF content timestamps or map
revisions.

## 2. ESDF Preparation

The production MPPI node prepares a mode-specific occupied-distance field
asynchronously. Static mode extracts a local dense ESDF3D from the precomputed
chunked cache associated with canonical Occupancy3D. Fingerprint or format
mismatch falls back to the exact runtime EDT. No-static 2D mode builds a local
ESDF2D. No-static 3D mode reconstructs the revisioned observed occupancy and
builds a recentered local ESDF3D that retains explicit unknown-space state.
MPPI continues using the last complete immutable field until a newer revision
is ready.

Rate-limited work is retained by a latest-wins deferred scheduler. The newest
pending state is processed when the rate deadline arrives even if no later
sensor message appears. A pose-driven recenter request is urgent and may build a
new local generation before that deadline. Planning starts only when the
captured CPU generation names the ESDF revision active in the GPU engine.

Distance classifications are:

- raw collision: hard reject;
- critical band: highest non-collision risk;
- planning band: elevated risk;
- preferred space: normal risk.

The current defaults are defined in `config/urban_mvp.yaml`; documentation must
not duplicate YAML as a second parameter source of truth.

## 3. Global Lattice Guide

Static mode uses a 3D lattice over physical free voxels and produces
`RouteSample3D` samples. No-static 2D retains the planar lidar-driven lattice.
No-static 3D uses the same generic 3D lattice over observed known-free voxels;
it does not classify a separate online passage domain or consume static
topology.

Lattice output is classified as reached-goal, viable frontier, search
incomplete, or exhausted. Only reached-goal and viable-frontier results are
executable. Incomplete search is continued when possible; exhausted output is
not accepted as a guide.

No-static frontier selection keeps candidates from distinct departure
directions before evaluating continuation depth. Temporary zero or negative
Euclidean progress toward the mission goal is allowed when it provides a
locally viable detour. Goal distance is a soft ranking term, not a frontier
eligibility condition.

An accepted guide is sticky. It is retained across ESDF revisions while its
remaining portion is valid and useful. Replacement reasons include blocking,
exhaustion, excessive cross-track error, and observed stall.

Direct and incremental-topology searches produce at most two plan-level
candidates. Both are materialized, geometry-optimized, certified, rebased onto
one immutable resident generation, and checked for dynamic handoff before route
arbitration. A failed final check makes that candidate ineligible, so another
fully prepared candidate can win; an empty selection publishes no route.

The `StrategicRouteManager3D` owns the complete accepted incremental-topology
intent and corridor under a nonzero monotonically allocated plan identifier. It
also owns a monotonic station/segment cursor, so local replanning cannot move
backward along that corridor. The lattice adapter materializes only the next
finite segment from this persistent plan. A partial mission-continuation segment
therefore retains strategic priority without claiming that its local endpoint is
the mission endpoint. Direct search remains an independent plan-level candidate
in the same preparation and arbitration pipeline.

Semantic frontier discovery belongs only to the incremental topological
planner. The 3D lattice receives a typed strategic directive and materializes
that local target; it does not run a competing observation-frontier search.
Compiled static topology contributes passage traversal evidence directly and
does not instantiate a separate route owner.

Incremental-topology edges and connectors carry typed transition evidence.
`observed_free` means the swept transition is supported without unknown-space
exposure; `optimistic_unknown` records that the same raw-collision-free
transition crosses unknown space. Each record also carries its supporting
segment count, validation revision, last complete geometry revision, and
geometry lineage. Route steps preserve those records and the plan reports their
minimum revisions and aggregate unknown exposure. Unknown remains traversable
and receives no topology cost or execution penalty.

An observation-frontier connector follows the sampled component's stored
parent-cell path from its supporting viewpoint back to the representative. It
does not substitute a straight representative chord that may cross an obstacle
inside a non-convex component. Every segment remains subject to the same raw
swept-footprint validation policy.

Route activation is then a single optimistic transaction over that immutable
resident world plus the jointly captured pose/applied-control snapshot. The
current-pose connector and remaining suffix are swept against the raw occupancy
from the same producer lineage at or beyond the resident ESDF source revision;
this lets final validation consume newer obstacle evidence without mixing the
resident CPU ESDF, GPU ESDF, and topology generation used for planning.
Collisions in an already passed prefix do not reject the route. The same pose
and applied control drive the dynamic handoff simulation. Publication is
abandoned if the resident world, objective, or captured raw snapshot changes
before the route supervisor commits it. Unknown voxels remain traversable
during this raw check.

Initial search heading uses a cascade:

1. velocity heading at normal speed;
2. previous accepted-guide tangent at low speed;
3. mission direction when no accepted guide exists.

## 4. Target And Speed Policy

The planner selects a lookahead point on the active guide. Sensor and map mode
select guide geometry and observation limits, while cruise speed, absolute
speed, and acceleration are explicit map-independent parameters.

Reference speed is bounded by:

- mode cruise and absolute limits;
- curvature preview;
- sensor-observation range and physical stopping capability;
- goal approach;
- an unresolved route frontier, whose terminal speed is zero until an actual
  route extension is accepted;
- constrained-route span limits.

When no executable route exists in either mode, direct flight to the distant
mission goal is forbidden. The planner publishes a typed stationary hold while
route search continues. Direct interception remains valid without a global route
only when the current target is visible and the direct swept path is physically
validated.

Route availability and local-horizon executability are separate contracts. If
the active route or validated direct interception exists but MPPI produces no
physically executable next horizon, the planner first validates the unchanged
remaining trajectory of the previously published finite path against the
current raw world. If that trajectory became invalid, the planner may rebuild
the remaining control intent from the measured state, embed endpoint
deceleration inside the remaining control slots, and validate the complete
rebuilt path. Either continuation retains the previous validity deadline and
ends at rest. Only when neither path is executable does the planner publish a
`no_executable_horizon` position hold. A newly validated finite path supersedes
that hold immediately;
recovery does not wait for the vehicle to become stationary. Loss of the global
route first preserves any still-executable finite path, and clearance tiers do
not trigger this hold.

## 5. Constrained Route Spans

Static air passages are ordinary physical free space represented by traversal
edges in the separately loaded FreeSpaceTopology3D index. Global search explicitly evaluates
`start -> entry -> passage -> exit -> planning goal` topology candidates. A
selected traversal edge creates its constrained station interval directly;
local clearance analysis remains a validation step rather than a
nearest-opening selector or separate passage lifecycle.

## 6. GPU MPPI

Each planning tick:

1. shifts the previous nominal controls by elapsed time;
2. generates CUDA control perturbations;
3. simulates thousands of dynamic-state rollouts;
4. queries the 3D ESDF with the swept oriented physical footprint;
5. rejects physically infeasible rollouts and applies strong soft clearance
   exposure costs to the remaining rollouts;
6. computes the weighted control update;
7. limits the first control relative to applied-control feedback;
8. reconstructs the selected nominal horizon on the host;
9. uses the final control slots of that same path for dynamically bounded
   deceleration, so the endpoint has zero translational and yaw velocity without
   adding samples beyond the configured duration;
10. validates the complete finite path before publication.

MPPI optimizes a short receding horizon. It does not publish a long mission
path for open-loop execution.

## 7. Post-Update And Route-Availability Checks

The reconstructed horizon is validated against the physical occupancy and the
configured flight envelope. Every state must retain enough vertical stopping
room under the configured acceleration and jerk limits, not merely lie inside
the numeric altitude interval. In no-static mode, the newest timestamp-aligned
raw lidar returns additionally validate the complete finite path, independently
of persistent-memory integration latency. A path that intersects raw occupancy
or cannot remain inside the flight envelope is not published. The planner moves
the start of the in-path deceleration earlier until the path validates; it never
appends motion after the finite endpoint. Critical or planning clearance exposure
affects cost and diagnostics, but cannot independently reject motion or latch a
hold.

A failed replacement update does not invalidate the previous path by itself.
The planner first checks its remaining timed trajectory using raw occupancy, the
full swept footprint, and fresh direct lidar returns. The already published path
continues without republishing when that trajectory remains valid. If it does
not, the planner may re-simulate the unexecuted controls from the current state,
reshape the arrival profile in the remaining duration, and validate the
complete rebuilt path. The rebuilt command starts at the current timestamp, but
its deadline never exceeds the previous `valid_until`.

Route validity is checked separately. An unresolved frontier receives a zero
terminal speed so normal speed policy can stop before its endpoint. If no
physically executable route remains, the planner latches the current admissible
position and publishes `no_executable_route` hold horizons until a replacement
route is atomically accepted.

The liveness monitor compares predicted and actual progress. Persistent
prediction without real movement can reseed the MPPI nominal controls and
release a stalled guide.

## 8. Horizon Publication And Execution

The production node publishes the execution horizon immediately after planning
and safety evaluation. RViz, INFO summaries, and JSONL diagnostics run outside
the control-critical publication path. JSONL writes have a configurable rate and
batched flush period; a bounded recent-record ring is dumped when a collision
episode begins.

Offboard consumes the fresh horizon, interpolates by timestamp, and emits PX4
trajectory setpoints. The finite path reaches zero velocity by its deadline;
offboard then holds its terminal point instead of extrapolating motion.

## Removed Legacy Stages

The production runtime no longer contains:

- grid A* path publication;
- corridor construction;
- post-corridor trajectory optimization;
- separate turn smoothing;
- partial-replan races;
- prefix/suffix stitching;
- safe truncation;
- planner/prohibited inflated grids;
- inflation relaxation or escape tunnels;
- the legacy speed planner and terminal-capture path lifecycle.
