# Persistent Route Repair And Receding-Horizon Execution

Production navigation separates persistent strategic route ownership from the
short executable horizon. It does not use the retired full/partial A* repair,
risk-lattice replacement, or online-topology arbitration protocols.

## Persistent Strategic Route

`PersistentDStarLitePlanner3D` is the only production strategic route producer.
It keeps search state across compatible raw-world revisions and incrementally
updates affected graph vertices when occupied voxels change. Unknown space is
traversable and contributes neither a hard gate nor a cost.

The world-fixed graph keeps all minimum-resolution 26-connected edges and adds
aligned 2x/4x overlays by configuration. Exact raw swept-footprint checks lazily
admit every edge, so an open region can be crossed with few states while a narrow
passage remains reachable through the complete fine graph. Occupied updates
invalidate only cached edges whose endpoint reach can intersect the changed raw
cells.

`RouteExecutionManager3D` preserves the mission intent and solely owns the
accepted immutable route plan, full-3D progress, pending successor, and certified
reserve. It also publishes the resident plan, controller lease, exact versioned
execution input, and matching applied-control evidence together as one immutable
`CommittedExecutionAuthority3D`. Readers capture a single atomic pointer, and
writers replace it only when the exact expected authority is still current.
Ordinary world updates do not replace a still-valid route. Planning starts from
the current mission coordinate or from a certified future stitch station;
vehicle yaw is never a strategic search constraint.

## Successors And Suffix Repair

A non-terminal route must retain enough certified suffix for:

```text
stopping_distance + speed * measured_p99_planning_latency + certified_overlap
```

Successor planning begins before that boundary. A successor is compiled and
certified independently, then spliced only where the old and new immutable
geometries have verified overlap. A compare-and-swap conflict requests a fresh
snapshot; it does not clear the current owner.

When newer raw occupied evidence intersects the unexecuted route, validation
keeps the valid prefix and repairs only the affected suffix. If repair cannot
finish before the braking boundary, ownership transfers to the already certified
braking plan. Only exact raw collision evidence may report `raw_collision`;
unknown labels, distance-cache boundaries, or a failed search may not.

## Local Horizon Updates

GPU MPPI recomputes a finite local horizon at the configured control cadence
using:

- the jointly captured pose, velocity, and applied control;
- the exact immutable route geometry and tracking-error tube;
- a coherent CPU/GPU world generation;
- latest raw lidar evidence for final swept-footprint validation;
- the previous control sequence as a warm start.

Route geometry, nominal finite horizon, braking fallback, raw-validation
certificate, owner identity, versioned input, applied-control evidence, and all
evidence revisions cross the publication boundary as one atomic committed
authority. A mixed generation or stale owner fails closed.

Only a fresh timestamped `MppiTrajectoryHorizon` is executable. Offboard tracks
its position, velocity, and acceleration feed-forward and publishes exact applied-
control feedback. A finite path contains its own terminal deceleration; after the
last zero-velocity sample, offboard holds that same terminal position.

## The Planner Update Budget

One planner update runs, in order: the feasibility search, affected-vertex
repair, change scheduling, the D* Lite shortest-path search, and the
execution-time refinement. They shared the update first-come, and between them
the stages that run first took all of it: D* Lite was measured expanding
nothing in over half the recorded updates, and 105 of 166 published routes came
from the unranked feasibility branch.

The tail of the update is now reserved for the shortest-path search
(`persistent_planner_guaranteed_spatial_search_fraction`), and the
execution-time refinement is guaranteed a share of the update's expansions
(`persistent_planner_guaranteed_refinement_expansion_fraction`) rather than
only what the spatial search leaves. Without the second one the refinement runs
only while D* is idle, and the refinement is what turns a first-found route
into a ranked one.

Seeding the refinement with the feasibility route — so that it would improve it
directly — was tried and does not work: the refinement treats its seed as an
anytime *bound*, so a feasibility seed makes it declare at once that it cannot
improve on that route by the admissible margin, and it never expands. What
makes a ranked route exist is the budget, not a different seed.

## Choosing Between Candidates

The planner keeps one incumbent and replaces it when a candidate is better on
ranked execution time. Two rules shape that comparison.

A candidate that leaves the vehicle's own position on a different heading than
the incumbent turns the vehicle around and discards the motion it already has.
Such a candidate must be better by
`persistent_planner_continuity_improvement_margin_s`; one that continues the
same heading replaces the incumbent as soon as it is better at all. Only the
first segment matters, because that is the part the vehicle is flying now.

A release reason no longer discards the incumbent. A route released as blocked
is blocked at one station, not everywhere, and throwing it away restarted the
search from nothing: the feasibility branch then published another first-found
route, which the next observed voxel blocked in turn — the churn that gave
routes a median life of under two seconds. Evidence decides instead: the
incumbent is re-validated against the current world every update and reset when
it no longer clears the body, which is the same outcome whenever the block is
real and on the part still to fly. A consumer that could not enter the
incumbent it was delivered says so through the rejection sequence, and that
does reset it.

## Published Route Geometry

Every published route — from the feasibility search and from the execution-time
refinement alike — goes through the same two passes before it is offered:

1. **Shortcut simplification.** A lattice zig-zag left in a route costs a
   stop-and-turn at every corner during execution. Each shortcut is raw-
   validated, and a shortcut is judged on the clearance-ranked execution time
   the candidates themselves compete on, not on raw travel time: judging it on
   travel time alone lets a shortcut buy seconds by dragging the route back
   against the wall the search climbed away from.
2. **Clearance centering.** A lattice node lands wherever the grid puts it, so
   a route through a 2.4 m doorway runs within centimetres of the jamb:
   execution then has to crawl through it, and the first freshly observed voxel
   of that jamb blocks the route. Each interior vertex slides across the local
   route direction toward the clearance maximum, which in a passage is its
   middle. A move is kept only when it raises that vertex's clearance and both
   incident segments still validate against raw evidence, so the pass can never
   turn a valid route into an invalid one. Endpoints never move: the first is
   the vehicle's own position and the last is the goal.

`persistent_planner_clearance_centering_passes` and
`persistent_planner_maximum_clearance_centering_queries` bound the work, and
`PRODUCTION_MPPI_ROUTE3D` reports `clearance_centering=<moves>/<queries>`.

## Lattice Resolution

The planner lattice is a dense integer grid at `minimum_horizontal_step_m` /
`minimum_vertical_step_m`, and its adaptive levels are *coarser* multiples of
that step. There is no finer level, and there cannot be one without changing
the representation: the Urban world at 0.5 m spacing is 58 million nodes, and
one of the several per-node arrays alone would be 232 MB, before the D* label
maps and the edge cache. A finer level confined to chunks near occupied
evidence therefore needs a sparse, chunk-local node set, which is a redesign of
the lattice indexing together with the edge cache, the occupied-change stamping
and the D* label storage.

Finer routing therefore comes from refining the *edges* rather than the nodes.
A level-zero edge whose straight node-to-node segment fails its sweep is not
blocked outright: the search probes `persistent_planner_edge_refinement_offsets`
points across the edge, perpendicular to it in the horizontal plane, for a
waypoint that clears the body on both legs. Both legs are validated by the
ordinary raw rule, so the hard criterion of what the body may touch is
unchanged; the refinement widens what the graph can express. The edge keeps
its straight-line cost and the waypoint is carried in every extracted path —
the D* path, the feasibility path and the execution-time refiner's — so the
route the executor validates is the one the search priced. A refined edge is
cached like any other and is re-examined when occupied evidence lands on it.
`persistent_planner_maximum_edge_refinement_probes` bounds the sweeps one
update may spend on refinement: near occupied evidence most edges fail their
straight sweep, and unbounded the refinement would take the whole compute
budget. Zero offsets disable it.

The two other consequences of the sparse lattice are addressed as before: a
vehicle in a column that carries no node leaves through a refined free
waypoint (below), and a route whose nodes land against a jamb has its vertices
slid to the middle of the passage before it is published.

## Leaving The Vehicle's Own Position

The search reaches the lattice from wherever the vehicle stands. Normally that
is one segment to the nearest admissible node, with the departure exemption for
contact evidence the body already holds.

A vehicle that has come to rest close to occupied evidence — beside a wall
after a blocked-route stop — can be in a position where the swept body sweeps a
jamb on every such segment. The lattice is sparse relative to the map, so the
column the vehicle sits in may carry no node at all. The planner then reports
`start_unavailable` for as long as the vehicle stays put, and nothing changes
it: this is the pocket that ended two of the four recorded Urban runs.

When no node is reachable in a single segment, the search probes a grid
`persistent_planner_departure_refinement_subdivisions` times finer than one
lattice cell around the vehicle, nearest first and bounded by
`persistent_planner_maximum_departure_refinement_probes`, for a free point it
can reach and from which a node is reachable under the ordinary raw rule. That
point becomes the route's first waypoint. It is a route through free space like
any other: the body contract validates both legs, the leg leaving the vehicle
under the departure exemption and every later leg under ordinary raw evidence.
`PRODUCTION_MPPI_ROUTE3D` reports `departure_waypoint=true` when it was needed.

## Liveness And Safety

The liveness monitor measures progress the vehicle actually made over an
observation window. Progress is the larger of the route-station gain and the
displacement projected on the route tangent held when the window opened: a
route replaced under a moving vehicle restarts its station from a new geometry,
and the ground covered along the route it was following is progress the station
coordinate alone misses. A route generation change restarts the window, because
the two stations are not comparable.

Ground covered without route progress above
`liveness_minimum_offroute_displacement_m` is movement: a lateral or vertical
manoeuvre around an obstacle is flying, and replacing its optimised sequence
with a route connector would cut the manoeuvre short. Loops in place stay below
it, so a collapsed sampler oscillating on the spot is still recovered. A stall
must show in `liveness_stalled_windows_before_reseed` consecutive windows before
a reseed is requested.

Recovery may reseed MPPI, request strategic suffix repair, or publish a typed
hold while persistent search continues. It cannot manufacture direct goal motion
or revive an expired horizon.

Every retained or rebuilt continuation is validated from the measured state
against current raw occupied evidence and the physical swept footprint. Low
clearance by itself does not prohibit motion. If neither a nominal continuation
nor its certified fallback remains executable, `no_executable_horizon` or
`no_executable_route` hold owns execution until a new atomic plan is admitted.

## Removed Protocols

The following concepts are not part of production runtime:

- 2D or 3D risk-aware lattice route producers;
- online incremental topology, semantic frontier, and strategic lattice adapters;
- plan-level route-strategy arbitration;
- partial A* repair and parallel repair races;
- unsafe truncation generations or fingerprints;
- moving/after-hold successor negotiation;
- path-id ACK/coalescing lifecycle;
- launch-departure route-purpose dispatch.
