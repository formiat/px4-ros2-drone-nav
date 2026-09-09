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

### The First Route

A vehicle without a route asks for a search once per observed world. That
search holds no replan gate, so the gate cannot retire it: the route that ends
it is the one that becomes resident, however it was found. Until then the
search's own continuation is queued straight after every planner update, so
the planner searches continuously instead of spending one budget per observed
world and idling until the next one arrives, and a later world request finds
the continuation already queued and leaves it in place. Judged by the gate it
never held, the initial search used to end after every update; the planner
ran for a fraction of each second while the vehicle held at the start.

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

One planner update runs, in order: change scheduling, affected-vertex repair,
the feasibility search (only while no route is held), the D* Lite
shortest-path search, and the execution-time refinement. They once shared the
update first-come, and between them the stages that ran first took all of it:
D* Lite was measured expanding nothing in over half the recorded updates, and
105 of 166 published routes came from the unranked feasibility branch.

The tail of the update is reserved for the persistent session
(`persistent_planner_guaranteed_spatial_search_fraction`), and the
execution-time refinement is guaranteed a share of the update's expansions
(`persistent_planner_guaranteed_refinement_expansion_fraction`) rather than
only what the spatial search leaves. Without the second one the refinement runs
only while D* is idle, and the refinement is what turns a first-found route
into a ranked one.

The refinement itself runs only while an incumbent is held. It improves one,
and with none held it has nothing to improve while the vehicle needs a route
rather than a better one: its guaranteed share then goes to the repair and the
feasibility search. One recorded flight spent two seconds without a route
while the refinement took up to nineteen hundred expansions an update and the
feasibility search, which was finding the routes that flight actually flew,
took a hundred and fifty.

A route released as blocked and replaced from the vehicle, rather than
stitched onto its own certified prefix, is a route the vehicle cannot follow
at all: the search drops it as its incumbent and starts from the vehicle.
Kept, it left the search improving a route nobody could fly while the
feasibility branch that finds the replacement stayed idle, and recorded
flights waited one to three planner updates for a successor at every such
block. A stitched replacement keeps the incumbent, because its prefix is
exactly what the vehicle is still flying.

A route the session has resolved is published as it stands whenever the
planner holds no incumbent (`spatial_search` in the candidate source). The
refinement treats that route as its anytime bound and returns a path only when
it beats the bound, so with nothing to improve on the resolved route reached
nobody: a vehicle whose route had just been retired waited for the feasibility
search to find one of its own while D* already had one, and one recorded
flight held for three seconds that way. The refinement improves it from the
same bound as before.

While no route is held the session's bookkeeping is not paid for at all.
Mapping a scan's occupied changes onto its vertices and repairing the labels
they touch is what keeps the ranked branch true, and the ranked branch is not
the one finding a route then. Measured on one recorded flight, an update with
no route held spent 78 to 107 ms scheduling and 10 ms repairing against a
60 ms budget, and D* still expanded nothing for a whole second: the repair
queue stood at three thousand states while every scan added two thousand more,
so the session reported its shortest path complete on labels the world had
long moved. The vehicle waited on the feasibility search, which was holding a
capped third of the update because the stale queues read as work. Suspended,
the whole update goes to the search that finds the route, and the session is
begun again from the current world as soon as there is a route to improve --
the labels it would have caught up to are the labels it starts from.

The paragraphs below describe that bookkeeping as it runs with a route held.

Mapping an update's occupied changes onto the session's vertices is the
session's own bookkeeping, and on a scan it covers thousands of cells. Measured
on one recorded flight it took the whole update on every fourth one, leaving
the feasibility search three expansions instead of a hundred and thirty, and
the vehicle waited five and a half seconds for a route the search was finding
at a tenth of its usual rate. While no route is held the scheduling therefore
runs behind that search, later in the same update: the labels it marks are
repaired one update later, and a vehicle without a route needs the search that
finds one more than it needs the session current a tick sooner. The search
reserves what the scheduling last cost, a decayed maximum, so running it first
never pushes the update past its own budget; without that reserve one recorded
flight's planner p95 rose to 225 ms against a 200 ms bound. With a route held
the order is unchanged.

The repair runs before the feasibility search, not after it. The session's
labels are only as true as its repair queue is short: with repairs pending, D*
reports its shortest path complete on labels the world has already moved, the
refinement then searches from that answer, and the reserve meant for the
session goes to a refinement of a route that is not there. With the repair
after the feasibility search, and its deadline clamped to the same point the
feasibility search ran up to, the repair received nothing at all while no
route existed — the situation it exists for; one recorded Urban run held
4,118 pending repairs for 540 s without processing one. While a route is held
the repair takes half the update and the search the rest; while none is, the
repair is bounded by half the session's reserve, so the feasibility search
still gets most of the update for a first route.

Every share is a share of what is left when its stage starts, never a point
on the clock. Change scheduling is not budgeted — it has to see every change —
and on a fresh scan it runs long; a feasibility deadline fixed at the reserve
boundary then fell before the search began, and a vehicle without a route
waited on D* alone for the first route. The feasibility search keeps the
session's reserve out of what remains when it starts
(`persistent_planner_maximum_feasibility_compute_time_ms`, capped at a third
of the remainder) and takes the rest.

An update that begins with no route held is given a shorter budget than one
that begins with a route (`persistent_planner_maximum_no_route_compute_time_ms`,
capped by `persistent_planner_maximum_compute_time_ms`). A route found inside an
update reaches the vehicle only when that update ends, so the update's length is
the wait: with a hundred and fifty millisecond update the vehicle waits up to
that long after the route already exists. Almost none of an update is fixed
cost — measured with no route held, an update spent 1.5 ms installing the world
against 105 ms of search — so harvesting the same searches two or three times as
often costs the searches almost nothing and cuts the wait by the same factor. A
vehicle with a route can wait for a better one; a vehicle without one cannot.

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
does drop it — the incumbent, not the feasibility labels. It drops it only
when the refusal lies inside what the vehicle is committed to: the departure
leg it flies at hover, and the certified overlap a successor has to splice
onto. A refusal beyond that is a block ahead like any other, the route is
still enterable, and the search repairs the blocked part against the world it
re-validates every update; one recorded flight dropped its incumbent to a
refusal twelve metres ahead and stood for nine tenths of a second while the
feasibility search explored its way back out to twenty-five. The labels are
validated lazily against the resident world, which the consumer's evidence
reaches through the raw overlay, so the chains through the block are dropped
and re-parented where the search next touches them and the next candidate is
extracted from what was explored. Restarting the search from nothing cost one
to two seconds per rejection and, whenever the world had not changed, found
the same first route again.

Admission applies the same idea once more, against the route the vehicle
actually has. A successor that reaches the same mission goal as the resident
must be materially faster than it; a release as blocked or diverged waives
that, because a route the vehicle cannot follow is no base to improve on. The
base has to be one the vehicle is following, though: a route the plan keeps
while the vehicle brakes on a continuation stop, or holds with no executable
horizon, is not. Measured against such a route, one recorded flight refused a
successor for being two hundredths of a second slower while it had nothing at
all to fly. A pending certified route stays a base worth improving on, since
the vehicle is about to fly it.

A route blocked far enough ahead is replaced onto its own certified prefix.
The replan carries the incumbent as the successor's continuity base
(`RouteLifecycleReplanSnapshot3D::active_route`, `route_projection`,
`blocked_station_m`) with a stitch limit one certified overlap short of the
block (`PlannerSearchContinuityBase3D::stitch_limit_station_m`): the successor
is searched from the stitch one overlap ahead of the vehicle, with the
incumbent's own velocity there, the certified prefix is frozen and the
executor splices onto it, exactly as an extension does. A block inside that
reach leaves nothing certified worth keeping and the replacement is searched
from the vehicle as before (`stitch_fallback_to_vehicle`). Measured over
three urban flights, more than half of the controller's candidate rejections
came within two seconds of a route replacement and within four metres of its
start: a replacement searched from the vehicle turned it around, or sent a
vehicle descending into a hole climbing instead, and the horizon that still
carried the old intent swept the evidence beside it. The replan log reports
the limit as `stitch_limit_m`, -1 when the search starts from the vehicle.

Holding the first-found feasibility route back after such a loss — to give
the persistent search's repair a chance to deliver the shortest path past the
block before the vehicle commits to a detour — was tried and does not work.
Measured over three urban flights, the repair delivered inside a 1.5 s window
in fewer than one loss in five; the other losses ran the whole window with no
route at all, 45–60 s per flight of no-route holds, and route availability
fell from 99 % to 92–98 %. The feasibility route is therefore published as
found, and the repaired route replaces it through the ordinary candidate
comparison when it arrives.

## Published Route Geometry

Every published route — from the feasibility search and from the execution-time
refinement alike — goes through the same two passes before it is offered, and
the materialised route's corners are then filleted:

1. **Shortcut simplification.** A lattice zig-zag left in a route costs a
   stop-and-turn at every corner during execution. Each shortcut is raw-
   validated, and a shortcut is judged on the clearance-ranked execution time
   the candidates themselves compete on, not on raw travel time: judging it on
   travel time alone lets a shortcut buy seconds by dragging the route back
   against the wall the search climbed away from.
   The ranking factor is sampled per segment and kept: a shortcut replaces one
   run of segments with one segment and leaves the rest, so only the new
   segment is sampled. Re-sampling the whole route's clearance for every
   candidate shortcut — a raw search of the occupancy for each sample — is what
   let one update take sixty times its budget. The pass, like the centering
   after it, stops at the update's deadline and publishes the route as
   simplified so far; the next update simplifies further.
2. **Clearance centering.** A lattice node lands wherever the grid puts it, so
   a route through a 2.4 m doorway runs within centimetres of the jamb:
   execution then has to crawl through it, and the first freshly observed voxel
   of that jamb blocks the route. Each interior vertex slides across the local
   route direction toward the clearance maximum, which in a passage is its
   middle. A move is kept only when it raises that vertex's clearance and both
   incident segments still validate against raw evidence, so the pass can never
   turn a valid route into an invalid one. Endpoints never move: the first is
   the vehicle's own position and the last is the goal. The clearance is
   measured to observed evidence only, so beside a wall that has not been
   scanned yet it grows without bound and its gradient points into the
   unknown; a vertex slid that way used to sit against the jamb or the lintel
   the first scan revealed, and the route was blocked there. A probe in
   unobserved space therefore contributes no gradient and a candidate in
   unobserved space is never taken: the vertex stays where the observed lane
   puts it. Unknown space remains traversable and free of charge — this only
   decides where an optional refinement may move a vertex.

`persistent_planner_clearance_centering_passes` and
`persistent_planner_maximum_clearance_centering_queries` bound the work, and
`PRODUCTION_MPPI_ROUTE3D` reports `clearance_centering=<moves>/<queries>`.

3. **Corner fillets.** Each corner of the materialised route is replaced by
   the widest quadratic fillet the raw swept body admits: the control distance
   starts at `static_route_corner_smoothing_distance_m`, bounded by the
   incident segments, and halves toward
   `static_route_corner_smoothing_minimum_distance_m` until a curve validates.
   The turn speed the curvature limiter admits grows with the square root of
   the radius, so a corner in open space is worth the wide arc, and a corner in
   a passage still gets the arc that fits instead of the stop-and-turn one
   fixed distance left it with whenever that distance did not clear.

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
cached like any other and is re-examined when occupied evidence lands on
either of its legs: the change scheduling tests the geometry the vehicle
flies, not the straight segment, because the waypoint can lie a metre off it
and a voxel beside the waypoint would otherwise leave the cache admitting a
leg the sweep rejects. Refined edges are indexed by the chunks their legs
touch, as adaptive edges are, and tested from that index; widening the node
walk that tests straight edges by the waypoint offset instead tripled the
scheduling cost of every change.
`persistent_planner_maximum_edge_refinement_probes` bounds the sweeps one
update may spend on refinement: near occupied evidence most edges fail their
straight sweep, and unbounded the refinement would take the whole compute
budget. Zero offsets disable it.

The raw sweep is the authority on every candidate, and a segment it rejects
has to reach the cache entry that admitted it. A rejected segment names the
lattice edge it stands for — a straight edge, or a refined edge through its
waypoint — and that edge is forgotten, withheld from the feasibility search on
this world, and repaired in the D* session; only a rejected departure or a
degenerate candidate restarts the feasibility search from nothing. The first
version of the refinement treated the waypoint as it treated the departure,
and a stale refined edge then restarted the search on every update: it found
the same edge in the same cache, and the same leg failed, 2,821 times over the
540 s a vehicle stood without a route in the shaft at x ≈ 54 of the Urban
world. The execution-time refinement discards its search state on a blocked
path in the same way, forgetting the edge first.

The two other consequences of the sparse lattice are addressed as before: a
vehicle in a column that carries no node leaves through a refined free
waypoint (below), and a route whose nodes land against a jamb has its vertices
slid to the middle of the passage before it is published.

## Leaving The Vehicle's Own Position

The search reaches the lattice from wherever the vehicle stands. Normally that
is one segment to the nearest admissible node, with the departure exemption for
contact evidence the body already holds. The anchor the searches are seeded
from is kept for as long as it stays admissible, whichever node is nearest at
the moment: a vehicle resting near occupied evidence sees its nearest reachable
node flip with every raw scan, and each flip beyond a lattice diagonal restarts
the feasibility search from nothing. The walk over the anchors on exhaustion
takes precedence over the preference.

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

The leg leaving the vehicle carries the departure exemption for contact the
body already holds, and the proprioceptive seed that grants it is the
vehicle's own pose. The request's copy of the seed was made when its raw
world was captured — a continuation earlier — while the request start is
the vehicle's pose now: the planner re-anchors the seed at the start before
it selects a departure, and reports how far the copy had lagged
(`seed_distance_m`). Over one urban flight the copy lay 0.4–2.5 m from the
start at every refusal, and a vehicle that had drifted into contact since
was refused every departure the node's own validators, seeded at the current
pose, still granted.

A refusal at the input stage — `start_unavailable`, `goal_unavailable` — runs
no search and is decided by the raw world alone. The lifecycle latches a
failed search so that an exhausted search is not repeated on the same input,
and retries it after `route_failed_search_retry_interval_s` unless the vehicle
or the objective moved; an input refusal waited out the same interval. A
vehicle that has just stopped beside evidence it met is refused a departure
while it settles, and the next scans and its own settling are what change what
the body clears: over one urban flight five such refusals held the vehicle for
1.2–1.5 s each, most of it the interval. An input refusal is therefore retried
on every newer raw world (`retry_trigger=raw_world_changed`).

A candidate the raw world refused is retried the same way. When a search runs
and the activation rejects what it produced -- the candidate's own validation,
its certification, its execution geometry -- the refusal is the world's answer
to that candidate, not the search's answer to the problem, and a newer world
is a different answer. Held to the retry interval instead, one recorded flight
stood for nine tenths of a second with no route at all, four candidates
refused in half a second and then nothing asked again while the evidence under
it changed ten times over. A search that ran and failed on its own terms --
exhausted, or converged on no route -- still waits out the interval.

The latch only decides requests that reach it, so something has to keep asking
while the vehicle has no route. The pending-route recovery is what asks, and
it stays silent while an execution owner holds the wire. A certified
stationary hold was already exempt from that rule — it pins a position but
executes no route — and a stop is the same case: it brakes to one. Left
unexempt, the stop suppressed the successor request for its whole braking
tail, the physical invalidation that produced it was latched for a single
tick, and the deferred replan it raised was consumed by the first search; a
latched failure then sat with nothing to re-evaluate it. One recorded flight
braked and rested for two and a half seconds with the planner idle, a quarter
of that run's no-route time. Both owners are now routeless owners, and the
latch decides how often a search actually runs.

The departure envelope decides which anchors a vehicle may leave through, and
only where it admits none at all does the hull decide instead
(`departure_hull=true` in `PRODUCTION_MPPI_ROUTE3D`). A vehicle leaves a tight
spot at hover and upright, so the envelope that contains the hull at every
tilt is not what decides whether it may leave; a vehicle resting beside
evidence its own stop had just met was otherwise refused every departure with
`start_unavailable` for a second or more at a time. Judging every departure by
the hull was tried and withdrawn: ordinary routes then left the vehicle closer
to evidence than the envelope admits, and the flights that followed were worse
on every count, one of them lost to a collision.

Selecting the connection fixes the body every departure leg answers to for the
rest of that update. Judged separately, the fallback found a departure that the
candidate validation then refused on the same leg, and the search spent whole
updates on candidates nothing could publish while the vehicle waited without a
route.

## Leaving A Closed Component

A vehicle can stand where every lattice node it reaches belongs to a component
the goal is not in. The lattice is sparse relative to the map: a 2.4 m
corridor carries no valid node unless its centre happens to fall on the grid,
and once its walls are fully observed the nodes inside it vanish from the
graph. A vehicle that entered the corridor on a route the graph still had —
while the walls were partly unobserved — then stands in a region the graph has
no way out of. Both lattice searches exhaust it: the feasibility search
empties its frontier, restarts from the same anchor and empties it again, and
D* holds no route. Two recorded Urban runs ended this way in the shaft at
x ≈ 54–61, the vehicle holding for 540 s and 640 s with the planner reporting
`feasibility_exhausted=false` throughout, because the restart cleared the
flag it had just set.

The exhaustion is now reported for the update it happened in, whatever the
search did afterwards, and the nodes the frontier had labelled are remembered
as the closed component for as long as the vehicle stays put. The anchor walk
advances on it, and the escape search runs on the next updates: a flood fill
over a grid `persistent_planner_departure_refinement_subdivisions` times finer
than the lattice, within `persistent_planner_escape_search_radius_cells` of the
vehicle, through steps validated by the ordinary raw rule (the first under the
departure exemption), until it reaches a point from which a lattice node
outside the closed component is reachable. The chain of fine steps becomes the
departure: the route leaves the vehicle through every point of it and joins
the lattice at that node, so the body contract is unchanged — every leg is a
raw-validated segment — and the searches are re-seeded from the node outside.
The vehicle flies the chain and the planner trims it as each point is passed.
`persistent_planner_escape_search_maximum_probes_per_update` bounds the sweeps
one update spends on the fill; it resumes on the next update, and a fill that
found nothing is repeated only when the world changed. `PERSISTENT_PLANNER3D`
reports `escape_attempted`, `escape_found`, `escape_active`, `escape_exhausted`,
`escape_probes` and `escape_cells`; `departure_waypoints` counts the chain.

A route search serves a consumer session: the first update of a session
delivers the search's incumbent, later updates only improvements. A vehicle
that lost its route — its execution revoked after a certified stop, its
pending route retired with the execution base — holds nothing of the search
any more, and the fresh request it raises for a route is displaced by, or
held behind, the continuation of the search that is already running for the
same base. That continuation therefore opens a new consumer session whenever
the vehicle holds neither a resident nor a pending route, and the planner
delivers its incumbent again on the next update. Before this, a continuation
that kept its session answered with improvements only, and one recorded run
held for five seconds after every stop until the refinement happened to find
one.

The goal end of the search has the same weakness in a smaller form. The
search ends at a lattice node within the connector radius of the goal whose
straight connector to the goal validates. A goal set a hand's breadth above a
floor the lidar only sees late — or beside a wall a fresh scan has just put
into the memory — loses every such node at once, and a planner with no goal
anchor ran no search at all: one recorded run held for the rest of its
mission with the planner reporting `goal_unavailable` on every update, the
vehicle at rest and nothing left to observe the evidence away. The goal
connection now probes the same fine grid around the goal, within
`persistent_planner_goal_tolerance_m`, for the nearest free point a node
reaches, and the searches end there; the route still counts as reaching the
mission goal, which the mission captures within that tolerance.
`PERSISTENT_PLANNER3D` reports `goal_refined`.

The choice was between this and validating the way out with the physical
body (0.55 m) instead of the envelope (0.82 m). The vehicle entered the region
on the envelope, so a way out on the envelope exists with high probability and
is simply not expressible on the lattice; the fill finds it without touching
the safety contract, whereas a body-radius exit would have to be accepted by
the executor as well, which validates on the envelope.

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
