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
