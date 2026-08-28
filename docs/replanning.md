# Persistent Route Repair And Receding-Horizon Execution

Production navigation separates persistent strategic route ownership from the
short executable horizon. It does not use the retired full/partial A* repair,
risk-lattice replacement, or online-topology arbitration protocols.

## Persistent Strategic Route

`PersistentDStarLitePlanner3D` is the only production strategic route producer.
It keeps search state across compatible raw-world revisions and incrementally
updates affected graph vertices when occupied voxels change. Unknown space is
traversable and contributes neither a hard gate nor a cost.

`ActiveIntent3D` preserves the mission intent while `RouteManager3D` owns the
accepted immutable route identity, full-3D progress, and certified reserve.
Ordinary world updates do not replace a still-valid route. Planning starts from
the current mission coordinate or from a certified future stitch station; vehicle
yaw is never a strategic search constraint.

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
certificate, and all evidence revisions cross the publication boundary as one
atomic execution plan. A mixed generation or stale owner fails closed.

Only a fresh timestamped `MppiTrajectoryHorizon` is executable. Offboard tracks
its position, velocity, and acceleration feed-forward and publishes exact applied-
control feedback. A finite path contains its own terminal deceleration; after the
last zero-velocity sample, offboard holds that same terminal position.

## Liveness And Safety

The liveness monitor compares predicted route progress with measured full-3D
motion. Recovery may reseed MPPI, request strategic suffix repair, or publish a
typed hold while persistent search continues. It cannot manufacture direct goal
motion or revive an expired horizon.

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
