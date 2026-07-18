# Obstacle Mapping

Obstacle mapping combines static obstacles, current lidar hits, and accumulated
memory into planner inputs. The main rule is that raw sources stay raw; the
planner owns inflation and planning clearance.

## Static Map

The static map is loaded from:

```text
drone_city_nav/worlds/generated_city.map2d
```

Configured by:

```yaml
use_static_map: true
static_map_path: worlds/generated_city.map2d
static_map_min_blocking_height_m: 0.0
```

The static map is cached and reused when possible.

## Current Lidar Overlay

The planner can project the current `/scan` into a raw dynamic overlay. Current
lidar overlay is useful for obstacles that are visible now but not yet stable
in memory.

Important parameters:

- `max_current_lidar_staleness_s`
- `max_lidar_range_m`
- `range_hit_epsilon_m`
- lidar pose latency and attitude compensation settings.

Before a current lidar hit is written into the overlay, the always-on 3D known
static classifier compares its map-frame ray and measured range with the known
passage-building solids. A confident matching physical-solid hit is suppressed.
A closer hit, a hit through a free opening, and a boundary or otherwise
ambiguous hit remains in the overlay. This decision is independent of the
current trajectory and distance to a passage.

The same decision is applied before a hit changes obstacle-memory scores. It is
range-based rather than a blanket spatial exclusion around an opening, so an
unknown object in front of a known wall or inside free opening space remains
obstacle evidence. `known_passages.md` documents the 3D ray construction,
asymmetric tolerances, and memory-reset behavior when known geometry changes.

## Obstacle Memory

`obstacle_memory_node` accumulates scan evidence into
`/drone_city_nav/obstacle_memory_grid`.

Memory uses hit/miss scoring:

- `hit_weight`
- `miss_weight`
- `min_score`
- `max_score`
- `occupied_score`
- `free_score`

Mapping starts only above `min_mapping_altitude_m`.

For passage debugging, every first transition of a nearby memory cell to
occupied is logged as `PASSAGE_MEMORY_HIT` with endpoint XYZ, beam/range,
attitude, score transition, and known-static classification. The event is
diagnostic only and does not alter hit scoring or filtering.

All first occupied transitions are additionally written to a bounded JSONL dump
configured by `lidar_memory_hit_dump_enabled`, `lidar_memory_hit_dump_path`,
and `lidar_memory_hit_dump_max_records`. The default runner supplies a distinct
`log/lidar_memory_hits/<run-id>.jsonl` file. A row preserves the complete 3D
ray, scan and callback timestamps, pose/attitude inputs, motion compensation,
both ground and known-static range candidates, and the decision that retained
the hit. It is a post-run diagnostic artifact, not a planner input.

Every active occupied memory cell also owns sparse 3D diagnostic provenance:
the hit that first made the cell occupied, the latest accepted hit, the observed
minimum/maximum endpoint Z, and the accepted-hit count. This metadata never
participates in scoring, inflation, A*, or trajectory control. It is published
on `/drone_city_nav/obstacle_memory_provenance` for standalone diagnostics. The
planner receives the same data through the authoritative atomic
`/drone_city_nav/obstacle_memory_snapshot` message, which carries the raw grid
and provenance together.

For RViz, `obstacle_memory_node` also derives
`/drone_city_nav/raw_memory_obstacle_points_3d` directly from the same active
provenance at the standalone debug cadence. The cloud contains exactly the
finite XYZ from each cell's `occupancy_trigger`, not the cell center or
`last_hit`, and applies only the established visualization Z compensation. It
contains no inflation and no removed-cell history. The existing
`/drone_city_nav/raw_memory_obstacle_points` remains a separate ground-plane
view of active 2D cell centers. Neither visualization cloud is a planner input,
and the 3D cloud is not a 3D obstacle-memory implementation.

The planner replaces its current memory state only when stamp, frame, complete
map metadata, raw row-major grid hash, occupied count, and every provenance
record agree exactly inside that one message. Callback backlog may drop an old
snapshot, but it cannot deliver its grid without its provenance or vice versa.
An invalid atomic pair is ignored as a whole, leaving the previous valid pair in
use. Planning never waits for a later diagnostics callback.

The throttled producer summary reports `serialized_bytes`, measured from the
actual ROS CDR serialization buffer. This includes variable-length cell and
string payloads and is intended for monitoring DDS bandwidth growth.

Beam acquisition time is derived from the scan stamp and `time_increment`.
Receive time is stored separately and is never substituted for a missing sensor
stamp. The recorded attitude is the attitude actually used by the projection;
the current implementation does not interpolate vehicle pose independently for
every beam timestamp.

## Motion Compensation

Lidar projection can account for:

- PX4 heading;
- motion-compensated lidar pose;
- lidar pose latency;
- attitude compensation;
- lidar mount roll/pitch/yaw offsets;
- lidar z offset.

The same concepts appear in obstacle memory, planner current lidar overlay, and
lidar debug configuration.

## Per-Beam Expected-Surface Rejection

Obstacle memory and the planner current-lidar overlay share one immutable
`LidarBeamObservation` and one ingestion decision before either path changes a
grid. The decision compares the measured range with the nearest expected 3D
surface along the map-frame ray. Providers currently include known passage
solids and the configured flat ground plane.

Only surfaces reachable within the beam's effective sensor range participate in
the decision. If no provider has such a candidate, a valid measured hit remains
ordinary `free_and_hit` evidence even when an unbounded diagnostic ray query
could find known geometry farther away.

Ground rejection is range based, not endpoint-distance based and not a global
vehicle-tilt cutoff. A fast level-flight attitude therefore does not disable
lidar mapping. For a downward ray, the expected flat-ground range is computed
from the ray origin, ray direction, and `ground_lidar_altitude_m`:

```text
expected_ground_range =
    (ground_altitude - ray_origin.z) / ray_direction.z
```

The resulting policy is asymmetric:

- a hit clearly before every nearest expected surface remains unknown-obstacle
  evidence;
- a return consistent with the ground is suppressed;
- a ground-facing return beyond the allowed farther tolerance is ambiguous and
  is also suppressed fail-safe;
- a ground-facing no-return beam whose finite sensor range reaches the ground
  is suppressed without free-space clearing, but remains classified as
  `ambiguous_ground` because no measured return confirms the ground surface;
- expected or ambiguous ground beams perform neither endpoint-hit integration
  nor 2D free-space clearing.

The last rule is essential. A downward 3D ray passes through air before reaching
the ground, but its XY projection does not prove that the same cells are free at
the executable trajectory altitude.

Provider failures are isolated. Disabling ground rejection is reported as
`disabled`; invalid ground parameters or missing required 3D attitude geometry
are reported as `unavailable`. In either case known-static classification still
runs. When multiple expected surfaces have effectively equal nearest ranges, a
hit is retained only if it is clearly before every tied candidate; otherwise no
grid update is applied.

The projected-altitude filter remains a final non-mutating veto. Ground and
known-static classification happens first for diagnostics, including beams
whose endpoint is below `min_projected_lidar_altitude_m`, but an
`altitude_rejected` beam still cannot mutate either grid.

## Raw, Prohibited, And Planning Clearance

Raw obstacles are direct evidence. The planner merges raw sources and produces:

- prohibited grid: hard safety grid;
- planning clearance: extra planner margin.

The current default is:

```yaml
inflation_radius_m: 1.0
planning_clearance_m: 3.0
```

## Atomic Memory Transport

Runtime planning receives a single `ObstacleMemorySnapshot` containing the raw
2D grid and exact sparse provenance. A monotonically increasing producer
sequence and the grid stamp make delivery and replacement observable. The
planner rejects zero, duplicate, or out-of-order sequences and retains its last
valid state when nested grid/provenance validation fails.

The authoritative atomic snapshot is published after every accepted memory
scan update. Standalone grid/provenance topics are diagnostics-only and default
to a 1 Hz cadence to avoid duplicate serialization and transient-local history
cost. The provenance debug publisher uses KeepLast(1); exact runtime history is
carried by the currently applied atomic snapshot rather than a retained queue of
large standalone messages.

The planner's snapshot callback runs separately from planning work. It fully
validates and parses each delivered atomic pair, retains only the newest parsed
pair, and records both DDS sequence gaps and replacements in that pending slot.
At the start of each planning check, the pair is moved into the active planner
state. The active pair remains immutable for the rest of that planning cycle.

The prohibited grid is published for validation and visualization. Planning
clearance affects route/trajectory construction and should not be interpreted
as a hard runtime failure by itself.

## RViz Outputs

Useful visualization topics:

- `/drone_city_nav/static_map_grid`
- `/drone_city_nav/static_map_points`
- `/drone_city_nav/static_building_markers`
- `/drone_city_nav/obstacle_memory_grid`
- `/drone_city_nav/obstacle_memory_provenance`
- `/drone_city_nav/obstacle_memory_snapshot`
- `/drone_city_nav/prohibited_grid`
- `/drone_city_nav/lidar_debug_points`
- `/drone_city_nav/raw_lidar_hit_points_3d`
- `/drone_city_nav/remembered_lidar_points`
- `/drone_city_nav/prohibited_obstacle_points`
- `/drone_city_nav/raw_memory_obstacle_points`
- `/drone_city_nav/raw_memory_obstacle_points_3d`

## Common Problems

- If lidar hits are shifted, check PX4 origin, heading source, mount yaw, and
  attitude compensation.
- If obstacles appear too wide, check inflation and planning clearance.
- If memory contains stale obstacles, inspect hit/miss scoring and free-space
  clearing.
- If the planner replans unexpectedly, compare raw sources, prohibited grid,
  and the current executable trajectory.

## Obstacle Evidence Contract

Obstacle mapping starts from evidence, not from final safety decisions. A lidar
hit, a remembered obstacle cell, and a static map cell all mean that obstacle
evidence exists at a location. They do not mean that the whole safety margin is
already occupied. The grid builder owns the conversion from raw evidence to
hard prohibited space and planning-clearance preference space.

This contract prevents hidden double margins. If obstacle memory stored already
inflated cells and the planner inflated them again, the effective blocked area
would grow with every source and become difficult to reason about. Keeping raw
sources raw makes every margin visible in configuration and diagnostics.

## Static, Dynamic, And Memory Roles

Static map data represents known world geometry. It should be stable across
the run and is the best candidate for caching or preprocessing.

Current lidar overlay represents the most recent sensor evidence. It is useful
for quick reaction, but it can be sparse or noisy because a lidar scan sees only
what the current pose exposes.

Obstacle memory bridges the gap between static map and current scan. It keeps
recently observed obstacles available after they leave the instantaneous scan.
Memory is especially important when the drone turns away from an obstacle but
the planner still needs to avoid it.

The same known-static classifier is applied before a hit changes obstacle-memory
scores. It suppresses only new confident physical-solid hits; it does not remove
older cells selectively or create a temporary memory copy. When classifier
geometry is installed or changed, obstacle memory and its associated provenance
are reset together so no cell survives under a different geometry contract.

These sources are complementary:

- static map gives persistent structure;
- current lidar gives fresh evidence;
- memory gives temporal continuity;
- inflation turns merged evidence into safety space.

Known-passage geometry is used consistently by both lidar ingestion paths. The
classifier never filters static-map cells and never changes A* route selection.
It only prevents known physical masses from becoming new dynamic evidence; a
real object before a wall or inside an opening still follows normal prohibited
grid and replan behavior.

The ground provider follows the same shared decision path but does not add a 3D
planning layer. Obstacle memory remains a 2D scored grid. Accepted occupied cells
carry sparse diagnostic 3D provenance from the observation that created and
last confirmed the cell; rejected ground observations are kept only in bounded
counters/log samples and never become obstacle-memory provenance.

## Inflation And Distance Fields

Inflation answers whether a cell is too close to obstacle evidence. Clearance
answers how far the nearest obstacle evidence is. These two ideas can come
from the same distance field:

- cells with distance less than hard inflation become prohibited;
- cells with distance less than hard inflation plus planning clearance become
  planning-disfavored;
- the raw distance can be reused for diagnostics, corridor clearance, and
  scoring.

The current architecture already treats clearance as a reusable artifact where
possible. A future Euclidean Distance Transform implementation would make this
relationship more direct: build one distance-to-obstacle field, threshold it
for hard and planning grids, and reuse it for clearance diagnostics.

The risk of changing inflation is boundary behavior. Two algorithms can differ
by one cell near the radius threshold. Any replacement must be tested against
expected hard safety behavior and RViz output.

## Motion Compensation Diagnostics

Lidar hits must be projected into a stable frame. The drone is moving and
tilting while scans arrive, so treating every scan as if the vehicle were level
and stationary can shift obstacle evidence. Attitude compensation and PX4 pose
freshness matter because a small angular error can become a large map error at
range.

Symptoms of bad compensation:

- obstacle memory appears rotated around the drone;
- lidar points are offset from static buildings;
- replans happen near turns where vehicle attitude changes quickly;
- obstacles appear to move relative to the world when the drone rolls or
  pitches.

When debugging, compare current lidar points, obstacle memory, prohibited grid,
and static map in RViz. The layers should agree in map coordinates even though
they are produced by different stages.

## Mapping Diagnostics Checklist

For an unexpected replan, inspect:

1. Was the prohibited intersection caused by static map, memory, or current
   lidar?
2. Did the raw evidence actually overlap the trajectory, or only the inflated
   margin?
3. Was the intersecting span ahead of the drone or already behind it?
4. Did planning clearance get mistaken for hard prohibited space?
5. Did pose or attitude compensation shift the lidar overlay?
6. Did obstacle memory keep an old obstacle longer than expected?
7. Did the known-static classifier suppress only confident physical-solid hits,
   while retaining closer, opening, or ambiguous hits?
8. Did the planner retain the previous trajectory while rebuilding?

Answering these questions usually separates a real obstacle from a mapping
artifact.
