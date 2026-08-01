# Obstacle Mapping

Obstacle mapping combines static obstacles, current lidar hits, and accumulated
memory into planner inputs. The main rule is that raw sources stay raw. The
planner derives distance-based risk tiers without inflating hard occupancy.

## Static World

The static map is loaded from:

```text
drone_city_nav/worlds/generated_city.occupancy3d
```

Configured by:

```yaml
use_static_map: true
static_occupancy_3d_path: worlds/generated_city.occupancy3d
```

The sparse static world is generated from the same canonical specification as
Gazebo SDF and is used only in static mode.

## Current Lidar Overlay

The planner can project the current `/scan` into a raw dynamic overlay. Current
lidar overlay is useful for obstacles that are visible now but not yet stable
in memory.

Important parameters:

- `max_current_lidar_staleness_s`
- `max_lidar_range_m`
- `range_hit_epsilon_m`
- lidar pose latency and attitude compensation settings.

Lidar evidence is never filtered against hand-authored passage geometry. Static
planning reads Occupancy3D; no-static uses the physical 2D lidar returns and
treats connector occluders as ordinary obstacles.

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

All first occupied transitions are additionally written to a bounded JSONL dump
configured by `lidar_memory_hit_dump_enabled`, `lidar_memory_hit_dump_path`,
and `lidar_memory_hit_dump_max_records`. The default runner supplies a distinct
`log/lidar_memory_hits/<run-id>.jsonl` file. A row preserves the complete 3D
ray, scan and callback timestamps, pose/attitude inputs, motion compensation,
the ground range candidate and the decision that retained
the hit. It is a post-run diagnostic artifact, not a planner input.

Every active occupied memory cell also owns sparse 3D diagnostic provenance:
the hit that first made the cell occupied, the latest accepted hit, the observed
minimum/maximum endpoint Z, the accepted-hit count, the score transition that
first crossed the occupied threshold, that threshold, and the number of
independent scans supporting the trigger decision. This metadata never
participates in risk scoring, lattice search, MPPI, or trajectory control. It is published
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
stamp. Position XYZ and quaternion attitude are sampled independently for every
beam acquisition timestamp from bounded histories. `TimesyncStatus` recovers
PX4-local acquisition time from the DDS-adjusted `timestamp_sample`, and a
bounded affine mapper relates PX4-local time to ROS simulation time. The mapper
uses the lower callback-latency envelope so average executor delay is not baked
into the clock offset. XYZ is interpolated linearly and the complete body-to-NED
quaternion is interpolated with SLERP at the same acquisition timestamp.

Projection reports one explicit pose source:

- `source_timestamp_aligned` for PX4 `timestamp_sample` alignment;
- `receive_timestamp_aligned` while the clock mapper is not ready;
- `motion_extrapolated_fallback` for the legacy velocity/latency fallback;
- `callback_pose_fallback` when no temporal compensation is available.

The fixed `lidar_pose_latency_s` shift is used only by the fallback path. It is
not added to a source-timestamp-aligned pose. If either acquisition history
cannot cover the requested timestamp, the node logs the exact fallback source,
bracketing samples, interpolation/extrapolation age, and mapper residual.

## Motion Compensation

Lidar projection can account for:

- PX4 heading;
- motion-compensated lidar pose;
- lidar pose latency;
- attitude compensation;
- a full rigid body-to-lidar extrinsic.

The transform is:

```text
T_map_lidar(t) = T_map_body(t) * T_body_lidar
```

The configured translation is expressed in body FRD. The configured quaternion
rotates lidar FLU vectors into body FRD. For the shipped X500 model the sensor
center is `(0.12, 0.0, 0.315)` in SDF FLU, represented as
`[0.12, 0.0, -0.315]` in body FRD; the aligned FLU-to-FRD quaternion is
`[0.0, 1.0, 0.0, 0.0]` in WXYZ order. Both ray direction and the complete XYZ
lever arm rotate with the interpolated body quaternion before NED is converted
to the map Z-up convention.

`lidar_z_offset_m` and mount RPY remain only as a compatibility fallback when
`use_full_lidar_extrinsic=false`. The normal configuration uses
`lidar_extrinsic_translation_body_frd_m` and
`lidar_extrinsic_quaternion_lidar_flu_to_body_frd` in obstacle memory, planner
current-lidar overlay, and lidar debug alike.

The same concepts appear in obstacle memory, planner current lidar overlay, and
lidar debug configuration.

## Per-Beam Expected-Surface Rejection

Obstacle memory and the planner current-lidar overlay share one immutable
`LidarBeamObservation` and one ingestion decision before either path changes a
grid. The decision compares the measured range with the nearest expected 3D
surface along the map-frame ray. The configured flat ground plane is the only
expected-surface provider.

Expected ray intersections are bounded by the beam's effective sensor range.
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
  evidence when its endpoint is spatially detached from known geometry;
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

Before either 2D grid is updated, a shared confidence stage separates certain
obstacles from uncertain candidates. Confident obstacles before an expected
surface and obstacles inside a free opening are integrated immediately with the
normal memory hit weight. Known-static boundaries, low contradictory ground
returns, and unknown returns with uncertain timestamp alignment or range-limit
geometry remain pending. A pending candidate performs no hit update, no
free-space clearing, and no current-lidar overlay update.

Candidates are keyed by hypothesis kind, associated surface, and 3D endpoint
voxel. Multiple beams from one scan provide only one vote. Confirmation requires
independent scans plus sufficient viewpoint translation or ray-direction change.
Repeated surface-attached evidence is suppressed; a stable detached cluster is
integrated as an obstacle. Unconfirmed candidates expire without leaving state
in either grid. This keeps `hit_weight=4` and fast occupancy transitions for
accepted obstacles while preventing one geometrically uncertain scan from
creating a replan blocker.

Provider failures are isolated. Disabling ground rejection is reported as
`disabled`; invalid ground parameters or missing required 3D attitude geometry
are reported as `unavailable`. In either case known-static classification still
runs when independently enabled. When multiple expected surfaces have
effectively equal nearest ranges, a hit is retained only if it is clearly
before every tied candidate; otherwise no grid update is applied.

The projected-altitude filter remains a final non-mutating veto. Ground and
known-static classification happens first for diagnostics, including beams
whose endpoint is below `min_projected_lidar_altitude_m`, but an
`altitude_rejected` beam still cannot mutate either grid.

## Raw Occupancy And Soft Risk

Raw obstacles are direct evidence. The planner merges raw sources once and
builds an occupied-distance field. It does not materialize prohibited or
planning-clearance occupancy grids.

The current default critical boundary is 1 m and the preferred boundary is 6 m.
Raw occupied cells and evaluation bounds are hard rejects. MPPI first selects
the best eligible risk class, then applies its soft dynamics and progress cost
within that class.

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

The atomic raw obstacle snapshot is published for runtime validation. Its
debug-only raw grid mirror is published for visualization.

## RViz Outputs

Useful visualization topics:

- `/drone_city_nav/static_map_grid`
- `/drone_city_nav/static_map_points`
- `/drone_city_nav/static_building_markers`
- `/drone_city_nav/obstacle_memory_grid`
- `/drone_city_nav/obstacle_memory_provenance`
- `/drone_city_nav/obstacle_memory_snapshot`
- `/drone_city_nav/raw_obstacle_snapshot`
- `/drone_city_nav/raw_obstacle_grid`
- `/drone_city_nav/lidar_debug_points`
- `/drone_city_nav/raw_lidar_hit_points_3d`
- `/drone_city_nav/remembered_lidar_points`
- `/drone_city_nav/prohibited_obstacle_points`
- `/drone_city_nav/raw_memory_obstacle_points`
- `/drone_city_nav/raw_memory_obstacle_points_3d`

## Common Problems

- If lidar hits are shifted, check PX4 origin, heading source, mount yaw, and
  attitude compensation.
- If routes run too close to obstacles, check critical and preferred risk
  thresholds.
- If memory contains stale obstacles, inspect hit/miss scoring and free-space
  clearing.
- If the planner replans unexpectedly, compare raw sources, the atomic raw
  snapshot, and the current executable trajectory.

## Obstacle Evidence Contract

Obstacle mapping starts from evidence, not from final safety decisions. A lidar
hit, a remembered obstacle cell, and a static map cell all mean that obstacle
evidence exists at a location. Only that raw cell is hard occupied. The ESDF
classifies surrounding free cells into critical, planning, and preferred risk
tiers.

This contract prevents hidden hard margins. Keeping raw sources raw makes risk
classification visible in configuration and diagnostics without changing
reachability.

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

These sources are complementary:

- static map gives persistent structure;
- current lidar gives fresh evidence;
- memory gives temporal continuity;
- the occupied distance field turns merged evidence into risk tiers.

The ground provider follows the same shared decision path but does not add a 3D
planning layer. Obstacle memory remains a 2D scored grid. Accepted occupied cells
carry sparse diagnostic 3D provenance from the observation that created and
last confirmed the cell; rejected ground observations are kept only in bounded
counters/log samples and never become obstacle-memory provenance.

## Risk And Distance Fields

The planner builds one occupied distance field from merged raw occupancy.
Distance below `critical_distance_m` is the critical tier; distance below
`preferred_distance_m` is the planning tier. These tiers are categorical
preferences, not materialized occupied grids.

Raw occupied and outside-grid samples remain hard rejects. The same field
supplies GPU collision queries, risk exposure, and diagnostics.

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

When debugging, compare current lidar points, obstacle memory, raw obstacle
grid, and static map in RViz. The layers should agree in map coordinates even
though they are produced by different stages.

## Mapping Diagnostics Checklist

For an unexpected replan, inspect:

1. Was the raw occupied intersection caused by static map, memory, or current
   lidar?
2. Did the raw occupied cell actually overlap the trajectory, or was only a
   non-collision risk band entered?
3. Was the intersecting span ahead of the drone or already behind it?
4. Did planning clearance get mistaken for hard prohibited space?
5. Did pose or attitude compensation shift the lidar overlay?
6. Did obstacle memory keep an old obstacle longer than expected?
7. Did both lidar ingestion paths retain the same projected obstacle return?
8. Did the planner retain the previous trajectory while rebuilding?

Answering these questions usually separates a real obstacle from a mapping
artifact.
