# Obstacle Mapping

Obstacle mapping owns the 2D lidar-memory input used by no-static planning.
Static Occupancy3D is a separate source loaded directly by the production
planner. The main rule is that raw sources stay raw: distance-based risk tiers
do not inflate hard occupancy.

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
Gazebo SDF and is used only in static mode. `production_mppi_node`, not
`obstacle_memory_node`, owns this map. The current static path does not fuse 2D
lidar memory into Occupancy3D.

## Lidar Input

`obstacle_memory_node` first resolves one strict full-6DoF acquisition pose for
every `/scan` beam. Only then does it integrate accepted beams into scored
memory and publish the resulting runtime state. An unresolved temporal binding
rejects the complete scan before either hit integration or free-space carving.

Important parameters:

- `max_lidar_range_m`
- `range_hit_epsilon_m`
- calibrated sensor time offset and attitude compensation settings;
- source timestamp receive-delay and future-skew limits;
- latest-lidar safety scan age limit in the planner.

The vehicle can accelerate in any horizontal body direction, so the shipped
2D lidar covers the full 360-degree horizontal sector. It retains 720 samples;
the wider sector therefore does not increase scan size or DDS traffic. This is
still a single horizontal 2D lidar, not a 3D perception system. Complete
azimuth coverage is required so a backwards or sideways stopping path cannot
fall into a sensor blind sector.

Lidar evidence is never filtered against hand-authored passage geometry. Static
planning reads Occupancy3D. No-static uses the 2D lidar-memory result, and its
runtime sensor mask exposes channel masses plus collisionless connector
occluders as ordinary obstacles. Every current channel has an occluder across
its intersection and each open bridge. See `world3d.md` for that mode contract.

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

Mapping activates after the vehicle first reaches `min_mapping_altitude_m` and
remains latched for the airborne mission. Descending through a low channel does
not freeze lidar snapshots.

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
participates in risk scoring, lattice search, MPPI, or trajectory control. It is
published on `/drone_city_nav/obstacle_memory_provenance` and in the atomic
`/drone_city_nav/obstacle_memory_snapshot` at the standalone debug cadence. The
planner does not deserialize this provenance; it receives a lightweight status
heartbeat and the raw runtime obstacle snapshot.

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
The calibrated sensor time offset is applied to the first-beam stamp before any
position or attitude lookup; all subsequent beam stamps derive from that
adjusted stamp. Receive time is diagnostic only and is never substituted for a
missing sensor stamp. Position XYZ and quaternion attitude are sampled for
every beam from the same bounded acquisition-time history. `TimesyncStatus`
recovers PX4-local acquisition time from the DDS-adjusted `timestamp_sample`,
and a bounded affine mapper relates PX4-local time to ROS simulation time. XYZ
is interpolated linearly and the complete body-to-NED quaternion is interpolated
with SLERP at the same requested timestamp.

Production mapping accepts only `source_timestamp_aligned`. The clock mapper
must be ready, source timestamps must be valid and close to their receive time,
and both histories must cover every beam. Receive-time alignment and the legacy
velocity extrapolation remain library diagnostics but are not accepted for
mapping or lidar-debug projection. Failure rejects the whole scan, leaving both
occupied and free memory unchanged.

`lidar_pose_latency_s` is retained as a configuration-compatible name for the
calibrated sensor time offset. A positive value samples both position and
attitude later than the raw scan stamp. Diagnostics report the adjusted stamp,
bracketing samples, interpolation/extrapolation age, mapper residual, and the
single accepted pose source.

## Motion Compensation

Lidar projection accounts for:

- PX4 heading;
- acquisition-time position and attitude;
- a calibrated sensor time offset;
- attitude compensation;
- a full rigid body-to-lidar extrinsic.

When PX4 heading is enabled, projection and position-history insertion remain
disabled until the configured consecutive heading samples are stable. The
handoff starts a new pose-history generation, so interpolation cannot bridge
startup or quality-loss samples into valid PX4-heading geometry.

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
`lidar_extrinsic_quaternion_lidar_flu_to_body_frd` in obstacle memory and lidar
debug alike.

The same concepts appear in obstacle-memory and lidar-debug configuration.

## Latest-Scan Safety

Each strictly aligned scan also publishes its actual hit endpoints on
`/drone_city_nav/latest_lidar_safety_scan`. Endpoints are expressed in a fixed
body-FRD frame at the adjusted first-beam acquisition pose. The message contains
no persistent-memory cells and no free-space interpretation. Tracked drone hits
are filtered before publication, using the same input scan as mapping.

The planner reconstructs those endpoints in `map` and, while the scan is fresh,
checks every swept horizon sample against the configured physical 3D cylinder
of the vehicle. A hit blocks execution only when it intersects that physical
volume. A point above or below the body does not create a vertical column, and
no radius beyond the real footprint is added. Missing or stale latest-scan data
does not turn unknown space into a hard zone; the existing raw world and normal
safety lifecycle continue to apply.

This safety input is intentionally independent of scored persistent memory. A
later free-space update cannot erase the physical return from the latest scan
before the planner evaluates it. When persistent memory is disabled, the same
node runs in a lightweight safety-only mode: it keeps acquisition-time pose
histories and publishes latest-scan returns, but it does not allocate a memory
grid, integrate hits, publish snapshots, or start the memory diagnostics worker.
Static intercept GUI runs additionally gate diagnostic memory by the latched
spectator target. Only the selected interceptor integrates and publishes this
memory; all other vehicles remain in latest-scan safety mode. No-static mode does
not apply this gate because every vehicle requires its own persistent map for
navigation.

## Per-Beam Expected-Surface Rejection

Obstacle memory creates one immutable `LidarBeamObservation` and one ingestion
decision before changing its grid. The decision compares the measured range
with the nearest configured expected surface along the map-frame ray. In the
current production configuration, the flat ground plane is the only enabled
expected-surface provider; no legacy passage/known-solid classifier is loaded.

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

Before the 2D grid is updated, a confidence stage separates certain obstacles
from uncertain candidates. Confident obstacles before the expected ground
surface are integrated immediately with the normal memory hit weight. Low
contradictory ground returns and unknown returns with uncertain timestamp
alignment or range-limit geometry remain pending. A pending candidate performs
no hit update and no free-space clearing.

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
are reported as `unavailable`. The generic ingestion library can represent
multiple expected-surface providers, but production does not configure a static
channel provider.

The projected-altitude filter remains a final non-mutating veto. Ground
classification happens first for diagnostics, including beams whose endpoint
is below `min_projected_lidar_altitude_m`, but an `altitude_rejected` beam still
cannot mutate the grid.

## Raw Occupancy And Soft Risk

Raw obstacles are direct evidence. The planner merges raw sources once and
builds an occupied-distance field. It does not materialize prohibited or
planning-clearance occupancy grids.

The current default critical boundary is 1 m and the preferred boundary is 6 m.
Raw occupied cells and evaluation bounds are hard rejects. MPPI first selects
the best eligible risk class, then applies its soft dynamics and progress cost
within that class.

## Memory Transport

Every accepted memory update publishes a small `ObstacleMemoryStatus` carrying
the producer identity, monotonically increasing sequence, occupied count, and
which larger artifacts were emitted for that update. The planner consumes this
message for memory revision diagnostics without receiving the grid or sparse
provenance payload.

No-static runtime planning receives `RawObstacleSnapshot`, which contains the raw
2D grid and risk-policy identity but no sparse diagnostic provenance. It is
published after every accepted memory update and moved into the planner as an
immutable revision. Static planning does not consume it; canonical Occupancy3D
is its authoritative world.

`ObstacleMemorySnapshot` remains an atomic raw-grid/provenance artifact for
offline auditing and debug consumers. It, the standalone grid/provenance topics,
and the 3D diagnostic point cloud default to a 1 Hz cadence. This avoids repeated
multi-megabyte provenance assembly and DDS deserialization in the runtime path.

## RViz Outputs

Useful visualization topics:

- `/drone_city_nav/static_map_points`
- `/drone_city_nav/obstacle_memory_grid`
- `/drone_city_nav/obstacle_memory_provenance`
- `/drone_city_nav/obstacle_memory_snapshot`
- `/drone_city_nav/obstacle_memory_status`
- `/drone_city_nav/raw_obstacle_snapshot`
- `/drone_city_nav/raw_obstacle_grid`
- `/drone_city_nav/latest_lidar_safety_scan`
- `/drone_city_nav/lidar_debug_points`
- `/drone_city_nav/raw_lidar_hit_points_3d`
- `/drone_city_nav/remembered_lidar_points`
- `/drone_city_nav/raw_occupied_cells`
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

Static Occupancy3D represents known world geometry and remains stable across a
static run. No-static obstacle memory integrates fresh sensor evidence and
keeps observed obstacles available after they leave the instantaneous scan.

These are alternative production planning sources, selected by mode:

- static: canonical Occupancy3D + precomputed chunked ESDF3D -> local ESDF3D;
- no-static: accumulated 2D lidar memory -> ESDF2D.

They are not merged in the current implementation. Each occupied distance
field turns its selected raw source into risk tiers.

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
