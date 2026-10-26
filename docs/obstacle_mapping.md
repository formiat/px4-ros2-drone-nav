# Obstacle Mapping

Obstacle mapping owns the revisioned 3D obstacle memory — a voxel occupancy
grid in the manner of OctoMap, fed by the 3D lidar or the stereo depth — used
by no-static production planning. Static Occupancy3D is a separate source loaded directly by
the production planner. The main rule is that raw sources stay raw:
distance-based risk tiers do not inflate hard occupancy.

## Static World

The static map is loaded from the Occupancy3D artifact of the environment being
flown, configured by:

```yaml
use_static_map: true
static_occupancy_3d_path: <environment occupancy3d>
```

Both are empty by default because every current mission runs no-static. The
sparse static world describes the same geometry as the Gazebo world and is used
only in static mode. `production_mppi_node`, not `obstacle_memory_3d_node`,
owns this map. Derived regions, portals, and traversal edges are stored in the
separate fingerprint-bound FreeSpaceTopology3D artifact. No environment has a
3D static map yet (roadmap item 11); a static-map request refuses to start
without one.

## 3D Lidar Input

`obstacle_memory_3d_node` receives an organized `PointCloud2` from the Gazebo
GPU lidar. The known beam geometry reconstructs both finite hits and maximum
range misses. One timestamp-aligned full-6DoF acquisition pose transforms every
accepted beam into map coordinates; an unresolved pose bracket rejects the
complete scan instead of mixing times or frames. The important parameters are
`max_lidar_range_m`, `range_hit_epsilon_m`, the calibrated sensor time offsets
and attitude compensation settings, the source-timestamp receive-delay and
future-skew limits, and the latest-sensor safety scan age limit in the planner.
Sensor evidence is never filtered against hand-authored route geometry. A scan whose bracket has not
arrived yet waits for it, bounded by `lidar_scan_alignment_maximum_wait_s`, and
keeps its place ahead of the scans that arrive meanwhile; those coalesce to the
newest behind it. Letting each newer scan replace the waiting one starved the
memory whenever the bracket wait came close to the scan period: the sensor
latency and the PX4 time mapping put the requested pose about a hundred
milliseconds after the scan stamp, so every scan was replaced by its successor
just before its own bracket arrived, and one recorded flight took off on a
single integrated scan and flew five seconds into an unobserved platform.

The node removes returns from the physical sensor body and typed dynamic-agent
volumes, publishes the fresh hit set for immediate finite-path validation, and
integrates hit/miss rays into sparse `unknown/free/occupied` `Occupancy3D`.
Transport publishes a revisioned base snapshot and dirty chunks rather than a
complete dense map on every scan. Unknown voxels remain distinct from occupied
and confirmed free voxels.

## Vision Returns

On the default sensor set the memory's producer is vision (roadmap item 14):
with `NAVIGATION_SENSOR_PROFILE=stereo_tof` the launch's one
`obstacle_memory_3d_node` integrates the returns below on the production
topics, with the overrides of `launch/sensor_profile.py` (hit-only returns, no
surface interpolation, the left camera's mount as the extrinsic, two hits to a
voxel). With `NAVIGATION_SENSOR_PROFILE=lidar` and the camera set mounted, a
second instance integrates them in shadow on `/drone_city_nav/vision_shadow/*`
beside the lidar's, and nothing consumes it.

What a vision memory does not have is the lidar's view all around: space
beside and behind the vehicle stays unknown until the pair has faced it.
Unknown space is traversable as before, so what protects the vehicle there is
the speed law: a motion the vehicle does not face is admitted what this memory
has observed along it ([`trajectory_optimization.md`](trajectory_optimization.md)).
Over the accepting flights 95.5 to 97.1 percent of the flown path had been
observed before it was entered.

`stereo_depth_node` feeds the same beam contract the lidar does. `stereo_depth_node` recovers depth from
the forward stereo pair and publishes one ray a matched pixel (every fourth
pixel each way) in the left camera's forward-left-up frame, stamped with the
frame:

- a pixel matched within the confident depth is a **hit**. The confident
  depth is where the depth error `z^2 * e / (b * f)` reaches one 0.25 m voxel
  for the disparity error measured at p90 (0.68 px on r1476, the pair's
  noise in effect; 0.45 px and 6.4 m on r476 without it): 5.2 m for the
  1280 px, 120 degree, 0.20 m pair (specification K7);
- a pixel matched deeper than that is a **free ray**: no surface measurement,
  but the surface is no nearer than the disparity plus its error allows, so
  the ray is free up to that depth or the confident depth, whichever is
  nearer;
- a pixel **without a match says nothing**, where an organized lidar scan
  reads a missing return as a miss to the maximum range. Since roadmap item
  18 stage 0 the lidar's scan is read for its failure patterns first
  (`lidar_scan_patterns_3d.cpp`, specification K37): weak returns among
  weak returns are an aerosol, a miss to the return and fail evidence there;
  a small bundle of missing returns in a near, flat border of returns is a
  dropout, a miss to the border's range and fail evidence there; a missing
  return among missing returns stays a free ray. A time-of-flight zone
  returning under half a surface's intensity is smoke (K38): its rays are
  fail evidence too, with the cloud's `intensity` of 2.

The two time-of-flight sensors join the same cloud. A zone is a cone, not a
ray: its evidence is spread over 3 x 3 rays, a surface filling the zone's
cross-section at the measured range, a zone that saw nothing free across it
to the rated 2.8 m. The cloud's `intensity` field is 1 for a surface and 0
for a free ray; `decodeHitOnlyReturns3D` turns it into the beams the memory
integrates (`lidar_3d_hit_only_returns`).

The matcher is OpenCV's semi-global matching, far and near apart: the far
surfaces at full resolution over 64 disparities, the near ones at half
resolution over the whole 256. On the recording flight r476 that recovers the
same depth as one full search (0.97, 0.84 and 0.66 of the pixels within one
voxel of the simulator's depth at 4 to 6, 6 to 8 and 8 to 10 m) for 108 ms on
two threads against 187.

A second instance of `obstacle_memory_3d_node`
(`vision_obstacle_memory_3d_node`) integrates these returns on topics of its
own under `/drone_city_nav/vision_shadow/`, with the lidar node's parameters
and two hits to call a voxel occupied: a matcher's outliers beside depth
edges do not repeat from frame to frame the way a surface does. Nothing
consumes that memory; the lidar's stays the navigation world. Measured against
the lidar memory over whole flights: with one hit 93.7 percent of the
vision-occupied voxels had a lidar-occupied voxel within one voxel and 4.6
percent none within two (r481); with two hits 98.4 and 0.7 percent, and 98.8
percent of the lidar's occupied voxels inside the volume the pair observed
were found (r483); with the free rays and the time-of-flight zones 97.1
percent within one voxel and 99.4 within two, 96.0 and 97.8 percent found,
over 35 percent of the lidar's observed volume (r487).

## Smoke In The Simulator

Roadmap item 18 stage 1 (specification A13). A smoky place of a flight is a
box of white smoke, `SMOKE_ZONES="x,y,z,sx,sy,sz,sigma"` in the world frame
(several separated by `;`, `SMOKE_SEED` the seed of its form), sigma its
extinction coefficient of visible light per metre; the visibility through it
is about 3/sigma metres. Its place and its amount never change; its form
churns, two octaves of value noise drifting through it (1.2 m at 0.15 m/s and
0.35 m at 0.3 m/s, the density between about half and one and a half of
sigma), and it thins out over the half metre inside each face.
`simulation_smoke_sensor_node` (started by the launch when the zones are set)
reads every sensor of the vehicle from Gazebo, lays the smoke over it and
publishes it again under `/smoke`, where the vehicle's drivers read it; a
message none of whose rays crosses smoke passes unchanged, and nothing of the
vehicle reads the zones. RViz draws them as grey boxes, the denser the less
transparent (`scripts/evaluation_overlay.py`).

**Why a model and not Gazebo's particles.** Measured on the vehicle's own
sensors (`tools/smoke_rig.py`, its levels in `log/tools/part2/final`): the
`ParticleEmitter`'s particles are unlit, so the pair saw them at grey 99 with
the light out and no ambient light at all, and a light set aside lit them
exactly as one at the pair; a depth sensor stops on the first particle with a
fixed chance, whatever the path through the plume, and the documented
`particle_scatter_ratio` does not set that chance (at 10 particles a second
0.38 of the lidar's forward rays returned from the particles at a ratio of 0.1
and 0.36 at 0.9; 0.19 and 0.31 at 3 and 10 a second); the simulator's own
depth camera, the evaluation's truth, scatters on them as well; `<scene><fog>`
acts on no sensor under ogre2 (the wall matched whole with it and without).
The thermal camera does not see the particles at all and the model leaves it
as it is: the wall reads 311 K at 5 m at every level; the thermal channel's
response to smoke is stage 3's.

**The pair.** Each frame is the clean one seen through the smoke both ways
and lit through it, plus the light of the carried light the smoke scatters
back: single scattering by an albedo of 0.9 and a Henyey-Greenstein phase of
0.45, marched along one ray per 8 x 8 pixels in 0.1 m steps up to the clean
frame's own depth (the evaluation depth cameras of both sides, rendered only
for the model) and interpolated per pixel. The light's brightness on a surface
is the simulator's own, measured on a matte panel on the light's axis from
0.25 to 15 m (`tools/smoke_rig_report.py --calibrate`). The camera's noise
goes down with the surface it came with and is added again as Gazebo adds it.
A frame costs 20 to 70 ms on four threads, the most with smoke round the
vehicle.

**A surface on a voxel boundary** keeps both its layers: the sensor's range
noise lands its hits on either side of the face, and once the rays grazing
the surface carve neither layer (K42 below) each holds its hits. The rule
that marked both voxels on every such hit (K41, 2026-10-15) thickened a
quarter of the surfaces toward the air and cost the lidar's ordinary flights
15 percent of their speed (r1601 to r1605); it is withdrawn.

**A ray grazing a plane of the grid** (specification K42) carves no voxel
that holds a surface: within 15 degrees of a plane it crosses such a voxel
without telling whether a surface parallel to that plane lies under or
beside its path. From a hover the lidar's rays graze the floors and the
corridor walls, hit them in rings a row apart and, before this rule, carved
the voxels between the rings free within seconds; the proof of item 19 then
leaked through the floor (r1564, r1566); with it, the lidar's blocked flights gave the goal up by the proof in five of five (r1576 to r1580).

**The time-of-flight ring and the lidar.** Each ray reports the strongest
echo over the sensor's threshold, from the smoke or from the surface behind
it, by the lidar equation: an echo is the backscatter met over the square of
its distance, both ways through the smoke; the smoke's backscatter is its
extinction over a lidar ratio of 53 sr (the albedo and the phase above; 40 to
70 sr is measured for smoke), summed over the pulse's range gate (a metre for
the lidar, a pulse of some nanoseconds; a decimetre for a time-of-flight
unit's histogram bin), the threshold that of a surface of 0.1 at the lidar's
maximum range (its datasheet's 10 percent reflectivity) and of 0.5 at a
time-of-flight unit's, and the receiver takes the whole beam in from 1 m for
the lidar and from 0.25 m for a time-of-flight unit. The lidar reports its last echo over the threshold, as a lidar
driver set to last return (roadmap item 23: through smoke of 0.1 or 0.2/m
it reads the wall 8 m on, which the strongest echo, the smoke a metre off,
had hidden), a time-of-flight unit its strongest. A smoke return lies
somewhere in its 0.1 m bin and moves with the churn from scan to scan; the
intensity is the echo over a matte surface's at that range. The two overlap
distances are chosen, not measured on a real unit: with them a time-of-flight
unit is blinded by smoke round it once its visibility falls under about 4 m,
not by a haze. Read by bins against a surface of 0.5, the lidar could not see
a 3/m plume from 4 m and flew into the one closing a corridor at 5.7 m/s
(r1546, r1547); with the gate it returns from such a plume from about 20 m,
from 1/m from 15 m and from 0.3/m from 9 m, and inside dense smoke from
0.35 m (log/tools/part6/lidar_range_gate1_r01).

**What each sensor sees.** A wall 5 m ahead, a box of smoke 1 to 4 m ahead,
the light at the pair, the noise of specification A12 (the clear wall at 7.7
m: 0.87 matched):

| sigma, 1/m (visibility) | Wall grey, contrast | Pair: wall matched | Pair: matched in the smoke | Front time-of-flight | Lidar forward: smoke, wall |
|---|---|---|---|---|---|
| 0 | 73, 16.5 | 1.00 | 0 | nothing (the wall beyond 2.8 m) | 0, 1.00 |
| 0.05 (60 m) | 71, 13.1 | 1.00 | 0 | nothing | 0, 1.00 |
| 0.1 (30 m) | 69, 10.3 | 0.98 | 0 | nothing | 0, 1.00 |
| 0.2 (15 m) | 67, 6.4 | 0.81 | 0 | nothing | 0, 1.00 |
| 0.3 (10 m) | 66, 3.9 | 0.38 | 0 | nothing | 0, 1.00 |
| 0.5 (6 m) | 66, 1.7 | 0 | 0 | nothing | 0.23, 0.77 |
| 1 (3 m) | 68, 0.9 | 0 | 0 | nothing | 1.00, 0 |
| 2 (1.5 m) | 71, 0.9 | 0 | 0 | nothing | 1.00, 0 |

The pair loses the wall behind about one optical depth (0.3 per metre over
3 m), and smoke never becomes a matched surface: its veil is a smooth grey
the noise mask refuses, so a plume is silence to the pair, as distance is,
not a wall. With the noise of before A12 (0.01, a tenth of a level) the
wall was matched whole through 0.2 per metre and 0.15 of it through 0.5. The
lidar sees through 0.3 per metre and returns from the smoke's face from 0.5;
the front time-of-flight unit sees nothing of smoke outside it, even of 2
per metre 0.4 m ahead.

With the smoke round the vehicle (3 m behind to 4 m ahead, the wall 5 m
ahead): the pair matched 0.99, 0.58 and 0 of the wall at 0.1, 0.2 and 0.5 per
metre; every time-of-flight zone read the smoke at 0.05 m from 1 per metre
and none at 0.5; the lidar returned from the smoke from 0.5. **The light set
apart** (0.45 m aside and 0.55 m below the pair, the remedy item 18 named)
lowered the veil (grey 67, 64 and 58 against 69, 68 and 68) and matched less
of the wall (0.91 and 0.44 at 0.1 and 0.2): what blinds the pair is the wall's
light lost both ways through the smoke, not the veil.

**The camera estimator** (`tools/vio/replay_vio.cpp` over the pair as the
camera driver passes it on, an IMU of the motion alone, the rotor drag term
off for a carrier that is no multicopter): standing still 30 s before a
plume 1 m away, the features tracked fall from 135 (clear) to 142, 84 and 69
at 0.2, 0.5 and 1 per metre, and the estimate stays within 0.07 m. Flying 11
m at 1 m/s through the 3 m box:

| sigma, 1/m | Features, mean (least) | Error at the end | Largest error |
|---|---|---|---|
| 0 | 134 (82) | 0.04 m | 0.47 m |
| 0.2 | 134 (82) | 0.13 m | 0.59 m |
| 0.5 | 124 (54) | 0.24 m | 0.79 m |
| 1 | 110 (0) | 2.2 m | 3.6 m |
| 2 | 104 (0) | 0.83 m | 1.3 m |
| 0.2, 7 m of smoke | 130 (74) | 1.0 m | 1.8 m |
| 0.5, 7 m of smoke | 106 (23) | 2.2 m | 4.6 m |

Smoke the vehicle flies through costs the camera estimator metres over a few
metres of path: what item 23 has to measure and answer first.

**The smoke face** (specification K44, 2026-10-19 to 24): dense smoke ahead
of the stereo set is an obstacle the vehicle sees before it is inside. The
pair is silent on a plume, and the silence is lit (raw grey 20 or more
under the vehicle's own lamp; a shadow or the dark beyond the lamp reads 0
to 10, F13). The look of such a region (its rim's scatter and trend, the
chance matches through it) is kept as its label in the log and tells a
plume from the lit untextured floor of a hall or the veil of thin smoke no
better than chance; the motion does. The plume candidate is the frame's
largest lit rimmed region of 15 000 cells or more in the central band
(the middle three quarters across, a sixth to two thirds down), matched
through at a quarter percent of its cells at most, its depth the rim's in
the central rows. It is confirmed, and its marks laid, while over a window
of the last 2.4 m of travel its depth falls by 0.3 m or more per metre and
its size grows by 0.3 of itself per metre (or by 0.1 with the depth falling
by 0.8), over 1.2 m at least, the vehicle neither climbing nor sinking by
more than four tenths of that travel, the candidate one region from frame
to frame (its centroid's drift judged after the heading's turn). From
inside the halo's edge the travel confirms nothing (the smoke's churn can
carry the halo 9 m out, where the depth reads 1.4 to 2 m from 10 m on,
r2045): the candidate is laid at once when the pair's reach, the farthest
metre of the frame with matched hits in it and the one before, has fallen
to 4 m or under while a lit silence of 40 000 cells, half the frame, fills
it; the thin hall's dark frames keep hits at 5 to 8 m. The face is off unless
the depth node's `smoke_face` parameter is true (`SMOKE_FACE=true` in the
flight's environment; the owner's rule of 2026-10-24, specification K44). A lit body
in the dark with a sharp edge, placed by its brightness, is confirmed by
four frames. The rays enter the memory observed unobservable from 1.5 m
before the face to 1.5 m beyond, as the dark frustum's do (K14), never
nearer than 1.5 m; the motion is bounded 1.5 m before the face while the
plume is laid, a bound fallen to nought being a stop, not an absent bound
(r2047); the front time-of-flight unit's silence through lit silence
is not read as free (K38). The way round a plume is the planner's; a
corridor a plume fills is closed in the memory, and item 19's proof, or the
light's charge on the stereo set (A8), gives the goal up. The rig's
corridor (`tools/smoke_rig.py corridor=4.4,ceiling=3`,
`tools/smoke_signature_report.py`) gave the lit silence; the rules of the
motion were set on the flights' logged candidates replayed offline
(`tools/plume_replay.py`) and judged against the truth
(`tools/smoke_marks_truth.py`).

**The heat source** (specification K49, 2026-10-26): the stereo set's thermal
core sees a fire's body through the smoke that carries it (longwave infrared
crosses 1/m untouched, K39), which the pair, the ring and the lidar read by
density alone. A source is four pixels or more at 30 K over the frame's
median (the location's surfaces are 23 K apart at most); its range the median
of the hits in the hot cone, the ring's or the pair's, else the conservative
5.2 m; the standoff 4.0 m. The mark is a measurement like the face's: the near
half of the standoff ball round the range point enters the memory observed
unobservable every half second while the core reads the source, nothing
nearer than 1.5 m, the pair's returns inside the ball dropped as the plume's
own texture; the motion is bounded at the range less the standoff, a bound
fallen to nought being a stop. Nothing is latched: a frame without the source
lays nothing, and what was laid is lifted as any unobservable mark is, by the
pair's measured rays. The way round the ball is the planner's; a room whose
crossing the ball closes is given up by item 19's proof or the light's charge.

## Obstacle Memory

`obstacle_memory_3d_node` accumulates scan evidence into its sparse
`Occupancy3D`.

Memory uses hit/miss scoring:

- `hit_weight`
- `miss_weight`
- `min_score`
- `max_score`
- `occupied_score`
- `free_score`

Classification is a Schmitt trigger over the score: a voxel becomes occupied
when its score reaches `occupied_score`, becomes free when it drops to
`free_score`, and keeps its current state while the score lies between the two.
A wall surface voxel that collects one hit and a few grazing misses per scan
therefore stays occupied instead of flipping every scan, and consumers of
occupied evidence (planner repair, route validation, tube profiles) are not
re-invalidated at the scan rate by evidence that has not really changed.

Mapping activates after the vehicle first reaches `min_mapping_altitude_m` and
remains latched for the airborne mission. Descending through a low passage does
not freeze lidar snapshots.

`obstacle_memory_3d_node` publishes the selected-spectator accumulated cloud
from occupied voxel centers and separately publishes the current scan on
`/drone_city_nav/current_lidar_returns_3d`. Neither visualization cloud is a
planner input.

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

The scan stamp is the acquisition instant. Each pose source is sampled at
that instant plus its own signed offset, `lidar_position_source_time_offset_s`
for the PX4 local position and `lidar_attitude_source_time_offset_s` for the
PX4 attitude (`motion_compensate_lidar_pose: false` zeroes both). A negative
value reads the source earlier than the stamp. Diagnostics report both
offsets, the bracketing samples of each source, interpolation/extrapolation
age, mapper residual, and the single accepted pose source.

The simulated 3D GPU lidar runs with a position offset of `0` and an attitude
offset of `0`. Both were measured against the true vehicle pose from Gazebo:

- the position estimate stamped `t` matches the true position of `t`
  (-0.003 s along the motion over 6258 samples of the urban flight r312, read
  after aligning the clocks on the speed profile). It used to match the true
  position of `t + 0.09..0.12 s` (r203, r205), which was not the vehicle's
  latency but EKF2 subtracting its default 110 ms GNSS delay from a simulated
  sample the Gazebo bridge stamps at receipt; the run script sets
  `EKF2_GPS_DELAY 0`, and the `-0.12 s` that compensated the lead here is
  gone with it;
- the attitude estimate stamped `t` matches the true attitude of `t`
  (0.16 degrees mean in r203/r205, already 1 degree at ±60 ms);
- the body-frame hits placed with the true pose of `stamp + tau` are most
  consistent from one scan to the next at `tau = 0` (0.04 m median
  nearest-neighbour distance between scans 0.5 s apart during manoeuvres,
  against 0.20 m at `tau = -0.12 s`): the returns belong to the stamp.

The former single offset of `-0.12 s` for both sources came from projecting
scans against the world mesh, a test that sees position and not attitude. It
placed the returns correctly and tilted them by the attitude the vehicle had
120 ms earlier, 4–7 degrees at the angular rates of a turn, which rocked the
current cloud in RViz and smeared the persistent memory on every manoeuvre.
Re-run the three measurements when the sensor model, its update rate, the
estimator or the simulator changes.

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
center is `(0.12, 0.0, 0.315)` from base_link in FLU, represented as
`[0.12, 0.0, -0.315]` in body FRD. Base_link, the body the autopilot
localizes, stands 0.24 m above the vehicle model's origin (the x500 base
model's own pose), so in the model's frame the lidar is included at 0.50 m
and the camera set at 0.24 m; included by the model frame's numbers they sat
0.24 m lower than these extrinsics and the memory stood a voxel above the
world (specification K22). The aligned FLU-to-FRD quaternion is
`[0.0, 1.0, 0.0, 0.0]` in WXYZ order. Both ray direction and the complete XYZ
lever arm rotate with the interpolated body quaternion before NED is converted
to the map Z-up convention.

`lidar_z_offset_m` and mount RPY remain only as a compatibility fallback when
`use_full_lidar_extrinsic=false`. The normal configuration uses
`lidar_extrinsic_translation_body_frd_m` and
`lidar_extrinsic_quaternion_lidar_flu_to_body_frd` in obstacle memory and lidar
debug alike.

The same concepts appear in obstacle-memory and lidar-debug configuration.

## Latest Raw Obstacle Scan

Every accepted, acquisition-time-aligned scan publishes its actual hit endpoints
on `/drone_city_nav/latest_sensor_obstacle_scan`. Endpoints are expressed in a
fixed body-FRD frame at the adjusted first-beam acquisition pose. The message
contains no persistent-memory cells, inflation, clearance boundary, or free-space
interpretation. Dynamic-agent hits selected by the mapping contract are filtered
before publication.

The planner reconstructs the endpoints in `map`. While the scan is fresh, it
validates the complete finite path against the configured physical 3D footprint,
including every sample of its nominal motion and arrival profile. A hit
rejects a path only when it intersects the real swept vehicle volume. Missing or
stale scan data does not turn unknown space into a hard zone; persistent raw
occupancy remains the normal world source.

This direct raw evidence is independent of persistent-memory integration latency.
It does not implement a separate stopping controller or obstacle-triggered latch.
The planner still constructs one self-contained finite path whose endpoint is at
rest. Proximity and ESDF clearance remain soft ranking costs.

When persistent memory is disabled, the obstacle-memory node keeps the pose and
scan-alignment lifecycle but does not allocate a memory grid, integrate hits,
publish snapshots, or start the memory diagnostics worker.
Static multi-vehicle GUI runs additionally gate diagnostic memory by the latched
spectator target. Only the selected vehicle integrates and publishes this
memory. No-static mode does not apply this gate because every vehicle requires
its own persistent map for navigation.

## Raw Occupancy And Soft Risk

Raw obstacles are direct evidence. The planner selects the mode-authoritative
raw source and builds derived occupied-distance evidence. It does not
materialize prohibited or planning-clearance occupancy grids.

The current default critical boundary is 1 m and the preferred boundary is 6 m.
Raw occupied cells and the physical flight envelope are hard rejects. Missing
or outside-cache distance evidence is neutral. MPPI applies strong
critical- and planning-exposure costs together with its dynamics and progress
costs; a risk band does not independently remove a physically free rollout.

## Memory Transport

Every bounded transport update publishes a small `ObstacleMemoryStatus` carrying
the producer identity, monotonically increasing sequence, occupied count, and
which larger artifacts were emitted for that update. The planner consumes this
message for memory revision diagnostics without receiving the grid.

No-static production planning receives an adaptive `RawObstacleSnapshot3D` base and the latest
cumulative `RawObstacleDelta3D` dirty chunks relative to that base. The direct
current-scan safety message is published before persistent-memory integration.
Memory integration and DDS serialization run on separate coalescing workers, so
an expensive snapshot cannot make the physical scan stale. Static planning
consumes neither sensor-world transport; raw Occupancy3D is its
authoritative world.

The 3D diagnostic point clouds use bounded cadences. The 3D runtime transport
coalesces superseded revisions, publishes cumulative deltas at a configured
rate, and rebases only when the delta approaches snapshot size or the maximum
base age expires. This avoids rebuilding a multi-megabyte full snapshot for
every scan. The rate is the scan rate, 10 Hz
(`obstacle_memory_3d_transport_rate_hz`): DDS delivers an update in 0.15 ms,
so the age the planner sees is the publication period plus the memory's own
scan-to-publication time, and at 2 Hz that age sat at 636 ms at p95 against
the 600 ms the braking contract charges; the measurement is in
[resource_budget.md](resource_budget.md).

## RViz Outputs

Useful visualization topics:

- `/drone_city_nav/static_map_points`
- `/drone_city_nav/obstacle_memory_grid`
- `/drone_city_nav/obstacle_memory_status`
- `/drone_city_nav/raw_obstacle_snapshot_3d`
- `/drone_city_nav/raw_obstacle_delta_3d`
- `/drone_city_nav/latest_sensor_obstacle_scan`
- `/drone_city_nav/current_lidar_returns_3d`
- `/drone_city_nav/raw_lidar_hit_points_3d`
- `/drone_city_nav/raw_memory_obstacle_points_3d` (the surfaces) and
  `/drone_city_nav/raw_memory_darkness_points_3d` (the space observed
  unobservable, specification K14: occupied without a measured hit)

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

These are the two production planning sources, selected by mode:

- static: raw Occupancy3D + precomputed chunked ESDF3D -> local dense
  controller projection;
- no-static 3D: revisioned observed Occupancy3D -> exact capped dense
  `KnownObstacleDistance3D` over the chunk-aligned local window -> controller
  projection shared without copying.

They are not merged in the current implementation. Each occupied distance
field turns its selected raw source into risk tiers.

## Risk And Distance Fields

The planner builds one occupied-distance field from the selected raw occupancy.
Distance below `critical_distance_m` is the critical tier; distance below
`preferred_distance_m` is the planning tier. These tiers are categorical
preferences, not materialized occupied grids.

Raw occupied samples and the physical flight envelope remain hard rejects.
Missing or outside-cache distance evidence is neutral. The derived field
supplies soft risk exposure and bounded controller queries; exact raw occupancy
remains authoritative for hard swept-footprint validation.

The no-static field indexes only confirmed occupied sources that can influence
the capped local output window. Immutable 8-cubed chunks are shared across
compatible revisions, while inserted or removed sources invalidate only their
bounded output chunks. Free and unknown labels are deliberately absent from this
cache identity and remain equal distance-query inputs.

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
