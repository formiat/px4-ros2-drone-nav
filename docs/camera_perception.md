# Camera-Based 3D Perception Without Lidar Or Static Maps

This is the design record of roadmap item 14, completed on 2026-09-19: what
the item set out to do, the decisions taken before implementation with the
numbers behind them, and what each stage measured. The summary and the
acceptance figures are in the Completed section of
[`roadmap.md`](roadmap.md). The text below the first section is the item as it
stood while it was open; where a stage's measurement corrected an estimate,
the correction is recorded beside it.

## What Was Built And What It Measured

- **Sensors** (`models/stereo_tof_v1`, [`gazebo_simulation.md`](gazebo_simulation.md)):
  a forward pair of 1280 x 960 cameras, 120 degrees, 0.20 m baseline, 7.5 Hz,
  and two 8 x 8 time-of-flight sensors, 45 by 45 degrees, 2.8 m, looking up
  and down. `NAVIGATION_SENSOR_PROFILE=stereo_tof` removes the lidar from the
  vehicle model.
- **Depth** (`stereo_depth_node`): semi-global matching, far field at full
  resolution over 64 disparities and near field at half resolution over 128,
  110 ms on two CPU threads. Disparity error 0.11 to 0.17 px at the median,
  0.29 to 0.49 px at p90; depth it stands behind out to 6.4 m.
- **Returns** ([`obstacle_mapping.md`](obstacle_mapping.md)): every ray is a
  hit, a free ray or nothing; a time-of-flight zone is a cone of 3 x 3 rays.
  The launch's one `obstacle_memory_3d_node` integrates them as it integrated
  the lidar's, two hits to a voxel.
- **Speed and gaze** ([`trajectory_optimization.md`](trajectory_optimization.md)):
  the braking contract reads 6.4 m and a 2.0 m margin inside 60 degrees of
  the heading and 52.4 degrees of the horizon (2.452 m/s), 2.8 m and a 1.0 m
  margin inside 22.5 degrees of the vertical (1.29 m/s); the gaze turns the
  heading to where the horizon moves over 1.5 s, and to the route's tangent
  while the vehicle rests, in a position hold as well; a motion the vehicle
  does not face is admitted what memory has observed along it, the body's
  radius wide, and nothing where it has observed no more than the margin.
- **What the flights found.** r498: a goal capture broken by one lost
  feedback sample could not be taken again over the resident hold 0.03 m
  away. r500, the one crash of the stage: the vehicle climbed a shaft facing
  south-west, left it northward at the 1 m/s a fixed "unobserved speed" then
  admitted, and met a wall 1 m away that no sensor had looked at; the
  distance field treats unknown as free, so the frontier law read 20 m, the
  edge of its grid. Memory now answers for such a motion and the gaze turns
  the vehicle first. After it: r505 to r510 and r516, seven flights, no crash.
  r518, the first attempt at the final series: the simulator fell behind the
  wall clock, the autopilot's timestamps were reacquired (10 to 15 times a
  camera flight, none or once on the lidar), no horizon could be committed for
  1.7 s, and the resident horizon flew the route's turn at 2 m/s with the pair
  still facing away, into a structure memory first held 0.67 s before the
  contact. The speed law bounds the tick; a horizon owns the vehicle for its
  lease after it. The assembler's candidate validator now refuses a horizon
  whose states carry speed along a motion no sensor sees at their own planned
  heading, faster than memory admits, and the arrival search ends it at rest
  before that motion.
- **Acceptance** (r506 to r510 on 948d4df2): five of five, 1.48 to 1.67 m/s,
  route availability 97.6 to 98.8 percent, planner p95 152 to 158 ms, 95.5 to
  97.1 percent of the flown path observed before it was entered, 40 percent
  of the planned ticks flown without facing the route and 4.7 percent of all
  ticks waiting for the gaze (r505). The final series on the defaults, with
  the horizon rule (r523 to r527 on cd461d01): five of five, 1.17 to 1.57 m/s,
  availability 98.2 to 99.2 percent; the rule costs about a fifth of the mean
  speed, and one flight of the five is under the gate.
- **Decisions the numbers made.** The speed gate of the stereo profile is
  half of what the contract admits forward: 1.226 m/s, not the 1.75 m/s
  estimated from 8.6 m of expected depth. Camera flights use the `gnss`
  localization profile: beside the textured render, the image bridge and the
  matcher the workstation does not carry the lidar-inertial estimator in real
  time, and without the lidar it has nothing to register. The evidence
  admitted for final revalidation is named latest sensor evidence, because
  "latest raw" already names the raw world's revision.
- **Known to remain.** The evidence age exceeds the contract's 600 ms on
  about 1 percent of the ticks on this workstation; a faced motion between the
  pair's field and a time-of-flight cone is flown at 1 m/s unobserved; lateral
  deviations of the horizon from the route enter unknown space at the
  validators' speed, as they did under the lidar, where the lidar had already
  seen it; the multi-vehicle launch has not been flown on the profile.

## The Item As It Stood

**Type:** dependent perception stage.

**Hard prerequisites:** items 8 and 12.

**Validation prerequisite:** the route availability floor of the 3D-lidar
profile, which item 9 stage A is to derive; until it does, the floor measured
over the fifteen flights r430 to r434, r448 to r452 and r470 to r474 stands in
for it: 90 percent (lowest flight 90.2, median 94.9).

**Validation environment:** Urban Circuit Practice 01.

Navigate the same no-static missions that item 8 and item 12 accept with the
3D lidar, with no lidar at all and no static map: the vehicle carries one
forward stereo pair of video cameras, and software recovers the shape of the
surrounding geometry from the video stream; two short-range multizone
time-of-flight sensors, the kind serial drones carry, cover straight up and
straight down, where a forward camera cannot look. The sensor set is chosen by
price: three stereo pairs cost as much as the 3D lidar they replace, one pair
and two time-of-flight sensors cost a fraction of it. The world model does not
change. Item 8 fixed the boundary between sensing and the raw world as a
bundle of timestamped rays with a hit at a range or a miss to a range,
integrated into revisioned `unknown/free/occupied` `Occupancy3D` under one
full-6DoF acquisition pose. A depth image is exactly such a bundle: one ray
per pixel through the calibrated optics, a hit where the pixel's depth is
known and free space along the ray up to it. Vision therefore enters the
pipeline as a second producer of the same beam observations that
`obstacle_memory_3d_node` already integrates, and everything downstream —
obstacle memory, dirty-chunk transport, immutable raw snapshots, swept
validation, the persistent planner, MPPI, finite raw-safe execution — runs
unchanged. The stage proves that the world model is not bound to one sensor,
which is the property a real vehicle needs before any sensor is swapped or
lost.

### What The Simulator Provides

The simulator provides one calibrated stereo pair of RGB cameras rigidly
mounted on the airframe and looking forward, its intrinsics and baseline, two
multizone time-of-flight sensors looking up and down (VL53L8 class: an 8 x 8
zone matrix over 45 by 45 degrees, 65 on the diagonal, at up to 15 Hz; 4 m of
range in the dark and 2.8 m on a large bright target in 5 000 lux, which is
the figure the simulated sensor takes, since the flights are outdoors; the
manufacturer states none for direct sunlight, where the range is shorter
still), the IMU, and a
pose source that does not need the lidar: the `gnss` localization profile,
kept for this stage, since the default `lidar_inertial` estimator of item 13
registers lidar scans and has nothing to register without them. It provides no
depth camera, no RGB-D sensor and no point cloud in the control path: depth
from a simulated depth sensor is a lidar by another name and would prove
nothing. The two time-of-flight sensors are the stated exception, and an
honest one: 64 rays to 4 m are a tiny flash lidar, they are what a real
vehicle of this price carries, and they see only the two directions the camera
cannot. Everything the vehicle flies towards horizontally is recovered from
video alone. Simulator depth and Gazebo truth occupancy are available to
evaluation and referee components only, as item 13 treated ground-truth pose,
and must never cross into the perception, planning or control data path.

One pair and not three, because of price, in hardware and in the simulator
alike: three pairs are six renders for a simulator that holds a real-time
factor of 1.00 without margin, one pair is two. What one pair costs is the
vertical. A multirotor can turn about yaw before it moves but cannot tilt to
look where it climbs or descends: it is underactuated, its thrust follows the
body axis, and a pitch of 15 degrees held for a look upwards is 2.6 m/s² of
horizontal acceleration, 0.2 to 0.5 m of drift in a shaft whose walls stand
0.5 to 1 m away. The speed policy never moves the vehicle faster than it can
stop within the range at which an obstacle is guaranteed to be detected in
that direction, so a forward pair alone gives every vertical motion a
detection range of zero: the two shafts of Urban Circuit Practice 01, which
item 12's routes climb and descend regularly, would be crawled through or not
entered, against the rule that vertical motion is free. The time-of-flight
sensors give the vertical its guaranteed range, and the braking contract
says what that range is worth (evidence age, reaction latency, the jerk ramp
from the largest vertical acceleration, deceleration at 1.4 m/s², and the
physical margin): with the 2.0 m margin of the lidar profile, 2.8 m admits
0.7 m/s at the lidar's 0.6 s of evidence age and 0.9 m/s at the 0.2 s a
15 Hz sensor owes; with the margin a vertical approach needs (about 1.0 m:
the body's half height, the estimate's vertical error, a voxel and the
tracking error) 1.3 and 1.6 m/s; in the dark, at 4 m, 1.4 to 2.2 m/s. That
is slower than the lidar profile and not a crawl in shade or indoors; in
direct sunlight it is a crawl, and the tilt servo below is then the answer.
Their miss is real evidence, as the lidar's is, inside the rated range. A
zone is a cone, not a ray: at 2.8 m it is 0.3 m wide, and a narrow object in
it returns a mixed range. A zone therefore enters the beam contract as a hit
that fills the zone's whole cross-section at the measured range and as free
space only up to that range less the sensor's ranging error, never as a thin
free ray through a volume the sensor does not resolve. Backward and sideways sensing is not needed:
the gaze policy below turns the vehicle before such motion.

The lens and the baseline are a measured trade. Depth error grows as the
focal length in pixels shrinks, so a wider lens sees more of the vertical and
less far, and a longer baseline sees farther and loses the near field. With a
quarter pixel of disparity error and one voxel (0.25 m) of allowed depth
error the confident range is the square root of baseline times focal length;
the nearest range is baseline times focal length over the largest disparity
(256 px at 1280 px of width). The braking contract then prices the range
exactly as it prices the lidar's 14 m (5.65 m/s): 2.0 m of physical margin,
the evidence age, the reaction latency and the jerk ramp from the largest
acceleration. An earlier version of this section read the speed off the
stopping distance alone and promised 4.8, 3.9 and 3.0 m/s for 5.7, 4.3 and
2.9 m; the contract admits 2.1, 1.3 and 0.4 m/s there.

| Width | Lens | Baseline | Range | Nearest | Forward speed at 0.6 s / 0.25 s of evidence age | Vertical half-angle (4:3) |
|---|---|---|---|---|---|---|
| 640 px | 90° | 0.10 m | 5.7 m | 0.25 m | 2.1 / 2.5 m/s | 37° |
| 640 px | 120° | 0.10 m | 4.3 m | 0.14 m | 1.3 / 1.6 m/s | 52° |
| 1280 px | 90° | 0.20 m | 11.3 m | 0.50 m | 4.6 / 5.4 m/s | 37° |
| 1280 px | 120° | 0.20 m | 8.6 m | 0.29 m | 3.5 / 4.1 m/s | 52° |
| 1280 px | 120° | 0.30 m | 10.5 m | 0.43 m | 4.3 / 5.0 m/s | 52° |

A 640 px pair on a 10 cm baseline is therefore not a candidate: it flies at
1.3 to 2.5 m/s. The working choice is 1280 px, a 120 degree lens and a 20 cm
baseline, which a 0.5 m airframe carries: 8.6 m of confident range, a
nearest range of 0.29 m for the shafts whose walls stand 0.5 to 1 m away, 3.5
to 4.1 m/s forward against the 2.3 to 2.8 m/s the lidar profile averages on
this location, and a vertical half-angle of 52 degrees. These are estimates;
stage 1 measures the matcher's real disparity error, the evidence age of the
vision path and what two 1280 px renders cost the simulator, and fixes the
geometry among these rows.

The vertical half-angle matters because of what no sensor covers. The pair
sees up to its vertical half-angle above and below the horizon and each
time-of-flight sensor 22.5 degrees about the vertical, so a velocity whose
elevation lies between the two is observed by nothing. Over the flights r470
to r474 the elevation of the velocity is 4 degrees at the median, 22 at p90
and 39 at p95; the uncovered band holds 3.0 percent of the flown time with a
90 degree lens (37 to 67.5 degrees) and 1.4 percent with a 120 degree lens
(52 to 67.5), at 1.6 to 1.8 m/s. A motion in that band is unobserved motion
and flies at the speed unobserved motion is allowed, like any other: no
route, cost or latch keeps the vehicle out of it. At 1.4 percent of the
flight the price is below a second.

One alternative stays on record. A one-axis tilt servo under the pair (the
pair faces the motion in the vertical plane as the vehicle's yaw faces it in
the horizontal one) keeps the vertical purely visual at the price of a moving
extrinsic, one degree of servo error being 17 cm at 10 m; it is the choice if
the time-of-flight exception is ever withdrawn. The directional detection
range below is written for any set of frustums, so it changes nothing above
the sensor boundary. Three fixed pairs are not an alternative: at the price
of the lidar they lose to it in range, in the dark and in compute, nobody
would mount them, and the reference this profile is measured against is the
lidar profile itself.

Environments used for acceptance must carry surface texture, on shaft walls
and floors as much as on facades. A stereo matcher recovers depth from
texture; an untextured flat wall is exactly where it fails, and a wall that
yields no depth is unobserved, not absent. Environment candidates that
render as uniform flat colour are textured before they are used for this
stage; the geometry, spawn points and mission goals do not change.

### Depth Recovery

The primary track is classical calibrated stereo: rectification, a dense
disparity matcher such as semi-global matching, left-right consistency and
texture checks, and metric depth from the known baseline. It is deterministic,
metric without a learnt scale, runs on the CPU or the GPU the controller
already uses, and its failure modes are known and observable. Two tracks may
follow it, ordered by what they add:

1. multi-view depth from the vehicle's own motion, using the timestamped pose
   contract for the baseline, to densify depth where the stereo baseline is
   too short for the range;
2. learnt monocular depth as a prior for regions the stereo matcher rejects,
   with its metric scale anchored by stereo and never used alone.

Depth is a measurement with a range-dependent error: for a baseline `b`, focal
length `f` and disparity error `e`, the depth error at range `z` is about
`z² · e / (b · f)`. Every pixel therefore carries a confidence and a range
beyond which it is not evidence. Only a confident depth becomes a hit, and only
up to its confident range; a pixel without depth contributes nothing, not a
miss. This is the one semantic difference from the lidar, whose maximum-range
miss is real evidence: a vision miss exists only along a ray that ended in a
confident hit. Semantic understanding of what the shapes are — doors, glass,
vegetation, vehicles — is a separate stage; this stage recovers geometry only.

### What Changes Above The Sensor Boundary

Unknown space stays traversable without penalty; nothing in this stage may
add a prohibition, a penalty or a latch on space the camera has not seen. What
protects the vehicle in unobserved space is the sensor braking law the speed
policy already applies: the vehicle never moves faster than it can stop within
the range at which it is guaranteed to detect an obstacle. The lidar profile
states that range as one omnidirectional number. A camera sees a cone. The
guaranteed detection range becomes a function of direction relative to the
sensors' frustums (the forward pair's and the two time-of-flight cones) and
of each one's confident range: the range of the frustum that contains the
direction, and nothing where no frustum does. The speed policy limits speed
along the commanded motion by the guaranteed range in that direction. A
vehicle commanded sideways or backwards, out of every frustum, slows to what
unobserved motion allows, which is the existing law applied honestly rather
than a new rule; a climb or a descent is observed by the upward or the
downward time-of-flight sensor and flies the speed its range admits.

That makes heading a perception decision. The execution layer gains a gaze
policy that yaws the vehicle so the forward pair faces the horizontal
component of the commanded motion before the motion exceeds what unobserved
space allows, so that ordinary forward flight is observed flight; there is no
pitch or roll to command, the time-of-flight sensors cover the vertical.
Active choice of viewpoint for its own sake — moving to see into a shaft
before committing to it — is a later stage; here the camera only follows the
motion. The latest-lidar evidence that item 8 admitted for bounded final
execution revalidation became latest sensor evidence from whichever sensor
produced it; the admission rule, the freshness bound and the swept validation
did not change. That is a change of names, not of rules, and it is the one
place where "runs unchanged" above is not literal: the
`LatestLidarObstacleScan` message, the `latest_lidar_*` parameters and
diagnostics, the `VersionedLatestLidarEvidence3D` owner and the
`kLatestLidarRawCollision` verdict named the sensor and are now
`LatestSensorObstacleScan`, `latest_sensor_*`,
`VersionedLatestSensorEvidence3D` and `kLatestSensorRawCollision`.

Localization is not part of this stage. Stages 1 to 3 fly with the lidar
still mounted and keep the default lidar-inertial profile; stage 4, with the
lidar removed, flies on the `gnss` profile, and item 13's rule holds in
reverse: visual-inertial odometry, which item 16 adds, is a separate
estimator that this stage must not depend on and must not be depended on by.
The roadmap dependency between the two must not become a code dependency.

### Implementation Order

1. Add the forward stereo pair and the two time-of-flight sensors to the
   vehicle model and bridge them; record timestamped stereo frames, ranges
   and poses from lidar missions, and evaluate recovered depth offline
   against evaluation-only simulator depth by range, texture and view angle
   for the geometries of the table above (90 and 120 degree lenses, 0.2
   and 0.3 m baselines at 1280 px) to fix the confident range model and
   the lens; calibrate the offset between a frame's exposure stamp and the
   pose the way the lidar's was (its pose led the scan by 160 ms until
   `lidar_pose_latency_s` was measured, and the vehicle met walls for it);
   measure what the matcher costs on the GPU the controller shares
   (41 percent on the lidar profile) and what two renders cost the simulator,
   which holds a real-time factor of 1.00 without margin.
2. Add the stereo depth producer and the depth-to-beam adapter that emits the
   item 8 beam observations with per-ray confidence, and the time-of-flight
   adapter that emits its zones as the same beams, and integrate them in
   shadow: lidar remains authoritative, and the vision occupancy is compared
   with lidar occupancy and truth occupancy for occupied precision and recall,
   unknown fraction and latency.
3. Make the guaranteed detection range directional and the gaze policy part of
   execution; validate with lidar still integrated that speed and heading
   behave as the observability model says.
4. Fly Urban Circuit Practice 01 on the stereo profile alone, with the lidar
   removed from the model, against item 12's gates.

Stages 1 and 2 (2026-09-19). Stage 1: the sensor set mounts with
`CAMERA_PROFILE=stereo_tof`; two 1280 x 960 cameras cost the simulator nothing
as RGB (real-time factor 0.99) and 0.39 as `L8`; the headless world carries no
textures, so camera flights run on the GUI world; semi-global matching
recovers depth on 98 to 100 percent of the pixels with a disparity error of
0.11 to 0.17 px at the median and 0.29 to 0.49 px at p90, which puts the
confident depth of the working geometry at 6.4 m, not the 8.6 m estimated
above from a quarter pixel; a frame's stamp is its render time within one
frame. Stage 2: the vision returns, their free rays and the time-of-flight
zones feed a shadow obstacle memory whose occupied voxels agree with the
lidar memory's to 97 to 98 percent within one voxel and find 96 to 99 percent
of the lidar's inside the volume they observe, a third of the lidar's
([obstacle_mapping.md](obstacle_mapping.md)). Two findings bind the stages
that follow: beside the GUI-world render, the image bridge and a 110 ms
matcher on two threads the workstation no longer carries the lidar-inertial
estimator in real time, so camera flights use the `gnss` profile already; and
with 6.4 m of confident depth the braking contract admits about 2.5 m/s
forward at the lidar's evidence age and 3.0 m/s at 0.25 s.

### Measurement And Completion

Measure depth coverage and depth error against evaluation-only truth by range
and view angle, occupied precision and recall of the vision raw world against
truth occupancy, the fraction of the flown route that was observed before it
was entered, time spent speed-limited by observability, perception latency
from exposure to raw-world revision, planner p95, route availability, minimum
obstacle clearance and physical collisions.

This stage is complete when a series of five consecutive Urban Circuit
Practice 01 missions on one commit runs with the lidar absent from the vehicle model, no depth or point cloud sensor
in the control path beyond the two time-of-flight sensors, the raw-world and
planner contracts unchanged, and the mission gates of the 3D-lidar profile:
mission complete, collision-free, route availability at the floor named
above and planner p95 below 200 ms. Speed is not gated at the lidar
profile's 2.5 m/s: the mission check on the stereo profile reports the mean
speed and gates it at half of what the braking contract admits forward for
the geometry stage 1 fixes, a rule written before stage 4 flies and not moved
after it. The pre-implementation estimate of that bound was 1.75 m/s, from
8.6 m of expected depth; the 6.4 m stage 2 measured admit 2.452 m/s under the
contract, so the gate is 1.226 m/s. The routes through both shafts
are flown without crawling: the vertical speed held in a shaft is at least
half of what the time-of-flight range admits there.
