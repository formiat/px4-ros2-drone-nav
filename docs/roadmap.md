# Gazebo Roadmap

## Dependency Model

Roadmap numbering identifies project milestones; it is not always a strict
execution order. The dependency annotations below use three meanings:

- **hard prerequisite**: implementation cannot begin meaningfully before the
  prerequisite contract exists;
- **validation prerequisite**: the feature can be developed independently, but
  its complete mission-level acceptance requires the prerequisite;
- **independent recurring workstream**: work may run in parallel with any
  milestone and should be repeated as the architecture evolves.

Numbers are stable identifiers. A completed item keeps its number, is
summarized in the Completed section at the end of this file, and is never
renumbered or reused; a new item takes the next free number.

## 9. Large-Scale Realistic City And Full-Mission Validation

**Type:** integration and validation milestone in two stages.

**Hard prerequisites:** item 12 (complete) for stage A; a suitably licensed
city for stage B.

### Stage A: Point-To-Point On Urban Circuit Practice 01

Fly the point-to-point mission on Urban Circuit Practice 01 with the 3D lidar
and no static map, at one release commit, headless, with nothing changed
between runs, as a series of five flights. Stage A needs no new environment
and may begin immediately.

Acceptance uses the numbers the mission check already enforces, as measured in
the v0.2.0 series (r288 to r292) and the r303 to r307 series on the
descent-arrest commit: zero building collisions, zero execution-ownership gaps,
persistent planner p95 below 200 ms, mean flight speed of at least 2.5 m/s on
the urban point-to-point mission, and the controller-dynamics checks described
in `testing.md`. Route availability after bootstrap is measured, not yet gated:
item 12 closed at 88 to 95 percent against the 97 percent mission check and the
99 percent target written before any measurement, with physical blocks at
surfaces as the remaining cause. This stage re-derives the availability
threshold from the measured runs and records it in the mission check before
stage B begins.

### Stage B: Large-Scale Realistic City

Find a suitably licensed high-quality city environment or build a new one for
the project. The location should be substantially larger and more visually and
geometrically varied than the current locations, with realistic street layouts,
building shapes, heights, materials, and urban topology.

Where practical, include complex physically traversable 3D free-space
structures such as multi-turn tunnels, junctions, shafts, and entrances at
different altitudes. Imported visual assets must have explicit provenance and a
license compatible with the repository. Rendering meshes, collision geometry,
lidar-visible surfaces, static occupancy, and generated planning artifacts must
remain aligned instead of becoming separate hand-maintained versions of the
world.

Use the new location as a full-system validation environment rather than only a
visual showcase. Re-run stage A on it, covering multiple start and goal
placements and repeated headless runs, and preserve physical
outcome checks, zero tolerance for building collisions, planner and controller
diagnostics, real-time-factor monitoring, and measured CPU/GPU timing.

Stage B is complete only when the point-to-point mission succeeds on the new
city without scenario-specific route scripts or geometry exceptions, at the
stage A thresholds. One successful 3D-lidar exploration flight is integration
evidence, not completion.

## 11. Valid 3D Static Maps For New Environments

**Type:** dependent implementation and validation stage.

**Hard prerequisites:** items 8 and 12 for the primary autonomous-survey
acquisition path.

Create a valid static map for every new complex environment. Here, quality
means geometrically correct, physically valid, and aligned with the real
collision environment: every real obstacle relevant to the aircraft footprint
must be represented. It does not require unnecessarily high visual or voxel
detail.

Every new-environment static map must be three-dimensional. Two-dimensional
maps are insufficient for multi-level geometry, tunnels, shafts, windows,
doors, and other traversable 3D passages.

The primary acquisition path uses item 8's production 3D lidar and item 12's
incremental exploration backend to survey every reachable part of an
environment, then persists the resulting validated obstacle memory as the
environment's static-map artifact. Direct generation from collision geometry
may remain as a secondary generation or cross-validation tool. Every artifact
must be versioned with the environment collision geometry, source provenance,
coordinate transform, resolution, coverage evidence, and validation result.

This stage is complete when every supported new environment has a reproducible
3D static-map generation or acquisition path and that map passes coverage,
alignment, and raw-collision validation against its physical world.

## 14. Camera-Based 3D Perception Without Lidar Or Static Maps

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
motion. The latest-lidar evidence that item 8 admits for bounded final
execution revalidation becomes latest raw evidence from whichever sensor
produced it; the admission rule, the freshness bound and the swept validation
do not change. That is a change of names, not of rules, and it is the one
place where "runs unchanged" above is not literal: the `LatestLidarObstacleScan`
message, the `latest_lidar_*` parameters and diagnostics, the
`VersionedLatestLidarEvidence3D` owner and the `kLatestLidarRawCollision`
verdict name the sensor and are renamed for the evidence.

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

Stages 1 and 2 are done (2026-09-19). Stage 1: the sensor set mounts with
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
the geometry stage 1 fixes (1.75 m/s for the working choice), a bound written
before stage 4 flies and not moved after it. The routes through both shafts
are flown without crawling: the vertical speed held in a shaft is at least
half of what the time-of-flight range admits there.

## 15. Realistic Cooperative Communication

**Type:** dependent realism stage.

**Hard prerequisites:** items 6 and 13.

**Validation environment:** Urban Circuit Practice 01, the cooperative
traffic scenario.

The cooperative traffic of item 6 works, but its exchange is not one a
real fleet has. Four vehicles publish a `CooperativeFlightIntent` twenty
times a second into one shared DDS topic on one host: 1 to 2 KB per
message, 20 to 40 KB/s per vehicle, every vehicle receiving every message
with no latency, no loss, no range and no partition. That is a test bench
for the separation algorithm, not a model of the air. This item replaces the
channel with one that behaves like radio, cuts the exchange to what such a
channel carries, and makes the separation survive what the channel does. The
navigation invariants hold throughout: a peer the vehicle does not hear of
is unknown, not an obstacle and not a prohibition; nothing here adds a
latch, a penalty on free space or a restriction of motion.

### Stage 0: Remove The Interception Missions And The Radar (Done)

The interception missions (items 1 to 5 and 6.1) and the airborne radar
they rest on leave the repository first. The radar is the one sensor of the
stack modelled from Gazebo truth with no physical counterpart on this class
of vehicle: an ideal sphere of range, bearing, elevation and radial velocity
to 100 m, where a real radar an x500 could lift is a sector with degrees of
angular error, multipath in streets and a detection range on a 0.01 m²
target that starts at a few hundred metres. The cooperative traffic does not
use it. What goes: the radar simulators, trackers, guidance, the evader and
interceptor referees and truth boundaries, the intercept launches,
scenarios, scripts, messages, tests and documentation, and the
non-cooperative avoidance parameters of the controller; 56 files and about
8 700 non-blank lines by name, plus the interception branches woven into the
multi-vehicle launch (55 references), the Makefile (19), the simulation
script (14), README (57) and `architecture.md` (43). What stays: the
multi-vehicle launch and spectator infrastructure, the cooperative agents
and referee, and the `Completed` entries 1 to 5 and 6.1 as history, each
with the line that this stage removed the feature. Done on 2026-09-17, together with the grid-city world the interception
missions flew in: the world specification, its generated SDF, occupancy, ESDF
and topology artifacts, the world generator, the two grid-city scenarios and
the grid-city make targets and wrappers went with them. Only the imported Urban
Circuit environment remains, with its point-to-point and cooperative traffic
missions. Unit and script tests were deleted rather than skipped, and no
document names a removed node, scenario or command.

Confirmed in flight by a single-vehicle series on the lidar-inertial profile,
r448 to r452 on one commit: five urban point-to-point flights of five without
a crash, 2.58 to 2.94 m/s, zero ownership gaps, planner search 152 to 156 ms
at p95. The removal itself lost four general tick diagnostics, which were
restored; the other defects the series met and repaired were older than the
removal. The cooperative traffic mission is not flight-verified after the
removal and is not flown until this item's later stages land.

### Stage 1: A Link Model Between Vehicles

A link simulator is a simulation component like the lidar: it may read the
simulator's true vehicle positions and the world to decide what radio does,
and it hands each vehicle only the messages that arrive. Every message a
vehicle sends enters the link simulator on the vehicle's own output topic
and leaves on the receiving vehicle's input topic; a contract test holds
that no agent subscribes to another vehicle's output directly, the way the
cooperative referee's ground-truth boundary holds its graph. Three channel classes, each a parameter set
of the same component, chosen per mission:

- **mesh** (Wi-Fi 802.11s or batman-adv class): a pair is linked in line
  of sight within about 150 m and behind a building within tens of metres,
  line of sight read from the world; a message reaches a vehicle out of
  direct range through peers that hear both, each hop adding 5 to 20 ms;
  loss rises with range and the mesh partitions when the vehicles spread;
- **cellular** (LTE class over a SIM): every vehicle reaches every other
  through a relay with 50 to 150 ms of latency and jitter of the same
  order, no direct links, and coverage that ends where the location goes
  indoors, in the shafts and under the covered passages;
- **telemetry radio** (900 MHz class): range over the whole location, a
  shared budget of about 100 kbit/s, latency tens of milliseconds, so the
  rate of the fleet's messages is what the budget allows.

Line of sight is one bit per pair that switches the path-loss exponent of
a log-distance model: about 2 in the open, 3.5 to 4 behind a building,
which is what leaves tens of metres of range where there is no sight line.
Received power against the receiver's sensitivity decides whether the pair
is linked, and the margin over it the loss probability. The bit comes from
the segment between the two antennas, at the vehicles' true poses, tested
against the world's geometry. Two sources exist for that test, and the
first is the one to build:

1. a world system plugin that casts the segment through the physics
   engine's collision meshes (`GetRayIntersection` of gz-physics 7, which
   the dartsim plugin of the container implements, with gz-sim 8's
   `RaycastData` component) and publishes the pair matrix on a Gazebo topic
   the bridge carries to the link simulator: exact geometry, no map, six
   pairs at 10 Hz for four vehicles; it is checked on Urban Circuit Practice
   01 against pairs known to stand inside and outside the same structure;
2. a sampled walk along the segment through an
   evaluation-only voxel occupancy of the world built offline from the
   SDF collisions (`voxelize_sdf_collisions`), kept as the fallback because
   the urban location has no valid such map until item 11 delivers one.

The same plugin casts one ray straight up from each vehicle: a ray that
hits a ceiling puts the vehicle indoors, in a shaft or under a covered
passage, where the cellular class has no coverage. None of this reaches an
agent: a vehicle does not learn why it cannot hear a peer, only that it
cannot, and the geometry lives in the link simulator alone.

Real fleets combine a cellular link for command with a local broadcast for
deconfliction; a mission may run two classes at once, each carrying what it
is for. The parameters of each class are stated with their source, the
channel's delivery latency, loss and partition are logged per message, and
the mission check reports them per flight.

### Stage 2: An Intent That Fits The Channel

The intent shrinks to what the channels carry: position, velocity, the
footprint, the maneuver state and a coarse predicted trajectory over the
5 s the conflict prediction already uses, within about 200 bytes, sent at 1
to 5 Hz, faster only while a conflict is predicted. The bounded validity and
the predicted trajectory that item 6 already sends are what make the low
rate sufficient: a peer is extrapolated along its last intent until the
intent expires, and an expired intent is no knowledge at all. The bandwidth
the fleet uses is measured per class against the class's budget.

### Stage 3: Separation That Survives The Channel

The separation cost and the maneuver selection are given the channel's
failures as ordinary input: a peer whose intent is late, lost, expired or
never heard; an asymmetric link where one vehicle hears and the other does
not; a partition that hides half the fleet; a message that arrives after
the vehicle it describes has moved. Complementary maneuver choice must stay
deterministic under asymmetric knowledge. The referee's separation gates of
item 6 are held under each channel class and under scripted outages of the
link simulator (an evaluation component; no fault injection enters
production code).

### Stage 4: A Shared Frame Without GNSS

The cooperative missions still fly the `gnss` profile, and the exchange of
positions works because GNSS gives every vehicle one frame. On the default
lidar-inertial profile each vehicle has its own frame with its own drift,
and a peer's "I am at X" means nothing without a common anchor. The options
are measured against each other on the cooperative scenario: a GNSS anchor
where the sky is open, relative observation of peers by each vehicle's own
lidar (a vehicle at 10 to 30 m is a return the obstacle memory already
sees), and alignment of frames through the shared world. This stage is
complete when the cooperative acceptance series flies on the lidar-inertial
profile with the multi-vehicle launch running one estimator per vehicle.

### Measurement And Completion

Measure, per flight and per channel class: message rate and bytes per
vehicle, delivery latency and loss, the share of flight time each vehicle
spends with each peer unknown, minimum separation and its margin over the
gate, maneuver decisions taken under asymmetric knowledge, and the frame
error between vehicles on the lidar-inertial profile. This item is complete
when stage 0 has landed, the cooperative acceptance series passes under the
mesh and cellular classes with the intent within the channel's budget, the
referee's separation gates hold under scripted outages, and the series
flies on the lidar-inertial profile.

## 16. Flight Without GNSS And Without Lidar

**Type:** dependent localization stage.

**Hard prerequisites:** items 13 and 14.

**Validation environment:** Urban Circuit Practice 01.

Item 13 took away GNSS and the magnetometer while the lidar carried the
position. Item 14 takes away the lidar while GNSS carries the position. Each
removed one thing so that a failed flight had one possible cause. This item
removes both at once: the vehicle carries the forward stereo pair and the two
time-of-flight sensors of item 14 and the IMU, and nothing else. Position and
heading come from a visual-inertial estimator on the `visual_inertial`
profile, and the world is the vision raw world of item 14. Nothing below the
estimator changes: the autopilot's EKF2 remains the owner of the estimate, the
estimator publishes `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`
exactly as `lidar_inertial_odometry_node` does, and the map frame is still
fixed by the scenario's start pose ([`localization.md`](localization.md)).

The estimator is a separate component from the perception of item 14. Both
read the same images, and neither reads the other's output: an estimator that
takes its pose from the map built with that pose closes a loop that hides its
own drift. The estimator keeps its own landmarks and its own window, as
`LidarInertialOdometry` keeps its own submap and depends on Eigen alone
(`test_navigation_dependency_contract.py` holds that boundary); it must not
consume the obstacle memory's occupancy, the planner's route or the
controller's state, and nothing in perception, planning or control may depend
on its internals. The roadmap dependency between items 14 and 16 must not
become a code dependency.

### What The Simulator Provides

The calibrated forward stereo pair of item 14, its intrinsics and baseline,
the two time-of-flight sensors, and the IMU. No GNSS in the control path, no
magnetometer fusion, no `simulation_heading_source_node`; the simulator's true
pose and its depth stay where item 13 and item 14 put them, in evaluation and
referee components only. The environment must carry the surface texture item
14 already requires: a feature tracker fails on a flat untextured wall for the
same reason a stereo matcher does.

### The Estimator

Keyframe visual-inertial odometry, built on the contracts item 13 established:

- features tracked across the rectified images of a pair and triangulated
  against the known baseline, so the scale is metric and measured, never
  learnt;
- the IMU preintegrated between frames as the motion prior, with the
  gyroscope bias as a state and the level from the accelerometer at rest, as
  item 13 integrates it to a scan stamp;
- the pose from minimizing reprojection error over a sliding window of
  keyframes with a robust cost, the window marginalized rather than grown;
- item 13's honesty rule in its own form: a direction observed by too few
  landmarks carries no measurement, a landmark whose reprojection lies
  outside the gate is dropped for that frame, and a frame that did not
  converge is not published at all, so the autopilot sees no estimate rather
  than a wrong one.

The forward pair feeds the estimator. Its weak place is measured, not
assumed: during the vertical motion through the two shafts a forward tracker
watches a wall slide past at under a metre, fast in the image and poor in
texture, and the time-of-flight ranges are the only other exteroception the
vehicle has there. Stage 1 measures the drift through the shafts and whether
those ranges are needed as a vertical constraint.

A visual-inertial estimate drifts without a closure, and the mission is
hundreds of metres long. The threshold is not chosen for the estimator, it is
the one the stack already needs: the persistent memory integrates a beam only
as far as the pose error keeps its hit within two voxels of the surface, which
at 0.25 m voxels bounds the heading error to about 2 degrees
(`lidar_pose_heading_uncertainty_rad`,
[`gazebo_simulation.md`](gazebo_simulation.md)), and a map built with a worse
pose smears its walls and closes the passages the
planner is trying to use. Stage 1 measures drift against that bound and
decides what it costs to hold: a closure against the estimator's own
keyframes, or a mission short enough that pure odometry stays inside it.

The health and failure contour of item 13 applies unchanged and gains no new
latch: without odometry for 200 ms the autopilot's external-vision fusion
stops, after `EKF2_NOAID_TOUT` it withdraws the horizontal estimate, the local
position arrives with `xy_valid` false, the controller revokes execution on
the stale pose and the offboard node holds.

### Implementation Order

1. Offline first, as item 13 set its filter: replay recorded stereo frames,
   IMU and true pose from item 14's flights (`log/tools/replay`), and measure
   drift per 100 m, along-track and cross-track error and heading error
   against the truth, by texture, speed, lighting and lens. Flights
   are stochastic; estimator parameters are set on recordings and only
   confirmed in flight.
2. Add the `visual_inertial_shadow` profile, as `gnss_shadow` did for item 13:
   the estimator runs beside GNSS, publishes to the diagnostic topic only, and
   the mission check compares it with the true pose on every flight.
3. Fly on `visual_inertial` with the lidar still mounted and still
   authoritative for perception, so a failure separates the estimator from the
   vision perception.
4. Fly with the lidar absent from the vehicle model and item 14's vision raw
   world: no GNSS, no magnetometer, no lidar.

Multi-vehicle missions stay out of this item; the shared frame between
vehicles without GNSS is item 15 stage 4.

### Measurement And Completion

The mission check reports, as it does for item 13: the autopilot's position
estimate against the true pose (cross-track p95 and along-track offset), the
estimator's own published estimate against the true pose, drift per 100 m of
flown route, the profile proved from the autopilot and ROS logs
(`EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE 5`, `EKF2_EV_CTRL 11`, `EKF2_HGT_REF 3`, no
heading source, odometry published at rate over the flight), and estimator
health: tracked landmark count, reprojection residual, the observability of
the weakest direction, gated and degenerate directions, and the frames that
left the autopilot without an estimate, which must be none in a clean flight.

This item is complete when repeated Urban Circuit Practice 01 missions run
with no GNSS, no magnetometer and no lidar in the vehicle model, the estimate
holds the pose error the mapping contract above requires, and the mission
gates of item 12 hold: mission complete, collision-free, route availability at
the threshold item 9 stage A derives, planner p95 below 200 ms, and the routes
through both shafts flown at the speed the lidar profile flies them.

## Completed

Each entry keeps its original number. The release that shipped it is linked;
the detailed contracts live in the code, its tests, `CHANGELOG.md`, and the
documents named below.

### 1. Interceptor Drone (Completed, Removed)

Shipped before the first tag; see the `v0.1.0` entry in `CHANGELOG.md`. Three
interceptors pursue one attacking drone in isolated PX4 and ROS namespaces,
each from an independent radar-derived target track with predictive guidance
and no terminal goal hold. A separation of 5 m or less destroys the capturing
pair and records the intercept outcome.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 2. Radar Measurement Simulation (Completed, Removed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). The interceptor
sees only range, bearing, elevation, and radial velocity from an ideal radar
with a correlated random-walk cadence between 0.1 s and 3.0 s; swept raw-clear
visibility commands 20 Hz track mode. Truth adapters, referees, radar
simulators, trackers, and guidance are separate nodes, and contract tests keep
absolute target position out of the interceptor-facing `RadarScan`.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 3. Target Motion Prediction (Completed, Removed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Guidance solves
the constant-velocity intercept from the latest track, capped at 15 s and at
1 s inside the target corridor, with spatial hysteresis and a smoothed horizon.
The planner clips the prediction at the first raw occupied cell without
inflation or prohibited regions; swept visibility of the target switches to
direct moving-target MPPI pursuit.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 4. Multiple Interceptors Versus One Attacker (Completed, Removed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Three interceptors
from three city corners own independent PX4, navigation, radar, tracker, and
guidance pipelines; optional directional hypotheses converge to zero near the
attacker. The first interceptor within 5 m destroys the pair, survivors enter a
typed stationary hold, and interceptor-to-interceptor proximity is collateral.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 5. Multiple Interceptors Versus Multiple Attackers (Completed, Removed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). The generic N x M
launch pipeline runs the 2x2 scenario through the `sim_multi_intercept_*.sh`
wrappers. Each interceptor keeps one radar-derived track per detection, a typed
assignment coordinator minimizes estimated intercept time with hold, threshold,
and confirmation hysteresis, and the referee records one terminal outcome per
attacker. Attackers are not respawned.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 6. Cooperative Multi-Drone Air Traffic (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Four civilian
drones exchange typed bounded-validity flight intents at 20 Hz and select
deterministic complementary vertical or lateral maneuvers from predicted
closest approach. Separation is a strong soft MPPI cost, never a prohibited
grid or inflated obstacle; static passages expose raw-validated lane capacity
with deterministic right-of-way. Static and no-static scenarios passed the
referee.

### 6.1. Non-Cooperative Collision Avoidance In Interception Missions (Completed, Removed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Every attacker
carries an anonymous airborne radar and a variable-time tracker; a finite
trajectory cost below 10 m with anticipation to 20 m and a raw-validated
maximin acquisition drive avoidance. Raw occupancy remains stronger than
separation and no exclusion volume exists, so physical interception stays
possible. The 3x1 and 2x2 scenarios passed with and without a static map.

Removed from the repository by item 15 stage 0 on 2026-09-17; this entry is history.

### 7. Advanced 3D Passages (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). A chunked compiler
turns raw `Occupancy3D` and the matching `ESDF3D` into a map-fingerprint-bound
`FreeSpaceTopology3D` of portal patches and medial passage segments with strong
IDs. Planning resolves `PassageTraversal` objects lazily over the sparse graph
and derives a varying 3D cross-section envelope from raw occupancy; MPPI and
route activation keep final raw swept-footprint validation. Strict artifacts
exist for the compact fixture and the Urban, Cave, and Finals maps.

### 8. 3D Perception And Raw-World Foundation (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Organized Gazebo
3D lidar beams are timestamp-aligned, resolved to a full-6DoF acquisition pose,
and ray-integrated into revisioned `unknown/free/occupied` `Occupancy3D` with
dirty-chunk transport and chunked immutable snapshots. Only confirmed
`Occupied` is a hard prohibition; relabeling `Free` and `Unknown` changes
nothing. No-static production navigation uses the 3D lidar profile; there is no
2D-lidar production fallback.

### 10. Architectural Review And Optimization (Completed)

Closed on 2026-09-16 on the urban point-to-point mission with the 3D lidar
and no static map; the measurements are in
[`resource_budget.md`](resource_budget.md) and [`performance.md`](performance.md),
the gates in [`testing.md`](testing.md). Every flight records what its
processes consume and what each transport hop's delivery took, and the
mission check bounds the tick (30 ms p50, 45 p95), the record's coverage,
the onboard processes' memory growth and the memory hop's delivery. Measured
at the r352 to r356 series: the onboard processes use 3.0 to 3.4 cores at
p50 and 0.75 to 0.83 GiB, the GPU 39 to 41 percent of an RTX 3060 Laptop,
DDS delivers every hop in 0.06 to 1.3 ms at p50, and the obstacle memory
transports at the scan rate so the observation age is 184 to 200 ms at p50
against the 600 ms the braking contract charges, from 404 at 2 Hz. Along the
way the tick went from 55 to 23 ms at p50 (one collision oracle per path),
the position estimate from 0.11 s ahead of the true pose to 0.01 s
(`EKF2_GPS_DELAY 0`) and the vertical law from 2.0 to the measured 1.4 m/s².

Known leftovers, measured and not gated: the loop runs near 43 Hz with about
80 percent of ticks over the 20 ms deadline and no single bottleneck left;
the planner spends its whole 150 ms budget, so the p95 check measures the
configuration; a holding vehicle drifts 0.37 m at p95 while the rest
clearance rule keeps 0.27 m; the ordinary no-route holds sit at 3 to 13
percent against the 3 percent check and route availability at 92 to 96
against 97; the 2D obstacle memory node is still selectable by the launch
files, fourteen sources sit near the 1000-line cap and 226 lie flat in
`src/`.

### 12. Persistent Full-3D Strategic Navigation (Completed)

Shipped in [v0.2.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.2.0)
and [v0.2.1](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.2.1)
on 2026-09-14; the laws in force and the validated series are in
`CHANGELOG.md`, the contract checklist in
[`navigation_architecture_remediation.md`](navigation_architecture_remediation.md).

One navigation architecture for static and no-static maps with no
environment-specific planner mode and no location knowledge. The planner
consumes a global sparse world-fixed `RawOccupancy3D`; confirmed `Occupied`
cells and the physical hull are its only hard constraints, `Free` and
`Unknown` are identical in traversability and cost, and the exact capped
`KnownObstacleDistance3D` supplies soft clearance evidence only. Unknown-space
safety is one sensor-and-braking inequality, speed times latency plus stopping
distance plus margin within the guaranteed lidar range, enforced as a hard
translational speed bound in host and CUDA dynamics and in the route ETA model.
One persistent sparse D* Lite planner over an adaptive 26-connected lattice
retains search state across a moving start and raw updates, publishes an
incumbent independently of convergence, and refines predicted 3D execution
time with the shared station/time model. One `ActiveIntent3D` owns the mission
route. `ExecutionSupervisor3D` owns the sole `RouteExecutionManager3D`, the
tagged execution variant with its pure reducer, and the atomic
`CommittedExecutionAuthority3D` with a certified braking fallback. The ROS
component is a composition adapter over independent world, route, and MPPI
runtime targets, and domain transactions are tested through executable APIs.

Accepted at closure: the raw world and distance evidence, the persistent
planner, the route owner and execution plan contracts, the contact law, the
honest vertical dynamics, the body-clearance bound on the progress floor,
immediate blocked-route replacement, the autopilot contract with `px4_msgs`
confined to the PX4 adapter, and the controller-dynamics mission checks.
Closure evidence: five urban no-static point-to-point flights on `9ab94040`
(r288 to r292) and five on `46823cac` (r303 to r307), no collisions, no
execution-ownership gaps, planner p95 between 153 and 163 ms, mean flight
speed between 2.72 and 3.19 m/s.

Not repeated at closure and carried into item 9 stage A: the three-run
grid-city gate (that world has since been removed), the 97 and 99 percent
availability targets (measured 88 to 95 percent after bootstrap), and the
cooperative re-flights. The technical debt measured during closure is listed in
item 10.

### 13. GNSS- And Magnetometer-Denied Lidar-Inertial Navigation (Completed)

Shipped in [v0.3.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.3.0)
on 2026-09-17, closed on the urban point-to-point mission with the 3D lidar
and no static map; the profile, the estimator and its health are in
[`localization.md`](localization.md), the checks in [`testing.md`](testing.md).
`LOCALIZATION_PROFILE=lidar_inertial`, the default of every single-vehicle
flight since, flies on the IMU and a lidar-inertial estimator alone, through the autopilot's external-odometry interface, with
GNSS, magnetometer and simulation-heading fusion off (`EKF2_GPS_CTRL 0`,
`EKF2_MAG_TYPE 5`, `EKF2_EV_CTRL 11`, `EKF2_HGT_REF 3`); the mission check
proves the profile from the logs and reports the estimator's health, and a
scan that does not register is not published, so a lost estimate reaches the
stack as the autopilot's withdrawn position and the existing pose-age
revocation. Measured at the r430 to r434 series on 5a113bc5: no crash,
2.57/2.97/2.58/2.78/2.80 m/s, the autopilot's estimate 0.15, 0.18, 0.22, 0.19 and 0.18 m from the true pose across
the track at p95 (GNSS baseline 0.19 to 0.25), the estimator's own 0.06 to
0.17 m, tick 24.1 to 25.0 ms at p50, the estimator at 0.5 cores and 59 MiB;
the profile, health, dynamics, resource and transport checks green on every
flight, the known reds of the stack (no-route holds 3.9 to 9.2 percent,
route availability 90 to 96) as before, and r433 without the route-volume
witness, which reads the controller's sampled positions and not the estimate.

Deferred with the measured reason: loop closure, because the drift does not
grow with the flight's length over the 400 m mission (0.1 to 0.2 m at the
goal, accrued along one bare corridor, not with the distance). Known and not
gated: the corridor at x 30 to 62 leaves its axis to the IMU, where the
estimate drifts 0.08 to 0.22 m; the along-track check reads at its own
resolution (-0.029 to +0.013 s over r415 to r434 against 0.02); and the flights that lost
the track while the estimator was set (r386, r387, r399 to r401, r411 to
r413) were each an ordering or timing fault of the scan and IMU streams,
not of the registration.
