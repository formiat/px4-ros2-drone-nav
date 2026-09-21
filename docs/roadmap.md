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

Order and sensor profile. Items 15 and 16 did not depend on each other, and
item 16 was done first (closed 2026-09-20). That had a reason: stage 4 below
was written for the lidar-inertial profile, and the default sensor set since
item 14 is the stereo pair, so a shared frame is better solved once, for the
estimator item 16 added, than twice. The cost of that order is that the cooperative
mission stays unflown for longer: it has not been flown since stage 0 and
not at all on the camera defaults the multi-vehicle launch now carries.

Which sensor set this item flies is an open decision, because the
workstation does not carry four camera vehicles. Measured on single flights:
a lidar vehicle's onboard processes take 2.7 cores at p50 and a stereo
vehicle's 3.8, of which 1.4 are the depth matcher on the CPU (OpenCV in the
container has no CUDA); the simulator with one vehicle's two 1280 x 960
cameras on the textured world takes 3.3 cores and drops its real-time factor
to 0.35 to 0.45 for moments, which is what makes the autopilot reacquire its
timestamps 10 to 15 times a flight (r518). Four vehicles are about 15 onboard
cores and eight cameras on eight host cores; the lidar profile runs four
vehicles on the collision-only world at a real-time factor of 1.00, as item
6 was accepted. The exchange and the separation cost the same on either. The
options: fly this item on the lidar profile, whose subject is the channel and
not perception, as an exception to the camera default; move the matcher to
the GPU; lower the cameras' resolution or rate, which shortens the confident
depth and the speed with it; or a larger host. The extrapolation from one
vehicle to four is not measured.

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

The direction to measure first: vehicles share a frame, not a memory. Each
vehicle keeps its own obstacle memory, as now; occupancy is not exchanged,
because it is megabytes on a channel this item cuts to tens of bytes, and
because merging maps built under different drifts corrupts both. The frame
is fixed once, at the start, where every vehicle stands on a known pad, and
the estimator's drift (0.1 to 0.2 m over the 400 m mission on the
lidar-inertial profile) enters the separation as an uncertainty of the peer's
position that grows with the distance each has flown. The stage measures the
frame error between vehicles against the simulator's truth; if it stays
inside the margin of the 5 m separation gate, nothing more is needed.
Relative observation of peers is the second step, taken only if the first
falls short, and it is a lidar's remedy: a stereo pair with 6.4 m of
confident depth sees a peer too late for a separation of several times that.
Item 16 was done first, so this stage is rewritten for the visual-inertial
estimator before it starts: its drift, 0.1 to 0.4 percent of the path with no
bound, is what two vehicles' frames will differ by.

Visualization stays as it is: one spectator owns the follow transform and the
simulator's camera and moves to the next living vehicle when its own is lost;
every vehicle's path is shown at once, and the heavy layers (the memory
cloud, the planner's markers, the execution horizon) are the selected
vehicle's only. What this stage adds is a diagnostic layer of frame
disagreement, each peer's reported position against its true one, which is
evaluation only and never reaches a vehicle.

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

## 17. Flight In Degraded Visual Conditions

**Type:** dependent realism stage, with one repair that does not wait for it.

**Hard prerequisites:** item 14 for the sensor set; stage 0 below has none.

**Validation environment:** Urban Circuit Practice 01, the point-to-point
mission.

The camera stack flies on light it never measures, over surfaces whose texture
it has never varied. Those are two different failures and the item carries
both: too few photons, where the matcher has signal it cannot trust, and too
little texture, where it has all the signal it wants and nothing to match.
Three facts established on 2026-09-20 set the item up:

- **The location has no lamp in it.** The imported
  `urban_circuit_practice_01.sdf` carries no `<light>` element at all; every
  photon of every camera flight comes from
  `<scene><ambient>0.1 0.1 0.1</ambient>`, a uniform fill that nothing shadows
  and no failure can touch. "Turning the light off" here is one number set to
  zero.
- **The pair carries no noise model.** The cameras of
  `drone_city_nav/models/stereo_tof_v1/model.sdf` have no `<noise>` element, so
  an image is a noiseless render scaled by the ambient. Dividing every pixel by
  the same number leaves the local contrast the matcher works on intact, so
  dimming alone does not degrade depth gradually: it holds, and then collapses
  when 8-bit quantization removes the texture. A ramp without a noise model
  tests quantization, not low light.

- **One location's surfaces are the only ones ever rendered.** The confident
  depth of 6.4 m was measured on Urban Circuit Practice 01, whose materials
  carry the photographed grain of a real industrial interior. Camera flights
  render `world_gui.sdf` for that reason: the collision-only `world_sensor.sdf`
  carries no textures and a camera sees nothing on it
  ([`gazebo_simulation.md`](gazebo_simulation.md)). The project has flown the
  two extremes and nothing in between.

So the vehicle's guaranteed 6.4 m is an assumption about the world, and the
world is incapable of violating it. A real building loses its light, and an
airframe that carries its own loses that instead: a brownout, a driver, a hot
emitter, a splashed lens. This item makes the illumination something that can
fail and makes the vehicle answer for it.

The navigation invariants hold throughout. Unknown space stays traversable at
no penalty, there are no prohibited zones, vertical motion stays free, and
nothing here is a latch. One rule was proposed and rejected on 2026-09-20 for
exactly that reason: "stand no closer to an obstacle than where the vehicle
stood when the light went out" is the "no closer than now" form the project
forbids, and in the dark it is worse than anywhere else, because a vehicle
drifting onto a surface would be forbidden to leave it. The behaviour that rule
asked for comes out of stage 4 with no new rule at all: a ring of
time-of-flight sensors is another sensor with its own range and field inside
the same braking contract, which makes the speed towards a surface 1.2 m away
small and never zero.

### Stage 0: Both Inputs Of The Contract Become Measurements

Independent of light, and a hole in the first requirement today.
`SensorBrakingContract3D::guaranteed_detection_range_m` is a configured
constant (6.4 m on the stereo profile, `launch/sensor_profile.py`), and nothing
lowers it when the pair returns nothing. The only reduction in the code is
`min(guaranteed_detection_range_m, observed_range_m)` in
`sensor_braking_contract_3d.cpp`, and it applies to a motion the vehicle does
not face, where the range comes from what memory has already observed. A faced
motion is flown at the speed 6.4 m admits whether the last frame carried 74 000
returns or none. The returns already vary by a factor of five inside one lit
flight (14 172 to 74 099 points per frame, r579 and r583) and the contract
never learns of it.

The same is true of the other input, and it is the cheaper of the two. The
inequality charges `maximum_evidence_age_s` as a fixed term of the latency
(`sensor_braking_contract_3d.cpp`), fed by `latest_sensor_obstacle_maximum_age_ms`,
600 ms in `urban_mvp.yaml`. The speed law never reads how old the newest
observation actually is. The stack already computes it:
`latestSensorEvidenceFreshness` in `production_mppi_node_execution.cpp` hands
the real age and a freshness flag to the validator, which drops a stale scan
from the points it checks the body against. So the question "will I hit what I
can see" is answered on measured evidence and the question "how fast may I fly"
on a constant, and the debt register already records the measured age reaching
624 to 948 ms against the 600 charged.

The repair, for both: the contract's forward range becomes what the depth of
the recent frames actually stood behind, and its evidence age becomes the age
the freshness computation already produces. The admitted speed then follows the
sensor down when the pair goes blind and follows the clock up when the returns
stop arriving. The range is judged for the frame as a whole and not per ray,
because a bare corridor legitimately returns little and a per-ray rule would
crawl the vehicle through every empty room; the age needs no new computation at
all, only that the speed law stop charging a constant where a measurement
exists. What it costs: the contract's two central inputs start moving, so both
acceptance series are re-flown and the speed figures of items 14 and 16 are
re-measured against them. Carried in
[`technical_debt.md`](technical_debt.md) until it lands.

This stage is also what answers a stalled stream rather than a blinded one, and
the two halves of the stack answer it very differently today. The estimator is
built for it: an estimate goes unhealthy once `maximum_unaided_s` (1.0 s) has
passed since the last visual update, the node stops publishing, the autopilot
ends its external-vision fusion 200 ms later, the pose ages out and the
controller revokes the execution authority. Under that timeout the node
deliberately carries the last state forward through the IMU every 40 ms, so
that one late frame does not restart the fusion (item 16). The perception has
no such rule: the returns ride the frame pair, so a stalled stream stops them
all, including the time-of-flight cones folded into the same message
(`stereo_depth_node.cpp`), and the speed law goes on admitting what 6.4 m and
600 ms admit. The two timeouts are therefore asymmetric, and between them lies
a window nobody has flown on purpose: from 0.6 s, where the charged age is
already exceeded, to 1.0 s, where the pose is pure dead reckoning and still
declared healthy. At 2.45 m/s that window is 2.4 m of travel against a 2.0 m
margin, which is why it eats the margin without crossing it.

### Stage 1: A Dark World And A Camera That Has Noise

`<noise type="gaussian">` on both cameras, so that the signal falls with the
light while the noise does not and the matcher degrades with the ratio, which
is what a real imager does at low light: read noise is what dominates there.
The noise is stated with its source, and its cost is stated with it — it
lowers the confident depth item 14 stage 1 measured, so 6.4 m is re-measured by
the same procedure and the speed baseline moves.

The world's `ambient` becomes a parameter of the scenario and a permanently
dark variant of the urban location (`ambient 0`) becomes one of its worlds. The
resource materialization already rewrites the SDFs it installs
(`configure_lidar_visibility.py`, `configure_drone_lidar_model.py`), which is
where this belongs: no production code learns that a world can be dark.

Transitions are ramps and not switches, as the project owner asked on
2026-09-20: the illumination moves over seconds, so that the depth spends time
in the range where it is partly right. That range is the interesting one and a
hard switch skips it.

### Stage 2: Surfaces The Matcher Cannot Match

Darkness and texture fail differently, and only one of them is about photons. A
blank painted wall, poured concrete, a large uniform panel: fully lit, all the
signal an imager wants, and nothing to match between the two images.
Semi-global matching answers with nothing, or with an interpolated surface that
is not there. Depth that is absent is survivable once stage 0 lands, because
the contract sees the range collapse; depth that is confidently wrong is the
dangerous case and it is the one this stage has to bound.

A materialization variant renders the same geometry on progressively poorer
surfaces: the location's own materials, then uniform matte at a stated albedo,
then a weak procedural grain between the two. Nothing reaches production code,
as in stage 1. Measured on each: depth coverage, depth error against
evaluation-only truth, the rate of matches the filter has to gate, and the
range the depth still stands behind. The result is a curve of confident range
against surface texture, and stage 0's measured range has to track it at run
time: if a bare wall leaves a metre of confident depth, the contract must say
one metre and the vehicle must fly what one metre admits.

The remedies are compared in the same place and none is assumed: a wider
matching window, a different matcher, and the projected pattern of active
stereo, which makes the vehicle carry its own texture exactly as stage 3 makes
it carry its own light. That one device answers both failures of this item at
once, which is the argument for its price in the cost section below.

### Stage 3: Light On The Vehicle

A dark location turns the question into a hardware one: an airframe that flies
on video underground carries its own light. In the simulator that is a
`<light type="spot">` inside the vehicle model's link, which gz-sim attaches to
the link and the sensor cameras see. It has to be a flood over the pair's
field and not a beam: stereo matches the whole overlap, and a bright circle in
the middle of the frame buys depth in the middle of the frame. The DARPA SubT
teams carried panels of high-power LEDs behind diffusers, tens of watts.

The trade this stage measures instead of assuming: a carried light guarantees a
shorter range than an ambient fill does, because illuminance falls with the
square of the distance and a 120 degree field has to be flooded rather than
spotted. The confident depth probably lands at 3 to 4 m instead of 6.4, and the
admitted speed with it. What it buys is that the guarantee becomes a property
of the vehicle instead of a property of the location, which is what the braking
contract has always claimed it was.

Three options are compared before one is built: a white LED flood; an infrared
flood with the pair's filters removed, which is the same photons and invisible;
and active stereo of the RealSense class, which projects its own texture and
needs no ambient light at all. The last is the strongest and the most
expensive, which is what the cost section below has to settle.

### Stage 4: A Time-Of-Flight Ring As A Bumper

Four more time-of-flight sensors on the horizontal, beside the two of item 14.
They are the one part of the sensor set that does not care about light: the
simulated sensor is a `gpu_lidar`, which raycasts, and the real VL53L8 class
emits its own pulse, reaching 4 m in the dark against the 2.8 m on a bright
target in 5 000 lux that the model takes
([`camera_perception.md`](camera_perception.md)). Its range grows when the
light goes.

The scope is the point of the stage. This is a proximity bumper, not a cheap
lidar: four metres and 45 degrees per sensor are enough to hold a position
without touching a surface, to leave one, and to land, and they are not enough
to fly on. They enter the braking contract as sensors with their own range and
field, exactly as the two vertical ones already do, and that is the whole
integration. No new rule, no latch, no mode.

### Stage 5: The Light Fails, And What The Vehicle Does

Deterministic injection of a failure of the source the perception depends on,
seeded and written into the flight's manifest like every other scenario
parameter, expressible at the carried light and at the world's illumination
alike. The project owner's range, 2026-09-20: an outage arriving every 1 s to
1 min and lasting 1 s to 1 min. The injector is an evaluation component; no
fault injection enters production code, as in item 15 stage 3.

The stream is injected as a second and separate fault, because it is a
different failure that reaches different code. A dark frame arrives on time and
is useless by its content: the matcher returns nothing and the range collapses,
while the evidence stays as fresh as the clock says. A dropped or delayed frame
does not arrive at all: the range input has nothing new to say and the age is
what moves, together with the estimator's unaided timeout, since one pair feeds
both consumers. Frames are dropped and delayed over the range that brackets
stage 0's two timeouts, from a single frame to several seconds, and the flight
records which rung of the ladder below the vehicle reached and when.

The vehicle's answer is a ladder whose first rung is free:

1. **Stop.** Stage 0 does this by itself. The measured range collapses, the
   contract admits almost no speed, and the vehicle brakes. No new mechanism,
   only an honest input.
2. **Hold on what is still active.** The time-of-flight ring, the downward
   sensor for height, the IMU for attitude. The position drifts, because the
   filter has no images, and the ring bounds how far it drifts into a surface.
3. **Descend and land** if the light has not returned within a stated time. A
   controlled landing beats an uncontrolled drift, and a landed vehicle with a
   dead emitter is recoverable where a crashed one is not.
4. **Relocalize** when the light returns. The estimate has moved and the memory
   was built under the old one. This is the unbounded-drift entry the debt
   register carries as a deep rework; this item does not solve it, it states
   where it bites.

This is a failsafe against a crash and not a way to keep flying. Its honest
scope is stop, hold, land.

Two kinds of flight, and neither is a speed measurement: the mean flight speed
of the project's second requirement is measured with the illumination healthy,
as it is today.

- A **long** flight with the outages running, which must still reach its goal
  in truth.
- A **short** flight of about five minutes whose only question is whether the
  vehicle survives.

### What The Additions May Cost

A condition the project owner set on 2026-09-20: these additions must not
approach the price of one ordinary 3D lidar, or the exercise is pointless.
Order of magnitude, single units, 2026, to be replaced by sourced figures
before stage 3 is built:

| Set | Parts | USD |
|---|---|---|
| Today | stereo pair, two time-of-flight sensors | 80 to 170 |
| Stages 3 and 4, a flood and the ring | pair, six time-of-flight sensors, emitter | 140 to 320 |
| Active stereo with the ring | RealSense-class module, six time-of-flight sensors | 360 to 550 |
| A cheap solid-state 3D lidar | Livox Mid-360, Unitree class | 500 to 1000 |

The first path is honestly two to four times cheaper than the lidar. The second
reaches the cheaper lidars, and there the question "why not a lidar then" is
fair and this item either answers it or takes the first path.

Two costs the table does not carry, and one of them can invert the answer.
**Power**: a time-of-flight sensor is a tenth of a watt, but flooding a 120
degree field to 6 m is tens of watts, while a solid-state lidar draws 8 to 15.
Carried light can cost more battery than the lidar it replaces, and on a
multirotor that is flight minutes. **Integration**: six sensors on one bus are
six addresses, six mounts, six extrinsic calibrations and six failure modes,
which is free in money and not in the project's time. Both are stated with
measurements before stage 4 is built.

### Measurement And Completion

Measure, per flight: the illumination at the vehicle over time; depth coverage
and depth error against evaluation-only truth at each illumination level and on
each surface variant; the
contract's forward range and the speed it admits; the time spent on each rung
of the ladder above; the estimate's error against truth through an outage and
after it; the minimum distance to true occupancy; and physical collisions.

This item is complete when stage 0 has landed and both acceptance series have
been re-flown on it; when the confident range is published as a curve against
surface texture and the contract is shown to track it; when five long flights
on the dark world, with the carried light and the outages running, reach the
goal in truth with no collision; and when five short flights under the most
aggressive outage the parameters allow end with the vehicle intact, whether
landed or flying.

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

### 9. Large-Scale Realistic Location And Full-Mission Validation (Completed)

Shipped in [v0.4.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.4.0)
on 2026-09-20, closed with item 16. The project owner decided on 2026-09-19
that the project has two requirements and no others (the vehicle never crashes
and always reaches its goal, in truth; the mean flight speed exceeds the
figure of its sensor set, 1.2 m/s on the stereo set and 2.4 m/s on the lidar),
that stage A is closed by the camera profile without GNSS and not by the 3D
lidar it was written for, and that stage B, written for a city, is met by the
imported underground locations.

**Stage A**, the point-to-point mission on Urban Circuit Practice 01 on the
default sensor set, with no lidar on the vehicle, no static map and no GNSS,
headless, five flights on one commit with nothing changed between them: r579
to r583 on 7e95336e, the repository's defaults with no profile variable set
(the manifests record `stereo_tof`, `stereo_tof`, `visual_inertial` and a clean
checkout). Five of five complete and collision-free; mean flight speed
1.58/1.79/1.60/1.60/1.48 m/s against 1.2; the true position 1.42, 1.14, 1.37,
0.52 and 0.47 m from the goal at its acknowledgement; no failing line. The
stage was written for a release commit: 7e95336e is the release candidate; the
tag, the version and the push are the project owner's.

**Stage B**, a large and varied realistic location as the full-system
validation environment.
That is what the imported DARPA SubT locations are
([`environment_candidates.md`](environment_candidates.md),
`environments/environment_manifest.yaml`): Urban Circuit Practice 01, Cave
Circuit Practice 01 and Finals Prize Round World 07, each under CC BY 4.0
with a committed, SHA-pinned inventory of every transitive resource's
license, each imported through one pipeline that derives the collision world,
the textured world, the occupancy and the ESDF from the same source bundle.
Urban Circuit Practice 01 (605 x 528 x 73 cells at 0.5 m: multi-level rooms,
corridors, bends, two shafts, entrances at different heights) has been the
project's validation environment since item 12: every acceptance series of
items 12, 13 and 14 was flown on it with no static map, first on the 3D
lidar and now on cameras, with physical collision detection, the planner's
and the controller's diagnostics, the real-time factor and the CPU and GPU
budgets recorded on every flight, and with no route script and no exception
for its geometry anywhere in production code. The generated grid city was
removed from the repository in item 15 stage 0.

What the stage's text asked for and was not done: the mission has been flown
from one start to one goal, not from several placements, and the cave and the
finals world are imported and load but have not been flown. Neither is
required any longer; a second placement or a second location is a scenario
file away when a change needs one.

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

Known leftovers, measured and not gated (those still open are in
[`technical_debt.md`](technical_debt.md)): the loop runs near 43 Hz with about
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
item 10. Since 2026-09-19 the availability targets are measurements and no
longer anyone's gate (item 9), and the cooperative re-flights wait for item
15.

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

### 14. Camera-Based 3D Perception Without Lidar Or Static Maps (Completed)

Shipped in [v0.4.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.4.0)
on 2026-09-20, closed on 2026-09-19 on the urban point-to-point mission with
no lidar on the vehicle and no static map; the design, the measurements
behind it and what is known to remain are in [`camera_perception.md`](camera_perception.md), the
vision path of the memory in [`obstacle_mapping.md`](obstacle_mapping.md), the
sensors in [`gazebo_simulation.md`](gazebo_simulation.md). The vehicle carries
a forward stereo pair (1280 x 960, 120 degrees, 0.20 m baseline, 7.5 Hz) and
two 8 x 8 time-of-flight sensors that look up and down (2.8 m);
`stereo_depth_node` recovers depth by semi-global matching and hands the raw
world the same beam observations the lidar produced, so the obstacle memory,
the raw snapshots, the persistent planner, MPPI and the execution core run
unchanged. Above the sensor boundary three things changed: the braking
contract reads the range of the sensor whose field holds the motion (6.4 m of
confident depth forward, 2.8 m vertically), a gaze policy turns the heading to
the motion, and a motion the vehicle does not face answers to what memory has
observed along it. This sensor set is the default of every flight since
(`CAMERA_PROFILE=stereo_tof NAVIGATION_SENSOR_PROFILE=stereo_tof`); the 3D
lidar stays available on request, and single-vehicle flights on the stereo
profile use the `gnss` localization profile until item 16.

Measured at the r506 to r510 series on 948d4df2, lidar absent from the model:
mission complete and no crash in five flights of five,
1.64/1.48/1.61/1.59/1.67 m/s against the 1.226 m/s gate (half of the 2.452 m/s
the contract admits forward), route availability 97.6 to 98.8 percent against
the 90 percent floor, no-route holds 1.2 to 2.5 percent, planner p95 152 to
158 ms, tick 20.8 to 21.4 ms at p50; 95.5 to 97.1 percent of the flown path
had been observed before it was entered (90.5 to 92.0 by the pair, 3.8 to 5.5
by the time-of-flight sensors); the shafts were climbed at 0.81 to 0.97 m/s at
the median, above half of the 1.29 m/s the time-of-flight range admits;
perception latency from exposure to the tick 300 to 320 ms at p50, 464 to
496 ms at p95; the depth node 1.4 cores, the onboard processes 2.3 cores at
p50 without it. The control series of the lidar profile on the same commit,
r511 to r515: 2.65 to 2.82 m/s, no crash. The final series on the defaults,
with no profile variable set (r523 to r527 on cd461d01): five of five, no
crash, 1.32/1.17/1.43/1.57/1.44 m/s, route availability 98.2 to 99.2 percent,
tick 21.8 to 23.5 ms at p50. Between the two series a first attempt at the
final one lost r518: a timestamp reacquisition left the vehicle on its
resident horizon for 1.7 s, and that horizon flew the route's turn with the
pair still facing away. A published horizon since may not carry speed along a
motion its own planned heading leaves unseen, which is what the difference in
speed between the two series is. Shadow comparison against the lidar
memory (stage 2): occupied precision 97 to 98 percent within one voxel,
recall 96 to 99 percent inside the third of the lidar's volume the vision
memory observes.

Known and not gated: the mean speed gate of 1.226 m/s, and the requirement of
1.2 m/s that replaced it, is missed by one flight of the final five
(1.165 m/s); the evidence age exceeds the 600 ms the contract charges
on about 1 percent of the ticks (624 to 832 ms at most) on a workstation that
holds a real-time factor of 0.9 beside the render, the image bridge and the
matcher; lateral tracking p99 0.305 and 0.320 m in two flights of five against
0.25 m; the closest pass to truth occupancy was 0.41 to 0.75 m from the
vehicle's centre to a 0.5 m voxel's centre against 0.84 m on the lidar; a
faced motion between the pair's field and a time-of-flight cone (52.4 to 67.5
degrees of elevation) is flown at the unobserved speed of 1 m/s, because
the pair is mounted rigidly and only the heading turns it: a tilt servo on
the pair or a third time-of-flight sensor would close that gap, and neither
is scheduled; a motion the vehicle does not face is flown only through space
memory has observed, so the vehicle turns before it enters unseen space
sideways or backwards where the lidar flew every direction alike; the
multi-vehicle launch carries the same defaults and has not been flown on them,
which waits for item 15 and for a host that renders eight cameras.

### 16. Flight Without GNSS And Without Lidar (Completed)

Shipped in [v0.4.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.4.0)
on 2026-09-20. The vehicle flies the urban point-to-point mission with
no GNSS, no magnetometer, no lidar and no static map: a visual-inertial
estimator on the forward stereo pair is the autopilot's external odometry, and
`visual_inertial` is the default localization profile wherever the stereo
sensor set is the default (the lidar keeps `lidar_inertial`, `gnss` is a
request, the multi-vehicle launches stay on `gnss` until item 15's shared
frame). The estimator, its measurements and its health are in
[`localization.md`](localization.md); the goal-in-truth check in
[`testing.md`](testing.md); real time on the camera profile in
[`gazebo_simulation.md`](gazebo_simulation.md).

Decided before implementation: a stereo multi-state constraint Kalman filter
written on Eigen alone with first-estimate Jacobians (OpenCV in the front end
only), offline replay before any flight, a reference system as an offline tool
and never a dependency, EKF2 the owner of the estimate, no code dependency
between the estimator and the camera perception, and completion by two series
on one commit.

Acceptance on 7e95336e, each flight inspected before the next, no failing line
in any of the ten:

| Series | Flights | Mean flight speed, m/s | True position from the goal, m |
|---|---|---|---|
| Stereo set, no GNSS, the defaults (`visual_inertial`) | r579 to r583 | 1.58 / 1.79 / 1.60 / 1.60 / 1.48 (above 1.2) | 1.42 / 1.14 / 1.37 / 0.52 / 0.47 |
| 3D lidar, no GNSS (`lidar_inertial`) | r584 to r588 | 2.67 / 2.51 / 2.53 / 2.42 / 2.65 (above 2.4) | 0.48 / 0.55 / 0.85 / 0.45 / 0.66 |

No crash and no contact in either. Notes of the first series: the autopilot's
position estimate 0.67 to 1.19 m from the truth across the track at p95 in
four flights (the odometry's drift; the 0.35 m figure was the GNSS profile's).
Notes of the second: route availability 93.4 to 96.7 percent and no-route
holds 3.0 to 6.8 percent, as that profile has held since item 12.

What the stages found:

- **Stage 0.** The autopilot's clock is the simulation's in lockstep and its
  synchronisation with the wall clock was what failed at a real-time factor
  under 1: `UXRCE_DDS_SYNCT 0` and an identity time mapper, reacquisitions 17
  to 0 per flight. The pair's images are taken from Gazebo inside the
  matcher's process, not over the bridge (55 MB/s of DDS), and the GPU is
  polled every 10 s: the real-time factor went from 0.84 to 0.93 to 0.97 at
  the mean. The matcher was not moved to the GPU: it was not what the
  simulator stalled behind. Headroom: 1.5 busy cores beside a flight change
  nothing; the pair at 15 Hz costs 0.05 to 0.1 of real-time factor and at
  30 Hz real time is not held. Four navigation defects of the camera profile
  were repaired on the way (a steep motion judged with the forward margin, a
  memory revision published under two stamps, low-speed refusals of the
  unseen-motion rule, a heading frozen while an arrival rests): five flights
  of five at 1.59 to 1.76 m/s (r539 to r543).
- **A defect older than this item**, found by a probe at 15 Hz that lost the
  vehicle: a planning tick lasted up to 2.5 s (r545; 6 s of resident horizon
  in r536) because the latest scan's returns, a centimetre apart on a wall the
  envelope touches, were each walked against the departure at every body
  position of every segment: 487 ms for one path validation. The scan is
  thinned to the nearest return of every 0.05 m cell and the walk made once
  per interval: 15 ms, the longest tick 226 ms (r548).
- **Stage 1.** Recorder, recordings and the offline replay under
  `log/tools/vio`; the frame stamp calibrated to the IMU (+4 ms, two records);
  the filter on Eigen alone with first-estimate Jacobians and its unit tests;
  the numbers in [localization.md](localization.md). The reference system
  was not run on the recordings: OpenVINS without ROS needs Ceres, which
  builds from source, and Boost.Filesystem, which the container image does
  not carry, and then a runner for these records; the attempt stopped there,
  as a tool and not a criterion. The filter's drift, 0.1 to 0.4 percent of the
  path, is at the level such systems publish.
- **Stage 2.** `visual_inertial_shadow` and the goal-in-truth check: five
  flights of five (r555 to r559), the estimator 0.53 core, 0.17 to 1.73 m from
  the truth after 400 to 490 m.
- **Stage 3.** `visual_inertial` with the lidar perceiving: the heaviest
  configuration (real-time factor 0.83, so its speeds of 1.9 to 2.4 m/s are
  not the lidar set's). r561 did not reach its goal: the autopilot threw the
  odometry away after a 0.4 m correction and reset its position by 1.19 m;
  fused with the noise a frame's update moves the pose by (0.3 m) five flights
  of five reached their goals, 1.19 to 1.62 m from them in truth (r566 to
  r570). The time-of-flight ranges are not fused: the vertical is the
  best-held axis.
- **Stage 4.** First flights with no lidar, no GNSS and no magnetometer: no
  crash, 1.41 to 1.60 m/s, but two of six were acknowledged 1.97 and 2.31 m
  from the goal in truth (r572, r575): the heading had walked 2 to 4 degrees.
  The filter had been told a gyroscope eighteen times noisier than it is; with
  the gyroscope's own noise the recorded flights end within a degree and
  0.25 to 0.89 m.

Known to remain. The estimate drifts 0.1 to 0.4 percent of the path and the
vehicle was acknowledged up to 1.42 m from its goal in truth against a 2.0 m
capture radius: an odometry without loop closure has no bound, and a mission
several times longer needs long-lived points in the filter's state or a map to
relocalize against (a deep rework). r587 flew 2.42 m/s against 2.4. No
reference system has been run on the recordings. The multi-vehicle path is
not flight-verified on cameras or without GNSS (item 15). The register of
what is set aside, with the class of every entry, is
[`technical_debt.md`](technical_debt.md).

