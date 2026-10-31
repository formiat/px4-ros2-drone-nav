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

The requirements, invariants and acceptance conditions every item is built
and accepted against are in [`specification.md`](specification.md).

Numbers are stable identifiers. A completed item keeps its number, is
summarized in the Completed section at the end of this file, and is never
renumbered or reused; a new item takes the next free number.

## 11. Valid 3D Static Maps For New Environments

**Type:** dependent implementation and validation stage.

**Hard prerequisites:** items 8 and 34 for the primary autonomous-survey
acquisition path (item 12's incremental exploration backend, which this
item named until 2026-10-24, left the repository on 2026-08-28, 8b53e983;
the survey is item 34's mission under specification F20).

Create a valid static map for every new complex environment. Here, quality
means geometrically correct, physically valid, and aligned with the real
collision environment: every real obstacle relevant to the aircraft footprint
must be represented. It does not require unnecessarily high visual or voxel
detail.

Every new-environment static map must be three-dimensional. Two-dimensional
maps are insufficient for multi-level geometry, tunnels, shafts, windows,
doors, and other traversable 3D passages.

The primary acquisition path uses item 8's production 3D lidar and item 34's
survey mission to survey every reachable part of an
environment, then persists the resulting validated obstacle memory as the
environment's static-map artifact. Direct generation from collision geometry
may remain as a secondary generation or cross-validation tool. Every artifact
must be versioned with the environment collision geometry, source provenance,
coordinate transform, resolution, coverage evidence, and validation result.

This stage is complete when every supported new environment has a reproducible
3D static-map generation or acquisition path and that map passes coverage,
alignment, and raw-collision validation against its physical world.

One entry of the debt register lands here, attached on 2026-09-26: the cave
and the finals locations, imported and never flown (N2), are flown first
by item 33's held-out point-to-point flights, with nothing adapted to
them, and then by the survey acquisition itself, which covers every
reachable part of an environment before it persists the map. The 2D lidar path (R1) was to go
with the first 3D static map and went before it, on 2026-09-26 (59f55e1f):
a static-map request now refuses to start while the environment has no 3D
static occupancy, which is what this item delivers.

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

### The Host, The Sensor Set, And Why Two Vehicles And Not Four

Decided by the project owner on 2026-09-21, from the budget below rather than
from preference: **this item flies two vehicles on the 3D lidar profile**, as
an exception to the camera default of item 14. Four camera vehicles are
impossible on this workstation and four lidar vehicles would only run at a
real-time factor that makes half the record meaningless. What follows is the
arithmetic, so that the decision can be re-checked on another machine instead
of taken on trust.

The reference workstation, the one every figure in this repository was
measured on:

| | |
|---|---|
| CPU | AMD Ryzen 9 5900HX, **8 physical cores, 16 threads** |
| Memory | 30 GiB |
| GPU | NVIDIA GeForce RTX 3060 Laptop, 6 GiB |

The unit of every CPU figure below is a core as `/proc` reports it: 1.00 is
one logical CPU fully busy, and the nominal ceiling is 16.0. That ceiling is
not the usable one. Each process was measured while the machine was otherwise
idle, so it had a physical core to itself; once both SMT siblings of a core
are loaded, each thread retires less per second, the work stretches, and the
demand in wall-clock terms grows further. With the p95 of these processes
sitting 25 to 30 percent above their p50, and with the operating system and
the desktop to leave room for, **the practical ceiling for real-time work is
about 10 to 11 of the 16**, not 16.

Measured per vehicle, on single flights, from the records
([`resource_budget.md`](resource_budget.md)):

| Vehicle | Onboard cores p50 | p95 | Flights |
|---|---|---|---|
| Lidar, `gnss` (what the cooperative mission flies today) | 3.03 to 3.43 | 3.85 to 4.14 | r352 to r356 |
| Lidar, `lidar_inertial` (what stage 4 accepts on) | 3.5 to 3.8 | 4.4 to 4.7 | r405 to r434 |
| Stereo pair, `visual_inertial` | 4.89 | 5.69 | r576 |

`multi_vehicle.launch.py` gives every vehicle its own full set — the MPPI
controller in its own container, the obstacle memory, the offboard node, the
crash node, and on cameras the depth node — so those figures multiply by the
number of vehicles almost exactly. Beside them the host carries the simulator:
the Gazebo server with one GPU lidar is 0.59 cores, a PX4 SITL instance 0.19,
and the bridges, the referee, the spectator, the truth adapters and the
captures 0.3 to 1.3 together. The simulator with **one** vehicle's two
1280 x 960 cameras on the textured world is 3.3 cores on its own.

The GPU on one lidar vehicle is 42 percent at p50 and 51 at p95. It divides,
by inference and not by measurement, into the controller's 8192 rollouts
(6.4 ms of GPU time at p50 inside a tick of 23 to 25 ms, so 26 to 28 percent)
and the simulated lidar's rendering (the remaining 14 to 16). `nvidia-smi pmon
-s um` reports `sm%` per process and would settle the split on one
single-vehicle flight; it has not been run.

**Four vehicles, 3D lidar:** 12.8 cores of onboard work at p50, 0.8 of PX4,
1.5 to 2.5 of a Gazebo server rendering four lidars and serialising four
clouds, and about 1.0 of referee, spectator, bridges and captures — **about 16
at p50 and 20 at p95** against a usable 10 to 11. The GPU is 168 percent at
p50 on its own. It runs, at a real-time factor near 0.6.

**Four vehicles, stereo pairs:** 19.6 cores of onboard work at p50 before the
simulator, which must render eight 1280 x 960 cameras. **About 34 cores**, or
three to four times the machine. No parameter closes that gap; it needs a
different host.

**Two vehicles, 3D lidar on `lidar_inertial`:** 7.3 cores of onboard work at
p50 and 9.1 at p95, 0.4 of PX4, about 1.0 of a two-lidar Gazebo server and
about 1.0 of the harness — **about 8.7 at p50 and 10.6 at p95**, and 84
percent of the GPU at p50, 102 at p95. That fits at the median and touches the
ceiling in bursts. Two cheap levers are held in reserve for the bursts: the
obstacle memory's transport at 5 Hz instead of 10 (about 0.3 cores a vehicle,
and the observation age goes from 190 to 252 ms against the 600 the braking
contract charges), and the diagnostic captures off (up to 1.1 cores at p95).

### What A Low Real-Time Factor Does And Does Not Invalidate

Worth stating, because it is what makes two vehicles sufficient rather than a
retreat. PX4 SITL is lockstep, so the autopilot waits for the simulator and
the flight dynamics are not distorted by a slow host: the flight merely takes
longer in wall-clock time. The quantities this item measures — minimum
separation and its margin in metres, delivery latency and loss, the share of
flight time a peer is unknown, the frame error between vehicles — are metres,
fractions and simulation-time intervals, and none of them is corrupted.

What a low real-time factor does corrupt is what is measured on the host's
clock: the tick and planner percentiles, and the time each computation takes,
which at a factor of 0.5 is half as long in flight. Since item 20 (2026-10-05,
specification K24, K25) the onboard loop keeps the simulation clock, so it
slows with the world and its rate stays at most its nominal 50 ticks a
simulated second (the 72 against 43 measured here before was the wall timer);
the mean flight speed is measured on the simulation clock too. What is left is
that a slowed flight is flown with a faster computer than the vehicle has: on
one vehicle at 0.5 the tick took 10.8 ms of simulation against 22 at 1.0, and
2 percent of the ticks passed 20 ms against 67 to 71. A slow host still
flatters this stack, which is why slowed flights are not mixed with the
single-vehicle series.

Whether four vehicles at 0.5 are flattered is not known, and is decided by
one measurement before anything is built (the owner's decision of
2026-10-05, specification K25): four vehicles share the host, each
computation is slowed by the others, and the contention may give back what
the slowing takes. The first four-vehicle flight at 0.5 reports each
vehicle's tick in milliseconds of simulation. Near 22 ms, the tick of one
vehicle at 1.0, the contention already flies each vehicle with the computer it
has, and the flight is taken as it is. Clearly shorter, the cooperative
flights are flattered, and a relay outside the onboard code holds each
estimate, memory update and horizon until its age on the simulation clock
reaches its age divided by the factor (size M; it leaves the tick's rate as
it is). The delay is never emulated inside the onboard nodes: that would put
simulation-only behaviour into the production code.

### What Two Vehicles Cover, And What They Do Not

Every subject of this item is pairwise. The link model of stage 1 is a
component with its own contract test and is exercised by any linked pair. The
intent of stage 2 is a message size and a rate, measured per vehicle. The
separation of stage 3 is computed per pair, and the cases that matter —
a peer whose intent is late, lost or expired, an asymmetric link where one
vehicle hears and the other does not, and the determinism of complementary
maneuver choice under that asymmetry — all appear with two. The frame error of
stage 4 is a difference between two frames.

What two vehicles do not cover: a partition that hides half the fleet, the
shared budget of the telemetry-radio class across four senders, the mesh's
multi-hop relaying through a peer that hears both, and the lane capacity of a
static passage with deterministic right-of-way that item 6 accepted on four.
Those need the fleet, and the way to reach them on this host is a confirmation
flight of four lidar vehicles at a real-time factor near 0.6, whose separation,
channel and frame figures are valid and whose timing figures are stated as not
comparable. Item 6's acceptance was on four vehicles, so narrowing the
acceptance here is a deliberate narrowing and is recorded as one.

If four vehicles at a real-time factor of 1.00 are ever wanted on this
machine, the levers, with the measured or expected effect of each: the
controller's rollouts from 8192 to 2048, which divides the MPPI's share of the
GPU by about four (112 percent of the GPU for four vehicles becomes 28) at the
cost of a worse local optimum that is not measured; the lidar from 360 x 181
to 180 x 91, which divides the render, the bridge and the memory node's 0.83
cores by about four, but which is an input of the braking contract and not a
quality knob — `configuration.md` records a wall that sailed between two rows
at 10 degree spacing; the transport rate and the captures above. Together they
bring four lidar vehicles to roughly 8 to 9 onboard cores and 60 percent of the
GPU, at the price that the cooperative figures are then no longer comparable
with any single-vehicle series.

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
   the bridge carries to the link simulator: exact geometry, no map, one pair
   at 10 Hz for the two vehicles of the acceptance and six for a fleet of
   four; it is checked on Urban Circuit Practice
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
profile with the multi-vehicle launch running one estimator per vehicle. Two
vehicles carry it: a frame error is a difference between two frames, and the
budget above is what the host holds at a real-time factor of 1.

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
estimator before it starts: its drift, 0.1 to 0.4 percent of the path on a
first pass, is what two vehicles' frames will differ by. Item 19 measured it
on a doubled path and bounded it where a vehicle returns to ground it has
seen, by registering its depth against a map of its own; two vehicles are
two such maps, never registered against each other, and their difference is
what this stage measures.

Visualization stays as it is: one spectator owns the follow transform and the
simulator's camera and moves to the next living vehicle when its own is lost;
every vehicle's path is shown at once, and the heavy layers (the memory
cloud, the planner's markers, the execution horizon) are the selected
vehicle's only. What this stage adds is a diagnostic layer of frame
disagreement, each peer's reported position against its true one, which is
evaluation only and never reaches a vehicle.

### Stage 5: A Memory Shared Between Vehicles

Decided by the project owner on 2026-10-03, after the shared frame of stage
4, which it needs. Stage 4's direction stands for the frame: vehicles share a
frame first, and occupancy was left out there because it is megabytes on a
channel this item cuts to tens of bytes and because maps built under
different drifts corrupt each other. This stage takes both objections on.

- **What is exchanged.** Not the grid: changes of occupancy since the last
  exchange, in the shared frame, coarsened and compressed to what the
  channel class carries, with the sender's frame uncertainty beside them.
  What a link of each class can carry is measured first, and decides the
  resolution.
- **What a peer's occupancy may do.** It reaches the strategic planner only:
  the choice of route, a dead end another vehicle has already found, the
  proof that a goal is unreachable (item 19), which several vehicles close
  sooner than one. It never reaches the braking contract, the collision
  checks or the tube: those stay on what the vehicle's own sensors measured,
  and the vehicle's own measurement overrides a peer's wherever both exist.
  The invariant that space is prohibited by measurement alone is restated
  for this stage: a peer's measurement, inflated by the frame error between
  the two, is a reason to plan elsewhere, never a reason to stop.
- **The hard part is the alignment**: 0.2 to 0.7 m of frame error between
  vehicles against a voxel of 0.25 m and a rest clearance of 0.35 m. A wall
  laid half a metre off closes a doorway that is open. The stage measures
  the error between two vehicles' memories of the same surfaces, and either
  registers one against the other (the point-to-plane registration both
  estimators already carry) or inflates the peer's occupancy by it.
- **Size:** L to XL. Complete when two vehicles on the mesh class each plan
  through space only the other has seen, a dead end found by one is not
  entered by the other, and no flight shows a peer's occupancy in a braking
  or collision decision.

### Measurement And Completion

Measure, per flight and per channel class: message rate and bytes per
vehicle, delivery latency and loss, the share of flight time each vehicle
spends with each peer unknown, minimum separation and its margin over the
gate, maneuver decisions taken under asymmetric knowledge, and the frame
error between vehicles on the lidar-inertial profile. Every flight records
the resources as the single-vehicle flights do, so that the budget above is
re-derived rather than assumed, and states its real-time factor.

This item is complete when stage 0 has landed and, with **two vehicles on the
3D lidar and the `lidar_inertial` profile** at a real-time factor of 1, the
acceptance series passes under the mesh and cellular classes with the intent
within the channel's budget, the referee's separation gates hold under
scripted outages, and the frame error stays inside the margin of the
separation gate. One further flight of four vehicles confirms what a pair
cannot show — a partition that hides half the fleet, the shared radio budget
across four senders, multi-hop relaying, and passage lane capacity — and it is
flown at whatever real-time factor the host gives, with its separation,
channel and frame figures counted and its timing figures recorded as not
comparable. Before that flight's figures are taken, its per-vehicle tick in milliseconds
of simulation decides whether the delay relay above is needed.

## 21. Moving Obstacles: Doors And Bodies That Move Slowly

**Type:** perception and safety, general; not tied to a sensor. In the
discussion of 2026-10-19 (recorded 2026-10-24) the item stands behind items
33 to 35, and for a product the bodies that move (people) matter more than
the doors: when it comes back, the people first.

**Hard prerequisites:** item 17 stage 8, for the confidence of an occupancy
(its count of confirmations); items 17 and 18 are not otherwise needed.

**Validation environment:** Urban Circuit Practice 01 with actuated doors, a
materialization variant of the location as item 17's dark world and item
18's smoke are. The location carries no door today: its "door" names are
room tiles, and its world has no joint.

Proposed by the project owner on 2026-09-27. A door is the case to design
for: a solid, observable body that is still most of the time and moves
sometimes, slowly. What the vehicle must do with it:

- **A body that moves while the vehicle sees it is not written into the
  obstacle memory as a wall, and the vehicle does not collide with it.** Both
  at once: keeping it out of the memory must not mean flying into it.
- **Slow motion counts as motion.** A door closing over several seconds is a
  moving body, not a sequence of walls.
- **A body that stood still while the vehicle passed is an ordinary static
  obstacle.** A door closed and motionless on the way out is a wall in the
  memory, as any wall is.
- **The memory keeps the latest state it observed.** On the way back the door
  is open: the vehicle sees the opening and flies through it, and the memory
  holds the opening, not the wall it recorded before. That comes from
  observation, never from rewriting the memory from scratch.

**The memory keeps what is static and forgets only what is transient.**
Decided by the project owner on 2026-10-03, and it replaces item 17 stage
8's decay, which is switched off since that day and not to be used again
(specification K10). That decay forgot by age: a wall the vehicle stopped
looking at went the way of a trail, ten minutes after twenty confirmations,
and item 19's proof never closed over a memory that kept losing its walls.
Age says nothing about what a thing is. This item gives the memory two
classes instead:

- **Static** is what the memory holds by default, for the whole flight. Only
  a free ray through a voxel clears it, as now.
- **Transient** is what behaves as transient, by measurement: an occupancy
  that appeared where free space had already been observed, one that
  collected few confirmations and was gone at the next look, a body that
  moved between two scans, and the darkness or smoke a blind frame marks
  (item 17's K14, item 18), which is a statement about the moment and not
  about the place. Only the transient class fades, and by re-observation
  first.
- **Until this lands**, with the decay off, a transient that the vehicle
  does not look at again stays a wall, and the darkness K14 marks fades only
  by being seen through. That is the state the acceptance of this item
  starts from and measures.

**Why this is not items 17 and 18.** Darkness and smoke are unobservability:
the sensor looks and cannot see, and item 17 stage 8 writes that as a
prohibition that decays. A door is the opposite, fully observable and solid,
and what it adds is motion. What the stack already does, stated precisely:
a door that opens is cleared as soon as free rays pass through where it
stood (`miss_weight` and the Schmitt trigger of the memory, the mechanism
item 18 describes), and a door that closes is written by its hits, so the
last observed state of a door the sensor looks at again is already kept
without any rewriting. What is missing is the rest: nothing tells a body in
motion from a wall — a door swinging in view leaves its sweep in the memory
until later rays clear it, and item 17 stage 8's decay and confidence only
shorten that — and nothing in the collision validation or in the braking
contract knows that an obstacle can come towards the vehicle: both assume a
static world, so a body closing on the vehicle is met with the stopping
distance of a wall.

Stages, as seen on 2026-09-27:

- **Stage 0.** The door variant of the location and the measurement of what
  the stack does today: a door closed on the way out and open on the way
  back, a door moving while the vehicle approaches, a slow one; the smear it
  leaves in the memory and how long it lives, the replacements of the route,
  the minimum distance to the moving body, and collisions.
- **Stage 1. Motion in the evidence.** A voxel that turns occupied and free
  again within seconds while the sensor keeps looking at it is moving, and
  so is a cluster of occupied voxels that shifts between scans. Moving
  evidence goes to a short-lived layer of its own and not into the
  persistent memory; a body that stops for long enough becomes an ordinary
  occupancy. The layer is measurement, decays, and closes nothing it does
  not observe.
- **Stage 2. Avoidance.** The moving layer enters the collision validation
  and the braking contract with the space the body can sweep over the
  horizon, bounded by the speed measured for it, so that a closing body is
  met at the distance its closing speed demands. The exit guarantee holds: a
  body that moves onto the vehicle's own position does not freeze it.
- **Stage 3. The door flown out and back.** Closed on the way out, open on
  the way back: the route through it is found from what the vehicle saw last,
  with no reset of the memory.

The navigation invariants hold throughout: unknown space stays free at no
penalty, the only prohibitions are measurements, and nothing keyed on the
vehicle's history or on a configured zone is a rule.

### Measurement And Completion

Measure, per flight: the moving evidence detected against the truth of the
doors' motion; the occupancy the memory kept from a moving body and how long;
the route replacements it caused; the minimum distance to a moving body; the
stopping margin against its closing speed; and collisions. Complete when, on
both profiles, five flights through the door variant reach the goal in truth
without a collision, a door moving in view leaves no occupancy older than
the stated decay, and the out-and-back door flight takes the open door on the
way back in every one of five.

### The Thermal Core As A Reader Of Moving Bodies (proposed 2026-10-29)

Proposed by the project owner on 2026-10-29: the bodies that move in a
place where the vehicle flies are mostly warm (people, animals, running
vehicles), and the thermal core the stack carries since item 32 (K39,
K49: a Lepton-class core, 160 x 120 at 57 degrees, 9 Hz) reads them as hot
pixels. The proposal reads "warm" as "moving" in a first iteration: a
warm body is not written into the persistent memory as a wall, either kept
out of the memory or written as a class of its own (a "heat body"), never
as an ordinary obstacle.

Weighed on 2026-10-29, and taken up as the first source of evidence for
stage 1 (the moving layer), with the proposal's own limits stated:

- **Warm is a strong hint of a body, not a measurement of motion.** A
  radiator, a lamp, a wall the sun warmed and a parked car with a hot
  engine are warm and still; a door, a cart and a cold vehicle move and are
  not warm. So the core's hot pixels do not decide motion by themselves:
  they say "a body that may move stands there", and the motion itself is
  measured as stage 1 says (an occupancy that appears in observed free
  space, a cluster that shifts between scans, a voxel that turns occupied
  and free again within seconds). The core raises the evidence to the
  moving layer at once, before the shift is seen, which is what a body
  walking towards the vehicle needs; the layer's decay and re-observation
  rules are the same for a warm body and a cold one.
- **A warm body is never left out of the collision validation.** Keeping it
  out of the persistent memory (so that a person who walked past leaves no
  wall) is the point; flying into it is not: the moving layer enters the
  braking contract and the collision validation with the sweep the body
  can make (stage 2), and item 32's standoff ball is the shape of that
  prohibition already in the code (K49: a ball round the range point,
  observed unobservable, carved only by the core's own look). A person
  differs from item 32's source in that the ball must move with the body
  and die with it: the heat marks of a body that left are cleared by the
  next look at the place (I2), not kept until a free ray carves them.
- **What the core gives for nothing:** the range (the pair's hits in the hot
  cone, the lidar's hits without the weak returns, the body's apparent
  size for a body of a known class: a person about 0.5 m across, K49's
  size rule with another constant), the bearing fix over the vehicle's
  motion, the event on the mission's words (`HEAT_SOURCE`'s line, with a
  class), and the overlay. What it lacks: the field of 57 degrees (a body
  beside the vehicle is unseen; the lidar and the ring see it as a body
  without a class), and the temperature threshold of K49 (30 K over the
  frame's median) set for a 500 K source, which a person at 305 K against
  a 288 K room does not reach: the reader needs a second, lower threshold
  for bodies, measured on the rig against the warm walls of item 32's
  signature (311 K).

So the stages gain one step: **stage 1a, warm bodies on the rig and in the
location** (a walking body model at 305 to 310 K, its signature at 2 to
20 m against the warm wall, the threshold with a 20 percent margin), and
stage 1 takes the core's class as evidence beside the shift between scans.
The invariants hold: the class is a measurement of the frame, no list of
kinds, no zone, and a warm body that stands still for long enough becomes an
ordinary occupancy as any body does. The acceptance adds a walking body to
the door variant: five flights past a body that crosses the route, no
contact, no occupancy of it older than the stated decay, and the mean
speed of R3 kept.

## 22. Passive Navigation Without Illumination: Thermal-Inertial

**Type:** sensing and localization, general.

**Hard prerequisites:** item 18 stage 3 for the thermal channel, and item 17
stage 8 for darkness as observed unobservability.

**Validation environment:** Urban Circuit Practice 01 in total darkness, item
17's dark world with the carried light absent, and a thermal materialization
of the location (below).

Decided by the project owner on 2026-09-27. Item 17 flies the dark on a
carried light, and without it the camera vehicle cannot place itself: its
estimator sees nothing, falls silent within a second, and what remains is a
few metres of retreat on the IMU. A light of another colour — infrared,
ultraviolet, a projector of active stereo — is the same thing as the lamp: it
emits in order to see. This item teaches the vehicle to fly in total darkness
with **no illumination of its own at all**.

**What may emit.** Nothing that lights the scene. The time-of-flight sensors
stay, as the project owner's stated exception of 2026-09-27: they emit short,
low-power, eye-safe infrared pulses to measure a few metres, and they are the
vehicle's bumper in the dark. No lamp, no infrared or ultraviolet illuminator,
no pattern projector.

**What sees without emitting.** A visible camera in total darkness receives
no photons, and no algorithm recovers a scene from nothing; that is physics,
not difficulty. The passive sensor that works there is the long-wave infrared
(thermal) camera: surfaces emit their own heat radiation, which it images in
total darkness and through smoke. Item 18 stage 3 brings that channel to the
vehicle; this item makes it the one the vehicle localizes and perceives on.
What it costs, to be measured and not assumed: indoor surfaces often sit
within a few degrees of each other, so the images are of low contrast and the
features few, the thermal counterpart of a textureless wall; the resolution
is low (160 x 120 to 640 x 512); and the cameras calibrate on a shutter that
drops frames for a fraction of a second. In the simulator a thermal camera
sees only the temperatures objects are given, so the location needs a thermal
materialization, and how realistic its temperatures are is part of what the
item has to state.

Stages, as seen on 2026-09-27:

- **Stage 0. The thermal world.** Temperatures assigned to the location's
  surfaces in a realistic indoor range, with the low contrast that implies;
  the thermal camera's view measured against truth: features per frame,
  contrast, the stretches with nothing to track.
- **Stage 1. Thermal-inertial odometry (TIO).** The stereo MSCKF of item 16
  on thermal frames: a tracker on radiometrically normalized images, the
  shutter's dropped frames handled as the stream's holes are, replayed
  offline on recordings before any flight.
- **Stage 2. A range for the braking contract.** Thermal stereo depth, with
  the time-of-flight ring as the near bumper: the confident range in total
  darkness is measured and the contract flies what it admits.
- **Stage 3. Relocalization against its own map**, as the visual estimator
  does since item 19 (SLAM in the broad sense), on the thermal depth.
- **Stage 4. Flights in total darkness** with nothing lit: the point-to-point
  mission on the thermal set and the time-of-flight ring.

The navigation invariants hold throughout, and darkness keeps item 17's
meaning: for the visible cameras total darkness is observed unobservable, and
what this item adds is a sensor for which it is not.

### Measurement And Completion

Measure, per flight: the thermal features per frame and the stretches without
them; the estimate's error against truth and its drift over the path; the
confident range and the speed it admits; the minimum distance to true
occupancy; and collisions. Complete when five flights of the point-to-point
mission in total darkness, with no illumination and the time-of-flight
sensors the only emitters, reach the goal in truth without a collision; the
mean speed they fly is recorded as this sensor set's figure.

## 26. GPU Offload

**Type:** decision, recorded 2026-10-09: not now, not before a target
onboard computer exists.

**Hard prerequisites:** a target board, whose measurements decide.

On the GPU today runs the MPPI rollout alone, and it is the smallest part of
the planning tick. On the CPU: the stereo matching (SGM, two threads, 1.4
cores at 7.5 Hz: [resource_budget.md](resource_budget.md) says its place on a
vehicle is a GPU or a depth accelerator, and the container's OpenCV is built
without CUDA), the ESDF (three passes in the worker pool, then uploaded as a
texture for MPPI), the swept-body collision checks of the horizon assembly
(what dominates the tick, [performance.md](performance.md)), the VIO's
feature tracker (part of 0.53 cores), the obstacle memory's ray tracing (0.6
to 0.8 cores), the lidar-inertial registration (40 to 51 ms) and the D* Lite
planner (its whole 150 ms budget, a setting).

Why nothing moves now:

- On the workstation nothing on the CPU fails a requirement: the flights pass
  at their speeds, the onboard processes take 2.7 cores at p50 and the
  planner's budget is a configuration, not a shortage.
- The gain only matters on an onboard computer, and one of the Jetson class
  has a GPU several times weaker than the RTX 3060 that shares memory with the
  CPU: moving work blindly moves the shortage.
- Another stereo matcher gives another depth, and the confident depth of 6.4 m
  the braking contract stands on is re-measured with it.

The order when a board exists: the stereo matching first, through a ready
library (NVIDIA VPI on a Jetson), which frees a core and a half with almost no
code of the project's; then measure on the board what is the bottleneck; only
then the ESDF and the collision checks, which are the project's own CUDA and
real work. For the ESDF a ready alternative exists, nvblox (NVIDIA's GPU
TSDF and ESDF), which would need a layer of the project's own for the
unobservable state (F6); only if the board's measurement asks for it
([design_questions.md](design_questions.md)). The D* Lite planner, the memory's ray tracing and the lidar
registration stay on the CPU.

## 27. Readiness For A Real Vehicle

**Type:** new capability, deferred. The project owner on 2026-10-09: no
vehicle is coming soon, so nothing is adapted to hardware yet; the safety
block is recorded here so that it is the first thing done when one is.

**Hard prerequisites:** block 1 can be built and flown in the simulator
today; everything after it waits for hardware.

What carries over as it is: the onboard nodes (the obstacle memory, the
planner, MPPI, both estimators, the offboard) read no truth from the
simulator and have no dependency on Gazebo in their logic; `stereo_depth_node`
takes plain `sensor_msgs/Image` (the launch starts its Gazebo variant); the
IMU comes from PX4 over uXRCE, not from Gazebo; a real clock synchronisation
with PX4 exists (`Px4RosTimeMapper`) and is switched off; the sources carry
no x86 instructions; the way home, the light's judgment and the blind descent
do not depend on the simulator.

### Block 1: Safety

The one block that blocks any flight with people near, and the one that can
be closed in the simulator:

- **The pilot has no priority.** `mppi_offboard_node` arms and enters offboard
  by itself and resends both every 2 s (`auto_arm`, `auto_offboard`): a pilot
  switching the mode is switched back, a vehicle that PX4's failsafe landed
  and disarmed is armed again. A real vehicle needs the pilot's mode to win
  and arming to come from the pilot.
- **No reading of the radio and the kill switch**; of PX4's status only
  "armed" and "offboard" are read, its failure flags are ignored.
- **A defect in the planner-loss branch**, in the simulator too: the branch
  that holds the vehicle when the planner's heartbeat is lost returns before
  the tick publishes `OffboardControlMode` (`controlTick`), so the offboard
  stream stops, PX4 leaves offboard within its `COM_OF_LOSS_T` and its own
  failsafe acts before the branch's landing after 5 s. Repaired on
  2026-10-09: the mode is published before the branch returns.
- **No nominal landing and disarm**: a mission ends in a hold at the goal.
- **No operator's commands**: stop, land, home, a new goal.
- **The main battery is not read**; no geofence; the loss of the radio and of
  the link are not handled.
- **No PX4 parameter set for a real vehicle**: the simulation's switches the
  supply check off (`CBRK_SUPPLY_CHK`) and the data-link loss reaction off
  (`NAV_DLL_ACT 0`, `scripts/px4_parameter_runtime.sh`).

### Block 1 built and flown in the simulator (2026-10-11 and 12)

Every point above is answered (f3e25734; specification K29 to K32, K34 and
the amended A9):

- **The pilot's authority** (K29): the stack never arms the vehicle and
  never switches its mode; the offboard's auto-arm, auto-offboard and the
  resends are gone. In the simulation an evaluation pilot
  (`scripts/sim_pilot.py`) puts each vehicle in offboard and arms it once,
  with the commands a radio or a ground station sends, and plays a schedule
  of the pilot's and the operator's actions (`PILOT_EVENTS`). A mode the
  pilot chooses or PX4's failsafe wins; the offboard reports
  `AUTOPILOT_CONTROL` (released by the pilot, a failsafe or the autopilot's
  landing; resumed; terminated; disarmed) and never answers it.
- **The radio and the ground station** are read from PX4's failure flags and
  reported (`RC_LINK`, `GCS_LINK`); PX4's own reaction acts.
- **Every mission ends landed and disarmed** (K30): after the goal or the
  return the vehicle holds its position and descends until PX4 lands and
  disarms it; the mission check asks for `MISSION_LANDED disarmed=true`.
- **The operator's commands** (K31) on `/drone_city_nav/operator_command`:
  stop, land, home, goal X Y Z.
- **The main battery** (K32): PX4's battery status weighed against the way
  home with the return's margin (K11); a drained simulated battery turns
  the vehicle home (`MAIN_BATTERY_DRAIN_S`).
- **A PX4 parameter set for a real vehicle** (K34):
  `drone_city_nav/config/px4/real_vehicle.params` with
  [px4_parameters.md](px4_parameters.md); a geofence is in it as an example,
  which PX4 evaluates on a global position a GNSS-denied flight does not
  have.

The injections, one flight each on f3e25734:

| Injection | PX4 | The stack | End |
| --- | --- | --- | --- |
| The pilot's Hold (r1318) | refused: Hold needs a global position | nothing to answer | flew on |
| The pilot's Position mode with a radio (r1326) | POSCTL in 36 ms | released, pilot | held by PX4 |
| The pilot's Land (r1319) | AUTO_LAND in 0.1 s | released, autopilot's landing | landed |
| The pilot's kill (r1320) | disarmed in 60 ms | terminated, never armed again | fell |
| The radio's loss (r1321) | its failsafe, AUTO_LAND in 0.6 s | RC_LINK lost; released, failsafe | landed |
| The operator's stop, then a goal (r1322) | offboard | still in 1.5 s from 2.3 m/s; the goal a new objective | flew to it |
| The operator's home (r1323) | offboard | the way home along the trail | landed 0.33 m from the start, disarmed |
| The operator's land (r1324) | offboard | the landing where it stood | landed, disarmed |
| The main battery drained over 300 s (r1325) | offboard | home at 130 s, the way home 180 s with the margin over the time left | landed 0.24 m from the start, disarmed |

Found on the way, none of them the stack's: a GNSS-denied PX4 refuses Hold
(it needs a global position), so the pilot's means there are the Position
mode with a radio, Land and the kill; the pilot's Land onto the staging
platform's edge left PX4's land detector without a landing for the rest of
the window (the vehicle held a quarter of its thrust on the edge); a goal
given straight at the start is sought through the unknown and the start's
room opens to the east, so the vehicle sought it far round, where the home
command retraces the trail.

### Later blocks, for when a vehicle exists

2. Build and launch: the package needs Gazebo to build (`gz-sim8`,
   `gz-transport13`, `sdformat14` in CMake) and the onboard and simulation
   targets are one; the image is x86 with CUDA architectures 75 and 86; there
   is no launch without the simulator and `use_sim_time` is set in every
   node and held by a contract test; the uXRCE agent is UDP only; MPPI keeps
   12 threads.
3. Sensors: the pair is configured by its calibration, distortion and
   rectification included (specification K33, run on EuRoC's 752 x 480
   pair offline), but there is no camera driver and the pair's stamps are
   taken as equal; no time-of-flight driver (64 zones in `gpu_lidar`'s
   layout, within 67 ms of the frame); the barometer, the light's charge
   and the camera's gain come from the simulator; no onboard control of the
   light; the lidar needs an organised 360 x 181 cloud without motion
   compensation (writing the compensation or adopting FAST-LIO2 is decided
   on a real recording, item 25); the extrinsics live in SDF, C++, YAML and
   Python at once.
4. Time and frames: the real clock synchronisation (`UXRCE_DDS_SYNCT=0`
   today, zero EKF2 delays, the 4 ms frame-to-IMU offset fitted in the
   simulator; before real cameras the frames' time offset and the
   camera-to-IMU extrinsics are to be estimated online, as OpenVINS and
   VINS-Fusion do); the map is anchored to Gazebo's spawn (origin 54, 54; the
   345 x 525 x 40 m grid of the Urban Circuit); the initial heading is a
   parameter.
5. The vehicle model: every constant is fitted to the x500 in Gazebo
   (accelerations, lags, the body, the rotor drag 0.106, the yaw-rate lead
   marked to fit again); a real frame is identified by hand first. The
   airframe itself is open (specification V7): the x500 is where the model
   was fitted, not what the vehicle must be.
6. Checks before a flight: no CI, no HITL, no replay of the whole stack from
   a record; onboard logs without rotation; the bag without setpoints and
   commands; crash detection rests on Gazebo's contacts, and the trial's
   referee shares a class with the mission's logic.

### The arm64 Build (block 2, weighed on 2026-10-13)

Needed for any real vehicle but one on an x86 board: every flight computer
of the hardware registry is arm64 (the Jetson Orin and Thor, the VOXL 2, the
RK3588), and the x86 boards are the heavy ones, which a product made in
numbers weighs against (specification R6, [hardware_builds.md](hardware_builds.md)).

What already holds: the sources carry no x86 instructions (no intrinsics,
no `-march` or SIMD flags; checked on 2026-10-13); the image's base,
`px4io/px4-dev`, is built for amd64 and arm64 on Ubuntu 24.04
([PX4's container documentation](https://docs.px4.io/main/en/test_and_ci/docker.html));
ROS 2 Jazzy ships for arm64; and since JetPack 7.2 (June 2026) the Orin runs
Ubuntu 24.04 with CUDA 13, the image's distribution
([JetPack 7.2](https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2)).

What is missing, in order:

1. An onboard build without Gazebo. **Built on 2026-10-25** (the root goal
   of 2026-10-24, part 3): `DRONE_CITY_NAV_ENABLE_SIMULATION` guards
   `gz-sim8`, `gz-transport13`, `gz-msgs10`, `gz-common5`, `sdformat14` and
   `ros_gz_interfaces` and the targets that use them (the contact system,
   the truth adapter, the smoke relay, the Gazebo stereo process, the
   heading source, the crash node, the SDF voxelizer); the Dockerfile's
   `onboard` stage carries no Gazebo, and `tools/onboard_image.sh` builds
   the package in it with the option off and runs the unit tests
   ([build_and_run.md](build_and_run.md)): amd64: the image 2.35 GB with 0 Gazebo packages and CUDA 13.0; the package with the option off built in 6 min and its 154 unit tests passed (the seven CUDA ones on the host's GPU).
2. The Orin's CUDA architecture, 87, in `DRONE_CITY_NAV_CUDA_ARCHITECTURES`
   and a build against CUDA 13. **Built on 2026-10-25:** `75;86;87`; the
   onboard stage installs CUDA 13.0 (`cuda-compiler-13-0`,
   `cuda-cudart-dev-13-0`) and the MPPI's tests pass on the host's GPU from
   it; the development image stays on 12.6.
3. An arm64 image and the unit tests run on arm64 under emulation. **Built
   on 2026-10-25:** `tools/onboard_image.sh --arm64` (qemu-user-static,
   docker buildx, CUDA off): arm64 under emulation: the stage 1.75 GB (77 min to build, px4_msgs's generation the long part), the package in 1 h 2 min on 16 threads, its warnings a subset of the amd64 build's, 140 of 142 unit tests passed (139 at first; the estimator test's ctest timeout raised from 60 to 600 s, its 4.7 s running past a minute at the emulation's speed); the two left, the D* Lite repair's share of a 20 ms budget and the initial search's certification within its budget, assume the development host's speed and are not x86 dependencies, of which none was found.
4. The stack on recorded data on an arm64 board, the Orin Nano Super
   developer kit ($399, the registry's bench): the cores, the tick and the
   rollouts the GPU carries, measured; the first number the registry and
   item 28 wait for.

Steps 1 to 3 need no vehicle and no board beyond an arm64 machine; step 4
needs the bench board. An x86 board spares the port at a cost in mass.

The hardware a vehicle could be built from, what fits and why, and what it
costs: [hardware_builds.md](hardware_builds.md).

Order: block 1 in the simulator; the build split, the arm64 build and a
launch on recorded data; cameras on a bench; HITL with a real Pixhawk; tethered flights in a
net, manual first, then a hold, then a short route. Months of one person's
work; the first two steps cost nothing.

## 28. The MPPI Engine Beyond CUDA

**Type:** new capability, deferred; weighed on 2026-10-13 at the project
owner's question (can the stack do without CUDA, or carry two backends), no
decision taken.

**Hard prerequisites:** none for step 1 (flights in the simulator); a chosen
onboard computer without an NVIDIA GPU for step 3.

**Where CUDA is.** In the MPPI engine alone: six files in `src/mppi/`
(`*.cu`, `*.cuh`, about 3 400 lines) and about twelve kernels: the sampling
noise, the rollouts against the ESDF, the reductions of costs and weights,
the control update. Nothing else of the stack uses CUDA. The boundary is one
class, `MppiCudaEngine` (`updateEsdf`, `plan`), with one consumer,
`MppiController3D`, so another implementation can stand in its place without
touching the rest. Today the planner is not built at all without CUDA
(`production_mppi_node` sits under `DRONE_CITY_NAV_ENABLE_CUDA_MPPI`), so the
stack does not fly without it. Two things make a port easy: the ESDF is read
through a texture with point sampling, not the hardware's interpolation, so
a lookup off the GPU gives the same value; the noise is a counter hash, not
cuRAND. Two make it harder: the kernels' functions are `__device__` only (a
host-and-device macro exists for two helpers), and the CPU reference
(`simulateReference`), with which the engine re-simulates the sequence it
selected, is a second implementation of the same model, not the kernel's code.

**What a CPU costs, measured.** One rollout of the CPU reference, 120 steps
(the 6 s static horizon), the urban body (0.82 m envelope, 12 x 2 x 3
samples, the broad phase on), a 0.25 m ESDF of 40 x 40 x 16 m
(`drone_city_nav/tools/mppi_cpu_rollout_benchmark.cpp`, built with the
package, on the development host's Zen 3 with the host quiet; a first
measurement under another task's load read 101 and 1151 us):

| | Open space | A corridor (walls 1.5 m either side, the floor 1.5 m below) |
|---|---|---|
| One rollout on one core | 61 us | 850 us |
| Rollouts eight threads finish in 8 ms | about 1000 | about 73 |
| On the GPU today (RTX 3060 Laptop) | 8192 in 6.4 ms | 8192 in 6.4 ms |

A gap of 8 to 110 times, the corridor's the one that counts: the routes run
within 2 m of a wall half the time (roadmap item 30). The reference is not
written for speed and a port of the kernel with SIMD would be several times
faster, but an Arm core (Cortex-A78) does about half of a Zen 3 core, and
the rest of the stack already takes 3.4 to 5.7 cores at p95
([resource_budget.md](resource_budget.md)): a CPU in flight would carry tens
to a few hundred rollouts, another controller, tuned again from the start.

**What a Jetson costs too.** The Orin NX 16GB's GPU does 1.9 to 2.4 TFLOPS
in FP32 (1024 CUDA cores at 918 to 1173 MHz), the RTX 3060 Laptop about 11:
the 6.4 ms of rollouts become 30 to 40 ms against the 20 ms tick, and the
rollouts (`rollouts`, 8192) fall to about 1500 to 2000 on the Jetson the
stack is sized for ([hardware_builds.md](hardware_builds.md)). How the
controller flies with fewer rollouts is therefore needed whatever the
vendor, and it is the same number that says whether a weaker GPU of another
vendor is enough.

The ways, weighed:

| Way | Runs on | Size | For flight |
|---|---|---|---|
| A. A CPU backend: the kernels' bodies as shared host-and-device functions, the rollouts in a loop over threads | anything | M: one to two weeks, with the parity tests against the GPU | No (the table above). For continuous integration, development on a machine without NVIDIA and offline replays; the rollout model becomes one code instead of two |
| B. A second backend in Vulkan compute or OpenCL | Vulkan: NVIDIA (the Jetson too), Qualcomm Adreno, Arm Mali, Intel and AMD integrated GPUs; OpenCL: the same but the Jetson, which ships no OpenCL | L to XL: about 1 300 lines of kernels again, the host side, parity tests, a board with such a GPU to test on | Yes, where the GPU is strong enough |
| C. Vulkan in CUDA's place | every GPU, one code | XL: on the Jetson likely slower and harder to debug; the controller's core, long tuned, at risk | Yes |
| D. SYCL or Kokkos, one C++ source | NVIDIA, AMD, Intel, CPUs | L | The mobile GPUs (Adreno, Mali) are poorly served, so the main case, the VOXL 2, stays closed |

What a backend without CUDA would give: a free choice of vendor (the
ModalAI VOXL 2 above all: the autopilot on board, 16 g, $1 250 with it,
against an Orin NX at $999 since July 2026 plus a carrier board and a
Pixhawk), independence from NVIDIA's prices, and continuous integration
without NVIDIA (way A). Counted build by build it saves 5 to 20 percent of
a vehicle, simplifies the hardware only on the VOXL 2 and the software
nowhere ([hardware_builds.md](hardware_builds.md), "Cheaper Or Simpler
Without CUDA?"); the vehicle is meant to be made in numbers
(specification R6), so a saving per vehicle counts against a fleet and the
backend is a candidate, not a curiosity; the larger lever is the stack's own
appetite for compute, which step 1 starts to measure. What it would not:
the rest of the stack needs 3.4
to 5.7 Zen 3 cores at p95 whatever runs MPPI, so on boards with four large
Arm cores (Raspberry Pi 5, RK3588, the VOXL 2) the CPU becomes the limit
once MPPI has moved to their GPU. The boards that would fit are in
[hardware_builds.md](hardware_builds.md).

The order:

1. Fly the controller at 8192, 4096, 2048 and 1024 rollouts in the
   simulator (the parameter exists; camera and lidar series): the number the
   Jetson needs anyway (item 27) and the one that decides whether a GPU of
   the Iris Xe or Adreno class is enough. With it, the onboard budget
   emulated on this host before any board (the discussion of 2026-10-19):
   the onboard processes confined to a core set of the Orin's class (four
   to six logical cores, `taskset` or a cgroup), the simulator and the
   autopilot on the rest; the launcher's subsystem masks of today
   (`ENABLE_SUBSYSTEM_CPU_AFFINITY`, control 0 to 7, planning 4 to 15,
   diagnostics 12 to 15 of 16) overlap and exclude nothing. The same
   confinement is what would let a flight share the host with the owner's
   other work (A4, A7): on the 8 physical cores the camera flight takes
   7.6 cores of load at the median (r2070) and keeps a real-time factor of
   0.85 to 1.00, 3 to 18 percent above A7's floor, so a browser's two or
   three threads void it today (r2074, r2075, r2078 at 0.78 to 0.81).
2. Way A when continuous integration or development without NVIDIA is
   wanted; not a way to flight.
3. Way B only for a chosen board without NVIDIA, the VOXL 2 most likely,
   inside item 27; CUDA stays the backend of the Jetson. **The owner's
   emphasis of 2026-10-24:** the board way B is built for is the VOXL 2, as
   the support of a possible entry into the United States government
   market, where the rules on parts (NDAA section 848, the Blue UAS lists)
   exclude Chinese-made flight controllers, cameras, radios and lidars
   (Holybro's Pixhawk, Arducam's pair, Livox, Unitree, Rockchip) and the
   VOXL 2 covers the computer, the autopilot and the cameras in one listed
   board. The Jetson itself is not excluded (a compute module of a United
   States company); what a Jetson build would have to replace for that
   market is its autopilot and its cameras. The market is still not set
   (specification R6); this is an emphasis, not a decision to build way B
   now, and the lists are to be checked against the buyer's rules when the
   market is chosen. What the board would take in full is item 37.

**Step 1 done on 2026-10-26** (the root goal of 2026-10-24, part 3;
specification K48, `MPPI_ROLLOUTS` sets the count for a flight): eight
groups of five clear flights, r2191 to r2234, no contact in forty.

| Set | Rollouts | Mean speed, m/s (min) | B by the truth, max | Holds, % | Tick p50 / p95, ms | GPU, % |
|---|---|---|---|---|---|---|
| stereo | 8192 | 1.390 (1.333) | 1.16 | 0.90 | 22.7 / 29.6 | 34 |
| stereo | 4096 | 1.380 (1.324) | 1.94 | 0.82 | 22.5 / 29.3 | 33 |
| stereo | 2048 | 1.411 (1.349) | 1.38 | 0.96 | 22.6 / 29.7 | 32 |
| stereo | 1024 | 1.418 (1.326) | 1.13 | 0.89 | 21.6 / 28.5 | 32 |
| lidar | 8192 | 2.375 (2.143) | 0.64 | 2.4 to 2.7 | 25.1 / 33.2 | 39 |
| lidar | 4096 | 2.424 (2.283) | 0.74 | 2.1 to 2.7 | 24.3 / 32.1 | 37 |
| lidar | 2048 | 2.375 (2.188) | 0.49 | 1.8 to 2.8 | 24.2 / 32.3 | 37 |
| lidar | 1024 | 2.192 (1.900) | 0.77 | 2.3 to 2.9 | 23.8 / 31.4 | 35 |

The stereo set's speed and arrival hold at 1024 as at 8192; the lidar's
hold to 2048 and lose 8 percent at 1024, one flight under R3. The true clearance of the modelled body (0.55 m, larger than the airframe) in the middle of the flight, the pad and the landing left out: its least value touches 0.00 in every group, the 8192 ones included (a graze of the modelled body, no contact), so the clause that tells the groups apart is the seconds under 0.10 m, and it is set by one flight in five: the 8192 groups' most 1.3 s (stereo) and 0.8 s (lidar); stereo 4096 3.6 s (r2208), 2048 0.7 s, 1024 5.3 s (r2229); lidar 4096 3.7 s (r2212), 2048 3.1 s (r2220, r2222), 1024 2.5 s (r2231). By the rule read as written the stereo set's count is 2048 (1024 fails the clearance clause by one flight) and the lidar's stays 8192 (every lower count fails it by one or two flights while keeping the speed to 2048); the means of the seconds, 0.2 to 1.1 s on the stereo set and 0.5 to 1.2 s on the lidar at every count, do not separate the counts, and ten flights a group would be needed to. The tick on the development host
barely follows the count: the MPPI's GPU share is the 6.4 ms at 8192
measured above, the rest of the tick is the CPU's; so on this host the count
is no lever, and on an Orin, where the 6.4 ms become 30 to 40, it is the
lever: 2048 rollouts there cost about what 8192 cost here; the counts
by the rule are 2048 for the stereo set and 8192 for the lidar
([hardware_builds.md](hardware_builds.md), specification K48).

Complete, step by step: (1) the series at the four counts with the mean
speed, the holds and the closest true clearance against 8192 (done, above); (2) the engine
built and its unit tests run on a machine without CUDA, the parity tests
holding the CPU backend's costs to the GPU's on the same noise; (3) a flight
series on the board.

**Step 2, way A, done on 2026-10-30** (the goal of 2026-10-29, part 3;
code f60c2928 to fa16e3af; specification K54). The rollout model is one
host-and-device body, `src/mppi/mppi_rollout_body.hpp` (the sampled noise,
the dynamics, the body's ESDF query through a lookup functor, the costs of a
rollout, the sequence limiter); the CUDA kernels are shells around it (the
texture read, the thread indexing and the block reductions stay the device's)
and the CUDA path flies as before (r2483: the tick 22.7 / 32.0 ms against
K53's 23.1 / 31.1). `MppiCpuEngine` runs the same stages on the host in plain
loops behind the engine interface `MppiEngine`; the controller makes the
engine by `mppi_engine` (cuda, the default, or cpu; `MPPI_ENGINE` of a flight).
The host library `drone_city_nav_mppi` and the planner's component build
with `DRONE_CITY_NAV_ENABLE_CUDA_MPPI=OFF` (156 unit tests there, the
controller's lifecycle test on the CPU engine); the parity test holds the
two engines to the measured spread on the same inputs: 3.4e-6 m/s^2 on the
controls, 2.4e-6 relative on the weight sum and the sample fraction, 1.7e-6
on the selected cost (bounds 1e-5). The CPU engine in flight at 256 rollouts
(r2482, cameras): 1.474 m/s, B 1.48 m, no contact; the tick 113.6 / 144.4 ms
(the MPPI update 98.9 / 127.2 ms, 0.39 ms a rollout of 80 steps on one
thread), the planner node 1.90 cores against 1.48 on CUDA: a number, not the
flight's engine. Not in the CPU engine: the cooperative candidates and the
separation acquisition of item 15's missions. The host evaluator
`simulateReference` stays what it was (it reads the footprint through the
known-clearance evidence and tracks peer separation, which the sampler's body
does not): the one body is the sampler's, on both backends.
The arm64 onboard image (item 27 block 2) rebuilt with the CPU engine as the
node's engine: 154 of 156 unit tests under qemu on 2026-10-30 (3b5746c3), the
two failures those of 2026-10-26 (`log/tools/board/onboard_arm64.log`).

**Step 2, way B, done on 2026-10-30** (the same goal, part 3; code baca86d5
and 4de27c60). Vulkan compute runs in the dev container after one change
(`scripts/container_run.sh`: the driver's graphics capability and the host's
ICD file mounted where the loader looks; the toolkit already injects
`libGLX_nvidia` with the GPU): `vulkan_probe` sees the RTX 3060 (API 1.4,
two compute families, 1024 invocations a work group, 48 KB shared, 64-bit
integers), and `glslc` was in the image. The engine: the rollout body once
more in GLSL (`src/mppi/shaders/mppi_rollout.comp`, every law of
`mppi_rollout_body.hpp` line for line, since a shader cannot share C++; the
ESDF a storage buffer read by cell), the noise (`mppi_noise.comp`, the body's
hash on 64-bit integers) and the weighted update (`mppi_update.comp`, a work
group a step), compiled by glslc at build time and carried in the library as
SPIR-V words (`cmake/embed_spirv.cmake`); a small compute runtime
(`src/mppi/vulkan_compute.hpp`); the CPU engine's tick on a population device
(`src/mppi/mppi_population_device.hpp`: the host loops, or the Vulkan device
with the noise, the rollouts and the weighted sum on the GPU and the
reductions, the repair and the evaluation on the host, one code);
`MppiVulkanEngine` behind `mppi_engine` and `MPPI_ENGINE=vulkan`, built where
the loader and glslc are (`DRONE_CITY_NAV_ENABLE_VULKAN_MPPI`). Parity
(`tests/mppi_vulkan_engine_parity_test.cpp`, a wall and a pillar, without and
with a route and the route-directed candidate): against the CPU engine
4.4e-6 m/s^2 on the controls, 2.4e-6 and 3.1e-6 relative on the weights and
the cost; against the CUDA engine 1.4e-6, 1.4e-6 and 8.1e-7; the noise alone
8.7e-6 against `rollout::noiseSample` (bounds 1e-5 and 1e-4, the measured
spread, never loosened). The benchmark (`mppi_engine_benchmark`, the corridor
scene, 120 steps, the urban body, RTX 3060; `log/tools/ilv/part3/vulkan_bench.txt`):
the population's milliseconds p50 / p95 at 8192, 4096, 2048 and 1024 rollouts
are CUDA 6.8 / 7.4, 6.2 / 6.9, 6.2 / 6.7, 6.0 / 6.8; Vulkan 10.8 / 11.5,
9.4 / 10.2, 8.9 / 9.9, 8.7 / 9.7; CPU 3143 / 3191, 1563 / 1601, 776 / 805,
397 / 404: Vulkan 1.4 to 1.6 times CUDA on the same GPU (the storage-buffer
ESDF against CUDA's texture, no fast math, a fence wait a dispatch). In flight
(r2484 to r2494, the acceptance location under the imperfections profile):
the lidar at 8192 five of five, no contact, 2.197 to 2.689 m/s, B 0.33 to
1.40 m; the cameras at 2048 (K48's rule) five of five without contact, the
speed 1.183 to 1.296 m/s, B 0.91 to 3.60 m, two B misses and one speed under
R3, the estimate's heading drift under the profile (the frame turn -2.5 and
-2.1 degrees on the misses against 0.2 to 1.2 on the hits; item 35's register
row), the same rate as on CUDA (B 4 of 25, R3 10 of 25); the clean vehicle on
the cameras (r2497, r2498, r2505) 1.190, 1.209, 1.424 m/s, B 0.70 to 1.89 m,
no contact. The planner's tick on Vulkan is 25.4 / 34.4 ms on the cameras
against CUDA's 22.7 / 32.0 and 31.5 to 32.7 / 65.8 to 78.2 on the lidar
against 25.9 to 27.3 / 32.8 to 58.8; the planner's process takes less CPU
(1.31 against 1.55 cores: no CUDA spin on the host); the flight's real-time
factor of the counted flights the same (p50 0.84 to 0.94 against 0.74 to
0.98), the GPU 7 to 9 points busier (37 to 40 percent against 29 to 35 on
the cameras, 43 to 48 against 34 to 39 on the lidar); six camera attempts of
the clean series were voided by the real-time factor, under a foreign host
load in part. The stage's flight clauses for the stereo set under the profile
(B within 2 m, R3 at or above 1.2 m/s in every flight) are not met and are
closed by record: the two B misses and the speed miss are the estimate's
heading drift under the imperfections profile (the register's row of item
35, class (b)), which no engine touches (the parity 1.4e-6) and which CUDA
shows at the same rate (part 1: B 4 of 25, R3 10 of 25; part 2's base: B 2
of 5, R3 3 of 5); the lidar set, whose estimate registers against its map,
passes five of five on Vulkan as on CUDA. The ways out are the register
row's: a heading reference where one exists, registration of the depth
against the long-lived map along the way, or online camera-to-IMU rotation
and bias; until one is built the stereo set's acceptance under the profile
is not guaranteed on any engine. The clean vehicle, which judges an engine
alone (K48's camera flights were clean), was flown the same day on both:
eight camera flights on Vulkan at 2048 (r2497, r2498, r2505, r2507, r2509,
r2511, r2512, r2513) 1.190, 1.209, 1.424, 1.242, 1.197, 1.144, 1.099 and
1.280 m/s (mean 1.223, four under R3), B 0.42 to 1.89 m, no contact; five on
CUDA (r2514 to r2517, r2520) 1.397, 1.395, 1.192, 1.278 and 1.394 (mean
1.331, one under R3), B 0.68 to 1.09 m, no contact. R3 for the stereo set
at 1.2 m/s is marginal on both engines on this day's host (K48's 2048 group
gave 1.418 on 2026-10-26, part 1's clean base 1.248 to 1.345), and the 8
percent between the engines is suggestive, not proven (eight against five
flights, a spread of 0.10 m/s each; the paths part by the noise's last bits
as the twins of item 36 do): the holds are alike (0.5 to 2.0 against 0.3 to
1.0 percent), the sensor-braking limiter rules both, the route-clearance
limit's median sits lower on Vulkan (1.3 to 3.0 against 2.0 to 2.7 m/s),
the planner's tick 25 to 27 against 22 to 23 ms, the GPU 34 to 40 against
27 to 32 percent. The stage's camera clauses are therefore closed by record
with these numbers; the ways to settle the 8 percent are a longer series on
a quiet host, the tick's cost taken out of the Vulkan engine (the debt row:
an `image3D`, a timeline semaphore), and the board itself, where no camera
rendering shares the GPU.
Not done: the Adreno 650's rollout count (no board; `shaderInt64`
unconfirmed there), the cooperative candidates (the CPU engine's limits).

## 30. The Airframe's Size

**Type:** new capability, deferred; weighed on 2026-10-13 at the project
owner's question, no decision taken. The airframe is open (specification
V7) and the vehicle is meant to be made in numbers (R6).

**Hard prerequisites:** none for step 1; a chosen airframe class for step 2,
which item 27's hardware (the computer, the sensors) has to fit.

The x500, chosen at the project's start, is a development platform: 500 mm,
about 2 kg built, about 18 min of flight without payload. Half of every
flight runs within 2 m of a wall and a fifth within 1.5 m; at the narrowest
points flown a wall stands 0.47 to 0.88 m from the body's axis, where the
stack's body (0.55 m) and envelope (0.82 m) are already in evidence and the
clearance laws hold the vehicle slow. By a what-if on the same flights a
body smaller by 0.1 m would fly the cameras 16 to 23 percent faster and the
lidar 31 to 43 percent, an optimistic bound
([design_questions.md](design_questions.md), "Is The x500 The Right
Vehicle?").

The order:

1. The body the stack models against the airframe it flies: 0.55 m against
   the model's rotor tips at 0.39 m. The 0.16 m between them is today's only
   horizontal margin against the estimate's drift, so a horizontal
   allowance of its own (as K27 is the vertical one) comes first, then the
   body at its true size, then the camera and lidar series. The first tenth
   of a metre of the gain, with no new airframe
   ([technical_debt.md](technical_debt.md)).
2. An airframe class chosen with item 27's hardware: the computer and the
   sensors it must carry (the 220 to 320 mm class needs a computer of the
   VOXL 2's mass, [hardware_builds.md](hardware_builds.md)).
3. Its Gazebo model with its real masses, the sensors' included (they weigh
   nothing in the x500 model today), and its PX4 airframe: geometry, gains,
   hover thrust.
4. Every constant fitted to the x500 fitted again: the accelerations of the
   speed profile (V3), the lags, the rotor drag (0.106, K13), the yaw-rate
   lead, the tube's 0.075 s, the rest clearance (0.35 m), the lean law, the
   time-of-flight reflex's distances (K19), the body and the envelope, the
   landing speeds (K19, K30); the sensors' extrinsics, which live in SDF,
   C++, YAML and Python at once; the model's name in the scripts and the
   crash judge.
5. Every acceptance series and scenario again.

Size: step 1 is M (a design, a change and the series); steps 2 to 5 are L to
XL.

Complete when the chosen airframe flies the camera and the lidar series and
the light scenarios with no contact, its speeds measured against the x500's.

## 33. Held-Out Worlds: Point-To-Point Flights On The Cave Circuit And The Finals World

**Type:** validation without a vehicle, general; the first of the directions
of the discussion of 2026-10-19 with the project owner (recorded 2026-10-24
at the owner's word, with items 34 and 35).

**Hard prerequisites:** none. The two worlds are versioned release artifacts
and load as they are ([environment_candidates.md](environment_candidates.md),
[scenarios.md](scenarios.md)); the flights need no static map (the no-static
ESDF of item 12), so item 11 is not needed for them.

**Validation environment:** `cave_circuit_practice_01` and
`finals_prize_round_world_07`, never flown. `urban_circuit_practice_01` is
the learning set; the dark hall is the only other world flown, and it is
small.

The risk this item answers: every flight of the project has been flown on
one location, and the specification's forty K rules are each justified by
flights there. Whether they generalise or fit the location cannot be told
from the location itself; the register has carried this as N2 since
2026-09-26. The held-out test is the plain one: fly the other worlds with
nothing changed for them, and read what breaks.

The rules of the test:

1. **Nothing adapted.** No parameter, threshold or scenario constant is
   changed for a held-out world before its flights; the urban scenario's
   location constants that exist today are named and kept out of these
   scenarios (the route volume and the snapshot bounds of the Makefile's
   urban targets, the smoke zones of F4, the urban start and goal). What a
   held-out world then needs is a finding, not a tuning.
2. **The scenarios.** Point-to-point scenarios in the v2 schema (as
   `dark_hall_point_to_point_scenario.json`), the start and the goal drawn
   with a recorded seed from the reachable free component of each world's
   truth voxelization (the flood of K16 on the truth grid with the body's
   radius), the route's length in the urban mission's band (300 to 600 m),
   the goal where the light's reach makes sense for the camera set; the
   pairs are written into the scenario files, so a flight is reproducible.
   A Makefile target a world, headless, both sensor sets.
3. **The series.** Five single flights a world and a sensor set, through
   `tools/fly_until_valid.sh` (A4, A7): twenty flights. R1 and R2 are the
   gates (no contact; B in truth within 2 m); the mean speed is measured and
   written, not gated: R3's numbers are the urban mission's, and a number a
   held-out world cannot reach is a finding about the rule that holds it.
4. **Every failure read to its rule.** A flight that fails names the K rule
   or the law that failed it, and the rule is classified: a defect that
   would fail anywhere is repaired, on all worlds, and the urban series flown
   again (the acceptance rule); a rule that is the urban location's goes to
   the register with its class and the owner's call. No rule is relaxed on
   the held-out world to make the flight pass.
5. **The truth grids.** The 0.5 m Occupancy3D of each world exists
   (environment_candidates.md's table); the body's true clearance
   (`tools/body_truth_clearance.py`) wants 0.25 m, built as
   `log/tools/truth25` was, if a flight's contact or crawl needs it.

Size: S for the scenarios and the truth grids; the flights about three
hours a sensor set with the voids; M with the reading of the failures.

This item is complete when the twenty flights are flown and tabulated (the
world, the sensor set, the result by R1 and R2, the speed, the true position
at B, the closest true clearance), every failure has its rule named and
either repaired or in the register, and N2 is closed.

## 34. The Survey Mission: Explore And Come Back With The Map

**Type:** new capability, a mission layer over the single planner; the rule
it obeys is specification F20 (the owner's, 2026-10-19, recorded
2026-10-24). The second of the directions of 2026-10-19.

**Hard prerequisites:** item 33 (the held-out worlds, which the survey then
covers whole); item 19's proof (K16) for the end of the survey; K10 (the
memory does not forget) for the map. Item 15's shared memory is needed only
when several vehicles divide a survey, later.

**Validation environment:** Urban Circuit Practice 01 first, surveyed from
the start without a goal; then one held-out world of item 33.

What it is. A frontier is the boundary between the free space the sensor has
observed and the unknown. The survey flies to a worthwhile frontier, looks,
chooses the next, until the reachable space has no frontier left within the
mission's volume, and comes home along its own track with the map. It is a
layer that chooses goals for the planner that exists (`NavigationObjective`,
the way the return home of item 19 hands the planner its goal); it is not a
second planner and not a second producer of routes (item 12).

What it gives: the product's mission in a building nobody has a map of
(the operator does not know B's coordinates; "fly in and bring the 3D map
back" is the work of the indoor vehicles without GNSS), the map as the
result of the work (the obstacle memory already builds it; item 11 persists
it), a test of generalisation on a whole world instead of one 400 m route
(the coverage counted against the truth grid), long flights that exercise
the drift of a first pass (L1) and the relocalization against the vehicle's
own map, and the ground for several vehicles (item 15). What it does not
give: a better flight from A to B.

What exists: the planner and its route repair, the goal handed through
`NavigationObjective`, the return along the own track (item 19), the flood
of K16 that says what is reachable, the obstacle memory with no forgetting.
What is to be written: the frontiers out of the voxel memory, the choice of
the next one (by the planner's own route cost, for instance), the coverage
metric, the end condition, the mission's state.

The rules (F20): nothing in transit to any goal prefers unknown or
unvisited space or a frontier (I1 unchanged); a frontier disappears only by
measurement, when the sensor has looked at it or when it lies outside the
reachable component by the flood of K16, never because it was "tried"
(I3); no second producer of routes beside the planner.

The open questions, to be settled when the item starts, the wording the
owner's:

- **The survey's volume.** Unknown space is free (I1), so frontiers exist
  everywhere, above the ceiling's height and beyond the walls. The volume a
  survey covers is the mission's order, as a goal point is, not a
  prohibition of the configuration: the vehicle may route outside it and
  seeks frontiers inside it. The agent's reading; the owner words it.
- **The end on a whole location.** Item 24 proved the flood on a corridor
  closed by smoke; on a whole location it did not close before, when the
  memory still forgot; the forgetting is off (K10) and the closure on the
  location's scale is unmeasured.
- **The value of a frontier.** The route's cost from the planner, the
  size of the unknown behind it, or the plainest rule that surveys the
  urban location whole in the fewest metres: measured, not chosen.

The order:

1. **Frontiers and coverage, offline.** The frontiers out of a raw snapshot
   of the memory (`raw_snapshot_3d_*.json` of any flight) and the coverage
   against the truth grid, as a tool under `tools/`, read on the recorded
   urban flights first.
2. **The mission.** The frontier chosen, handed as the goal, the next one
   when it is observed or unreachable, the end by the flood, home by the
   own track; a scenario (`sim-urban-survey-headless`) with no B.
3. **Acceptance.** Five flights a sensor set on the urban location: no
   contact, home within 2 m in truth, the coverage of the reachable truth
   within the sensor's reach above a figure measured in step 1 and written
   into the specification, the mean speed recorded; then the same on one
   held-out world. The map persisted as item 11 asks.

Size: M for steps 1 and 2, L with the series.

This item is complete when the urban location is surveyed whole in five of
five flights a sensor set with no contact and the vehicle home, the
coverage figure stands in the specification, one held-out world is
surveyed, and the survey's rules are in force as F20 says.

## 35. The Real Vehicle's Imperfections In The Loop

**Type:** realism stage, general; the third of the directions of
2026-10-19. As the carried light's flicker runs in every acceptance flight
(F1), a profile of the real vehicle's imperfections with a recorded seed is
to run in every acceptance flight once it lands.

**Hard prerequisites:** item 25 for the magnitudes (the IMU's noise and
the camera-to-IMU time offset come from a real recording's calibration
already; the lidar's from the lidar recording of item 25's second half);
the pair's noise (A12) and the rotors' drag (K21) are the two imperfections
already in force. Items 33 and 34 before it, by the discussion's order.

**Validation environment:** Urban Circuit Practice 01, the acceptance
series under the profile; the held-out worlds once item 33 has flown them.

The imperfections, each a measurement the stack reads today in its perfect
form:

- the rotors' vibration on the IMU (K21 fuses the rotors' drag straight
  from the accelerometer, so the vibration tests that fusion first);
- errors of the extrinsics: the mount error of 0.24 m lived through many
  series unnoticed (K22); a flight's extrinsics perturbed within the
  tolerance a real mount has, millimetres and tenths of a degree, the stack
  reading the nominal values;
- the camera-to-IMU time offset and its jitter, and the drift of the IMU's
  biases (a random walk on the biases the estimators carry);
- the aerodynamics near walls and the ceiling (half of a flight is within
  2 m of a wall, item 30) and the spread of thrust and mass between
  vehicles.

The magnitudes come from measurements (item 25's recordings, the sensors'
data sheets), never from a guess; each is written into the specification
with its source before it runs, and the mean speed of R3 is measured under
the profile, as under the flicker. An imperfection that fails the
acceptance is a finding about the stack, repaired or registered as the
acceptance rule says, not a reason to soften the profile.

Size: M an effect family (the model in the simulator or the sensor relay,
its magnitude, its test), the series after each.

This item is complete when the profile runs in every acceptance flight
with its seed in the manifest, every effect's magnitude has its source in
the specification, and both sensor sets pass the urban series five of five
under it.

**Done on 2026-10-29 but for the stereo set's acceptance under the profile**
(the goal of 2026-10-29, part 1; code 16468cd3 to 93ec0747; specification K52,
F21). Built: `IMPERFECTIONS=nominal` with `IMPERFECTION_SEED` (the run's number
in `tools/series2.sh`, so every acceptance flight carries it and the manifest
reproduces it), four families each a measurement with its source
(`tools/imperfections_sources.py`): the IMU's bias walk from the EuRoC imu0
calibration written into a runtime copy of `x500_base` as the model's dynamic
bias (the element takes the walk's density, 73a6d193); the rotors' vibration
from the NTU VIRAL eee_01 recording as a wrench at 55 Hz on `base_link`
(`drone_vibration_system`); the sensor set's pose turned by the calibration's
residual of 0.08 degrees (K33); the stamps as a real driver writes them, the
IMU stream's jitter and gaps (the K28 relay's `jitter` profile), the frames'
offset of 2 ms (the stream relay), the lidar sweeps' cadence
(`lidar_sweep_jitter.py`). The aerodynamics near walls and the spread of
thrust and mass have no source on this machine and are in the register.

Flown apart (`tools/imperfections_series.sh`, forty counted flights) and whole
(ten), seeds 1 to 5, the clean location, no contact in fifty:

| family | stereo set, m/s (R3 1.2) | B, m (within 2) | lidar set, m/s (R3 2.0) | B, m |
|---|---|---|---|---|
| clean (r2333 to r2338, r2349 to r2353) | 1.248 to 1.345 | 0.63 to 1.44 | 2.029 to 2.493 | 0.22 to 0.49 |
| stamps (r2376 to r2386) | 1.093 to 1.253, two under | 0.55 to 1.75 | 2.023 to 2.219 | 0.24 to 1.30 |
| imu (r2389 to r2399) | 1.178 to 1.261, three under | 0.53 to 2.21, one over | 1.620 to 2.418, one under | 0.32 to 0.71 |
| vibration (r2400 to r2409) | 1.173 to 1.362, one under | 0.56 to 1.29 | 2.265 to 2.527 | 0.24 to 0.48 |
| extrinsics (r2410 to r2419) | 1.159 to 1.335, one under | 1.17 to 2.29, one over | 1.610 to 2.530, one under | 0.20 to 0.64 |
| whole (r2420 to r2433) | 1.074 to 1.258, three under | 0.88 to 2.82, two over | 1.906 to 2.559, one under | 0.44 to 1.19 |

The lidar set passes under the profile (its three slow flights are of the
clean location's own kind, r2368 1.968). The stereo set does not: its
estimate's heading turns 0.2 to 2.0 degrees a flight against the truth under
the profile (0.3 to 0.7 clean; the lidar 0.2 at most), which is the whole of
its error (0.2 m remains after the turn) and misses B and R3; the physics of
the magnitudes alone (jitter of 2.6 ms, 0.08 degrees, a ripple of
micrometres) gives centimetres, so the finding is the estimator's sensitivity
and it is in the register (class (b), the remedies named), not a reason to
soften the profile. The cost: the onboard set within the clean figures
(cameras 4.28 to 4.42 cores at p50, lidar 3.41 to 3.72), the relays 0.27 cores
outside it. What remains of the item: the stereo set's five of five under the
profile, through the register's row, which item 40 takes up (the depth
registered against the vehicle's own map along the way).

## 36. The Simulator Waits For The Onboard Loop: Deterministic Flights

**Type:** infrastructure of the simulation, deferred; weighed on 2026-10-24
at the project owner's question (why a host load of 15 to 20 percent voids
a flight, and whether the stack can live with a loaded host) and recorded
as an item at the owner's word. Not proposed for now: an L-sized work that
also removes what the acceptance measures today, unless the budget is
modelled (below).

**Hard prerequisites:** item 20 stage 0 (every period of the stack on the
simulation clock, K24; the autopilot in lockstep); K25 (a slowed flight
flatters the onboard budget, and what a relay would have to delay). Item
28's emulation of the onboard budget and the reserved core set of the
flights (A4) are the near answers to the same question and come first.

**Validation environment:** any; the point of the item is that the
environment of the host stops mattering.

Today the simulator runs at the wall clock's pace and the onboard nodes run
beside it: whatever the host does to the nodes, the world does not wait. So
a flight is a measurement of the host as much as of the stack, and the
verdict A7 (the real-time factor at the median, the sampler's gaps, the
poses' age) decides whether it counted. A host that slows the simulator
hands the stack a faster computer than the vehicle has (K25); a host that
stalls the loop crashes a vehicle that was not at fault (r720, r723). The
reserved core set (A4) narrows the host's share of the measurement; this
item would remove it: the simulator advances a step only when the onboard
loop has finished with the previous one, as the autopilot already does in
lockstep. Two flights with the same seeds then fly the same metres on any
host, under any load, as slowly as the host needs, several at once on one
machine, and in continuous integration.

What it costs, and why it is not proposed now:

- **The budget's honesty.** With the simulator waiting, the tick's 20 ms,
  the planner's 150 ms and the estimators' scan periods stop being
  deadlines: a slow computer flies as well as a fast one, which is exactly
  the flattery K25 names. The item is only acceptable with a model of the
  onboard computer in the loop: every node's measured wall duration on the
  reference host (or on the target board, item 27) becomes a configured
  age in simulated time, so that an estimate, a memory update or a horizon
  reaches its consumer as late as it would on board (the relay K25
  describes for item 15's four vehicles). The model's numbers are measured
  figures of the specification, kept against the host of record, not a
  parameter one turns down; and the acceptance's compute checks (the tick's
  and the planner's wall-clock figures, A7's justification) become checks
  of the model's inputs instead.
- **The mechanism.** A step controller over the world's control service
  (`/world/<name>/control`, which the GUI controller already speaks,
  `scripts/gazebo_gui_control.py`) advancing the world by the loop's step;
  an acknowledgement from every onboard node per step (the planning tick,
  the obstacle memory, the estimators, the mission monitor), ordered so
  that each step's sensor data is delivered before the nodes run; the
  sensors' rates (the lidar at 10 Hz, the pair at 7.5 Hz) and the
  autopilot's lockstep kept consistent with the step; the recorded flights
  of A11 unaffected (a recording is flown at real time by its own rule).
- **Size.** L: the controller, the hooks in five nodes, the budget model
  and its measurement, the acceptance rules rewritten (A1, A4, A6, A7, K25,
  the host's verdict), the series flown again on the new footing against
  the old. It is justified when the flights have to run where the host
  cannot be kept quiet (continuous integration, several flights at once, a
  weaker machine), or when item 28's budget emulation needs the computer's
  model anyway.

The order, if it is taken up: (1) the budget model alone, measured on the
reference host (the per-node wall durations as the specification's
figures) and applied as a relay at a real-time factor of 1.0, where it must
change nothing (the control that the model is honest); (2) the step
controller and the acknowledgements, flown at 1.0 and at 0.5 with the same
seeds, the flights compared metre by metre; (3) the series on both sensor
sets and the acceptance rules moved to the new footing.

This item is complete when two flights with the same seeds reproduce each
other on a quiet and on a loaded host, the onboard budget is a measured
model in the loop whose removal is visible in the flight, and A7's verdict
is no longer needed for a flight to count.

**Measured on 2026-10-30** (the goal of 2026-10-29, part 2; code 20414079
to be4fa324; specification K53). Built, in order: (1) the onboard budget as a
measurement (`tools/node_budget.py`) and as a relay, `budget_relay_node`
(`BUDGET_RELAY=on`), which holds every output of the loop until it is as old
on the simulation clock as the host of record measured it at its consumer
(the ages in K53), the bytes untouched; at a real-time factor of 1.0 it
changes nothing: cameras r2456 to r2464 at 1.137 to 1.335 m/s and the lidar
r2467 to r2471 at 1.947 to 2.429, the ages the planner sees the base's, no
contact. (2) The step controller, `scripts/step_controller.py`
(`STEP_CONTROLLER=on`): the world starts paused and seeded (`gz sim
--seed`), the controller advances it 20 ms at a time (`STEP_MS`) and after
each step waits for the acknowledgement of every onboard node (the planner,
the memory, the offboard, the mission monitor, the estimator, the stereo
depth node; `StepAcknowledger` behind `step_ack_enabled`, off by default and
silent on the vehicle) and of the pilot, which acts inside the protocol. In
r2479 and r2480 the world moved only by the controller's steps (15100 steps,
0 runaways, 0 timeouts, the acknowledgements' median wait 22 ms, the planner
the slowest), at a wall speed of 0.34: the flight's window (`SMOKE_DURATION_S`,
wall seconds) then needs three times the clean one's.

Two flights with the same seeds (`tools/flight_diff.py`) do not fly the same
metres: before the controller, r2472 and r2473 parted at the arming (25.45
against 25.00 s, the pilot's timers and the free-running start); with the
world paused from the start and the pilot in the protocol, r2474 and r2475
parted at the climb (8.6 s; the depth returns of a step matched before or
after the next); with the depth node acknowledging and the world truly held,
r2479 and r2480 part at 9.0 s (0.15 m at 10 s, 1.0 m at 15 s, 9 to 20 m
from 30 s on): the pilot arms at the same 5.008 s, the estimator's first
healthy frames track 71 against 73 features, the memory's first accepted
scan is one frame earlier in one twin (2.526 against 2.658 s). What remains
lies inside a step: the order in which the step's messages reach the nodes
(DDS, best effort on the sensor paths), the memory's and the planner's worker
threads and the MPPI's GPU reductions, and the autopilot's own threads under
its lockstep. The item's criterion (within 0.10 m over a flight, Р2.4) is
not met after the two repairs the part allowed; the acceptance rules (A1,
A4, A6, A7, K25) stand as they were and the series fly as before. What the
controller gives today: a flight whose pace the host cannot change (the
world waits for the loop), with the budget relay keeping the loop's
latencies honest. Variants for the rest: an in-step order (every node
consumes the step's messages in one fixed order, which means the nodes read
their inputs through the controller, an L-sized rework of the transport);
a CPU MPPI engine with a seeded, ordered reduction (item 28's way A gives
it); the autopilot replayed from its own log instead of run (its lockstep is
not deterministic across runs). Recommendation: take the deterministic
flight up again after item 28's way A, on the lidar set first (its estimate
registers against the map and does not drift), with the in-step order as
the one remaining mechanism. Registered in `technical_debt.md`.

## 37. Full Support Of The ModalAI VOXL 2

**Type:** new capability, deferred; recorded on 2026-10-24 at the project
owner's word after the reading of the compute gap and the builds without
CUDA (item 28, [hardware_builds.md](hardware_builds.md)): the VOXL 2 is the
board a backend without CUDA would be built for, as the support of a
possible entry into the United States government market (NDAA section 848,
the Blue UAS lists), where it covers the computer, the autopilot and the
cameras in one listed board of 16 g. The market is not set (specification
R6); this item says what the board would take, in full, so that the
decision is taken on a list and not on the backend alone.

**Hard prerequisites:** item 28 way B (the MPPI engine on Vulkan or OpenCL)
and its step 1 (how many rollouts the flight needs); item 27 blocks 2 to 6
(the build without Gazebo and for arm64, the sensor drivers, time and
frames, the vehicle model, the checks before a flight), which every real
vehicle needs and this board does not spare; item 26 or the camera's own
depth for the stereo matcher off the CPU. A VOXL 2 board ($1 250) for
everything past the host.

**Validation environment:** none in the simulator beyond what items 27 and
28 use; the board on a bench with recorded data first, then a vehicle of
its class (the Starling 2 Max's: 322 mm, 500 g, 500 g of payload).

What the board is, by the registry: a Qualcomm QRB5165 (one Cortex-A77 at
3.09 GHz, three at 2.42, four Cortex-A55; an Adreno 650 with OpenCL 2.0 and
Vulkan 1.1, its FP32 rate not found, about 1 TFLOPS a guess; 8 GiB shared),
PX4 on the board, ModalAI's own camera family on MIPI, 16 g. What the stack
is: ROS 2 Jazzy nodes, the autopilot over uXRCE-DDS on UDP with `px4_msgs`,
a pair of 1280 x 960 at 0.20 m configured by its calibration (K33), six
VL53L5CX on I2C, a carried light with a driver that reports, CUDA in the
controller alone.

The work, in three layers; what is checked and what is not is marked:

1. **The controller without CUDA** (item 28 way B, L to XL): the twelve
   kernels on Vulkan compute or OpenCL, parity tests against the CUDA
   engine on the same noise, built and tested on the host's GPU first (the
   RTX 3060 speaks Vulkan); then the rollout count the Adreno carries in
   the 20 ms tick (about 1 000 or fewer by the guess above) and the
   controller tuned at it, which item 28 step 1 prepares.
2. **The CPU**: four large cores come to roughly 2.5 to 3 Zen 3 cores
   against the onboard set's 3.4 to 3.8 (lidar) and 4.3 to 4.6 (stereo
   set) at p50 (hardware_builds.md, "The Compute Gap, Measured"): the
   stereo matcher off the CPU (the board's own depth server on its
   accelerators, if its depth is taken, or item 26), the controller's CPU
   share and the planner's budget revisited, the heavy threads pinned to
   the A77 cores (big.LITTLE scheduling, which the launcher's affinity
   masks do not know), the thermal behaviour of a 16 g board under four
   busy cores measured.
3. **The platform**, each point to be checked on ModalAI's documentation
   and then on the board, none checked yet (2026-10-24, from memory of the
   platform, not from the project's documents):
   - the operating system and middleware: voxl-suite with its modal pipe
     architecture (MPA); ROS 2 runs on it in ModalAI's containers; which
     ROS 2 distribution is supported, and whether Jazzy's `rclcpp` and the
     stack's messages build there, decides whether the nodes run as they
     are or inside a container of ours;
   - the autopilot link: PX4 runs on the board (voxl-px4) and talks to the
     applications through MPA; whether it carries the uXRCE-DDS client the
     stack uses over UDP to the same board is unchecked
     (hardware_builds.md); if not, an adapter from MPA to the stack's
     topics (the offboard setpoints, `sensor_combined`, the status and
     failure flags, the external odometry in);
   - the cameras: ModalAI's pairs come through voxl-camera-server as MPA
     frames, not `sensor_msgs/Image`, at about 8 cm of baseline against the
     0.20 m the speeds rest on (about 0.6 of the confident range,
     camera_perception.md): an adapter and a calibration, and either a
     slower vehicle by the braking contract or the stack's own 0.20 m pair
     on the board's MIPI connectors with sensors its drivers support;
   - the time-of-flight ring: six VL53L5CX on I2C need the board's I2C
     lines and a driver, or ModalAI's single PMD time-of-flight camera
     through an adapter with K19's reflex and K30's landing read against one
     unit instead of six;
   - the IMU (the board's own and PX4's `sensor_combined`, which the stack
     reads), the light's driver on the board's GPIO, the power budget.
4. **The parts around the board for that market**: the frame, motors,
   ESCs, radios and batteries checked against the buyer's rules; the lidar
   profile most likely not offered (Livox and Unitree excluded, an Ouster
   heavy and dear), so the VOXL 2 vehicle is the camera build.

What the board gives back: the autopilot, the computer and the cameras in
one listed board; a vehicle of a few hundred grams flying two to three times
as long as the x500; and, if its depth server is taken, the stereo matcher
off the CPU for nothing.

Size: the backend L to XL (item 28); the platform layer M to L; the drivers
and the identification M (item 27); months in all, against a Jetson, which
spares the backend and the adapters and needs item 27's blocks the same.
Without the board: the backend and its parity tests on the host's GPU, the
rollout series (item 28 step 1), the arm64 build (item 27 block 2); the
rest needs the board and ModalAI's documentation read.

This item is complete when the stack flies a point-to-point mission on a
VOXL 2 vehicle of its class with the camera profile, through the pilot's
authority and the mission's end as item 27 block 1 asks, the acceptance of
A1 flown on the vehicle itself, and every point of layer 3 is written as
checked, with what was adapted and what was taken from the board.

**Layers 1 to 4 on 2026-10-30, by the documentation read that day and
without the board** (the goal of 2026-10-29, part 3; closed by record under
K6). Sources: docs.modalai.com pages voxl-suite, voxl-px4, voxl-camera-server,
voxl-imu-server, voxl-vision-hub, voxl2-connectors, voxl2-camera-configs,
ros2-installation-voxl2, mpa, M0040, calibrate-imu, voxl2-linux-user-guide and
the product page modalai.com/products/voxl-2; the pages voxl2-datasheet,
ros2-on-voxl2, voxl-tof, opencl-guide, voxl-microdds-agent and voxl-gpio
answered 404 that day.

| Point | Done | By the documentation (2026-10-30) | Requires the board |
|---|---|---|---|
| 1. The controller without CUDA | Way A (one body, the CPU engine, parity) and way B (the Vulkan engine, parity with CUDA 1.4e-6, ten flights; above) | The Adreno 650 is "1024 ALU" on the product page, no FLOPS given; Vulkan's version and `shaderInt64` not in the pages read | The rollout count the Adreno carries in the tick (the RTX 3060 carries 8192 in 10.8 ms); `shaderInt64`, or the hash in two 32-bit halves |
| 2. The CPU | The flight's core set takes a CPU list (`FLIGHT_CPUSET`, A4), so the A77 cores can be named; the CPU engine is not the controller for the board (0.39 ms a rollout) | "8 cores up to 3.091 GHz", 8 GB LPDDR5, 128 GB, 0.1 to 7 W, 16 g, 70 x 36 mm (product page) | The big.LITTLE numbering, the thermal behaviour under four busy cores, the stereo matcher's place |
| 3. OS and middleware | - | Ubuntu 18.04.5 LTS on kernel 4.19 (a console log on the user guide; the product page says 18.04, kernel 4.19); voxl-suite as a meta-package, releases `sdk-1.8.0` (SDK 1.8 the latest, 1.6.8 before); MPA on POSIX pipes with `libmodal_pipe`, services as systemd units; ROS 2 Foxy native (`voxl-ros2-foxy`), "for a newer ROS 2 version you must use Docker": the stack's Jazzy nodes run in a container of ours; `voxl-mpa-to-ros2` bridges IMU (`/imu_apps`, `sensor_msgs/Imu`), images (`sensor_msgs/Image`), `/tof_pc` (`PointCloud2`) and VIO (`/qvio`, `PoseStamped` or `Odometry`) | Whether Jazzy's `rclcpp` and the stack's messages build in that container on kernel 4.19; DDS between the bridge's Foxy and our Jazzy |
| 3. The autopilot link | The stack's path exists on the board: `voxl-microdds-agent` "connects ROS 2 to a PX4 instance on the board over UDP port 8888, which exposes `/fmu/...` topics" (topic names differ between SDK 1.6 and 1.8) | PX4 runs "as a Linux service on QRB5165" (voxl-px4), the product page says "on the sensors DSP"; SDK 1.8 carries PX4 1.18 (`voxl-dev-1.18.0`); VIO reaches PX4 as MAVLink `ODOMETRY` through `voxl-vision-hub` and `voxl-mavlink-server`; MAVROS and MAVSDK on UDP 14551 | The `px4_msgs` of PX4 1.18 against the stack's, the agent's rate and the versioned topic names (K-row on the `_v1` suffixes) |
| 3. The cameras | - | `voxl-camera-server` (HAL3) publishes MPA frames, preview "NV12 or RAW8"; stereo pairs OV7251 640 x 480 at 30 (M0015) or OV9782 1280 x 800 at 30 (M0113) with frame-sync pins, `independent_exposure`; the configurations C-01 to C-17 are fixed ("not mix-and-match"); the baseline is not on the pages read (the 8 cm of this item's first text stays unverified) | The pair's baseline and a calibration of ours (K33) at 1280 x 800, the frame adapter from MPA to the depth node's input, the braking contract at that baseline |
| 3. The time of flight | - | PMD M0040 (IRS1645, 224 x 171, 5 to 45 FPS, 850 nm, "indoors up to 6 m", 3 g) is EOL, the M0178 current; mutually exclusive with the stereo pair on the same connector group; published as `/tof_pc` | One forward unit against the ring of six: K19's reflex and K30's landing read on one unit; or the ring's VL53L5CX on the board's I2C (J3 I2C9/I2C10 and J5's, 1.8 V) with a driver of ours |
| 3. The IMU | - | `voxl-imu-server`, pipe `imu#`, `imu_data_t` (m/s^2, rad/s, `timestamp_ns` on `clock_monotonic`), `imu_sample_rate_hz` 1000; the product page says two ICM-42688-P, the calibration page one IMU ("IMU0") on VOXL 2; PX4's own IMU reaches the stack as `sensor_combined` through the agent | Which IMU the autopilot reads and at what rate over the agent; the vibration profile of the airframe (K52's figures came from a VN100) |
| 3. GPIO, the light | - | Numbered GPIOs on J3 and J5 (1.8 V CMOS) and on J6 to J8; J10 at 3.3 V (SPI or UART); the GPIO access page answered 404 | The light's driver on 1.8 V lines (a level shifter) and its reading (K29) |
| 3. The power | - | J4 +5 V +-5 %, about 6 A of in-rush; 0.1 to 7 W | The budget with the cameras, the light and the ring |
| 4. The parts around the board | - | Starling 2 (280 g, over 40 min, VOXL 2, three tracking cameras, a colour camera, a time-of-flight camera, PX4, $2 949.99) and Starling 2 Max (322 mm, 500 g, 500 g of payload, over 55 min, $2 999.99) in the registry | A buyer's rules (R6): no decision here |

Recommendation: a Starling 2 Max as the vehicle of the class (the board,
PX4, the cameras and the ToF in one listed airframe with the payload for the
light and the ring), and, before any board, a bench of recorded data (the
board's camera frames, IMU and `/fmu` topics replayed into the stack) to
settle the adapters; the register's row (c) names it.

## 38. Faster Than Real Time On A Stronger Host

**Type:** infrastructure of the simulation, proposed by the project owner
on 2026-10-29 at the question whether the slowed flight of K25 has an
inverse: the same flights flown faster than the wall clock on a computer
stronger than this host, so that a series takes fewer hours.

**Hard prerequisites:** K24 (every period of the stack on the simulation
clock; the autopilot in lockstep); the core set of A4. Item 36 supersedes
it: a simulator that waits for the onboard loop runs as fast as the host
allows by construction, and the factor disappears.

**Validation environment:** any; Urban Circuit Practice 01 for the
comparison with the series at 1.0.

Possible, by what is in place: the factor is written into the world by the
preparation (`scripts/prepare_environment_simulation.py`,
`configure_real_time_factor`), PX4 SITL follows the simulation clock in
lockstep, every period and watchdog of the stack runs on the node clock
(K24), the sensors' rates are the world's (the lidar at 10 Hz, the pair at
7.5 Hz of simulated time), and the mission check measures speed on the
simulation clock (R3). What stops it today is a rule, not the code: K25
says the factor is never above 1, because a factor under 1 flatters the
onboard budget. Above 1 the flattery turns the other way: the stack gets a
slower computer than the vehicle has, by the factor, so the tick's 20 ms
and the planner's 150 ms must be met in less wall time. That is harsher,
not kinder, and it is why a flight above 1 can serve regression and
exploration but not the acceptance's figures until item 36 models the
budget.

What limits the gain is this host: the camera flights already run at a
real-time factor of 0.85 to 1.00 of the wall clock on it (the depth node,
the controller and the simulator share the cores, A4), and the smoke
relay's frames stall the simulator at 1.0 (K25); the lidar flights hold
1.00 with an unmeasured margin. A factor of 2 needs a host that renders
the pair's frames and runs the matcher twice as fast, so the item is for a
stronger computer (a rented host, development.md, or a workstation), and its
first measurement is how far above 1.0 each profile goes before A7's floor
(scaled by the factor) is lost.

The work: the factor allowed above 1 in the preparation and the run
script, A7's floor and the watchdog's read against the factor as they are
below 1, the manifest's record, and K25 rewritten (a factor above 1 is a
stress, counted for regression, never for the acceptance's figures); then
five flights a profile at the largest factor the host holds, compared
with the series at 1.0 (the mean speed, the arrival, the holds, the
tick's and the planner's wall figures). Size: S for the switch, M with the
rules and the comparison.

This item is complete when a flight above 1.0 counts by the rules written
for it, the largest factor each profile holds on a named host is a
measured figure, and the series at that factor matches the series at 1.0
on the metres flown.

## 39. Open Sky: The Hybrid Stack, Underground And Outdoors

**Type:** direction of the project, decided by the project owner on
2026-10-29: the vehicle goes outdoors, and the stack becomes a hybrid that
flies the cave and the open sky with one set of rules. The owner's figure
for the open sky's speed: 10 m/s at least, 20 m/s the aim, against the
2 m/s of the cave (R3); recorded as the open-sky profile's requirement in
the specification (R8), to be set as the cave's was, by measurement, when
the profile's first series flies. The location for such flights (found or
built) is a question of its own, stage 0 below.

**Hard prerequisites:** none for stages 0 to 2 (the simulator and the
parts in place); item 35 for the sensors' imperfections of stage 3; item
25's method for the drift of stage 4. The order against items 33 to 35 is
the owner's, when the item is taken up.

**Validation environment:** an open-sky world of its own (stage 0); Urban
Circuit Practice 01 keeps the cave's acceptance.

What the open sky changes: a satellite signal that is there, then gone,
then wrong (jammed or spoofed), against the cave's none at all; heights of
tens of metres over ground that the forward pair cannot see; kilometres of
distance, where 0.1 to 0.4 percent of drift is 10 to 40 m; and the speed.
What carries over as it is: the camera estimator with its health rules
(the declared dead reckoning, the ladder of failures, the rotors' drag,
the barometer for height), the relocalisation on the vehicle's own map,
the way failures are read (an external component fails, the vehicle
decides on its own measurements), the `gnss` profile and the autopilot's
fusion. What the open sky does not need: the dark, the smoke, the thermal
core's roles of items 17, 18 and 32, and most of the obstacle logic of a
corridor.

The stages:

0. **The location.** An open-sky world with ground, a few buildings and
   trees, kilometres across, in the simulator: a Fuel world or one built
   from the environment pipeline (item 11's materialisation), with a
   truth the mission check reads as now. Chosen by the same route as
   `environment_candidates.md` chose the cave worlds.
1. **The signal lost on the way.** A relay that drops GNSS in flight, as
   the smoke relay drops the frames; the `gnss` and `visual_inertial`
   profiles, exclusive today, fused at once with their frames aligned,
   so that the flight goes on when the signal goes; the drift over
   kilometres measured. Size S to M, all in the simulator.
2. **The signal lied to.** The relay walks the satellite position away
   slowly; the stack reads a steady disagreement with the estimator,
   stops trusting the signal and takes it back when it agrees again:
   deterministic, a measurement, no latch. The most valuable stage: an
   independent check of the signal's honesty. Size M.
3. **The sensors for height.** A camera looking down for the estimator (the
   forward pair at 30 to 100 m sees nothing near), the magnetometer back
   for the heading (`EKF2_MAG_TYPE` 5 today), a rangefinder down; a new
   vehicle model and the estimator tuned again. Size L.
4. **Absolute correction without the signal.** Terrain against a height
   model, or frames against an orthophoto, deterministic; or an external
   absolute source modelled as a noisy coordinate. Size L. The cave's
   correction without any external source is item 40 (the depth registered
   against the vehicle's own map along the way); the two share the way a
   correction is applied and bounded.
5. **The speed and the obstacles.** The braking contract on the pair's
   6.4 m gives about 2.4 m/s; the open sky asks 10 to 20. Either the route
   above the obstacles (the planner nearly idle) or a lidar of 40 m; the
   speed profile's ceiling (10 m/s, V3) raised with the airframe's
   envelope measured (item 30). Size L; this is where R8 is measured.

The signal's ways of failing, all of which the vehicle handles (the
owner, 2026-10-30), and all read by one measurement, the disagreement
between the signal and the estimator over time, never a latch:

| Variant | What the vehicle sees | Where it is flown |
|---|---|---|
| Lost | no fix, or a fix flagged invalid | stage 1 |
| Flapping | the fix lost and back every few seconds | stage 1 |
| Jammed | no fix, or a fix with a reported error of tens of metres | stage 1 (jamming is a loss with noise) |
| Degraded | an honest fix with a large, varying error (multipath, few satellites) | stage 2 |
| Spoofed, slow walk | the fix drifts from the truth at a walking pace, the reported error small | stage 2 |
| Spoofed, jump | the fix leaps tens or hundreds of metres at once | stage 2 |
| Spoofed, replayed or delayed | a fix consistent with itself but late, so its velocity and position disagree with the estimator's | stage 2 |
| Spoofed velocity or time | the fix's velocity or clock inconsistent with its own positions | stage 2 |
| Back but wrong | the fix returns after an outage at an offset | stage 2 |

The relay of stages 1 and 2 (`GNSS_FAULTS`, a list of these with their
times and magnitudes, the seed the run's) plays every one of them; each
has a mission check of its own (the vehicle arrives by the truth within the
rule, the signal distrusted and taken back at measured times) and the
acceptance of the stage flies them all.

Stages 1 and 2 are cheap, made of parts in place, and give the strongest
demonstration of the direction (the signal lost, jammed or spoofed in any
of the ways above, the vehicle knows it and arrives); stages 3 to 5 are a
second product of the size of the roadmap behind this one. The invariants stay: unknown space free, every
prohibition a measurement, no history latch, and the signal's trust a
measurement too.

This item is complete when the vehicle flies a point-to-point mission of
kilometres over the open-sky world through every variant of the signal's
failure in the table above,
arrives by the truth within the rule written for it, keeps the cave's
acceptance unchanged on the same code, and R8 is a measured figure.

## 40. The Stereo Estimate Registered Against Its Own Map Along The Way

**Type:** localization, the stereo set; decided on 2026-10-30 after the
owner's direction to settle the register's row on the heading drift as a
roadmap item (the cave's own problem, with no signal to lean on; the open
sky's absolute sources are item 39 stage 4).

**Hard prerequisites:** item 19's relocalisation on the vehicle's own map
(the offset moved on return) and the long-lived map of the obstacle memory;
item 35's profile (K52, F21) as the measurement that shows the drift.

**Validation environment:** Urban Circuit Practice 01, the acceptance
configuration under the imperfections profile.

The finding (item 35, the register's Localization row): under the real
vehicle's imperfections the visual-inertial estimate's frame turns 0.2 to
2.0 degrees over the 440 to 510 m of the acceptance path (0.3 to 0.7 clean,
the lidar 0.2 at most), the residual after the turn 0.2 m, so the goal is
acknowledged 2 to 3.6 m from B in two flights of five and the mean speed
falls under the target in half of them (r2420 to r2428 on CUDA, r2484 to
r2489 on Vulkan: the same on every engine, the parity 1e-6). Heading is
unobservable to an odometry; the lidar set does not drift because its
estimator registers every sweep against its map. The stereo set's depth
reaches the same map and is registered against it today only on return
(item 19). This item registers it along the way.

What is built:

1. **The registration.** The depth node's surfaces of the current frame
   against the long-lived map's surfaces around the vehicle (the voxels the
   memory holds as occupied with their evidence), a rigid fit (a point-to-
   plane step as the lidar's, K45's priors for the level and the height kept),
   run at the depth's rate or every few frames; its result a correction of
   the estimate's frame (heading first, then position and scale), applied
   as the relocalisation applies its offset: deterministic, a measurement,
   no latch, the correction's size and residual published every time.
2. **When it may act.** Only where the map has surfaces the frame sees
   (the fit's residual and its point count the measurement; a fit over few
   points or a flat residual does nothing), never in the void (K46) or
   unknown space (the invariants); the correction bounded per step by what
   the estimate's own uncertainty admits, so a wrong fit cannot throw the
   vehicle.
3. **The measurement.** `tools/estimate_error.py`'s frame turn and scale
   per flight, the correction's history per flight (how often, how much,
   the residual), the estimate's error against the truth before and after
   the correction, and the cost in the onboard set (the depth node's and the
   estimator's cores and the frame's latency).
4. **The series.** The stereo set under the profile, five flights at the
   acceptance configuration, then the clean vehicle, then the lidar set
   unchanged on the same code; item 31's void crossings flown again (the
   registration must stay silent there).

Size: L (the estimator's core and the depth node; the fit itself is the
lidar's, the gating is new).

This item is complete when the stereo set under the imperfections profile
reaches B within 2 m in five of five flights with the series' mean speed at
R3's target, the frame turn per flight is within the clean vehicle's 0.7
degrees, the lidar set and the void crossings are unchanged on the same
code, and the register's row on the heading drift leaves.

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
clearance rule keeps 0.27 m (measured on 2026-09-25 as the takeoff spool-up;
the hover's own figures are in the register); the ordinary no-route holds sit at 3 to 13
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

### 17. Flight In Degraded Visual Conditions (Completed)

Shipped in [v0.5.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.5.0) on 2026-10-10; closed on 2026-10-03 on 6e7ea775. The camera vehicle
flies the urban point-to-point mission in a location with no light but the
one it carries, under a light that flickers in every flight, and comes home
when that light fails or when its battery will not last to the goal; with its light gone for good it descends and stays whole. The
ordinary flight of the repository is that dark flight. The estimator in the
dark is in [`localization.md`](localization.md), the perception in
[`camera_perception.md`](camera_perception.md), the scenarios in
[`scenarios.md`](scenarios.md), the checks in [`testing.md`](testing.md), and
every number in [`specification.md`](specification.md).

Built, by stage, each decision the smallest change found:

- **Stage 0, the contract reads the frame** (42f5b375, 70b737a1). The
  forward range and the evidence age of the braking contract are measured
  from the sensor's latest frame (specification K9). Both series re-flown on
  it: the stereo set 1.84 to 1.91 m/s (r747 to r751), the lidar 2.59 to 2.85
  with one flight at 2.26 (r752 to r756).
- **Stage 8, darkness in the memory.** An occupancy not confirmed decays, 30 s
  per confirmation it collected (01780e58, 88777c43, K10). A stereo frame the
  contract reads as blind for 2.5 s marks the frustum it looks into as
  observed unobservable: occupied evidence without free space, never over
  the vehicle's own cell or the way it came (8f6535ad to 2d91c391, K14). The
  planner's budget was measured before it (6306 updates, none over 185 ms)
  and left as it is.
- **Stages 1 and 3, a dark world and a light on the vehicle** (b5562e4d to
  a60388d9). The location without its ambient fill
  (`WORLD_ILLUMINATION=dark`), the cameras' noise and automatic gain, a match
  within the image's noise refused as an observation, and a spot light on
  the airframe over the pair's whole field. Five ways of lighting were
  compared on range, price and average power
  ([`illumination_options.md`](illumination_options.md)): the strobed
  near-infrared flood synchronized with the global shutter gives the full
  6.4 m at about 1.5 W on average, against 15 to 100 W for a continuous
  flood and 6.5 W for the lidar, for some tens of dollars (F12). The
  confident depth re-measured in the dark by that light stays 6.4 m (K7);
  with the pair's noise in effect (A12) it is 5.2 m (r1476, item 29).
- **Stage 2, surfaces the matcher cannot match** (d663be63). The curve of
  the matched share against texture is in
  [`camera_perception.md`](camera_perception.md): a texture-poor surface
  gives absent depth, not wrong depth, the contract's range falls with what
  the frame observes, and a uniform panel across the route entered the
  memory whole and was flown over (r783). No remedy is built (F13).
- **Stage 4, the time-of-flight ring** (a2a4433e): four more 8 x 8 sensors on
  the horizontal, in the same contract with no new rule; 99.9 percent of the
  flown path observed before it was entered (r784).
- **Stage 5, the light fails.** The injector (`scripts/carried_light.py`,
  293af128) dims and darkens the carried light by a seeded schedule, never
  announced; `scripts/camera_stream_faults.py` (37fa7f52) drops and delays
  the pair's frames through a relay. The vehicle knows its battery's charge
  and nothing else. The mission monitor weighs the charge against the way to
  B (K11) and judges the light from the frames (K12); either gives B up for
  the start through item 19's substitution (0e09f0f6). The time-bound
  return is gone, and with it every reading of the run's window by the
  vehicle (I8, A8). The ladder: stop (the contract), hold (K15), declared
  dead reckoning on the barometer's height (K13, cfae95f9 to 088d89c3),
  lost for the flight once it is spent (04295345), and a level descent at
  0.5 m/s after three seconds of it (K19, fabbd802, 35ab7244, 6e7ea775);
  [`localization.md`](localization.md). On the way home the vehicle flies
  back along the trail it flew out on, a point every 5 m, each a goal in its
  own mission epoch (K20, 6d7af22d): the planner's routes through unknown
  space had led a returning vehicle back into the dark (r894, r907).
- **Stage 6, a position reset of the autopilot** (3732b0ec): not built. Every
  reset over 3 m flown was the autopilot fusing a camera estimate that came
  back wrong after it had been lost, which K8 and K13 now keep silent; a
  reset up to 3 m is flown on (K1).
- **Stage 7, a zone that fails the light (removed on 2026-10-03)**
  (2b5d1658, 600d5f67): a region in the injector, the "magnetic anomaly",
  in which the vehicle's own light was halved six times across a falloff
  and out in the core, nothing of it known to the vehicle. One stood off
  the way to B in every ordinary flight, and one laid over B was a scenario
  of its own. It was accepted with the item (the tables below) and then
  removed by the project owner with its scenario, its switch and its RViz
  markers: a vehicle whose own light fails goes blind in every direction
  and cannot tell the place from a failed light, so a zone only ever sent
  it home, which the severe failure's scenario already checks. What the
  zones taught stays: the contract reads the frame's light, darkness is
  marked in the memory, a blind vehicle holds and judges its light.
- **The ordinary flight is dark** (bab016ee, A10): no ambient light, the
  carried light under the moderate flicker. Since 2026-10-03 the lit
  location is not selectable at all (the owner's rule).

Found by the flights and repaired on the way:

- **The estimator under its own light.** A light that moves with the cameras
  breaks the brightness constancy the tracker assumed: dark flights ended 2.1
  to 2.3 m from their goal. The tracker follows the frame's texture
  (214507c9; replayed on recorded flights 0.22 m against 2.96 m in the dark).
- **The goal's capture** latched on a vehicle crawling past 1.9 m from its
  goal; it waits for the approach to end (8de52e31).
- **The heading swung past its target** 14 to 37 times a camera flight: the
  offboard handed the autopilot a yaw sampled ahead on a horizon that starts
  at the measured one. It hands the yaw rate planned one rate-loop lag ahead
  (a130c02d, b2b24425, K17): no overshoot, 10 percent of the flight not
  facing its motion against 23 to 46.
- **A contact approached at the contact floor.** The 1 m/s a vehicle may
  leave a contact at also let it approach one: a rotor on a door's lintel
  (r837). A point the body nears is reached at its own tube (241e08bc, K18).
- **A stale stop beside walls.** The seed's box exemption bisected every
  contact pose per voxel; ticks of 250 to 700 ms let a stop go stale and the
  resident horizon met a wall (r804). One depth, no bisection (d7b39dc0).
- **The light judged late.** Read on the contract's range, which the memory
  raises back along the flown path, the severe failure was judged 1.5 to 3
  minutes late (r859, r860); the judgment reads the frame alone (5931dc52).
- **A hold that moved.** Holding on dead reckoning the vehicle moved 0.2 to
  0.6 m/s in truth with an estimate that stood still, and met walls a metre
  or two away (r856, r878, r879, r927, r932). The accelerometer across the
  rotor axis reads the rotors' drag, the body's velocity times 0.106 1/s
  (correlation 0.96 on recorded flights), and the filter fuses it every
  frame (bea5bb79, K21): replayed with the frames blanked, 10 s of dark
  drift 0.5 to 1.1 m against 1.0 to 5.3 m. A frame under 20 features aids
  nothing (c8241f00, K13): six points of the imager's noise had read as
  health to a landed vehicle, which took off blind (r933).
- **A failure on the pad.** An outage in the estimator's first seconds made
  the autopilot reset by 3.4 m and the flight never left (r924); the
  injected schedules count from when the vehicle is airborne (78d36dea,
  F14).
- **Landed and then lost.** A landing judged a crash when the autopilot's
  velocity drifted at rest (r819); an estimator that came back 35 m off
  (r820); the autopilot's landing mode tipping a landed vehicle over (r881).
  Each is in the ladder above.
- **The crash judge** itself: since the owner's rule of 2026-09-29 every
  contact is a crash but a landing on the ground or a floor (A9).

Acceptance on 6e7ea775, forty flights, each inspected before the next, the
speed on simulation time, positions in truth (the zones that failed the
light were part of these flights and were removed afterwards):

| Ordinary series | Flights | Mean flight speed, m/s | True position from the goal, m |
|---|---|---|---|
| Stereo set, dark, moderate flicker, a zone off the way | r949 to r953 | 1.79 / 1.84 / 1.90 / 1.83 / 1.85, mean 1.84 | 0.99 / 1.08 / 1.32 / 1.32 / 0.86 |
| 3D lidar | r954 to r958 | 2.55 / 2.56 / 2.68 / 2.63 / 2.26, mean 2.54 | 0.74 / 0.61 / 0.92 / 1.03 / 0.36 |

| Scenario (stereo set) | Flights | What ended the way to B, at s of simulation (flown, m) | Way home, s (m) | True position at the end, m |
|---|---|---|---|---|
| A long flight under failures: B, then the start | r959 to r963 | nothing: 853 to 1177 m at 1.73 to 1.83 m/s | — | B 0.83 / 0.94 / 0.78 / 1.11 / 1.24; the start 0.51 / 0.73 / 0.63 / 0.41 / 0.45 |
| The light lost | r939 to r943 | the level descent, 6.4 to 7.9 s after dead reckoning began | — | whole; touched down at 0.51 to 0.59 m/s, 0.63 to 0.95 m from where dead reckoning began |
| The severe failure | r944 to r948 | `unreliable_light` at 77 / 78 / 49 / 76 / 64 (111 / 116 / 63 / 104 / 82) | 86 / 71 / 12 / 58 / 27 (116 / 103 / 16 / 89 / 34) | the start 1.50 / 1.00 / 0.44 / 1.06 / 1.58 |
| A zone over B | r964 to r968 | `unreliable_light` at 225 / 268 / 284 / 213 / 240 (392 / 498 / 519 / 391 / 451), 5.6 to 7.1 m from B | 197 / 221 / 209 / 186 / 264 (356 / 404 / 420 / 371 / 511) | the start 0.69 / 0.62 / 0.63 / 0.69 / 0.82 |
| A low battery at launch | r969 to r973 | `battery` at 21 to 23, 20 m in | — | the start 0.38 / 0.63 / 0.43 / 0.45 / 0.43 |
| A goal outside the location, 720 s of light | r974 to r978 | `battery` at 174 / 122 / 190 / 193 / 131 (319 / 204 / 340 / 347 / 233) | 117 / 74 / 123 / 134 / 87 (258 / 160 / 265 / 269 / 174) | the start 0.43 / 0.33 / 0.43 / 0.62 / 0.44 |

No contact in the forty, no return and no light judged unreliable in an
ordinary flight, the zone off the way never nearer than 19.4 m (its dim
region ends at 17.5), and every return's trigger and moment in the log. One
lidar flight missed the speed target, 2.26 against 2.4 m/s (r958): 35.5 s in
the room of the shaft at (53, -7), the register's entry, the price of free
unknown space in an occluded dead end. The budgets held against stage 0's
series (r747 to r756): the tick 20.8 to 21.6 ms at p50 and 27.4 to 29.0 at
p95 on the cameras against 21.5 to 23.1 and 28.5 to 30.0, 23.2 to 24.3 and
30.6 to 33.2 on the lidar against 23.7 to 24.2 and 31.2 to 32.5; the onboard
processes 4.52 to 4.75 cores at p50 and 1001 to 1042 MiB on the cameras
against 4.62 to 5.04 and 961 to 1026, 3.59 to 4.00 cores and 833 to 943 MiB
on the lidar against 3.68 to 3.96 and 817 to 859. The light's energy is in
[`illumination_options.md`](illumination_options.md): about 1.5 W on average
for the strobed near-infrared flood.

After the acceptance, on 2026-10-03 and 2026-10-04, by the owner's decisions
and one repair, all of it flown again:

- **The sensors' mount** (8f7cedfd, specification K22). What item 19 had
  recorded as a spurious occupied layer over the staging base was the whole
  obstacle memory standing a voxel above the world, on both sensor sets: the
  x500's base_link stands 0.24 m above the model's origin, and the lidar and
  the camera set had been included by the model frame's numbers, 0.24 m
  under where the stack's extrinsics place them. Every surface was laid
  0.24 m too high, which gave clearance to floors and took it from ceilings.
  Mounted against base_link, the memory is level with the truth grid within
  about 2 cm (r980, r993).
- **No forgetting by time** (03e02055, K10): the memory's decay is off and
  stays off; what is transient is item 21's.
- **The zones that failed the light are removed** (61e9d3db), and the
  location's own light never comes back (d7bcef66).
- **Both ordinary series on all of it**, every flight counted by the host's
  measured verdict (A7):

| Ordinary series | Flights | Mean flight speed, m/s | True position from the goal, m |
|---|---|---|---|
| Stereo set, dark, moderate flicker | r983 to r987 | 1.74 / 1.97 / 1.94 / 1.90 / 1.89, mean 1.89 | 0.80 / 1.25 / 1.25 / 0.90 / 1.00 |
| 3D lidar | r988 to r992 | 3.12 / 2.57 / 2.68 / 2.71 / 2.48, mean 2.71 | 0.19 / 0.36 / 0.69 / 0.51 / 0.71 |

  No contact and no return. The lidar is over its 2.4 m/s on every flight;
  the tick stands where it stood, 21.3 to 22.2 ms at p50 on the stereo set
  and 23.2 to 24.6 on the lidar; route availability 97.5 to 98.4 percent
  and holds 1.5 to 2.6 percent on the stereo set.

On 2026-10-05 to 2026-10-08, by the owner's decisions, for the release:

- **The battery is weighed against the way home** (70ac7a0e, specification
  F7, K11): the way to B is not known until it is flown, so the vehicle
  flies on while its charge covers 1.1 times the way home's estimate and
  turns when it no longer does; a battery only in the scenarios that test
  one. Flown at 1.0 on 52dc179c: the low battery (240 s) turns 61 s in at
  101 to 103 m with 155 s left against 142 estimated home, and is home at
  0.28 to 0.58 m with 104 to 114 s left (r1140 to r1144); the goal outside
  (720 s) turns 211 s in at 389 to 404 m with 485 s left against 442, home
  at 0.33 to 0.41 m with 354 to 364 s left (r1145 to r1149). Both ordinary
  series on the same commit, now without a battery: the stereo set 1.818 to
  1.918 m/s (mean 1.850), 1.00 to 1.38 m from B (r1130 to r1134); the lidar
  2.420 to 2.891 (mean 2.607), 0.26 to 0.71 m (r1135 to r1139); the other
  three scenarios once each (r1150 to r1152).
- **The RViz evaluation overlay** (specification K26, [`rviz.md`](rviz.md)):
  the mission's events published by the nodes that decide them, and over
  the RViz view, in its top left corner, what the vehicle decided, the
  carried light's bar and charge, and the vehicle's estimate against the
  truth as two trails (white and green), and what the vehicle sees beside
  what the light was set to. It costs 0.20 cores at the median and runs
  only where RViz is open. The four scenarios recorded with it on dfe7121f
  are in `log/videos/2026-10-08/` (r1182 to r1185), every flight counted by
  the verdict ([`scenarios.md`](scenarios.md) says what each shows).
- **Darkness told apart, and the recording second for second** (2026-10-08):
  what the vehicle sees is the frame's share, not the measured range with
  its floor at the margin (`SEES 4 %` in full dark, where it read 31 %); a
  blind frame's stereo returns never reach the memory; the memory tells an
  occupancy no measured return has hit from a surface and RViz draws it grey
  (specification K14, K26, 5934e5ca); every frame of a recording shows its
  own moment of the flight (A11, 633e77a1). The acceptance on 18ba583c, the
  stereo set at 1.0: 1.822 to 1.995 m/s (mean 1.897), the true position 0.76
  to 1.21 m from B at its acknowledgement, no contact, every flight counted
  (r1186 to r1190). The long flight to B and
  back and the goal-outside flight are set aside (A8).
- **A floor touched in flight is a crash, the clearance law carries the
  estimate's vertical error, the way home at 1.5** (2026-10-09, A9, K27,
  K11, 0a332c4d; the offboard split into three sources, acaa4b1f; its mode
  stream kept through a planner-loss hold, b48f89eb). The acceptance on
  b48f89eb, the stereo set at 1.0: 1.640 to 1.878 m/s (mean 1.750, the
  clearance law under K27 holding the reference under 2 m/s in 27 percent
  of the ticks against 11 before), the true position 0.74 to 1.66 m from B at
  its acknowledgement, no contact, every flight counted (r1197 to r1201). The four scenarios recorded again on it are in
  `log/videos/2026-10-09/` (r1191, r1194, r1195, r1196): the low battery now
  turns 72 s in with 140 s against 128 estimated and is home with about 81 s;
  two light-lost takes (r1192, r1193) tipped on uneven ground four seconds
  after a level touchdown, the known risk of the blind landing
  ([technical_debt.md](technical_debt.md)). r1194 stood, on a slope of 12
  degrees, and its estimate ran 100 m away in four seconds: the rotor-drag
  model read the ground's friction (2.3 m/s² across the rotor axis) as 21
  m/s; readings beyond any flight's drag are left out now (K13, 208ab74e),
  and the acceptance on 5dd839b4 holds: 1.576 to 1.772 m/s (mean 1.700), the
  true position 0.79 to 1.79 m from B at its acknowledgement, no contact,
  every flight counted (r1205 to r1210). The
  light-lost scenario was recorded again as r1211 (89ac0f10: the overlay's
  `LANDED` by the truth, the recording ending with the mission's result).
- **Flights under the host's load** (specification A4): a flight is watched
  while it flies and stopped when the load slows it, and flown again when
  the host is quiet (`tools/fly_until_valid.sh`, `tools/record_until_pass.sh`).

Known to remain. The lidar vehicle's return from a goal outside the location
is not flown: the proof cannot hold a closure the size of a location and the
lidar carries no light battery (F15, the owner's decision). Dead reckoning
still moves the vehicle, 0.6 to 1.0 m in the 6 to 8 s to the ground: a hold
nearer a wall than that is not covered. An outage in the estimator's first
seconds on the pad is not survived; the scenarios do not inject one. The
shaft room costs the lidar's speed its tail. No smoke (item 18), no moving
obstacle (item 21). The register with the class of every entry is
[`technical_debt.md`](technical_debt.md).

**Repaired after** (the technical-debt goal, 2026-10-10 to 13): the frames
warn of a fading light, the contract reading the light's lit range over
the depth and its share against the gain the driver applies (K15); the
time-of-flight ring keeps surfaces off the body from the first moment of
dead reckoning through the level descent, closed on the range it reads
(K19); the ring's units carry the cameras' visibility mask. Accepted on
the final commit: the light lost five times (r1384 to r1387, r1389: whole, 0.74 to 1.14 m of drift in 5.3 to 8.6 s of dead reckoning, the body's least true clearance 0.26 to 0.56 m), the loss beside a wall five times (r1390 to r1394: no contact, 0.60 to 1.23 m of drift, the least clearance 0.22 to 0.49 m), the light's failure five times (r1397, r1402 to r1405: home 0.77 to 1.28 m from the start by the truth). r1387 lay armed on the staging platform's edge after its blind landing: PX4's land detector, which a quarter of the thrust on an edge does not satisfy.

### 18. Flight Through Transient And Scattering Obstacles (Completed)

Shipped in [v0.6.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.6.0) on 2026-10-24; closed on 2026-10-17; accepted on 2026-10-16 (76367bee) on both
sensor sets and flown again on the stereo set on 2026-10-17 (7bc3941b, the
estimator's heading and the pair's principal point repaired). Smoke is a model
the sensors read (A13); the lidar reads a plume by its echo and thin smoke
through it (K37), the stereo set detects smoke on its time-of-flight ring
(K38); the blocked corridor sends the lidar set home by the proof (the
ladder that sent the stereo set home, K40, was withdrawn by the owner on
2026-10-18: smoke is what makes space not free, nothing more), the smoky
location is crossed without contact. What was asked for and how it was built is kept below as it was
written; this is what came of it.

**Type:** dependent realism stage, with one repair that does not wait for it.

**Hard prerequisites:** item 17 stage 0 for stages 1 to 4 (smoke is detected
through the measured range); item 17 stage 8 for stage 0 below, whose fail
evidence and restated invariant it applies to smoke and the lidar. The
memory's decay by time, which this item was first written on, is switched
off since 2026-10-03 (specification K10): how a mark of unobservability
fades is settled here with item 21's transient class, not assumed.

**Validation environment:** Urban Circuit Practice 01, the point-to-point
mission, on both sensor profiles.

Two readers of the v0.4.0 release found the same hole from opposite sides. One
asked what the obstacle memory does with a person walking through the frame;
another asked what smoke would do to the vision. The answer to both is the
same, and it is not about the sensor: an obstacle that was there and is not
any more stays in the map. Smoke adds two things of its own — it blinds the
active sensors as well, and it looks like a surface — which is why it gets an
item and not a paragraph.

**What the memory already does, stated precisely, because this item was
first described wrongly.** `obstacle_memory_3d.cpp` scores every voxel by
hits and misses: a free ray through a voxel lowers its score by
`miss_weight`, and a Schmitt trigger returns it to free at `free_score`
([`obstacle_mapping.md`](obstacle_mapping.md)). A person who walks away is
therefore cleared as soon as rays pass through where they stood.
`forgetDynamicVolumes` erases the volumes of known dynamic agents, the
cooperative vehicles whose positions arrive by intent. What is missing is
narrower than "no forgetting": clearing needs re-observation, so a voxel
never looked at again is occupied forever, and nothing distinguishes an
occupancy confirmed by a thousand scans from one that collected two hits and
vanished. The planner treats both as a wall — a person crossing the frame is
a route replacement, and a trail left where the vehicle does not return is a
wall for the rest of the flight. Smoke is the case that rays cannot clear at
all while it lasts, because to a stereo pair it is a surface.

Three independent axes, decided by the project owner on 2026-09-23, so
that no one builds the item as a mode of one sensor. **Smoke is a switch of
the scenario**: particle emitters in a variant of the location, over the
ordinary point-to-point mission, which does not change. **The perception
profile is the vehicle's**: the stereo set or the 3D lidar, either one flown
into the same plume. **The thermal channel is an addition to the vehicle**,
not a part of the smoke scenario: it belongs to the sensor set that the
scenario tests, and a flight may carry it on either profile or on neither.
The acceptance matrix is the product of the three, not a list of modes.

**What the flights look like once the item lands.** Decided by the project
owner on 2026-09-27, and where it differs from the paragraph above it
replaces it:

- **The smoke sensors are always on board**, on both profiles: whatever
  stages 2 and 3 settle on — the thermal channel, the particle counter — is
  part of the vehicle, not an option of a flight. As settled (F4, 2026-10-17):
  each profile's smoke sensor is its own detection channel, the time-of-flight
  ring on the stereo set and the echo's intensity on the lidar; no particle
  counter; the thermal core, measured in stage 3, is off the vehicle since
  2026-10-17 (R6) and returns with item 32, a heat source in the plume.
- **Smoke is always in the location**: a number of smoky places in every
  flight, so that every acceptance flight exercises the smoke handling, and
  none of them blocks the way from A to B (built on 2026-10-17: the ordinary
  flight's default carries the dense plumes and item 23's moderate rooms,
  `LOCATION_SMOKE_ZONES`; before that smoke was a scenario's switch, which
  the owner's rule of 2026-09-27 overrides); the ordinary mission stays
  reachable. Withdrawn from flight on 2026-10-24 by the owner's rule (F4):
  the ordinary flight is the clear location and no smoke flight is flown;
  the smoke targets stay as the record.
- **A separate scenario blocks the way**: smoke closes the passage to B, no
  route exists, and the vehicle flies home. It exercises the smoke handling
  and item 19's return together, by a cause the vehicle measures.
- **Smoke is drawn in RViz**: every smoky place of a flight is a marker in
  the picture, published by an evaluation component the vehicle never
  reads, so that a person watching sees what the location holds.
  Decided by the project owner on 2026-10-03.
- **Smoke is constant**: where it is and how much of it there is never change
  during a flight, so that the difficulty of the location does not change
  with time. Its shape does, as a smoke grenade's or a local source's plume
  churns, because that is what smoke looks like and what the sensors have to
  cope with. A mark of smoke in the memory is transient (item 21's class): it
  lifts when the vehicle sees through the place again, and how it fades when
  the vehicle does not come back is stage 4's question.

The blocking scenario is where item 19's proof is flown: its flights must
give the goal up by the proof (`trigger=topological`), not by the light's
battery or the light's judgment, and they close item 24.

The navigation invariants hold in the form restated for this item, below.
Vertical motion stays free, and nothing keyed on the vehicle's history is a
rule.

### Stage 0: Smoke And The Lidar In The Memory (Done)

**Built on 2026-10-15** (`lidar_scan_patterns_3d.cpp`, specification K37):
the organized scan is read for its patterns before the memory integrates
it. An aerosol is a hit whose intensity is under half the scan's surface
intensity (the median of its hits') with three or more of its eight
neighbours weak hits within a metre of its range: the beam becomes a miss
to the return and the return observed unobservable, a surface never
measured. A dropout is a bundle of no-return beams of 400 or fewer whose
border returns in three quarters of its beams at a median range of 8 m or
less and a spread of 1 m or less from the first to the third quartile: the
bundle's beams become misses to that range with fail evidence there; a
bundle whose border recedes is an opening, and a no-return beam among
no-return beams stays free to the maximum range. The smoke relay writes the
intensity a lidar's driver reports, the echo over a matte surface's at the
range (one for a surface, a thin fraction for smoke), where the cloud
carries the field. The dropout by material stays designed and unverified:
`gpu_lidar` returns geometry whatever the material.

**Accepted on 2026-10-15** (2c223640): cameras r1517, r1518, r1519, r1522,
r1523 at 1.467, 1.438, 1.451, 1.234, 1.419 m/s (mean 1.402; 1.429 before
the stage, r1485), the goal 0.87 to 1.85 m by the truth, no contact; lidar
r1524 to r1528 at 2.476, 2.577, 2.086, 2.357, 2.807 m/s (mean 2.461
against the base's 2.333, two of five under 2.4 then as now: the clearance
laws hold the lidar in the same places, the register's K27 entry), the goal
0.42 to 0.95 m, no contact. Dropout bundles were read in 4 to 8 of some 165
logged scans a lidar flight (135 to 608 beams), with the limiter profile
unchanged against the base (route clearance 40 to 47 percent of the slow
time, 40 to 44 before). Voids on the way: r1520, r1521 (watchdog at 0.68
and 0.69).


Item 17 stage 8 builds the memory's third kind of evidence, **fail**, and
the invariant restated around it, for darkness. This stage applies them to
what this item adds: smoke on every sensor, and the patterns in which the
lidar fails. A person crossing the frame is transient occupancy, item 21's
class, cleared by the next look and not by its age.

- **The lidar.** A single ray with no return is ambiguous: nothing within
  range, or something that absorbed or deflected the beam — black smoke, a
  matte black surface, glass at an angle, water, a grazing wall. One ray
  cannot tell them apart and the sensor does not know what should have been
  there; only the pattern of returns can. Two patterns are failure: an
  **aerosol**, weak, near, scattered returns across neighbouring rays (the
  `particle_scatter_ratio` model), with the space behind them unobservable;
  and a **dropout**, a bundle of rays with no return surrounded by rays that
  return at moderate range. A ray with no return whose neighbours have none
  either is open space and stays a free ray to the maximum range, as now.

That last line is a change to the lidar's free-ray logic and it belongs to
this stage: a missing return is a miss to the maximum range only when it is
not a dropout. It carries a bonus the simulator cannot check. A matte black
wall that returns nothing is a classic hazard of lidar navigation — the
memory as it stands would integrate it as free and the vehicle would fly
into it — and under the restated rule it is unobservable and closed. But
`gpu_lidar` returns geometry whatever the material, so the case never occurs
in this simulator; the repair is recorded as designed and not verified until
a dropout by material is modelled.

### Stage 0b: Unobservable By Direction, On The Stereo Pair

Added by the project owner on 2026-10-03, and built first: everything the
stereo set does with a local plume depends on it.

The stereo pair does not say "I looked and nothing is there". It gives a
depth where it matched and is silent everywhere else, and its silence is one
for every cause: too far for the light, no texture, smoke, a surface that
swallows the light. Item 17 taught the stack to read that silence for the
frame as a whole (specification K9, K14): a frame that observes too little
is blind, and what it looks at is marked. A local plume fills a part of the
frame. The frame as a whole stays healthy, the directions without depth are
unknown, unknown is free (I1), and the vehicle flies into the plume. The
same hole lets a uniform panel across the route into the memory as nothing
(the register's textureless-surface entry, r783). The lidar's half of this
is stage 0's dropout pattern; the pair has none.

What tells a silent direction that is far from one that is closed, by
measurement only:

- **Its neighbours.** A patch without depth whose border returns surfaces at
  two or three metres, with a sharp edge, is not a corridor: an opening
  shows something of what lies behind it at its rim.
- **The approach.** An opening opens as the vehicle nears it: the light
  begins to reach, depth appears behind the rim. A patch that stays silent
  while the range to its border falls under what the light carries is
  closed. This is the strongest sign and it arrives late, so the speed
  toward a silent patch is bounded by the range to its border, as the
  contract bounds every other motion.
- **A second sensor.** The time-of-flight ring is active and reads in the
  dark to 4 m: a return behind a patch the pair is silent on says the pair
  failed, not the world. An absorbing surface silences both.

A direction judged closed is marked unobservable in the memory, the fail
evidence of item 17, ray by ray and dense enough to close what it covers
(the frustum marking of K14 leaks between its rays, r816).

**The test object is a uniform matte panel** (`BLANK_PANELS`, already in the
repository since item 17 stage 2): lit by the vehicle's own light it is an
even bright patch with nothing to match, close to what a plume's veil looks
like, and it has a collision, so a rule that fails is a contact and not a
harmless pass through air. The rule is settled on it before smoke adds a
shape that churns.

Known and not attempted: a surface that returns no light at all, a matte
black one, looks to the pair like the unlit far end of a corridor until the
vehicle is close. One stereo pair cannot tell the two apart at range; the
speed toward a silent patch, bounded by the range to its border, is what
covers it.

Complete when, on five flights with a panel across the route and five with
one beside it, the vehicle does not fly into the panel, routes around it
where a way exists, and the memory holds it closed from every side it was
seen from.

**Measured and settled on 2026-10-14** (specification F13 rewritten, K36;
`stereo_silent_patches.cpp`, `tools/panel_memory.py`). On the code before
the stage a 3 x 2 m panel across the route entered the memory by its hits
as it is, 44 percent of its face 8 s after the vehicle came within 6 m and
96 percent by the end, and was flown over 0.89 m clear; beside the route 79
percent, passed 0.82 m clear (r1491, r1495): the light's falloff and the
panel's edges give the matcher some structure, two hits make a voxel, and
the ring sees it at 2.8 m. The marking this stage was written for, a silent
patch closed ray by ray where its rim is matched within reach, flat and
round it, closed the dark location when flown: the lamp's shadows, dark
corners and the ceiling bounded by lit walls are such patches too, 417 of
476 and 671 of 764 frames closed somewhere, five million marks, the mission
lost to a loop and a hold (r1496, r1497). It is a prohibition from a wrong
measurement and is not built. Kept is the stage's speed law: a closed patch
holding the frame's centre within 15 degrees bounds the braking contract's
range along the heading by its rim's depth, never under the margin
(`/stereo_depth/silent_range`); past the panel 1.300 m/s with the bound in
7 percent of the frames at 0.5 to 4.4 m (r1498).

**Accepted on 2026-10-15** (944241d3; the memory read by `tools/panel_memory.py`
from each flight's bag):

| Flights | Runs | Result |
|---|---|---|
| Panel across the route, five | r1499, r1500, r1501, r1503, r1504 | no contact, the body 0.88 to 2.63 m clear of the panel, the route over or beside it; the memory held 45 to 72 percent of the face occupied 8 s after the vehicle came within 6 m and 44 to 75 by the end; 1.29 to 1.41 m/s, the goal 0.83 to 1.32 m by the truth |
| Panel beside the route, five | r1505, r1506, r1509, r1510, r1511 | no contact, 0.58 to 3.88 m clear; the memory 77 to 106 percent of the face from the side seen; 1.38 to 1.53 m/s, the goal 0.83 to 1.44 m |
| Ordinary, five | r1512, r1513, r1514, r1515, r1516 | 1.38 to 1.45 m/s (r1485 before the stage: 1.43); no contact; r1514 acknowledged its goal 2.32 m from it by the truth, over the 2 m capture radius: the estimate's drift with the pair's noise (A12), 1.2 to 2.0 m at the end of a flight against 0.7 to 0.9 before it, with the tracked features and the residuals unchanged (p50 170 features, 0.48 px against 0.41); r1495 missed by 2.44 m the same way. The register holds it. |

Voids on the way: r1502 (watchdog), r1507 and r1508 (real-time factor
0.80 and 0.82 under the bag's load); r1492 and r1493 not flown.

### Stage 1: Smoke In The Simulator (Done)

Done on 2026-10-14 (specification A12, A13; the measurements in
[`obstacle_mapping.md`](obstacle_mapping.md), "Smoke In The Simulator", the
tool `tools/smoke_rig.py`). Gazebo's `ParticleEmitter` did not serve: its
particles are unlit, so a camera sees them in the dark and the light's place
does not matter to them; a depth sensor stops on the first one by a fixed
chance whatever the path, and `particle_scatter_ratio` does not set that
chance; the evaluation's depth camera scatters too; `<scene><fog>` acts on no
sensor; the thermal camera never sees them. Smoke is therefore a model, one
field per smoky place that every sensor reads through a relay beside the
flight: the pair by single scattering of the carried light, the time-of-flight
ring and the lidar by the lidar equation (`SMOKE_ZONES`; the places are
drawn in a recording's world half, never in RViz, since 2026-10-18). On the way the cameras' noise was found a tenth of a level where
item 17 stage 1 had meant 2.5, and set to a small sensor's shot noise, about
one (A12).

What the vehicle meets: a plume is silence to the pair, not a surface (a wall
5 m away is lost behind about one optical depth, smoke is never matched);
the lidar sees through 0.3 per metre and returns from the smoke's face from
0.5; a time-of-flight unit sees smoke only round itself, from 1 per metre;
the carried light set apart from the pair lowers the veil and does not give
the wall back; the camera estimator tracks half its features in a plume and,
flying 11 m through 3 m of smoke, ends 0.1 to 2.2 m off (3.6 m at the worst),
through 7 m of it up to 4.6 m.

The question as it stood: Gazebo already carries the tool, built for this
purpose, the `ParticleEmitter` system of the DARPA SubT virtual track, and its
`particle_scatter_ratio` sets how the particles reach each sensor; the
vehicle's own light in smoke, which the location's darkness makes the only
light, scatters straight back into the cameras, and whether that helps or
harms, and a light set apart from the pair as the remedy, were the first
measurements.

### Stage 2: Detecting Smoke (Done)

**Measured and built on 2026-10-15** (specification K38). On the rig the
ring inside smoke of 1/m returns from every zone at 0.05 m with 0.012 of a
surface's intensity, and nothing from a plume ahead until it is inside
(the lidar returns from it at 0.3/m already); the pair matches nothing
through 0.3/m and more. A zone returning under half a surface's intensity
is smoke: its rays are fail evidence, and the share of smoky zones is
published. The pair-against-ring signature has nothing to
read in the simulator; no particle counter is mounted.

**The smoke face, built on 2026-10-19 and flown to 2026-10-24**
(specification K44, F19; the owner's model of 2026-10-18: dense smoke is an
obstacle the vehicle sees and never enters). The pair is silent on a dense
plume under the carried light, and that silence is lit (raw grey 20 or more
against 0 to 10 for shadow and the dark, F13); but so is the lit untextured
floor of a hall and the veil of thin smoke, and no look of the region (its
rim's scatter or trend, the chance matches through it) tells them apart
(r1822 to r1937, 1 472 laid regions judged against the truth). What tells
them apart is the motion: the plume is the frame's largest lit rimmed region
of 15 000 cells or more in its central band, matched through at a quarter
percent at most, whose rim depth in the central rows falls by 0.3 m or more
per metre of the vehicle's travel while its size grows by 0.3 of itself (or
0.1 with the depth falling 0.8) over 1.2 m, the vehicle neither climbing
nor sinking by more than four tenths of that travel; the veil keeps its
distance and its size, the floor comes nearer only as the vehicle sinks.
Its rays enter the memory observed unobservable from 1.5 m before the face
(the halo that blinds the pair) to 1.5 m beyond, and the motion is bounded
1.5 m before it, a bound fallen to nought being a stop (r2047 read it as
none and flew 0.9 m into the box); the ring's silence through lit silence
is not read as free. The lamp reaches a dense face from 3 m only, and the
smoke's churn can carry the halo 9 m out, where the candidate's depth
reads 1.4 to 2 m from 10 m on and never falls (r2045): so the plume is
laid as well, at once, when the pair's reach (the farthest metre of the
frame with hits in it and the one before) has fallen to 4 m or under while
a lit silence of half the frame fills it; the thin hall's dark frames keep
hits at 5 to 8 m, the plume's halo none beyond 2 to 4 (on no status line
of nine ordinary flights, on every blocked approach at 6.2 to 7.9 m from
the box). The acceptance on the final rules, 71d03da5: the blocked
corridor 5 of 5 (r2049 to r2053: the plume laid 1.8 to 3.9 m before the
face, the vehicle turned 1.2 to 3.2 m before it, came home on the light's
charge 0.3 to 0.5 m from the start, phantoms 0.3 to 0.7 percent of the
hall's occupied), the ordinary flight 5 of 5 (r2054 to r2058, 1.23 to 1.37
m/s, B 0.4 to 1.6 m, no entry into the halls' plumes, phantoms 0.3 to 1.9
percent; one to four false lays of a frame each in the thin halls, 0.4 to
4.8 s of hover a flight), the clear location 5 of 5 (r2061 to r2065, 1.40
to 1.51 m/s, B 0.5 to 1.5 m; a lit wall laid for a frame or a second in
two flights at no cost in hover, nothing in three) and the lidar 5 of 5
(r1983 to r1987, 2.23 to 2.38 m/s; the stereo node does not run on the
lidar set, which the last two rules do not touch). Before those two rules
the blocked corridor was 5 of 5 on e549fd41 (r1961 to r1965) and on
02dd2679 (r1996 to r2000) and 7 of 10 on 18fff7af (r2033 came home late
and the light ran out with the planner at no route; r2044 and r2045 had
the plume laid once or not at all on the approach and crept into its halo
at the end of the search, blind, and K19 landed them), the ordinary flight
5 of 5 on 09f68f22 (r2003 to r2010, 1.28 to 1.44 m/s). What stays open: a
plume's marks are lifted by the chance matches through its halo (I2), so a
vehicle creeping back into a laid plume has the halo's reach alone to lay
by, from 1.8 m before the face; the rules of the motion that keep the thin
hall's veil and the lit floor out (the growth, the climb) refuse some
approaches to the real plume (r2015, r2031, r2045), which the reach then
lays with 1.2 m to spare at 1.8 m/s. Since 2026-10-24 the face is off unless
asked for (`smoke_face`, `SMOKE_FACE=true`; the owner's rule): the clear
ordinary flight carries no smoke detection. Its cost there, measured on
d979c92d: five clear flights with it off 1.343 to 1.456 m/s (mean 1.417, B
0.58 to 1.30 m; r2066 to r2070), five with it on 1.330 to 1.522 (mean 1.432, B
0.73 to 1.64 m; r2071 to r2073, r2076, r2079), no false lay in either group and
the depth node at 2.2 to 2.4 cores in both: nothing measurable. The standing
of the stereo detector, as v0.6.0 states it: a detector of smoke itself was
not reached on the pair (the scattered light of a dense face under the lamp
does not separate from a lit blank surface or the thin veil in the frame);
what stands detects the pair's lit silence near the vehicle, passes this
location's acceptance, reads a close lit wall as a plume now and then, has
the lamp's 3 m of reach and this location's thresholds, and is not a smoke
sensor. The lidar's detection (K37) is the sensor's own and holds anywhere.
Its limits: a plume of 2 to 3 m at
1/m (optical depth 2 to 3) the pair matches through at 2 to 6 percent is
laid only from 1.2 m, and the vehicle grazed one by 0.3 m creeping (r1966)
and one by 0.12 m spinning in place (r2002); a false laid region in a thin
hall's narrow place still closes the way (r1953, r1971, r2002, each
repaired by a rule of the motion). The confirmation is replayed offline on
every frame's logged candidate (`tools/plume_replay.py`), the laid regions
judged against the truth (`tools/smoke_marks_truth.py`), the acceptance
flown by `tools/smoke_series.sh`.


Free only where smoke fills the frame. Item 17 stage 0 already turns the
contract's forward range into a measurement, and smoke all round is one more
way that range collapses: the vehicle slows and stops without a classifier.
A plume in a part of the frame does not collapse it, and is stage 0b's. Two signatures separate smoke from a
wall where that matters: the time-of-flight zones report signal rate and
ambient per zone, and smoke is a low signal spread over every zone at short
range in every direction, which no wall is; and a surface the pair sees that
the time-of-flight ring does not, or that moves, is not a surface. An optical
particle counter of the Sensirion SPS30 class — about 25 USD, grams — is the
independent channel, and it is compared against the free ones before it is
mounted.

### Stage 3: A Thermal Channel (Done)

**Measured and settled on 2026-10-15** (specification K39, the rig's
thermal pair of the Boson+ 640 class): thermal stereo matched nothing with
the wall 3.7 m away and 0.46 of the frame at 7.7 m, every match on the
wall's edge against the floor and none within a voxel of the truth, clear or
through 1/m of smoke alike. The world's surfaces carry one temperature each,
so the band has no texture; the result is conditional on a texture the
simulator does not model, as the stage foresaw. The vehicle carries one core
of the Lepton 3.5 class as a detector (`models/stereo_tof_v1`, 160 x 120 at
9 Hz, relayed as the pair's frames are), and the depth node reports the
share of its pixels 5 K warmer than the frame. The ladder of cheapening is
not flown: there is no stereo to cheapen. The share is published and read by
no decision: the location has no hot source for it to find. On 2026-10-17
the owner took the core off the vehicle (F4, R6: a part that decides nothing
earns no place on a vehicle made in numbers); the measurement stands, and
item 32 brings the core back with a reading and a rule of its own.


Longwave infrared (8 to 14 µm) is the one passive sensor that sees through
smoke: the wavelength is far larger than the particles and there is little
scattering. It is why firefighters carry it, and the simulator models it: the
thermal camera is the sensor type the particle emitter does not touch.

**Price is not a criterion of the exploration, and it is a criterion of the
accepted build.** Both halves were decided by the project owner on
2026-09-23. The first follows from the cost section below: the thermal
channel is an addition to either sensor set and competes with nothing, so the
comparison that made price matter in item 17 does not exist here, and the
stage takes the best and most suitable core first to learn what the problem
needs — if the best cannot make smoke workable, no cheaper core will, and a
cheaper core is a harder problem taken up afterwards, not a simplification
lost. In the simulator that costs nothing: a thermal camera is a resolution,
a rate and a field, so the best core and the cheapest fly in the same series
and the downgrade is measured beside the target rather than deferred.

The second half is why the exploration is not the target. The simulated
vehicle stands for an industrial prototype with a bill of materials, and a
build nobody would assemble proves nothing about the product: a pair of
Boson 640 cores is 7 100 USD on an airframe whose whole sensor set is 80 to
170. The item therefore completes on a core of a class someone would mount,
and every result records what in it rests on the resolution and the rate, so
that the distance between the best core and the accepted one is a measured
loss and not a redesign.

The role, and it is the ambitious one because price no longer argues
against it: **thermal stereo, a pair of the best cores, for metric depth
through smoke.** That is the only role that returns flight through a plume
rather than an informed stop, because it is the only one that gives the
braking contract a measured range. It is its own hard problem — the band
carries little texture, matchers built for visible light do poorly on it,
and the cheap cores run at 9 Hz — and the stage measures whether it works
before anything is built on it. The detector role is what remains if it does
not: one core, no depth, answering whether there is a surface or a body
behind the smoke and whether what the pair sees is smoke or a wall.

**The core the exploration starts from: a synchronised pair of Teledyne
FLIR Boson+ 640** — 640 x 512 at a 12 µm pitch, NETD under 20 mK, 60 Hz, an
external frame-sync input, controllable flat-field correction, and the widest
lens of the family, about 95 degrees horizontal, on the 0.20 m baseline the
visible pair uses. About 7 000 to 8 000 USD the pair. Nothing above it is
worth the exploration: a cooled mid-wave camera is better in noise and speed
but carries a Stirling cooler — hundreds of grams, tens of watts, from
20 000 USD — and is not a drone even for an experiment, and its advantage
does not reproduce in the simulator; the 1280 x 1024 uncooled cores that
exist add pixels where the bottleneck is not, since 640 x 512 is already
forty times fewer pixels than the visible pair and the limit is the texture
of the band. In the simulator the Boson+ 640 is a bound on the parameters,
tied to a device that exists — 640 x 512, 60 Hz, 95 degrees, 20 mK of noise —
so that the exploration does not drift into a sensor nobody makes.

**Four properties of every microbolometer decide suitability more than the
module does**, and three of them reach the braking contract:

- **The shutter.** A microbolometer closes a shutter for its flat-field
  correction, for hundreds of milliseconds every tens of seconds or on a
  drift of its own temperature, and delivers no frame meanwhile. To this
  stack that is a hole in the evidence age — the constant that item 17
  stage 0 turns into a measurement — so any core in the speed path must
  expose control of its correction, to schedule it into a hover, and the
  measured age must cover the gap. The FLIR cores expose it; the Chinese
  cores vary. This is a selection criterion, not a detail.
- **Texture in the band**, the chief risk of the stereo role, and
  independent of the module. Longwave infrared sees differences of
  temperature and emissivity; a corridor of evenly warmed concrete is smooth
  in the band however cracked and stained it is to the eye, and no
  resolution repairs that. It has a consequence for the simulator that is
  written here so that it is not forgotten: Gazebo's thermal camera renders
  the temperature of visuals that carry one, the imported SubT world almost
  certainly carries none, and so the materialization variant of stage 1 will
  have to tag its surfaces with a temperature the way item 17's variants
  change their materials. The thermal texture is then **ours to choose**, and
  the answer "does thermal stereo work" is conditional on what we chose. The
  simulator cannot say how much thermal texture a real corridor has; the
  literature and a core in hand can. The first action of the stage is that
  check of the world's thermal model, and every stereo result of the stage is
  labelled as conditional on the assumed texture.
- **Frame synchronisation.** Stereo on a moving vehicle needs synchronous
  frames. Boson has a sync input, Lepton 3.x a VSYNC line, the Chinese OEM
  cores usually one of the two, the Chinese consumer USB modules none. The
  detector role does not care.
- **Field of view.** The visible pair covers 120 degrees; a Lepton 57, the
  Chinese cores about 56, a Boson with its widest lens about 95. A narrower
  field shrinks the contract's faced zone, so more of the motion becomes
  unfaced and answers to memory; the gaze policy compensates in part, and the
  cost is measured.

The thermal time constant of a bolometer, about 10 ms, is not a criterion:
at 2.45 m/s and a yaw rate of 80 degrees a second it is two to four pixels of
blur on a 640 core.

**The modules, judged against the two roles and the two phases:**

| Module | Detector | Stereo, exploration in the simulator | Accepted build | Verdict |
|---|---|---|---|---|
| FLIR Lepton 3.5 | the natural choice: 164 USD, a gram, 150 mW | the lowest point of the curve | yes, as a detector | fits, as a detector |
| FLIR Boson 320 | more than the role needs | the middle point | no | simulator only |
| FLIR Boson+ 640 | more than the role needs | the top point, the core above | no | simulator only |
| FLIR Hadron 640R | no | no | no | excluded |
| Chinese 256 x 192 (InfiRay class) | yes, and better than a Lepton: 2.6 times the pixels, 25 Hz | marginal | yes, as a detector | fits, as a detector |
| Chinese 640 x 512 OEM | more than the role needs | yes | **the one stereo candidate**, on a quote and a sync line | fits, conditionally |

The Lepton is not a stereo core: 160 x 120 is sixty-four times fewer pixels
than the visible pair, and the depth would be sparse and noisy — though its
9 Hz is not what disqualifies it, since the visible pair flies at 7.5. The
Hadron is the Boson 640 with a 64-megapixel visible camera in the same
housing: the visible half duplicates the pair, the pixels are useless to
this pipeline, radiometry adds nothing to geometry, and two of them are
8 000 USD. The Chinese 256 x 192 modules are consumer USB devices whose
drivers and software are a lottery and which carry no sync between two of
them, which the detector does not need and stereo does. The Chinese
640 x 512 OEM cores are the only path to thermal stereo in a build someone
would assemble — 1 000 to 1 600 USD the pair, the same order as a lidar, and
nothing below that exists — on two conditions: a quote that lands in the
lower half of the range, and a frame-sync line, which OEM cores usually
carry.

**The ladder of cheapening.** Each step changes one variable, so that the
loss is attributed to it and not to "worse"; the ladder is also the ablation
of the stereo role.

| Step | Core | What it isolates | The pair, USD |
|---|---|---|---|
| 0 | Boson+ 640, 60 Hz, 95 degrees, synced | the reference | 7 000 to 8 000 |
| 1 | the same core, the 9 Hz variant | frame rate | about 5 000 to 6 000 |
| 2 | Boson 320, the same lens family | resolution | about 3 100 |
| 3 | Chinese 640 x 512 OEM | the core itself: noise, sync, correction control, software | 1 000 to 1 600 |
| 4 | Chinese 256 x 192 | resolution and synchronisation together | about 600 |
| 5 | one core: Lepton 3.5 or Chinese 256 x 192 | the role: stereo becomes a detector | 164 to 300 |
| 6 | no thermal core | the floor: stage 2 alone, the ladder without seeing behind the plume | 0 |

Steps 1 and 2 answer what thermal stereo actually needs, rate or pixels.
Step 3 is the accepted build's target point: on paper the same specification
as step 0, in the hand a different noise, a different sync, a different
correction control and different software, and it is where thermal stereo in
a plausible build is decided. Step 4 is where stereo most likely dies. Step 5
is the change of role, and still better than nothing. Step 6 is what the
item gives without a thermal core at all.

The simulator varies **resolution, rate, field and noise**, so steps 0, 1,
2, 4 and 5 fly in one series as parameters. It does not vary the reliability
of a sync line, the control of the correction or the quality of a driver —
what actually separates step 3 from step 0 — and those are learned only with
a core in hand. In the simulator step 3 is therefore step 0 with 40 to 50 mK
of noise, and it is recorded as that assumption and not as a result.

Prices, single units, 2026, for the record and for the day hardware is
scheduled, not as an input to the exploration:

| Module | Resolution | USD |
|---|---|---|
| FLIR Lepton 3.5 | 160 x 120, 57 degrees, about 9 Hz | 164 |
| FLIR Boson 320 | 320 x 256 | 1 539 without a lens, up to 2 549 |
| FLIR Boson 640 | 640 x 512 | 3 558 |
| FLIR Hadron 640R, thermal beside visible | 640 x 512 | 3 992 to 4 330 |
| Chinese cores (InfiRay class), 256 x 192 | | about 300 as finished goods |
| Chinese cores, 640 x 512 | | roughly 400 to 1 700, OEM quotes not published |

### Stage 4: What The Vehicle Does (Withdrawn)

**Withdrawn on 2026-10-18** (the owner; specification K40): the vehicle has
no rung, hold or return of its own for smoke. It has no notion of a closed
way, only of free space, in which it lays its route; smoke is a measurement
that makes space not free (K37, K38), the way round a plume is the planner's,
and a goal is given up by the proof of item 19 alone, never for the smoke
itself. A count of plumes was a latch on the vehicle's history (I3). On the
stereo set the proof cannot close a corridor a plume fills, so the blocked
scenario is met on the lidar alone; a stereo vehicle in a plume is a blind one
(K12, K19). What was built is kept below as it was written.

**Built on 2026-10-15** (specification K40; flown in this item's
acceptance): a vehicle in a plume (blind with half the ring smoky, or
blind and dead reckoning under a light it judges reliable: at a plume's
edge the ring returns smoke from a few zones only, r1534) backs out on
dead reckoning along its heading through K19's hold, holds 30 s
where it sees again after the first plume and takes its goal back, and
gives the goal up for the start after a second (r1532 landed in the smoke
before any retreat; r1533 backed out twice and wandered 800 s, item 19's
proof exhausting its budget through the unknown beyond the plume). A
plume's marks are fail evidence lifted when seen through.

The ladder of item 17 stage 5, with the rung that item added between holding
and landing: **retreat along the flown path**. The path just flown is in
memory as observed and free, and for smoke as for darkness a vehicle that
backs out to where it could see is better placed than one that lands where it
cannot. Through dense smoke the visible cameras do not fly, and this item does not
try. What the thermal channel changes depends on which role stage 3
delivers: thermal stereo hands the braking contract a measured range through
the plume and the vehicle flies what that range admits, as it does in the
dark on any other sensor; a detector alone makes the stop an informed one,
with a retreat or a landing chosen on what is actually behind the plume.

With the fail evidence (item 17 stage 8, stage 0 here) the plume is a measured prohibition, and routing
around it is legitimate — not as a cost on free space, which stays
forbidden, but as the planner's ordinary answer to a closed region. The
sequence is then: the vehicle approaches, the range collapses at the plume's
edge, the region behind the edge is written unobservable, and the planner
replaces the route around it if one exists. If none exists the ladder runs,
and when the ladder is spent and the goal has been proven unreachable in
item 19's sense, the general return home of item 19 applies — a policy of
the mission, not of this item.
Two rules close the loop that the ladder alone leaves open. **After a
retreat the vehicle holds where it can see for a stated time** — the plume
is transient and expected to move — and if the range ahead has not returned
by then, it gives the goal up and flies home while it still has a position
source (when the way is proven closed, or on the camera profile when the
light's battery holds only the way back, item 17 stage 5), and it lands there, in sight, only when the start too is
proven unreachable; that is a rule of time on the vehicle's own state, not a
prohibition of space, and it is the same time the light of item 17 stage 5
is given to return. And the
closed region must not close its corridor for the rest of the flight once
the plume has drifted out of view. With the memory's decay by time switched
off (specification K10) nothing fades by age, and a place the planner will
not route into is a place the vehicle never looks at again. So a mark of
unobservability is transient in item 21's sense and carries its own rule,
settled in this stage by measurement: it lifts when the vehicle sees through
the place, and a mark not confirmed for a stated time returns to unknown,
which sends the vehicle to look again; the sensor then either sees through
or restores the mark. That re-approach is a probe bounded by the stated
time, not an oscillation, and the time applies to these marks alone, never
to what is static.

### What The Additions Cost, And Against What

The rule of item 17 — the additions must not approach the price of a 3D
lidar — does not apply here, and the reason is worth writing down. That rule
compares a substitute against what it replaces: the stereo set with its
flood and its ring against a lidar. Everything in this item is an addition on
top of either set. Smoke scatters a lidar's 905 or 1550 nm as it scatters a
time-of-flight pulse, and a thermal core costs the same 164 or 3 558 USD
whichever sensor it sits beside. So there is no comparative bar: each
addition is judged on whether it earns its place, charged once to both
profiles. What remains is the bar every addition of this project answers to,
that the vehicle stays a build someone would assemble; stage 3 explores at
the best core and completes on a plausible one.

One asymmetry keeps the separation from being perfectly clean. A lidar gets
part of its smoke robustness for free: it is active, so it needs no contrast,
and most 3D lidars carry multi-echo returns and per-return intensity, where
the last return passes thin smoke and a weak return is an aerosol and not a
wall. The stereo pair has none of that, and thin smoke that a lidar filters
already kills its contrast. The camera set's smoke addition therefore does
more work, and in the substitution comparison of item 17 it is the difference
between the two additions that is charged, not the whole.

### Measurement And Completion

Measure, per flight and per profile: the particle density along the path;
depth coverage and error against evaluation-only truth in and out of the
plume; the time-of-flight signal rates; the contract's forward range and the
speed it admits; the number and size of occupied voxels that were never true
occupancy, and how long each survived; route replacements caused by them;
the rung of the ladder reached and when; the minimum distance to true
occupancy; and physical collisions.

This item is complete when stage 0b's panel flights have passed;
when stage 0 has landed and both acceptance series
have been re-flown on it, with the route stability and the speed it costs
stated; when, **on the camera profile and on the lidar profile alike**,
five flights through the smoky location reach the goal in truth or retreat
and land without a collision, with no phantom occupancy surviving the
flight longer than the time stage 4 states for a mark of unobservability; and when, on both profiles, five flights of the
blocking scenario return to the start in truth without a collision, the
goal given up by item 19's proof (`trigger=topological`), which closes
item 24. Accepting on one profile would prove the
addition only on the set where it has the most to do and say nothing about
the set where it should be least needed.

**Accepted on 2026-10-16** (the vehicle's code of 76367bee, the documents
on top of it; the series driver and its tables in `log/tools/part6/`, the
journal in `log/tools/journal/goal_journal_smoke_light.md`). Every flight
below is on both sensor sets, one at a time, each counted only at a
real-time factor the host sustained (A7, A8; K25 where it says 0.8):

**The stereo set's groups flown again on 2026-10-17**, twice: on e9d26309
with the estimator's heading repaired (K28: clear r1688 to r1692 0.96 to
1.52 m from B at 1.31 to 1.49 m/s; smoky location r1693 to r1697 0.67 to
1.82 m; blocked corridor r1698 to r1702 home within 0.32 to 0.52 m) and on
7bc3941b with the pair's principal point at the pixel centres as well (K33:
the clear flight r1710 to r1714 ended 0.79 to 1.11 m from B at 1.38 to
1.46 m/s; the smoky location r1715 to r1719 0.77 to 1.57 m at 1.35 to
1.46 m/s; the blocked corridor r1720 and r1722 to r1725 home within 0.31 to
0.38 m by the ladder (`trigger=smoke`), 2 to 6 dead-reckoning episodes;
r1721 voided by the harness). No contact in any. The lidar's groups stand
as flown on 76367bee: both repairs touch the stereo set alone.

- **The smoky location** (`sim-urban-smoke-headless`, four plumes of 1/m,
  two of them in the start hall): the stereo set reached B in truth in
  r1653, r1654, r1655 and r1657 (0.84 to 1.79 m, at a real-time factor of
  0.8: 1.35 to 1.41 m/s of simulation) and in r1623 (1.22 m, at 1.0: 1.18
  m/s, the host at its limit); r1630 reached it 2.51 m off in truth, the
  estimator's drift of `technical_debt.md`. The lidar reached B in r1631 to
  r1635 (0.44 to 0.75 m) at 2.72, 2.15, 2.10, 2.36 and 2.20 m/s. No contact
  in any of them; the ring returns smoke only inside a plume (K38), the
  lidar sees a plume's face from 20 m (A13) and marks it (K37), and the
  plumes beside the route are flown round.
- **The blocking scenario** (`sim-urban-smoke-blocked-headless`, 3/m across
  the north corridor): the lidar gave the goal up by item 19's proof and
  came home in r1616 to r1620 (0.19 to 0.51 m from the start in truth,
  `trigger=topological`, within about 110 s of the plume's face); the
  stereo set by the ladder in smoke (K40, `trigger=smoke`; withdrawn on
  2026-10-18) in r1612 to
  r1615 and r1656 (0.30 to 0.42 m, 2 to 10 dead-reckoning episodes each).
  No contact. This closes item 24 (below).
- **The ordinary flight, re-flown on the final code** (the memory's K42 and
  the proof's K16 changed after part 4): the stereo set r1643 to r1647
  (0.92 to 1.56 m, 1.21 to 1.31 m/s), the lidar r1648 to r1652 (0.18 to
  0.55 m, 1.81 to 2.59 m/s, two of five under 2.4 as in part 4's base
  r1524 to r1528). No contact.

What the acceptance measured and what it did not: the speed in smoke is
recorded, not required here (item 23 sets the requirement under smoke); the
phantom occupancy inside a plume is the plume's own marks (K37, K38), lifted
when seen through (K10), and the ordinary memory holds no occupancy the
truth does not on the plain flights (`tools/phantom_blocks.py`: r1622 48
cells named, no phantom); the depth coverage and the thermal channel were
measured on the rig (stages 2 and 3), not per flight.

### 19. A Goal Proven Unreachable: Return Home (Completed)

Shipped in [v0.5.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.5.0) on 2026-10-10; closed on 2026-09-27 on bf92e952. When its goal is
proven unreachable, the vehicle gives it up and flies home: the mission monitor replaces the goal with the start through the
objective channel by which any goal enters the navigation in flight, and the
arrival at the start is a goal's arrival like any other (the 2.0 m capture
radius and the hold, no landing). The navigation did not change by a line
for it. The contracts are in `goal_reachability_proof_3d.hpp` and
`mission_monitor_node.cpp`, the check's lines in [`testing.md`](testing.md),
the flight in [`scenarios.md`](scenarios.md).

Built, each decision the smallest change found:

- **The proof** (`proveGoalUnreachable3D`, the core library, unit-tested) is
  a flood of the memory the navigation publishes, which the monitor decodes
  from the same snapshot and delta words the planner reads, from the vehicle
  through every voxel that is not occupied, unknown included, the grid's edge
  counting as unknown. It runs every 10 s on the monitor's own thread within a
  voxel budget that leaves it undecided, and it never reads the planner (r596
  stood 106 s without a route in an open world). It proves the goal
  unreachable on a component that holds no goal and touches no unknown, and
  the log line `GOAL_UNREACHABLE` carries its verdict and the moment. With no
  decay in the memory a closure holds for the rest of the flight; the decay
  and re-probe clause is item 17 stage 8's.
- **The position source** is the autopilot's position, valid and under a
  second old; without it nothing is substituted (`GOAL_UNREACHABLE_HELD`).
- **The substitution** publishes a `NavigationObjective` at the start with the
  next mission epoch; the planning tick rebases its waypoint sequence to the
  objective's goal, so the start is captured and acknowledged as any goal is,
  and the mission ends `goal_unreachable_returned`.
- **The injection** is the goal alone: (200, 100, 10) m, behind the outer
  walls of Urban Circuit Practice 01, recorded in the manifest
  (`MISSION_GOAL_UNREACHABLE`, `TRUTH_OCCUPANCY_3D_PATH`). The check floods the
  location's 0.5 m truth grid from the start and fails a flight whose start
  reaches the goal; a return counts only in an injected flight, and in every
  other flight a `GOAL_UNREACHABLE` fails it.

Found by the flights and repaired on the way:

- The planner accepted paths whose segments cleared whole while the
  activation refused them cut into the route sampler's 0.5 m pieces, and the
  return stood in a livelock (r669, r681, r685; 101 of 112 000 synthetic
  segments beside a plate flip between the two answers). The planner prices
  and accepts a segment in those pieces, the goal's connector included, and
  the geometry optimizer falls back to the canonical route when a shortcut
  fails in pieces (146c6393, c24cace4, 289583e2).
- While no route was held, an update's occupied changes reached the lattice's
  edge cache only after the feasibility search: the search validated its
  chains through edges the update had closed, extracted them, had them refused
  and dropped the labels behind them, update after update (r718: 1.09 million
  labels dropped, ten minutes without a route home 0.4 m from one). The test
  of a changed cell against an edge also missed a tenth of the cells an
  upright body passes on a climbing edge. Both repaired (bf92e952).
- An autopilot position reset beyond 0.33 m closed the navigation for good;
  r699 stood ten minutes six metres from its start after a 1.02 m reset. A
  reset up to 3 m is flown on (d71a5c6f, item 17 stage 6).
- P4 (130e484a) left r675 480 s without a horizon at contact: reverted
  (e81ae737), in the register.
- **The register's L1.** The doubled path measured the odometry's drift for
  the first time: 0.40 to 4.10 m at the end of 854 to 1467 m (eighteen camera
  flights), and r689 flew into the launch platform with the estimate 4.07 m
  off. Repaired inside this item by relocalization against the estimator's own
  long-lived map (6fa53f6d to 3f965181, [`localization.md`](localization.md)):
  0.20 to 0.68 m at the end of the same path. A first pass drifts as before.

Acceptance on bf92e952, each flight inspected before the next. The proof
never closed the vehicle's component in flight: at every check it touched the
grid's edge, since unknown lies above the flight band and in every corner not
yet looked at, so the topological proof is verified by its unit tests alone.
The goal was given up in every flight by the time-bound return the mission
monitor still carries from the implementation; the vehicle was never meant
to have a time limit, and item 17 stage 5 removes it for a return on the
light's battery, on the camera profile only.

| Goal outside the location | Flights | Given up at, s of simulation (flown, m) | Return, s | True position from the start, m | Estimator at the end, m |
|---|---|---|---|---|---|
| Stereo set, the defaults | r721, r722, r724, r725, r726 | 249 / 246 / 265 / 270 / 282 (456 / 434 / 508 / 456 / 504) | 417 / 366 / 433 / 325 / 282 | 1.06 / 0.65 / 0.40 / 0.65 / 0.59 | 0.68 / 0.22 / 0.20 / 0.34 / 0.21 |
| 3D lidar | r727 to r731 | 298 / 299 / 299 / 299 / 298 (687 / 704 / 694 / 666 / 681) | 53 / 37 / 52 / 57 / 43 | 0.52 / 0.26 / 0.64 / 0.42 / 0.66 | 0.26 / 0.17 / 0.30 / 0.15 / 0.25 |

| Ordinary series | Flights | Mean flight speed, m/s | True position from the goal, m |
|---|---|---|---|
| Stereo set, the defaults | r732 to r736 | 1.88 / 1.80 / 1.76 / 1.94 / 1.84, mean 1.84 (1.79 on 173155d6) | 1.14 / 1.63 / 1.42 / 0.70 / 0.64 |
| 3D lidar | r737 to r741 | 2.67 / 2.70 / 2.56 / 2.51 / 2.44, mean 2.58 (2.67 on 173155d6) | 0.89 / 0.19 / 0.58 / 0.24 / 0.74 |

No crash, no contact and no failing line in the twenty; no return in any
ordinary flight. The estimator and the host held their budget: on the camera
set 4.93 / 5.78 onboard cores at p50 / p95 against 4.82 / 5.67, 969 against
938 MiB, a frame 53 against 54 ms and the tick 21.3 / 28.5 against 21.8 /
29.7 ms; on the lidar 3.71 / 4.65 cores against 3.55 / 4.36 (the estimator
0.50 / 0.85 against 0.44 / 0.77) and the tick 23.5 / 31.7 against 24.1 /
34.5 ms. Two camera flights of the injected series, r720 and r723, were
voided and flown again: another task's disk writes froze the host for 3.7
and 7.3 s, the estimator took IMU holes of 1.9 and 3.6 s, diverged, and both
vehicles crashed after the autopilot reset by 26.6 and 6.0 m. That the
estimator does not survive such a hole is in the register. The return was
also the mission's first flight in the other direction and with another goal
(the register's N3); other starts and goals stay a series of their own.


**Addendum of 2026-09-30 (roadmap item 17).** What item 19 left unproven is
now tested, and the return has changed shape:

- **The time-bound return is gone** (0e09f0f6, I8, A8): nothing of the
  vehicle reads the run's window. A return has three triggers, each logged
  with its moment in `GOAL_UNREACHABLE`: the proof (`topological`), the
  carried light's battery (`battery`, the stereo set only, K11) and the light
  judged unreliable from the frames (`unreliable_light`, K12).
- **How the proof closes** (2642b48c, b3160689, 2f951ba3, K16). The flood
  runs within the space the planner flies in, the memory grid's box and the
  flight envelope's band of heights, with the goal clamped as the planner
  clamps it, under a budget of two million voxels; a second flood starts at
  the goal through the voxels the body fits, and a goal whose flood ends a
  metre short of the vehicle is unreachable; a proof substitutes the goal
  once it has stood 30 s, one confirmation's decay. After a return the same
  proof runs to the start (`START_UNREACHABLE_HELD`, `START_REACHABLE_AGAIN`)
  and the vehicle holds where it is by the planner's own rule. Twelve unit
  tests hold the sieve, the body's fit, the band, the clamp, a shut start and
  the grid's edge; the branch without a position source
  (`GOAL_UNREACHABLE_HELD`) is `returnHomePositionSourceFresh` and its test.
  A closure the size of the location is out of the proof's reach, because
  the memory forgets what the vehicle stops looking at (F15): measured on a
  full snapshot of r805's memory, the flood leaks out of the location even
  with the free space eroded by 0.75 m, while in truth the start's component
  is 31 186 m^3 and closed. The lidar vehicle, which carries no light
  battery, therefore explores for a goal outside the location without end,
  and its return by the proof waits for the owner's decision
  ([`technical_debt.md`](technical_debt.md)). In the zone over B the goal's
  dark is marked a ray apart and the goal region leaks round the frustum
  (r816): the return there is the light's judgment, not the proof's.
- **The way home is the way out** (6d7af22d, K20): the substituted goal is
  a point of the trail the vehicle flew, 10 m back (25 m until 2026-10-15,
  when r1538's return was routed through the plume it had backed out of to
  a point 28 m along the trail), moved on as it is neared, each in its own
  mission epoch, and the start at the end.
- **Flown and inspected on 6e7ea775** (the tables of item 17): the goal
  outside the location on the stereo set, given up by the battery after 122
  to 193 s and 204 to 347 m of exploration and returned in truth 0.33 to
  0.62 m from the start; the zone over B, five returns by the light's
  judgment 0.62 to 0.82 m from the start, the dark core never entered; the
  severe failure, five returns 0.44 to 1.58 m from the start; the low
  battery at launch, five returns 0.38 to 0.63 m from the start, B given up
  20 m in. The return scenario's light lasts 720 s, the owner's rule of
  2026-09-30: a test of the return does not fly for an hour. The mission
  check floods the truth grid within the flight space for every injected
  goal.

**Repaired after** (the technical-debt goal, 2026-10-10 to 13): the way home
moves its goal along the trail within one mission epoch, which the route
lifecycle replans (K20): one mission epoch on each of the sixteen returns of the final series (light-failure and low-battery, r1355 to r1364 on fba5bead, r1397, r1402 to r1405, r1407 on bc7c16a9), the longest stand 0 to 3.5 s; before, r1225 took nine epochs and seven revocations. The camera estimator
starts again in flight after a hole in its IMU stream (K28; the entry this
item's r720 and r723 left): five flights with holes of 2 and 4 s on fba5bead and one on the final commit, every check passed, the autopilot reset by at most 0.043 m at a restart.

### 20. Slowed Simulation And Unattended Recording (Completed)

Shipped in [v0.5.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.5.0) on 2026-10-10; closed on 2026-10-05; both final series were flown on
df5a86b8. The simulation can be slowed against the wall clock with the stack
slowed with it, and a recorded flight is a flight like any other. What was
asked for and how it was built is kept below as it was written; this is what
came of it.

**Stage 0, the onboard loop on the simulation clock** (specification K24;
the audit's table is in [`gazebo_simulation.md`](gazebo_simulation.md)).
Twelve wall timers and the monotonic clock in 62 sources were sorted: every
period and freshness watchdog of the single vehicle keeps the node clock
(the two ticks, the planner's heartbeat and its age, the mission monitor's
flood, the obstacle memory's transport period, the ESDF build rate), every
compute budget and measured duration keeps the monotonic clock. At a factor
of 1.0 nothing changed but the timer's grain:

| Series | Stereo set, m/s | True position at B, m | Tick p50, ms | 3D lidar, m/s | True position, m | Tick p50, ms |
|---|---|---|---|---|---|---|
| Base (r983 to r987, r1068 to r1072) | 1.742 to 1.973, mean 1.89 | 0.80 to 1.25 | 21.3 to 22.2 | 2.522 to 3.140, mean 2.70 | 0.33 to 1.07 | 23.9 to 25.1 |
| Stage 0 (666c0068: r1073 to r1077, r1078 to r1082) | 1.844 to 1.987, mean 1.895 | 0.86 to 1.24 | 21.6 to 22.5 | 2.637 to 2.734, mean 2.675 | 0.14 to 0.72 | 23.6 to 24.9 |
| Final (df5a86b8: r1098 to r1102, r1103 to r1107) | 1.822 to 1.939, mean 1.875 | 0.78 to 1.69 | 21.4 to 22.3 | 2.733 to 2.941, mean 2.852 | 0.08 to 0.75 | 23.2 to 25.0 |

Every flight counted by the host's verdict and none touched anything. The
loop runs 37.7 to 39.5 times a simulated second, against 40.6 to 41.6 a wall
second on the wall timer; the register's entry on the 20 ms deadline is
decided on these figures. Each scenario of item 17 was flown once headless on
the stage's commit and passed: the light lost (r1083, landed whole), the
severe failure (r1084, home at 0.68 m), the low battery (r1085, 0.40 m), the
goal outside (r1086, 0.66 m), the long flight under failures (r1087, 1.848
m/s, 1.02 and 0.59 m).

**Stage 1, the factor a parameter of the run** (`REAL_TIME_FACTOR`,
specification K25, [`configuration.md`](configuration.md)). At 0.5 the stereo
set reached B at 0.61 m at 1.948 m/s of simulation with the simulator at
0.50 from the 5th to the 95th percentile and the onboard processes on 2.48
cores at the median against 4.57 at 1.0 (r1088): the load falls with the
factor, which is what four vehicles need. What a slowed flight flatters is
measured: the tick takes 10.8 ms of simulation against 22, runs 47.5 times a
simulated second against 38 to 39.5, 2 percent of the ticks pass 20 ms
against 67 to 71, the observation reaches the tick 168 ms old against 260 to
296, and the planner's 150 ms are 75 ms of flight. So a change is accepted
at 1.0 only and a slowed flight is compared with flights at its own factor.
The heaviest configuration, the stereo pair beside the lidar, flew at 0.5
with its loop slowed with the world for the first time: 2.650 m/s, 0.26 m
from the goal, the simulator at 0.50, 3.23 cores (r1089).

**The recorded flight** (specification A11, A7). Beside the windows the
simulator holds 0.6 and does not hold 0.7 (r1091, r1092), so a recorded
flight asks for 0.6; the host's verdict is read against the factor asked
for, and one it counts is a flight like any other. The first batch's defects
were measured and repaired:

- **The slideshow** in eleven recordings was not the host's performance:
  with the screen blank (the desktop's idle timeout) the desktop presents
  one frame a second, and the Gazebo window, which waited for the display,
  redrew once a second. Reproduced on a test world: 1 redraw a second with
  the screen off, 24 with the wait switched off. The window no longer waits.
- **The green frame on the vehicle** was Gazebo's gizmo of the carried
  light, drawn until the light's first update reached the window: 611 green
  pixels on a test world, none after `<visualize>false</visualize>`.
- **The takeoff unseen**: the Gazebo window loads the scene for some 20 s
  after it opens and draws nothing meanwhile. A recorded flight starts 45 s
  after the simulator.
- **A recording is the whole flight**, never cut, at the flight's own pace:
  each window's frames are re-timed to the simulation clock from the true
  pose record, so a flight at 0.6 plays as long as it flew. The recorder
  refuses a recording whose window stood still, whose half was a slideshow
  while the vehicle moved, whose length is off the flight's by more than a
  second, or that is shorter than a minute. The low-battery flight, home 32
  s after its launch, is recorded at a cruise speed of 0.6 m/s, where the
  same flight lasts 66 s.
- **The host's verdict sees a starved estimator**: the poses' age on their
  way to the autopilot, 64 and 96 ms at the 95th percentile on a quiet host
  and 0.7 to 1.3 s in the two recorded lidar flights that crashed.

One pass over the six scenarios on the stereo set, on df5a86b8, every flight
counted by the verdict (`log/videos/2026-10-05/`):

| Scenario | Run | Mission check | True position, m | Length, s | Different frames a second, world / RViz |
|---|---|---|---|---|---|
| Ordinary flight | r1109 | pass, 1.883 m/s | 1.09 | 232 | 23.2 / 23.8 |
| Long flight under failures | r1121 | pass, 1.962 m/s | 0.96 and 0.82 | 667 | 23.0 / 23.8 |
| The light lost | r1111 | pass, landed whole | | 65 | 22.4 / 23.8 |
| The carried light fails | r1112 | pass, home by the unreliable light | 0.55 | 161 | 21.5 / 23.8 |
| A low battery at launch (cruise 0.6 m/s) | r1120 | pass, home by the battery | 0.29 | 66 | 14.7 / 24.0 |
| A goal outside the location | r1114 | pass, home by the battery | 0.30 | 161 | 22.7 / 24.0 |

The same ordinary flight at 0.6 without the windows: 1.848 m/s, 1.16 m from
the goal, route availability 98.9 percent against 97.7, tick 21.8 ms against
22.7, 2.89 onboard cores against 3.11 (r1108 against r1109).

**Found on the way.** The blind descent was published only while the
autopilot called its position valid; a flight that lost it at the touchdown
stood armed on the platform for six minutes (r1093, repaired, specification
K19). The mission check read the estimators' publication rates per wall
second. And the long flight under failures stands still near B when the
simulator runs below real time, with the windows or without: three of nine
such flights passed (r1110 and r1117 held, r1116 and r1121 passed at 0.6)
against nine of nine at real time; the route's lifecycle finds and certifies
a route and never activates it. It is in the register and is not met at a
factor of 1.0, where a change is accepted.

**Left for item 15**: the cooperative agent, referee, spectator, diagnostics
multiplexer and the truth adapter keep their wall timers until a cooperative
flight can be flown, and the flight of four lidar vehicles at 0.5 is that
item's. Stage 2's camera in the world was not built: the Gazebo window gives
the picture, and its price is paid in wall time since stage 1.


**Type:** simulation infrastructure and one honesty repair of the onboard
loop; no navigation policy.

**Hard prerequisites:** none. Stage 3 depends on nothing but a display
server and is built first (revised on 2026-10-03, below).

**Validation environment:** Urban Circuit Practice 01, the point-to-point
mission. The cooperative-traffic mission is what stages 0 and 1 are for, and
it is flown under item 15, not here: no cooperative flight is flown before
item 15 closes (the owner's rule).

Asked by the project owner on 2026-09-25. The reference workstation holds two
lidar vehicles at real time and not four (item 15), and a demonstration
recording today means a person at the desk with the GUI open for six
minutes. Both have the same answer: let the simulation run slower than the
wall clock on purpose, and let the picture be rendered and written without a
screen. The facts this item rests on were established on 2026-09-25:

- **The simulator's clock rate is a world parameter.** The materialized
  worlds carry `<max_step_size>0.004</max_step_size>` and
  `<real_time_factor>1.0</real_time_factor>`; Gazebo throttles its server to
  the factor, PX4 SITL runs in lockstep and follows, and since item 16
  (`UXRCE_DDS_SYNCT=0`) every autopilot stamp is the simulation clock at any
  factor. The mean flight speed of the second requirement is measured on the
  simulation clock since 2026-09-25, so the check does not care either.
- **Slowing the simulator halves only what runs on the simulation clock.**
  Physics, the GPU lidars, the stereo pair and the depth node (215 percent
  of a core, per frame), the obstacle memory (72 percent, per scan) and the
  autopilot all take half the wall time per simulated second at a factor of
  0.5. The planning tick (`tick_rate_hz`, a wall-clock rate) and the
  offboard tick (`create_wall_timer`, 20 ms) do not: `production_mppi_node`
  at 186 percent of a core costs the same wall second whatever the factor,
  and with four vehicles that is 7.4 of the 8 physical cores before anything
  else runs. Item 15 already records the other half of this fact: at a
  real-time factor of 0.6 the planner takes about 72 ticks per simulated
  second instead of 43, so a slow simulation flies the vehicle with a faster
  computer than it has.
- **The server already renders without a display.** Headless flights run
  `gz sim -s --headless-rendering`; the stereo pair is rendered that way. A
  camera sensor placed in the world for the picture renders the same way and
  its frames carry the simulation stamp, so a recording made from them plays
  at the flight's true speed whatever the factor was.
- **RViz does not.** Its overlays (the memory, the current depth, the
  committed route, the goal) exist only in RViz, which needs a display
  server, and a capture of that display runs on the wall clock.

**Revised on 2026-10-03, by the project owner's decision.** Three things
changed after the item was written:

- **The location is dark.** Since item 17 the only light is the one the
  vehicle carries, and the location's own never comes back (specification
  A10). A light in Gazebo belongs to the scene, not to a camera, so a
  spectator camera in the world sees what the vehicle's lamp lights and
  black around it. The picture that shows a flight is RViz: the memory, the
  route, the goal, the evaluation objects. Stage 3 is therefore the
  recording this item delivers and is built first; stage 2 is the secondary
  picture.
- **What the recording is for.** The demonstrations of item 17's scenarios,
  recorded at night with nobody at the desk, several takes of each and the
  best one kept, because no two flights are alike. A single camera vehicle
  holds real time headless, so these recordings need neither stage 0 nor
  stage 1.
- **The product is the split picture** the owner records by hand today: the
  3D world on the left, RViz on the right, one file. The world's half is
  stage 2's camera and RViz's half is stage 3's capture, joined on the
  flight's clock. The world's half is as dark as the Gazebo window of a
  hand-made recording is since item 17: the lamp's cone and what it lights.
- **Every way of recording is allowed** (the owner, 2026-10-03), the plain
  one included: the windows open on the workstation's own display and the
  screen captured. The recordings are made at night, the workstation is not
  used then, and an open window is in nobody's way. So the stages below are
  the candidates and not the prescription: the Gazebo window with its own
  following camera beside RViz, on the real display, is what the owner
  records by hand and is the first thing to try, and a camera sensor in the
  world or a virtual display is built only where the plain way fails. The
  way is chosen by measurement: the picture, the real-time factor of the
  recorded flight against specification A7, and whether a batch runs a
  night through with nobody at the desk. What the choice must state: the
  workstation's session is Wayland, where a capture of the screen goes
  through the desktop's screencast and not through X; windows have to be
  placed without a person; and a flight with the GUI open is the GUI
  scenario, not the headless acceptance flight.
- **Nothing but the picture is in the picture** (the owner, 2026-10-03). A
  window opened as it is carries what a recording does not want: RViz's
  displays list, its views and tool panels and its toolbars, Gazebo's entity
  tree, component inspector and world controls, title bars, the desktop.
  They cover the flight. A recording shows the two views and nothing else:
  RViz is started with a configuration for recording whose docks, panels and
  toolbars are hidden, Gazebo with a GUI configuration that holds the 3D
  scene alone, and whatever frame remains is cropped out of the capture.
  The recording configurations are files of the repository, beside the
  debugging ones, which stay as they are.
- **The order.** Stages 3 and 2 together, for the recordings; stages 0 and 1
  when item 15 needs four vehicles or the heaviest configuration is to be
  measured. The stage numbers are kept, because other pages name them.

#### Built On 2026-10-04: The Unattended Recording

Stages 3 and 2, for the recordings of item 17's scenarios, by the plain way
the revision above allows, chosen by measurement:

- **A recorded flight is the headless flight with its pictures on the
  desktop** (`RECORD_VIDEO=1`): the server renders headless and the mission
  check runs as in the acceptance, and beside them the Gazebo window opens
  with the 3D scene alone (`gazebo_gui_recording.config`, its following
  camera as in a GUI flight) and RViz in both views, written from the
  debugging configurations with the panels gone and the top-down view
  following the vehicle (`scripts/rviz_recording_view.py`).
- **Each window's picture is taken where it is drawn**
  (`scripts/frame_pace_shim.c`, preloaded into the three windows): on every
  buffer swap the middle of the 3D view, in the shape of a half of the split
  picture, is read back through a pixel buffer object and handed to a FIFO.
  What is read is the view, not the window, so no panel, toolbar or desktop
  can be in the frame, and nothing depends on how the windows lie on the
  screen. The same shim paces the Gazebo window to 24 frames a second: it
  redraws the scene at the display's 144 Hz otherwise.
- **The recorder** (`scripts/record_flight_video.py`, on the host) stores
  the three streams through the GPU's encoder while the flight flies and
  joins them after it: two files a flight, the world on the left and RViz
  on the right, 1920 x 1080, 24 frames a second, cut to the mission from its
  readiness to its result, and checked (size, length against the flight, a
  picture that is neither black nor still).
- **One command a flight, one a night**: `tools/record_flight.sh` and
  `tools/record_batch.sh`, which flies every scenario of item 17 on the
  stereo set round robin
  and writes an index beside the videos. The desktop is kept from going idle
  by an inhibitor while a flight lasts; none of its settings is changed.

Tried and left: the windows read through the X server (it works on the
Wayland desktop, the windows being XWayland's, but each window read cost the
simulator a seventh of its speed), and the desktop's own screen recorder
(not to be driven without a person).

**What the recording costs, and what that decides.** The windows beside a
flight hold the simulator at 0.72 to 0.80 of real time on the stereo set
(r1015 to r1017), under the floor of the host's verdict, and they do so
whatever captures them: stopped in flight the Gazebo window gives back 0.08
and the two RViz 0.13. A recorded flight is therefore a demonstration and
never an acceptance flight (specification A11); each scenario's acceptance is
its headless flight on the same commit. Making a recorded flight count is
this item's remainder: the pictures rendered after the flight from a
recording of its topics, or stages 0 and 1 below, with which the simulation
is slowed on purpose and the windows' price stops mattering. The world's
half is as dark as the location is; on the lidar, which carries no light, it
is black, and flights on the lidar are no longer recorded (the owner's rule
of 2026-10-04, specification A11).

**Recorded on 2026-10-04.** Three passes over the seven scenarios of item 17,
round robin, with nobody at the desk: 27 flights, 54 split files, the index
beside them (`log/videos/2026-10-04/`, kept from the pruning; the six lidar
recordings were deleted since, the batch flies the stereo set alone). Every
scenario has three takes whose flight passed its check but the long flight
under failures, which passed once in five. What the batch showed besides the
pictures:

- **The lidar's estimator was starved by the windows** and two recorded
  lidar flights crashed (r1053, r1066) although the simulator held real
  time: registrations ran over the scan period, the pose reached the
  autopilot up to 1.3 s old, the autopilot stopped fusing it and lost its
  position. Repaired (specification K23): a scan that waited a period and a
  half is let go, never two in a row. Under the same load the recorded
  flight then passed (r1067, 37 scans of 1550 let go), and the headless
  series on the repair is five of five (r1068 to r1072, 2.52 to 3.14 m/s).
- **The long flight under failures does not survive the recording**: an
  eight-minute hold beside phantom occupancy that no longer fades (r1032,
  the register's entry on the memory's decay), an acknowledgement 2.03 m
  from its goal (r1037), two crashes with the estimate stepping under the
  stream's failures and the slowed simulator (r1040, r1054). Headless it
  passes six of six.
- **One descent past the edge of the staging base clipped it** (r1044, the
  simulator at 0.62 of real time); six headless flights of the same scenario
  pass the same edge clean.

#### Stage 0: The Onboard Loop Keeps Simulation Time

The planning tick and the offboard tick move from wall-clock timers to the
node clock under `use_sim_time`, which every onboard node already declares
for its stamps. At a factor of 1.0 nothing should change but timer jitter,
which is what the acceptance measures: both series on the one commit,
against both series flown on the commit before it (not against an older
base: the stack has moved through items 19 and 17 since this was written),
the tick and planner percentiles beside their debt figures. At a
factor below 1.0 the loop then slows with the world and the load falls with
the factor; what remains unequal is the wall-clock latency of the transport
and the planner's 150 ms budget, which become shorter in simulated seconds by
the factor, so a slowed flight still flatters the stack by that much and its
speed figures are never compared with a real-time series. The stage lands
first because without it a slowed run neither lightens the host nor tells
the truth. The register's tick past its 20 ms deadline (P2) is re-measured
here, on the node clock, and the deadline question is decided on those
percentiles; the rework of the tick, if one is wanted, is not this item's.

What the stage has to state before it changes anything: the audit. Twelve
wall-clock timers sit in nine sources (the planning tick, the offboard tick,
three in the mission monitor, two in the controller's interfaces, the truth
adapter, and the cooperative agent, referee, spectator and diagnostics mux),
and 62 sources read the monotonic clock for budgets, latencies and
watchdogs. The audit lists each and says which follows the simulation: a
period or a freshness watchdog does, a compute budget (the planner's 150 ms,
the assembly's 12 ms) does not, since it measures the host. Startup waits
for `/clock`. `mppi_offboard_node.cpp` stood at its 1000-line cap then, so its
change came with a cut elsewhere in it; on 2026-10-09 the node was split into
a header and three sources (the session and the tick, the horizons and the
setpoints, the autopilot's state), the owner's decision.

#### Stage 1: The Factor As A Parameter Of The Run

`<real_time_factor>` becomes an input of the environment materialization,
which already rewrites the worlds it installs, and an environment variable
of the simulation scripts with 1.0 as the default; the runtime manifest
records the factor asked for and the resource record keeps reporting the
one achieved. The quiet-host gate is unaffected: it reads processes, not the
factor. A cooperative flight of four lidar vehicles at 0.5 would then cost
the host what two cost at 1.0, which the workstation holds (item 15), at
twice the wall time: twelve minutes for a six-minute flight; that flight is
item 15's. Here the factor is proven on one vehicle. The register's
heaviest configuration (S3), the stereo pair beside the lidar at a factor of
0.83 with the loop on the wall clock, is flown here at 0.5 with the loop
slowed with the world, which is the first time its figures mean anything;
they are compared with each other, never with a real-time series.

#### Stage 2: The Picture Written Without A Screen

A spectator camera as a sensor of the world, on the vehicle the spectator
selection names (the cooperative missions already select and reselect a
spectator) or at a stated pose, bridged like the pair's frames and written
to a video file by a recorder that consumes the image topic, in the run's
directory beside the logs. Resolution, rate and the camera's placement are
stated with the cost of the extra render on the GPU, which the camera
profile already loads to 40 percent at real time and which the factor of
stage 1 relieves. The recording plays at the flight's true speed and is a
product of every headless run that asks for it, with nobody at the desk.

In the dark location this picture shows the cone of the vehicle's lamp and
nothing else, which is true to the flight and says little about it alone.
It is the left half of the split recording, beside stage 3's RViz, and is
built with it. Whether the
camera's own gain may brighten it is stated with the recording: a gain is
the camera's, a light would be the location's and is not allowed.

#### Stage 3: RViz In The Background

Built first, and the recording this item delivers (revised on 2026-10-03):
in a dark location RViz is where a flight can be seen.

The only way to record RViz without a person is a virtual display: `Xvfb`
with software rendering, or a second X server on the GPU with a dummy
screen, and `ffmpeg` capturing it. The container image carries neither
`Xvfb` nor `ffmpeg` today; both are added to it. The capture runs on the
wall clock: at a factor of 1.0, which a single camera vehicle holds
headless, it is the flight's own speed to within the factor the resource
record samples, and at a factor below 1.0 it is slow motion by a varying
amount and is re-timed afterwards from that record.

What the stage delivers, for every scenario that asks for it by one switch
of the run: RViz started on the virtual display with the repository's
configuration and a camera that follows the vehicle, the capture written to
the run's directory beside the logs from the mission's start to its result,
and the simulation run headless as the acceptance flights are, so that the
recorded flight is an ordinary one. The cost of RViz's rendering beside the
flight is measured: a recording that pushes the real-time factor under 0.95
is not a recording of the flight the acceptance flies, and software
rendering is replaced by the GPU server if it does. The world camera's
recording of stage 2 and this capture are joined side by side into one
file, the world on the left and RViz on the right, aligned on the mission's
start.

Nobody moves the view during a recording: it is what the configuration
says, following the vehicle, for the whole flight. One fixed view loses the
vehicle behind a floor or in a shaft where a person at the desk would have
turned it, so **every recorded flight is captured in both of the
repository's views by default**, the third-person one
(`city_nav_debug.rviz`) and the top-down one
(`city_nav_debug_top_down.rviz`), two RViz instances on the one flight. Each
gives a split file of its own with the same world half, and the better one
is chosen when the recording is cut. One view alone is a switch of the run.
The second instance's cost is in the measurement above: both together must
leave the real-time factor at 0.95 or over. The memory's cloud in RViz is
drawn translucent as it is and does not hide the vehicle; nothing is
changed there. A batch launcher flies a list of scenarios one after another,
several takes each, unattended.

#### Measurement And Completion

Stages 3 and 2 are complete together when an unattended batch has recorded
every scenario of item 17 with nobody at the desk, each recording two split
files, the 3D world on the left and RViz on the right, one with the
third-person view and one with the top-down view, playable from the
mission's start to its result, with no panel, list, toolbar or desktop in
the frame, the real-time factor of each recorded flight at 0.95
or above, and the cost of the capture stated; at a factor below 1.0 a
capture re-timed to the flight's clock within one second over the flight.
Stage 2 is complete when a headless run writes a playable recording of its
whole flight from a camera in the world without a display server, with the
GPU cost stated. Stage 0 is complete with both acceptance series green on
its commit, against both series on the commit before it, and the tick and
planner percentiles reported beside the previous ones. Stage 1 is complete
when a single-vehicle flight at a factor of 0.5 reaches its goal with the
factor asked for in the manifest and the one achieved in the resource
record, and the heaviest configuration (S3) has been flown at 0.5; the
four-vehicle cooperative flight at 0.5 is item 15's.

**Repaired after** (the technical-debt goal, 2026-10-12): the long flight under failures stood near B below real time (P10). r1110 stood in a component its planner believed closed, the retreat along its trail refused; r1365 (fba5bead) stood with its way home open on the map while the feasibility search found, swept and rejected the same edges beside a wall on every update, because every change of the world cleared the edges it had rejected. Repaired (3223df80): a rejected edge stays withheld until the world changes within one lattice step of it; the planner reports the trail retreat's attempt and the withheld edges. B and back on the goal's final commit (bc7c16a9): three of three at 0.6 (r1369 to r1371) and two of two at 1.0 (r1372, r1373), home, landed and disarmed at the start. What a recorded flight cost is repaired as well (the goal's part 6, A11): a flight is rendered after it, flown at real time with no window (`tools/render_flight.sh`; r1409: 0.88 of real time at the median, the recording ok).
### 23. Flight Through Moderate Smoke (Completed)

Shipped in [v0.6.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.6.0) on 2026-10-24; closed on 2026-10-17 (d56f0f85 on 7bc3941b); five of five to
B in truth through a room of moderate smoke on the lidar (r1675 to r1679) and
on the stereo set (r1705 to r1709), the speed under smoke the requirement K43. What was asked for and how it was built is kept below as it was
written; this is what came of it.

**Type:** dependent realism stage. Decided by the project owner on
2026-10-03.

**Hard prerequisites:** item 18, whose stage 1 puts smoke into the simulator
and whose stages 2 and 3 detect it; item 17 stage 0, the contract that reads
the frame.

**Validation environment:** Urban Circuit Practice 01, the point-to-point
mission, on the stereo set and on the 3D lidar.

Item 18 handles smoke the vehicle cannot see through: a local plume is a
measured prohibition and the vehicle flies around it or home. This item is
the other half: smoke the vehicle can see through, not in one place but
filling whole rooms, which it has to fly **through**. It needs no new rule.
The braking contract already ties the speed to the range the frame measures
(specification K9): a range above the margin is a slower flight, a range
below it is item 18's prohibition, and where one ends and the other begins is
a measurement, not a setting.

What the flights look like once the item lands:

- **Rooms, not the location.** Several rooms are filled with moderate smoke
  wall to wall; the rest of the location is clear. A location filled
  throughout would only move every figure at once.
- **One smoky room is always on the way**: in every ordinary flight the
  route to B passes through a room of moderate smoke, as every ordinary
  flight carries item 18's plumes off the way. Both sensor sets fly it.
- **The smoke is drawn in RViz**, as every evaluation object is.

Three hard places, each measured before anything is built:

- **The vehicle's own light.** The light sits beside the cameras, and smoke
  scatters it back into them: a veil that lowers the contrast the matcher
  and the feature tracker live on, worst exactly where the light is
  strongest. What the confident range and the estimator's tracked features
  become in smoke lit from the camera's own position is the first
  measurement; a light set apart from the pair, or polarized, is the remedy
  to compare.
- **The lidar's false returns.** Particles return the beam. The memory must
  not write them as walls, or moderate smoke becomes item 18's prohibition
  by accident; it must not ignore a wall behind them either. Last-return
  selection, intensity and the returns' persistence between scans are what
  there is to tell them apart with.
- **The simulator.** Everything above depends on how Gazebo shows smoke to
  a camera and to a lidar, which item 18 stage 1 settles; if its smoke is a
  visual effect only, this item starts with the sensor models. Settled
  (2026-10-14): smoke is a model the sensors read (specification A13), with
  the lidar's single strongest echo; last-return selection needs the model
  to report more than one echo a ray first.

**Size:** L to XL.

### Measurement And Completion

Measure, per density of smoke and per sensor set: the contract's measured
range, the speed it admits, the estimator's tracked features and drift, the
false occupancy the memory holds inside the room and the walls it misses.

This item is complete when, with one smoky room on the way in every flight,
five flights on the stereo set and five on the 3D lidar reach B in truth
with no contact, the mean flight speed is recorded as the requirement under
smoke, and the memory of the smoky room holds no occupancy the truth does
not.

**Measured and flown on 2026-10-16 and 17** (`sim-urban-smoke-rooms-headless`:
the west hall at (-20,22.5) filled wall to wall with 0.1/m, on the way, and
the tunnel's mouth at (18,-6) beside it; the series in `log/tools/part7/`).
The three hard places, measured before the flights:

- **The vehicle's own light.** The pair through 0.1/m matches a wall 4 m
  off whole and one 8 m off not at all (`tools/smoke_rig.py`,
  log/tools/part7/rig_last); the light set apart lowered the veil and
  gained nothing (item 18 stage 1). The estimator replayed over 11 m of
  flight through 20 m of 0.1/m: 0.88 m off at the end (1.67 at most)
  against 0.41 (0.62) clear; through 0.2/m 0.33 (2.93); the features held
  (84 to 92).
- **The lidar's false returns.** The relay's strongest echo returned the
  smoke a metre off and hid the walls inside 0.3/m; the lidar reports its
  last echo over the threshold (A13, a driver set to last return) and reads
  a wall 8 m on through 0.1 and 0.2/m in every forward ray, 15 m on through
  0.1/m in 93 percent; dense smoke, 3/m, still returns the smoke. In the
  memory the smoke's own return is a miss to its range and nothing more
  under 0.01 of a matte surface's intensity, about 0.5/m (K37): read as
  dense, the room closed in front of the lidar at 7 m and the vehicle
  turned back (r1661). The relay reads a ray in steps that follow the
  zone's density, or a room of 26 by 40 m starved it to 7 scans a second
  (r1659).
- **The simulator**: settled by item 18 stage 1 (A13).

The flights, each at a real-time factor the host sustained (0.8 on the
stereo set, K25):

- **The lidar reached B in five of five**: r1675 to r1679, 0.11 to 0.52 m
  in truth, no contact, 2.16, 2.28, 2.33, 2.53 and 2.45 m/s (the clear
  flights 1.81 to 2.59). The memory of the smoky hall holds 0.2 to 0.9
  percent of its occupied voxels with no truth occupancy within 0.5 m,
  against 0.2 percent in the clear flight r1667.
- **The stereo set, first series (0455d22e): three of five in truth**:
  r1669 0.74 m, r1672 1.36 m, r1674 1.19 m; r1670 and r1673 came 2.85 and
  3.50 m off. No contact. 1.11, 1.32, 1.35, 1.33 and 1.34 m/s. The miss had
  a root cause of its own, found on 2026-10-17 by a recorded flight replayed
  offline (0.6 m of drift offline against 2.25 m in flight) and the heading
  sigma logged: the filter's first step after the declared pose spans the
  frame's latency and was read as a hole in the IMU stream, which gave the
  heading, which no feature observes, 9.5 degrees of uncertainty for the
  whole flight, and every update walked it (specification K28, repaired in
  5d6443bd; two of seventeen clear flights since 2026-10-14 had missed the
  radius by it too).
- **The stereo set, second series (e9d26309, the estimator repaired): four
  of five in truth**: r1683 1.23 m, r1684 0.93 m, r1685 1.14 m, r1687
  1.37 m; r1686 came 2.65 m off (2.25 m horizontal, 0.72 m vertical), the
  first-pass drift the register's L1 names: the estimate under-reads the
  distance flown by 0.3 to 0.6 percent in every flight and rises 0.10 to
  0.24 percent of the path above the truth, and r1686 added a random walk of
  1.5 m across the east hall. No contact. 1.38, 1.28, 1.26, 1.29 and
  1.21 m/s. The memory of the hall: 0.9 to 1.7 percent phantoms (the clear
  flight r1666 0.4). The frame stamp offset swept on two records (0 to 6 ms)
  moves the scale through 1.000 near 2 ms and the error at B within its
  noise, so the offset stays at the calibrated 4 ms.

- **The stereo set, third series (7bc3941b, the principal point at the
  pixel centres): five of five in truth**: r1705 1.38 m, r1706 0.44 m,
  r1707 1.16 m, r1708 1.01 m, r1709 1.07 m; 1.28, 1.37, 1.24, 1.28 and
  1.28 m/s; no contact; the memory of the hall 0.5 to 1.0 percent
  phantoms. The second series' systematic rise of the estimate (0.10 to
  0.24 percent of the path, every flight) had a root cause of its own: the
  pair's principal point stated at the frame's centre (640, 480) where the
  pixel coordinates the tracker and the depth use put it half a pixel in,
  at (639.5, 479.5): 0.08 degrees of pitch and yaw (specification K33,
  repaired in 67e2489e and 73cf6867; the records of r1680 and r1682
  replayed at the pixel centres end 0.07 m off vertically instead of 0.43
  and 0.55). In flight the rise is gone (r1705: 0.08 m over 422 m).

**The speed under moderate smoke is recorded as the requirement**
(specification K43): the lidar 2.1 m/s, the stereo set 1.1. From 2026-10-17
to 2026-10-24 the moderate rooms were part of the ordinary flight (F4: the
location's smoke its default), so every acceptance flight crossed the west
hall; since 2026-10-24 the ordinary flight is clear and no smoke flight is
flown (the owner's rule). **The item is
complete on both sets**: five of five to B in truth on the lidar
(r1675 to r1679) and on the stereo set (r1705 to r1709), no contact, the
way through the room, the contract's speed in it and the memory of it
measured. What the estimator still carries is the first-pass drift the
register's L1 names, without its two systematic parts: the scale under-read
of 0.3 to 0.6 percent stays, the rise does not.

### 24. Item 19's Proof, Flown (Completed)

Shipped in [v0.6.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.6.0) on 2026-10-24; closed on 2026-10-16 (06d44c27 on 76367bee); item 19's proof
flown on the lidar five of five in the blocked corridor (r1616 to r1620, the
flood through voxels the body fits in, K16, the grazing ray K42), the stereo
set home by the ladder since the pair cannot map a plume from outside (F15). What was asked for and how it was built is kept below as it was
written; this is what came of it.

**Type:** validation debt of item 19. Decided by the project owner on
2026-10-03, restated the same day.

**Hard prerequisites:** none of its own. It is closed by a flight series
another item flies anyway (below).

Item 19's topological proof that a goal is unreachable has never fired in a
flight: it is held by twelve unit tests alone (specification F15). Every
return flown was the light's battery or the light's judgment. What this item
validates is the algorithm, the proof that point B is **physically**
unreachable, and nothing else about the flight:

- **The sensor set does not matter**, and neither does what closes the way.
  The proof reads the memory, not a sensor: the 3D lidar serves, the stereo
  set serves, smoke serves, a physical partition serves.
- **The return is by the proof, not by the battery.** In these flights the
  vehicle is not bounded by its light's charge: the goal is given up with
  `trigger=topological`, and a flight that returns by `battery` or by
  `unreliable_light` does not count. On the stereo set the charge is set so
  that it cannot be what turns the vehicle home, or the lidar is flown,
  which carries no light battery at all.
- **The memory's decay stays off** (specification K10), so that what the
  vehicle has seen stays seen.
- **The reachable part of the location is small enough** to be explored in
  minutes, not an hour.

**What closes this item.** Item 18's blocking scenario, "smoke closes the
passage to B, no route exists, the vehicle flies home", flown and inspected
with the goal given up by the proof. When that series passes, this item is
complete: no flight of its own is needed, and nothing is re-flown with
another barrier.

What has to hold in those flights, and is checked in them:

- **The closure is marked densely enough to close.** Evidence laid rays
  apart let the goal's region leak round it when B lay in the dark (r816).
- **No other trigger speaks first.** A vehicle blind for 3.5 s judges its
  light unreliable and turns home (K12) before any proof stands; the
  scenario lets the proof speak, by the order of the triggers or by flying
  the lidar.
- **What else leaks.** Walls never looked at close by themselves, because
  the planner flies to every opening it believes in; free voxels behind
  true walls do not, and are measured on a memory snapshot of the flight.

Complete when five flights of item 18's blocking scenario give the goal up
with `trigger=topological`, the truth grid confirms no way existed, and the
vehicle is home in truth, on either sensor set.

**Completed on 2026-10-16.** On the lidar the blocking scenario gave the goal
up by the proof in ten of ten valid flights (r1576 to r1580 on e471157b,
r1616 to r1620 on 76367bee; r1556 and r7 to r10 before them), within about
110 s of the plume's face, and the vehicle came home 0.15 to 0.51 m from the
start in truth without a contact; the truth grid confirms no way existed
(`log/tools/truth25/flood`, the plume as a sphere of 3.5 m: the start's side
6 730 m3, B outside it). What it took, each measured on a flight and
recorded in the specification: the lidar's smoke echo over its pulse's range
gate (A13; the plume had been invisible from 4 m, r1546 and r1547 crashed in
it), a scan of smoke alone read as an aerosol (K37), the proof's floods
through the voxels the body's radius fits in (K16; the plume's face marked a
beam apart was a sieve, r1549), and a grazing ray carving nothing of a
surface (K42; the start's side opened under the hover within a sample, r1564,
r1566). On the stereo set the same scenario is given up by the ladder in
smoke (K40, `trigger=smoke`), not by the proof: the pair cannot map a plume
it cannot see into, so no closure stands in its memory. The ladder was
withdrawn on 2026-10-18 (the owner), the scenario admits the proof's trigger
alone, and on the stereo set it is open.

### 25. The Estimator On Real Recordings (Completed)

Shipped in [v0.7.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.7.0) on 2026-10-31; closed on 2026-10-27 (dba0daff, the root goal's part 5;
specification K50): the visual-inertial estimator replayed on EuRoC V1 beside OpenVINS (ours its equal or better but
on the hard sequence), the lidar-inertial one on NTU VIRAL's eee_01 beside FAST-LIO (0.20 against 0.42 percent, the
sweep stamped at its middle), the tool image `docker/Dockerfile.tools` apart from the flight; TUM-VI stays not done. What was asked for and how it was built is kept below as it was
written; this is what came of it.


**Type:** validation without a vehicle. Discussed with the project owner on
2026-10-09 and recorded; begun on 2026-10-10 by the owner's word, the first
step done on 2026-10-11 (below).

**Hard prerequisites:** none. It needs no hardware and no simulator.

The offline replay of the visual-inertial estimator (`tools/vio/replay_vio.cpp`,
`tools/vio/dark_drift.py`) runs the filter over a flight's record (frames, IMU,
truth) with no simulator. The records come from Gazebo today; the same replay
can take a public recording from a real vehicle, and that is the only way,
without a vehicle, to learn how the estimator behaves on real sensor noise
and whether anything in it is fitted to Gazebo. It also carries the debt of
the reference visual-inertial system ([technical_debt.md](technical_debt.md),
Localization): the reference and ours can be compared on the same record.

Candidate recordings, all public, each with a calibration and a reference
trajectory in the role Gazebo's truth plays here:

- EuRoC MAV (ETH): stereo and IMU on a drone, a machine hall and a room,
  reference from a laser tracker and motion capture. The first check and the
  industry's common one. 752 x 480 global-shutter stereo with radial and
  tangential distortion.
- TUM-VI, UZH-FPV: wide-angle stereo and IMU, corridors; aggressive flights.
  Drift over a first pass, the tracker's limits.
- Hilti SLAM Challenge, NTU VIRAL: 3D lidar and IMU, for the lidar-inertial
  estimator.
- SubT-MRS: underground, darkness, smoke, dust, with lidar, cameras and a
  thermal camera: the material of items 17, 18, 22 and 23.

What it takes, in order:

1. A converter from the recording's layout (images, IMU CSV, reference CSV)
   to the replay's record. Small for EuRoC.
2. The camera model as configuration, not geometry derived from the field of
   view: `stereo_depth_node` and the estimator take the intrinsics from
   `horizontal_fov_rad` and the image size (1280 x 960) and treat the pair as
   ideal and rectified, which Gazebo's cameras are and real ones are not.
   Intrinsics, distortion, the baseline, the camera-to-IMU extrinsics, the
   IMU's noise and the time offset come from the recording's calibration,
   and the frames are rectified first.
3. The comparison with the reference the way `dark_drift.py` compares with
   the truth.

The same path serves the stereo depth and the obstacle memory, fed with the
recording's frames, and the lidar-inertial estimator with the recording's
clouds.

What it cannot show: the loop is open. The vehicle in the recording flew as
it flew and does not answer to the stack, so the planner, the control and the
braking contract stay in the simulator. A first step that fits in a day: one
EuRoC sequence through the replay; the drift against the reference beside the
0.1 to 0.4 percent of the path the simulator shows.

### The first step, done (2026-10-11, specification K33)

The three steps above for EuRoC: the pair is configured by its calibration
and rectified in both camera nodes (the simulated pair passes untouched),
`tools/vio/euroc_to_replay.py` converts a sequence, `tools/vio/euroc_report.py`
measures a replay against the reference. Vicon room 1 from the ETH Research
Collection (DOI 10.3929/ethz-b-000690084), the data outside the tree:

| Sequence | Path | ATE (4-DOF aligned) | RTE over 10 m | At the end |
| --- | --- | --- | --- | --- |
| V1_01_easy | 58.6 m | 0.050 m | 0.183 m | 0.11 % of the path |
| V1_02_medium | 75.9 m | 0.046 m | 0.111 m | 0.17 % |
| V1_03_difficult | 78.9 m | 0.115 m | 0.165 m | 0.42 % |

Beside the simulator's first passes (0.1 to 0.4 % of the path) the real
recordings drift alike; the details are in
[localization.md](localization.md). Not done yet: the machine hall
sequences, TUM-VI (its fisheye pair needs another camera model; the archive
of room 1 is downloaded), the stereo depth and the memory on the
recordings, the lidar recordings, a reference system on the same records.

**Done on 2026-10-27 (the root goal's part 5; specification K50;
`docs/localization.md`):** the two steps below. OpenVINS beside ours on
V1_01, V1_02 and V1_03 (the machine hall's server answered nothing): ATE
0.057 / 0.080 / 0.058 m against ours 0.050 / 0.046 / 0.115, the relative
error over 10 m 0.082 / 0.092 / 0.083 against 0.065 / 0.047 / 0.130, the end
0.31 / 0.23 / 0.08 % against 0.11 / 0.17 / 0.42 %: ours the reference's equal
or better but on the difficult sequence. NTU VIRAL's eee_01 (an OS1-16 sweep
that moves) through ours and through FAST-LIO: stamped at the sweep's start
ours loses its map at 228 s; stamped at the sweep's middle (the one probe)
it drifts 0.20 % of 237 m (ATE 0.255 m, the registration p95 32 ms);
FAST-LIO with per-point compensation 0.42 % (ATE 0.379 m). The board's
driver needs the stamp at the sweep's middle at least; the tool image is
`docker/Dockerfile.tools` (`tools/tools_image.sh`), nothing of it in the
flight. TUM-VI stays not done.

### What came next (recommended on 2026-10-13, done on 2026-10-27)

The owner's question whether ready estimators would have served better
([design_questions.md](design_questions.md): not in vain, not to be redone,
to be added to) ends in two steps here, both on recordings and both before
any hardware:

1. A reference estimator, OpenVINS, over the same EuRoC sequences beside
   ours: the cheapest measure of how good ours is. It needs Ceres and
   Boost.Filesystem in a tool image apart from the flight image. The owner
   set it aside on 2026-09-25 until the drift rework
   ([technical_debt.md](technical_debt.md)); this recommends it before the
   hardware instead, and the decision stays the owner's.
2. One public lidar recording (Hilti or NTU VIRAL) through the
   lidar-inertial estimator and through FAST-LIO2: a real sweep moves during
   its 100 ms and ours does not compensate it, and the replay decides
   between writing the compensation and adopting FAST-LIO2 for the lidar
   build ([hardware_builds.md](hardware_builds.md)).

### 29. Telling A Failing Light From A Dark Place (Completed)

Shipped in [v0.6.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.6.0) on 2026-10-24; closed on 2026-10-14 (0d2141a4); the carried light judged by
its driver's reading and the time-of-flight ring (F18, K12), a dark place
under a working light left unmarked (K14, K15), the large dark hall crossed
(r1488 to r1490) and the failing and lost light flown home or landed; the
empty volume where the light reaches nothing is item 31's, the void (K46). What was asked for and how it was built is kept below as it was
written; this is what came of it.

**Type:** new capability; decided by the project owner on 2026-10-13
(specification F18), after the question of a camera vehicle in a large dark
space ([design_questions.md](design_questions.md)).

**Hard prerequisites:** none in the simulator; a light driver that reports
its state on a real vehicle ([hardware_builds.md](hardware_builds.md)).

In empty darkness, a large hall or open ground at night at a height, the
frames see nothing within the light's reach, and the vehicle cannot tell
that from a light that failed (specification I7): after 3.5 s it judges the
light unreliable (K12) and turns home. A mode that reads darkness as free is
not built: the estimator still has nothing to track (dead reckoning after
1 s, K13, the descent 3 s later, K19), a lit surface without texture gives
the stereo pair no depth, and a light that really fails in such a mode flies
the vehicle blind at speed.

What is built:

1. The light's own reading. On a vehicle, the driver's current or a
   photodiode at the emitter; in the simulator, the light's state as such a
   driver would read it, published on board (the injector's schedule never
   reaches the navigation: only what a driver measures does). K12 judges a
   failing light by it; the frames' judgment stays for what the reading
   cannot see (a soiled lens, a lamp that dims while drawing its current).
2. The time-of-flight ring as a second witness: a unit, with its own
   emitter, reads a surface within its range inside the camera's field
   where the frame is dark: the light has failed.
3. Darkness with a sound light stays observed unobservable to the braking
   contract (I2, F6, K14): no speed into it, the speed the ring confirms at
   most; but no "unreliable light, home" for it.

For a truly large space the answer stays another sensor: the lidar build
(40 m, its own emitter) or GNSS in the open at a height; this item does not
make the camera vehicle fast there, it stops it from aborting.

The darkness this item meets is the cameras' noise's (item 18 stage 1): since
2026-10-14 the simulated pair carries about a level of noise (specification
A12), where the noise in effect before was a tenth of one and the edge of the
light's reach matched as if noiseless. With it the frames alone judged a
sound light unreliable in ordinary flights (r1464 at 2.6 levels, 13.6 s in;
r1465 at 1.2, 50 s in, in a flicker's dip): the light's own reading is what
answers that. The guaranteed range (K7) measured again with it is 5.2 m,
not 6.4 (r1476), with the noise mask estimating the noise by brightness:
the error grows with the depth as the light's falloff leaves far surfaces
dim against the noise.

Complete when, in the simulator: a large dark hall (a world of its own, its
floor textured and lit by the carried light only) is flown to its goal by
the truth without a false judgment of the light and without contact (three
flights); the light's failure is still judged, by the reading, no later
than K12 judges it today (light-failure and light-lost, a flight each);
an ordinary camera flight unchanged in speed (one flight); I7 rewritten.

**The hall, built on 2026-10-14** (`environments/fixtures/dark_hall`,
`./scripts/sim_dark_hall_headless.sh`, [`scenarios.md`](scenarios.md)): 80 x
60 m and 20 m high, its floor and walls in a concrete texture of Urban's own
statistics, the pad 4 m from the west wall and the goal 64 m east, flown at 6 m,
where the pair's lowest rays meet the floor 7.6 m away. Urban's own largest
room (the start's hall, about 25 m across, its walls 12.5 m from its centre:
beyond the pair's confident depth but within the light's 25 m) is where every
flight takes off, and with the pair's noise in effect it is already where the
frames judged a sound light unreliable (r1464, r1465).

**Done on 2026-10-14** (code 0d2141a4; the recording checker 4c2a967d).
Built: the light's injector publishes what its driver would read of the
output (`/carried_light/output_share`, never its schedule); the depth node
publishes the share of the time-of-flight surfaces in the left frame the
frame shows dark; the mission monitor counts an outage when the reading is
under 0.25 of nominal, or when a blind frame shows half the ring's surfaces
in its field dark, and with no reading as before (K12, I7, F3). Under a
light read sound a blind frame is the place's darkness: it marks nothing in
the memory (K14; marked, the hall closed every direction and the routes
climbed to 17 m under the 20 m ceiling, r1473) and the memory's observed
range answers for every motion (K15), so the hall is crossed at about
0.5 m/s. Found on the way: the hall's grey background past the far clip
took the tracker's budget and a never-aided estimator restarted on the
autopilot's pose into a wall (r1466; the background black, K28); the pad
moved 4 m from the west wall facing it, the concrete given an aggregate
(r1468, r1469); every headless flight records its true pose, a snapshot
volume or none (r1466, r1471); the pair's noise (A12) cost the camera
flights their speed through the matcher's strays blocking routes by the
latest scan (K35, K7 5.2 m, the noise mask by brightness; 5563787d).

| Flight | Run | Result |
|---|---|---|
| Before, the rules of 8771edbe | r1475 | the light judged unreliable 41.8 s after readiness, home; video `log/videos/2026-10-14/dark-hall_r1475_*.mp4` (BAD by A11's minute alone: the flight lasts 50 s) |
| The hall, three | r1488, r1489, r1490 | goal by the truth 0.44, 0.53, 0.38 m; no contact, no judgment of the light; estimate error 0.43, 0.39, 0.42 m at most; r1490 marked darkness in a flicker's dip under 0.25 and flew around it; videos `dark-hall_r1488/9/90_{follow,top}.mp4`, RECORDING ok |
| The light fails | r1486 | judged 32.0 s after readiness (55.1 s of simulation; the reading's line says 55.0 after take-off; the frames judged r1445 at 76.9 / 100.6), home 0.29 m, landed |
| The light lost | r1487 | judged 55.0 s after readiness, dead reckoning from 75.0 s, blind landing at 79.8 s, whole |
| Ordinary camera | r1485 | 1.429 m/s, goal 1.09 m, no contact (R3 holds; the base's 1.714 was a noiseless pair's, the range 6.4 m against 5.2) |

Diagnostic flights on the way, none acceptance: r1464 to r1474, r1476 to
r1484.

### 31. Flight Through The Dark Void (Completed)

Shipped in [v0.7.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.7.0) on 2026-10-31; closed on 2026-10-25 (the root goal of 2026-10-24, part 2; specification
K46, K47): the void frame and the void state, the hall crossed at 14 m on dead reckoning without a hold, a descent or a
contact, the wall's goal within 2 m by the truth five times and the hall's middle within the published uncertainty
plus 2 m three times (r2173 to r2190), the drift budget and the arrival rule registered; the lidar's start in the
hall repaired on 2026-10-29 (K51, r2367). What was asked for and how it was built is kept below as it was
written; this is what came of it.


**Type:** new capability; decided by the project owner on 2026-10-17 after
the dark hall's recording (r1488): "complete darkness" means an empty dark
volume with nothing within the carried light's reach, not a floor lit a few
metres below. Built on 2026-10-25 in the root goal of 2026-10-24 (part 2);
the commits 96dea606, 7edd38eb, e5ce373c, 1f4e1eec, ecc12e03, affcd811 and the documents.

**Hard prerequisites:** items 17, 29 and 18 (the light judged by its driver,
F18; the blind frame, K12; the ladder's hold and descent, K19; the rotors'
drag as a velocity, K21).

What is settled: a black frame under a working light is a measurement of
emptiness to the light's reach, not an absence of guarantee; in the void the
safety rests on the light and the braking contract (K9) and the position on
dead reckoning, whose drift is the physics of the IMU; so the 2 m capture
radius is not required in the void: a goal there is reached when the true
error is within the estimator's own published uncertainty (two sigma) plus
2 m, and a goal with a surface within the light's reach at its end is reached
within 2 m as always. No GNSS in any flight.

**Stage 0, the light's reach (done).** On the rig (`tools/smoke_rig.py`
levels `height=Z,wall=60,record=20`, `tools/smoke_rig_report.py --reach`,
`log/tools/void/reach`): over a textured floor the pair is blind to the
braking contract from 5 m below (matched 0.062 of the frame), matches
nothing from 13 m; the tracker holds 20 features or more in 97 percent of
frames at 11 m and in none at 13 m (the 10 it holds there are the imager's
noise, which `minimum_frame_features` refuses, r933); the frame's raw
brightness at the 95th percentile 67, 40, 25, 14, 9, 7 levels at 3, 5, 7, 9,
11, 13 m; a facing matte panel of the location's darkest albedo (0.07) at the
pair's confident depth, 5.2 m, 52 levels, of the mean (0.10) at 7 m 46, at
9 m 33. The void's radius is the confident depth (K7); the tracker's reach
over a floor 11 m; both in K46.

**Stage 1, the void (done; K46, K13 and K19 amended).** The stereo node
judges a frame a void frame when the light's reading is sound (K12's 0.25),
the frame's raw brightness is under 46 levels at the 95th percentile (what a
surface of the mean albedo gives from beyond the reach), the pair's hits
within the confident depth cover under 1 percent of it, and no surface of the
time-of-flight ring lies in the camera's field, and publishes the confident
depth on `/stereo_depth/void_range` (`stereo_void_frame.hpp`). The planner
lifts the braking contract's frame range to it: the vehicle crosses the void
at the speed the depth admits, 1.8 m/s (0.5 before, on the memory's range,
K15). The camera estimator enters the void with such a frame
(`visual_inertial_void.hpp`): past its unaided second it is in the void, not
dead reckoning, the height held and the drag fused as on dead reckoning, the
pose given to the autopilot as a sound one (so that the offboard flies the
plan and the descent of K19 is not due), the ten seconds of K13 not spent;
the void ends with the next update or, the witness gone (a failed light),
with dead reckoning from that moment. The memory marks nothing (K14, as
before). The first lines (24 levels, any match) refused the hall at 14 m,
whose floor 14 m below and plain ceiling 6 m above, both beyond the reach,
lift the frame to 18 to 39 levels and give 5 percent of matches 12 to 20 m
away (r2174: the estimator flickering, 28 holds, two descents of K19 to
11 m); the lines are what surfaces beyond the reach can give.

**Stage 2, the drift and the stitch (done).** Measured on the crossings of 2026-10-25 (K47): through the void the camera estimator's horizontal error grows by 0.01 to 0.06 m/s at 1.8 m/s, 0.7 to 3.6 percent of the distance flown (eight crossings of 16 to 31 s and 29 to 53 m, r2180 to r2187), the estimate's speed 1 to 3.5 percent above the truth's along the track, the height within 0.6 m on the barometer; the published horizontal sigma after 25 to 31 s is 2.3 to 3.2 m and the error at the void's end 0.2 to 0.97 of it. The drift was the drag model's velocity: with K21's 0.106 1/s the estimate ran 8 to 13 percent fast and drifted 0.13 to 0.23 m/s, 8 to 14 percent of the distance, 2.8 sigma on one crossing (r2175 to r2178); re-fitted on those crossings to 0.118 the drift fell tenfold. The east wall's features take the estimate back within 2 m before the wall's goal is acknowledged; the memory maps the wall where the estimate stood (1 to 25 phantom voxels in the wall's band of five crossings, 0.0 to 0.5 percent of the hall's occupied voxels), and the map registration never applies at 14 m.

**Stage 3, the scenario and the acceptance (done).** `sim-dark-void-headless`
and `-gui` (`dark_void_point_to_point_scenario.json`: a climb straight up
over the pad to 14 m, then the goal 4.5 m from the east wall at 14 m, the
wall within the reach at the end; `dark_void_mid_point_to_point_scenario.json`
by `DARK_VOID_SCENARIO_PATH`: the hall's middle, an arrival on dead reckoning
alone, judged within two sigma plus 2 m by the mission check), the row in
[`scenarios.md`](scenarios.md), `tools/render_flight.sh dark-void`. A single
goal at height was flown as the straight route, a ramp 2 to 10 m over a floor
the light still reached (r2173), hence the climb first.

| Flight | Run | Result |
|---|---|---|
| The reach by height, the rig | log/tools/void/reach | the floor matched 0.179 of the frame at 3 m, nothing at 13 m; the tracker 91, 62, 49, 32, 23, 10 features at 3, 5, 7, 9, 11, 13 m; raw brightness 67 to 7 at p95 |
| The probes | r2173, r2174, r2175 | r2173 a ramp at 2 to 10 m, no void (one goal at height); r2174 at 14 m with the first lines, no void, 28 holds, two descents to 11 m; r2175 the first crossing in the void, 0.52 m by the truth, drift 0.19 m/s |
| The drift, three crossings and the coefficient | r2176, r2177, r2178 | 0.84, 0.27, 0.38 m by the truth; the drift 0.13 to 0.23 m/s with 0.106, the estimate 8 to 13 percent fast along the track; K21 re-fitted to 0.118 |
| The wall's goal, eight | r2180 to r2187 | 0.39, 0.80, 0.44, 0.42, 0.90, 0.31, 0.26, 0.36 m at the climb goal and 0.99, 0.38, 0.24, 0.94, 0.73, 0.76, 1.59, 1.27 m at the wall's by the truth; the drift 0.7 to 3.6 percent of 29 to 53 m of void; one hold a flight at the aided-void boundary, no descent, no contact |
| The hall's middle, three | r2188, r2189, r2190 | 0.65, 1.00, 1.21 m by the truth against two sigma plus 2 m of 5.59, 6.00, 6.55; the drift 1.0 to 2.6 percent |
| The lidar reference | r2179; r2367 | r2179 not flown: the lidar-inertial estimator unhealthy at the pad (matched 0.18 under 0.3), no pose, arming denied. Repaired on 2026-10-29 (K51, the count of matched points beside their share): r2367 crossed the hall on the lidar, the climb goal 0.34 m and the wall's 0.53 m by the truth, no contact, max speed 6.21 m/s, the height error within 0.08 m over 82 m, 70 of 71 scans healthy; r2366 with a floor of 500 points came down on the pad and set the floor at 200 |

**Measurement:** `log/tools/journal/goal_journal_dark_void.md`,
`log/tools/void/` (the rig's reach table, the crossings' void episodes with
their drift against the published sigma, the along-track scale of the drag's
velocity), the flights under `.keep`.

**Completion:** the void frame and the void state built and registered
(K46), the hall crossed at 14 m in the void without a hold, a descent or a
contact, the wall's goal reached within 2 m by the truth five times, the
middle's within the published uncertainty plus 2 m three times, the drift
budget and the arrival rule registered. Open: the lidar cannot start in the
hall (its estimator matches 0.18 of its own first scan, under its 0.3 line,
where the far walls and the ceiling give sparse points without planes), so
the lidar reference was flown on 2026-10-29 after the repair (K51: the health
reads the count of matched points beside their share; r2367 above); a real barometer drifts with the weather (item 35); a plain
surface darker than the location's albedo within the reach is the ring's
2.8 m to catch.

### 32. Hot Smoke: A Heat Source In The Plume (Completed)

Shipped in [v0.7.0](https://github.com/formiat/px4-ros2-drone-nav/releases/tag/v0.7.0) on 2026-10-31; closed on 2026-10-28 (3fad5b09, the root goal of 2026-10-24, part 4;
specification K49): the thermal core back on the vehicle with a decision, the hot plume and the hot room kept out of
in five of five flights on each sensor set with no contact (the lidar r2339 to r2348, the stereo set r2322 to r2331
on the whole ball), the cold room still crossed, the standoff a measured number, the core's reading the decision's
evidence in every flight. What was asked for and how it was built is kept below as it was
written; this is what came of it.


**Type:** new capability; proposed by the project owner on 2026-10-17, when
the thermal core of item 18 stage 3 was taken off the vehicle (F4): a sensor
earns its place with a decision that reads it, and this is the item that
gives the core its decision.

**Hard prerequisites:** items 18 and 23 (smoke in the simulator, A13; dense
dense smoke detected, K37, K38; moderate smoke crossed, K43);
item 18 stage 3 as the measurement of what a thermal frame carries (K39).
Item 27 (a real vehicle) comes after.

**Validation environment:** the urban location with its smoky places (F4),
some of them carrying a heat source.

Smoke the vehicle meets is cold or hot: fog, dust and smoke that has cooled
are crossed when moderate and avoided when dense (items 18 and 23); a plume
with a fire or a smouldering source in it is not entered at any density,
and the source is kept at a distance. Neither the stereo pair, the
time-of-flight ring nor the lidar can tell the two apart: they read density.
A thermal core does, through the smoke, which long-wave infrared crosses
almost untouched (K39: through 1/m the thermal frames are identical to the
clear ones).

The order:

1. **The simulator's heat.** A heat source as an object of the world with a
   temperature (the thermal camera of the exploration already renders the
   world's surfaces by temperature, `thermal_camera`); a smoke zone may carry
   one (`SMOKE_ZONES` gains a temperature, or a separate `HEAT_SOURCES`
   string, whichever is the smaller change), and the smoke relay passes the
   thermal frame through the smoke untouched (K39).
2. **The core back on the stereo set, with its reading.** The FLIR Lepton
   3.5 class core (160 x 120, 9 Hz) on the camera link, as mounted in stage
   3; its frame read for the share of pixels warmer than the frame's median
   by a threshold measured on the rig, and for their direction in the frame;
   the reading published with its direction, as the ring's smoke share is
   (K38), and recorded in the memory as a heat mark with a radius.
3. **The rule.** A heat mark is a prohibition with a standoff: the route
   keeps a measured distance from the source, the plume that carries it is
   not entered whatever its density (the moderate room's crossing of item 23
   is refused when the room is hot), and the mission events say so ("HEAT
   SOURCE", the overlay's words). The lidar vehicle, which carries no core,
   either takes the same core or records that it cannot tell hot smoke from
   cold: the owner's call, taken with the measurement of step 2.
4. **Scenarios and acceptance.** A hot plume on the way (gone round at the
   standoff), a hot moderate room on the way (not entered: round it, or home
   when there is no way round), and the cold moderate room of item 23 as
   the control (crossed as before); five flights each on the stereo set, and
   on the lidar as step 3 decides; no contact, B in truth within 2 m where B
   is reachable, home otherwise.

Size: M for steps 1 and 2, M for step 3, L with the series.

**Built on 2026-10-26 (steps 1 to 3, the root goal's part 4; specification
K49):** `HEAT_SOURCES` ("x,y,z,r,T") rides with `SMOKE_ZONES`, the smoke
model's relay creating each source in the running world as a static body
tagged with its temperature (A13); the Lepton-class core is back on the
stereo set with its cap at 650 K (K39), its frame relayed as the pair's and
read by the depth node (`thermal_reading.hpp`): a source is four pixels or
more at 30 K over the frame's median, the threshold set on the rig against
the location's 288 and 311 K surfaces (23 K apart; the lamp warms nothing,
the tag is binary); its range the nearer of the hits in the hot cone and the
source's bearing-only fix over the motion, else the body's apparent size
(7000 pixel-metres squared for the 0.3 m body the rule assumes); the
standoff 4.0 m, laid as the ball round the range point (observed
unobservable, every half second while the core reads the source, a fifth
of the range wider for the size's error; the near half alone until
2026-10-28), the motion bounded before it, every return through the ball
dropped, the words
`HEAT_SOURCE seen=true` on the mission's events and HEAT SOURCE on the
overlay; the truth judged never within the standoff
(`validate_standoff_from_heat_sources`). The scenarios: `sim-urban-hot-smoke-plume-*`
(the location's plume at (12,22,11) with a source at its centre) and
`sim-urban-hot-smoke-room-*` (the moderate rooms with a source on the west
hall's crossing), the cold room the rooms scenario as it is; flown only for
this item's acceptance (the owner, 2026-10-24).

**Accepted on 2026-10-27 (step 4; the stereo set; specification K49):**

| Group | Runs | Standoff (nearest, m) | B or home | Speed (m/s) |
|---|---|---|---|---|
| The hot plume, 5 of 5 | r2257 to r2261 | 4.97, 4.21, 5.52, 5.82, 5.79 | B at 1.31, 1.63, 1.32, 1.03, 1.59 m | 1.279, 1.424, 1.407, 1.354, 1.310 |
| The hot room, 5 of 5 | r2262 to r2266 | 10.73, 12.37, 11.51, 10.84, 11.48 | home every time, by the light's charge (the battery trigger at about 250 s), within 0.3 m of the start | — |
| The cold room, the control, 5 of 5 | r2271 to r2275 | no source | B at 0.52, 0.40, 1.30, 0.71, 1.80 m | 1.305, 1.137, 1.292, 1.273, 1.185 (K43's 1.1) |
| The lidar on the hot room, no core | r2276 | 1.30: crossed | B at 0.25 m | 2.343 |
| The clear location, the stereo set, 5 of 5 | r2277, r2279 to r2281, r2288 (r2282 acknowledged B at 2.09 m, the estimate's drift) | the core silent | B at 0.9 to 1.6 m | 1.268, 1.277, 1.395, 1.345, 1.291 |
| The clear location, the lidar, 5 of 5 | r2283 to r2287 | no core | B at 0.3 to 0.7 m | 2.207, 2.647, 2.249, 2.656, 2.467 |

No contact in any flight, the dense plume never entered (R7), the core's
reading in the log of every hot flight (`HEAT_SOURCE_MARKS`, the
`HEAT_SOURCE seen=true` event). Nineteen probe flights preceded the
groups (r2235 to r2253; `log/tools/journal/goal_journal_hot_smoke.md`):
the mark's shell had holes the vehicle flew through, the braking contract
let the memory's observed range override the sensor's bound, a range
guessed ahead moved with the vehicle, a gaze held on the source froze the
flight; what stands is in K49. The lidar set, which carries no core,
crosses the hot room (r2276): by the measurement the lidar build takes
the same core (`docs/hardware_builds.md`, L1), read on the lidar set as
below.

**Accepted on the lidar set on 2026-10-28 (step 4; specification K49):**
the core on `lidar_3d_v1`'s link, read by `thermal_core_node`: the range
from the scan's hits in the hot cone without the lidar's weak returns
(K37: the hot room's smoke answered in the cone at 4.6 to 4.9 m whatever
the source's range), the fix or the body's size otherwise; the source held
in the map until the core looks at the place and reads no heat for a
second (the lidar sees all round and carved a ball the core had turned
from); the memory reads the scan through the node, every ray that enters
the held ball ending at its surface; the bearings turned with the
lidar-inertial estimate's full orientation at their own stamps; the
heading follows the motion while the core is carried, at 0.8 rad/s at
most, inside what the estimate, which registers each sweep whole, has
kept up with (at 2 rad/s it lost 2.3 m of height in smoke and the vehicle
met the floor, r2297). Ten probes (r2289 to r2298) found and repaired
what K49's change log lists. The node costs 0.08 cores at p50 and 0.10
at p95 (r2299, `tools/process_cores.py`).

| Group | Runs | Standoff (nearest, m) | B or home | Speed (m/s) |
|---|---|---|---|---|
| The hot room, the lidar, 5 of 5 | r2299 to r2303 | 8.55, 8.55, 10.30, 8.50, 9.00 | home every time, by the topological proof, within 0.34 m of the start | — |
| The hot plume, the lidar, 5 of 5 | r2304 to r2308 | 5.30, 5.46, 4.75, 5.80, 4.74 | B at 0.32, 0.42, 0.34, 0.73, 0.21 m | 2.401, 2.108, 2.479, 2.518, 2.299 |
| The clear location, the lidar, 5 of 5 | r2309 to r2313 | no source | B at 0.25 to 0.57 m | 2.273, 2.008, 2.441, 2.478, 2.264 (R3's 2.0) |
| The hot plume, the stereo set, the whole ball, 5 of 5 | r2322 to r2326 (r2321 on the half ball: 3.66 m) | 7.15, 7.74, 5.78, 8.00, 7.31 | B at 0.89, 1.23, 1.72, 1.37, 0.87 m | 1.305, 1.339, 1.278, 1.357, 1.289 |
| The hot room, the stereo set, the whole ball, 5 of 5 | r2327 to r2331 | 9.38, 11.10, 9.20, 10.71, 9.35 | home every time, by the light's charge, within 0.4 m of the start | — |
| The clear location, the stereo set, 5 of 5 | r2333 to r2335, r2337, r2338 (r2336 at 1.179 m/s, under R3; r2314 to r2320 before the whole ball) | the core silent | B at 0.69 to 1.44 m | 1.248, 1.282, 1.286, 1.345, 1.336 |

The whole ball changed the code after the lidar's groups above, so they were
flown again on the final code (2d794294; the commits after it change
documents and tools alone), with the cold room as the control:

| Group, the final code | Runs | Standoff (nearest, m) | B or home | Speed (m/s) |
|---|---|---|---|---|
| The hot room, the lidar, 5 of 5 | r2339 to r2343 | 10.17, 10.38, 9.55, 7.90, 10.93 | home every time, by the topological proof, within 0.31 m | — |
| The hot plume, the lidar, 5 of 5 | r2344 to r2348 | 5.38, 5.66, 5.34, 4.50, 5.38 | B at 0.33, 0.30, 0.95, 0.49, 0.98 m | 2.438, 2.198, 2.385, 1.991, 2.020 (r2347 under R3's 2.0 by 0.009: a finding, the hot scenarios' criteria are the standoff, the arrival and no contact) |
| The clear location, the lidar, 5 of 5 | r2349 to r2353 | no source | B at 0.22 to 0.49 m | 2.493, 2.306, 2.052, 2.441, 2.029 |
| The cold room, the stereo set, the control, 5 of 5 | r2354, r2355, r2358, r2360, r2365 (r2356, r2357, r2359, r2361 to r2363 stopped by the watchdog under the host's load) | no source | B at 1.17, 0.53, 1.01, 1.28, 1.37 m | 1.310, 1.297, 1.263, 1.109, 1.239 (K43's 1.1) |

No contact in any flight. The core read the source in every hot room
flight on the lidar (`HEAT_SOURCE seen=true`, `HEAT_SOURCE_MARKS`), and in
no hot plume flight on it: the lidar never enters the dense plume (K37,
R7), it went round it with its heading along its way, and the source at the
plume's centre stayed outside the core's 57 degrees; there the standoff is
the plume's own reading's. The stereo set's regression after this work
found the half ball's open side (r2321: the vehicle lost the source, went
round the half and came within 3.66 m from behind, as r2250): the whole
ball is laid from every stand now and the stereo set's three groups were
flown again on it (above).

This item is complete when the hot plume and the hot room are kept out of
in five of five flights on the stereo set with no contact, the cold room is
still crossed, the standoff is a measured number in the specification, and
the thermal core's reading is the decision's evidence in every flight.
