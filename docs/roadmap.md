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

One entry of the debt register lands here, attached on 2026-09-26: the cave
and the finals locations, imported and never flown (N2), are flown by the
survey acquisition itself, which covers every reachable part of an
environment before it persists the map. The 2D lidar path (R1) was to go
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

## 18. Flight Through Transient And Scattering Obstacles

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
  part of the vehicle, not an option of a flight.
- **Smoke is always in the location**: a number of smoky places in every
  flight, so that every acceptance flight exercises the smoke handling, and
  none of them blocks the way from A to B; the ordinary mission stays
  reachable.
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

### Stage 0: Smoke And The Lidar In The Memory

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

### Stage 1: Smoke In The Simulator

Gazebo already carries the tool, built for this purpose: the
`ParticleEmitter` system was added for the DARPA SubT virtual track to mimic
the smoke machines of the systems track and the dust of collapses, and its
`particle_scatter_ratio` sets how the particles reach each sensor. The
effects are defined per sensor type: an RGB camera sees the particles, a
depth camera and a GPU lidar scatter on them with added Gaussian noise, and a
thermal camera does not see them at all. Our time-of-flight sensors are
`gpu_lidar`, so a plume blinds them in simulation as it does in reality, and
so is the 3D lidar of the lidar profile. A plume is a materialization variant
of the location, as the dark world of item 17 is; nothing reaches production
code. Global fog through `<scene><fog>` under ogre2 is checked before it is
relied on.

**The vehicle's own light in smoke** is measured here, before anything is
built on it. This item was written for a lit location; since item 17 the
only light is the one the vehicle carries, beside the cameras, and smoke
scatters it straight back into them. That may help, a bright veil being
unlike the black silence of distance, or it may harm, the matcher finding
correspondences in the veil and returning a depth that is not there. What
the pair's depth, its matched share and the estimator's tracked features do
in a plume lit from the camera's own position is the first measurement of
this stage, by density; a light set apart from the pair is the remedy to
compare.

### Stage 2: Detecting Smoke

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

### Stage 3: A Thermal Channel

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

### Stage 4: What The Vehicle Does

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

## 21. Moving Obstacles: Doors And Bodies That Move Slowly

**Type:** perception and safety, general; not tied to a sensor.

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

## 23. Flight Through Moderate Smoke

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
  visual effect only, this item starts with the sensor models.

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

## 24. Item 19's Proof, Flown

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

## 25. The Estimator On Real Recordings

**Type:** validation without a vehicle. Discussed with the project owner on
2026-10-09 and recorded; not started.

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
real work. The D* Lite planner, the memory's ray tracing and the lidar
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

### Later blocks, for when a vehicle exists

2. Build and launch: the package needs Gazebo to build (`gz-sim8`,
   `gz-transport13`, `sdformat14` in CMake) and the onboard and simulation
   targets are one; the image is x86 with CUDA architectures 75 and 86; there
   is no launch without the simulator and `use_sim_time` is set in every
   node and held by a contract test; the uXRCE agent is UDP only; MPPI keeps
   12 threads.
3. Sensors: no camera calibration, distortion or rectification (intrinsics
   from the field of view, an ideal pair, 1280 x 960 only, the pair's stamps
   equal); no time-of-flight driver (64 zones in `gpu_lidar`'s layout, within
   67 ms of the frame); the barometer, the light's charge and the camera's
   gain come from the simulator; no onboard control of the light; the lidar
   needs an organised 360 x 181 cloud without motion compensation; the
   extrinsics live in SDF, C++, YAML and Python at once.
4. Time and frames: the real clock synchronisation (`UXRCE_DDS_SYNCT=0`
   today, zero EKF2 delays, the 4 ms frame-to-IMU offset fitted in the
   simulator); the map is anchored to Gazebo's spawn (origin 54, 54; the
   345 x 525 x 40 m grid of the Urban Circuit); the initial heading is a
   parameter.
5. The vehicle model: every constant is fitted to the x500 in Gazebo
   (accelerations, lags, the body, the rotor drag 0.106, the yaw-rate lead
   marked to fit again); a real frame is identified by hand first.
6. Checks before a flight: no CI, no HITL, no replay of the whole stack from
   a record; onboard logs without rotation; the bag without setpoints and
   commands; crash detection rests on Gazebo's contacts, and the trial's
   referee shares a class with the mission's logic.

Order: block 1 in the simulator; the build split and a launch on recorded
data; cameras on a bench; HITL with a real Pixhawk; tethered flights in a
net, manual first, then a hold, then a short route. Months of one person's
work; the first two steps cost nothing.

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
  confident depth re-measured in the dark by that light stays 6.4 m (K7).
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
  stereo set at 1.0: 1.822 to 1.995 m/s (mean 1.897), B within 0.03 m, no
  contact, every flight counted (r1186 to r1190). The long flight to B and
  back and the goal-outside flight are set aside (A8).
- **A floor touched in flight is a crash, the clearance law carries the
  estimate's vertical error, the way home at 1.5** (2026-10-09, A9, K27,
  K11, 0a332c4d; the offboard split into three sources, acaa4b1f; its mode
  stream kept through a planner-loss hold, b48f89eb). The acceptance on
  b48f89eb, the stereo set at 1.0: 1.640 to 1.878 m/s (mean 1.750, the
  clearance law under K27 holding the reference under 2 m/s in 27 percent
  of the ticks against 11 before), B within 0.06 m, no contact, every flight
  counted (r1197 to r1201). The four scenarios recorded again on it are in
  `log/videos/2026-10-09/` (r1191, r1194, r1195, r1196): the low battery now
  turns 72 s in with 140 s against 128 estimated and is home with about 81 s;
  two light-lost takes (r1192, r1193) tipped on uneven ground four seconds
  after a level touchdown, the known risk of the blind landing
  ([technical_debt.md](technical_debt.md)). r1194 stood, on a slope of 12
  degrees, and its estimate ran 100 m away in four seconds: the rotor-drag
  model read the ground's friction (2.3 m/s² across the rotor axis) as 21
  m/s; readings beyond any flight's drag are left out now (K13, 208ab74e),
  and the acceptance on 5dd839b4 holds: 1.576 to 1.772 m/s (mean 1.700), B
  within 0.03 m, no contact, every flight counted (r1205 to r1210). The
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
  a point of the trail the vehicle flew, 25 m back, moved on as it is
  neared, each in its own mission epoch, and the start at the end.
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
