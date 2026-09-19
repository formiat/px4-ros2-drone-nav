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

### 14. Camera-Based 3D Perception Without Lidar Or Static Maps (Completed)

Closed on 2026-09-19 on the urban point-to-point mission with no lidar on the
vehicle and no static map; the design, the measurements behind it and what is
known to remain are in [`camera_perception.md`](camera_perception.md), the
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
r511 to r515: 2.65 to 2.82 m/s, no crash. Shadow comparison against the lidar
memory (stage 2): occupied precision 97 to 98 percent within one voxel,
recall 96 to 99 percent inside the third of the lidar's volume the vision
memory observes.

Known and not gated: the evidence age exceeds the 600 ms the contract charges
on about 1 percent of the ticks (624 to 832 ms at most) on a workstation that
holds a real-time factor of 0.9 beside the render, the image bridge and the
matcher; lateral tracking p99 0.305 and 0.320 m in two flights of five against
0.25 m; the closest pass to truth occupancy was 0.41 to 0.75 m from the
vehicle's centre to a 0.5 m voxel's centre against 0.84 m on the lidar; a
faced motion between the pair's field and a time-of-flight cone (52.4 to 67.5
degrees of elevation) is flown at the unobserved speed of 1 m/s; the
multi-vehicle launch carries the same defaults and has not been flown on them,
which waits for item 15 and for a host that renders eight cameras.
