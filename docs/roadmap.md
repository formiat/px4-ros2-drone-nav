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

## 13. GNSS- And Magnetometer-Denied Lidar-Inertial Navigation

**Type:** dependent localization stage.

**Hard prerequisites:** items 8 and 12.

Item 13 is accepted on Urban Circuit Practice 01 with the 3D lidar and no
static map. Autonomous test flights there require item 12's persistent
full-3D route and repair backend. This roadmap dependency must not create a
code dependency between the localization estimator and the route planner.

Add an optional navigation profile in which the aircraft does not use GNSS or
magnetometer fusion. This stage begins after item 8 provides production 3D
lidar and its timestamped full-6DoF acquisition-pose contract and item 12
provides autonomous routing through partially observed complex environments.
The aircraft retains its IMU and barometric altitude source and estimates
motion from lidar-inertial odometry instead of receiving global position and
heading from simulated navigation satellites and a simulated compass. Today
the heading comes from neither: the simulated magnetometer is not fused
(`EKF2_MAG_TYPE 5`) because it sits five to six degrees off, and
`simulation_heading_source_node` hands the autopilot the simulator's true
attitude, with a wandering bias and noise, through the external-vision
interface (`EKF2_EV_CTRL 8`). That node is simulation-only and is ground
truth inside the estimator's path; this profile removes it, and the
lidar-inertial estimator is the only source of heading.

Localization must remain a separate subsystem from obstacle memory and route
planning. A dedicated lidar-inertial estimator deskews 3D scans, propagates the
high-rate IMU state, registers scans against dedicated localization submaps,
and publishes a typed pose, velocity, covariance, quality, and frame identity.
Planner obstacle memory consumes that estimate; it must not become the
authoritative localization map, because a map assembled from an erroneous pose
can otherwise reinforce the same localization error.

Feed the estimate to PX4 through its supported external-odometry interface and
configure PX4 to fuse it while GNSS and magnetometer fusion are disabled. The
existing PX4 local-position output remains the stable contract for offboard
control, planning, mapping, and diagnostics. Gazebo ground truth is available
only to evaluation and referee components and must never cross into the
estimator or control data path.

Lidar-inertial SLAM builds revisioned localization submaps and uses loop
closure to maintain a locally consistent frame.

Missions with absolute map-frame goals require a declared initial map pose or
another explicit global reference, never a scenario-provided hidden
ground-truth transform.

Localization quality and geometric observability must be first-class runtime
signals. Repetitive facades and long feature-poor corridors can leave
translation or yaw weakly constrained even with 3D lidar. When the
estimate is stale, divergent, or insufficiently observable, the system must
stop publishing new executable motion and let the current finite path reach
its validated terminal state; it must not continue an invalid path or add a
sticky braking or geometric exclusion lifecycle.

Implement and validate this stage incrementally:

1. replay timestamped 3D lidar and IMU data offline and compare estimated poses
   with evaluation-only Gazebo truth;
2. fly one vehicle from a known initial pose using PX4 external odometry with
   GNSS and magnetometer fusion disabled;
3. add explicit estimator health;
4. add submaps and loop closure.

Measure position and attitude drift, velocity error, map alignment, loop
closure consistency, estimator latency, time without a valid executable
path, minimum obstacle clearance, and physical collisions. The starting
point is measured: with GNSS and the simulated heading, PX4's estimate sits
0.19 to 0.25 m from the true pose across the track at p95 and within 0.01 s
along it (the position-estimate check in `testing.md`, r340 to r356). This
stage is complete when repeated urban point-to-point 3D-lidar missions run
without GNSS, magnetometer data, the simulation heading source, or any other
control-visible simulator ground truth, hold that same check at no worse
than the GNSS figures, and localization failures produce an explicit safe
finite-path outcome instead of silent frame corruption.

## 14. Vision-Only 3D Perception Without Lidar Or Static Maps

**Type:** dependent perception stage.

**Hard prerequisites:** items 8 and 12.

**Validation environment:** Urban Circuit Practice 01.

Navigate the same no-static missions that item 8 and item 12 accept with the
3D lidar, with no lidar at all and no static map: the vehicle carries only a
video camera, and software recovers the shape of the surrounding geometry from
the video stream. The world model does not change. Item 8 fixed the boundary
between sensing and the raw world as a bundle of timestamped rays with a hit
at a range or a miss to a range, integrated into revisioned
`unknown/free/occupied` `Occupancy3D` under one full-6DoF acquisition pose. A
depth image is exactly such a bundle: one ray per pixel through the calibrated
optics, a hit where the pixel's depth is known and free space along the ray up
to it. Vision therefore enters the pipeline as a second producer of the same
beam observations that `obstacle_memory_3d_node` already integrates, and
everything downstream — obstacle memory, dirty-chunk transport, immutable raw
snapshots, swept validation, the persistent planner, MPPI, finite raw-safe
execution — runs unchanged. The stage proves that the world model is not
bound to one sensor, which is the property a real vehicle needs before any
sensor is swapped or lost.

### What The Simulator Provides

The simulator provides a calibrated stereo pair of RGB cameras rigidly
mounted on the airframe, their intrinsics and baseline, the IMU, and the same
pose source the lidar profile uses. It provides no depth camera, no RGB-D
sensor and no point cloud in the control path: depth from a simulated depth
sensor is a lidar by another name and would prove nothing. Simulator depth and
Gazebo truth occupancy are available to evaluation and referee components
only, as item 13 treats ground-truth pose, and must never cross into the
perception, planning or control data path.

Environments used for acceptance must carry surface texture. A stereo matcher
recovers depth from texture; an untextured flat wall is exactly where it
fails, and a wall that yields no depth is unobserved, not absent. Environment
candidates that render as uniform flat colour are textured before they are
used for this stage; the geometry, spawn points and mission goals do not
change.

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
camera frustum and of the confident depth range, and the speed policy limits
speed along the commanded motion by the guaranteed range in that direction. A
vehicle commanded sideways, backwards or vertically out of its own frustum
slows to what unobserved motion allows, which is the existing law applied
honestly rather than a new rule.

That makes heading a perception decision. The execution layer gains a gaze
policy that yaws the camera toward the commanded motion before the motion
exceeds what unobserved space allows, so that ordinary forward flight is
observed flight. Active choice of viewpoint for its own sake — moving to see
into a shaft before committing to it — is a later stage; here the camera only
follows the motion. The latest-lidar evidence that item 8 admits for bounded
final execution revalidation becomes latest raw evidence from whichever
sensor produced it; the admission rule, the freshness bound and the swept
validation do not change.

Localization is not part of this stage. The vehicle keeps the pose source the
lidar profile uses, and item 13's rule holds in reverse: visual-inertial
odometry, if it is ever added, is a separate estimator that this stage must
not depend on and must not be depended on by. The roadmap dependency between
the two must not become a code dependency.

### Implementation Order

1. Add the stereo rig to the vehicle model and bridge images and camera
   information; record timestamped stereo pairs and poses from lidar missions,
   and evaluate recovered depth offline against evaluation-only simulator depth
   by range, texture and view angle to fix the confident range model.
2. Add the stereo depth producer and the depth-to-beam adapter that emits the
   item 8 beam observations with per-ray confidence, and integrate them in
   shadow: lidar remains authoritative, and the vision occupancy is compared
   with lidar occupancy and truth occupancy for occupied precision and recall,
   unknown fraction and latency.
3. Make the guaranteed detection range directional and the gaze policy part of
   execution; validate with lidar still integrated that speed and heading
   behave as the observability model says.
4. Fly Urban Circuit Practice 01 on the stereo profile alone, with the lidar
   removed from the model, against item 12's gates.

### Measurement And Completion

Measure depth coverage and depth error against evaluation-only truth by range
and view angle, occupied precision and recall of the vision raw world against
truth occupancy, the fraction of the flown route that was observed before it
was entered, time spent speed-limited by observability, perception latency
from exposure to raw-world revision, planner p95, route availability, minimum
obstacle clearance and physical collisions.

This stage is complete when repeated Urban Circuit Practice 01 missions run
with the lidar absent from the vehicle model, no depth or point
cloud sensor in the control path, the raw-world and planner contracts
unchanged, and the same mission gates as the 3D-lidar profile: mission
complete, collision-free, route availability at the threshold item 9 stage A
derives, and planner p95 below 200 ms.
## Completed

Each entry keeps its original number. The release that shipped it is linked;
the detailed contracts live in the code, its tests, `CHANGELOG.md`, and the
documents named below.

### 1. Interceptor Drone (Completed)

Shipped before the first tag; see the `v0.1.0` entry in `CHANGELOG.md`. Three
interceptors pursue one attacking drone in isolated PX4 and ROS namespaces,
each from an independent radar-derived target track with predictive guidance
and no terminal goal hold. A separation of 5 m or less destroys the capturing
pair and records the intercept outcome.

### 2. Radar Measurement Simulation (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). The interceptor
sees only range, bearing, elevation, and radial velocity from an ideal radar
with a correlated random-walk cadence between 0.1 s and 3.0 s; swept raw-clear
visibility commands 20 Hz track mode. Truth adapters, referees, radar
simulators, trackers, and guidance are separate nodes, and contract tests keep
absolute target position out of the interceptor-facing `RadarScan`.

### 3. Target Motion Prediction (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Guidance solves
the constant-velocity intercept from the latest track, capped at 15 s and at
1 s inside the target corridor, with spatial hysteresis and a smoothed horizon.
The planner clips the prediction at the first raw occupied cell without
inflation or prohibited regions; swept visibility of the target switches to
direct moving-target MPPI pursuit.

### 4. Multiple Interceptors Versus One Attacker (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Three interceptors
from three city corners own independent PX4, navigation, radar, tracker, and
guidance pipelines; optional directional hypotheses converge to zero near the
attacker. The first interceptor within 5 m destroys the pair, survivors enter a
typed stationary hold, and interceptor-to-interceptor proximity is collateral.

### 5. Multiple Interceptors Versus Multiple Attackers (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). The generic N x M
launch pipeline runs the 2x2 scenario through the `sim_multi_intercept_*.sh`
wrappers. Each interceptor keeps one radar-derived track per detection, a typed
assignment coordinator minimizes estimated intercept time with hold, threshold,
and confirmation hysteresis, and the referee records one terminal outcome per
attacker. Attackers are not respawned.

### 6. Cooperative Multi-Drone Air Traffic (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Four civilian
drones exchange typed bounded-validity flight intents at 20 Hz and select
deterministic complementary vertical or lateral maneuvers from predicted
closest approach. Separation is a strong soft MPPI cost, never a prohibited
grid or inflated obstacle; static passages expose raw-validated lane capacity
with deterministic right-of-way. Static and no-static scenarios passed the
referee.

### 6.1. Non-Cooperative Collision Avoidance In Interception Missions (Completed)

Shipped before the first tag (`v0.1.0` in `CHANGELOG.md`). Every attacker
carries an anonymous airborne radar and a variable-time tracker; a finite
trajectory cost below 10 m with anticipation to 20 m and a raw-validated
maximin acquisition drive avoidance. Raw occupancy remains stronger than
separation and no exclusion volume exists, so physical interception stays
possible. The 3x1 and 2x2 scenarios passed with and without a static map.

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
Manhattan gate, the 97 and 99 percent availability targets (measured 88 to 95
percent after bootstrap), and the cooperative and interception re-flights. The
technical debt measured during closure is listed in item 10.
