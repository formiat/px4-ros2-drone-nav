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
city for stage B; item 11 for the static-map part of stage B.

### Stage A: Full Mission Suite On The Current Locations

Fly every supported mission on the locations that exist today, Manhattan and
Urban Circuit Practice 01, at one release commit, headless, with nothing
changed between runs: point-to-point with a static map and without one,
constrained 3D traversal, single-target and multi-target interception, and
cooperative traffic. The cooperative and interception scenarios have not been
flown since the September 2026 navigation changes, so this stage also decides
whether they still pass their referees. Stage A needs no new environment and
may begin immediately.

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

Stage A also repeats the unchanged three-run Manhattan no-static 3D-lidar gate
inherited from item 12: three sequential headless point-to-point missions, each
reaching the goal within the 120-second hard limit, crossing the required
low-altitude route volume, collision-free, with no route-ownership gap.

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
visual showcase. Re-run the complete stage A suite on it, covering multiple
start and goal placements and repeated headless runs, and preserve physical
outcome checks, zero tolerance for building collisions, planner and controller
diagnostics, real-time-factor monitoring, and measured CPU/GPU timing.

Stage B is complete only when the mission suite succeeds on the new city
without scenario-specific route scripts or geometry exceptions, at the stage A
thresholds. One successful 3D-lidar exploration flight is integration evidence,
not completion. Static-map acceptance is performed after item 11 provides
validated maps.

## 10. Architectural Review And Optimization

**Type:** independent recurring workstream.

**Dependencies:** none; this item is not part of the ordered execution sequence.

Perform systematic architecture reviews throughout development and repeat a
full review after the navigation, passage, and large-environment mission
contracts are established. Each review must trace the end-to-end data and
execution paths across sensing, mapping, topology, planning, MPPI, PX4 control,
cooperative coordination, simulation, and diagnostics.

Use repeatable representative missions to measure CPU, GPU, memory, ROS/DDS
transport, simulator real-time factor, planning latency, control deadline
misses, and scaling with vehicle count. Optimize confirmed bottlenecks while
preserving typed contracts, raw-occupancy safety validation, and observable
mission outcomes. Prefer removing duplicated work, stale data transport, and
unnecessary process or synchronization overhead over increasing worker counts
or weakening safety margins.

This stage also records architectural debt, defines ownership and lifetime
boundaries for shared resources, and converts validated optimizations into
regression benchmarks. It is complete when the supported mission suite has
measured performance budgets, reproducible baselines, and documented scaling
limits for both static-map and 3D-sensing configurations.

### Measured Debt (September 2026)

First recorded at the closure of item 12 from the r303 to r307 series on
commit `46823cac`, revised on 2026-09-16 from the r340 to r344 series on
`8dec84ea`. Each entry is a measurement, not a decision.

Closed since the first record:

- The production tick measures p50 23.5 ms and p95 37.8 ms against its 20 ms
  deadline, from p50 55 ms and p95 78 ms. A path validation built its
  collision oracle once per segment and the oracle's constructor scans every
  lidar return for finiteness, 60 us for the 63 700 points of an urban scan
  against 0.2 to 0.9 us for the segment itself; it is now built once per path.
  The mission check bounds the tick at 30 ms p50 and 45 ms p95 and reports the
  share of ticks over the deadline.
- Capture-to-publication latency of the control command is p50 20 ms and p95
  28 ms, from p50 44 ms and p95 60 ms, with the tick that carries it.
- The position estimate the tick reads matches the true Gazebo pose along the
  track within 0.01 s, from 0.10 to 0.12 s. EKF2 subtracted its default 110 ms
  GNSS delay from a simulated sample the Gazebo bridge stamps at receipt; the
  run script sets `EKF2_GPS_DELAY 0` and the lidar position source is read at
  the scan stamp again.
- `guaranteed_vertical_stopping_deceleration_mps2` is 1.4, the fifth
  percentile of the arrest plateau over 91 descents in 25 flights, from an
  optimistic 2.0. The descent-arrest check now holds the same statistic the
  law rests on.

Open:

- The loop still runs near 43 Hz rather than 50: 71 percent of r344's ticks
  overran the 20 ms deadline. What remains is spread thin, the CUDA
  controller at 11.4 ms and the CPU-side sweeps around it at about 12 ms
  together, with no single confirmed bottleneck left.
- The 3D obstacle memory still transports at 2 Hz; the observation age seen
  by the tick is unchanged at p50 416 ms and p95 644 ms against the 600 ms
  evidence-age term of the sensor-braking inequality. Raising the rate buys
  speed through that inequality and costs CPU in the memory node; neither
  side is measured.
- The persistent planner search still measures p50 150.0 ms, exactly its
  configured budget, with p99 185.8 ms. The p95 mission check therefore
  measures the configuration, and budget overruns are not gated.
- A holding vehicle drifts 0.102 m at the median, 0.370 m at p95 and 0.394 m
  at most over the 56 hold episodes of r303 to r334, measured against the
  true pose, while the rest-clearance rule asks for the 0.27 m the envelope
  carries over the hull. The margin a stop must keep at its rest pose is
  therefore smaller than the drift it exists to cover. Two flights ended in a
  contact when that margin was made releasable (r326 at 2.56 m/s, r334 at
  0.06 m/s while holding), so it is not a reserve to spend.
- Structure: the 2D obstacle memory node is still selectable by the launch
  files although no 2D production navigation path remains; fourteen sources
  sit within ten percent of the 1000-line cap after being split by size
  rather than by responsibility; 226 sources lie flat in `src/` beside the
  layered subdirectories.

Not started, and required by this item's own completion criteria: measured
CPU, GPU, memory, ROS/DDS transport and real-time-factor budgets; reproducible
baselines for the whole supported mission suite rather than the urban
point-to-point mission alone; scaling with vehicle count; and the same for the
static-map configuration. The tick budget above is the first of those budgets
and the only one that exists.

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

Item 13 is accepted only in the complex environments introduced after the
original Manhattan world. Manhattan is not a localization acceptance
environment because its repetitive geometry creates severe position and
heading ambiguity. Autonomous test flights in the selected labyrinths, caves,
and tunnel networks require item 12's persistent full-3D route and repair
backend. This roadmap dependency must not create a code dependency between the
localization estimator and the route planner.

Add an optional navigation profile in which the aircraft does not use GNSS or
magnetometer fusion. This stage begins after item 8 provides production 3D
lidar and its timestamped full-6DoF acquisition-pose contract and item 12
provides autonomous routing through partially observed complex environments.
The aircraft retains its IMU and barometric altitude source and estimates
motion from lidar-inertial odometry instead of receiving global position and
heading from simulated navigation satellites and a simulated compass.

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

Support two explicit localization configurations:

- with a valid static 3D map, lidar-inertial odometry provides continuous local
  motion while scan-to-map registration corrects accumulated drift and anchors
  the vehicle in the mission map frame;
- without a static map, lidar-inertial SLAM builds revisioned localization
  submaps and uses loop closure to maintain a locally consistent frame.

No-static missions with absolute map-frame goals require a declared initial
map pose or another explicit global reference. Unknown-pose localization in a
known static map is a separate global relocalization capability and must not be
implicitly replaced by a scenario-provided hidden ground-truth transform.

Localization quality and geometric observability must be first-class runtime
signals. Repetitive city blocks, long feature-poor tunnels, and symmetric caves
can leave translation or yaw weakly constrained even with 3D lidar. When the
estimate is stale, divergent, or insufficiently observable, the system must
stop publishing new executable motion and let the current finite path reach
its validated terminal state; it must not continue an invalid path or add a
sticky braking or geometric exclusion lifecycle.

Implement and validate this stage incrementally:

1. replay timestamped 3D lidar and IMU data offline and compare estimated poses
   with evaluation-only Gazebo truth;
2. fly one vehicle from a known initial pose using PX4 external odometry with
   GNSS and magnetometer fusion disabled;
3. add static-map correction, relocalization, and explicit estimator health;
4. add no-static submaps and loop closure;
5. validate multiple cooperative vehicles, each with an independent estimator
   and no shared localization state.

Measure position and attitude drift, velocity error, map alignment, loop
closure consistency, estimator latency, relocalization time, time without a
valid executable path, minimum obstacle clearance, and physical collisions.
This stage is complete when repeated static-map and no-static 3D-lidar missions
run without GNSS, magnetometer data, or control-visible simulator ground truth,
and localization failures produce an explicit safe finite-path outcome instead
of silent frame corruption.

## 14. Vision-Only 3D Perception Without Lidar Or Static Maps

**Type:** dependent perception stage.

**Hard prerequisites:** items 8 and 12.

**Validation prerequisite:** item 9's complex environments.

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
4. Fly no-static Manhattan on the stereo profile alone, with the lidar removed
   from the model, against item 8's mission gates.
5. Fly the complex environments — Urban, tunnels, caves — on the stereo profile
   alone against item 12's gates, then cooperative missions with every vehicle
   on its own cameras.

### Measurement And Completion

Measure depth coverage and depth error against evaluation-only truth by range
and view angle, occupied precision and recall of the vision raw world against
truth occupancy, the fraction of the flown route that was observed before it
was entered, time spent speed-limited by observability, perception latency
from exposure to raw-world revision, planner p95, route availability, minimum
obstacle clearance and physical collisions.

This stage is complete when repeated no-static Manhattan and complex-environment
missions run with the lidar absent from the vehicle model, no depth or point
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
