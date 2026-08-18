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

## 1. Interceptor Drone (Completed)

Implement an autonomous interceptor drone capable of pursuing an attacking
drone using external target information.

The current foundation launches three interceptors and one attacking drone in
isolated PX4 and ROS namespaces. The attacker flies from a fixed start to a
fixed goal. Every interceptor receives an independent radar-derived target
track, predicts its motion, and continuously updates a tracking objective
without entering terminal goal hold. A separation of 5 m or less destroys the
capturing pair and records a successful intercept outcome.

The interceptor mission, radar-derived tracking, predictive guidance, and
physical interception lifecycle have been implemented and validated in
repeated mission runs.

## 2. Radar Measurement Simulation (Completed)

Replace direct access to the target's ground-truth coordinates with a more
realistic radar measurement model.

The radar interface will provide only:

- range;
- bearing;
- elevation;
- radial velocity.

The initial implementation uses ideal measurements without noise,
interference, latency, or measurement errors. Measurement cadence follows a
deterministic correlated random walk between 0.1 s and 3.0 s. Guidance continues
at 20 Hz by coasting the latest target track between scans. Swept raw-clear
visibility of the current target estimate commands an immediate scan and 20 Hz
track mode at any range; target occlusion restores variable search cadence.

The physical radar will not be simulated. Only the radar measurement interface
and its integration with the interceptor are implemented.

Each pursuit data path is split into a simulation-truth adapter, mission
referee, radar simulator, target tracker, and interceptor guidance node. Gazebo
model poses are converted to typed physical truth once; only the referee and
radar simulators may subscribe to the target's typed truth. The
interceptor-facing `RadarScan` carries
range, azimuth, elevation, and relative radial velocity, but no absolute target
position, velocity, or simulator entity identity. The first detection supports
direct pursuit; subsequent variable-dt updates estimate Cartesian velocity for
predictive guidance. Runtime graph validation and source-contract tests enforce
this boundary.

## 3. Target Motion Prediction (Completed)

Implement target trajectory prediction. The initial model will intentionally
remain simple:

- use the latest known target position;
- use the latest known velocity vector;
- extrapolate the trajectory several seconds ahead;
- intercept the predicted future position instead of chasing the current
  position.

The implementation solves the constant-velocity intercept equation using the
configured interceptor speed. The solution is capped at 15 s. When the
interceptor is ahead and inside the target corridor, the horizon is capped at
1 s. The transition uses spatial hysteresis and a smoothed prediction horizon.
Slow or invalid target velocity falls back to the observed target position.

Prediction starts at the measurement timestamp, so telemetry age is included
in extrapolation. Interceptor guidance publishes a typed tracking objective from
the radar-derived target track without reading a map. The planner clips the
prediction segment at the first raw occupied cell and uses the last raw-free
sample. It does not search for a nearest free point and does not add inflation
or prohibited regions. Vertical prediction applies bounded deceleration until
the target stops climbing or descending and clamps altitude to the configured
flight envelope instead of rejecting the objective. Swept visibility of the
current target activates direct moving-target MPPI pursuit. If only the full
prediction is blocked, the planner shortens the lead toward the current target;
only current-target occlusion returns execution to ordinary global planning.
RViz and JSONL diagnostics expose observed, coasted current, predicted, and
resolved target points together with visibility, prediction-path clearance,
closing speed, commanded speed, active speed limiter, and radar age.

The initial target-motion prediction scope is complete. It now consumes the
target track produced by the radar pipeline described in section 2.

## 4. Multiple Interceptors Versus One Attacker (Completed)

The finite scenario now runs three interceptor drones against one attacking
drone. They start in three different city corners and own independent PX4,
navigation, radar, tracker, and guidance pipelines. All three use the measured
motion direction by default. Optional long-range directional hypotheses add
`-45` and `+45` degree alternatives; the offsets converge to zero near the
attacker and cannot move the predicted point more than 70 m laterally.

The first interceptor within 5 m of the attacker destroys that pair. Surviving
interceptors receive a typed hold objective and enter confirmed stationary
position hold.
Interceptor-to-interceptor separation within 5 m is accepted as collateral
damage; only that pair is destroyed and the pursuit continues. The spectator
camera starts on `interceptor_0` and uses the configurable living-vehicle
reselection lifecycle after a typed death event.

This stage deliberately contains one attacker and one episode. Attacker
despawn, respawn, and an endless campaign remain future work and are not part of
the current implementation.

## 5. Multiple Interceptors Versus Multiple Attackers (Completed)

The first finite scenario launches two attacking drones from the short city
side farthest from their shared corner destination. One attacker starts at the
corner and the other starts one block inward along that side. Two interceptors
start on the opposite short side, next to the destination.
The existing 3x1 `intercept` entry point remains unchanged; dedicated
`sim_multi_intercept_gui.sh` and `sim_multi_intercept_headless.sh` wrappers load
the 2x2 scenario through the same generic N x M launch pipeline.

Each interceptor receives an independent ideal radar scan containing relative
measurements for every attacker and maintains one radar-derived track per
detection. A typed assignment coordinator minimizes estimated intercept time,
covers distinct active targets where possible, and applies hold time,
improvement threshold, and confirmation hysteresis before changing an existing
assignment. Terminal attackers are removed immediately and the remaining fleet
is reassigned without restarting navigation.

The multi-target referee records one first terminal outcome for every attacker
and preserves physical 5 m proximity, typed death, disarm, and survivor-hold
settlement. Collision avoidance remains disabled, and mid-air collisions are
treated as acceptable collateral damage. The finite mission does not respawn
attackers and does not implement an endless campaign.

The `2x2` spectator starts on `evader_0` and uses cyclic `next_living`
reselection, preferring `evader_1` after the first attacker's destruction.

## 6. Cooperative Multi-Drone Air Traffic (Completed)

The finite cooperative scenario launches four autonomous civilian drones from
the city corners. Each vehicle has an independent point-to-point mission to the
opposite corner, starts at the same altitude, and owns an isolated PX4,
navigation, mapping, and MPPI pipeline. Fixed cruise-altitude layers are not
preassigned.

The drones exchange typed, bounded-validity flight intents at 20 Hz. Each intent
contains the vehicle's physical footprint, current position and velocity,
predicted MPPI trajectory, maneuver state, and constrained-passage use. Every
vehicle independently validates peer freshness, predicts continuous closest
approach over the shared horizon, and selects deterministic complementary
vertical or lateral maneuvers. Latching and release hysteresis prevent rapid
maneuver flapping.

Peer separation is implemented as a strong soft MPPI cost and preferred
acceleration direction. It does not create prohibited grids, inflated obstacles,
or hard exclusion volumes. Static passages expose capacity derived from
raw-validated lane geometry: opposite traffic may use separate lanes when the
physical width permits it, while exclusive or conflicting use is resolved by
deterministic right-of-way and a route-safe hold before entry.

In no-static mode, cooperative peer returns are filtered from persistent lidar
memory and from direct raw obstacle-path validation. A dedicated
referee verifies coordinate readiness, physical minimum separation, goal
arrival, stationary hold, vehicle destruction, and building collisions. The
supported headless contract requires every vehicle to settle at its own goal
without physical loss. Both static-map and no-static-map scenarios have passed
this full mission validation.

## 6.1. Non-Cooperative Collision Avoidance in Interception Missions (Completed)

Every attacker carries an independent high-rate simulated airborne radar. It
reports anonymous relative detections of all aircraft within physical range and
line of sight; it does not expose roles, mission assignments, global routes, or
the intent communication channel used by cooperative civilian traffic. A local variable-time
tracker converts those measurements into anonymous position and velocity tracks.

The attacker evaluates current separation and continuous closest approach for
every fresh track. A strong finite trajectory cost applies below 10 m, with a
lower anticipation cost between 10 m and 20 m and additional time-to-collision
weighting. The cost covers the full MPPI rollout. On entry into a strong threat,
a raw-validated maximin acquisition selects among route-directed, lateral,
vertical, speed-reduction, and reverse candidates; normal route progress breaks
ties.
Lifecycle hysteresis and one-time entry and release reseeds prevent maneuver
flapping.

Physical obstacle validity remains stronger than aircraft separation. Avoidance
cannot select a trajectory that intersects raw occupancy or violates the flight
envelope. Separation is still a soft objective: the implementation does
not create prohibited grids, inflated obstacles, hard exclusion volumes, or an
equivalent mandatory boundary around another drone. Physical interception
therefore remains possible.

Headless validation verifies the radar-only data boundary, fresh independent
tracks for every attacker, observable avoidance activity or cost, physical
mission settlement, and zero building collisions. The `3x1` and `2x2` scenarios
have passed this contract with and without a static map.

## 7. Advanced 3D Passages (Completed)

Static passage planning now uses a separately versioned, map-fingerprint-bound
`FreeSpaceTopology3D`. A chunked C++ compiler consumes raw `Occupancy3D` and the
matching `ESDF3D`, classifies footprint-feasible clearance topology, extracts
arbitrarily oriented portal voxel patches, and skeletonizes constrained free
space into sparse medial passage segments. Roof presence, axis-aligned portal
heuristics, rectangular authoritative openings, and pairwise portal edges are no
longer part of the contract.

Global planning resolves route-specific `PassageTraversal` objects lazily over
the sparse graph and caches them. The lattice uses a spatial index rather than
scanning every passage edge at every state. After route selection, raw occupancy
queries generate a varying 3D cross-section envelope along the traversal, so
sloped, vertical, curved, and changing-height routes do not collapse to one
`min_z/max_z` intersection. MPPI and route activation still perform final raw
swept-footprint validation; derived topology never creates a hard obstacle.

Region, portal, segment, traversal, and cooperative conflict-resource IDs are
distinct strong types. Cooperative passage reservations are scoped to shared
sparse segments instead of locking an entire free-space region. Deterministic
fixtures cover a sloped tunnel, vertical shaft, arch, curved tunnel, T and X
junctions, and a wide-hangar negative case.

The compiler has also produced strict artifacts for the compact fixture and the
Urban, Cave, and Finals release maps. This closes static extraction, planning,
execution, and cooperative passage use. It does not claim production mission
integration in those external environments; that remains item 9. Online
production of the same typed topology from 3D sensing remains item 8.

## 8. 3D Passage Support Without A Static Map

**Type:** ordered implementation stage.

**Hard prerequisite:** item 7.

Integrate the 3D passage system into navigation without a preloaded static map.
The planned components are:

- 3D lidar;
- passage detection;
- autonomous passage traversal;
- dynamic trajectory generation through detected passages.

## 9. Large-Scale Realistic City And Full-Mission Validation

**Type:** integration and validation milestone.

**Hard prerequisites:** items 8 and 12 for no-static autonomous traversal;
item 11 for full static-map validation.

Find a suitably licensed high-quality city environment or build a new one for
the project. The location should be substantially larger and more visually and
geometrically varied than the current regular test city, with realistic street
layouts, building shapes, heights, materials, and urban topology.

Where practical, include complex physically traversable 3D passages and
passages such as multi-turn routes, junctions, and entry and exit points at
different altitudes. Imported visual assets must have explicit provenance and a
license compatible with the repository. Rendering meshes, collision geometry,
lidar-visible surfaces, static occupancy, and generated planning artifacts must
remain aligned instead of becoming separate hand-maintained versions of the
world.

Use the new location as a full-system validation environment rather than only a
visual showcase. Re-run every supported point-to-point, static-map, no-static,
3D-passage, single-target interception, multi-target interception, and
cooperative-traffic mission that exists when this stage begins. Validation
should cover multiple start and goal placements and repeated headless runs, and
must preserve physical outcome checks, zero tolerance for building collisions,
planner and controller diagnostics, real-time-factor monitoring, and measured
CPU/GPU timing.

This stage is complete only when the mission suite succeeds on the new city
without scenario-specific route scripts or geometry exceptions. One successful
3D-lidar exploration flight is integration evidence, not completion: acceptance
requires repeated representative point-to-point and cooperative runs, plus the
other supported mission types claimed by this milestone. Static-map acceptance
is performed after item 11 provides validated maps.

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

## 12. Incremental Topological Exploration

**Type:** dependent implementation stage.

**Hard prerequisite:** item 8.

Extend no-static navigation with an incremental topological exploration backend
for partially observed 3D environments such as tunnel networks, caves, and
labyrinths. This is not a second flight-control stack: 3D lidar, online
Occupancy3D and ESDF, raw swept-footprint validation, finite executable paths,
MPPI, and PX4 offboard control remain shared with ordinary navigation.

The new backend incrementally extracts a metric-topological graph of corridors,
junctions, vertical connectors, frontiers, and confirmed dead ends. It keeps
two complementary forms of exploration memory:

- a volumetric coverage field used only as a soft preference for selecting the
  next frontier;
- directed graph-edge state (`unknown`, `frontier`, `explored`, `dead_end`, or
  `temporarily_blocked`) with the map revision that justified that state.

Visited space and explored branches must never be converted into occupancy,
prohibited grids, inflated obstacles, or any other hard exclusion volume. A
drone must be able to backtrack through a known corridor, and a later map
revision may invalidate an earlier dead-end conclusion.

When the destination is known but the environment is only partly observed, the
planner performs goal-biased frontier exploration rather than exhaustive
coverage: it trades estimated route progress, travel cost, clearance,
information gain, and repeated traversal cost. Once a connected graph route to
the destination is known, ordinary graph search takes precedence over further
exploration. If the mission explicitly has no known destination, the same
backend may select frontiers for coverage instead.

In a confirmed degree-two corridor with a sufficiently long raw-safe swept
path, the backend retains the current branch and emits a route-directed cruise
objective. It should slow or reconsider only at a junction, tight or curved
geometry, a changed map, a dynamic obstruction, or the end of the executable
path. It must not continue along an invalid path or create a sticky braking or
recovery lifecycle.

`GLOBAL_GUIDANCE_BACKEND` selects the guidance implementation through launch
configuration. The initial supported values are `current_lattice` and
`incremental_topological`; code must not infer the backend from an environment
name or an `underground` flag. Both implementations publish the same typed
route contract to the common local-planning and control layers.

Implementation begins after item 8 establishes 3D lidar and online
Occupancy3D. Validate first with deterministic 3D fixtures covering a T
junction, X junction, loop, cul-de-sac, and vertical shaft. Measure goal time,
coverage, repeated-edge distance, time without an executable route, minimum
clearance, and physical collisions. Then evaluate the backend on the Finals and
Cave environments with repeated cooperative mission runs.

## 13. GNSS- And Magnetometer-Denied Lidar-Inertial Navigation

**Type:** dependent localization stage.

**Hard prerequisite:** item 8.

**Validation prerequisite:** item 12 for complete autonomous validation in
labyrinths, caves, and tunnel networks.

Add an optional navigation profile in which the aircraft does not use GNSS or
magnetometer fusion. This stage begins after item 8 provides production 3D
lidar and its timestamped full-6DoF acquisition-pose contract. The aircraft
retains its IMU and barometric altitude source and estimates motion from
lidar-inertial odometry instead of receiving global position and heading from
simulated navigation satellites and a simulated compass.

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
