# Gazebo Roadmap

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

## 7. Advanced 3D Passages

Extend the current 3D passage system with more complex passage geometries and
varying altitude profiles.

Instead of supporting only constant-height passages, allow drones to enter and
exit passages at different altitudes while maintaining smooth three-dimensional
trajectories.

## 8. 3D Passage Support Without A Static Map

Integrate the 3D passage system into navigation without a preloaded static map.
The planned components are:

- 3D lidar;
- passage detection;
- autonomous passage traversal;
- dynamic trajectory generation through detected passages.

## 9. Large-Scale Realistic City And Full-Mission Validation

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
without scenario-specific route scripts or geometry exceptions.
