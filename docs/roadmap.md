# Gazebo Roadmap

## 1. Interceptor Drone (In Progress)

Implement an autonomous interceptor drone capable of pursuing an attacking
drone using external target information.

The current foundation launches three interceptors and one attacking drone in
isolated PX4 and ROS namespaces. The attacker flies from a fixed start to a
fixed goal. Every interceptor receives an independent radar-derived target
track, predicts its motion, and continuously updates a tracking objective
without entering terminal goal hold. A separation of 5 m or less destroys the
capturing pair and records a successful intercept outcome.

The remaining work is to validate and tune the radar tracking and predictive
guidance pipeline over repeated mission runs.

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

Each pursuit data path is split into a mission referee, radar simulator, target
tracker, and interceptor guidance node. Only the referee and radar simulators may
subscribe to attacker ground truth. The interceptor-facing `RadarScan` carries
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
interceptors brake and enter confirmed stationary position hold.
Interceptor-to-interceptor separation within 5 m is accepted as collateral
damage; only that pair is destroyed and the pursuit continues. The spectator
camera switches from a destroyed tracked interceptor to the next living one.

This stage deliberately contains one attacker and one episode. Attacker
despawn, respawn, and an endless campaign remain future work and are not part of
the current implementation.

## 5. Multiple Interceptors Versus Multiple Attackers

Start with three interceptor drones and three attacking drones, all spawned
simultaneously.

Implement dynamic target assignment, initially using a simple policy such as
nearest-target allocation. Collision avoidance remains disabled, and mid-air
collisions are treated as acceptable collateral damage for this experimental
scenario.

## 6. Cooperative Multi-Drone Air Traffic

Run multiple autonomous civilian drones simultaneously in the same urban
environment. Each drone has an independent point-to-point mission.

All drones initially fly at the same altitude. Fixed cruise-altitude layers
must not be preassigned. Instead, the drones should dynamically negotiate
collision-free trajectories during the mission, including temporary altitude
changes when necessary.

An open design question is whether the drones should exchange position,
velocity, and heading telemetry through a shared communication channel to
improve cooperative collision avoidance.

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
