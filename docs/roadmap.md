# Gazebo Roadmap

## 1. Interceptor Drone (In Progress)

Implement an autonomous interceptor drone capable of pursuing an attacking
drone using external target information.

The current foundation launches one interceptor and one attacking drone in
isolated PX4 and ROS namespaces. The attacker flies from a fixed start to a
fixed goal. The interceptor receives the attacker's current ground-truth state,
predicts its motion, and continuously updates a tracking objective without
entering terminal goal hold. A separation of 5 m or less destroys both drones
and records a successful intercept outcome.

The remaining work is to replace direct ground-truth state access with the
measurement and target-tracking stages described below and to tune predictive
guidance from repeated mission runs.

## 2. Radar Measurement Simulation

Replace direct access to the target's ground-truth coordinates with a more
realistic radar measurement model.

The radar interface will provide only:

- range;
- bearing;
- elevation;
- radial velocity.

The initial implementation will use ideal measurements without noise,
interference, latency, or measurement errors.

The physical radar will not be simulated. Only the radar measurement interface
and its integration with the interceptor will be implemented.

## 3. Target Motion Prediction (In Progress)

Implement target trajectory prediction. The initial model will intentionally
remain simple:

- use the latest known target position;
- use the latest known velocity vector;
- extrapolate the trajectory several seconds ahead;
- intercept the predicted future position instead of chasing the current
  position.

The first implementation uses a 3 s lead while the interceptor is behind or
outside the target's flight corridor. When the interceptor is ahead and inside
the corridor, the lead is reduced to 1 s. The transition uses spatial
hysteresis and a smoothed prediction horizon. Slow or invalid target velocity
falls back to the observed target position.

Prediction starts at the measurement timestamp, so telemetry age is included
in extrapolation. The mission coordinator publishes a typed tracking objective
without reading a map. The planner clips the prediction segment at the first
raw occupied cell and uses the last raw-free sample. It does not search for a
nearest free point and does not add inflation or prohibited regions. RViz and
JSONL diagnostics expose observed, predicted, and resolved target points.

The remaining work is to validate and tune the model in repeated static and
no-static intercept runs, then replace ground-truth target state with the radar
measurement and tracking pipeline from item 2.

## 4. Multiple Interceptors Versus One Attacker

Run a scenario with three interceptor drones cooperating against one attacking
drone.

After the attacker is neutralized, all surviving interceptors land and wait for
the next attacker. The scenario may later become an endless mission that spawns
a new attacker periodically.

Collision avoidance between drones will not be implemented for this
experimental stage. Drone-to-drone collisions are acceptable collateral
damage. As a simplified physical model, two drones coming within 5 m of each
other may both be considered destroyed.

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
