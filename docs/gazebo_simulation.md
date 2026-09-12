# Gazebo Simulation

The simulator run combines Gazebo, PX4 SITL, ROS 2, the navigation nodes, and
optional RViz. The default world is a generated city-like environment, but the
navigation stack is not tied to a city scenario.

## World Files

World and map assets live under `drone_city_nav/worlds/`:

- `generated_city.sdf`
- `generated_city.occupancy3d`
- `generated_city.topology3d`
- `canonical_city.world3d.json`

The SDF, sparse Occupancy3D, and FreeSpaceTopology3D are generated from the
canonical JSON world spec. Static planning loads raw occupancy and its derived
topology separately; no-static planning uses lidar only.

## Drone Model

The launch scripts use a PX4-spawned `x500_lidar_2d_0` model with a 2D lidar.
Gazebo publishes the lidar scan on a Gazebo topic, and the launch file starts a
`ros_gz_bridge` parameter bridge that remaps it to `/scan`.

The lidar topic configured in `city_nav.launch.py` is:

```text
/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
```

ROS nodes consume the bridged `/scan` topic.

## PX4 SITL

PX4 publishes vehicle state topics such as:

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_status_v1`

The offboard node publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

PX4 local coordinates are aligned with the planner map through configured
origin parameters such as `px4_local_origin_x_m` and `px4_local_origin_y_m`
and the `px4_to_map_m00..m11` matrix. PX4 reports NED positions (X north, Y
east) while Gazebo worlds are ENU, so the launch derives the matrix from the
canonical world's `map_to_sdf`: the Manhattan world already swaps its map axes
and keeps the identity, whereas a world whose map frame equals the SDF frame
(Urban Circuit) receives the axis swap `(0, 1, 1, 0)`. Every node that turns
a PX4 pose into map coordinates, including the mission monitor, receives the
same matrix.

## Heading Source

The simulated magnetometer is not usable as a heading source in this SITL
(gz-sim 8.11, gz-sensors 8.2.2, PX4 v1.17). Measured against the simulator's
true attitude over several urban flights, the EKF heading sat five to six
degrees off at hover and wandered with a standard deviation of 2.2 degrees in
flight, with excursions past ten degrees. The error did not follow the world's
magnetic field (the world model was rotated by 41.6 degrees and the sensor's
azimuth moved by 3) and did not respond to `EKF2_MAG_DECL`, `EKF2_DECL_TYPE`
or `EKF2_MAG_TYPE`. A lidar map built with that heading copies every wall a
metre sideways at the integration range, and the copies open and close
passages the planner is trying to use.

The single-vehicle simulation therefore hands the autopilot the heading a
calibrated attitude reference would give: `simulation_heading_source_node`
takes the simulator's true attitude, adds a slowly wandering bias (0.5 degree
standard deviation, 60 s correlation time) and 0.3 degree of white noise, and
publishes it through PX4's external vision interface as orientation only.
`scripts/run_drone_nav_sim.sh` enables it by default
(`ENABLE_SIMULATION_HEADING_SOURCE=true`, PX4 parameters `EKF2_EV_CTRL 8`,
`EKF2_MAG_TYPE 5`, `EKF2_EV_NOISE_MD 1`, `EKF2_EVA_NOISE 0.01`);
`ENABLE_SIMULATION_HEADING_SOURCE=false` restores the magnetometer. With the
source enabled the measured heading error is 0.85 to 1.5 degrees standard
deviation.

**Release assumption.** The navigation stack is validated only for a heading
of that quality: a standard deviation of about 1.5 degrees or better, from a
vision, GNSS-heading or calibrated attitude reference. It is not validated for
a compass-grade heading of 2 degrees and more; at that level the persistent
map smears and the stack's route stability degrades (measured in flights
r165 to r180: loops of 170 to 250 metres where a 40 metre passage existed).
The assumption is carried by one parameter,
`lidar_pose_heading_uncertainty_rad` of `obstacle_memory_3d_node`: the
persistent memory integrates beams only as far as the heading error keeps a
hit within two voxels of its surface, `2 * resolution / tan(sigma)`. The
sensor braking contract needs a guaranteed detection range of 14 metres (the
1 degree lidar resolves a 0.25 m voxel with two beams inside 14.3 m), so at
0.25 m voxels the heading error may not exceed 2 degrees (0.036 rad) before
the two contracts conflict. Set the parameter to the actual heading
uncertainty of the vehicle's source; the configured 0.026 rad (1.5 degrees,
19 m of range) is the upper end of the simulation source's measured error.
The compass-era 0.038 rad (2.2 degrees) bounded the integration to 13 m.

## Spawn, Start, And Goal

The default configuration uses:

- start: `(54.0, 54.0)`
- goal: `(216.0, 378.0)`
- takeoff climb: `2.0 m` above the spawn point
- PX4 local origin: `(54.0, 54.0)`

These values are configured in `drone_city_nav/config/urban_mvp.yaml` for the
planner, offboard node, obstacle memory node, and mission monitor.

The X500 wrapper also loads `DroneContactSystem`. The simulation runner prepends
the package install library directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`, and the ROS
launch bridges `/drone_city_nav/drone_contacts` from `gz.msgs.Contacts` to
`ros_gz_interfaces/msg/Contacts`. Do not replace this with a distance or lidar
heuristic: Gazebo physics contact is the source of truth for a crash.

When changing the scenario, keep these values consistent across nodes.

The intercept mission does not duplicate these values. Its four map starts and
evader goal live in `drone_city_nav/config/intercept_scenario.json`; Gazebo
spawns are derived automatically from the canonical world's `map_to_sdf`
transform. At runtime, `simulation_truth_adapter_node` converts Gazebo dynamic
poses back to map coordinates and the referee blocks mission start until all
four navigation poses agree with physical truth.

## Static World Generation

The planner static obstacle source is configured with:

```yaml
use_static_map: true
static_occupancy_3d_path: worlds/generated_city.occupancy3d
static_free_space_topology_3d_path: worlds/generated_city.topology3d
```

Regenerate all static world artifacts with the exact container command documented in
`world3d.md`. Occupied voxels are physical geometry only. Clearance bands remain
ranking costs.

## Changing The Environment

When changing the world:

1. Update `canonical_city.world3d.json`.
2. Regenerate the SDF, Occupancy3D, ESDF3D, and FreeSpaceTopology3D artifacts.
3. Update grid bounds in `urban_mvp.yaml` if the navigable area changes.
4. Update start/goal/origin values consistently.
5. Check RViz overlays against Gazebo geometry.
6. Run a headless smoke test before relying on GUI behavior.

The relevant grid defaults are:

```yaml
planning_grid_resolution_m: 0.5
planning_grid_width_m: 345.0
planning_grid_height_m: 525.0
planning_grid_origin_x: -30.0
planning_grid_origin_y: -30.0
```

## GUI Camera

The GUI launch asks the Gazebo `CameraTracking` GUI plugin to follow
`x500_lidar_2d_0`. Useful environment variables:

- `ENABLE_GZ_GUI_FOLLOW_CAMERA=false`
- `ENABLE_RVIZ_FOLLOW_CAMERA=false`
- `GZ_GUI_FOLLOW_TARGET=<model>`
- `GZ_GUI_FOLLOW_OFFSET="-12 0 6"`
- `INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID=<scenario vehicle ID>`
- `INTERCEPT_SPECTATOR_RESELECTION_POLICY=first_living|next_living`

## Known Gazebo-Specific Issues

- Stale Gazebo or PX4 processes can keep ports/resources busy. Use
  `./scripts/stop_sim.sh`.
- GUI display forwarding can fail on restrictive X11/Wayland setups.
- RViz navigation debug uses the `gazebo_map` fixed frame plus the
  `gazebo_aligned_map_tf` transform from the launch file, derived from the
  canonical world's `map_to_sdf`. The generated city keeps the legacy rotation
  that swaps the horizontal axes and flips Z; a world whose map frame equals
  the SDF frame (Urban Circuit) gets the identity transform. Every overlay
  publisher receives the matching `gazebo_aligned_rviz_axes_swapped`
  parameter.
- Multiple simultaneous Gazebo instances are not supported by the standard
  workflow.

## Simulation Responsibilities

Gazebo is the physics and sensor environment. It should not contain navigation
logic. The world defines geometry, models, plugins, and sensor output. The ROS
nodes decide how to interpret that data and how to control PX4.

Keep these responsibilities separate:

- Gazebo world: buildings, ground, drone model, sensor placement;
- PX4 SITL: vehicle state, arming, offboard mode, low-level control;
- ROS navigation stack: mapping, planning, trajectory generation, setpoints;
- RViz: visualization and debug interpretation.

This separation matters when debugging. A bad trajectory color is rarely a
Gazebo problem. A shifted lidar point cloud can be a Gazebo frame, TF, origin,
or compensation problem. A drone that refuses offboard control may be PX4 state
or arming rather than planner geometry.

## World Artifact Consistency

`canonical_city.world3d.json` is the only hand-edited geometry source. The
generator derives both Gazebo SDF and sparse Occupancy3D from it, so rendering,
physics, and static planning cannot encode different buildings. The current
world adds two L-shaped passages, one straight-through passage, and one T
junction between neighboring Manhattan buildings. Every open bridge and
intersection receives a collisionless no-static lidar occluder.

When editing a world, verify:

- ordinary building positions and sizes in Gazebo;
- regenerated SDF and Occupancy3D match the canonical specification;
- all passage orientations and constrained envelopes are represented in
  Occupancy3D;
- map origin relative to PX4 local origin;
- grid bounds cover the full mission;
- spawn and goal are inside navigable area;
- RViz static Occupancy3D points overlay the visible city.

The runner configures the lidar visibility mask from the resolved
`ENABLE_STATIC_MAP` mode. Static lidar excludes passage masses and no-static
occluders because Occupancy3D is authoritative. No-static lidar includes both,
so every passage is observed as an ordinary obstacle rather than traversed.

The static map should remain raw. Do not pre-inflate buildings in the map to
"help" the planner. MPPI derives categorical risk bands from the occupied
distance field.

## Spawn And Goal Consistency

The spawn pose, PX4 local origin, and mission goal are coupled. A mismatch can
look like a planner bug because navigation and RViz operate in map coordinates
while Gazebo renders world coordinates.

Useful checks:

- the drone spawns where RViz says it spawns;
- the goal marker is reachable and not inside a raw occupied cell;
- the first planned segment starts near the drone, not offset by a map origin
  error;
- lidar points align with static map buildings while the drone is stationary.

If these checks fail, fix coordinate configuration before tuning planning or
control.

## Headless Versus GUI Runs

GUI runs are better for visual inspection. Headless runs are better for repeat
checks, script automation, and quality validation. Both should use the same
navigation configuration.

Use GUI when investigating:

- visual trajectory shape;
- RViz color interpretation;
- Gazebo model placement;
- obvious vehicle motion anomalies.

Use headless when investigating:

- regression checks;
- mission completion;
- planner timing;
- blackbox metrics;
- repeated comparison after a parameter change.

Do not rely only on visual judgment. A trajectory can look slightly worse but
track better, or look smooth while producing excessive setpoint lag.

## Simulation Change Checklist

After changing simulation assets or launch setup:

1. Stop old Gazebo and PX4 processes.
2. Start a clean GUI run and confirm model placement.
3. Verify static map and raw obstacle grid in RViz.
4. Confirm lidar points align with visible obstacles.
5. Confirm start and goal are valid.
6. Run a headless smoke test.
7. Inspect logs for unexpected replans or frame warnings.

This checklist catches environment problems before they are misdiagnosed as
planner or controller regressions.
