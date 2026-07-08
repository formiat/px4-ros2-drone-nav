# Gazebo Simulation

The simulator run combines Gazebo, PX4 SITL, ROS 2, the navigation nodes, and
optional RViz. The default world is a generated city-like environment, but the
navigation stack is not tied to a city scenario.

## World Files

World and map assets live under `drone_city_nav/worlds/`:

- `generated_city.sdf`
- `generated_city.map2d`

The `.sdf` world describes the simulator scene. The `.map2d` file is the static
planner obstacle source used by `planner_node`.

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
origin parameters such as `px4_local_origin_x_m` and `px4_local_origin_y_m`.

## Spawn, Start, And Goal

The default configuration uses:

- start: `(54.0, 54.0)`
- goal: `(216.0, 378.0)`
- initial altitude: `18.0 m`
- PX4 local origin: `(54.0, 54.0)`

These values are configured in `drone_city_nav/config/urban_mvp.yaml` for the
planner, offboard node, obstacle memory node, and mission monitor.

When changing the scenario, keep these values consistent across nodes.

## Static Map Generation

The planner static obstacle source is configured with:

```yaml
use_static_map: true
static_map_path: worlds/generated_city.map2d
static_map_min_blocking_height_m: 0.0
```

The static map is a raw obstacle source. It must not contain planner inflation.
Inflation and planning clearance are applied by the planner grid builder.

## Changing The Environment

When changing the world:

1. Update or add the Gazebo `.sdf` world.
2. Update or add the matching `.map2d` static map.
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
- `GZ_GUI_FOLLOW_TARGET=<model>`
- `GZ_GUI_FOLLOW_OFFSET="-12 0 6"`

## Known Gazebo-Specific Issues

- Stale Gazebo or PX4 processes can keep ports/resources busy. Use
  `./scripts/stop_sim.sh`.
- GUI display forwarding can fail on restrictive X11/Wayland setups.
- Gazebo visual coordinates and RViz map coordinates require the fixed
  `gazebo_aligned_map_tf` transform from the launch file.
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

## World And Static Map Consistency

The `.sdf` world and `.map2d` static map must describe the same obstacle
layout. Gazebo uses the world for rendering and collision. The planner uses the
static map as raw obstacle evidence. If they drift apart, RViz can show a
planner-valid route through a building, or the planner can avoid empty space.

When editing a world, verify:

- building positions and sizes in Gazebo;
- matching static map cells;
- map origin relative to PX4 local origin;
- grid bounds cover the full mission;
- spawn and goal are inside navigable area;
- RViz static map overlays the visible city.

The static map should remain raw. Do not pre-inflate buildings in the map to
"help" the planner. Inflation and planning clearance are planner parameters.

## Spawn And Goal Consistency

The spawn pose, PX4 local origin, and mission goal are coupled. A mismatch can
look like a planner bug because A* and RViz operate in map coordinates while
Gazebo renders world coordinates.

Useful checks:

- the drone spawns where RViz says it spawns;
- the goal marker is reachable and not inside hard prohibited cells;
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
3. Verify static map and prohibited grid in RViz.
4. Confirm lidar points align with visible obstacles.
5. Confirm start and goal are valid.
6. Run a headless smoke test.
7. Inspect logs for unexpected replans or frame warnings.

This checklist catches environment problems before they are misdiagnosed as
planner or controller regressions.
