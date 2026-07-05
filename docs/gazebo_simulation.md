# Gazebo Simulation

The simulator run combines Gazebo, PX4 SITL, ROS 2, the navigation nodes, and
optional RViz. The default world is a generated city-like environment, but the
navigation stack is not tied to a city scenario.

## World Files

World and map assets live under `drone_city_nav/worlds/`:

- `generated_city.sdf`
- `generated_city_mixed_heights.sdf`
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
- cruise altitude: `18.0 m`
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
