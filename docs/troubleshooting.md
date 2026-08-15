# Troubleshooting

## Supported First Steps

```bash
./scripts/stop_sim.sh
./scripts/build.sh
./scripts/test.sh
```

Use only the container workflow. Every diagnostic simulation must use a timeout
of at least 120 seconds.

## Drone Takes Off And Holds

Check:

- `production_mppi_node` started and CUDA is available;
- raw obstacle and ESDF revisions advance;
- pose and ESDF age remain below configured limits;
- target source is not `no_executable_route_*`;
- execution reason is not `no_executable_horizon`;
- selected horizon is not collision-classified;
- offboard receives fresh increasing horizon sequences.

## Drone Drives Into An Obstacle

Inspect in this order:

1. raw obstacle grid and lidar projection;
2. ESDF revision and age;
3. lattice guide status and target source;
4. post-update collision classification;
5. route availability and unresolved-frontier terminal speed;
6. `no_executable_horizon` entry and exit transitions;
7. offboard deadline and applied command.

For air channels, inspect the 3D route samples and constrained route spans.

## Drone Stalls At A Wall

Compare:

- active global guide endpoint;
- lattice reached/frontier/dead-end classification;
- guide remaining length;
- head and terminal predicted progress;
- actual displacement;
- liveness reseed and guide-stall generations.

A short frontier aimed at a wall and repeatedly high predicted progress indicate
guide/recovery failure, not a speed-policy problem.

## Route Flaps Left And Right

Check whether the global guide generation changes. If it does, inspect sticky
guide release reasons. If the guide remains stable but the blue MPPI horizon
flaps, inspect warm-start shift, first-control delta, horizon stability, and
local cost hierarchy.

## Constrained Channel Traversal Stalls

Check:

- active route generation and constrained span stations;
- current route station and 3D reference sample;
- vertical/lateral tracking error and reference speed;
- fresh horizon reception;
- physical channel occupancy around the route.

The constraint must come from the accepted 3D route, never from proximity to an
opening.

## No-Static Flight Is Too Fast

Confirm:

- `use_static_map=false` reached both obstacle memory and MPPI;
- no-static horizon duration, cruise, cap, acceleration, and jerk values;
- PX4 `MPC_XY_VEL_MAX` and acceleration parameters were set for the run;
- target source is never distant `mission_goal_direct` without a guide.

## Lidar Is Rotated At Startup

Projection must remain gated until PX4 heading is valid and stable for the
configured number of consecutive samples. The gate deliberately does not
compare PX4 heading with `initial_heading_rad`; that value is not a lidar yaw
fallback when `use_px4_heading_for_scan=true`. A transition to stable PX4
heading starts a new pose-history generation. Inspect heading variance, stable
sample diagnostics, pose-history generation, scan timestamp, and mount
quaternion. Do not correct this with a fixed arbitrary yaw subtraction.

## RViz Is Empty

Check the actual topics:

- `/drone_city_nav/raw_obstacle_grid`;
- `/drone_city_nav/mppi/path`;
- `/drone_city_nav/mppi/markers`;
- `/drone_city_nav/drone_marker`.

Verify reliable/transient-local QoS for durable world markers.

## Gazebo Camera Does Not Follow

Verify:

- the spawned entity is `x500_lidar_2d_0`;
- `ENABLE_GZ_GUI_FOLLOW_CAMERA` is true;
- CameraTracking diagnostics show a non-empty entity id;
- stale Gazebo processes were stopped before launch.

RViz follow uses a separate `drone_follow` TF and is not evidence that Gazebo
CameraTracking is configured.

## Ctrl+C Or Cleanup Hangs

Run:

```bash
./scripts/stop_sim.sh --dry-run
./scripts/stop_sim.sh
```

Inspect stale container, Gazebo, PX4, Micro XRCE-DDS, RViz, and ROS process
lists printed by the script. Do not start another GUI world until cleanup
finishes.
