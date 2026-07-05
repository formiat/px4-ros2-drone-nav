# Troubleshooting

This page lists common failure modes and first checks.

## Simulation Does Not Start

Check:

- Docker permissions: `docker ps`;
- stale simulator processes: `./scripts/stop_sim.sh --dry-run`;
- full cleanup: `./scripts/stop_sim.sh`;
- container image availability;
- host display access for GUI runs.

## Drone Takes Off And Hovers

Check:

- whether `/drone_city_nav/path` is published;
- planner logs for A* or trajectory failure;
- offboard logs for path acceptance/rejection;
- PX4 offboard mode and arming state;
- mission start/goal/origin consistency.

## RViz Does Not Show The Path

Check:

- `enable_rviz:=true`;
- RViz fixed frame is `map`;
- `/drone_city_nav/final_trajectory_path`;
- `/drone_city_nav/path`;
- whether the planner published an empty hold path;
- `log/final_trajectory_samples/latest.csv`.

## Path Flickers Or Disappears

Likely causes:

- invalid new trajectory;
- failed replan;
- diagnostics/path stamp mismatch;
- empty hold path after planning failure;
- stale pose or path update rejection.

Inspect planner logs and offboard trajectory update logs.

## Replan Loop

Check:

- prohibited-grid intersection logs;
- raw obstacle sources;
- current lidar overlay quality;
- obstacle memory scoring;
- inflation and planning clearance settings;
- whether the drone is actually crossing the prohibited grid or only the
  planning-clearance margin.

## Gazebo Or PX4 Processes Remain After Closing GUI

Run:

```bash
./scripts/stop_sim.sh --dry-run
./scripts/stop_sim.sh
```

The standard workflow does not support multiple simultaneous Gazebo instances.

## Docker / X11 Problems

Check:

- Docker group membership;
- `DISPLAY`;
- Xauthority or Wayland/XWayland setup;
- whether host security policy blocks GUI clients from containers.

For headless validation, use `./scripts/sim_headless.sh`.

## Lidar Or Memory Is Shifted

Check:

- `px4_local_origin_x_m`, `px4_local_origin_y_m`;
- `use_px4_heading_for_scan`;
- `scan_yaw_offset_rad`;
- lidar mount roll/pitch/yaw;
- `lidar_pose_latency_s`;
- attitude compensation settings.

Use lidar debug snapshots and the static map analyzer script.

## A* Does Not Find A Path

Check:

- prohibited grid coverage;
- start and goal positions;
- grid bounds;
- static map path;
- inflation and planning clearance;
- whether current lidar or memory creates a blocking wall.

## Trajectory Diagnostics Do Not Match

Diagnostics should match accepted trajectories by `path_stamp_ns`. If they do
not:

- check `/drone_city_nav/path` header stamp;
- check `/drone_city_nav/trajectory_diagnostics`;
- check offboard logs for accepted planner id confirmation;
- check whether a candidate trajectory was rejected after diagnostics arrived.

## Drone Takes Too Long To Position At The Finish

Check terminal logs:

- terminal state;
- velocity terminal capture activation distance;
- position capture reason;
- final hold speed and radius;
- current speed near the final point.

Position capture should take over near the end. If it does not, check terminal
thresholds and speed.
