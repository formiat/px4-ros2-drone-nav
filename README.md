# PX4 ROS 2 Drone Navigation

This repository is a ROS 2 workspace for a PX4/Gazebo drone navigation stack.
The main package is `drone_city_nav`, an ament CMake package built with
`colcon`.

## Status And Safety

This project is a simulation-oriented research and development stack. It is
tested with PX4 SITL in Gazebo and is not certified or validated for real
aircraft operation.

Use it as a planning, simulation, and offboard-control testbed. Do not use it
on physical drones without a separate safety review, hardware-specific failsafe
design, controlled test environment, and compliance with local regulations.

## Approved Commands

Run commands from the repository root through the dev container. The container
workflow is the only supported build, test, quality, and simulation workflow for
this repository.

Use the top-level wrapper scripts for common workflows:

```bash
./scripts/build.sh
./scripts/test.sh
./scripts/sim_gui.sh
./scripts/sim_headless.sh
./scripts/stop_sim.sh
```

These wrappers start the dev container with the current UID/GID so generated and
formatted files remain owned by the invoking user. `./scripts/dev_shell.sh`
remains available when you need an interactive container shell. Inside that
shell, use these targets:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-gui
make sim-headless
```

The shared container entrypoint sources the supported ROS 2 and PX4 message
workspaces automatically before running any command. Do not manually source ROS
or `px4_msgs` setup files for the normal workflow. For a custom image or
external dependency checkout, override `ROS_SETUP_FILE` or `PX4_MSGS_SETUP_FILE`
instead of editing the scripts.

Build the ROS package:

```bash
./scripts/build.sh
```

Run unit tests:

```bash
./scripts/test.sh
```

Run script-level tests inside an interactive container shell:

```bash
make test-scripts
```

Run the non-mutating C++ quality checks inside an interactive container shell:

```bash
make quality
```

Format only changed C++ files inside an interactive container shell:

```bash
make format
```

Run the GUI simulation:

```bash
./scripts/sim_gui.sh
```

Stop all running simulation leftovers, including related Gazebo/PX4/ROS
processes and simulation containers:

```bash
./scripts/stop_sim.sh
```

Preview what would be stopped without killing anything:

```bash
./scripts/stop_sim.sh --dry-run
```

Gazebo GUI runs stop conflicting stale Gazebo simulator processes before
starting, because this project does not support multiple simultaneous Gazebo
instances on the same workstation. The cleanup is enabled by default and logs
all candidate containers and PIDs before terminating them. Use
`DRONE_GAZEBO_CLEAN_STALE_DRY_RUN=true` to list candidates without killing, or
`DRONE_GAZEBO_CLEAN_STALE_PROCESSES=false` only for intentional debugging.

By default, the Gazebo 3D view asks the Gazebo `CameraTracking` GUI plugin to
follow the PX4-spawned drone model `x500_lidar_2d_0`. Disable this with
`ENABLE_GZ_GUI_FOLLOW_CAMERA=false`, change the target with
`GZ_GUI_FOLLOW_TARGET`, or adjust the third-person camera offset with
`GZ_GUI_FOLLOW_OFFSET="-12 0 6"`. The GUI launch keeps the default Gazebo GUI
config path and unpauses the simulation separately through Gazebo world
control.

By default, RViz also opens in a follow-camera debug view that targets the
visualization-only `drone_follow` TF frame. Disable that behavior with
`ENABLE_RVIZ_FOLLOW_CAMERA=false`; the runner will then open the top-down RViz
layout instead. This switch only changes RViz visualization and does not affect
navigation or offboard control.

After a GUI run, validate deterministic Gazebo launch diagnostics:

```bash
python3 scripts/validate_gazebo_gui_launch_log.py \
  log/gz_drone_nav.log \
  --gui-log log/gz_gui_drone_nav.log \
  --scene-diagnostics-dir log/gazebo_scene_debug
```

GUI runs keep Gazebo server/world orchestration output in
`log/gz_drone_nav.log` and Gazebo GUI client output in
`log/gz_gui_drone_nav.log`. The launcher also captures bounded Gazebo scene
diagnostics under `log/gazebo_scene_debug/` by default. Disable only the
scene diagnostics with `ENABLE_GZ_SCENE_DIAGNOSTICS=false` when you need a
minimal run.

Run a headless smoke validation:

```bash
./scripts/sim_headless.sh
```

Equivalent explicit command inside an interactive container shell:

```bash
make sim-headless
```

Record a debug rosbag while the simulation is running:

```bash
./scripts/record_debug_bag.sh
```

The container targets use `build/`, `install/`, and `log/`.

The simulation uses three planner obstacle sources by default: the static
`generated_city.map2d` city map, accumulated lidar obstacle memory, and the
current lidar hit overlay. Source toggles and map format details are documented
in `docs/obstacle_mapping.md` and `docs/configuration.md`.

Obstacle topics follow a strict raw/prohibited/debug contract. Raw sources such
as `/drone_city_nav/obstacle_memory_grid` must contain only direct obstacle
evidence and must never include safety inflation. The planner overlays all raw
sources, inflates that merged grid once, and publishes the final planning output
as `/drone_city_nav/prohibited_grid`. Debug topics such as lidar point clouds or
prohibited-cell point clouds are visualization outputs and must not be wired
back into planner raw inputs.

After a headless run, validate lidar projection snapshots without GUI:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

Offboard flight diagnostics are also written as JSON Lines to
`log/offboard_blackbox.jsonl` by default. This file mirrors the 2 Hz telemetry
logs with path ids, cross-track error, heading error, commanded target motion,
commanded velocity, vehicle attitude, and nearest-obstacle bearing.
Final trajectory samples are dumped to `log/final_trajectory_samples/` on every
trajectory rebuild. Corridor samples are dumped to `log/corridor_samples/` on
every corridor rebuild. Each directory contains timestamped history files and a
`latest.csv` shortcut for the most recent rebuild.

## Build System

The approved build system entry point is `colcon`, not direct top-level CMake.
The C++ package itself uses modern target-based CMake in
`drone_city_nav/CMakeLists.txt`.

Do not introduce an ad-hoc build directory when an existing `build/` directory
and compile database are already available. The normal build
commands keep the build out of the source package and export a compile database
for tooling.

## Dependencies

Project dependencies are managed through:

- ROS 2 and Gazebo system packages in `docker/Dockerfile`.
- `px4_msgs` built into `/opt/px4_msgs_ws` by the dev image.
- PX4 Autopilot cloned by `scripts/setup_px4_autopilot.sh` into `external/`.

The wrapper scripts source `/opt/ros/${ROS_DISTRO}/setup.bash` and
`/opt/px4_msgs_ws/install/setup.bash` inside the container before invoking
`make`, `colcon`, or simulation commands.

Do not vendor new dependencies without documenting why they cannot be provided
by ROS, system packages, or a clearly pinned external checkout.

## Formatting And Static Analysis

Formatting uses the repository `.clang-format`. Do not run mutating
`clang-format -i` over the whole repository. Use `make format`, or pass `--all`
to `./scripts/format_cpp_changed.sh` only when intentionally normalizing the
project in the active environment.

Reviewer checks should be non-mutating:

```bash
make quality
```

`clang-tidy` is only run when a compile database is available. If the database
or a tool is missing, the check script reports an explicit skipped check with a
reason.

## Documentation

The main documentation set starts at `docs/overview.md`.

Key pages:

- `docs/installation.md`
- `docs/build_and_run.md`
- `docs/gazebo_simulation.md`
- `docs/architecture.md`
- `docs/navigation_pipeline.md`
- `docs/known_passages.md`
- `docs/trajectory_optimization.md`
- `docs/drone_control.md`
- `docs/terminal_capture.md`
- `docs/replanning.md`
- `docs/obstacle_mapping.md`
- `docs/configuration.md`
- `docs/diagnostics.md`
- `docs/testing.md`
- `docs/development.md`
- `docs/troubleshooting.md`
- `docs/performance.md`
