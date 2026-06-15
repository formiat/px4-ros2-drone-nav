# Drone Gazebo ROS 2 PX4 MVP

This repository is a ROS 2 workspace for a Gazebo + PX4 SITL drone navigation
MVP. The main package is `drone_city_nav`, an ament CMake package built with
`colcon`.

## Approved Commands

Run commands from the repository root inside the dev container or an equivalent
environment where ROS 2 Jazzy and `px4_msgs` are sourced.

Start the dev container with the repository helper:

```bash
./scripts/dev_shell.sh
```

The helper runs the container with the host UID/GID so generated and formatted
files remain owned by the host user.

Build the ROS package:

```bash
make build
```

Equivalent explicit command:

```bash
colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Run unit tests:

```bash
make test
```

Equivalent explicit command:

```bash
ctest --test-dir build/drone_city_nav --output-on-failure
```

Run script-level tests:

```bash
make test-scripts
```

Run the non-mutating C++ quality checks:

```bash
make quality
```

Equivalent explicit command:

```bash
./scripts/check_cpp_quality.sh
```

Format only changed C++ files:

```bash
./scripts/format_cpp_changed.sh
```

Run the GUI simulation:

```bash
make sim-gui
```

Equivalent explicit command:

```bash
./scripts/run_city_mvp.sh
```

Run a headless smoke validation:

```bash
make sim-headless
```

Equivalent explicit command:

```bash
HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
```

The simulation uses three planner obstacle sources by default: the static
`generated_city.map2d` city map, accumulated lidar obstacle memory, and the
current lidar hit overlay. Source toggles and map format details are documented
in `docs/MVP_SIMULATION.md`.

After a headless run, validate lidar projection snapshots without GUI:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

## Build System

The approved build system entry point is `colcon`, not direct top-level CMake.
The C++ package itself uses modern target-based CMake in
`drone_city_nav/CMakeLists.txt`.

Do not introduce an ad-hoc build directory when an existing `build/` directory
and `compile_commands.json` are already available. The normal build command
keeps the build out of the source package and exports a compile database for
tooling.

## Dependencies

Project dependencies are managed through:

- ROS 2 and Gazebo system packages in `docker/Dockerfile`.
- `px4_msgs` built into `/opt/px4_msgs_ws` by the dev image.
- PX4 Autopilot cloned by `scripts/setup_px4_autopilot.sh` into `external/`.

Do not vendor new dependencies without documenting why they cannot be provided
by ROS, system packages, or a clearly pinned external checkout.

## Formatting And Static Analysis

Formatting uses the repository `.clang-format`. Do not run mutating
`clang-format -i` over the whole repository. Use
`./scripts/format_cpp_changed.sh`, or pass `--all` only when intentionally
normalizing the project.

Reviewer checks should be non-mutating:

```bash
./scripts/check_cpp_quality.sh --format --tidy
```

`clang-tidy` is only run when a compile database is available. If the database
or a tool is missing, the check script reports an explicit skipped check with a
reason.

## Documentation

Simulation launch and validation details are documented in
`docs/MVP_SIMULATION.md`.
