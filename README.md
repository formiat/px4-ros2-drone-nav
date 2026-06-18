# Drone Gazebo ROS 2 PX4 MVP

This repository is a ROS 2 workspace for a Gazebo + PX4 SITL drone navigation
MVP. The main package is `drone_city_nav`, an ament CMake package built with
`colcon`.

## Approved Commands

Run commands from the repository root. The default workflow for agents and
routine development is the native host workflow through `./scripts/host_shell.sh`
and the `host-*` Make targets. Container commands remain supported for
container-specific validation and reproducible isolated runs.

### Default Native Host Workflow

The native host workflow uses a pinned micromamba environment and writes
generated files to `build-host/`, `install-host/`, and `log-host/`, leaving the
container build cache separate.

Prerequisites already installed on the main development machine:

- ROS 2 Jazzy and Gazebo Harmonic in
  `~/.local/share/mamba/envs/drone-gazebo-host`.
- `px4_msgs` built in `external/px4_msgs_ws_host`.
- `MicroXRCEAgent` installed into the same micromamba environment.
- PX4 SITL configured in `external/PX4-Autopilot/build-host` so the native
  cache does not conflict with the container-created PX4 cache. The wrapper
  patches the generated host Ninja file to C++17 because the Robostack/Gazebo
  protobuf headers require C++17 while PX4 1.17 defaults to C++14.

Enter the native host shell:

```bash
make host-shell
```

Build the ROS package natively:

```bash
make
```

Equivalent explicit target:

```bash
make host-build
```

Run unit tests natively:

```bash
make host-test
```

Run script-level tests natively:

```bash
make host-test-scripts
```

Run the non-mutating C++ quality checks natively:

```bash
make host-quality
```

Format only changed C++ files natively:

```bash
make host-format
```

Run the native GUI simulation:

```bash
make host-sim-gui
```

Equivalent explicit command:

```bash
./scripts/run_city_mvp_host.sh
```

Gazebo GUI runs stop stale Gazebo servers for the same runtime world before
starting, because duplicate servers can corrupt simulator time and make PX4
reject IMU samples. By default, the Gazebo 3D view asks the Gazebo
`CameraTracking` GUI plugin to follow the PX4-spawned drone model
`x500_lidar_2d_0` and opens the Gazebo camera tracking panel for interactive
tracking controls. The generated runtime Gazebo GUI config is only a launch
implementation detail: it loads the panel and keeps the GUI unpaused.

Run a native headless smoke validation:

```bash
make host-sim-headless
```

Equivalent explicit command:

```bash
HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp_host.sh
```

Run the native speed sweep:

```bash
make host-speed-sweep
```

Record a native debug rosbag while the simulation is running:

```bash
make host-record-debug-bag
```

### Container Workflow

Use the container workflow when validating the container image, reproducing a
container-only issue, or working on a machine without the native host
environment.

Start the dev container with the repository helper:

```bash
./scripts/dev_shell.sh
```

The helper runs the container with the host UID/GID so generated and formatted
files remain owned by the host user. Inside the container, use the
current-environment targets:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-gui
make sim-headless
```

The current-environment targets use `build/`, `install/`, and `log/`.

The simulation uses three planner obstacle sources by default: the static
`generated_city.map2d` city map, accumulated lidar obstacle memory, and the
current lidar hit overlay. Source toggles and map format details are documented
in `docs/MVP_SIMULATION.md`.

After a headless run, validate lidar projection snapshots without GUI:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log-host/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

## Build System

The approved build system entry point is `colcon`, not direct top-level CMake.
The C++ package itself uses modern target-based CMake in
`drone_city_nav/CMakeLists.txt`.

Do not introduce an ad-hoc build directory when an existing `build-host/` or
`build/` directory and compile database are already available. The normal build
commands keep the build out of the source package and export a compile database
for tooling.

## Dependencies

Project dependencies are managed through:

- Native host dependencies in the pinned micromamba environment and ignored
  `external/*_host` workspaces.
- ROS 2 and Gazebo system packages in `docker/Dockerfile`.
- `px4_msgs` built into `/opt/px4_msgs_ws` by the dev image.
- PX4 Autopilot cloned by `scripts/setup_px4_autopilot.sh` into `external/`.

Do not vendor new dependencies without documenting why they cannot be provided
by ROS, system packages, or a clearly pinned external checkout.

## Formatting And Static Analysis

Formatting uses the repository `.clang-format`. Do not run mutating
`clang-format -i` over the whole repository. Use `make host-format`, or pass
`--all` to `./scripts/format_cpp_changed.sh` only when intentionally
normalizing the project in the active environment.

Reviewer checks should be non-mutating:

```bash
make host-quality
```

`clang-tidy` is only run when a compile database is available. If the database
or a tool is missing, the check script reports an explicit skipped check with a
reason.

## Documentation

Simulation launch and validation details are documented in
`docs/MVP_SIMULATION.md`.
