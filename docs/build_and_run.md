# Build And Run Workflow

All build, test, quality, and simulation commands must use the container
workflow. Do not run ad-hoc top-level CMake commands on the host.

## Common Host Scripts

Run these from the repository root:

```bash
./scripts/build.sh
./scripts/test.sh
./scripts/sim_gui.sh
./scripts/sim_headless.sh
./scripts/sim_intercept_gui.sh
./scripts/sim_intercept_headless.sh
./scripts/stop_sim.sh
```

Use `./scripts/dev_shell.sh` when an interactive container shell is needed.

All host scripts go through `scripts/container_run.sh`. The container runner
sources the supported ROS 2 setup and the container-built `px4_msgs` setup
before running the requested command. Normal builds do not require manual
`source /opt/ros/...` or `source /opt/px4_msgs_ws/...` steps.

If you intentionally use a custom image or external dependency install, set:

```bash
ROS_SETUP_FILE=/path/to/ros/setup.bash
PX4_MSGS_SETUP_FILE=/path/to/px4_msgs/setup.bash
```

## Commands Inside The Dev Shell

Inside `./scripts/dev_shell.sh`, use:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-gui
make sim-headless
make sim-intercept-gui
make sim-intercept-headless
```

`make build` runs `colcon build` for `drone_city_nav` with build, install, and
log directories rooted at `build/`, `install/`, and `log/`.

`make test` builds and then runs:

```bash
ctest --test-dir build/drone_city_nav --output-on-failure
```

`make test-scripts` runs Python script-level tests:

```bash
python3 -m unittest discover scripts/tests
```

`make quality` runs the repository quality gate, including dry-run formatting,
build, C++ tests, scoped `clang-tidy`, and scoped `cppcheck` when the required
inputs are available.

## Simulation Commands

GUI simulation:

```bash
./scripts/sim_gui.sh
```

Headless smoke run:

```bash
./scripts/sim_headless.sh
```

Finite three-interceptor versus one-evader mission:

```bash
./scripts/sim_intercept_gui.sh
./scripts/sim_intercept_headless.sh
```

Set `EVADER_SPEED_SCALE` to override the default `1.0` evader speed multiplier.
By default, the evader flies diagonally across the city from map position
`(270, 54)` to `(54, 378)` at `18 m` altitude.
The complete finite scenario is defined in
`drone_city_nav/config/intercept_scenario.json`. The shell runner and ROS launch
both load this file, and each Gazebo spawn is derived from the canonical
`map_to_sdf` transform. Set `INTERCEPT_SCENARIO_PATH` to run another validated
scenario; do not add independent shell spawn overrides.
Each interceptor uses a latency-compensated analytic intercept solution capped
at 15 s. While ahead inside the target corridor, the smoothed lead is capped at
1 s. All three interceptors use the measured motion direction by default. Set
`INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED=true` to enable the `0`, `+45`, and
`-45` degree long-range hypotheses; the lateral hypotheses converge to zero
within 30 m and are capped at 70 m.
Set `INTERCEPT_NONCOOPERATIVE_AVOIDANCE_ENABLED=true` to enable the attacker's
radar-only collision-avoidance pipeline in interception missions. It is
disabled by default.
Interceptors receive no evader coordinates. Three ideal radar adapters publish
only range, azimuth, elevation, and radial velocity at a deterministic varying
cadence between 0.1 s and 3.0 s. A typed planner command switches it immediately
to 20 Hz whenever the current target estimate has swept raw-clear visibility,
independent of range, and returns it to varying search cadence when the target is
occluded. A variable-dt tracker reconstructs and coasts a target track, and
guidance continues at 20 Hz between scans.
The mission start barrier waits for all four planner worlds, the first valid
target position from every tracker, and confirmed agreement between each PX4
navigation pose and its physical Gazebo model pose. A persistent mismatch
blocks mission start and commands airborne vehicles to hold. Static planner
readiness comes from the resident Occupancy3D ESDF and does not wait for a lidar
snapshot.
Prediction includes measurement age and is clipped only by physical raw
occupancy in the active static or sensor-derived map. Vertical prediction
decelerates vertical target motion to a stop and remains inside the configured
flight envelope. Visibility of the current target uses direct moving-target MPPI
pursuit. If only the full predicted intercept point is blocked, the planner
shortens the lead while preserving direct mode.
The headless command validates all four PX4 logs. An intercept result requires
typed Gazebo physical-proximity evidence at or below 5 m, confirmed disarm of
the capturing pair, and confirmed holds from every surviving
interceptor. Survivors must publish and maintain a stationary
position-hold horizon. An evader-goal result requires all surviving interceptors
to stop tracking and confirm the same hold transition without disarming. If
inertial motion causes a late capture after evader goal arrival, both pair
disarms are required while the original evader-goal outcome remains unchanged.
The GUI command keeps Gazebo and RViz open after either outcome; stop it
explicitly when inspection is complete. No attacker respawn or repeated episode
is performed.

A mission error never requests disarm. Force-disarm occurs only after a typed
physical-collision or proximity-intercept destruction event. A physical evader
crash is a failed technical run and is settled only after evader disarm plus a
confirmed interceptor hold.

Stop simulator leftovers:

```bash
./scripts/stop_sim.sh
```

Preview cleanup without terminating processes:

```bash
./scripts/stop_sim.sh --dry-run
```

The GUI workflow logs Gazebo server output to `log/gz_drone_nav.log` and Gazebo
GUI output to `log/gz_gui_drone_nav.log`.

## Logs And Artifacts

Important runtime artifacts:

- `log/mppi/` - production MPPI JSONL diagnostics;
- `log/lidar_debug/` - lidar snapshots and projection diagnostics;
- `log/lidar_memory_hits/` - accepted and classified lidar-memory hits;
- `log/gazebo_scene_debug/` - bounded Gazebo scene diagnostics;
- `build/` - colcon build tree;
- `install/` - colcon install tree.

Generated build outputs and logs are not project source and should not be
committed.

## Debug Bags

Record a debug ROS bag while simulation is running:

```bash
./scripts/record_debug_bag.sh
```

The script records the main planning, obstacle, and debug topics used for
post-run analysis.
