# Build And Run Workflow

All build, test, quality, and simulation commands must use the container
workflow. Do not run ad-hoc top-level CMake commands on the host.

## Common Host Scripts

Run these from the repository root:

```bash
./scripts/bootstrap.sh
./scripts/build.sh
./scripts/test.sh
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
./scripts/stop_sim.sh
```

Use `./scripts/dev_shell.sh` when an interactive container shell is needed.
`./scripts/bootstrap.sh` prepares a fresh clone end to end (dev image, PX4
checkout and SITL build, workspace build, urban assets) and starts the urban
point-to-point simulation; see `installation.md`.

All host scripts go through `scripts/container_run.sh`. The container runner
sources the supported ROS 2 setup and the container-built `px4_msgs` setup
before running the requested command. Normal builds do not require manual
`source /opt/ros/...` or `source /opt/px4_msgs_ws/...` steps.

If you intentionally use a custom image or external dependency install, set:

```bash
ROS_SETUP_FILE=/path/to/ros/setup.bash
PX4_MSGS_SETUP_FILE=/path/to/px4_msgs/setup.bash
```

## The Onboard Build (Roadmap Item 27 Block 2)

What a flight computer carries is built without the simulator:
`drone_city_nav/CMakeLists.txt` puts everything that needs Gazebo or SDFormat
(the contact system, the truth adapter, the smoke relay, the Gazebo stereo
process, the heading source, the crash node, the SDF voxelizer) behind
`DRONE_CITY_NAV_ENABLE_SIMULATION` (ON in the development image), and the
Dockerfile's `onboard` stage carries ROS 2 base, the messages, Eigen, OpenCV,
zstd, `px4_msgs` and CUDA 13 (on amd64; a Jetson brings CUDA with JetPack)
and no Gazebo. `tools/onboard_image.sh` builds the stage as
`drone-gazebo-onboard:amd64`, then the package in it from the mounted
workspace with the option off (into `build-onboard-amd64/`) and runs the
unit tests, the CUDA ones on the host's GPU; `--arm64` builds the arm64
stage under emulation (`qemu-user-static`, `binfmt-support`) without CUDA
and tests it there, slowly. The development image is the Dockerfile's `dev`
stage (`scripts/build_dev_image.sh`). Measured on 2026-10-25: amd64: the image 2.35 GB with 0 Gazebo packages and CUDA 13.0; the package with the option off built in 6 min and its 154 unit tests passed (the seven CUDA ones on the host's GPU). arm64 under emulation: the stage 1.75 GB (77 min to build, px4_msgs's generation the long part), the package in 1 h 2 min on 16 threads, its warnings a subset of the amd64 build's, 140 of 142 unit tests passed (139 at first; the estimator test's ctest timeout raised from 60 to 600 s, its 4.7 s running past a minute at the emulation's speed); the two left, the D* Lite repair's share of a 20 ms budget and the initial search's certification within its budget, assume the development host's speed and are not x86 dependencies, of which none was found.

## Commands Inside The Dev Shell

Inside `./scripts/dev_shell.sh`, use:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-urban-point-to-point-gui
make sim-urban-point-to-point-headless
make sim-cooperative-traffic-urban-gui
make sim-cooperative-traffic-urban-headless
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

Point-to-point mission:

```bash
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
```

Both fly the default sensor set, the forward stereo pair and the two
time-of-flight sensors, with the lidar absent from the vehicle, and the
`visual_inertial` localization profile: no GNSS and no magnetometer either.
The 3D lidar profile, which defaults to `LOCALIZATION_PROFILE=lidar_inertial`,
is a request, as is `LOCALIZATION_PROFILE=gnss` on either sensor set:

```bash
./scripts/sim_urban_point_to_point_lidar_headless.sh
```

A headless stereo flight of the mission takes 245 to 459 s of wall time on the
reference workstation (real-time factor 0.8 to 1.0); the target's flight
window is 600 s.

Finite cooperative traffic mission:

```bash
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
```

The cooperative launch carries the same sensor defaults per vehicle and has
not been flown on them; `CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar`
is the configuration it was accepted with.

The complete finite scenario is defined in
`drone_city_nav/config/cooperative_traffic_urban_scenario.json`. The shell
runner and ROS launch both load this file, and each Gazebo spawn is derived from
the world's `map_to_sdf` transform. Set `MULTI_VEHICLE_SCENARIO_PATH` to run
another validated scenario; do not add independent shell spawn overrides.

Every vehicle plans its own route and publishes its own flight intent. Conflicts
are resolved between peers through those intents; no node plans for another
vehicle and no vehicle receives the physical truth of another.

The mission start barrier waits for every planner world and for confirmed
agreement between each PX4 navigation pose and its physical Gazebo model pose. A
persistent mismatch blocks mission start and commands airborne vehicles to hold.
Static planner readiness comes from the resident Occupancy3D ESDF and does not
wait for a lidar snapshot.

The headless command validates every PX4 log. The mission result requires all
vehicles to reach their goals with their separation contract satisfied, or it
records a typed failure. The GUI command keeps Gazebo and RViz open after either
outcome; stop it explicitly when inspection is complete.

A mission error never requests disarm. Force-disarm occurs only after a typed
physical-collision or proximity-collision destruction event, and a physical
crash is a failed technical run settled only after that vehicle's disarm and the
confirmed holds of the survivors.

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
