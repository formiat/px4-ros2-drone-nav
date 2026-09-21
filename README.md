# PX4 ROS 2 Drone Navigation

This repository is a ROS 2 workspace for a PX4/Gazebo drone navigation stack.
The main package is `drone_city_nav`, an ament CMake package built with
`colcon`.

## Demo Video

[![Autonomous navigation of an urban location with no map, no GNSS and no lidar](https://img.youtube.com/vi/OUuAj2WNKzs/maxresdefault.jpg)](https://www.youtube.com/watch?v=OUuAj2WNKzs)

[Watch on YouTube](https://www.youtube.com/watch?v=OUuAj2WNKzs): point-to-point
flight through the Urban Circuit Practice 01 location with no static map, no
GNSS, no magnetometer and no lidar, on the forward stereo pair that both
perceives the building and localizes the vehicle, as released in v0.4.0
(September 2026).

## Quick Start

On a Linux host with Docker, git and the NVIDIA container runtime, one script
prepares a fresh clone and starts the urban point-to-point simulation:

```bash
git clone https://github.com/formiat/px4-ros2-drone-nav.git
cd px4-ros2-drone-nav
./scripts/bootstrap.sh
```

It builds the dev image, clones PX4 Autopilot into `external/`, builds PX4
SITL and the workspace inside the container, fetches the versioned urban
environment assets, and launches the Gazebo GUI flight. `--headless` runs the
same flight headless with the mission check; `--no-run` only prepares. Every
step is skipped when its result already exists, so the script can be rerun.
The first run downloads several gigabytes and builds for tens of minutes.

## Roadmap

The project roadmap is maintained in [`docs/roadmap.md`](docs/roadmap.md). It
covers cooperative air traffic, generalized static 3D passages, no-static 3D
lidar perception, lidar-inertial localization, realistic cooperative
communication, camera-based 3D perception without lidar or static maps,
flight without GNSS and without lidar, and flight in degraded visual
conditions. What is known to be wrong or unfinished and has been set aside
is in [`docs/technical_debt.md`](docs/technical_debt.md).

## Releases

Code releases are tagged `vMAJOR.MINOR.PATCH` on `main` and described in
[`CHANGELOG.md`](CHANGELOG.md); environment asset bundles carry their own
`environment-assets-*` tags. The current release is `v0.4.0`: point-to-point
navigation through a complex 3D urban location with no static map, no GNSS, no
magnetometer and no lidar, on a stereo pair that both perceives and
localizes. Each flight's runtime manifest records the package version and
`git describe`.

## Status And Safety

This project is a simulation-oriented research and development stack. It is
tested with PX4 SITL in Gazebo and is not certified or validated for real
aircraft operation.

Use it as a planning, simulation, and offboard-control testbed. Do not use it
on physical drones without a separate safety review, hardware-specific failsafe
design, controlled test environment, and compliance with local regulations.
No onboard computer has run it: the resource figures are from a workstation
([docs/resource_budget.md](docs/resource_budget.md)). By default the vehicle
carries no lidar: it navigates on a forward stereo pair and two time-of-flight
sensors ([docs/camera_perception.md](docs/camera_perception.md)), and it flies
without GNSS: a visual-inertial estimator on the same pair replaces the
simulated GNSS, the magnetometer and the simulation heading source
(`LOCALIZATION_PROFILE=visual_inertial`, the default,
[docs/localization.md](docs/localization.md)). On the 3D lidar profile
(`CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar`) a lidar-inertial
estimator does the same (`lidar_inertial`). `gnss` is a request, and what the
multi-vehicle missions fly until the vehicles share a frame (roadmap item 15). Both camera assumptions are untested: every flight so far was lit by a uniform
ambient fill, because the imported world carries no light source at all and the
cameras carry no noise model, and the 6.4 m of confident depth was measured on
one location's photogrammetric surfaces rather than on the blank walls a matcher
cannot match (roadmap item 17).

## Integration Notes

What cost the most flights was rarely the algorithm. These are the findings
that are easy to lose weeks to, each with the flight it was measured on:

- **The autopilot's clock.** PX4 SITL is lockstep, so its clock is the
  simulation clock, but uXRCE-DDS synchronises it against the agent's wall
  clock. Below a real-time factor of 1 the two drift, PX4 resets the
  synchronisation every 15 s or so, and each reset left the stack about a
  second without an authoritative state, 17 times in one flight, and lost one
  vehicle. `UXRCE_DDS_SYNCT` is 0 and the nodes map the stamps with the
  identity ([docs/gazebo_simulation.md](docs/gazebo_simulation.md)).
- **What an odometry's noise means to EKF2.** `EKF2_EVP_NOISE` has to cover
  what one update moves the pose by, not how accurate the estimate is: at
  0.1 m a 0.4 m correction was gated out, the vehicle dead-reckoned 1.8 s and
  reset its position by 1.19 m. The estimator's own variances are not passed
  at all, because an odometry's honest variance grows without bound and the
  autopilot has no other position ([docs/localization.md](docs/localization.md)).
- **The noise densities are not the sensor's.** A filter told the gyroscope is
  eighteen times noisier than it is lets every visual update walk the heading,
  which no camera observes: 2 to 4 degrees per flight, 1.6 to 2.1 m at the
  goal. The accelerometer's is not its datasheet figure either, because tilt
  error leaks gravity ([docs/localization.md](docs/localization.md)).
- **One sensor, two consumers, no dependency between them.** The stereo pair
  both perceives and localizes, but the estimator never reads the obstacle
  memory: a pose taken from a map built from that pose hides its own drift.
  A contract test holds the estimator's library to Eigen and the standard
  library alone.
- **Arrival has to be judged by the truth.** The mission monitor asks the
  vehicle where it thinks it is, so with an odometry it can arrive in its own
  coordinates while standing elsewhere. Every flight fails unless the true
  pose is inside the 2.0 m capture radius at the acknowledgement
  ([docs/testing.md](docs/testing.md)).

What is known to be wrong or unfinished, with the class of each entry, is in
[docs/technical_debt.md](docs/technical_debt.md).

## Approved Commands

Run commands from the repository root through the dev container. The container
workflow is the only supported build, test, quality, and simulation workflow for
this repository.

Use the top-level wrapper scripts for common workflows:

```bash
./scripts/bootstrap.sh
./scripts/build.sh
./scripts/test.sh
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh
./scripts/stop_sim.sh
```

These wrappers start the dev container with the current UID/GID so generated and
formatted files remain owned by the invoking user. Every `sim_*.sh` wrapper runs
the same cleanup as `./scripts/stop_sim.sh` before its run and again when the
run ends, however it ends; `stop_sim.sh` stops every container of this
repository that runs a simulation target and every simulation process on the
host, and touches nothing else. Before every run the wrappers also delete
simulation logs older than a week (`./scripts/prune_sim_logs.sh`: entries of
`log/`, run directories, PX4 flight logs; `log/tools` and any entry holding a
`.keep` file stay; `--dry-run` previews, `DRONE_GAZEBO_PRUNE_LOGS=false`
disables). `./scripts/dev_shell.sh`
remains available when you need an interactive container shell. Inside that
shell, use these targets:

Every development container requires and receives the NVIDIA runtime. The
container wrapper fails before starting if the host NVIDIA runtime is not
available, so CUDA runtime failures are not deferred to ROS nodes or tests.

```bash
make build
make test
make test-scripts
make quality
make format
make sim-cooperative-traffic-urban-gui
make sim-cooperative-traffic-urban-headless
make sim-urban-point-to-point-gui
make sim-urban-point-to-point-headless
ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 make sim-environment-demo
```

The point-to-point mission visits sequential waypoints. Its scenario owns the
default sequence; override it with `MISSION_GOALS_XYZ_M` using
`x,y,z;x,y,z;...` syntax. Each waypoint is terminal and the mission succeeds
only after the vehicle settles at the last one. A single-destination mission
uses the same parameter with one `x,y,z` triple.

Build and run the isolated CUDA MPPI benchmark:

```bash
make mppi-benchmark \
  MPPI_BENCHMARK_ARGS="--scenario urban_blocks --rollouts 8192 --steps 80"
```

The benchmark target is opt-in and does not add CUDA to the normal ROS runtime
targets. It requires the NVIDIA container runtime; `scripts/container_run.sh`
passes the host GPU through automatically when that runtime is available.

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
./scripts/sim_urban_point_to_point_gui.sh
```

Simulation runs use no static map by default. Set `ENABLE_STATIC_MAP=true`
explicitly when a static-map run is required.

## Environment Spectator Demos

Launch a downloaded environment without PX4, ROS, RViz, lidar, or a mission:

```bash
ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh
```

Gazebo's free camera is the spectator: use its normal mouse and keyboard camera
controls to inspect the world. The demo materializes local visual resources and
does not require a network connection after the relevant environment assets have
been fetched.

Its position, orientation, and world-space forward direction are logged once
per second by default in `log/environment_demo/<environment-id>/gz_gui_free_camera.jsonl`.
Override the cadence or destination with `GZ_GUI_CAMERA_LOG_INTERVAL_S` and
`GZ_GUI_CAMERA_LOG_FILE`.

Available IDs are:

```text
finals_prize_round_world_07
cave_circuit_practice_01
urban_circuit_practice_01
tunnel_circuit_practice_01
cave_world
industrial_warehouse
aws_robomaker_small_warehouse
aws_robomaker_hospital
```

The first three IDs use versioned release artifacts. The remaining IDs are
local evaluation candidates and report a clear error if their cached source
assets are absent.

`LIDAR_PROFILE=none|3d` selects the production perception profile. Every
simulation entry point defaults to the 3D lidar. A static-map run may disable
lidar entirely:

```bash
ENABLE_STATIC_MAP=true LIDAR_PROFILE=none ./scripts/sim_urban_point_to_point_gui.sh
```

Since roadmap item 14 closed, every simulation entry point flies on cameras:
`CAMERA_PROFILE=stereo_tof` mounts a forward stereo pair (1280 x 960, 120
degrees, 0.20 m baseline, 7.5 Hz) and two 8 x 8 time-of-flight sensors looking
up and down, and `NAVIGATION_SENSOR_PROFILE=stereo_tof` navigates on them with
the lidar absent from the vehicle model
([`docs/camera_perception.md`](docs/camera_perception.md)). The 3D lidar
remains available on request:

```bash
CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar ./scripts/sim_urban_point_to_point_headless.sh
```

`CAMERA_PROFILE=stereo_tof NAVIGATION_SENSOR_PROFILE=lidar` flies on the lidar
with the vision path in shadow beside it. `LIDAR_PROFILE=3d` still names the
3D perception pipeline and the vehicle wrapper either way. On the camera
profile the headless flights run on the textured GUI world instead of the
collision-only sensor world, which a camera sees nothing of, and
single-vehicle flights default to the `visual_inertial` localization profile:
no GNSS, no magnetometer, no lidar.

No-static navigation requires `LIDAR_PROFILE=3d` and rejects `none` before
starting the simulation. Unknown and free volume have identical traversability
and base cost; only confirmed occupied geometry is a hard spatial obstacle.

Acceptance uses the urban environment, no static map, and only the 3D profile:

```bash
./scripts/sim_urban_point_to_point_headless.sh
```

Every run writes `log/runs/<run-id>/manifest.json`. The manifest binds the exact
Git commit, navigation configuration, world, mission scenario, and runtime
profile by SHA-256. When constrained-volume validation is enabled, a
transient-local subscriber also retains the exact observed/occupied bit words
for that volume in a revisioned `raw_snapshot_3d_revision_<revision>.json`
artifact. The headless acceptance gate verifies the artifact hash and payload,
zero ownership gaps, greater than 97 percent post-bootstrap route availability,
fewer than 3 percent of post-bootstrap ticks in ordinary no-route holds, the
measured planner p95 target, and every admitted successor-reserve proof (see
`docs/testing.md`, "Headless Acceptance Gate", for the thresholds and why).
Set `DRONE_GAZEBO_RUN_ID` and `DRONE_GAZEBO_LOG_DIR` when several acceptance
runs must remain independently discoverable.

No-static single-vehicle headless runs automatically enable the persistent-3D
acceptance gate. Repeated acceptance runs must be sequential and use distinct
`DRONE_GAZEBO_RUN_ID` and `DRONE_GAZEBO_LOG_DIR` values so their manifests,
metrics, and raw-volume snapshots remain independently auditable. Optional
route-volume bounds are consumed only by the headless evaluator and are never
supplied to mapping or planning.

When `POINT_TO_POINT_SCENARIO_PATH` is set, the scenario owns its waypoint
sequence unless `MISSION_GOALS_XYZ_M` is also supplied explicitly.

## Flight Speed Profile

Navigation uses one map-independent horizontal flight profile. The defaults are
6.5 m/s cruise speed (just under the speed the sensor-braking contract admits
for the guaranteed lidar range), 10 m/s absolute speed limit, 4 m/s² maximum
horizontal acceleration and 6 m/s² lateral acceleration in turns. The planner,
finite-path stopping model, and PX4 configuration receive the same values, and
the simulation targets in the `Makefile` pass the same cruise.

Override the profile for an individual run with environment variables:

```bash
CRUISE_SPEED_MPS=6.5 \
ABSOLUTE_SPEED_LIMIT_MPS=10 \
MAXIMUM_HORIZONTAL_ACCELERATION_MPS2=4 \
./scripts/sim_cooperative_traffic_urban_headless.sh
```

These values are independent of map source.

Run the finite cooperative civilian traffic mission in the imported Urban
Circuit Practice 01 environment. Both entrypoints use only 3D lidar and online
obstacle memory:

```bash
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
```

This target verifies and installs the versioned environment release artifacts,
materializes a Gazebo Harmonic collision world, leaves all static navigation
artifact paths empty, and launches the four-vehicle online-mapping scenario from
`drone_city_nav/config/cooperative_traffic_urban_scenario.json`.

Run the base single-drone no-static flight in the same environment:

```bash
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
```

The scenario is defined once in
`drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json`.
Its map-space launch pose is transformed by the world's `map_to_sdf` contract
for Gazebo, while the same pose sets the PX4 origin and the navigation start. The
headless target additionally validates the persistent full-3D planner, route
ownership continuity, successor reserve, physical route-volume traversal, and
measured runtime latency.

The finite scenario in
`drone_city_nav/config/cooperative_traffic_urban_scenario.json` launches two
pairs of civilian drones from opposite ends of the same urban corridor. The two
parallel routes of a pair start only 2 m apart, deliberately forcing cooperative
separation immediately after launch. Each pair carries opposing traffic through
the corridor. Every vehicle owns its own PX4, navigation, mapping, and MPPI
pipeline. All vehicles start and cruise at the same altitude; no fixed altitude
layers are assigned.

At 20 Hz, each vehicle publishes a typed `CooperativeFlightIntent` containing
its current state, physical footprint, bounded-validity MPPI horizon, and active
passage use. Every cooperative agent independently rejects stale or out-of-order
peer intents, predicts the continuous closest approach over a five-second
horizon, and optimizes a deterministic space-time maneuver against all current
conflicting trajectories together. Candidate plans combine continuous lateral or
vertical displacement with an optional bounded entry-time shift; safe plans are
ranked by separation, route progress, effort, and stable pair preference. Conflict
and incumbent-plan state are latched briefly and released with hysteresis. The
result is a soft planner preference and peer-separation cost, not a prohibited
grid, inflated obstacle, or hard exclusion volume; raw physical obstacles remain
the only hard collision constraint.

Static-map passage topology is derived offline from matching raw `Occupancy3D`
and `ESDF3D` artifacts. The generalized compiler classifies footprint-feasible
clearance topology, extracts arbitrarily oriented portal voxel patches, and
stores a sparse medial segment graph. It does not use roof presence, axis-aligned
portal assumptions, or all-pairs portal edges. This fingerprint-bound topology
is offline passage evidence for visualization, diagnostics, and optional
execution metadata; it is not a competing strategic route producer.

For route-associated passage execution, the planner samples route-orthogonal cross-sections from
raw `Occupancy3D`, validates the full drone footprint in both transverse axes,
and builds a varying local free-space envelope. Opposing traffic coordinates by
shared sparse segment resources and uses deterministic continuous offsets when
the measured volume provides enough separation. A narrow or otherwise
conflicting passage schedules entry time with deterministic right-of-way; an
active constrained span is never replaced mid-traversal. In no-static mode, peer
lidar returns are removed only from persistent obstacle memory. The unchanged
latest scan still reaches immediate safety validation, so cooperative filtering
cannot hide a real close-range obstacle.

The mission referee uses Gazebo ground truth only for readiness and physical
adjudication. Headless success requires all four drones to reach and physically
hold at their own goals, no vehicle destruction or building collision, and a
complete minimum-separation report. Both static-map and no-static-map workflows
are supported. The GUI spectator starts on `civilian_0` and uses cyclic
`next_living` selection.

Mission outcome and vehicle death are separate contracts. Mission failures never
request disarm. Force-disarm is owned only by the latched death lifecycle and is
accepted only for a physical Gazebo collision or a typed 5 m proximity death. A
vehicle that physically crashes has its death and disarm confirmed while the
survivors receive a typed objective for confirmed stationary position hold.

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

By default, the Gazebo 3D view uses Gazebo's `CameraTracking` plugin. The
point-to-point mission follows the PX4-spawned model `x500_lidar_2d_0`;
multi-vehicle missions derive the model from the typed spectator selection. Disable the
camera with `ENABLE_GZ_GUI_FOLLOW_CAMERA=false`, change the point-to-point target
with `GZ_GUI_FOLLOW_TARGET`, or adjust the third-person camera offset with
`GZ_GUI_FOLLOW_OFFSET="-7 0 3.5"`. The runner waits for the initial model to appear
in the server scene before starting the GUI, then repeatedly publishes an
ID-aware native `CameraTrack` command until the resulting target state remains
stable. The conflicting `/gui/follow` service is intentionally not used.
Simulation unpause remains a separate Gazebo world-control operation.

Multi-vehicle scripts expose `MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID` and
`MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY`. The latter accepts `first_living`
or `next_living`. `first_living` always selects the lowest-index living scenario
vehicle; `next_living` scans forward from the destroyed vehicle and wraps at the
end of the scenario list. `MULTI_VEHICLE_SPECTATOR_RESELECTION_DELAY_S` controls
the handoff delay and defaults to three seconds.

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
`log/gz_gui_drone_nav.log`. The current free-camera position, orientation, and
world-space forward direction are sampled once per second in
`log/gz_gui_camera.jsonl`; set `GZ_GUI_CAMERA_LOG_INTERVAL_S` or
`GZ_GUI_CAMERA_LOG_FILE` to override that behavior. The launcher also captures
bounded Gazebo scene diagnostics under `log/gazebo_scene_debug/` by default.
Disable only the scene diagnostics with `ENABLE_GZ_SCENE_DIAGNOSTICS=false`
when you need a minimal run.

Run a headless smoke validation:

```bash
./scripts/sim_urban_point_to_point_headless.sh
```

Equivalent explicit command inside an interactive container shell:

```bash
make sim-urban-point-to-point-headless
```

Record a debug rosbag while the simulation is running:

```bash
./scripts/record_debug_bag.sh
```

The container targets use `build/`, `install/`, and `log/`.

Static mode loads an environment's raw Occupancy3D, its fingerprint-bound
FreeSpaceTopology3D, and its precomputed chunked ESDF3D in
`production_mppi_node`. Those artifacts are optional per environment; the
current Urban Circuit environment ships none of them and every mission flies
no-static. Static planning loads the separate free-space topology index as
optional static passage metadata. Route
production remains owned by the same persistent full-3D planner used in
no-static mode; the topology artifact does not instantiate a competing search
or route owner. There is no hand-authored planner centerline, semantic lane, or
nearest-portal selector.
No-static mode has one production perception pipeline. The 3D profile decodes
every organized scan into hit and miss beams, resolves the full 6DoF acquisition
pose, and integrates the rays into revisioned `unknown/free/occupied`
`Occupancy3D`. Dirty chunks update derived distance evidence without giving it
collision authority. Planning treats free and unknown identically; exact raw
occupied geometry remains the only hard spatial obstacle.

The static free-space topology index remains an optional compatibility
acceleration for static planning. It is not generated or consumed by
the no-static 3D pipeline. Source contracts are documented in
`docs/world3d.md`, `docs/obstacle_mapping.md`, and `docs/configuration.md`.

Obstacle topics follow a strict raw/runtime/debug contract.
`/drone_city_nav/obstacle_memory_status` is the lightweight per-update heartbeat.
For production navigation, `/drone_city_nav/raw_obstacle_snapshot_3d` and
`/drone_city_nav/raw_obstacle_delta_3d` carry adaptive base snapshots and the
latest cumulative dirty chunks. Persistent integration and DDS serialization
run on independent latest-value workers; superseded work is coalesced rather
than allowed to make sensor evidence stale. The larger debug representation is
published at a bounded cadence and is not deserialized by the planner. Raw grids
contain only direct sensor evidence. Each timestamp-aligned scan first publishes
`/drone_city_nav/latest_sensor_obstacle_scan`; while fresh, those physical hit
points validate the complete finite path without waiting for persistent-memory
integration. The planner builds a distance-derived risk field without
materializing inflated grids. Static mode instead loads raw Occupancy3D
directly. `/drone_city_nav/raw_obstacle_grid` is visualization-only and must not
be wired back into planner or offboard validation.

RViz publishes the selected spectator's latest 3D returns on
`/drone_city_nav/current_lidar_returns_3d` and its downsampled accumulated
occupied voxels on `/drone_city_nav/raw_memory_obstacle_points_3d`. The current
cloud has queue depth one and no decay; accumulated memory is rate-limited.
Other vehicles retain only their lightweight pose and path displays.

All published targets and execution horizons use the configured flight envelope
`1.0 <= z < 32.0 m`. Raw collision checks use the drone's swept oriented 3D
footprint, including horizontal radius and upper/lower body extents. This is
physical vehicle geometry, not an inflated prohibited region; ESDF risk bands
remain finite route-ranking costs.

Every normal execution path uses the configured horizon duration and includes
an arrival speed profile within its own samples. Its final point has zero
translational velocity and yaw rate. No separate motion phase is appended after
the endpoint. Arrival shaping uses a conservative horizontal deceleration limit that is
independent of the larger acceleration available to ordinary static-map manoeuvres.
When a new valid receding-horizon path arrives it immediately supersedes the
previous path or a temporary no-executable position hold. If one planning
update cannot provide a replacement, both the remaining path geometry and its
remaining controls from the measured vehicle state are validated. A divergent
path is rebuilt from that measured state, with a complete arrival profile inside
the remaining control slots. It continues
only after complete raw-world validation and never past the previous deadline.
It therefore reaches its own terminal rest instead of being discarded solely
because the next update failed.

After a headless run, validate lidar projection snapshots without GUI:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl
```

Production MPPI diagnostics are written as rate-limited JSON Lines under
`log/mppi/`; recent full records are also retained in a bounded ring and dumped
to `mppi_error_context.jsonl` when a collision episode begins.
Synchronized lidar, raw-grid, and local-horizon snapshots are written under
`log/lidar_debug/`. The simulation wrapper prints the exact per-run artifact
directory.

## Resource Budget

Every headless flight records what its processes consume, and the mission
check reads the record. On r345 (392.8 m in 142 s, the same flight as the
figures above) the onboard processes, the navigation nodes and the DDS
agent, used 2.73 cores at p50 and 3.36 at p95 of a Ryzen 9 5900HX, 798 MiB
of resident memory and 206 MiB of GPU memory, with the GPU at 42 percent of
an RTX 3060 Laptop; the simulator ran at a real-time factor of 1.00. What
that does and does not say about a drone computer, and the assumptions
behind naming the Jetson Orin family as the class the measurements do not
exclude, are in [docs/resource_budget.md](docs/resource_budget.md).

## Build System

The approved build system entry point is `colcon`, not direct top-level CMake.
The C++ package itself uses modern target-based CMake in
`drone_city_nav/CMakeLists.txt`.

Production navigation services are compile-time separated from ROS composition:

```text
drone_city_nav_production_mppi_component  (src/runtime/ros)
  -> drone_city_nav_mppi_runtime          (src/runtime)
  -> drone_city_nav_route_runtime         (src/planning, trajectory, execution)
  -> drone_city_nav_world_runtime         (src/world)
```

These package-private libraries have independent build-only include roots. The
world, route, and MPPI runtimes are ROS-free; the component contains the ROS
capture/apply/publish adapters and does not link the compatibility core umbrella.
Header-graph and manifest tests enforce the boundary rather than relying only on
declared `target_link_libraries` edges. See
[`docs/architecture.md`](docs/architecture.md#compile-time-runtime-boundaries).

Do not introduce an ad-hoc build directory when an existing `build/` directory
and compile database are already available. The normal build
commands keep the build out of the source package and export a compile database
for tooling.

## Dependencies

Project dependencies are managed through:

- ROS 2 and Gazebo system packages in `docker/Dockerfile`.
- `px4_msgs` built into `/opt/px4_msgs_ws` by the dev image.
- PX4 Autopilot cloned by `scripts/setup_px4_autopilot.sh` into `external/`
  and built there as `px4_sitl` inside the dev container; the 3D-lidar
  profile launches that binary directly. `scripts/bootstrap.sh` does both.

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
- `docs/roadmap.md`
- `docs/navigation_pipeline.md`
- `docs/world3d.md`
- `docs/environment_candidates.md`
- `environments/environment_manifest.yaml`
- `docs/trajectory_optimization.md`
- `docs/drone_control.md`
- `docs/terminal_capture.md`
- `docs/replanning.md`
- `docs/obstacle_mapping.md`
- `docs/camera_perception.md`
- `docs/configuration.md`
- `docs/diagnostics.md`
- `docs/testing.md`
- `docs/development.md`
- `docs/troubleshooting.md`
- `docs/performance.md`
- `docs/technical_debt.md`
