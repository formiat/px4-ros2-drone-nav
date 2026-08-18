# PX4 ROS 2 Drone Navigation

This repository is a ROS 2 workspace for a PX4/Gazebo drone navigation stack.
The main package is `drone_city_nav`, an ament CMake package built with
`colcon`.

## Demo Videos

| Autonomous interceptor mission | Navigation without a static map |
|:--:|:--:|
| [![Autonomous interceptor mission](https://img.youtube.com/vi/ouRDE7C2NvM/maxresdefault.jpg)](https://youtu.be/ouRDE7C2NvM) | [![Navigation without a static map](https://img.youtube.com/vi/DAHCt6dmAAE/maxresdefault.jpg)](https://youtu.be/DAHCt6dmAAE) |
| [Watch on YouTube](https://youtu.be/ouRDE7C2NvM) | [Watch on YouTube](https://youtu.be/DAHCt6dmAAE) |

The videos demonstrate autonomous interception and sensor-driven navigation as
of August 2026.

## Roadmap

The project roadmap is maintained in [`docs/roadmap.md`](docs/roadmap.md). It
covers the interceptor mission, radar-derived target tracking, predictive
guidance, multi-drone scenarios, cooperative air traffic, generalized static 3D
passages, and future no-static 3D perception.

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
./scripts/sim_intercept_gui.sh
./scripts/sim_intercept_headless.sh
./scripts/sim_multi_intercept_gui.sh
./scripts/sim_multi_intercept_headless.sh
./scripts/sim_cooperative_traffic_gui.sh
./scripts/sim_cooperative_traffic_headless.sh
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh
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
make sim-intercept-gui
make sim-intercept-headless
make sim-multi-intercept-gui
make sim-multi-intercept-headless
make sim-cooperative-traffic-gui
make sim-cooperative-traffic-headless
make sim-cooperative-traffic-urban-gui
make sim-cooperative-traffic-urban-headless
make sim-urban-point-to-point-gui
make sim-urban-point-to-point-headless
ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 make sim-environment-demo
```

The base `sim` mission visits sequential point-to-point waypoints. The default
route follows the four city corners. Override it with `MISSION_GOALS_XYZ_M`
using `x,y,z;x,y,z;...` syntax; each waypoint is terminal and the mission
succeeds only after the vehicle settles at the last one.

A single-destination mission uses the same parameter with one `x,y,z` triple.

```bash
MISSION_GOALS_XYZ_M='216,54,18;216,378,18;54,378,18;54,54,18' \
  ./scripts/sim_headless.sh
```

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
./scripts/sim_gui.sh
```

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

The 2D lidar is enabled by default. In a static-map run it can be disabled to
remove the simulated sensor and its ROS scan bridge:

```bash
ENABLE_2D_LIDAR=false ./scripts/sim_gui.sh
```

No-static navigation requires the 2D lidar and rejects
`ENABLE_2D_LIDAR=false` before starting the simulation.

## Flight Speed Profile

Navigation uses one map-independent horizontal flight profile. The defaults are
5 m/s cruise speed, 10 m/s absolute speed limit, and 4 m/s² maximum horizontal
acceleration. The planner, finite-path stopping model, and PX4 configuration
receive the same values.

Override the profile for an individual run with environment variables:

```bash
CRUISE_SPEED_MPS=5 \
ABSOLUTE_SPEED_LIMIT_MPS=10 \
MAXIMUM_HORIZONTAL_ACCELERATION_MPS2=4 \
./scripts/sim_cooperative_traffic_headless.sh
```

These values are independent of map source. Complex environments use the
default profile; Manhattan can use a faster explicit profile for experiments.

Run the finite three-interceptor mission:

```bash
./scripts/sim_intercept_gui.sh
./scripts/sim_intercept_headless.sh
```

The point-to-point mission remains the default. The intercept mission launches
three isolated interceptor PX4/ROS stacks and one evader stack. The
interceptors start in three separated city sectors; one starts at the evader's destination
but receives neither that destination nor any other attacker ground truth. The
evader flies diagonally to its fixed goal with the same speed policy as the
interceptors. Each interceptor receives only its own ideal radar measurements
containing range, azimuth, elevation, and radial velocity; an independent
variable-dt tracker derives the target state used by predictive guidance. Scan
cadence follows a deterministic correlated random walk from 0.1 s to 3.0 s
while the current target estimate is occluded. Once a planner validates swept
raw-clear visibility of that estimate, a typed command triggers an immediate
scan and 20 Hz track mode without a range limit. Tracker coasting and guidance
continue at 20 Hz between scans. Only the three simulation radar adapters and
the mission referee may consume the typed physical target truth produced by the
simulation-truth adapter. Radar measurements and mission proximity are derived
from Gazebo model poses, not independently configured PX4 origins. Mission
motion starts only after all four planners report a resident world, all three
trackers have published a valid target position, and several consecutive
samples confirm that every navigation pose agrees with its Gazebo pose.
This coordinate agreement is a startup contract: once mission motion begins it
is latched for the episode. Later navigation-to-truth residuals remain visible
as diagnostics but do not stop physical adjudication or place the fleet in
hold.

All four map-frame starts and the evader goal are owned by
`drone_city_nav/config/intercept_scenario.json`. The runner derives each Gazebo
spawn from the canonical world's `map_to_sdf` transform; there are no separate
intercept spawn coordinates in the shell or launch file.

The continuous guidance objective has no terminal goal hold. It uses a
latency-compensated analytic intercept solution, capped at 15 s, and smoothly
caps the lead at 1 s when the interceptor is already ahead in the evader's
motion corridor. Vertical prediction models the target stopping its climb or
descent under bounded acceleration and clamps the result to the configured
half-open flight envelope. The planner treats current-target visibility and the
path to the predicted intercept point separately. A visible current target keeps
direct MPPI interception active; a blocked full-lead path shortens the prediction
toward the current target instead of dropping direct mode.
By default, all three interceptors predict the measured target direction. Set
`INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED=true` to assign the other two
interceptors `+45` and `-45` degree long-range motion hypotheses. Those offsets
converge continuously to zero below 30 m and their lateral lead is capped at
70 m. The radar track itself is never rotated or falsified.

A physically measured swept Gazebo separation of 5 m between any interceptor
and the evader publishes
typed `VehicleDestroyed` events for that pair. Their offboard nodes force-disarm
and confirm both deaths, while the other interceptors receive a typed hold objective
and settle into confirmed stationary position hold. A physical or 5 m proximity
collision between
interceptors destroys only the involved vehicles and the mission continues
while another interceptor is available. If the evader reaches its goal first,
the first airborne sample inside the goal radius latches that outcome, all
surviving interceptors stop tracking and settle into confirmed stationary
position hold; no vehicle is disarmed. A later inertial approach cannot change
the first outcome, although entering the capture radius still applies the normal
pair disarm. Evader goal arrival is an intercept failure but still a technically
successful simulation outcome. In the GUI workflow, RViz and Gazebo initially
follow the attacker `evader`. The default `first_living` policy selects the
first surviving scenario vehicle three seconds after the observed vehicle dies.
RViz keeps the lightweight route and direction
arrow of every interceptor visible. Its
full MPPI, memory, and lidar layers are routed from the current spectator only
and switch with the same spectator selection; optional per-interceptor memory
clouds remain disabled by default. The GUI
workflow remains open after either outcome. The headless workflow exits only
after all applicable hold and disarm settlements are confirmed in the log. The
mission contains one evader only; it does not respawn attackers or start another
episode.

Run the finite two-interceptor versus two-attacker mission separately:

```bash
./scripts/sim_multi_intercept_gui.sh
./scripts/sim_multi_intercept_headless.sh
```

This entry point uses the same generic launch and navigation code with
`drone_city_nav/config/multi_intercept_2v2_scenario.json`. The attackers start
on the short city side farthest from the destination: `evader_0` starts at its
corner and `evader_1` starts one block inward along that side. The interceptors
start on the opposite short side, next to the destination corner at
`(54, 378, 18)`. Both attackers fly toward that same fixed goal. Each interceptor
owns an independent radar simulator and
multi-target tracker; its `RadarScan` contains one relative spherical detection
per active attacker and still exposes no absolute target coordinates.

A central typed assignment coordinator compares estimated constant-velocity
intercept times and computes a deterministic minimum-cost allocation. In the
2x2 case it covers both active attackers with distinct interceptors whenever
valid tracks permit it. Assignment changes require a material, sustained cost
improvement, which prevents rapid target flapping. If an attacker is
intercepted, reaches its goal, or is destroyed, it is removed from future
allocation immediately and the surviving interceptors are reassigned to the
remaining active attackers. Radio transport and communication impairments are
not simulated.

The referee records exactly one first terminal outcome per attacker. An
interceptor-attacker separation of 5 m destroys and disarms only that pair;
other assignments continue. The finite episode ends after every attacker has a
terminal outcome, or fails if no interceptor remains while an attacker is still
active. Headless validation requires every captured pair to have physical
Gazebo proximity evidence and confirmed disarms, every survivor to confirm
position hold, and no vehicle to collide with a building. Directional motion
hypotheses are disabled in this supported scenario.

The `2x2` GUI starts with `evader_0` as the spectator and uses the cyclic
`next_living` policy. Three seconds after its destruction, the camera selects
`evader_1` when it is alive; otherwise it continues through the scenario order
and wraps to the first living vehicle. Gazebo, the RViz `drone_follow` frame,
and selected planner diagnostics consume the same typed spectator selection.

Run the cooperative civilian traffic mission:

```bash
./scripts/sim_cooperative_traffic_gui.sh
./scripts/sim_cooperative_traffic_headless.sh
```

Run the static cooperative mission in the imported Urban Circuit Practice 01
environment:

```bash
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
```

Static scenario preflight is disabled by default. To verify physical spawns and
the configured static-route contract before an Urban run, enable it explicitly:

```bash
ENABLE_STATIC_SCENARIO_PREFLIGHT=true \
  ./scripts/sim_cooperative_traffic_urban_headless.sh
```

This target verifies and installs the versioned environment release artifacts,
materializes a Gazebo Harmonic collision world, compiles the manifest-bound
FreeSpaceTopology3D artifact when needed, and launches the four-vehicle static
scenario from
`drone_city_nav/config/cooperative_traffic_urban_scenario.json`.

Run the base single-drone static flight in the same environment:

```bash
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
```

The scenario is defined once in
`drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json`.
Its map-space launch pose is transformed by the canonical world contract for
Gazebo, while the same pose sets the PX4 origin and the navigation start. The
when explicitly enabled, the static preflight check requires a supported
physical spawn, a clear vertical takeoff, and the selected static-route
contract before simulation starts.

The finite scenario in
`drone_city_nav/config/cooperative_traffic_scenario.json` launches two pairs of
civilian drones from opposite ends of the western interior street containing
the straight `passage_structure_54_162_straight` 3D passage. The two parallel routes start
only 2 m apart, deliberately forcing cooperative separation immediately after
launch, then fan out to destinations separated by 8 m. Each route carries
opposing traffic through the passage between the building rows. Every vehicle
owns its own PX4, navigation, mapping, and MPPI pipeline. All vehicles start and
cruise at the same altitude; no fixed altitude layers are assigned.

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
portal assumptions, or all-pairs portal edges. A route-specific
`PassageTraversal` is resolved lazily and cached only when global search needs it.

After route selection, the planner samples route-orthogonal cross-sections from
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

In the Gazebo view, interceptor visibility markers remain yellow and the evader
visibility marker is red. RViz continues to use its distinct per-role colors.

Mission outcome and vehicle death are separate contracts. Mission failures never
request disarm. Force-disarm is owned only by the latched death lifecycle and is
accepted only for a physical Gazebo collision or a typed 5 m proximity death. If the
evader physically crashes, its death/disarm is confirmed and a surviving
interceptor receives a typed objective for confirmed stationary position hold. A typed proximity
collision between two interceptors is the same physical death contract, not a
mission-failure disarm path.

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
point-to-point mission follows the PX4-spawned model `x500_lidar_2d_0`; intercept
missions derive the model from the typed spectator selection. Disable the
camera with `ENABLE_GZ_GUI_FOLLOW_CAMERA=false`, change the point-to-point target
with `GZ_GUI_FOLLOW_TARGET`, or adjust the third-person camera offset with
`GZ_GUI_FOLLOW_OFFSET="-12 0 6"`. The runner waits for the initial model to appear
in the server scene before starting the GUI, then repeatedly publishes an
ID-aware native `CameraTrack` command until the resulting target state remains
stable. The conflicting `/gui/follow` service is intentionally not used.
Simulation unpause remains a separate Gazebo world-control operation.

Intercept scripts expose `INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID` and
`INTERCEPT_SPECTATOR_RESELECTION_POLICY`. The latter accepts `first_living` or
`next_living`. `first_living` always selects the lowest-index living scenario
vehicle; `next_living` scans forward from the destroyed vehicle and wraps at the
end of the scenario list. `INTERCEPT_SPECTATOR_RESELECTION_DELAY_S` controls the
handoff delay and defaults to three seconds.

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

Static mode loads raw `generated_city.occupancy3d`, its fingerprint-bound
`generated_city.topology3d`, and precomputed chunked `generated_city.esdf3d` in
`production_mppi_node`. All three artifacts and `generated_city.sdf` are
generated from the same canonical world specification. The current city is a `5 x 8` Manhattan
building grid with two horizontal L-shaped air-passage structures, one
straight-through structure, and one T junction. Static planning loads the
separate free-space topology index and objectively compares ordinary lattice
routes with lazy sparse-graph traversals; no passage is mandatory. Selected
traversals directly create typed route spans with varying 3D cross-sections.
There is no hand-authored planner centerline, semantic lane, or nearest-portal
selector.
No-static mode uses the accumulated raw 2D lidar-memory world; collisionless lidar
occluders make all four passages appear closed in that mode. Source contracts are documented in
`docs/world3d.md`, `docs/obstacle_mapping.md`, and `docs/configuration.md`.

Obstacle topics follow a strict raw/runtime/debug contract.
`/drone_city_nav/obstacle_memory_status` is the lightweight per-update heartbeat,
while `/drone_city_nav/raw_obstacle_snapshot` carries the current raw grid used by
no-static planning. The larger atomic memory/provenance snapshot is published at
the debug cadence and is not deserialized by the planner. Raw grids contain only
direct obstacle evidence. Each timestamp-aligned scan also publishes
`/drone_city_nav/latest_lidar_obstacle_scan`; while fresh, those physical hit
points validate the complete finite path without waiting for persistent-memory
integration. In no-static mode the planner builds a
distance-derived risk field from that raw 2D world without materializing
inflated grids. Static mode instead loads canonical Occupancy3D directly. The
atomic `/drone_city_nav/raw_obstacle_snapshot` remains the runtime sensor-world
contract and freshness trigger. The `/drone_city_nav/raw_obstacle_grid` topic is
visualization-only and must not be wired back into planner or offboard
validation.

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
- `docs/configuration.md`
- `docs/diagnostics.md`
- `docs/testing.md`
- `docs/development.md`
- `docs/troubleshooting.md`
- `docs/performance.md`
