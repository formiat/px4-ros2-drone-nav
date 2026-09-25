# Testing And Quality

All verification must use the container workflow.

## Main Commands

From the host:

```bash
./scripts/build.sh
./scripts/test.sh
```

Inside `./scripts/dev_shell.sh`:

```bash
make build
make test
make test-scripts
make quality
make format
```

The dev shell and host wrapper scripts use the same container entrypoint. That
entrypoint sources ROS 2 and `px4_msgs` automatically, so test commands should
not require manual setup-file sourcing.

## Unit Tests

C++ tests are registered through CMake and run with:

```bash
make test
```

The command builds `drone_city_nav` and then runs:

```bash
ctest --test-dir build/drone_city_nav --output-on-failure
```

## Script Tests

Python script-level tests are run with:

```bash
make test-scripts
```

These tests cover scripts, contracts, Gazebo log validation, topic contracts,
source size contracts, and telemetry contracts.

## Quality Gate

Before committing code changes, run:

```bash
make quality
```

This runs:

- clang-format dry-run;
- package build;
- C++ tests;
- scoped clang-tidy when a compile database is available;
- scoped cppcheck.

For C++ formatting, use:

```bash
make format
```

Do not run broad formatting over the entire repository unless intentionally
normalizing the project.

## Headless Acceptance Gate

**What fails a flight, and what does not (project owner, 2026-09-19).** The
project has two requirements, and the mission check fails on nothing else:

- the vehicle does not crash and completes its mission: no crash event, no
  contact with a static obstacle, the mission monitor's successful result and
  every waypoint (for the cooperative mission, the referee's separation and
  every vehicle at its goal), and the goal reached in truth: at every goal
  acknowledgement the TRUE position of the vehicle (`gz_pose.csv`) is inside
  the 2.0 m capture radius of the goal the acknowledgement names
  (`validate_goal_reached_in_truth`). The mission monitor judges by the
  autopilot's estimate, and an odometry drifts by metres over a flight:
  without this line a vehicle arrives in its own coordinates only. The truth
  is read by the check and nowhere in the control loop;
- the mean flight speed exceeds the figure of the sensor set: 2.4 m/s on the
  lidar, 1.2 m/s on the stereo set. The path is the vehicle's positions
  between mission readiness and the successful result, the time is the
  simulation clock between the two, read from the true pose record
  (`gz_pose.csv` carries both clocks); the wall-clock figure is printed
  beside it.

What says the flight was the one asked for fails too, because a pass would
otherwise say nothing: the stack came up and flew (memory, ESDF, a route, an
applied horizon, arming, take-off), the runtime manifest binds the commit, the
configuration, the world and the scenario, the localization profile is proved
from the logs, and the autopilot logged no critical simulator error.

Every other measurement described below is a **note**: route availability and
no-route holds, execution-ownership gaps, the planner's p95, the tick's wall
time, the controller-dynamics measurements, the sensor evidence age, the
resource record and the route-volume witness. A measurement outside its
reference figure is printed as `NOTE:` with the figure it is outside of, where
it used to print `FAIL:`, and never changes the check's result. The reference
figures stay in the code as what the programme has measured before; they are
for reading a flight and for optimization work, which may add any metric it
needs as long as it fails nothing. A note that moves is looked into because it
may be how one of the two requirements will fail.

`scripts/headless_runtime_evidence.py` evaluates every no-static single-vehicle
headless run. Besides the artifact and reserve proofs it holds two flight
metrics from the final `PRODUCTION_MPPI_SUMMARY` to thresholds that are a
product decision, not a tuning target:

- post-bootstrap route availability above 97 percent of ticks
  (`MINIMUM_POST_BOOTSTRAP_ROUTE_AVAILABILITY`);
- ordinary post-bootstrap no-route holds below 3 percent of ticks
  (`MAXIMUM_POST_BOOTSTRAP_NO_ROUTE_HOLD_RATIO`).

It also bounds the production tick's wall time from the same summary, snapshot
to publication: 30 ms at p50 and 45 ms at p95 (`MAXIMUM_TICK_TOTAL_P50_MS`,
`MAXIMUM_TICK_TOTAL_P95_MS`), and prints the share of ticks over the 20 ms
deadline. These are regression bounds on the measured 22.7 / 30.8 ms of r314;
the vehicle receives a fresh horizon at the rate the whole tick allows, and 71
percent of r314's ticks still overran the deadline.

The pair was set on 2026-09-11, replacing 99 and 1 percent. Measured urban
point-to-point flights of the current stack fall into two groups: clean
flights with three to five no-route episodes and 0.4 to 0.7 percent of hold
ticks, and flights that meet two or three of the known tight spots (the
corner shaft, the goal approach, the northern corridor) with eight to twelve
episodes and 2.0 to 2.5 percent. Availability sits at 98 to 99.6 percent in
both. The old pair was met by one flight in eleven and every other flight
failed on the same two lines, so the checks below them stopped being read.
The 97/3 pair is met by both groups and still rejects the regressions the
programme has seen (3.7 and 4.3 percent of holds). Five consecutive flights
on one commit pass it (r198 to r202).

What the remaining holds cost, and where, is recorded per flight in
`log/runs/<run-id>/ros_drone_nav.log`; the next step towards 1 percent is
the recovery after a physical block, 0.3 to 1.2 s without a route each time.

### Controller dynamics

Every headless flight records the offboard setpoints against the autopilot's
local position (`tracking.npz`, `scripts/capture_tracking_setpoints.py`) and
the true pose of the vehicle from Gazebo (`gz_pose.csv`,
`scripts/capture_gazebo_pose.py`); a GUI flight records them with
`DRONE_GAZEBO_CAPTURE_DYNAMICS=true`. `scripts/controller_dynamics_evidence.py`
holds four measurements of those records to the assumptions the navigation
laws stand on, so a change of the autopilot, the simulator or the airframe
that breaks one is seen on the next flight rather than in a crash:

- lateral tracking error at p99 between 1.5 and 4.5 m/s within 0.25 m (the
  tube law budgets 0.075 s times the speed, the envelope keeps 0.27 m beyond
  the body; measured 0.11 to 0.22 m on r288 to r292 with the position
  estimate 0.3 m ahead of the vehicle, 0.08 to 0.18 m on r312 to r314 with
  EKF2_GPS_DELAY 0);
- the plateau of each arrest of a descent faster than 1.5 m/s (the peak of
  the 0.2 s window over the episode), at the median over at least three
  episodes, at least the vertical law's 1.4 m/s^2 (the law is the fifth
  percentile of the plateau over 91 episodes of 25 flights; measured 2.0 to
  2.13 at the median on r312 to r314);
- the position estimate against the true pose, with the clocks aligned on the
  speed profile: the cross-track error at p95 within 0.35 m on GNSS (measured
  0.19 to 0.25) and within 1.0 m, half the capture radius, on the odometry
  profiles, whose autopilot position follows the estimate and carries its
  drift (measured 0.42 to 0.99 m on the camera flights r607 to r622; the
  owner's decision of 2026-09-25), and the offset along the motion within
  0.20 s (measured 0.10 to 0.11 s, 0.3 m at 3 m/s: how far apart in time the
  estimate a tick reads and the true pose are stamped);
- the sensor evidence age the planning tick reports at most 600 ms, the bound
  the braking contract charges (measured 200 to 376 ms at most on the lidar;
  on the stereo profile 256 to 284 ms at p50, 436 to 508 at p95 and 624 to
  948 ms at most, about 1 percent of the ticks over the bound: the check is
  red on that profile and is left so).

On the `gnss_shadow` and `lidar_inertial` localization profiles the same
machinery reports the lidar-inertial estimator's own estimate against the
true pose (`lio_estimate.csv`, `scripts/capture_lidar_inertial_estimate.py`),
an `OK` line and no gate: the gate stays on the estimate the stack flies.
The along-track offset is read at the check's own resolution, the 0.02 s
alignment grid plus the pose age at the tick: the GNSS flights r340 to r356
read -0.013 to +0.022 s and the lidar-inertial flights r415 to r434 -0.029 to
+0.013 s,
while the autopilot's position and the odometry it fuses agree to 5 ms.

The clocks of the three records differ; each measurement aligns them on the
motion itself (least squares over a grid of offsets). The grid assumes the
clocks run at one rate, which a simulation slower than the wall clock breaks:
the setpoints and the true pose are stamped on the simulation clock, the
autopilot's positions on its wall-synchronised one and the log on the wall
clock (r493, real-time factor 0.86: 43 m of lateral error read on a sound
flight). Both recorders therefore keep the wall time each record was received
at, and the lateral-tracking and position-estimate measurements read that
clock when the record carries it; the lidar-inertial comparison stays on the
simulation clock, which both of its records share.

The mean flight speed is the only speed the programme targets, and it is
gated by the navigation sensor profile the manifest records, whatever
localizes the vehicle: it has to exceed 2.4 m/s on the lidar and 1.2 m/s on
the stereo sensor set. These are the project owner's requirements of
2026-09-19; they replaced 2.5 m/s and the 1.226 m/s derived for the stereo
profile as half of what its braking contract admits forward (6.4 m of
confident depth admit 2.452 m/s).

The clock the speed is measured on was the wall clock through the acceptance
series of items 14, 16 and the speed work of 2026-09-24/25, so a simulation
slower than real time lowered the figure by as much (r577: 1.22 m/s on the
wall clock for 1.53 m/s of simulation time under foreign desktop load; the
camera profile runs at a real-time factor of 0.89 to 1.00 on the reference
host). On 2026-09-25 the project owner decided that the requirement is
measured on the simulation clock, with the thresholds 2.4 and 1.2 m/s
unchanged, and the check reads it so since then. The base series, re-read by
the same check:

| Series | Commit | Wall clock, m/s | Simulation clock, m/s |
|---|---|---|---|
| cameras r607 to r611 | 8f6d248f | 1.572, 1.572, 1.728, 1.604, 1.742 (mean 1.644) | 1.702, 1.798, 1.842, 1.754, 1.858 (mean 1.791) |
| lidar r612 to r616 | 8f6d248f | 2.705, 2.604, 2.718, 2.666, 2.493 (mean 2.637) | the same: the lidar profile runs at real time |
| cameras r617, r618, r620 to r622 | 3df703ce | 1.706, 1.628, 1.689, 1.614, 1.637 (mean 1.655) | 1.928, 1.858, 1.866, 1.778, 1.761 (mean 1.838) |
| lidar r623 to r627 | 3df703ce | 2.580, 2.433, 2.703, 2.654, 2.606 (mean 2.595) | 2.587, 2.433, 2.703, 2.654, 2.614 (mean 2.598) |

Whatever the clock, a flight under foreign host load is not counted:
`scripts/quiet_host_gate.sh`, run by the simulation wrapper before every
flight, waits until the host has been quiet for a minute (no `rustc`, `cargo`
or `clippy` above 5 percent of a core, a one-minute load under 3, no other
`gz sim`), foreign processes are never touched, and a flight the load reaches
anyway is replayed.

### Resource record

Every flight, headless or not, records what its processes consume
(`resources.csv`, `scripts/capture_process_resources.py`): once a second,
per process, CPU in cores, resident memory and threads; the container's
cgroup totals; and Gazebo's real-time factor; and every ten seconds the GPU's
utilisation and memory with the memory by process name (asking the driver
every second stalled the simulator while the stereo pair rendered: real-time
factor under 0.9 for a third of a flight's seconds). The host is described once in
`resources_host.json`. `scripts/resource_budget_evidence.py` holds the record
against two gates and reports the rest, so a change of cost is seen on the
next flight:

- the record covers at least 90 percent of the seconds between mission
  readiness and the successful result (measured 98 percent on r345);
- no onboard process (`production_mppi_node`, `obstacle_memory_3d_node`,
  `mppi_offboard_node`, `MicroXRCEAgent`) gains more than 256 MiB of
  resident memory over the flight, the last tenth against the first at the
  median of each (measured +128 MiB for the controller and +53 MiB for the
  obstacle memory on r345, both the map growing with the observed volume; a
  leak at the tick rate crosses the bound within a flight).

The onboard set includes `lidar_inertial_odometry_node` when the flight
runs it. The reported lines give each onboard process's cores at p50, p95 and most,
memory at p95 and growth, the per-second sums of the onboard processes, of
the captures and of the simulator with the harness, the GPU figures and the
real-time factor. [resource_budget.md](resource_budget.md) reads them.

The same check reads what every consumer of a transport hop measured of its
deliveries (`transport_latency_ros.hpp`: the middleware's receive timestamp
against the source timestamp the publisher's middleware set): the
controller's two hops from its summary, the obstacle memory's from its
alignment report and the offboard node's from its applied-horizon report,
each as p50, p95 and maximum. One gate: the obstacle memory's snapshots and
deltas reach the controller within 2.5 ms at p95 (measured 0.25 to 0.32 ms,
1.24 under a foreign build on the host). From the tick line it also splits
the observation age at the median into the memory's scan-to-publication
time, the delivery and the wait for the tick (measured 184 to 200 ms at p50
at the 10 Hz transport: 114 to 136, 0.16 and 48 to 56).

### Localization profile

`validate_localization_profile` (`scripts/headless_runtime_evidence.py`)
reads the profile from the runtime manifest. On `lidar_inertial` it fails
the flight unless the autopilot log shows `EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE
5`, `EKF2_EV_CTRL 11` and `EKF2_HGT_REF 3`, the ROS log has no
`simulation_heading_source_node`, and the estimator published its odometry
at 5 Hz or more between mission readiness and the result (measured 10.4 to
10.5 Hz). It then reports the estimator's health over the flight, from the
node's once-a-second line: the matched share, the registration residual and
the weakest-axis information at p50 with their worst, and the scans that
left the autopilot without an estimate; any such scan fails the flight
(measured 0 of 1300 to 1700 on every accepted flight, matched share 0.88
to 0.91 at p50, residual 0.047 to 0.050 m). On `visual_inertial` the same four autopilot parameters, the absence of the
simulation heading source and of the lidar-inertial node, and a non-zero count
of poses sent to the autopilot are what prove the profile; the estimator's
health (features, residual, least certain velocity direction, frame cost, IMU
gaps, frames without a healthy estimate) and its error against the true pose
(`vio_estimate.csv`: cross-track, along-track, the error at the end of the
flight and the most it grew over 100 m of path) are notes on it and on
`visual_inertial_shadow`. The other profiles report their
name and gate nothing; a manifest without a profile is read as
`lidar_inertial`, the default. [localization.md](localization.md) describes the
estimator and the profiles.

## Adding Tests

Use:

- `drone_city_nav/tests/` for C++ unit tests;
- `scripts/tests/` for Python script/contract tests.

Add tests near the feature being changed. For shared serialization,
configuration, planner, or control contracts, add direct tests rather than only
relying on integration behavior.

Private runtime tests link the narrowest owning target. The navigation
dependency contract additionally verifies disjoint source manifests,
target-specific include roots, transitive local-header bans, and the ROS-only
component boundary. Do not replace executable orchestration coverage with a
Python test that depends on C++ expression order.

## Before Commit

Minimum expected checks:

1. Format changed C++ files if any.
2. Run targeted new/changed tests if any.
3. Run `make quality`.
4. Confirm `git status --short` contains only intended changes.

Generated logs, bags, build outputs, and `.agent-io` transport files must not
be committed.
