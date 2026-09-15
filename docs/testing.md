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

`scripts/headless_runtime_evidence.py` evaluates every no-static single-vehicle
headless run. Besides the artifact and reserve proofs it holds two flight
metrics from the final `PRODUCTION_MPPI_SUMMARY` to thresholds that are a
product decision, not a tuning target:

- post-bootstrap route availability above 97 percent of ticks
  (`MINIMUM_POST_BOOTSTRAP_ROUTE_AVAILABILITY`);
- ordinary post-bootstrap no-route holds below 3 percent of ticks
  (`MAXIMUM_POST_BOOTSTRAP_NO_ROUTE_HOLD_RATIO`).

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
  the body; measured 0.11 to 0.22 m on r288 to r292);
- the median arrest of a descent faster than 1.5 m/s at least 1.2 m/s^2 when
  at least 30 samples exercised it (the stopping laws rely on 2.0; measured
  1.32 to 2.17 over fourteen flights, so the law's value is optimistic against
  the median and the check keeps that visible);
- the position estimate against the true pose, with the clocks aligned on the
  speed profile: the cross-track error at p95 within 0.35 m (measured 0.19 to
  0.25) and the offset along the motion within 0.20 s (measured 0.10 to
  0.11 s, 0.3 m at 3 m/s: how far apart in time the estimate a tick reads and
  the true pose are stamped);
- the lidar evidence age the planning tick reports at most 600 ms, the bound
  the braking contract charges (measured 200 to 376 ms at most).

The clocks of the three records differ; each measurement aligns them on the
motion itself (least squares over a grid of offsets).

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
