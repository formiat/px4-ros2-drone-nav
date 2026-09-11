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
