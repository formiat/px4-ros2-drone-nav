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

## Adding Tests

Use:

- `drone_city_nav/tests/` for C++ unit tests;
- `scripts/tests/` for Python script/contract tests.

Add tests near the feature being changed. For shared serialization,
configuration, planner, or control contracts, add direct tests rather than only
relying on integration behavior.

## Before Commit

Minimum expected checks:

1. Format changed C++ files if any.
2. Run targeted new/changed tests if any.
3. Run `make quality`.
4. Confirm `git status --short` contains only intended changes.

Generated logs, bags, build outputs, and `.agent-io` transport files must not
be committed.
