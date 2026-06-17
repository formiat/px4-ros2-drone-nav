# Contributing

All C++ development must follow `CPP_BEST_PRACTICES.md`.

## C++ Workflow Decision Tree

1. Prefer repository-approved commands from `README.md`, this file, `Makefile`,
   scripts, and CI configuration.
2. The default routine workflow is the native host workflow. Use `make
   host-build`, `make host-test`, `make host-quality`, and
   `make host-sim-headless` from the repository root unless the task explicitly
   requires container validation. The host workflow writes to `build-host/`,
   `install-host/`, and `log-host/`.
3. Use the container workflow only for container-specific work, reproducibility
   checks, or when the native host environment is unavailable. Start it with
   `./scripts/dev_shell.sh`; inside the container use `make build`,
   `make test`, `make quality`, `make sim-headless`, and `make sim-gui`.
4. This repository is a ROS 2 workspace. Use `colcon` as the approved build
   entry point. Do not invent a top-level direct CMake workflow unless the
   repository later adds one explicitly.
5. If CMake presets are added in the future, prefer
   `cmake --preset <name>` and `cmake --build --preset <name>` for the scope
   they cover. Until then, use the documented `colcon` commands.
6. Reuse existing build directories and compile databases when they are valid.
   For native work, prefer `build-host/`; for container work, prefer `build/`.
   Do not delete or recreate build directories unless the failure mode requires
   it.
7. Prefer tests through the documented Make targets:

   ```bash
   make host-test
   ```

   Inside the container, use:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure
   ```

   If the build directory does not exist, build first with the documented
   `colcon build` command.
   For Python helper scripts, run:

   ```bash
   make host-test-scripts
   ```

8. Do not run mutating formatters across the whole repository. Format changed
   C++ files only:

   ```bash
   make host-format
   ```

9. Reviewer checks must be non-mutating. Use:

   ```bash
   make host-quality
   ```

   This runs dry-run formatting, scoped `clang-tidy` when a compile database is
   available, the approved build command, and `ctest`.
10. Use `--all` only for an explicit whole-project audit. The default policy is
   changed-file scope so routine reviews do not create broad formatting churn.
11. If a command or tool choice is ambiguous, do not guess. Record the skipped
   check and the reason in the review or task notes.

## Scope Rules

- Keep production C++ in `drone_city_nav/include` and `drone_city_nav/src`.
- Keep tests in `drone_city_nav/tests`.
- Keep generated files, build outputs, logs, bags, and simulator runtime data
  out of version control.
- Treat `external/` as a local dependency checkout area, not project source.
