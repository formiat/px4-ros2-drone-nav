# Contributing

All C++ development must follow `CPP_BEST_PRACTICES.md`.

## C++ Workflow Decision Tree

1. Prefer repository-approved commands from `README.md`, this file, `Makefile`,
   scripts, and CI configuration.
2. The default and only supported workflow is the container workflow. Use
   `./scripts/build.sh`, `./scripts/test.sh`, `./scripts/sim_headless.sh`, and
   `./scripts/sim_gui.sh` for common workflows. Start `./scripts/dev_shell.sh`
   only when an interactive container shell is needed; inside the container use
   `make build`, `make test`, `make test-scripts`, `make quality`,
   `make sim-headless`, and `make sim-gui`.
3. This repository is a ROS 2 workspace. Use `colcon` as the approved build
   entry point. Do not invent a top-level direct CMake workflow unless the
   repository later adds one explicitly.
4. If CMake presets are added in the future, prefer
   `cmake --preset <name>` and `cmake --build --preset <name>` for the scope
   they cover. Until then, use the documented `colcon` commands.
5. Reuse existing build directories and compile databases when they are valid.
   The container workflow uses `build/`, `install/`, and `log/`. Do not delete
   or recreate build directories unless the failure mode requires it.
6. Prefer tests through the documented Make targets:

   ```bash
   make test
   ```

   Inside the container, use:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure
   ```

   If the build directory does not exist, build first with the documented
   `colcon build` command.
   For Python helper scripts, run:

   ```bash
   make test-scripts
   ```

7. Do not run mutating formatters across the whole repository. Format changed
   C++ files only:

   ```bash
   make format
   ```

8. Reviewer checks must be non-mutating. Use:

   ```bash
   make quality
   ```

   This runs dry-run formatting, scoped `clang-tidy` when a compile database is
   available, the approved build command, and `ctest`.
9. Use `--all` only for an explicit whole-project audit. The default policy is
   changed-file scope so routine reviews do not create broad formatting churn.
10. If a command or tool choice is ambiguous, do not guess. Record the skipped
   check and the reason in the review or task notes.

## Scope Rules

- Keep production C++ in `drone_city_nav/include` and `drone_city_nav/src`.
- Keep tests in `drone_city_nav/tests`.
- Keep generated files, build outputs, logs, bags, and simulator runtime data
  out of version control.
- Treat `external/` as a local dependency checkout area, not project source.
- Keep planner obstacle inputs raw. Runtime inflation is owned by the planner
  grid builder, which publishes the final `/drone_city_nav/prohibited_grid`.
  Debug or prohibited outputs must not be connected back to planner raw inputs.
