# Contributing

All C++ development must follow `CPP_BEST_PRACTICES.md`.

## C++ Workflow Decision Tree

1. Prefer repository-approved commands from `README.md`, this file, `Makefile`,
   scripts, and CI configuration.
2. This repository is a ROS 2 workspace. Use `colcon` as the approved build
   entry point. Do not invent a top-level direct CMake workflow unless the
   repository later adds one explicitly.
3. If CMake presets are added in the future, prefer
   `cmake --preset <name>` and `cmake --build --preset <name>` for the scope
   they cover. Until then, use the documented `colcon` commands.
4. Reuse existing `build/`, `install/`, and `compile_commands.json` artifacts
   when they are valid. Do not delete or recreate build directories unless the
   failure mode requires it.
5. Prefer tests through:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure
   ```

   If the build directory does not exist, build first with the documented
   `colcon build` command.
6. Do not run mutating formatters across the whole repository. Format changed
   C++ files only:

   ```bash
   ./scripts/format_cpp_changed.sh
   ```

7. Reviewer checks must be non-mutating. Use:

   ```bash
   ./scripts/check_cpp_quality.sh
   ```

   This runs dry-run formatting, scoped `clang-tidy` when a compile database is
   available, the approved build command, and `ctest`.
8. Use `--all` only for an explicit whole-project audit. The default policy is
   changed-file scope so routine reviews do not create broad formatting churn.
9. If a command or tool choice is ambiguous, do not guess. Record the skipped
   check and the reason in the review or task notes.

## Scope Rules

- Keep production C++ in `drone_city_nav/include` and `drone_city_nav/src`.
- Keep tests in `drone_city_nav/tests`.
- Keep generated files, build outputs, logs, bags, and simulator runtime data
  out of version control.
- Treat `external/` as a local dependency checkout area, not project source.
