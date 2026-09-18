# Contributing

All C++ development must follow `CPP_BEST_PRACTICES.md`.

## C++ Workflow Decision Tree

1. Prefer repository-approved commands from `README.md`, this file, `Makefile`,
   scripts, and CI configuration.
2. The default and only supported workflow is the container workflow. Use
   `./scripts/build.sh`, `./scripts/test.sh`,
   `./scripts/sim_urban_point_to_point_headless.sh`, and
   `./scripts/sim_urban_point_to_point_gui.sh` for common workflows. Start `./scripts/dev_shell.sh`
   only when an interactive container shell is needed; inside the container use
   `make build`, `make test`, `make test-scripts`, `make quality`,
   `make sim-urban-point-to-point-headless`, and
   `make sim-urban-point-to-point-gui`.
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

## Releases

Code releases are annotated tags `vMAJOR.MINOR.PATCH` on `main`; environment
assets keep their own `environment-assets-*` tags. To cut a release:

1. Fly the validation series on the release commit with nothing changed
   between flights (`log/tools/series.sh` runs the urban point-to-point
   flights one at a time and records `gz_pose.csv` and `tracking.npz`); record
   the table in `CHANGELOG.md` with the commit, the flights, the mean speed,
   the crashes, the route availability and the planner p95.
2. In one commit: bump `<version>` in `drone_city_nav/package.xml`, add the
   `CHANGELOG.md` entry (validated scenario, laws in force, known
   limitations, compatible asset tags), and update the roadmap status.
3. Run `make format`, `make quality` and `make test-scripts` in the container,
   commit, then `git tag -a vX.Y.Z -m "..."` on that commit and push `main`
   and the tag.
4. Publish the GitHub release from the tag with the changelog entry as its
   text. No build artifacts are attached: the code builds from source and the
   assets are released separately.

Every flight's runtime manifest records the package version and
`git describe`, so a run is attributable to a release by name.

## Scope Rules

- Keep production C++ in `drone_city_nav/include` and `drone_city_nav/src`.
- Keep tests in `drone_city_nav/tests`.
- Keep generated files, build outputs, logs, bags, and simulator runtime data
  out of version control.
- Treat `external/` as a local dependency checkout area, not project source.
- Keep planner obstacle inputs raw. The no-static planner consumes one immutable
  revisioned 3D raw-obstacle snapshot and derives a sparse occupied-distance cache
  for soft queries; exact raw occupancy remains the hard validator. The matching
  `/drone_city_nav/raw_obstacle_snapshot_3d` and cumulative dirty chunks provide
  activation provenance. Visualization grids must not be connected back to
  planner inputs.
