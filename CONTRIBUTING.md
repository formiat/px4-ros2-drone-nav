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

## Before A Commit

Every commit is made with the whole gate green (specification A5):

```bash
./tools/gates.sh                 # detached; returns at once
cat log/tools/gate_flag.txt      # "X=0" is green; output in log/tools/gate_all.log
```

It runs `make format`, `make build`, `make test`, `make test-scripts` and
`make quality` in the container, in that order. Do not edit tracked files
while it runs. Then confirm `git status --short` holds only what was meant
and commit.

- Code, comments, documents and commit messages are in English, with the
  field's standard terms (UAV, SLAM, VIO, LIO, MSCKF, GNSS-denied, D* Lite,
  MPPI).
- A C++, launch or runner source holds at most 1000 non-blank lines; the
  script tests enforce it.
- A change of behaviour comes with its test: C++ in `drone_city_nav/tests`,
  scripts and contracts in `scripts/tests`
  ([`docs/testing.md`](docs/testing.md)). No production code exists for a
  test alone.
- A change of anything a flight could feel (navigation, estimation, control,
  the simulation runner, the models, the configuration) is flown before it is
  called done; see below. A change of the launch scripts is followed by one
  real flight at once: a contract test does not start a simulator.

## Flying

A flight is one simulation run of a scenario, started from the host:

```bash
./scripts/stop_sim.sh
DRONE_GAZEBO_RUN_ID=r1100 ./scripts/dev_shell.sh make sim-urban-point-to-point-headless
./scripts/stop_sim.sh
```

The default is the stereo set without GNSS in the dark location with the
carried light; `CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar` flies
the 3D lidar. The scenarios and their make targets are in
[`docs/scenarios.md`](docs/scenarios.md), the switches in
[`docs/configuration.md`](docs/configuration.md). The run's artifacts land
in `log/runs/<run>/`; the target ends with the mission check, whose lines
(`OK`, `NOTE`, `FAIL`) are the flight's result.

The rules of a flight (specification A2 to A4, A8):

- strictly one at a time, `./scripts/stop_sim.sh` before and after each, also
  after an interrupted one;
- never beside foreign load on the host; foreign processes are left alone,
  the flight waits;
- no edit of the tracked tree while a flight runs: the run's manifest binds
  the commit;
- no cooperative flights until roadmap item 15 closes;
- `SMOKE_DURATION_S` is the harness's wall-clock window only (600 to 900 s
  for an ordinary flight); a return home is tested with a short battery, not
  by waiting an hour.

`tools/series2.sh` keeps these rules by itself and is the ordinary way to
fly; everything the flights are launched and read with is in
[`tools/README.md`](tools/README.md).

## Accepting A Change

A change is accepted by a series on one commit (specification A1):

```bash
./tools/series2.sh r1100 r1101 r1102 r1103 r1104           # stereo set
./tools/series2.sh --lidar r1105 r1106 r1107 r1108 r1109   # 3D lidar, the flag first
```

Five flights on the stereo set, then five on the 3D lidar, both without
GNSS, each inspected before the next (`./tools/mxs.sh r1100`). A commit
between flights restarts the series. A change that touches a scenario of
its own (a return home, a light failure) also flies that scenario:
`TARGET=sim-urban-light-lost-headless ./tools/series2.sh r1110`. The series
table is written to `log/tools/journal/series_<commit>_<profile>.txt`.

**What a flight must show** (specification R1 to R4;
[`docs/testing.md`](docs/testing.md) has every line of the check):

| Requirement | Figure |
|---|---|
| The vehicle never crashes | no crash event and no contact with any surface; the one exception is a landing on the ground or a floor (A9) |
| The vehicle reaches its goal in truth | the true position inside the 2.0 m capture radius at every goal acknowledgement |
| Mean flight speed, simulation clock | above 1.2 m/s on the stereo set, above 2.4 m/s on the 3D lidar |

Nothing else fails a flight; every other measurement is a note. A scenario
that is meant to end otherwise (a return home, a landing with the light
lost) states its own result in [`docs/scenarios.md`](docs/scenarios.md).

**Whether a flight counts** (specification A6, A7) is decided by
measurement: the simulator's real-time factor at the median is at least 0.82
on the stereo set or 0.97 on the 3D lidar, and the resource sampler's record
has no gap of 2 s or more. `python3 tools/host_verdict.py r1100` prints
`HOST valid` or `HOST VOID`; the series appends it to the table. A void
flight is flown again and a crash in it is not a defect. A recorded flight
(`RECORD_VIDEO=1`, `tools/record_flight.sh`) is a demonstration and never an
acceptance flight (A11).

**A failure in a flight that counts** is repaired with its measured cause,
and the series is flown again from its first flight (R5). Read the flight
first: `tools/mx_inspect.py`, `tools/crash_context.py`,
`tools/body_truth_clearance.py`. A defect met on the way is repaired at
once; the register takes only what is (a) very hard, (b) in need of a deep
rework, or (c) in need of the owner's decision, and the entry says which.

## What A Change Records

| Where | What |
|---|---|
| [`docs/specification.md`](docs/specification.md) | a requirement, invariant, acceptance condition or key number added or changed: the entry and a change-log line with the justification, in the same commit |
| [`docs/technical_debt.md`](docs/technical_debt.md) | a defect found and set aside, with its class and the runs that show it; an entry repaired is removed |
| [`docs/roadmap.md`](docs/roadmap.md) | the status of the item worked on, with the commit and the runs of its acceptance |
| the document of the area under `docs/` | behaviour, parameters and figures that changed |
| `CHANGELOG.md` | releases only |

A run cited as evidence is cited by its name (`r1100`) and commit. Logs are
pruned after a week (`scripts/prune_sim_logs.sh`, before every flight):
anything that has to outlive it — a series table, a journal, a recording, a
run a document cites — goes into a directory with a `.keep` file at its top
(`log/tools/journal/`, `log/tools/diagnostics/`); a loose file cannot be
kept.

## Releases

Code releases are annotated tags `vMAJOR.MINOR.PATCH` on `main`; environment
assets keep their own `environment-assets-*` tags. To cut a release:

1. Fly the validation series on the release commit with nothing changed
   between flights (`tools/series2.sh` runs the urban point-to-point
   flights one at a time and records `gz_pose.csv` and `tracking.npz`); record
   the table in `CHANGELOG.md` with the commit, the flights, the mean speed,
   the crashes, the route availability and the planner p95.
2. In one commit: bump `<version>` in `drone_city_nav/package.xml`, add the
   `CHANGELOG.md` entry (validated scenario, laws in force, known
   limitations, compatible asset tags), and update the roadmap status.
3. Run the gate (`./tools/gates.sh`, see Before A Commit), commit, then `git tag -a vX.Y.Z -m "..."` on that commit and push `main`
   and the tag.
4. Publish the GitHub release from the tag with the changelog entry as its
   text. No build artifacts are attached: the code builds from source and the
   assets are released separately.

Every flight's runtime manifest records the package version and
`git describe`, so a run is attributable to a release by name.

## Scope Rules

- The requirements, invariants, acceptance conditions and key numbers are in
  [`docs/specification.md`](docs/specification.md); a change to one of them is
  logged there with its justification in the same commit.

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
