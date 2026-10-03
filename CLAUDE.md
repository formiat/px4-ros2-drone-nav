# Development Instructions

All development work in this repository must follow the code requirements and
guidelines described in [CPP_BEST_PRACTICES.md](CPP_BEST_PRACTICES.md).

Use the repository-approved workflow documented in [README.md](README.md) and
[CONTRIBUTING.md](CONTRIBUTING.md). This is a ROS 2 workspace; prefer `colcon`
through the provided `Makefile` and scripts, not ad-hoc top-level CMake
commands.

Use only the container workflow for build, test, quality checks, and simulation.
Use `./scripts/build.sh`, `./scripts/test.sh`,
`./scripts/sim_urban_point_to_point_headless.sh`, and
`./scripts/sim_urban_point_to_point_gui.sh` for common workflows. Start `./scripts/dev_shell.sh`
only when an interactive container shell is needed; inside the container, use
`make build`, `make test`, `make test-scripts`, `make quality`,
`make sim-urban-point-to-point-headless`, and
`make sim-urban-point-to-point-gui`. Do not run workspace-writing scripts
as root unless doing intentional maintenance with `ALLOW_ROOT_WORKSPACE_WRITE=1`.

Before committing after file changes:

1. Format changed C++ files with `make format`.
2. Run `make quality`.
3. Commit the completed file changes.

Keep code comments and repository documentation in English.

Logs are pruned after a week (`scripts/prune_sim_logs.sh`: `log/`,
`log/runs/`, `log/tools/` and the PX4 logs), before every simulation run.
Anything worth more than a week — a recording, a journal, a series table, a
run cited as evidence — goes into a directory that holds a `.keep` file at
its top, which is never deleted; a loose file cannot be kept
([tools/README.md](tools/README.md)).

The project's requirements, navigation invariants, acceptance conditions and
the numbers the work relies on are in [docs/specification.md](docs/specification.md),
each marked as set by the owner or by an agent. Read it before planning work
on navigation, acceptance or a roadmap item; a change to any of its entries
is recorded in its change log in the same commit, with the justification,
and reported to the owner.
