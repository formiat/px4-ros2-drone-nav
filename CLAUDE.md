# Development Instructions

All development work in this repository must follow the code requirements and
guidelines described in [CPP_BEST_PRACTICES.md](CPP_BEST_PRACTICES.md).

Use the repository-approved workflow documented in [README.md](README.md) and
[CONTRIBUTING.md](CONTRIBUTING.md). This is a ROS 2 workspace; prefer `colcon`
through the provided `Makefile` and scripts, not ad-hoc top-level CMake
commands.

Use only the container workflow for build, test, quality checks, and simulation.
Use `./scripts/build.sh`, `./scripts/test.sh`, `./scripts/sim_headless.sh`, and
`./scripts/sim_gui.sh` for common workflows. Start `./scripts/dev_shell.sh`
only when an interactive container shell is needed; inside the container, use
`make build`, `make test`, `make test-scripts`, `make quality`,
`make sim-headless`, and `make sim-gui`. Do not run workspace-writing scripts
as root unless doing intentional maintenance with `ALLOW_ROOT_WORKSPACE_WRITE=1`.

Before committing after file changes:

1. Format changed C++ files with `make format`.
2. Run `make quality`.
3. Commit the completed file changes.

Keep code comments and repository documentation in English.
