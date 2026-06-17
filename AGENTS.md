# Development Instructions

All development work in this repository must follow the code requirements and
guidelines described in [CPP_BEST_PRACTICES.md](CPP_BEST_PRACTICES.md).

Use the repository-approved workflow documented in [README.md](README.md) and
[CONTRIBUTING.md](CONTRIBUTING.md). This is a ROS 2 workspace; prefer `colcon`
through the provided `Makefile` and scripts, not ad-hoc top-level CMake
commands.

Prefer the native host workflow for routine agent work. From the repository
root, use `make host-build`, `make host-test`, `make host-quality`,
`make host-sim-headless`, and `make host-sim-gui`. These targets run through
`./scripts/host_shell.sh` and keep generated files under `build-host/`,
`install-host/`, and `log-host/`.

Use `./scripts/dev_shell.sh` only for container-specific work or when validating
container compatibility. It runs the dev container with the host UID/GID so
generated files remain owned by the host user. Inside the container, use
`make build`, `make test`, `make quality`, `make sim-headless`, and
`make sim-gui`. Do not run workspace-writing scripts as root unless doing
intentional maintenance with `ALLOW_ROOT_WORKSPACE_WRITE=1`.

Before committing after file changes:

1. Format changed C++ files with `make host-format`.
2. Run `make host-quality`.
3. Commit the completed file changes.

Keep code comments and repository documentation in English.
