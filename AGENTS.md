# Development Instructions

All development work in this repository must follow the code requirements and
guidelines described in [CPP_BEST_PRACTICES.md](CPP_BEST_PRACTICES.md).

Use the repository-approved workflow documented in [README.md](README.md) and
[CONTRIBUTING.md](CONTRIBUTING.md). This is a ROS 2 workspace; prefer `colcon`
through the provided `Makefile` and scripts, not ad-hoc top-level CMake
commands.

Use `./scripts/dev_shell.sh` for container work. It runs the dev container with
the host UID/GID so generated files remain owned by the host user. Do not run
workspace-writing scripts as root unless doing intentional maintenance with
`ALLOW_ROOT_WORKSPACE_WRITE=1`.

Before committing after file changes:

1. Format changed C++ files with `./scripts/format_cpp_changed.sh`.
2. Run `./scripts/check_cpp_quality.sh`.
3. Commit the completed file changes.

Use Russian for assistant-user communication. Keep code comments and repository
documentation in English.
