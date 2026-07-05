# Installation And Host Requirements

The supported workflow is container-based. The host provides Docker, display
access for GUI runs, and access to the repository files. ROS 2, Gazebo, PX4
messages, and build tools are provided by the project container image.

## Supported Host

The repository is developed for a Linux host. The scripts assume:

- a POSIX shell environment;
- Docker with permission to run containers;
- X11 or Wayland/XWayland support for GUI runs;
- a local checkout with normal user ownership.

The project may work on other host distributions, but the documented path is
the repository Docker workflow.

## Required Host Packages

Install at least:

- `docker` or Docker Engine from your distribution/vendor;
- `git`;
- a shell with standard GNU/Linux command-line tools.

For GUI simulation, the host must allow containerized GUI applications to reach
the display server. The scripts handle the common X11 case; Wayland setups
usually need XWayland compatibility.

## Docker Access

The user running the scripts must be able to run Docker. Usually this means the
user is in the `docker` group:

```bash
docker ps
```

If this fails with a permission error, fix host Docker permissions before using
the project scripts.

## What Not To Install On The Host

Do not install or depend on host ROS 2, host Gazebo, or host PX4 packages for
normal development. The dev image provides the supported environment.

Avoid mixing host and container build artifacts. The repository scripts run the
container with the current UID/GID so generated files remain owned by the host
user.

## Build The Dev Image

If the image is missing or needs to be refreshed:

```bash
./scripts/build_dev_image.sh
```

Common workflow scripts also use the container entrypoint and can build or use
the configured image depending on the local setup.

## First Sanity Check

From the repository root:

```bash
./scripts/build.sh
./scripts/test.sh
```

For a simulator sanity check:

```bash
./scripts/sim_headless.sh
```

For a GUI sanity check:

```bash
./scripts/sim_gui.sh
```

If GUI startup fails, first check Docker display permissions and see
`troubleshooting.md`.
