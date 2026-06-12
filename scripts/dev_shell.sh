#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
host_uid="$(id -u)"
host_gid="$(id -g)"
container_home="/tmp/drone-gazebo-home-${host_uid}"
container_runtime="/tmp/drone-gazebo-runtime-${host_uid}"

group_args=()
if getent group render >/dev/null; then
  group_args+=(--group-add "$(getent group render | cut -d: -f3)")
fi
if getent group video >/dev/null; then
  group_args+=(--group-add "$(getent group video | cut -d: -f3)")
fi

docker run --rm -it \
  --privileged \
  --network host \
  --user "${host_uid}:${host_gid}" \
  "${group_args[@]}" \
  --env DISPLAY="${DISPLAY:-}" \
  --env HOME="${container_home}" \
  --env XDG_RUNTIME_DIR="${container_runtime}" \
  --volume "${repo_root}:/workspace:rw" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --workdir /workspace \
  drone-gazebo-dev:latest \
  bash -lc 'mkdir -p "${HOME}" "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" && exec bash -l'
