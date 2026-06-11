#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run --rm -it \
  --privileged \
  --network host \
  --env DISPLAY="${DISPLAY:-}" \
  --env LOCAL_USER_ID="$(id -u)" \
  --volume "${repo_root}:/workspace:rw" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --workdir /workspace \
  drone-gazebo-dev:latest \
  bash -l
