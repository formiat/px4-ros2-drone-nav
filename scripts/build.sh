#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DRONE_GAZEBO_CONTAINER_GPU="${DRONE_GAZEBO_CONTAINER_GPU:-off}"
exec "${repo_root}/scripts/container_run.sh" make build
