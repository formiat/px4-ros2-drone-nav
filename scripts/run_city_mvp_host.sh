#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export COLCON_BUILD_BASE="${COLCON_BUILD_BASE:-build-host}"
export COLCON_INSTALL_BASE="${COLCON_INSTALL_BASE:-install-host}"
export COLCON_LOG_BASE="${COLCON_LOG_BASE:-log-host}"
export DRONE_GAZEBO_LOG_DIR="${DRONE_GAZEBO_LOG_DIR:-${repo_root}/log-host}"

exec "${repo_root}/scripts/host_shell.sh" "${repo_root}/scripts/run_city_mvp.sh" "$@"
