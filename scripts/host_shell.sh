#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
micromamba_bin="${MICROMAMBA_BIN:-${HOME}/.local/bin/micromamba}"
host_env_prefix="${DRONE_GAZEBO_HOST_ENV:-${HOME}/.local/share/mamba/envs/drone-gazebo-host}"

if [[ ! -x "${micromamba_bin}" ]]; then
  echo "micromamba was not found: ${micromamba_bin}" >&2
  exit 1
fi
if [[ ! -d "${host_env_prefix}" ]]; then
  echo "Host environment was not found: ${host_env_prefix}" >&2
  exit 1
fi

export ROS_SETUP_FILE="${ROS_SETUP_FILE:-${host_env_prefix}/setup.bash}"
export PX4_MSGS_SETUP_FILE="${PX4_MSGS_SETUP_FILE:-${repo_root}/external/px4_msgs_ws_host/install/setup.bash}"
export COLCON_BUILD_BASE="${COLCON_BUILD_BASE:-build-host}"
export COLCON_INSTALL_BASE="${COLCON_INSTALL_BASE:-install-host}"
export COLCON_LOG_BASE="${COLCON_LOG_BASE:-log-host}"
export DRONE_GAZEBO_LOG_DIR="${DRONE_GAZEBO_LOG_DIR:-${repo_root}/log-host}"

if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
  echo "ROS setup file was not found: ${ROS_SETUP_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PX4_MSGS_SETUP_FILE}" ]]; then
  echo "px4_msgs setup file was not found: ${PX4_MSGS_SETUP_FILE}" >&2
  exit 1
fi

if [[ "$#" -eq 0 ]]; then
  exec "${micromamba_bin}" run -p "${host_env_prefix}" bash -lc '
    source "${ROS_SETUP_FILE}"
    source "${PX4_MSGS_SETUP_FILE}"
    cd "'"${repo_root}"'"
    if [[ -f "${COLCON_INSTALL_BASE}/setup.bash" ]]; then
      source "${COLCON_INSTALL_BASE}/setup.bash"
    fi
    exec bash -i
  '
fi

exec "${micromamba_bin}" run -p "${host_env_prefix}" bash -lc '
  source "${ROS_SETUP_FILE}"
  source "${PX4_MSGS_SETUP_FILE}"
  cd "'"${repo_root}"'"
  if [[ -f "${COLCON_INSTALL_BASE}/setup.bash" ]]; then
    source "${COLCON_INSTALL_BASE}/setup.bash"
  fi
  "$@"
' bash "$@"
