#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
user_uid="$(id -u)"
user_gid="$(id -g)"
container_home="/tmp/drone-gazebo-home-${user_uid}"
container_runtime="/tmp/drone-gazebo-runtime-${user_uid}"
image_name="${DRONE_GAZEBO_DEV_IMAGE:-drone-gazebo-dev:latest}"

group_args=()
if getent group render >/dev/null; then
  group_args+=(--group-add "$(getent group render | cut -d: -f3)")
fi
if getent group video >/dev/null; then
  group_args+=(--group-add "$(getent group video | cut -d: -f3)")
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

env_args=(
  --env DISPLAY="${DISPLAY:-}"
  --env HOME="${container_home}"
  --env XDG_RUNTIME_DIR="${container_runtime}"
)

optional_env_vars=(
  ALLOW_MISSION_FAILURE
  CITY_NAV_PARAMS_FILE
  DRONE_GAZEBO_CLEAN_STALE_DRY_RUN
  DRONE_GAZEBO_CLEAN_STALE_PROCESSES
  DRONE_GAZEBO_LOG_DIR
  ENABLE_CURRENT_LIDAR
  ENABLE_EVASIVE_MANEUVERING
  ENABLE_GAZEBO_BRIDGE
  ENABLE_GZ_GUI_FOLLOW_CAMERA
  ENABLE_GZ_SCENE_DIAGNOSTICS
  ENABLE_LIDAR_DEBUG
  ENABLE_MISSION_MONITOR
  ENABLE_OBSTACLE_MEMORY
  ENABLE_RVIZ
  ENABLE_STATIC_MAP
  EVASIVE_MANEUVERING_STRAIGHT_COST_WEIGHT
  GZ_GUI_FOLLOW_OFFSET
  GZ_GUI_FOLLOW_TARGET
  GZ_GUI_FOLLOW_WAIT_S
  GZ_GUI_LOG_FILE
  GZ_LOG_FILE
  GZ_SCENE_DIAGNOSTICS_DIR
  GZ_VERBOSE
  GZ_WORLD_UNPAUSE_WAIT_S
  HEADLESS
  LIDAR_DEBUG_DIR
  MISSION_CHECK
  PX4_AUTOPILOT_DIR
  PX4_LOG_FILE
  PX4_MODEL_TARGET
  PX4_PARAM_DELAY_S
  ROS_LOG_FILE
  SIM_START_X_M
  SIM_START_YAW_RAD
  SIM_START_Y_M
  SIM_START_Z_M
  SMOKE_DURATION_S
  STATIC_CITY_MAP_PATH
  STARTUP_SLEEP_S
  UXRCE_AGENT_LOG_FILE
)

for env_name in "${optional_env_vars[@]}"; do
  if [[ -v "${env_name}" ]]; then
    env_args+=(--env "${env_name}")
  fi
done

if [[ "$#" -eq 0 ]]; then
  container_command='mkdir -p "${HOME}" "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" && exec bash -l'
else
  container_command='mkdir -p "${HOME}" "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" && exec "$@"'
fi

docker run --rm "${tty_args[@]}" \
  --privileged \
  --network host \
  --user "${user_uid}:${user_gid}" \
  "${group_args[@]}" \
  "${env_args[@]}" \
  --volume "${repo_root}:/workspace:rw" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --workdir /workspace \
  "${image_name}" \
  bash -lc "${container_command}" bash "$@"
