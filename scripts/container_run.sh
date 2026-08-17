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

gpu_args=()
if command -v nvidia-smi >/dev/null 2>&1 &&
    docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  gpu_args+=(--gpus all)
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
  COOPERATIVE_DESIRED_MINIMUM_SEPARATION_M
  COOPERATIVE_MISSION_TIMEOUT_S
  COOPERATIVE_PREDICTION_HORIZON_S
  COOPERATIVE_RELEASE_SEPARATION_M
  DRONE_GAZEBO_CLEAN_STALE_DRY_RUN
  DRONE_GAZEBO_CLEAN_STALE_PROCESSES
  DRONE_GAZEBO_LOG_DIR
  ENABLE_GAZEBO_BRIDGE
  ENABLE_GZ_GUI_FOLLOW_CAMERA
  ENABLE_GZ_SCENE_DIAGNOSTICS
  ENABLE_2D_LIDAR
  ENABLE_LIDAR_DEBUG
  ENABLE_MISSION_MONITOR
  ENABLE_RVIZ
  ENABLE_RVIZ_FOLLOW_CAMERA
  ENABLE_STATIC_MAP
  ENVIRONMENT_DEMO_ID
  EVADER_PX4_LOG_FILE
  EVADER_SPEED_SCALE
  GZ_GUI_FOLLOW_OFFSET
  GZ_GUI_FOLLOW_TARGET
  GZ_GUI_FOLLOW_WAIT_S
  GZ_GUI_LOG_FILE
  GZ_LOG_FILE
  GZ_SCENE_DIAGNOSTICS_DIR
  GZ_SPECTATOR_LOG_FILE
  GZ_VERBOSE
  GZ_WORLD_UNPAUSE_WAIT_S
  HEADLESS
  INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED
  INTERCEPT_NONCOOPERATIVE_AVOIDANCE_ENABLED
  STATIC_GLOBAL_LATTICE_DEADLINE_MS
  STATIC_ROUTE_TRACKING_MARGIN_M
  STATIC_CRUISE_SPEED_MPS
  STATIC_ABSOLUTE_SPEED_LIMIT_MPS
  INTERCEPT_SCENARIO_PATH
  INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID
  INTERCEPT_SPECTATOR_RESELECTION_DELAY_S
  INTERCEPT_SPECTATOR_RESELECTION_POLICY
  INTERCEPTOR_1_PX4_LOG_FILE
  INTERCEPTOR_2_PX4_LOG_FILE
  LIDAR_DEBUG_DIR
  MISSION_CHECK
  MISSION_TYPE
  MULTI_VEHICLE_SCENARIO_PATH
  MULTI_VEHICLE_SHUTDOWN_ON_TERMINAL_OUTCOME
  MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID
  MULTI_VEHICLE_SPECTATOR_RESELECTION_DELAY_S
  MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY
  PX4_AUTOPILOT_DIR
  PX4_LOG_FILE
  PX4_MODEL_TARGET
  PX4_PARAM_DELAY_S
  PX4_MSGS_SETUP_FILE
  ROS_SETUP_FILE
  ROS_LOG_FILE
  RVIZ_CONFIG_FILE
  SIM_START_X_M
  SIM_START_YAW_RAD
  SIM_START_Y_M
  SIM_START_Z_M
  SIM_WORLD_NAME
  SIM_WORLD_RESOURCE_PATH
  SIM_WORLD_SDF_PATH
  SIM_COLLISION_WORLD_SDF_PATH
  SIM_GUI_WORLD_SDF_PATH
  SMOKE_DURATION_S
  STATIC_CITY_MAP_PATH
  STATIC_ESDF_3D_CACHE_PATH
  STATIC_FREE_SPACE_TOPOLOGY_3D_PATH
  STATIC_OCCUPANCY_3D_PATH
  STARTUP_SLEEP_S
  UXRCE_AGENT_LOG_FILE
)

for env_name in "${optional_env_vars[@]}"; do
  if [[ -v "${env_name}" ]]; then
    env_args+=(--env "${env_name}")
  fi
done

container_command='
set -euo pipefail

source_setup_file() {
  local setup_file="$1"
  if [[ ! -f "${setup_file}" ]]; then
    return 0
  fi

  set +u
  source "${setup_file}"
  set -u
}

mkdir -p "${HOME}" "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"
export PATH="/usr/local/cuda/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

source_setup_file "${ROS_SETUP_FILE:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
source_setup_file "${PX4_MSGS_SETUP_FILE:-/opt/px4_msgs_ws/install/setup.bash}"

if [[ "$#" -eq 0 ]]; then
  exec bash -i
fi

exec "$@"
'

docker run --rm "${tty_args[@]}" \
  --privileged \
  --network host \
  --user "${user_uid}:${user_gid}" \
  "${gpu_args[@]}" \
  "${group_args[@]}" \
  "${env_args[@]}" \
  --volume "${repo_root}:/workspace:rw" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --workdir /workspace \
  "${image_name}" \
  bash -c "${container_command}" bash "$@"
