#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

normalize_bool() {
  local value="${1}"
  case "${value,,}" in
    1|true|yes|on)
      printf 'true'
      ;;
    0|false|no|off)
      printf 'false'
      ;;
    *)
      printf '%s' "${value}"
      ;;
  esac
}

clean_stale_processes_enabled="$(
  normalize_bool "${DRONE_GAZEBO_CLEAN_STALE_PROCESSES:-true}"
)"
clean_stale_processes_dry_run="$(
  normalize_bool "${DRONE_GAZEBO_CLEAN_STALE_DRY_RUN:-false}"
)"
container_stop_timeout_s="${DRONE_GAZEBO_CONTAINER_STOP_TIMEOUT_S:-10}"

for arg in "$@"; do
  case "${arg}" in
    --dry-run)
      clean_stale_processes_dry_run="true"
      ;;
    --help|-h)
      cat <<EOF
Usage: $0 [--dry-run]

Stops stale simulation containers and processes for this repository.

Environment:
  DRONE_GAZEBO_CLEAN_STALE_DRY_RUN       List candidates without stopping them.
  DRONE_GAZEBO_CLEAN_STALE_PROCESSES     Set false to disable cleanup.
  DRONE_GAZEBO_CONTAINER_STOP_TIMEOUT_S  docker stop timeout in seconds.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

if [[ "${clean_stale_processes_enabled}" != "true" &&
  "${clean_stale_processes_enabled}" != "1" ]]; then
  echo "WARNING: stale simulation process cleanup is disabled"
  exit 0
fi

bool_is_true() {
  [[ "$1" == "true" || "$1" == "1" ]]
}

container_has_simulation_processes() {
  local container_id="$1"
  local inspect_text
  local top_text
  local combined_text

  inspect_text="$(
    docker inspect \
      --format '{{json .Path}} {{json .Args}} {{json .Config.Cmd}}' \
      "${container_id}" 2>/dev/null || true
  )"
  top_text="$(docker top "${container_id}" -eo pid,ppid,pgid,cmd 2>/dev/null || true)"
  combined_text="${inspect_text}"$'\n'"${top_text}"

  grep -Eiq \
    'gz[[:space:]]+sim|MicroXRCEAgent|PX4-Autopilot|px4_sitl|ros2[[:space:]]+launch[[:space:]]+drone_city_nav[[:space:]]+city_nav\.launch\.py|run_drone_nav_sim\.sh|make[",[:space:]]+sim-(gui|headless)|sim-(gui|headless)|rviz2.*city_nav_debug(_top_down)?\.rviz|/drone_city_nav/(lidar_debug_node|mission_monitor_node|obstacle_memory_node|planner_node|px4_offboard_node)' \
    <<< "${combined_text}"
}

stop_stale_simulation_containers() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "Simulation container cleanup: docker not found, skipping"
    return 0
  fi

  local container_ids=()
  mapfile -t container_ids < <(
    docker ps --format '{{.ID}}' 2>/dev/null || true
  )

  local selected_ids=()
  local container_id
  for container_id in "${container_ids[@]}"; do
    if container_has_simulation_processes "${container_id}"; then
      selected_ids+=("${container_id}")
    fi
  done

  if [[ "${#selected_ids[@]}" -eq 0 ]]; then
    echo "Simulation container cleanup: no simulation containers found"
    return 0
  fi

  echo "Simulation container cleanup: candidates=${#selected_ids[@]} dry_run=${clean_stale_processes_dry_run}"
  for container_id in "${selected_ids[@]}"; do
    echo "Simulation container cleanup candidate: container=${container_id}"
  done

  if bool_is_true "${clean_stale_processes_dry_run}"; then
    return 0
  fi

  docker stop -t "${container_stop_timeout_s}" "${selected_ids[@]}"
}

stop_stale_simulation_containers

cleanup_args=(
  --self-pid "$$"
  --protect-pid "${BASHPID}"
  --repo-root "${repo_root}"
  --project-marker "/workspace"
)
if [[ "${clean_stale_processes_dry_run}" == "true" ||
  "${clean_stale_processes_dry_run}" == "1" ]]; then
  cleanup_args+=(--dry-run)
fi

python3 "${repo_root}/scripts/gazebo_process_cleanup.py" "${cleanup_args[@]}"
