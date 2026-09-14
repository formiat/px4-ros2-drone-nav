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
container_stop_timeout_s="${DRONE_GAZEBO_CONTAINER_STOP_TIMEOUT_S:-2}"
dev_image_name="${DRONE_GAZEBO_DEV_IMAGE:-drone-gazebo-dev:latest}"
docker_command_timeout_s="${DRONE_GAZEBO_DOCKER_COMMAND_TIMEOUT_S:-4}"

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
  DRONE_GAZEBO_DOCKER_COMMAND_TIMEOUT_S  hard timeout for Docker commands.
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

# A container of this repository: it runs the dev image or mounts the
# repository as its workspace. Only such containers are ever stopped, whatever
# their command says.
container_belongs_to_repository() {
  local container_id="$1"
  local identity_text
  identity_text="$(
    timeout "${docker_command_timeout_s}s" docker inspect \
      --format '{{.Config.Image}} {{range .Mounts}}{{.Source}} {{end}}' \
      "${container_id}" 2>/dev/null || true
  )"
  [[ "${identity_text}" == *"${dev_image_name}"* ||
    "${identity_text}" == *"${repo_root} "* ]]
}

container_has_simulation_processes() {
  local container_id="$1"
  local inspect_text
  local top_text
  local combined_text

  if ! container_belongs_to_repository "${container_id}"; then
    return 1
  fi
  inspect_text="$(
    timeout "${docker_command_timeout_s}s" docker inspect \
      --format '{{json .Path}} {{json .Args}} {{json .Config.Cmd}}' \
      "${container_id}" 2>/dev/null || true
  )"
  top_text="$(
    timeout "${docker_command_timeout_s}s" \
      docker top "${container_id}" -eo pid,ppid,pgid,cmd 2>/dev/null || true
  )"
  combined_text="${inspect_text}"$'\n'"${top_text}"

  # Every Makefile simulation target is a sim-* target (sim-gui,
  # sim-urban-point-to-point-headless, sim-cooperative-traffic-urban-gui, ...).
  # The former pattern named only sim-gui and sim-headless, and a container
  # left behind by make sim-urban-point-to-point-gui survived every stop_sim.
  grep -Eiq \
    'gz[[:space:]]+sim|MicroXRCEAgent|PX4-Autopilot|px4_sitl|ros2[[:space:]]+launch[[:space:]]+drone_city_nav[[:space:]]+city_nav\.launch\.py|run_drone_nav_sim\.sh|run_environment_demo\.sh|make[",[:space:]]+sim-[a-z0-9-]+|"sim-[a-z0-9-]+"|rviz2.*city_nav_debug(_top_down)?\.rviz|/drone_city_nav/(collision_crash_node|lidar_debug_node|mission_monitor_node|obstacle_memory_node|production_mppi_node|mppi_offboard_node)' \
    <<< "${combined_text}"
}

stop_stale_simulation_containers() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "Simulation container cleanup: docker not found, skipping"
    return 0
  fi

  local container_ids=()
  mapfile -t container_ids < <(
    timeout "${docker_command_timeout_s}s" \
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

  if ! timeout "$((container_stop_timeout_s + docker_command_timeout_s))s" \
    docker stop -t "${container_stop_timeout_s}" "${selected_ids[@]}"; then
    echo "WARNING: graceful container stop timed out; forcing kill" >&2
    local selected_id
    for selected_id in "${selected_ids[@]}"; do
      timeout "${docker_command_timeout_s}s" \
        docker kill "${selected_id}" >/dev/null 2>&1 || true
    done
  fi
  # The wrappers run containers with --rm; a container that had to be stopped
  # from outside may still leave a dead record behind.
  timeout "${docker_command_timeout_s}s" \
    docker rm -f "${selected_ids[@]}" >/dev/null 2>&1 || true
}

stop_stale_simulation_containers

cleanup_args=(
  --self-pid "$$"
  --protect-pid "${BASHPID}"
  --repo-root "${repo_root}"
  --project-marker "/workspace"
  --term-timeout-s 2
)
if [[ "${clean_stale_processes_dry_run}" == "true" ||
  "${clean_stale_processes_dry_run}" == "1" ]]; then
  cleanup_args+=(--dry-run)
fi

python3 "${repo_root}/scripts/gazebo_process_cleanup.py" "${cleanup_args[@]}"
