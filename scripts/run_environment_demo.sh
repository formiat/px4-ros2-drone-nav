#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
environment_id="${1:-${ENVIRONMENT_DEMO_ID:-}}"

if [[ -z "${environment_id}" ]]; then
  cat >&2 <<'EOF'
ENVIRONMENT_DEMO_ID is required.

Example:
  ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh
EOF
  exit 2
fi

python3 "${repo_root}/scripts/prepare_environment_demo.py" \
  --environment "${environment_id}"
environment_file="${repo_root}/external/environment-artifacts/derived/${environment_id}/demo/environment.env"

if [[ ! -f "${environment_file}" ]]; then
  echo "Demo environment file was not created: ${environment_file}" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${environment_file}"
if [[ -n "${SIM_DEMO_RESOURCE_PATH:-}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${SIM_DEMO_RESOURCE_PATH}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
fi

camera_log_interval_s="${GZ_GUI_CAMERA_LOG_INTERVAL_S:-1}"
camera_log_file="${GZ_GUI_CAMERA_LOG_FILE:-${repo_root}/log/environment_demo/${environment_id}/gz_gui_free_camera.jsonl}"
mkdir -p "$(dirname "${camera_log_file}")"
python3 "${repo_root}/scripts/gazebo_gui_camera_logger.py" \
  --output "${camera_log_file}" \
  --interval-s "${camera_log_interval_s}" &
camera_logger_pid=$!

cleanup_camera_logger() {
  kill "${camera_logger_pid}" 2>/dev/null || true
  wait "${camera_logger_pid}" 2>/dev/null || true
}
trap cleanup_camera_logger EXIT INT TERM

echo "Launching Gazebo spectator demo: environment=${environment_id} world=${SIM_DEMO_WORLD_NAME}"
echo "Gazebo free-camera log: ${camera_log_file} (interval=${camera_log_interval_s}s)"
gz sim -v "${GZ_VERBOSE:-3}" "${SIM_DEMO_WORLD_SDF_PATH}"
