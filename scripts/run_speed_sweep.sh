#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_params="${CITY_NAV_SWEEP_BASE_PARAMS:-${repo_root}/drone_city_nav/config/urban_mvp.yaml}"
sweep_dir="${CITY_NAV_SWEEP_DIR:-${repo_root}/build/speed_sweep}"
smoke_duration_s="${SMOKE_DURATION_S:-300}"

if [[ ! -f "${base_params}" ]]; then
  echo "Base params file was not found: ${base_params}" >&2
  exit 1
fi

if [[ "$#" -eq 0 ]]; then
  set -- 3 5 7
fi

mkdir -p "${sweep_dir}"

write_speed_params() {
  local speed="$1"
  local output_file="$2"
  awk -v speed="${speed}" '
    /^px4_offboard_node:/ {
      in_offboard = 1
      print
      next
    }
    in_offboard && /^[^[:space:]]/ {
      in_offboard = 0
    }
    in_offboard && /^[[:space:]]+desired_speed_mps:/ {
      print "    desired_speed_mps: " speed
      found = 1
      next
    }
    {
      print
    }
    END {
      if (!found) {
        exit 2
      }
    }
  ' "${base_params}" > "${output_file}"
}

for speed in "$@"; do
  speed_label="${speed//./_}"
  params_file="${sweep_dir}/urban_mvp_speed_${speed_label}.yaml"
  run_log_dir="${repo_root}/log/speed_sweep_${speed_label}"
  mkdir -p "${run_log_dir}"
  write_speed_params "${speed}" "${params_file}"

  echo "RUN: desired_speed_mps=${speed} params=${params_file}"
  env \
    CITY_NAV_PARAMS_FILE="${params_file}" \
    HEADLESS=1 \
    MISSION_CHECK=1 \
    SMOKE_DURATION_S="${smoke_duration_s}" \
    PX4_LOG_FILE="${run_log_dir}/px4_city_mvp.log" \
    UXRCE_AGENT_LOG_FILE="${run_log_dir}/uxrce_agent_city_mvp.log" \
    ROS_LOG_FILE="${run_log_dir}/ros_city_mvp.log" \
    GZ_LOG_FILE="${run_log_dir}/gz_city_mvp.log" \
    LIDAR_DEBUG_DIR="${run_log_dir}/lidar_debug" \
    "${repo_root}/scripts/run_city_mvp.sh"
done
