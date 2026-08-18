#!/usr/bin/env bash

make_abs_path() {
  local path="$1"
  case "${path}" in
    /*) printf '%s\n' "${path}" ;;
    *) printf '%s/%s\n' "${repo_root}" "${path}" ;;
  esac
}

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

bool_is_true() {
  [[ "$1" == "true" || "$1" == "1" ]]
}

format_mission_goal_sequence() {
  local sequence="$1"
  python3 - "${sequence}" <<'PY'
import math
import sys

raw = sys.argv[1].strip()
if not raw:
    raise SystemExit("MISSION_GOALS_XYZ_M must not be empty when provided")
values = []
for waypoint_index, waypoint in enumerate(raw.split(";")):
    components = [component.strip() for component in waypoint.split(",")]
    if len(components) != 3 or any(not component for component in components):
        raise SystemExit(
            "MISSION_GOALS_XYZ_M must use x,y,z;x,y,z syntax; "
            f"invalid waypoint {waypoint_index}"
        )
    try:
        numeric = [float(component) for component in components]
    except ValueError as error:
        raise SystemExit(
            f"MISSION_GOALS_XYZ_M contains a non-numeric waypoint {waypoint_index}"
        ) from error
    if not all(math.isfinite(component) for component in numeric):
        raise SystemExit(
            f"MISSION_GOALS_XYZ_M contains a non-finite waypoint {waypoint_index}"
        )
    values.extend(numeric)
print("[" + ", ".join(f"{value:.12g}" for value in values) + "]")
PY
}

run_with_cpu_affinity() {
  local cpu_list="$1"
  shift
  if [[ -n "${cpu_list}" ]]; then
    taskset --cpu-list "${cpu_list}" "$@"
  else
    "$@"
  fi
}

exec_with_cpu_affinity() {
  local cpu_list="$1"
  shift
  if [[ -n "${cpu_list}" ]]; then
    exec taskset --cpu-list "${cpu_list}" "$@"
  else
    exec "$@"
  fi
}
