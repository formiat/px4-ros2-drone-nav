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

# The three subsystem CPU lists (control, planning, diagnostics) from the CPUs
# this process may run on: the first half, from the first quarter to the end,
# and the last quarter of the allowed list, the proportions of the masks on a
# whole 16-CPU host (0-7, 4-15, 12-15). A flight confined to a core set
# (FLIGHT_CPUSET, the container's cpuset, specification A4) has fewer CPUs and
# other numbers than 0 to nproc - 1: a mask built from nproc would name CPUs
# the container has not got and keep the intersection only. Prints nothing
# under four CPUs.
derive_subsystem_cpu_lists() {
  local allowed part i cpus=()
  allowed="$(taskset -pc $$ 2>/dev/null | sed 's/.*: //')"
  for part in ${allowed//,/ }; do
    if [[ "${part}" == *-* ]]; then
      for ((i = ${part%-*}; i <= ${part#*-}; i++)); do cpus+=("${i}"); done
    else
      cpus+=("${part}")
    fi
  done
  local n="${#cpus[@]}"
  ((n >= 4)) || return 0
  local IFS=,
  printf '%s %s %s\n' "${cpus[*]:0:$((n / 2))}" "${cpus[*]:$((n / 4))}" "${cpus[*]:$((3 * n / 4))}"
}
