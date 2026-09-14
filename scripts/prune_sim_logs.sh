#!/usr/bin/env bash
# Deletes simulation logs older than a week, from the places the stack writes
# them and nowhere else:
#
#   log/<entry>                 every entry directly under log/ (a run set, an
#                               experiment, a colcon build log) except log/tools
#   log/runs/<id>               every run directory
#   external/PX4-Autopilot/build/px4_sitl_default/rootfs/<n>/log/<entry>
#                               PX4 flight logs of every instance
#
# An entry goes when nothing inside it was modified within the window. An entry
# holding a .keep file at its top level is never deleted; log/tools never is.
# Symbolic links are left alone. Nothing outside the repository is ever touched.
#
#   ./scripts/prune_sim_logs.sh [--dry-run] [--older-than-days N] [--root DIR]
#
# Environment: DRONE_GAZEBO_PRUNE_LOGS=false disables the pruning (the sim
# wrappers call this before every run); DRONE_GAZEBO_PRUNE_LOGS_DAYS sets the
# window, 7 by default.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dry_run="false"
days="${DRONE_GAZEBO_PRUNE_LOGS_DAYS:-7}"

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --dry-run) dry_run="true" ;;
    --older-than-days)
      shift
      days="${1:-}"
      ;;
    --root)
      shift
      root="$(cd "${1:-}" && pwd)"
      ;;
    -h | --help)
      sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

case "${DRONE_GAZEBO_PRUNE_LOGS:-true}" in
  0 | false | no | off)
    echo "Simulation log pruning: disabled"
    exit 0
    ;;
esac
if ! [[ "${days}" =~ ^[0-9]+$ ]] || [[ "${days}" -lt 1 ]]; then
  echo "The window must be a whole number of days, at least 1: '${days}'" >&2
  exit 2
fi

px4_root="${PX4_AUTOPILOT_DIR:-${root}/external/PX4-Autopilot}"
parents=("${root}/log" "${root}/log/runs")
for instance_log in "${px4_root}"/build/px4_sitl_default/rootfs/*/log; do
  [[ -d "${instance_log}" ]] && parents+=("${instance_log}")
done

# True when the entry is a real directory or file inside the repository whose
# contents were all last modified before the window, with no .keep on top.
prunable() {
  local entry="$1"
  local resolved
  [[ -L "${entry}" ]] && return 1
  [[ -e "${entry}" ]] || return 1
  resolved="$(realpath -e "${entry}")"
  [[ "${resolved}" == "${root}/"* ]] || return 1
  [[ "${resolved}" == "${root}/log/tools" || "${resolved}" == "${root}/log/runs" ]] && return 1
  [[ -e "${entry}/.keep" ]] && return 1
  # Any file or directory modified within the window keeps the whole entry.
  [[ -z "$(find "${entry}" -mtime "-${days}" -print -quit 2>/dev/null)" ]]
}

deleted=0
kept=0
bytes_kib=0
for parent in "${parents[@]}"; do
  [[ -d "${parent}" ]] || continue
  for entry in "${parent}"/* "${parent}"/.[!.]*; do
    [[ -e "${entry}" || -L "${entry}" ]] || continue
    if ! prunable "${entry}"; then
      kept=$((kept + 1))
      continue
    fi
    size_kib="$(du -sk "${entry}" 2>/dev/null | cut -f1)"
    bytes_kib=$((bytes_kib + ${size_kib:-0}))
    deleted=$((deleted + 1))
    if [[ "${dry_run}" == "true" ]]; then
      echo "would delete ${entry#"${root}"/} (${size_kib:-0} KiB)"
    else
      rm -rf -- "${entry}"
    fi
  done
done

verb="deleted"
[[ "${dry_run}" == "true" ]] && verb="would delete"
printf 'Simulation log pruning: %s %d entries older than %d days (%.1f GB), kept %d\n' \
  "${verb}" "${deleted}" "${days}" "$(awk "BEGIN {print ${bytes_kib}/1048576}")" "${kept}"
