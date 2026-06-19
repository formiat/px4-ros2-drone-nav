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

if [[ "${clean_stale_processes_enabled}" != "true" &&
  "${clean_stale_processes_enabled}" != "1" ]]; then
  echo "WARNING: stale simulation process cleanup is disabled"
  exit 0
fi

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
