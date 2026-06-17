#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec "${repo_root}/scripts/host_shell.sh" "${repo_root}/scripts/run_speed_sweep.sh" "$@"
