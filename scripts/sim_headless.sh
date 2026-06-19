#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${repo_root}/scripts/cleanup_sim_processes.sh"
exec "${repo_root}/scripts/container_run.sh" make sim-headless
