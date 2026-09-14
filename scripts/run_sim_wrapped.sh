#!/usr/bin/env bash
# Runs one simulation make target in the dev container between two cleanups:
# whatever a previous run left behind is stopped first, and whatever this run
# leaves behind (an interrupted launch, a container that outlived its
# simulator) is stopped once it ends, however it ends.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 make <sim-target> [...]" >&2
  exit 2
fi

"${repo_root}/scripts/prune_sim_logs.sh"
"${repo_root}/scripts/cleanup_sim_processes.sh"
trap '"${repo_root}/scripts/cleanup_sim_processes.sh" || true' EXIT
"${repo_root}/scripts/container_run.sh" "$@"
