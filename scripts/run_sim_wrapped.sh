#!/usr/bin/env bash
# Runs one simulation make target in the dev container between two cleanups:
# whatever a previous run left behind is stopped first, and whatever this run
# leaves behind (an interrupted launch, a container that outlived its
# simulator) is stopped once it ends, however it ends. It never waits for the
# host: the user's run starts at once.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 make <sim-target> [...]" >&2
  exit 2
fi

"${repo_root}/scripts/prune_sim_logs.sh"
"${repo_root}/scripts/cleanup_sim_processes.sh"
# No wait for a quiet host here (the owner, 2026-10-30): a run the user starts
# starts at once, whatever the host is doing. The quiet-host gate of the
# acceptance series (A4) lives in tools/series2.sh, which the series tools call.
trap '"${repo_root}/scripts/cleanup_sim_processes.sh" || true' EXIT
"${repo_root}/scripts/container_run.sh" "$@"
