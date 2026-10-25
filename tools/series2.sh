#!/usr/bin/env bash
# series2.sh [--lidar] rNNN [rNNN...]: acceptance flights one at a time on the current commit.
# Before every flight the host must be quiet for 60 s: no busy rustc/clippy/cargo (> 5 % CPU), 1-min load < 3,
# no gz sim of ours. Results go to log/tools/journal/series_<commit>_<profile>.txt (appended, never truncated).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
./scripts/prune_sim_logs.sh
profile=cameras; envs=()
if [ "${1:-}" = "--lidar" ]; then profile=lidar; envs=(CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar); shift; fi
mkdir -p log/tools/journal; touch log/tools/journal/.keep
out="log/tools/journal/series_$(git rev-parse --short HEAD)_${profile}.txt"
# The flight's core set (specification A4): CPUs 0-11 of this host, six physical cores with both SMT siblings each,
# unless the caller sets FLIGHT_CPUSET (empty: the whole host, as before 2026-10-24). The quiet-host gate then reads
# the set's own CPUs, not the host's load.
export FLIGHT_CPUSET="${FLIGHT_CPUSET-0-11}"
for id in "$@"; do
  ./scripts/quiet_host_gate.sh 2>/dev/null
  ./scripts/stop_sim.sh > /dev/null 2>&1
  echo "== $id start $(date -u +%H:%M:%S) load $(cut -d' ' -f1-3 /proc/loadavg) io $(awk '/some/{print $2}' /proc/pressure/io) cpuset ${FLIGHT_CPUSET:-all} commit $(git rev-parse --short HEAD) dirty=$(git status --short | wc -l) profile=$profile target=${TARGET:-sim-urban-point-to-point-headless}" >> "$out"
  env "${envs[@]}" DRONE_GAZEBO_RUN_ID=$id SMOKE_DURATION_S=${SMOKE_DURATION_S:-900} ./scripts/dev_shell.sh make ${TARGET:-sim-urban-point-to-point-headless} > "log/tools/run_$id.log" 2>&1
  ./scripts/stop_sim.sh > /dev/null 2>&1
  grep -E '^FAIL|^(OK|NOTE): (mean flight|crash|no crash|post-bootstrap|persistent planner p95|mission monitor|execution ownership|the true position|real-time factor|the goal was given up|the vehicle returned|the vehicle is whole|production tick wall time|no vehicle collided)' "log/tools/run_$id.log" >> "$out"
  # whether the flight counts, by the host it was flown on (specification A7)
  python3 tools/host_verdict.py "$id" >> "$out" 2>&1
  # foreign load during the flight, for the record
  echo "   rust_busy_after=$(ps -eo pcpu,comm | grep -E 'rustc|clippy|cargo' | awk '$1>5{c++} END{print c+0}') load_after=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$out"
  echo "== $id end $(date -u +%H:%M:%S)" >> "$out"
done
echo "SERIES DONE $(date -u +%H:%M:%S)" >> "$out"
