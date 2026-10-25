#!/usr/bin/env bash
# cores_series.sh GROUP FIRST COUNT [TAG]: a group of flights that count on the flight's core set (FLIGHT_CPUSET, 0-11
# unless set; specification A4), one at a time through tools/fly_until_valid.sh (3 attempts a flight), each numbered
# after the last; GROUP is cameras or lidar. Beside every flight the host's CPU temperature and clocks are sampled every
# 10 s (log/tools/flight_cores/thermal_rNNNN.txt: time, the thermal zones in millidegrees, the CPUs' MHz) and, after it,
# the verdict lines, tools/host_verdict.py, tools/process_cores.py and the manifest's core set are appended to
# log/tools/flight_cores/series_TAG.txt (TAG: the group unless given). The batch logs its start, its total and its
# progress. The root goal of 2026-10-24, part 0.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
group=$1; first=$2; count=$3; tag=${4:-$group}
out="log/tools/flight_cores/series_${tag}.txt"
mkdir -p log/tools/flight_cores; touch log/tools/flight_cores/.keep
lidar=(); [ "$group" = lidar ] && lidar=(--lidar)
export FLIGHT_CPUSET="${FLIGHT_CPUSET-0-11}"
echo "== batch $group start $(date -u +%H:%M:%S) first r$first count $count cpuset ${FLIGHT_CPUSET:-all} commit $(git rev-parse --short HEAD)" | tee -a "$out"
next=$first
for i in $(seq 1 "$count"); do
  (while sleep 10; do echo "$(date -u +%H:%M:%S) $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | tr '\n' ' ')| $(grep MHz /proc/cpuinfo | awk '{printf "%d ", $4}')"; done) > "log/tools/flight_cores/thermal_r${next}.txt" &
  sampler=$!
  tools/fly_until_valid.sh "${lidar[@]}" "$next" 3 > "log/tools/flight_cores/fly_r${next}.log" 2>&1
  kill "$sampler" 2>/dev/null
  run=$(grep -E "^r[0-9]+ VALID" log/tools/fly_until_valid.txt | tail -1 | cut -d' ' -f1)
  [ -z "$run" ] && run="r$next"
  {
    echo "== $group $i of $count: $run $(date -u +%H:%M:%S)"
    grep -E "VOID|WATCHDOG" "log/tools/flight_cores/fly_r${next}.log" | head -5
    grep -E '^FAIL|^(OK|NOTE): (mean flight|no crash|the true position|real-time factor|production tick|post-bootstrap no-route)' "log/tools/run_${run}.log" 2>/dev/null
    python3 tools/host_verdict.py "$run" 2>&1 | tail -1
    python3 -I tools/process_cores.py "$run" 2>&1 | head -1
    echo "   thermal max $(awk '{for (f=2; f<=NF && $f != "|"; f++) if ($f > m) m = $f} END {print m/1000 "C"}' "log/tools/flight_cores/thermal_r${next}.txt" 2>/dev/null)"
  } | tee -a "$out"
  echo "== done $i of $count $(date -u +%H:%M:%S)" | tee -a "$out"
  next=$(( ${run#r} + 1 ))
  ./scripts/stop_sim.sh > /dev/null 2>&1
done
echo "== batch $group end $(date -u +%H:%M:%S)" | tee -a "$out"
