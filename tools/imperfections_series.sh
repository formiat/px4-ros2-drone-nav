#!/usr/bin/env bash
# imperfections_series.sh FAMILY...: the real vehicle's imperfections (roadmap item 35, K52) flown apart or whole: for each
# FAMILY (imu, vibration, extrinsics, stamps, or all for the whole profile) five camera flights then five lidar flights,
# seeds 1 to 5, on the clear location, each through fly_until_valid.sh (three attempts). The series file carries the
# start, the total and the count done; the flights' lines as series2.sh writes them.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p log/tools/journal; touch log/tools/journal/.keep
out="log/tools/journal/imperfections_series_$(git rev-parse --short HEAD).txt"
next() { echo $(( $(ls log/runs | sed "s/^r//" | sort -n | tail -1) + 1 )); }
total=$(( ${#} * 10 )); done_count=0
echo "== imperfections ${*}: ${total} flights from r$(next) start $(date -u +%H:%M:%S) commit $(git rev-parse --short HEAD)" >> "$out"
for family in "$@"; do
  for sensor in camera lidar; do
    lidar=(); [ "$sensor" = lidar ] && lidar=(--lidar)
    for seed in 1 2 3 4 5; do
      ./scripts/stop_sim.sh > /dev/null 2>&1
      echo "-- nominal_${family} ${sensor} seed ${seed} from r$(next) $(date -u +%H:%M:%S)" >> "$out"
      IMPERFECTIONS="$([ "$family" = all ] && echo nominal || echo "nominal_${family}")" IMPERFECTION_SEED="${seed}" TARGET=sim-urban-clear-headless SMOKE_DURATION_S=900 \
        ./tools/fly_until_valid.sh "${lidar[@]}" "$(next)" 3 > "log/tools/imperfections_fly_${family}_${sensor}_${seed}.log" 2>&1
      cat log/tools/fly_until_valid.txt >> "$out"
      run=$(grep -E ' VALID$' log/tools/fly_until_valid.txt | tail -1 | cut -d' ' -f1)
      [ -n "$run" ] && grep -E '^FAIL|^(OK|NOTE): (mean flight|crash|no crash|the true position)' "log/tools/run_$run.log" | sed "s/^/$run /" >> "$out"
      done_count=$((done_count + 1))
      echo "-- done ${done_count}/${total} $(date -u +%H:%M:%S)" >> "$out"
    done
  done
done
./scripts/stop_sim.sh > /dev/null 2>&1
echo "== imperfections ${*} done $(date -u +%H:%M:%S)" >> "$out"
