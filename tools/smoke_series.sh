#!/usr/bin/env bash
# smoke_series.sh GROUP FIRST: one acceptance group of the smoke detector (roadmap item 18, specification K44), five
# flights that count, one at a time through tools/fly_until_valid.sh (ATTEMPTS a flight, 3 unless set), each numbered
# after the last run. GROUP: blocked (the corridor closed by a plume, REAL_TIME_FACTOR 0.8) | ordinary (the location's
# smoke, 0.8) | clear (no smoke) | lidar (the lidar set, the ordinary smoke). Every flight's verdict lines land in
# log/tools/smoke_detector/part3/series.txt; the flights' own lines in log/tools/journal/series_<commit>_<profile>.txt
# (series2.sh) and log/tools/fly_until_valid.txt.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
group=$1; number=$2
out=log/tools/smoke_detector/part3/series.txt
mkdir -p log/tools/smoke_detector/part3 && touch log/tools/smoke_detector/part3/.keep
case "$group" in
  blocked) envs=(TARGET=sim-urban-smoke-blocked-headless REAL_TIME_FACTOR=0.8 SMOKE_DURATION_S=900); lidar=() ;;
  ordinary) envs=(REAL_TIME_FACTOR=0.8 SMOKE_DURATION_S=900); lidar=() ;;
  clear) envs=(TARGET=sim-urban-clear-headless SMOKE_DURATION_S=900); lidar=() ;;
  lidar) envs=(SMOKE_DURATION_S=900); lidar=(--lidar) ;;
  *) echo "unknown group $group"; exit 2 ;;
esac
for flight in 1 2 3 4 5; do
  echo "== $group flight $flight from r$number $(date -u +%H:%M:%S) commit $(git rev-parse --short HEAD)" >> "$out"
  env "${envs[@]}" ./tools/fly_until_valid.sh "${lidar[@]}" "$number" "${ATTEMPTS:-3}" > "log/tools/smoke_detector/part3/fly_${group}_${flight}.log" 2>&1
  cat log/tools/fly_until_valid.txt >> "$out"
  run=$(grep -E ' VALID$' log/tools/fly_until_valid.txt | tail -1 | cut -d' ' -f1)
  if [ -n "$run" ]; then
    grep -E '^FAIL|^(OK|NOTE): (mean flight|crash|no crash|the true position|the goal was given up|the vehicle returned|the vehicle is whole)' "log/tools/run_$run.log" | sed "s/^/$run /" >> "$out"
    grep -o 'smoke_faces=[0-9]* face_range_m=[0-9.na-]* face_marks_total=[0-9]*' "log/runs/$run/gz_drone_nav.log" 2>/dev/null | awk -v r="$run" 'BEGIN{f=0} /smoke_faces=[1-9]/{if(!f){print r" first face: "$0; f=1}} END{print r" last: "$0}' >> "$out"
  fi
  number=$(( $(ls log/runs | sed "s/^r//" | sort -n | tail -1) + 1 ))
done
echo "== $group done $(date -u +%H:%M:%S)" >> "$out"
