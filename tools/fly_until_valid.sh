#!/usr/bin/env bash
# fly_until_valid.sh [--lidar] FIRST_RUN ATTEMPTS: one flight that counts, on a host another task may load at any time
# (specification A4, A7). Each attempt is a series2.sh flight (the quiet-host gate, a stop before and after), watched
# every 20 s by rtf_watchdog.py: a flight the load has slowed under the verdict's floor is stopped at once. After the
# flight host_verdict.py decides; a flight that does not count, or was stopped, is flown again under the next run
# number, at most ATTEMPTS times. A flight that counts is the result, whatever its mission check says: a FAIL in it
# is a finding, not a reason to fly again. TARGET, SMOKE_DURATION_S and REAL_TIME_FACTOR pass through to series2.sh.
# One line a flight in log/tools/fly_until_valid.txt (rNNN VALID | VOID reason | WATCHDOG reason), then END; exits 0
# on a flight that counts.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
lidar=(); watch=()
if [ "${1:-}" = "--lidar" ]; then lidar=(--lidar); watch=(--lidar); shift; fi
first=$1; attempts=$2
flag=log/tools/fly_until_valid.txt; rm -f "$flag"
for number in $(seq "$first" $((first + attempts - 1))); do
  run="r$number"
  if [ -d "log/runs/$run" ]; then echo "$run EXISTS: not flown" >> "$flag"; continue; fi
  ./tools/series2.sh "${lidar[@]}" "$run" > /dev/null 2>&1 &
  flight=$!
  stopped=""
  while kill -0 "$flight" 2> /dev/null; do
    sleep 20
    [ -f "log/runs/$run/resources.csv" ] || continue
    if ! verdict=$(python3 tools/rtf_watchdog.py "$run" "${watch[@]}" --factor "${REAL_TIME_FACTOR:-1.0}"); then
      stopped="${verdict#WATCHDOG slow $run }"
      # By the process table: a pattern search of command lines would match this script's own.
      kill $(ps -eo pid,args | awk '/tools\/series2.sh/ && !/awk/ {print $1}') 2> /dev/null
      sleep 2; ./scripts/stop_sim.sh > /dev/null 2>&1
      break
    fi
  done
  wait "$flight" 2> /dev/null
  if [ -n "$stopped" ]; then
    echo "$run WATCHDOG $stopped" >> "$flag"
  elif verdict=$(python3 tools/host_verdict.py "$run" 2>&1); then
    echo "$run VALID" >> "$flag"; echo END >> "$flag"; exit 0
  else
    echo "$run VOID ${verdict#HOST VOID $run: }" >> "$flag"
  fi
done
echo END >> "$flag"
exit 1
