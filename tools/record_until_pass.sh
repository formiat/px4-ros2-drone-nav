#!/usr/bin/env bash
# record_until_pass.sh SCENARIO FIRST_RUN ATTEMPTS DIRECTORY: records SCENARIO (tools/record_flight.sh) until one flight
# passes its check with a recording that is one, at most ATTEMPTS times, runs numbered from FIRST_RUN (an integer).
# A flight that stands still for twelve mission summaries in a row, a minute of simulation, is stopped and flown
# again without waiting for its window to end: the long flight under failures does that below real time (the
# register's P10). One line a flight in log/tools/record_until_pass.txt (PASS, STALLED or FAILED), then END.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
scenario=$1; first=$2; attempts=$3; directory=$4
flag=log/tools/record_until_pass.txt; rm -f "$flag"
for number in $(seq "$first" $((first + attempts - 1))); do
  run="r$number"; log="log/runs/$run/ros_drone_nav.log"; stalled=0
  ./tools/record_flight.sh "$scenario" "$run" "$directory" > /dev/null 2>&1 &
  flight=$!
  while kill -0 "$flight" 2> /dev/null; do
    sleep 20
    [ -f "$log" ] || continue
    rest=$(grep "mission_monitor_node\]: Mission summary" "$log" | tail -12 |
      awk '{for (i = 1; i <= NF; i++) if ($i ~ /^speed=/) s = substr($i, 7); if (s + 0 < 0.05) c++} END {print c + 0}')
    if [ "$rest" -ge 12 ] && [ "$(grep -c "moved=true" "$log")" -gt 20 ]; then
      stalled=1
      # By the process table: a pattern search of command lines would match this script's own.
      kill $(ps -eo pid,args | awk '/tools\/record_flight.sh|tools\/series2.sh|scripts\/record_flight_video.py/ && !/awk/ {print $1}') 2> /dev/null
      sleep 2; ./scripts/stop_sim.sh > /dev/null 2>&1
      rm -f "$directory/${scenario}_${run}"_*.mp4
      break
    fi
  done
  wait "$flight" 2> /dev/null
  if [ "$stalled" -eq 1 ]; then
    echo "$run STALLED" >> "$flag"
  elif [ "$(grep -c "RECORDING ok" "log/tools/record_$run.log" 2> /dev/null)" -eq 2 ] &&
    ! grep -q "^FAIL" "log/tools/run_$run.log" 2> /dev/null; then
    echo "$run PASS" >> "$flag"; break
  else
    echo "$run FAILED" >> "$flag"
  fi
done
echo END >> "$flag"
