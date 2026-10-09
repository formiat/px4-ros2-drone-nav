#!/usr/bin/env bash
# record_batch.sh DIRECTORY PASSES FIRST_RUN: records every scenario of roadmap item 17 on the stereo set with nobody at the desk
# (roadmap item 20). The scenarios are flown round robin, all of them once, then all of them again, PASSES times, so
# that a batch cut short still holds every scenario; a flight that failed its check or whose recording is not one is
# flown again at the end of its pass, once. Runs are numbered from FIRST_RUN (an integer: 1020 is r1020). The videos
# and index.md land in DIRECTORY, which is kept from the log pruning. Flights run one at a time; the batch's own log
# is DIRECTORY/batch.log.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
directory=$1; passes=$2; number=$3
mkdir -p "$directory"; touch "$directory/.keep"
# The long flight to B and back and the goal-outside flight are set aside (the owner, 2026-10-08): neither is flown
# nor recorded without the owner's word, the latter until item 19's proof is flown (roadmap item 24).
scenarios=("point-to-point" "light-lost" "light-failure" "low-battery")
fly() {  # fly SCENARIO: 0 when the flight and its recording are good
  local run="r$number"
  number=$((number + 1))
  echo "$(date +%H:%M:%S) $1 $run" >> "$directory/batch.log"
  ./tools/record_flight.sh "$1" "$run" "$directory" >> "$directory/batch.log" 2>&1
  [ "$(python3 tools/record_index.py "$1" "$run" "$directory")" = OK ]
}
for pass in $(seq 1 "$passes"); do
  again=()
  for scenario in "${scenarios[@]}"; do
    fly "$scenario" || again+=("$scenario")
  done
  for scenario in "${again[@]}"; do
    fly "$scenario" || true
  done
  echo "$(date +%H:%M:%S) pass $pass done" >> "$directory/batch.log"
done
echo "BATCH DONE" >> "$directory/batch.log"
