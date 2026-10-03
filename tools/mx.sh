#!/usr/bin/env bash
# mx.sh LINE RUN: one flight of the item-17 acceptance matrix line LINE (1..8) through series2.sh; marks its end in
# log/tools/mx_flag.txt. 1 cameras ordinary, 2 lidar ordinary, 3 long failures, 4 light lost, 5 severe failure,
# 6 zone across B, 7 low battery, 8 goal outside (cameras).
cd "$(dirname "${BASH_SOURCE[0]}")/.."
line=$1; run=$2
if [ "$(pgrep -fc "[s]eries2.sh")" -gt 0 ] || [ -d "log/runs/$run" ]; then echo "REFUSED $line $run: another flight is running or the run exists" > log/tools/mx_flag.txt; exit 1; fi
rm -f log/tools/mx_flag.txt
case $line in
  1) SMOKE_DURATION_S=900 ./tools/series2.sh "$run" ;;
  2) SMOKE_DURATION_S=600 ./tools/series2.sh --lidar "$run" ;;
  3) TARGET=sim-urban-long-failures-headless SMOKE_DURATION_S=1500 ./tools/series2.sh "$run" ;;
  4) TARGET=sim-urban-light-lost-headless SMOKE_DURATION_S=400 ./tools/series2.sh "$run" ;;
  5) TARGET=sim-urban-light-failure-headless SMOKE_DURATION_S=1800 ./tools/series2.sh "$run" ;;
  6) TARGET=sim-urban-zone-across-b-headless SMOKE_DURATION_S=1800 ./tools/series2.sh "$run" ;;
  7) TARGET=sim-urban-low-battery-headless SMOKE_DURATION_S=1800 ./tools/series2.sh "$run" ;;
  8) TARGET=sim-urban-return-home-headless SMOKE_DURATION_S=900 ./tools/series2.sh "$run" ;;
esac
echo "DONE $line $run" > log/tools/mx_flag.txt
