#!/usr/bin/env bash
# record_flight.sh SCENARIO RUN DIRECTORY: one flight of a named scenario (point-to-point, light-lost,
# light-failure, low-battery, long-failures, return-home) flown as the headless acceptance flight with its pictures on
# the desktop, and recorded: two split files in DIRECTORY, the 3D world on the left and RViz on the right, one with the
# third-person view and one with the top-down view (roadmap item 20). Nobody has to be at the desk; the desktop is
# kept from going idle while the flight lasts and nothing of its settings is changed. A recorded flight is a
# demonstration: the windows hold the simulator under the host verdict's floor (specification A11). Output of the
# recorder: log/tools/record_RUN.log. The stereo set only: a lidar flight is not recorded (specification A11).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
scenario=$1; run=$2; directory=$3
case $scenario in
  point-to-point) duration=900 ;;
  light-lost) duration=400 ;;
  long-failures) duration=1500 ;;
  return-home) duration=900 ;;
  light-failure | low-battery) duration=1800 ;;
  *) echo "unknown scenario $scenario" >&2; exit 2 ;;
esac
# The flight starts 45 s after the simulator: the Gazebo window loads the scene for some 20 s, drawing nothing, and
# a flight started at once took off unseen (r1090, r1092).
RECORD_VIDEO=1 STARTUP_SLEEP_S=45 TARGET="sim-urban-${scenario}-headless" SMOKE_DURATION_S=$duration \
  ./tools/series2.sh "$run" > /dev/null 2>&1 &
flight=$!
name="${scenario}_${run}"
gnome-session-inhibit --inhibit idle --reason "recording flight $run" \
  python3 scripts/record_flight_video.py "$run" "$flight" "$directory" "$name" \
  > "log/tools/record_${run}.log" 2>&1
wait "$flight"
cat "log/tools/record_${run}.log"
