#!/usr/bin/env bash
# render_flight.sh SCENARIO FIRST_RUN DIRECTORY [RATE]: one flight of a named scenario (point-to-point, light-lost,
# light-failure, low-battery, smoke-blocked, smoke, smoke-rooms, clear on Urban, dark-hall in roadmap item 29's hall, dark-void across it at 12 m (item 31);
# the stereo set only) flown as its ordinary headless flight at a real-time factor of 1.0 (the factor's variable
# passes through to the flight: the stereo set's smoke flights held 0.8, K25; the smoke scenarios are not flown since
# 2026-10-24, specification F4)
# with no window open, through fly_until_valid.sh (two attempts), recording what RViz shows (RENDER_BAG=1); then its
# pictures rendered afterwards into two split files in DIRECTORY, the world on the left and RViz on the right, one per
# view (scripts/render_flight_video.py, at RATE of the wall clock, 1.0 unless given). The flight is judged as any
# other (specification A7, A11); the recording is checked as a live one is. Output: log/tools/render_RUN.log.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
scenario=$1; first=$2; directory=$3; rate=${4:-1.0}
case $scenario in
  point-to-point | light-lost | light-failure | low-battery | smoke-blocked | smoke | smoke-rooms | clear) target="sim-urban-${scenario}-headless" ;;
  dark-hall) target="sim-dark-hall-headless" ;;
  dark-void) target="sim-dark-void-headless" ;;
  *) echo "unknown or set-aside scenario $scenario" >&2; exit 2 ;;
esac
RENDER_BAG=1 ENABLE_RVIZ=true RVIZ_CONFIG_FILE=none ENABLE_LIDAR_DEBUG=true \
  TARGET="$target" ./tools/fly_until_valid.sh "$first" 2 > /dev/null 2>&1
result=$(grep -v END log/tools/fly_until_valid.txt | tail -1)
echo "FLIGHT $result"
case "$result" in *VALID*) ;; *) exit 1;; esac
run=${result%% *}
mkdir -p "$directory"; touch "$directory/.keep"
# The desktop is kept from going idle while the windows draw, as a live recording keeps it.
gnome-session-inhibit --inhibit idle --reason "rendering flight $run" \
  python3 scripts/render_flight_video.py "$run" "$directory" "${scenario}_${run}" "$rate" \
  > "log/tools/render_${run}.log" 2>&1
status=$?
grep "^RECORDING" "log/tools/render_${run}.log"
exit $status
