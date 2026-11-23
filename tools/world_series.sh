#!/usr/bin/env bash
# world_series.sh FIRST GROUP:COUNT [GROUP:COUNT...]: counted flights on the current commit, one at a time through
# tools/fly_until_valid.sh: COUNT flights of each GROUP in the order given, the first under the run number FIRST and
# every next one under the number after the last flown. A GROUP is WORLD-SET:
#
#   WORLD  urban | cave | finals | open-sky (the point-to-point flight of that world), or
#          walking-body | walking-body-oncoming | walking-body-side (the urban flight past a warm body, roadmap item 21)
#   SET    lidar | cameras; on the open sky also hybrid (the camera set with the satellite signal), visual (the
#          camera set without it) and full (the lidar for the obstacles with the camera set for the estimate: R8's set)
#
# The lidar flies at a real-time factor of 1.0, a camera set at 0.8, the finals world's cameras at 0.5 (K25); the
# window is 900 s of simulation, 2400 s on the open sky (SMOKE_DURATION_S overrides). A flight past a body holds the
# memory's snapshots of its corridor (RAW_SNAPSHOT_BOUNDS_M) for tools/body_encounters.py. EXTRA holds more NAME=value
# pairs for every flight (EXTRA="GNSS_FAULTS=lost@30:90"), ATTEMPTS the attempts a counted flight may take (4). A slot
# none of whose attempts counted (the host voided them) is flown again after ROUND_PAUSE_S (600), ROUNDS times (12)
# at most, then the series stops with a line saying so: a slot is never written done without a flight that counts.
#
# The series file is log/tools/journal/world_series_<commit>.txt: the start with the total, one line a flight (the
# group, its number, the run, the verdict lines, how many are done), the end. This is the one chain for the
# acceptances and series of the worlds (the held-out series of roadmap item 33, the open sky's of item 39, the urban
# five and five, the meetings of item 21), in place of a chain written under log/ for each.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [ $# -lt 2 ]; then
  sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 2
fi
n=${1#r}; shift
# The flag and the NAME=value pairs of a group; a group that is none is refused before the first flight.
resolve() {
  local world=${1%-*} set_name=${1##*-}
  flag=(); pairs=(REAL_TIME_FACTOR=0.8)
  case $set_name in
    lidar) flag=(--lidar); pairs=(REAL_TIME_FACTOR=1.0) ;;
    cameras) [ "$world" = finals ] && pairs=(REAL_TIME_FACTOR=0.5) ;;
    hybrid) pairs+=(LOCALIZATION_PROFILE=hybrid) ;;
    visual) pairs+=(LOCALIZATION_PROFILE=visual_inertial) ;;
    full) pairs+=(LOCALIZATION_PROFILE=hybrid NAVIGATION_SENSOR_PROFILE=lidar CAMERA_PROFILE=stereo_tof) ;;
    *) return 2 ;;
  esac
  case $world in
    urban | cave | finals | walking-body*)
      [ "$set_name" = lidar ] || [ "$set_name" = cameras ] || return 2 ;;&
    urban) ;;
    cave | finals) pairs+=(TARGET="sim-$world-point-to-point-headless") ;;
    open-sky)
      [ "$set_name" = cameras ] && pairs+=(LOCALIZATION_PROFILE=hybrid)
      pairs+=(TARGET=sim-open-sky-point-to-point-headless SMOKE_DURATION_S="${SMOKE_DURATION_S:-2400}") ;;
    walking-body | walking-body-oncoming | walking-body-side)
      pairs+=(TARGET="sim-urban-$world-headless" RAW_SNAPSHOT_BOUNDS_M="-33,-26,0,60,42.5,18") ;;
    *) return 2 ;;
  esac
}
total=0
for spec in "$@"; do
  [[ $spec =~ ^[a-z-]+:[0-9]+$ ]] && resolve "${spec%%:*}" ||
    { echo "world_series.sh: '$spec' is not WORLD-SET:COUNT (see the head of the script)" >&2; exit 2; }
  total=$((total + ${spec##*:}))
done
# One flight chain a checkout: refused with exit status 3 while another flies (scripts/flight_lock_runtime.sh).
. scripts/flight_lock_runtime.sh
mkdir -p log/tools/journal; touch log/tools/journal/.keep
out=log/tools/journal/world_series_$(git rev-parse --short HEAD).txt
done_count=0
echo "SERIES start $(date '+%F %T %z') commit $(git rev-parse --short HEAD) groups $* extra ${EXTRA:-none} total $total" >> "$out"
for spec in "$@"; do
  group=${spec%%:*}; count=${spec##*:}
  resolve "$group"
  for k in $(seq 1 "$count"); do
    # A flight of the series is one that counts: attempts the host voided are flown again after a pause, and a
    # slot is never written done without one (r3886 to r3889: four attempts of one slot voided under a foreign
    # emulator's bursts, two more and the slot had passed for flown).
    round=0
    while :; do
      # shellcheck disable=SC2086
      env "${pairs[@]}" ${EXTRA:-} tools/fly_until_valid.sh "${flag[@]}" "$n" "${ATTEMPTS:-4}"
      run=$(grep -o '^r[0-9]\+' log/tools/fly_until_valid.txt | tail -1)
      if [ -n "$run" ]; then n=$(( ${run#r} + 1 )); else n=$(( n + ${ATTEMPTS:-4} )); fi
      grep -q '^r[0-9]\+ VALID' log/tools/fly_until_valid.txt && break
      round=$((round + 1))
      echo "$group $k not counted, round $round: $(grep -v '^END' log/tools/fly_until_valid.txt | tr '\n' ';') at $(date '+%T')" >> "$out"
      if [ "$round" -ge "${ROUNDS:-12}" ]; then
        echo "SERIES stopped $(date '+%F %T %z'): $group $k not counted in $round rounds, $done_count of $total done" >> "$out"
        exit 1
      fi
      sleep "${ROUND_PAUSE_S:-600}"
    done
    done_count=$((done_count + 1))
    echo "$group $k $run: $(grep -v '^END' log/tools/fly_until_valid.txt | tr '\n' ';') done $done_count of $total at $(date '+%T')" >> "$out"
  done
done
echo "SERIES end $(date '+%F %T %z')" >> "$out"
