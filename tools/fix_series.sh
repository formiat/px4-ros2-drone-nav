#!/usr/bin/env bash
# fix_series.sh [-n] FIRST [SCOPE...]: the flights owed after a change of flight code (specification A14,
# docs/autonomous_development.md, section 5): what the change acts on, then the Urban acceptance, and nothing else.
# Without a SCOPE it is the acceptance alone, five flights on the stereo set and five on the lidar (A1). A SCOPE adds
# the flights of what the change touches, flown before the acceptance:
#
#   bodies    the warm bodies (the moving layer, the following, the body's speed law, the core's marks): the three
#             meetings of roadmap item 21, three flights a set, eighteen
#   tunnels   the lidar-inertial estimator's unobserved axis (K70): the finals world on the lidar, five
#   open-sky  the open sky's profile, leg and descent: R8's five on the full set
#   signal    the satellite trust and the estimate without the signal: open-sky and one flight with the signal lost
#             for 90 s
#   release   the whole set, once before a release and on the owner's word: bodies, signal, the open sky's camera
#             set (three), the cave and the finals world on both sets (five each); fifty-seven flights with the
#             acceptance, 13 to 14 hours
#
# The cave, the finals world and the open sky are not flown after a change that does not touch their code. -n prints
# the chain and flies nothing. FIRST is the first run number; the flights go through tools/world_series.sh, one at a
# time (ATTEMPTS, ROUND_PAUSE_S, ROUNDS and EXTRA pass through to it).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 2
dry=""
if [ "${1:-}" = "-n" ]; then dry=1; shift; fi
if [ $# -lt 1 ]; then
  sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 2
fi
first=${1#r}; shift
bodies=(walking-body-lidar:3 walking-body-oncoming-lidar:3 walking-body-side-lidar:3
  walking-body-cameras:3 walking-body-oncoming-cameras:3 walking-body-side-cameras:3)
before=(); lost=""; after=()
for scope in "$@"; do
  case $scope in
    bodies) before+=("${bodies[@]}") ;;
    tunnels) before+=(finals-lidar:5) ;;
    open-sky) before+=(open-sky-full:5) ;;
    signal) before+=(open-sky-full:5); lost=1 ;;
    release)
      before+=("${bodies[@]}" open-sky-full:5); lost=1
      after+=(open-sky-hybrid:3 cave-lidar:5 cave-cameras:5 finals-lidar:5 finals-cameras:5) ;;
    *) echo "fix_series.sh: '$scope' is no scope (bodies, tunnels, open-sky, signal, release)" >&2; exit 2 ;;
  esac
done
# One group once, in the order first asked for: a release names what bodies and signal name.
groups=()
for group in "${before[@]}"; do
  [[ " ${groups[*]} " == *" $group "* ]] || groups+=("$group")
done
next() {
  local last
  last=$(ls log/runs 2> /dev/null | sed 's/^r//' | grep -E '^[0-9]+$' | sort -n | tail -1)
  [ "${last:-0}" -ge "$first" ] && echo $((last + 1)) || echo "$first"
}
fly() {
  if [ -n "$dry" ]; then echo "tools/world_series.sh FIRST $*"; else tools/world_series.sh "$(next)" "$@"; fi
}
[ ${#groups[@]} -eq 0 ] || fly "${groups[@]}" || exit $?
if [ -n "$lost" ]; then
  if [ -n "$dry" ]; then echo 'EXTRA="GNSS_FAULTS=lost@30:90" tools/world_series.sh FIRST open-sky-full:1'
  else EXTRA="GNSS_FAULTS=lost@30:90" tools/world_series.sh "$(next)" open-sky-full:1 || exit $?; fi
fi
[ ${#after[@]} -eq 0 ] || fly "${after[@]}" || exit $?
fly urban-cameras:5 urban-lidar:5
