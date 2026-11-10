#!/usr/bin/env bash
# view_flight.sh RUN|DIR [SECONDS] | --install: a flight's record open in a viewer without the stack (roadmap item
# 42). RUN is a run of log/runs flown with RECORD_STACK_BAG=true, DIR any directory that holds a stack_bag (a kept
# one, log/tools/replay/regression/RUN). The viewer is Lichtblick (MPL-2.0, offline, no account): it reads the bag's
# MCAP by the message definitions embedded in it, the stack's own messages and the autopilot's included, so nothing
# is built or sourced. Beside the bag the run's other records (the true pose, the obstacle memory's snapshots, the
# speed laws, the budgets) are written once into DIR/view/records.mcap by tools/flight_records_mcap.py and opened
# with it on the same clock; a directory with the bag alone opens the bag alone. The layout of the flight's sensor
# set (tools/viewer/stereo_tof.json or lidar.json: the memory with the true and the estimated pose, the frames, the
# mission's events and the speed limiter, the speeds, the estimate's error, the budgets) is copied to
# ~/.lichtblick-suite/layouts under a name with its digest and chosen; SECONDS seeks the simulation clock there.
# --install fetches the viewer's pinned release and the `mcap` Python package into VIEWER_ROOT (/data/gazebo/tools
# unless set); LICHTBLICK and VIEWER_PYTHON name a viewer and a Python that are elsewhere; VIEWER_ARGS passes
# arguments to the viewer (a virtual display's, for a screenshot). Host.
set -eu
cd "$(dirname "${BASH_SOURCE[0]}")/.."
root=${VIEWER_ROOT:-/data/gazebo/tools}
version=1.30.0
sha256=a8970972fd6b9fa2e197055821a385db230969acf4f5fb2e80c7f164f610ae38
viewer=${LICHTBLICK:-$root/lichtblick/lichtblick-$version-linux-x64/lichtblick}
python=${VIEWER_PYTHON:-$root/viewer/venv/bin/python}
if [[ ${1:-} == --install ]]; then
  mkdir -p "$root/lichtblick" "$root/viewer"
  if [[ ! -x $viewer ]]; then
    archive=$root/lichtblick/lichtblick-$version-linux-x64.tar.gz
    [[ -f $archive ]] || curl -L --fail -o "$archive" \
      "https://github.com/lichtblick-suite/lichtblick/releases/download/v$version/lichtblick-$version-linux-x64.tar.gz"
    echo "$sha256  $archive" | sha256sum -c -
    tar -xzf "$archive" -C "$root/lichtblick"
  fi
  if [[ ! -x $python ]]; then
    python3 -m venv "$root/viewer/venv"
    "$root/viewer/venv/bin/pip" install -q mcap
  fi
  echo "viewer: $viewer"
  echo "python: $python"
  exit 0
fi
if [[ $# -lt 1 ]]; then
  echo "usage: tools/view_flight.sh RUN|DIR [SECONDS] | --install" >&2
  exit 2
fi
if [[ -d $1/stack_bag ]]; then dir=${1%/}; else dir=log/runs/$1; fi
if ! ls "$dir"/stack_bag/*.mcap > /dev/null 2>&1; then
  echo "no stack bag in $dir: fly with RECORD_STACK_BAG=true" >&2
  exit 1
fi
if [[ ! -x $viewer ]]; then
  echo "the viewer is not installed: tools/view_flight.sh --install" >&2
  exit 1
fi
set=stereo_tof
if grep -q '^navigation_sensor_profile:=lidar' "$dir/ros_launch_args.txt" 2> /dev/null; then set=lidar; fi
# The viewer imports a layout file once by its name and keeps its own copy: the name carries the file's digest, so
# a layout changed in the repository is a new one to it.
layouts=$HOME/.lichtblick-suite/layouts
layout=drone_gazebo_${set}_$(sha256sum "tools/viewer/$set.json" | cut -c1-8)
mkdir -p "$layouts"
if [[ ! -f $layouts/$layout.json ]]; then
  rm -f "$layouts/drone_gazebo_${set}"_????????.json
  cp "tools/viewer/$set.json" "$layouts/$layout.json"
fi
records=$dir/view/records.mcap
if [[ -f $dir/gz_pose.csv && -x $python ]] &&
  [[ ! -f $records || tools/flight_records_mcap.py -nt $records ]]; then
  "$python" tools/flight_records_mcap.py "$dir/" --out "$records"
fi
sources=$(for file in "$dir"/stack_bag/*.mcap; do realpath "$file"; done | paste -sd,)
if [[ -f $records ]]; then sources+=",$(realpath "$records")"; fi
echo "viewing $dir on the $set layout: $sources"
# The release's sandbox helper needs root's ownership to work; the viewer reads local files alone.
# shellcheck disable=SC2086
exec "$viewer" --no-sandbox ${VIEWER_ARGS:-} --source="$sources" --defaultLayout="$layout" \
  ${2:+--time="$2"}
