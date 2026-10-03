#!/usr/bin/env bash
# mxs.sh RUN: the acceptance lines of one finished matrix flight (after mxw.sh says DONE).
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 tools/mx_inspect.py "$1" | grep -v "^   $" | grep -v "resource\|NOTE\|tick wall\|real-time" | cut -c1-235
echo "   FAIL count: $(grep -c '^FAIL' log/tools/run_$1.log); landing descents logged: $(grep -c DEAD_RECKONING_LANDING log/runs/$1/ros_drone_nav.log)"
