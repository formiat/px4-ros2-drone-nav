#!/usr/bin/env bash
# mxw.sh RUN: wait (up to 9.5 min) for the matrix flight RUN launched by mx.sh; prints DONE or RUNNING.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
for i in $(seq 1 57); do grep -q "$1\$" log/tools/mx_flag.txt 2>/dev/null && { echo DONE; exit 0; }; sleep 10; done; echo RUNNING
