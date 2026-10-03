#!/usr/bin/env bash
# build_recorder.sh: inside the container, builds the stereo frame recorder into log/tools/vio/record_frames.
mkdir -p /workspace/log/tools/vio && cd /workspace/log/tools/vio
g++ -O2 -std=c++17 /workspace/tools/vio/record_frames.cpp -o record_frames $(pkg-config --cflags --libs gz-transport13 gz-msgs10) 2>&1 | grep -E "error" | head
ls -la record_frames | cut -c1-70
