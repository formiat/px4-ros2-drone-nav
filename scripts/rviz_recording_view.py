#!/usr/bin/env python3
"""The RViz configurations of a recorded flight (roadmap item 20).

Written from the repository's debugging configurations, with the same
displays, and with what a recording needs changed:

- no panel and no dock, and a window half the screen wide: the 3D view is
  what is recorded, and drawn larger it costs the flight more;
- both views draw at half the debugging rate: two RViz instances beside a
  flight hold the simulator a seventh under its speed whatever they draw
  (r1013, r1016), and at 30 frames a second a little more;
- the top-down view follows the vehicle, looking straight down from a stated
  distance. The debugging view is fixed over a place of the map, which nobody
  at the desk turns during a recording; following, it keeps the flight in the
  middle of the picture in any location.

Usage: rviz_recording_view.py follow|top SOURCE.rviz OUTPUT.rviz
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

FOLLOW_FRAME = "drone_follow"
TOP_DISTANCE_M = 50
FRAME_RATE = 15
WINDOW = (
    "Window Geometry:\n  Height: 1080\n  Hide Left Dock: true\n"
    "  Hide Right Dock: true\n  Width: 960\n"
)


def recording_view(config: str, view: str) -> str:
    config, rates = re.subn(r"Frame Rate: \d+", f"Frame Rate: {FRAME_RATE}", config)
    if rates != 1:
        raise ValueError("the configuration states no frame rate")
    body = config[config.index("Visualization Manager:"):config.index("Window Geometry:")]
    config = "Panels: []\n" + body + WINDOW
    if view == "follow":
        return config
    head, current = config.split("    Current:\n", 1)
    current, tail = current.split("    Saved:", 1)
    current = re.sub(r"Distance: [\d.]+", f"Distance: {TOP_DISTANCE_M}", current)
    current = re.sub(r"(X|Y|Z): [-\d.]+", r"\1: 0", current)
    current = re.sub(r"Target Frame: \S+", f"Target Frame: {FOLLOW_FRAME}", current)
    return head + "    Current:\n" + current + "    Saved:" + tail


if __name__ == "__main__":
    Path(sys.argv[3]).write_text(recording_view(Path(sys.argv[2]).read_text(), sys.argv[1]))
