#!/usr/bin/env python3
"""video_check.py FILE...: what a finished split recording looks like, with no run beside it: its length, how many
different frames a second each half shows (in windows of 5 s over the whole file: the median, the least, and the share
of windows under the recorder's floor of 6), and in how many sampled frames the world half holds the bright green of
Gazebo's light gizmo. Prints "VIDEO ok ..." or "VIDEO BAD ..." per file and exits 1 when any is bad.

A slideshow shows about one different frame a second in the world half (the first batch, r1030 to r1058); a smooth
recording 19 to 24. A still stretch is not a slideshow: a vehicle that holds leaves the world half unchanged, so the
verdict is on the median, as the recorder's is (scripts/record_flight_video.py judges it where the vehicle moves)."""

from __future__ import annotations

import statistics
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import record_flight_video as recorder  # noqa: E402

GIZMO_PIXELS = 15  # in a 480 x 540 world half; the gizmo of r1041 filled 20 to 330


def gizmo_frames(path: Path) -> tuple[int, int]:
    """(frames looked at, frames with the gizmo's green) of the world half, one frame every 4 s."""
    width, height = 480, 540
    raw = subprocess.run(
        ["ffmpeg", "-v", "error", "-i", str(path), "-vf",
         f"crop={recorder.HALF_WIDTH}:{recorder.HEIGHT}:0:0,fps=1/4,scale={width}:{height}",
         "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True, check=False).stdout
    size = width * height * 3
    frames = [raw[start:start + size] for start in range(0, len(raw) - size + 1, size)]
    green = sum(
        sum(1 for i in range(0, size, 3)
            if frame[i + 1] > 150 and frame[i] < 110 and frame[i + 2] < 110) > GIZMO_PIXELS
        for frame in frames)
    return len(frames), green


def check(path: Path) -> list[str]:
    info = recorder.probe(path)
    duration_s = info["duration_s"]
    reasons = []
    if (info["width"], info["height"]) != (2 * recorder.HALF_WIDTH, recorder.HEIGHT):
        reasons.append(f"frame {info['width']}x{info['height']}")
    if duration_s < recorder.MINIMUM_DURATION_S:
        reasons.append(f"{duration_s:.0f} s is shorter than a minute")
    report = []
    begins = [recorder.MOVING_WINDOW_S * step
              for step in range(int(duration_s // recorder.MOVING_WINDOW_S))]
    for half in ("world", "RViz"):
        rates = [recorder.distinct_frames_per_second(path, half == "world", begin)
                 for begin in begins] or [0.0]
        median = statistics.median(rates)
        slow = sum(rate < recorder.MINIMUM_DISTINCT_FRAMES_PER_SECOND for rate in rates)
        report.append(f"{half} {median:.1f} at the median, {min(rates):.1f} at least, "
                      f"{100.0 * slow / len(rates):.0f}% of windows under "
                      f"{recorder.MINIMUM_DISTINCT_FRAMES_PER_SECOND:.0f}")
        if median < recorder.MINIMUM_DISTINCT_FRAMES_PER_SECOND:
            reasons.append(f"a slideshow in the {half} half")
    looked, green = gizmo_frames(path)
    if green:
        reasons.append(f"the light's gizmo in {green} of {looked} sampled frames")
    print(f"VIDEO {'BAD' if reasons else 'ok'} {path} {duration_s:.1f} s; different frames a second: "
          + "; ".join(report) + (": " + "; ".join(reasons) if reasons else ""), flush=True)
    return reasons


def main() -> int:
    bad = [path for path in map(Path, sys.argv[1:]) if check(path)]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
