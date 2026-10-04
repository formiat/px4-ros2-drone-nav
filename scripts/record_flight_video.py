#!/usr/bin/env python3
"""Roadmap item 20: the split recording of one flight, with nobody at the desk.

Run on the host, beside a flight started with RECORD_VIDEO=1. That flight is
the headless one with its pictures on the desktop: the Gazebo window showing
the 3D scene alone and RViz in the repository's two views. Each window hands
its frames to a FIFO in the run's directory, read back where the window
draws them (frame_pace_shim.c), with the size of what it hands over beside
it. This script waits for the three, stores their frames through the GPU's
encoder while the flight flies, and after the flight joins them into two
files, the world on the left and RViz on the right, one for each view, cut
to the mission from its readiness to its result, and checks them.

Nothing is read through the X server and nothing is put together during the
flight: a window read through the server cost the simulator a seventh of its
speed (r1005 to r1007). What remains is the price of the windows themselves:
beside a flight they hold the simulator at 0.72 to 0.80 of real time on the
stereo set (r1015 to r1017), under the floor of the host's verdict, so a
recorded flight is a demonstration and never an acceptance flight
(specification A11).

Nobody moves a view and nothing but the two pictures is in the frame: what
is read is the 3D view each window draws, not the window.
"""

from __future__ import annotations

import argparse
import json
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

HALF_WIDTH = 960
HEIGHT = 1080
FRAME_RATE = 24
SOURCES = ("world", "follow", "top")
VIEWS = ("follow", "top")
# Frames a second each window hands over: RViz draws 15 (rviz_recording_view.py).
CAPTURE_RATES = {"world": 24, "follow": 15, "top": 15}
# Seconds of picture kept before the mission is ready and after its result.
LEAD_S = 3.0
TAIL_S = 3.0


def capture_sizes(run_directory: Path) -> dict[str, tuple[int, int]]:
    """What each window says it hands over, by source; a source that has not
    said yet is absent."""
    sizes = {}
    for name in SOURCES:
        try:
            text = (run_directory / f"capture_{name}.bgra.size").read_text()
            width, height = (int(value) for value in text.strip().split("x"))
        except (OSError, ValueError):
            continue
        sizes[name] = (width, height)
    return sizes


def capture_command(sizes: dict[str, tuple[int, int]], run_directory: Path,
                    outputs: dict[str, Path]) -> list[str]:
    """One ffmpeg process over the three windows' FIFOs while the flight
    flies. Each FIFO carries BGRA frames, bottom row first, at its rate by
    the wall clock. Nothing is scaled, joined or converted here, and the
    GPU's encoder takes the frames as they come."""
    command = ["nice", "-n", "10", "ffmpeg", "-hide_banner", "-loglevel", "error", "-y"]
    for name in SOURCES:
        command += ["-f", "rawvideo", "-pixel_format", "bgra",
                    "-video_size", f"{sizes[name][0]}x{sizes[name][1]}",
                    "-framerate", str(CAPTURE_RATES[name]),
                    "-i", str(run_directory / f"capture_{name}.bgra")]
    for index, name in enumerate(SOURCES):
        command += ["-map", f"{index}:v", "-c:v", "h264_nvenc", "-preset", "p1",
                    "-rc", "constqp", "-qp", "18", "-g", str(CAPTURE_RATES[name]),
                    str(outputs[name])]
    return command


def compose_command(world: Path, view: Path, begin_s: float | None, length_s: float,
                    output: Path) -> list[str]:
    """The split picture of one view, after the flight: the world on the
    left, the view on the right, cut to the mission."""
    cut = [] if begin_s is None else ["-ss", f"{begin_s:.2f}", "-t", f"{length_s:.2f}"]
    return ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            *cut, "-i", str(world), *cut, "-i", str(view),
            "-filter_complex",
            f"[0:v]vflip,fps={FRAME_RATE},scale={HALF_WIDTH}:{HEIGHT},setsar=1[w];"
            f"[1:v]vflip,fps={FRAME_RATE},scale={HALF_WIDTH}:{HEIGHT},setsar=1[v];"
            "[w][v]hstack=inputs=2,format=yuv420p",
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-movflags", "+faststart", str(output)]


def mission_window(log_text: str) -> tuple[float, float] | None:
    """Wall-clock seconds of the mission's readiness and of its result."""
    ready = re.search(r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_READINESS ready=true",
                      log_text)
    result = None
    for result in re.finditer(r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_RESULT ",
                              log_text):
        pass
    if ready is None or result is None:
        return None
    return float(ready.group(1)), float(result.group(1))


def probe(path: Path) -> dict:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "stream=width,height:format=duration", "-of", "json", str(path)],
        capture_output=True, text=True, check=False).stdout
    data = json.loads(out or "{}")
    stream = (data.get("streams") or [{}])[0]
    return {"width": stream.get("width"), "height": stream.get("height"),
            "duration_s": float(data.get("format", {}).get("duration", 0.0) or 0.0)}


def picture_statistics(path: Path, duration_s: float) -> dict:
    """Mean brightness (0..255) of each half at a quarter, a half and three
    quarters of the recording, and how much the picture changes between them:
    a black half or a still picture is not a recording of a flight."""
    means = []
    for share in (0.25, 0.5, 0.75):
        raw = subprocess.run(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-ss", f"{share * duration_s:.2f}",
             "-i", str(path), "-frames:v", "1", "-vf", "scale=96:54,format=gray",
             "-f", "rawvideo", "-"], capture_output=True, check=False).stdout
        if len(raw) != 96 * 54:
            return {"readable": False}
        rows = [raw[i * 96:(i + 1) * 96] for i in range(54)]
        left = sum(sum(row[:48]) for row in rows) / (48 * 54)
        right = sum(sum(row[48:]) for row in rows) / (48 * 54)
        means.append((left, right, raw))
    change = sum(abs(a - b) for a, b in zip(means[0][2], means[2][2])) / (96 * 54)
    return {"readable": True,
            "world_brightness": round(sum(m[0] for m in means) / 3, 2),
            "rviz_brightness": round(sum(m[1] for m in means) / 3, 2),
            "change": round(change, 2)}


def verdict(info: dict, statistics: dict, flight_s: float) -> list[str]:
    """Why a recording is not one; empty when it is."""
    reasons = []
    if (info.get("width"), info.get("height")) != (2 * HALF_WIDTH, HEIGHT):
        reasons.append(f"frame {info.get('width')}x{info.get('height')}")
    if abs(info.get("duration_s", 0.0) - (flight_s + LEAD_S + TAIL_S)) > 5.0:
        reasons.append(f"duration {info.get('duration_s', 0.0):.0f} s for a flight of "
                       f"{flight_s:.0f} s")
    if not statistics.get("readable"):
        reasons.append("frames unreadable")
    else:
        if statistics["rviz_brightness"] < 3.0:
            reasons.append("the RViz half is black")
        if statistics["change"] < 0.5:
            reasons.append("the picture does not change")
    return reasons


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    # Positional, with no option among them: the desktop's idle inhibitor this
    # runs under reads every option on its command line as its own.
    parser.add_argument("run")
    parser.add_argument("flight_pid", type=int,
                        help="the process whose end is the flight's end")
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("name", help="file name stem: scenario and run")
    parser.add_argument("wait_s", type=float, nargs="?", default=900.0)
    args = parser.parse_args()
    run_directory = Path(f"log/runs/{args.run}")

    def flight_alive() -> bool:
        return Path(f"/proc/{args.flight_pid}").exists()

    # The windows open and settle: a size that stands for five seconds is the
    # size the frames will have.
    deadline = time.time() + args.wait_s
    sizes: dict[str, tuple[int, int]] = {}
    stable_since = time.time()
    settled = False
    while time.time() < deadline and flight_alive():
        now = capture_sizes(run_directory)
        if now != sizes:
            sizes, stable_since = now, time.time()
        if len(sizes) == len(SOURCES) and time.time() - stable_since >= 5.0:
            settled = True
            break
        time.sleep(1.0)
    if not settled:
        print(f"RECORDING none: windows that reported {sorted(sizes)}", flush=True)
        return 1

    args.output_directory.mkdir(parents=True, exist_ok=True)
    raw = {name: run_directory / f"capture_{name}.mkv" for name in SOURCES}
    started = time.time()
    capture = subprocess.Popen(capture_command(sizes, run_directory, raw),
                               stdin=subprocess.PIPE)
    print(f"RECORDING started sizes={sizes}", flush=True)
    while flight_alive() and capture.poll() is None:
        time.sleep(1.0)
    if capture.poll() is None:
        capture.send_signal(signal.SIGINT)
        try:
            capture.wait(timeout=30)
        except subprocess.TimeoutExpired:
            capture.kill()

    log = run_directory / "ros_drone_nav.log"
    span = mission_window(log.read_text(errors="ignore")) if log.exists() else None
    begin = max(0.0, span[0] - started - LEAD_S) if span else None
    flight_s = span[1] - span[0] if span else 0.0
    failed = False
    for view in VIEWS:
        final = args.output_directory / f"{args.name}_{view}.mp4"
        subprocess.run(compose_command(raw["world"], raw[view], begin,
                                       flight_s + LEAD_S + TAIL_S, final), check=False)
        info = probe(final)
        statistics = picture_statistics(final, info["duration_s"]) if info["duration_s"] else {}
        reasons = verdict(info, statistics, flight_s) if span else ["no mission in the log"]
        failed = failed or bool(reasons)
        print(f"RECORDING {'BAD' if reasons else 'ok'} {final} "
              f"duration={info['duration_s']:.1f}s {json.dumps(statistics)}"
              + (": " + "; ".join(reasons) if reasons else ""), flush=True)
    if not failed:
        for source in raw.values():
            source.unlink(missing_ok=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
