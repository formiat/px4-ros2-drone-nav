#!/usr/bin/env python3
"""Roadmap item 20: the split recording of one flight, with nobody at the desk.

Run on the host, beside a flight started with RECORD_VIDEO=1. That flight is
the headless one with its pictures on the desktop: the Gazebo window showing
the 3D scene alone and RViz in the repository's two views. Each window hands
its frames to a FIFO in the run's directory, read back where the window
draws them (frame_pace_shim.c), with the size of what it hands over and the
count of its redraws a second beside it. This script waits for the three,
stores their frames through the GPU's encoder while the flight flies, and
after the flight joins them into two files, the world on the left and RViz
on the right, one for each view, and checks them.

The recording is the whole flight, from the windows standing to the
flight's end, with nothing cut out, and it plays at the flight's own pace:
the frames are taken on the wall clock and re-timed to the simulation clock
(the true pose record carries both), so a flight slowed against the wall
(REAL_TIME_FACTOR) plays as long as it flew.

Nothing is read through the X server and nothing is put together during the
flight: a window read through the server cost the simulator a seventh of its
speed (r1005 to r1007).

Nobody moves a view and nothing but the two pictures is in the frame: what
is read is the 3D view each window draws, not the window.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
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
# A recording shorter than this is not kept (the owner, 2026-10-04).
MINIMUM_DURATION_S = 60.0
# The recording against the flight's simulation clock, over the whole flight.
MAXIMUM_CLOCK_ERROR_S = 1.0
# A window that redraws less often than this for a second is standing still:
# the slideshow of the first batch redrew once a second (r1030 to r1058).
MINIMUM_REDRAWS_PER_SECOND = {"world": 12, "follow": 7, "top": 7}
MAXIMUM_STILL_SECONDS_SHARE = 0.02
# While the vehicle moves, each half shows this many different frames a
# second or more; the slideshow showed one.
MINIMUM_DISTINCT_FRAMES_PER_SECOND = 6.0
MOVING_SPEED_MPS = 0.5
MOVING_WINDOW_S = 5.0
MAXIMUM_WINDOWS_LOOKED_AT = 24


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


def compose_command(world: Path, view: Path, world_pace: float, view_pace: float,
                    output: Path) -> list[str]:
    """The split picture of one view, after the flight: the world on the
    left, the view on the right, the whole of both, each re-timed by its
    pace (seconds of simulation to a second of its frames)."""
    return ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", str(world), "-i", str(view),
            "-filter_complex",
            f"[0:v]vflip,setpts={world_pace:.6f}*PTS,fps={FRAME_RATE},"
            f"scale={HALF_WIDTH}:{HEIGHT},setsar=1[w];"
            f"[1:v]vflip,setpts={view_pace:.6f}*PTS,fps={FRAME_RATE},"
            f"scale={HALF_WIDTH}:{HEIGHT},setsar=1[v];"
            "[w][v]hstack=inputs=2:shortest=1,format=yuv420p",
            "-c:v", "libx264", "-preset", "medium", "-crf", "20",
            "-movflags", "+faststart", str(output)]


def flight_clock(run_directory: Path) -> list[tuple[float, float, float, float, float]]:
    """(wall clock, simulation clock, x, y, z) of every true pose recorded."""
    rows = []
    try:
        with (run_directory / "gz_pose.csv").open(newline="") as stream:
            for row in csv.reader(stream):
                if len(row) >= 9:
                    rows.append((float(row[8]), float(row[0]), float(row[1]),
                                 float(row[2]), float(row[3])))
    except (OSError, ValueError):
        return []
    rows.sort()
    return rows


def simulation_time_at(clock: list[tuple[float, float, float, float, float]],
                       wall_s: float) -> float:
    index = min(max(bisect.bisect_left(clock, (wall_s,)), 1), len(clock) - 1)
    (wall_a, sim_a, *_), (wall_b, sim_b, *_) = clock[index - 1], clock[index]
    if wall_b <= wall_a:
        return sim_a
    share = min(max((wall_s - wall_a) / (wall_b - wall_a), 0.0), 1.0)
    return sim_a + share * (sim_b - sim_a)


def moving_windows(clock: list[tuple[float, float, float, float, float]],
                   begin_sim_s: float, end_sim_s: float) -> list[float]:
    """Seconds into the recording at which a window of MOVING_WINDOW_S begins
    over which the vehicle moved."""
    windows = []
    start = begin_sim_s
    index = 0
    while start + MOVING_WINDOW_S <= end_sim_s:
        path_m = 0.0
        previous = None
        while index < len(clock) and clock[index][1] < start:
            index += 1
        scan = index
        while scan < len(clock) and clock[scan][1] < start + MOVING_WINDOW_S:
            if previous is not None:
                path_m += sum((a - b) ** 2 for a, b in zip(clock[scan][2:], previous)) ** 0.5
            previous = clock[scan][2:]
            scan += 1
        if path_m >= MOVING_SPEED_MPS * MOVING_WINDOW_S:
            windows.append(start - begin_sim_s)
        start += MOVING_WINDOW_S
    return windows


def redraws(run_directory: Path, name: str, begin_s: float, end_s: float) -> list[int]:
    """How many times the window redrew in each wall second of the recording
    (frame_pace_shim.c writes a line for every second it redrew in)."""
    counts = {}
    try:
        for line in (run_directory / f"capture_{name}.bgra.rate").read_text().split("\n"):
            if line:
                second, count = line.split()
                counts[int(second)] = int(count)
    except (OSError, ValueError):
        return []
    return [counts.get(second, 0) for second in range(int(begin_s) + 1, int(end_s))]


def distinct_frames_per_second(path: Path, left: bool, begin_s: float) -> float:
    """Different frames a second in one half over MOVING_WINDOW_S of the
    recording."""
    log = subprocess.run(
        ["ffmpeg", "-hide_banner", "-ss", f"{begin_s:.2f}", "-t", f"{MOVING_WINDOW_S:.2f}",
         "-i", str(path), "-vf",
         f"crop={HALF_WIDTH}:{HEIGHT}:{0 if left else HALF_WIDTH}:0,scale=240:270,"
         "mpdecimate=hi=400:lo=200:frac=0.2,showinfo", "-f", "null", "-"],
        capture_output=True, text=True, check=False).stderr
    return log.count("pts_time:") / MOVING_WINDOW_S


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


def verdict(info: dict, statistics: dict, flight_s: float,
            still_shares: dict[str, float], distinct: dict[str, float]) -> list[str]:
    """Why a recording is not one; empty when it is. `flight_s` is the
    recorded flight on the simulation clock, `still_shares` the share of the
    seconds in which a window all but stood still, `distinct` the fewest
    different frames a second a half showed while the vehicle moved."""
    reasons = []
    if (info.get("width"), info.get("height")) != (2 * HALF_WIDTH, HEIGHT):
        reasons.append(f"frame {info.get('width')}x{info.get('height')}")
    duration_s = info.get("duration_s", 0.0)
    if duration_s < MINIMUM_DURATION_S:
        reasons.append(f"{duration_s:.0f} s is shorter than a minute")
    if abs(duration_s - flight_s) > MAXIMUM_CLOCK_ERROR_S:
        reasons.append(f"duration {duration_s:.1f} s for {flight_s:.1f} s of flight")
    if not statistics.get("readable"):
        reasons.append("frames unreadable")
    else:
        if statistics["rviz_brightness"] < 3.0:
            reasons.append("the RViz half is black")
        if statistics["change"] < 0.5:
            reasons.append("the picture does not change")
    for name, share in still_shares.items():
        if share > MAXIMUM_STILL_SECONDS_SHARE:
            reasons.append(f"the {name} window stood still {100.0 * share:.0f}% of the time")
    for half, rate in distinct.items():
        if rate < MINIMUM_DISTINCT_FRAMES_PER_SECOND:
            reasons.append(f"a slideshow: {rate:.1f} different frames a second in the "
                           f"{half} half while the vehicle moved")
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
    ended = time.time()
    if capture.poll() is None:
        capture.send_signal(signal.SIGINT)
        try:
            capture.wait(timeout=30)
        except subprocess.TimeoutExpired:
            capture.kill()

    clock = flight_clock(run_directory)
    if len(clock) < 2:
        print("RECORDING none: no true pose record to time the frames by", flush=True)
        return 1
    begin_sim_s = simulation_time_at(clock, started)
    flight_s = simulation_time_at(clock, ended) - begin_sim_s
    paces = {name: flight_s / max(probe(raw[name])["duration_s"], 1.0e-3) for name in SOURCES}
    still_shares = {}
    for name in SOURCES:
        counts = redraws(run_directory, name, started, ended)
        still = sum(count < MINIMUM_REDRAWS_PER_SECOND[name] for count in counts)
        still_shares[name] = still / len(counts) if counts else 1.0
    windows = moving_windows(clock, begin_sim_s, begin_sim_s + flight_s)
    windows = windows[::max(1, len(windows) // MAXIMUM_WINDOWS_LOOKED_AT)]
    failed = False
    for view in VIEWS:
        final = args.output_directory / f"{args.name}_{view}.mp4"
        subprocess.run(compose_command(raw["world"], raw[view], paces["world"], paces[view],
                                       final), check=False)
        info = probe(final)
        statistics = picture_statistics(final, info["duration_s"]) if info["duration_s"] else {}
        distinct = {
            half: min((distinct_frames_per_second(final, half == "world", begin)
                       for begin in windows), default=MINIMUM_DISTINCT_FRAMES_PER_SECOND)
            for half in ("world", "RViz")}
        reasons = verdict(info, statistics, flight_s,
                          {name: still_shares[name] for name in ("world", view)}, distinct)
        failed = failed or bool(reasons)
        print(f"RECORDING {'BAD' if reasons else 'ok'} {final} "
              f"duration={info['duration_s']:.1f}s flight={flight_s:.1f}s "
              f"still={json.dumps(still_shares)} distinct={json.dumps(distinct)} "
              f"{json.dumps(statistics)}"
              + (": " + "; ".join(reasons) if reasons else ""), flush=True)
    if not failed:
        for source in raw.values():
            source.unlink(missing_ok=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
