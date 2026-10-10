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
flight's end, with nothing cut out, and it plays at the flight's own pace
at every moment: the frames are taken on the wall clock, and each frame of
the recording, FRAME_RATE a second of simulation, shows the last frame taken
at or before its moment of the flight (the true pose record carries both
clocks), so a flight slowed against the wall (REAL_TIME_FACTOR), or held
unevenly by the host, plays as it flew, second for second (the owner's rule
of 2026-10-08: no frame further than half a second from its moment).

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
import re
import signal
import statistics
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
# And at every moment of it: no frame shown further than this from the
# moment of the flight it is shown at (the owner's rule of 2026-10-08). One
# pace over the whole flight, with the host holding the simulator unevenly,
# had frames 6.9 s from their moment (r1179).
MAXIMUM_FRAME_OFFSET_S = 0.5
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


def decode_command(source: Path, flight_wall_s: float) -> list[str]:
    """One window's stored frames, up to the flight's end, as raw pictures
    of a half of the recording, the right way up."""
    return ["ffmpeg", "-hide_banner", "-loglevel", "error", "-t", f"{flight_wall_s:.3f}",
            "-i", str(source), "-vf", f"vflip,scale={HALF_WIDTH}:{HEIGHT}",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]


def encode_command(output: Path) -> list[str]:
    """The recording from raw split pictures, FRAME_RATE a second."""
    return ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
            "-pix_fmt", "bgr24", "-video_size", f"{2 * HALF_WIDTH}x{HEIGHT}",
            "-framerate", str(FRAME_RATE), "-i", "-", "-c:v", "libx264", "-preset",
            "medium", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(output)]


def frame_table(clock: list[tuple[float, float, float, float, float]], started: float,
                rate: int, begin_sim_s: float, flight_s: float) -> tuple[list[int], float]:
    """For each frame of the recording, FRAME_RATE a second of simulation
    from `begin_sim_s` over `flight_s`, the index of the window's frame to
    show: the last one taken (on the wall clock, `rate` a second from
    `started`) at or before that moment of the flight. With it, the largest
    distance, in seconds of simulation, between a frame shown and the moment
    it is shown at: the frames come `rate` a second of wall time, so it is
    about a wall frame's worth of simulation, and more only where a window
    stood or the clock jumped."""
    table = []
    offset = 0.0
    index = 0
    shown_sim_s = simulation_time_at(clock, started)
    for frame in range(int(flight_s * FRAME_RATE)):
        moment_s = begin_sim_s + frame / FRAME_RATE
        while True:
            next_sim_s = simulation_time_at(clock, started + (index + 1) / rate)
            if next_sim_s > moment_s:
                break
            index += 1
            shown_sim_s = next_sim_s
        table.append(index)
        offset = max(offset, moment_s - shown_sim_s)
    return table, offset


def compose(world: Path, view: Path, tables: dict[str, list[int]], flight_wall_s: float,
            output: Path) -> int:
    """The split picture of one view, after the flight: the world on the
    left, the view on the right, frame by frame as the tables say, until
    either window's frames run out. Returns the frames written."""
    import numpy

    half_bytes = HALF_WIDTH * HEIGHT * 3
    sources = {"world": world, "view": view}
    decoders = {name: subprocess.Popen(decode_command(path, flight_wall_s),
                                       stdout=subprocess.PIPE)
                for name, path in sources.items()}
    encoder = subprocess.Popen(encode_command(output), stdin=subprocess.PIPE)
    held: dict[str, bytes | None] = {name: None for name in sources}
    read = {name: -1 for name in sources}
    written = 0
    try:
        for frame in range(min(len(table) for table in tables.values())):
            for name, table in tables.items():
                while read[name] < table[frame]:
                    data = decoders[name].stdout.read(half_bytes)
                    if len(data) < half_bytes:
                        return written
                    held[name] = data
                    read[name] += 1
            picture = numpy.hstack([
                numpy.frombuffer(held[name], dtype=numpy.uint8).reshape(HEIGHT, HALF_WIDTH, 3)
                for name in ("world", "view")])
            encoder.stdin.write(picture.tobytes())
            written += 1
        return written
    finally:
        for decoder in decoders.values():
            decoder.kill()
            decoder.wait()
        encoder.stdin.close()
        encoder.wait()


def flight_end_wall_s(run_directory: Path, clock_end_wall_s: float) -> float:
    """The wall clock of the flight's end: half a second after the mission's
    result (the logger stamps the run's log on the wall clock), for the word
    to show, or the clock's end where no result was logged. The pose record
    goes on after the result while the launch winds down (r1194: 3.2 s, the
    nodes gone and RViz's clouds with them)."""
    try:
        text = (run_directory / "ros_drone_nav.log").read_text(errors="ignore")
    except OSError:
        return clock_end_wall_s
    stamps = re.findall(r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_RESULT", text)
    return min(clock_end_wall_s, float(stamps[0]) + 0.5) if stamps else clock_end_wall_s


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
            still_shares: dict[str, float], distinct: dict[str, float],
            offset_s: float) -> list[str]:
    """Why a recording is not one; empty when it is. `flight_s` is the
    recorded flight on the simulation clock, `still_shares` the share of the
    seconds in which a window all but stood still, `distinct` the different
    frames a second a half showed while the vehicle moved, at the median,
    `offset_s` the furthest a frame shown is from its moment of the flight."""
    reasons = []
    if offset_s > MAXIMUM_FRAME_OFFSET_S:
        reasons.append(f"a frame {offset_s:.2f} s from its moment of the flight")
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
    # size the frames will have. The wait is counted from the run's start,
    # not from this script's: the flight's launcher waits for a quiet host
    # first, for as long as the host's load lasts (r1166, r1174: the windows
    # opened after the recorder had given up).
    deadline = None
    sizes: dict[str, tuple[int, int]] = {}
    stable_since = time.time()
    settled = False
    while (deadline is None or time.time() < deadline) and flight_alive():
        if deadline is None and run_directory.is_dir():
            deadline = time.time() + args.wait_s
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
    # The encoder's own words go to the run: a capture that ends early is
    # explained there (r1125 left no stream and no word in the recorder's log).
    with (run_directory / "capture_ffmpeg.log").open("w") as capture_log:
        capture = subprocess.Popen(capture_command(sizes, run_directory, raw),
                                   stdin=subprocess.PIPE, stderr=capture_log)
        print(f"RECORDING started sizes={sizes}", flush=True)
        while flight_alive() and capture.poll() is None:
            time.sleep(1.0)
        # The windows close when the simulation ends, while the flight's check
        # still runs: a capture that ended well then is in order.
        if capture.poll() not in (None, 0):
            print(f"RECORDING capture ended with {capture.returncode} (capture_ffmpeg.log)",
                  flush=True)
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
    # The flight's clock ends with the launch, which closes the following
    # RViz with it, while the world and the top-down windows live on until
    # the simulation is stopped (r1179: 34 s more): each window is cut at the
    # clock's end, and every frame of the recording is timed by the clock
    # alone (frame_table), so the halves cannot part (a pace per window over
    # its own length ran the world 17 s ahead of RViz by the end of r1179).
    begin_sim_s = simulation_time_at(clock, started)
    flight_wall_s = max(flight_end_wall_s(run_directory, clock[-1][0]) - started, 1.0e-3)
    spans, still_shares = {}, {}
    for name in SOURCES:
        wall_s = min(probe(raw[name])["duration_s"], flight_wall_s)
        spans[name] = simulation_time_at(clock, started + wall_s) - begin_sim_s
        counts = redraws(run_directory, name, started, started + wall_s)
        still = sum(count < MINIMUM_REDRAWS_PER_SECOND[name] for count in counts)
        still_shares[name] = still / len(counts) if counts else 1.0
    flight_s = min(spans.values())
    windows = moving_windows(clock, begin_sim_s, begin_sim_s + flight_s)
    windows = windows[::max(1, len(windows) // MAXIMUM_WINDOWS_LOOKED_AT)]
    tables, offsets = {}, {}
    for name in SOURCES:
        tables[name], offsets[name] = frame_table(clock, started, CAPTURE_RATES[name],
                                                  begin_sim_s, flight_s)
    failed = False
    for view in VIEWS:
        final = args.output_directory / f"{args.name}_{view}.mp4"
        compose(raw["world"], raw[view], {"world": tables["world"], "view": tables[view]},
                flight_wall_s, final)
        offset_s = max(offsets["world"], offsets[view])
        info = probe(final)
        picture = picture_statistics(final, info["duration_s"]) if info["duration_s"] else {}
        # At the median: a view of the world may be dark or hidden for a
        # while (the camera behind a wall at the start), a slideshow is one
        # all along.
        distinct = {
            half: statistics.median(
                [distinct_frames_per_second(final, half == "world", begin)
                 for begin in windows] or [MINIMUM_DISTINCT_FRAMES_PER_SECOND])
            for half in ("world", "RViz")}
        reasons = verdict(info, picture, flight_s,
                          {name: still_shares[name] for name in ("world", view)}, distinct,
                          offset_s)
        failed = failed or bool(reasons)
        print(f"RECORDING {'BAD' if reasons else 'ok'} {final} "
              f"duration={info['duration_s']:.1f}s flight={flight_s:.1f}s "
              f"frame_offset={offset_s:.2f}s "
              f"still={json.dumps(still_shares)} distinct={json.dumps(distinct)} "
              f"{json.dumps(picture)}"
              + (": " + "; ".join(reasons) if reasons else ""), flush=True)
    if not failed:
        for source in raw.values():
            source.unlink(missing_ok=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
