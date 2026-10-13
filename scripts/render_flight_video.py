#!/usr/bin/env python3
"""Part 6 of the technical-debt goal: the split recording of a flight, rendered after it.

The flight was the ordinary headless one at a real-time factor of 1.0 with no
window open, flown with RENDER_BAG=1 (ENABLE_RVIZ=true, RVIZ_CONFIG_FILE=none):
it published all RViz shows and recorded it (render_bag), beside its own true
pose and light records. Run on the host after it: the pictures are drawn again
in the simulation container (render_flight_runtime.sh) -- the flight's world
held paused with the vehicle moved through its true poses and its light as it
was set, in the recording's Gazebo window, and the bag played into both RViz
views on the simulation's clock -- at RATE of the wall clock, each window
handing its frames to its FIFO as in a live recording. The frames are stored
through the GPU's encoder, timed by the clock each half was drawn on, and
joined exactly as a live recording's (record_flight_video.py): the world on the
left, RViz on the right, one file a view, one frame a 24th of a second of the
flight, every frame within half a second of its moment.
"""

from __future__ import annotations

import argparse
import csv
import json
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_flight_video import (  # noqa: E402
    CAPTURE_RATES, MINIMUM_DISTINCT_FRAMES_PER_SECOND, MINIMUM_REDRAWS_PER_SECOND,
    MAXIMUM_WINDOWS_LOOKED_AT, SOURCES, VIEWS, capture_command, capture_sizes, compose,
    distinct_frames_per_second, flight_clock, flight_end_wall_s, frame_table, moving_windows,
    picture_statistics, probe, redraws, simulation_time_at, verdict)


def read_clock(path: Path) -> list[tuple[float, ...]]:
    """(wall clock, simulation clock, ...) rows of a rendering's clock record."""
    rows = []
    try:
        with path.open(newline="") as stream:
            for row in csv.reader(stream):
                rows.append(tuple(float(value) for value in row) + (0.0,) * (5 - len(row)))
    except (OSError, ValueError):
        return []
    rows.sort()
    return rows


def flight_span(run_directory: Path) -> tuple[float, float]:
    """The flight on the simulation clock: from the first true pose to half a
    second after the mission's result (the end of a live recording too)."""
    clock = flight_clock(run_directory)
    end_wall_s = flight_end_wall_s(run_directory, clock[-1][0])
    return clock[0][1], simulation_time_at(clock, end_wall_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run")
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("name", help="file name stem: scenario and run")
    parser.add_argument("rate", type=float, nargs="?", default=1.0)
    args = parser.parse_args()
    run_directory = Path(f"log/runs/{args.run}")
    manifest = json.loads((run_directory / "manifest.json").read_text())
    world_sdf = manifest["world"]["path"]
    begin_s, end_s = flight_span(run_directory)

    container = ["./scripts/container_run.sh", "bash", "-lc"]
    workspace_run = f"/workspace/{run_directory}"
    for name in SOURCES:
        for suffix in (".bgra", ".bgra.size", ".bgra.rate"):
            (run_directory / f"capture_{name}{suffix}").unlink(missing_ok=True)
    subprocess.run(container + [
        f"cd /workspace && source scripts/video_recording_runtime.sh && "
        f"prepare_video_recording /workspace {workspace_run}"], check=True,
        stdout=subprocess.DEVNULL)
    runtime = subprocess.Popen(container + [
        f"bash /workspace/scripts/render_flight_runtime.sh {workspace_run} "
        f"/workspace/{world_sdf} {args.rate} {begin_s:.3f} {end_s:.3f}"],
        stdout=(run_directory / "render_runtime.log").open("w"), stderr=subprocess.STDOUT,
        start_new_session=True)

    sizes: dict[str, tuple[int, int]] = {}
    stable_since = time.time()
    deadline = time.time() + 300.0
    while time.time() < deadline and runtime.poll() is None:
        now = capture_sizes(run_directory)
        if now != sizes:
            sizes, stable_since = now, time.time()
        if len(sizes) == len(SOURCES) and time.time() - stable_since >= 5.0:
            break
        time.sleep(1.0)
    else:
        print(f"RECORDING none: windows that reported {sorted(sizes)}", flush=True)
        runtime.send_signal(signal.SIGTERM)
        return 1

    args.output_directory.mkdir(parents=True, exist_ok=True)
    raw = {name: run_directory / f"capture_{name}.mkv" for name in SOURCES}
    started = time.time()
    with (run_directory / "capture_ffmpeg.log").open("w") as capture_log:
        capture = subprocess.Popen(capture_command(sizes, run_directory, raw),
                                   stdin=subprocess.PIPE, stderr=capture_log)
        print(f"RECORDING started sizes={sizes} rate={args.rate}", flush=True)
        # The Gazebo window draws nothing while it loads the scene, for some
        # seconds after it reports its size (r1408: eleven without a redraw):
        # the replay begins once it redraws steadily.
        ready_deadline = time.time() + 120.0
        while time.time() < ready_deadline and runtime.poll() is None:
            counts = redraws(run_directory, "world", time.time() - 5.0, time.time())
            if len(counts) >= 3 and min(counts[-3:]) >= MINIMUM_REDRAWS_PER_SECOND["world"]:
                break
            time.sleep(1.0)
        time.sleep(1.0)
        go = time.time()
        (run_directory / "render_go").touch()
        while runtime.poll() is None and capture.poll() is None:
            time.sleep(1.0)
    if capture.poll() is None:
        capture.send_signal(signal.SIGINT)
        try:
            capture.wait(timeout=30)
        except subprocess.TimeoutExpired:
            capture.kill()
    if runtime.poll() is None:
        runtime.send_signal(signal.SIGTERM)
        runtime.wait(timeout=60)

    clocks = {"world": read_clock(run_directory / "render_world_clock.csv"),
              "follow": read_clock(run_directory / "render_rviz_clock.csv")}
    clocks["top"] = clocks["follow"]
    if any(len(clock) < 2 for clock in clocks.values()):
        print("RECORDING none: no clock to time the frames by", flush=True)
        return 1
    # The recording spans the flight from where both halves were drawn.
    begin_sim_s = max(simulation_time_at(clocks[name], started) for name in SOURCES)
    begin_sim_s = max(begin_sim_s, begin_s)
    spans, still_shares = {}, {}
    for name in SOURCES:
        wall_s = probe(raw[name])["duration_s"]
        spans[name] = min(simulation_time_at(clocks[name], started + wall_s), end_s) - begin_sim_s
        counts = redraws(run_directory, name, go, started + wall_s)
        still = sum(count < MINIMUM_REDRAWS_PER_SECOND[name] for count in counts)
        still_shares[name] = still / len(counts) if counts else 1.0
    flight_s = min(spans.values())
    flight = flight_clock(run_directory)
    windows = moving_windows(flight, begin_sim_s, begin_sim_s + flight_s)
    windows = windows[::max(1, len(windows) // MAXIMUM_WINDOWS_LOOKED_AT)]
    tables, offsets = {}, {}
    for name in SOURCES:
        tables[name], offsets[name] = frame_table(clocks[name], started, CAPTURE_RATES[name],
                                                  begin_sim_s, flight_s)
    failed = False
    for view in VIEWS:
        final = args.output_directory / f"{args.name}_{view}.mp4"
        compose(raw["world"], raw[view], {"world": tables["world"], "view": tables[view]},
                max(probe(raw["world"])["duration_s"], probe(raw[view])["duration_s"]), final)
        offset_s = max(offsets["world"], offsets[view])
        info = probe(final)
        picture = picture_statistics(final, info["duration_s"]) if info["duration_s"] else {}
        distinct = {
            half: statistics.median(
                [distinct_frames_per_second(final, half == "world", begin) for begin in windows]
                or [MINIMUM_DISTINCT_FRAMES_PER_SECOND])
            for half in ("world", "RViz")}
        reasons = verdict(info, picture, flight_s,
                          {name: still_shares[name] for name in ("world", view)}, distinct,
                          offset_s)
        failed = failed or bool(reasons)
        print(f"RECORDING {'BAD' if reasons else 'ok'} {final} "
              f"duration={info['duration_s']:.1f}s flight={flight_s:.1f}s "
              f"frame_offset={offset_s:.2f}s rate={args.rate} "
              f"still={json.dumps(still_shares)} distinct={json.dumps(distinct)} "
              f"{json.dumps(picture)}"
              + (": " + "; ".join(reasons) if reasons else ""), flush=True)
    if not failed:
        for source in raw.values():
            source.unlink(missing_ok=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
