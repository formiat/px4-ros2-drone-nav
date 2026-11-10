#!/usr/bin/env python3
"""flight_records_mcap.py RUN|DIR [--out FILE]: a flight's records beside its stack bag, as one MCAP file the viewer
opens with the bag on the same clock (roadmap item 42; tools/view_flight.sh builds it and opens both).

The stack bag (RECORD_STACK_BAG=true) holds what the stack heard and said; the true pose, the obstacle memory, the
speed laws and the nodes' budgets are records of the run's directory. They are written here on the simulation clock
the bag is recorded on, in the viewer's own message schemas (JSON), so that no workspace is needed to read them:

  /records/truth/pose        foxglove.PoseInFrame    gz_pose.csv, the true pose at 10 Hz
  /records/truth/path        foxglove.PosesInFrame   the true path flown so far, every 2 s
  /records/truth/speed       drone_gazebo.TruthSpeed the true speed: whole, horizontal, vertical
  /records/memory/occupied   foxglove.PointCloud     the obstacle memory's occupied voxels, each raw snapshot
  /records/mppi/track        drone_gazebo.MppiTrack  mppi/mppi_track.jsonl: the reference, commanded and actual
                                                     speed, the active limiter, the clearances
  /records/estimate/error    drone_gazebo.EstimateError  the bag's estimate pose against the true pose at its stamp
                                                     (raw: no rest offset, no frame fit; tools/estimate_error.py fits)
  /records/budget/<process>  drone_gazebo.Budget     resources.csv: CPU cores and memory of each stack process
  /records/budget/host       drone_gazebo.HostBudget the real-time factor

A record the run does not hold is left out. Needs the `mcap` Python package (tools/view_flight.sh --install puts it
in a virtual environment); nothing of ROS. Host."""
from __future__ import annotations

import argparse
import base64
import bisect
import csv
import json
import math
import struct
import sys
import time
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
TOPICS = ("/records/truth/pose", "/records/truth/path", "/records/truth/speed", "/records/memory/occupied",
          "/records/mppi/track", "/records/estimate/error", "/records/budget/host")
ESTIMATE_TOPICS = ("/drone_city_nav/visual_inertial_odometry/pose", "/drone_city_nav/lidar_inertial_odometry/pose")
# The processes whose budgets are worth a plot: the stack's nodes, the autopilot and the simulator.
BUDGET_PROCESSES = ("production_mppi_node", "obstacle_memory_3d_node", "mppi_offboard_node", "mission_monitor_node",
                    "survey_mission_node", "gazebo_stereo_depth_node", "stereo_depth_node",
                    "lidar_inertial_odometry_node", "thermal_core_node", "world_visualization_node", "px4", "ruby")
CHUNK = 16

TIME = {"type": "object", "title": "time",
        "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}}}
VECTOR = {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}}}
QUATERNION = {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"},
                                               "z": {"type": "number"}, "w": {"type": "number"}}}
POSE = {"type": "object", "properties": {"position": VECTOR, "orientation": QUATERNION}}
SCHEMAS = {
    "foxglove.PoseInFrame": {"type": "object", "properties": {
        "timestamp": TIME, "frame_id": {"type": "string"}, "pose": POSE}},
    "foxglove.PosesInFrame": {"type": "object", "properties": {
        "timestamp": TIME, "frame_id": {"type": "string"}, "poses": {"type": "array", "items": POSE}}},
    "foxglove.PointCloud": {"type": "object", "properties": {
        "timestamp": TIME, "frame_id": {"type": "string"}, "pose": POSE, "point_stride": {"type": "integer"},
        "fields": {"type": "array", "items": {"type": "object", "properties": {
            "name": {"type": "string"}, "offset": {"type": "integer"}, "type": {"type": "integer"}}}},
        "data": {"type": "string", "contentEncoding": "base64"}}},
}


def numbers(*names: str, strings: tuple[str, ...] = ()) -> dict:
    properties = {name: {"type": "number"} for name in names}
    properties.update({name: {"type": "string"} for name in strings})
    return {"type": "object", "properties": properties}


SCHEMAS.update({
    "drone_gazebo.TruthSpeed": numbers("speed_mps", "horizontal_mps", "vertical_mps"),
    "drone_gazebo.MppiTrack": numbers("reference_speed_mps", "commanded_speed_mps", "actual_speed_mps",
                                      "minimum_esdf_m", "constrained_clearance_m", strings=("limiter",)),
    "drone_gazebo.EstimateError": numbers("horizontal_m", "vertical_m"),
    "drone_gazebo.Budget": numbers("cpu_cores", "rss_mib"),
    "drone_gazebo.HostBudget": numbers("real_time_factor"),
})


def stamp(seconds: float) -> dict:
    whole = math.floor(seconds)
    return {"sec": int(whole), "nsec": int(round((seconds - whole) * 1e9)) % 1_000_000_000}


def read_truth(run: Path) -> list[list[float]]:
    """gz_pose.csv: simulation time, x, y, z, the orientation (w, x, y, z), the wall clock; the map frame."""
    path = run / "gz_pose.csv"
    if not path.is_file():
        return []
    rows = [[float(v) for v in row] for row in csv.reader(path.open()) if row and row[0][:1].isdigit()]
    return [row for row in rows if len(row) >= 9]


def occupied_points(snapshot: dict) -> bytes:
    """The occupied voxels' centres as packed little-endian float32 triples."""
    grid = snapshot["grid"]
    res = float(grid["resolution_m"])
    ox, oy, oz = (float(v) for v in grid["origin_m"])
    points = bytearray()
    for chunk in snapshot["chunks"]:
        cx, cy, cz = (int(v) * CHUNK for v in chunk["index"])
        for word_index, word_hex in enumerate(chunk["occupied_words_hex"]):
            word = int(word_hex, 16)
            while word:
                low = word & -word
                bit = word_index * 64 + low.bit_length() - 1
                word ^= low
                x, y, z = bit % CHUNK, (bit // CHUNK) % CHUNK, bit // (CHUNK * CHUNK)
                points += struct.pack("<fff", ox + (cx + x + 0.5) * res, oy + (cy + y + 0.5) * res,
                                      oz + (cz + z + 0.5) * res)
    return bytes(points)


def pose_stamped(data: bytes) -> tuple[float, tuple[float, float, float]]:
    """geometry_msgs/msg/PoseStamped in CDR: the header's stamp and the position."""
    sec, nanosec, length = struct.unpack_from("<iII", data, 4)
    offset = 16 + length
    offset += (-(offset - 4)) % 8
    x, y, z = struct.unpack_from("<ddd", data, offset)
    return sec + nanosec * 1e-9, (x, y, z)


def build(run: Path, out: Path) -> dict[str, int]:
    from mcap.writer import Writer  # the package of tools/view_flight.sh --install

    counts: dict[str, int] = {}
    truth = read_truth(run)
    with out.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile="", library="drone-gazebo tools/flight_records_mcap.py")
        schema_ids = {name: writer.register_schema(name=name, encoding="jsonschema",
                                                    data=json.dumps(schema).encode())
                      for name, schema in SCHEMAS.items()}
        channels: dict[str, int] = {}

        def publish(topic: str, schema: str, seconds: float, message: dict) -> None:
            if topic not in channels:
                channels[topic] = writer.register_channel(topic=topic, message_encoding="json",
                                                          schema_id=schema_ids[schema])
            nanoseconds = int(round(seconds * 1e9))
            writer.add_message(channels[topic], log_time=nanoseconds, publish_time=nanoseconds,
                               data=json.dumps(message, separators=(",", ":")).encode())
            counts[topic] = counts.get(topic, 0) + 1

        # The true pose, path and speed.
        path_poses: list[dict] = []
        last_pose = last_path = -math.inf
        previous: list[float] | None = None
        for row in truth:
            t, x, y, z, qw, qx, qy, qz = row[:8]
            if t - last_pose < 0.1:
                continue
            pose = {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}}
            publish("/records/truth/pose", "foxglove.PoseInFrame", t,
                    {"timestamp": stamp(t), "frame_id": "map", "pose": pose})
            if previous is not None and t > previous[0]:
                dt = t - previous[0]
                horizontal = math.hypot(x - previous[1], y - previous[2]) / dt
                vertical = (z - previous[3]) / dt
                publish("/records/truth/speed", "drone_gazebo.TruthSpeed", t,
                        {"speed_mps": math.hypot(horizontal, vertical), "horizontal_mps": horizontal,
                         "vertical_mps": vertical})
            previous = [t, x, y, z]
            last_pose = t
            if not path_poses or math.dist((x, y, z), tuple(path_poses[-1]["position"].values())) >= 0.5:
                path_poses.append(pose)
            if t - last_path >= 2.0 and len(path_poses) >= 2:
                publish("/records/truth/path", "foxglove.PosesInFrame", t,
                        {"timestamp": stamp(t), "frame_id": "map", "poses": path_poses[::max(1, len(path_poses) // 600)]})
                last_path = t

        # The obstacle memory's occupied voxels, every raw snapshot.
        snapshots = sorted(run.glob("raw_snapshot_3d_revision_*.json"), key=lambda p: int(p.stem.rsplit("_", 1)[1]))
        for index, path in enumerate(snapshots):
            snapshot = json.loads(path.read_text())
            header = snapshot["header"]["stamp"]
            t = header["sec"] + header["nanosec"] * 1e-9
            publish("/records/memory/occupied", "foxglove.PointCloud", t, {
                "timestamp": stamp(t), "frame_id": snapshot["header"].get("frame_id") or "map",
                "pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
                "point_stride": 12,
                "fields": [{"name": "x", "offset": 0, "type": 7}, {"name": "y", "offset": 4, "type": 7},
                           {"name": "z", "offset": 8, "type": 7}],
                "data": base64.b64encode(occupied_points(snapshot)).decode()})
            print(f"  memory snapshots {index + 1}/{len(snapshots)}", flush=True)

        # The speed laws' record.
        track = run / "mppi" / "mppi_track.jsonl"
        if track.is_file():
            for line in track.open():
                if not line.strip():
                    continue
                record = json.loads(line)
                publish("/records/mppi/track", "drone_gazebo.MppiTrack", record["stamp_ns"] * 1e-9, {
                    "reference_speed_mps": record.get("reference_speed_mps"),
                    "commanded_speed_mps": record.get("commanded_speed_mps"),
                    "actual_speed_mps": record.get("actual_speed_mps"),
                    "minimum_esdf_m": record.get("minimum_esdf_m"),
                    "constrained_clearance_m": record.get("constrained_clearance_m"),
                    "limiter": record.get("limiter") or ""})

        # The budgets: resources.csv is stamped on the wall clock, which gz_pose.csv ties to the simulation's.
        resources = run / "resources.csv"
        if resources.is_file() and truth:
            walls = [row[8] for row in truth]
            last: dict[str, float] = {}
            for row in csv.DictReader(resources.open()):
                wall = float(row["stamp_s"])
                i = bisect.bisect_left(walls, wall)
                if i <= 0 or i >= len(truth):
                    continue
                t = truth[i][0]
                name = row["name"]
                if name in BUDGET_PROCESSES and t - last.get(name, -math.inf) >= 1.0:
                    publish(f"/records/budget/{name}", "drone_gazebo.Budget", t,
                            {"cpu_cores": float(row["cpu_cores"]), "rss_mib": float(row["rss_bytes"]) / 1048576.0})
                    last[name] = t
                factor = row.get("real_time_factor", "nan")
                if factor not in ("nan", "") and t - last.get("host", -math.inf) >= 1.0:
                    publish("/records/budget/host", "drone_gazebo.HostBudget", t, {"real_time_factor": float(factor)})
                    last["host"] = t

        # The estimate against the truth, from the bag's own pose topic.
        bags = sorted((run / "stack_bag").glob("*.mcap"))
        if bags and truth:
            from mcap.reader import make_reader

            stamps = [row[0] for row in truth]
            with bags[0].open("rb") as bag:
                for _schema, channel, message in make_reader(bag).iter_messages(topics=list(ESTIMATE_TOPICS)):
                    t, (x, y, z) = pose_stamped(message.data)
                    i = bisect.bisect_left(stamps, t)
                    if i <= 0 or i >= len(truth):
                        continue
                    a, b = truth[i - 1], truth[i]
                    share = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                    tx, ty, tz = (a[k] + share * (b[k] - a[k]) for k in (1, 2, 3))
                    publish("/records/estimate/error", "drone_gazebo.EstimateError", message.log_time * 1e-9,
                            {"horizontal_m": math.hypot(x - tx, y - ty), "vertical_m": z - tz})
        writer.finish()
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="a run of log/runs, or a directory that holds a flight's records")
    parser.add_argument("--out", type=Path, help="the file to write (RUN/view/records.mcap unless given)")
    args = parser.parse_args()
    run = Path(args.run) if "/" in args.run else REPOSITORY / "log/runs" / args.run
    if not run.is_dir():
        print(f"{run}: no such run", file=sys.stderr)
        return 1
    out = args.out or run / "view" / "records.mcap"
    out.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    print(f"records of {run.name} start {time.strftime('%T')}", flush=True)
    counts = build(run, out)
    for topic in sorted(counts):
        print(f"  {topic:44s} {counts[topic]}")
    print(f"records of {run.name} end {time.strftime('%T')}: {sum(counts.values())} messages, "
          f"{out.stat().st_size / 1048576.0:.1f} MiB in {time.time() - started:.0f} s -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
