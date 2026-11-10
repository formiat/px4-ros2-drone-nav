#!/usr/bin/env python3
"""Roadmap item 42: a flight's record in a viewer without the stack. The layouts of tools/viewer read topics the
stack bag records and topics tools/flight_records_mcap.py writes; the records file of a synthetic run holds the true
pose, the memory's voxels and the budgets on the simulation clock (where the `mcap` package is installed); the
command refuses a directory without a bag and names its tools in the README."""
from __future__ import annotations

import base64
import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "tools"))
sys.path.insert(0, str(REPOSITORY / "scripts"))

import flight_records_mcap  # noqa: E402
import stack_bag_topics  # noqa: E402

try:
    import mcap  # noqa: F401
    HAVE_MCAP = True
except ImportError:
    HAVE_MCAP = False


def topic_of(path: str) -> str:
    """The topic of a message path: `/topic.field` without its fields."""
    return path.split(".", 1)[0]


class ViewFlightTest(unittest.TestCase):
    def layouts(self) -> dict[str, dict]:
        return {name: json.loads((REPOSITORY / "tools/viewer" / f"{name}.json").read_text(encoding="utf-8"))
                for name in ("stereo_tof", "lidar")}

    def test_the_layouts_read_what_the_bag_and_the_records_hold(self) -> None:
        recorded = set(stack_bag_topics.OUTPUTS)
        for profile in ("stereo_tof", "lidar"):
            recorded |= set(stack_bag_topics.inputs(profile, profile if profile == "stereo_tof" else "none"))
        written = set(flight_records_mcap.TOPICS) | {f"/records/budget/{name}"
                                                    for name in flight_records_mcap.BUDGET_PROCESSES}
        for name, layout in self.layouts().items():
            config = layout["configById"]
            self.assertTrue(any(key.startswith("3D!") for key in config), name)
            topics: set[str] = set()
            for key, panel in config.items():
                for path in panel.get("paths", []):
                    topics.add(topic_of(path["value"]))
                if key.startswith("3D!"):
                    topics |= set(panel["topics"])
                if panel.get("imageMode", {}).get("imageTopic"):
                    topics.add(panel["imageMode"]["imageTopic"])
                if panel.get("topicPath"):
                    topics.add(topic_of(panel["topicPath"]))
            for topic in sorted(topics):
                self.assertIn(topic, written if topic.startswith("/records/") else recorded, f"{name}: {topic}")
            # The panels the item names: the memory with the poses, the events, the speeds, the budgets.
            scene = next(panel for key, panel in config.items() if key.startswith("3D!"))
            self.assertIn("/records/memory/occupied", scene["topics"])
            self.assertIn("/records/truth/pose", scene["topics"])
            self.assertTrue(any(key.startswith("StateTransitions!") for key in config), name)
            self.assertTrue(any("/records/budget/" in path["value"] for panel in config.values()
                                for path in panel.get("paths", [])), name)
        self.assertIn("/stereo/left/image", json.dumps(self.layouts()["stereo_tof"]))
        self.assertIn("/lidar_3d/points", json.dumps(self.layouts()["lidar"]))

    def test_the_command_and_its_tools_are_documented(self) -> None:
        readme = (REPOSITORY / "tools/README.md").read_text(encoding="utf-8")
        for tool in ("view_flight.sh", "flight_records_mcap.py"):
            self.assertIn(f"`{tool}`", readme)
        script = (REPOSITORY / "tools/view_flight.sh").read_text(encoding="utf-8")
        self.assertIn("tools/flight_records_mcap.py", script)
        self.assertIn('--defaultLayout="$layout"', script)
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(["bash", str(REPOSITORY / "tools/view_flight.sh"), directory],
                                    capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 1)
        self.assertIn("no stack bag", result.stderr)

    def test_the_memorys_voxels_are_points_at_their_centres(self) -> None:
        # One chunk at (1, 0, 2) with the voxel (3, 4, 5) occupied: bit (5 * 16 + 4) * 16 + 3 of the chunk.
        bit = (5 * 16 + 4) * 16 + 3
        words = ["0" * 16] * 64
        words[bit // 64] = f"{1 << (bit % 64):016x}"
        snapshot = {"grid": {"resolution_m": 0.25, "origin_m": [-30.0, -30.0, 0.0]},
                    "chunks": [{"index": [1, 0, 2], "occupied_words_hex": words}]}
        points = flight_records_mcap.occupied_points(snapshot)
        self.assertEqual(len(points), 12)
        x, y, z = struct.unpack("<fff", points)
        self.assertAlmostEqual(x, -30.0 + (16 + 3 + 0.5) * 0.25, places=5)
        self.assertAlmostEqual(y, -30.0 + (4 + 0.5) * 0.25, places=5)
        self.assertAlmostEqual(z, (32 + 5 + 0.5) * 0.25, places=5)

    def test_a_pose_is_read_from_the_bags_bytes(self) -> None:
        # geometry_msgs/msg/PoseStamped in CDR: the encapsulation, the stamp, "map", the padding, seven doubles.
        data = (b"\x00\x01\x00\x00" + struct.pack("<iI", 12, 500_000_000) + struct.pack("<I", 4) + b"map\x00"
                + struct.pack("<7d", 1.5, -2.5, 3.5, 0.0, 0.0, 0.0, 1.0))
        stamp, position = flight_records_mcap.pose_stamped(data)
        self.assertAlmostEqual(stamp, 12.5)
        self.assertEqual(position, (1.5, -2.5, 3.5))

    @unittest.skipUnless(HAVE_MCAP, "the mcap package is the viewer's, not the container's")
    def test_a_runs_records_are_written_on_the_simulation_clock(self) -> None:
        from mcap.reader import make_reader

        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "r1"
            (run / "mppi").mkdir(parents=True)
            with (run / "gz_pose.csv").open("w") as stream:
                for step in range(100):
                    t = 10.0 + 0.05 * step
                    stream.write(f"{t},{1.0 + 0.1 * step},2.0,3.0,1.0,0.0,0.0,0.0,{1000.0 + 0.05 * step}\n")
            words = ["0" * 16] * 64
            words[0] = f"{1:016x}"
            (run / "raw_snapshot_3d_revision_7.json").write_text(json.dumps({
                "header": {"frame_id": "map", "stamp": {"sec": 12, "nanosec": 0}},
                "grid": {"resolution_m": 0.25, "origin_m": [0.0, 0.0, 0.0]},
                "chunks": [{"index": [0, 0, 0], "occupied_words_hex": words}]}))
            (run / "mppi/mppi_track.jsonl").write_text(json.dumps({
                "stamp_ns": 11_000_000_000, "reference_speed_mps": 2.0, "commanded_speed_mps": 1.9,
                "actual_speed_mps": 1.8, "minimum_esdf_m": 0.7, "constrained_clearance_m": 0.4,
                "limiter": "clearance"}) + "\n")
            (run / "resources.csv").write_text(
                "stamp_s,pid,name,cpu_cores,rss_bytes,threads,real_time_factor\n"
                "1002.0,5,production_mppi_node,0.75,104857600,8,0.8\n")
            out = run / "view/records.mcap"
            out.parent.mkdir()
            counts = flight_records_mcap.build(run, out)
            self.assertEqual(counts["/records/memory/occupied"], 1)
            self.assertEqual(counts["/records/mppi/track"], 1)
            self.assertEqual(counts["/records/budget/production_mppi_node"], 1)
            self.assertGreaterEqual(counts["/records/truth/pose"], 30)
            with out.open("rb") as stream:
                messages = {channel.topic: (message, schema) for schema, channel, message
                            in make_reader(stream).iter_messages()}
            cloud, schema = messages["/records/memory/occupied"]
            self.assertEqual(schema.name, "foxglove.PointCloud")
            self.assertEqual(cloud.log_time, 12_000_000_000)
            point = struct.unpack("<fff", base64.b64decode(json.loads(cloud.data)["data"]))
            self.assertEqual(point, (0.125, 0.125, 0.125))
            budget, _ = messages["/records/budget/production_mppi_node"]
            # The wall clock 1002.0 is the simulation's 12.0 by the true pose's two clocks.
            self.assertAlmostEqual(budget.log_time * 1e-9, 12.0, places=1)
            self.assertEqual(json.loads(budget.data), {"cpu_cores": 0.75, "rss_mib": 100.0})
            speed, _ = messages["/records/truth/speed"]
            self.assertAlmostEqual(json.loads(speed.data)["speed_mps"], 2.0, places=3)


if __name__ == "__main__":
    unittest.main()
