#!/usr/bin/env python3
"""Tests for the per-flight resource record."""

from __future__ import annotations

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import capture_process_resources as capture  # noqa: E402


class CaptureProcessResourcesTest(unittest.TestCase):
    def test_gpu_rows_become_utilisation_memory_and_memory_by_process_name(self) -> None:
        utilization, used, by_name = capture.parse_gpu_sample(
            [["37", "1843"]],
            [["/workspace/install/lib/drone_city_nav/production_mppi_node", "612"],
             ["/usr/bin/gz-sim-server", "1100"],
             ["/usr/bin/gz-sim-server", "[N/A]"]])

        self.assertEqual(utilization, 37.0)
        self.assertEqual(used, 1843 * 1024 * 1024)
        self.assertEqual(by_name, {"production_mppi_node": 612 * 1024 * 1024,
                                   "gz-sim-server": 1100 * 1024 * 1024})

    def test_no_gpu_leaves_the_sample_empty(self) -> None:
        utilization, used, by_name = capture.parse_gpu_sample([], [])

        self.assertTrue(math.isnan(utilization))
        self.assertEqual(used, 0)
        self.assertEqual(by_name, {})

    def test_an_interpreter_is_named_after_its_script(self) -> None:
        self.assertEqual(
            capture.process_label("python3", ["python3", "/w/scripts/capture_gazebo_pose.py",
                                              "out.csv", "--world", "urban"]),
            "capture_gazebo_pose.py")
        self.assertEqual(capture.process_label("python3", ["python3", "-u", "/w/s.py"]),
                         "s.py")
        self.assertEqual(capture.process_label("production_mppi_node",
                                               ["/w/production_mppi_node", "--ros-args"]),
                         "production_mppi_node")
        self.assertEqual(capture.process_label("bash", ["bash"]), "bash")
        self.assertEqual(capture.process_label("bash", ["bash", "-c", "sleep 1; ls"]), "bash")
        self.assertEqual(capture.process_label("ruby", ["ruby", "", "-s"]), "ruby")

    def test_the_record_is_rewritten_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "resources.csv")
            capture.write_atomically(output, [("1.0", 7, "node", "0.500", 10, 3,
                                               "12.0", 20, 30, 40, 50, "0.9800")])
            capture.write_atomically(output, [("1.0", 7, "node", "0.500", 10, 3,
                                               "12.0", 20, 30, 40, 50, "0.9800"),
                                              ("2.0", 7, "node", "0.600", 11, 3,
                                               "13.0", 21, 31, 41, 51, "0.9700")])

            with open(output, newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], list(capture.COLUMNS))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[2][3], "0.600")
            self.assertFalse(Path(output + ".tmp").exists())


if __name__ == "__main__":
    unittest.main()
