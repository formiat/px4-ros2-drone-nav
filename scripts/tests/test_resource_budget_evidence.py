#!/usr/bin/env python3
"""Tests for the resource-budget checks of the headless mission check."""

from __future__ import annotations

import csv
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import resource_budget_evidence as evidence  # noqa: E402

COLUMNS = ("stamp_s", "pid", "name", "cpu_cores", "rss_bytes", "threads",
           "gpu_utilization_percent", "gpu_memory_used_bytes", "gpu_process_memory_bytes",
           "cgroup_cpu_usec", "cgroup_memory_bytes", "real_time_factor")
MIB = 1024 * 1024


def flight_log(started_s: float, finished_s: float) -> str:
    return (f"[{started_s:.3f}] [mission_monitor_node]: MISSION_READINESS ready=true\n"
            f"[{finished_s:.3f}] [mission_monitor_node]: MISSION_RESULT success=true\n")


def write_record(directory: Path, seconds: range, growth_bytes: int = 0,
                 skip_every: int = 0) -> None:
    """A flight of two onboard processes and the simulator, one sample a
    second from 1000 s, the controller's memory growing by growth_bytes."""
    with (directory / "resources.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        for second in seconds:
            if skip_every and second % skip_every == 0:
                continue
            stamp = 1000.0 + second
            share = second / max(1, len(seconds) - 1)
            writer.writerow([f"{stamp:.3f}", 5, "production_mppi_node", "1.200",
                             400 * MIB + int(growth_bytes * share), 9, "35.0", 1500 * MIB,
                             600 * MIB, 0, 0, "0.9500"])
            writer.writerow([f"{stamp:.3f}", 3, "obstacle_memory_3d_node", "0.400",
                             300 * MIB, 5, "35.0", 1500 * MIB, 0, 0, 0, "0.9500"])
            writer.writerow([f"{stamp:.3f}", 2, "gz-sim-server", "3.000", 2000 * MIB, 40,
                             "35.0", 1500 * MIB, 800 * MIB, 0, 0, "0.9500"])
            writer.writerow([f"{stamp:.3f}", 9, "capture_gazebo_pose.py", "0.050",
                             50 * MIB, 2, "35.0", 1500 * MIB, 0, 0, 0, "0.9500"])
    (directory / "resources_host.json").write_text(
        json.dumps({"cpu_model": "Test CPU", "gpu_name": "Test GPU"}), encoding="utf-8")


def run_check(directory: Path, log: str) -> tuple[list[str], str]:
    errors: list[str] = []
    output = io.StringIO()
    with redirect_stdout(output):
        evidence.validate_resource_budget(directory, log, errors)
    return errors, output.getvalue()


class ResourceBudgetEvidenceTest(unittest.TestCase):
    def test_a_missing_record_fails(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            errors, _ = run_check(Path(directory), flight_log(1000.0, 1100.0))
        self.assertEqual(len(errors), 1)
        self.assertIn("records its process resources", errors[0])

    def test_a_record_that_covers_the_flight_reports_the_onboard_usage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            write_record(Path(directory), range(0, 120), growth_bytes=64 * MIB)
            errors, output = run_check(Path(directory), flight_log(1005.0, 1105.0))
        self.assertEqual(errors, [])
        self.assertIn("OK: the resource record covers 100% of the flight", output)
        self.assertIn("OK: production_mppi_node uses 1.20 cores at p50", output)
        self.assertIn("RSS 454 MiB at p95, +48 MiB over the flight; 9 threads", output)
        self.assertIn("OK: mppi_offboard_node is not in the resource record", output)
        self.assertIn("OK: onboard processes use 1.60 cores at p50, 1.60 at p95", output)
        self.assertIn("OK: captures use 0.05 cores at p50", output)
        self.assertIn("OK: simulator and harness use 3.00 cores at p50", output)
        self.assertIn("OK: GPU utilisation is 35% at p50, 35% at p95; memory used 1500 MiB "
                      "at p95, of which production_mppi_node 600 MiB", output)
        self.assertIn("OK: real-time factor is 0.95 at p50, 0.95 at least", output)

    def test_a_record_with_gaps_fails_the_coverage_check(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            write_record(Path(directory), range(0, 120), skip_every=5)
            errors, _ = run_check(Path(directory), flight_log(1005.0, 1105.0))
        self.assertEqual(len(errors), 1)
        self.assertIn("covers at least 90% of the flight (80%)", errors[0])

    def test_an_onboard_process_that_keeps_growing_fails(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            write_record(Path(directory), range(0, 120), growth_bytes=400 * MIB)
            errors, output = run_check(Path(directory), flight_log(1005.0, 1105.0))
        self.assertEqual(len(errors), 1)
        self.assertIn("production_mppi_node gains at most 256 MiB", errors[0])
        self.assertIn("(+303 MiB)", errors[0])
        self.assertIn("OK: obstacle_memory_3d_node uses", output)

    def test_without_a_successful_flight_nothing_is_measured(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            write_record(Path(directory), range(0, 20))
            errors, output = run_check(Path(directory), "no mission lines")
        self.assertEqual(errors, [])
        self.assertIn("not measured without a successful flight", output)

    def test_memory_growth_is_the_last_tenth_against_the_first(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            write_record(Path(directory), range(0, 100), growth_bytes=100 * MIB)
            record = evidence.load_resource_record(Path(directory) / "resources.csv")
        usage = evidence.process_usage(record, "production_mppi_node", (1000.0, 1099.0))
        self.assertIsNotNone(usage)
        assert usage is not None
        self.assertAlmostEqual(usage.rss_growth_bytes / MIB, 90.0, delta=1.5)
        self.assertEqual(usage.samples, 100)


if __name__ == "__main__":
    unittest.main()
