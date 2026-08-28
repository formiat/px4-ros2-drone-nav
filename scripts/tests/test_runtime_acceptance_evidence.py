#!/usr/bin/env python3
"""Tests for reproducible runtime manifests and exact raw-volume capture."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1]
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import capture_raw_snapshot_3d as capture  # noqa: E402
import runtime_manifest  # noqa: E402
import validate_drone_nav_headless as validator  # noqa: E402


RUNNER = SCRIPTS / "run_drone_nav_sim.sh"
RUNTIME_HELPER = SCRIPTS / "multi_vehicle_sim_runtime.sh"
EVIDENCE_RUNTIME = SCRIPTS / "runtime_evidence_runtime.sh"
CONTAINER_RUNNER = SCRIPTS / "container_run.sh"


def words_with_bits(*indices: int) -> list[int]:
    words = [0] * 64
    for index in indices:
        words[index // 64] |= 1 << (index % 64)
    return words


def local_bit(x: int, y: int, z: int) -> int:
    return (z * 16 + y) * 16 + x


def snapshot_message(revision: int = 7) -> SimpleNamespace:
    inside_free = local_bit(1, 1, 1)
    inside_occupied = local_bit(2, 1, 1)
    outside = local_bit(10, 10, 1)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34), frame_id="map"
        ),
        producer_instance_id=99,
        obstacle_snapshot_revision=revision,
        origin_x_m=0.0,
        origin_y_m=0.0,
        origin_z_m=0.0,
        resolution_m=1.0,
        width_cells=32,
        height_cells=32,
        depth_cells=16,
        chunk_size_cells=16,
        chunks=[
            SimpleNamespace(
                x=0,
                y=0,
                z=0,
                observed_words=words_with_bits(
                    inside_free, inside_occupied, outside
                ),
                occupied_words=words_with_bits(inside_occupied, outside),
            )
        ],
    )


class RawSnapshotCaptureTest(unittest.TestCase):
    def test_capture_masks_exact_bits_outside_the_requested_volume(self) -> None:
        document = capture.snapshot_document(
            snapshot_message(), (0.0, 0.0, 0.0, 4.0, 4.0, 4.0)
        )

        self.assertIsNotNone(document)
        assert document is not None
        self.assertEqual(document["selection"]["observed_voxel_count"], 2)
        self.assertEqual(document["selection"]["occupied_voxel_count"], 1)
        observed = [
            int(word, 16) for word in document["chunks"][0]["observed_words_hex"]
        ]
        occupied = [
            int(word, 16) for word in document["chunks"][0]["occupied_words_hex"]
        ]
        self.assertTrue(observed[local_bit(1, 1, 1) // 64])
        self.assertEqual(
            observed[local_bit(10, 10, 1) // 64]
            & (1 << (local_bit(10, 10, 1) % 64)),
            0,
        )
        self.assertTrue(
            all(
                not (occupied_word & ~observed_word)
                for observed_word, occupied_word in zip(
                    observed, occupied, strict=True
                )
            )
        )

    def test_capture_waits_for_observed_occupied_problem_geometry(self) -> None:
        message = snapshot_message()
        message.chunks[0].occupied_words = [0] * 64

        self.assertIsNone(
            capture.snapshot_document(message, (0.0, 0.0, 0.0, 4.0, 4.0, 4.0))
        )


class RuntimeManifestTest(unittest.TestCase):
    def test_raw_snapshot_update_is_atomic_and_monotonic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest_path = Path(directory) / "manifest.json"
            runtime_manifest.atomic_write_json(
                manifest_path,
                {
                    "schema": runtime_manifest.SCHEMA,
                    "raw_snapshot": {"status": "pending"},
                },
            )
            runtime_manifest.update_raw_snapshot_record(
                manifest_path, {"status": "captured", "revision": 8}
            )
            runtime_manifest.update_raw_snapshot_record(
                manifest_path, {"status": "captured", "revision": 7}
            )

            document = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(document["raw_snapshot"]["revision"], 8)

    def test_acceptance_metrics_enforce_availability_latency_owner_and_reserve(
        self,
    ) -> None:
        log = (
            "PRODUCTION_MPPI_GUIDE3D certified_pending=true "
            "certified_reserve=sufficient reserve_available_m=20.0 "
            "reserve_required_m=19.0\n"
            "PRODUCTION_MPPI_SUMMARY ticks=1000 ownership_gap_ticks=0 "
            "post_bootstrap_route_observations=990 "
            "post_bootstrap_route_available_ticks=989 "
            "post_bootstrap_route_availability_ratio=0.998990 "
            "post_bootstrap_no_executable_route_hold_ticks=1 "
            "planner_latency_samples=12 planner_p95_ms=81.5 "
            "planner_p99_ms=92.0 planner_build_and_planning_p99_ms=101.0\n"
        )
        errors: list[str] = []

        validator.validate_persistent_3d_acceptance_metrics(log, errors)

        self.assertEqual(errors, [])

    def test_acceptance_metrics_reject_an_ownership_gap_and_short_reserve(self) -> None:
        log = (
            "PRODUCTION_MPPI_GUIDE3D certified_pending=true "
            "certified_reserve=insufficient reserve_available_m=10.0 "
            "reserve_required_m=20.0\n"
            "PRODUCTION_MPPI_SUMMARY ticks=100 ownership_gap_ticks=1 "
            "post_bootstrap_route_observations=80 "
            "post_bootstrap_route_available_ticks=70 "
            "post_bootstrap_route_availability_ratio=0.875 "
            "post_bootstrap_no_executable_route_hold_ticks=10 "
            "planner_latency_samples=5 planner_p95_ms=250.0 "
            "planner_p99_ms=240.0 planner_build_and_planning_p99_ms=230.0\n"
        )
        errors: list[str] = []

        validator.validate_persistent_3d_acceptance_metrics(log, errors)

        self.assertGreaterEqual(len(errors), 5)

    def test_acceptance_metrics_reject_a_ratio_that_disagrees_with_ticks(self) -> None:
        log = (
            "PRODUCTION_MPPI_GUIDE3D certified_pending=true "
            "certified_reserve=terminal_exempt reserve_available_m=0.0 "
            "reserve_required_m=0.0\n"
            "PRODUCTION_MPPI_SUMMARY ownership_gap_ticks=0 "
            "post_bootstrap_route_observations=1000 "
            "post_bootstrap_route_available_ticks=980 "
            "post_bootstrap_route_availability_ratio=1.0 "
            "post_bootstrap_no_executable_route_hold_ticks=0 "
            "planner_latency_samples=2 planner_p95_ms=20.0 "
            "planner_p99_ms=25.0 planner_build_and_planning_p99_ms=30.0\n"
        )
        errors: list[str] = []

        validator.validate_persistent_3d_acceptance_metrics(log, errors)

        self.assertTrue(any("ratio matches" in error for error in errors))

    def test_simulator_wires_per_run_manifest_capture_and_strict_gate(self) -> None:
        runner = RUNNER.read_text(encoding="utf-8")
        evidence_runtime = EVIDENCE_RUNTIME.read_text(encoding="utf-8")
        helper = RUNTIME_HELPER.read_text(encoding="utf-8")
        container = CONTAINER_RUNNER.read_text(encoding="utf-8")

        self.assertIn("runtime_evidence_runtime.sh", runner)
        self.assertIn("runtime_manifest.py", evidence_runtime)
        self.assertIn("capture_raw_snapshot_3d.py", evidence_runtime)
        self.assertIn('runs/${run_id}', evidence_runtime)
        self.assertIn('--runtime-manifest "${runtime_manifest_path}"', helper)
        self.assertIn("--require-persistent-3d-acceptance", helper)
        self.assertIn("DRONE_GAZEBO_RUN_ID", container)


if __name__ == "__main__":
    unittest.main()
