#!/usr/bin/env python3
"""Contracts for the sequential incremental-topology mission matrix."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
RUNNER = REPOSITORY / "scripts/run_incremental_topology_validation.sh"
WRAPPER = REPOSITORY / "scripts/validate_incremental_topology_headless.sh"
MAKEFILE = REPOSITORY / "Makefile"


class IncrementalTopologyValidationContractTest(unittest.TestCase):
    def test_matrix_is_sequential_and_covers_all_required_missions(self) -> None:
        runner = RUNNER.read_text(encoding="utf-8")
        stages = (
            "manhattan_low_altitude_smoke",
            "manhattan_four_waypoints",
            "manhattan_cooperative",
            "urban_point_to_point",
            "urban_cooperative",
        )

        offsets = [runner.index(f"run_stage {stage}") for stage in stages]

        self.assertEqual(offsets, sorted(offsets))
        self.assertEqual(runner.count("run_stage "), len(stages))
        self.assertIsNone(re.search(r"(?:^|\n)[^\n]*\s&\s*(?:\n|$)", runner))
        self.assertIn("ENABLE_STATIC_MAP=false", runner)
        self.assertIn("LIDAR_PROFILE=3d", runner)
        self.assertIn("REQUIRE_INCREMENTAL_TOPOLOGY_EVIDENCE=true", runner)
        self.assertIn("REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true", runner)
        self.assertIn("manhattan_low_altitude_point_to_point_scenario.json", runner)
        self.assertIn("MISSION_GOALS_XYZ_M=", runner)
        self.assertIn('MANHATTAN_POINT_TO_POINT_TIMEOUT_S:-1800', runner)
        self.assertIn("cooperative_traffic_scenario.json", runner)
        self.assertIn('MANHATTAN_COOPERATIVE_MISSION_TIMEOUT_S:-1200', runner)
        self.assertIn('MANHATTAN_COOPERATIVE_TIMEOUT_S:-1300', runner)
        self.assertIn("sim-urban-point-to-point-headless", runner)
        self.assertIn("sim-cooperative-traffic-urban-headless", runner)
        self.assertIn('URBAN_COOPERATIVE_MISSION_TIMEOUT_S:-780', runner)

    def test_host_wrapper_uses_one_container_for_the_complete_matrix(self) -> None:
        wrapper = WRAPPER.read_text(encoding="utf-8")
        makefile = MAKEFILE.read_text(encoding="utf-8")

        self.assertEqual(wrapper.count("container_run.sh"), 1)
        self.assertIn("make validate-incremental-topology-headless", wrapper)
        self.assertIn("validate-incremental-topology-headless:", makefile)
        self.assertIn("run_incremental_topology_validation.sh", makefile)


if __name__ == "__main__":
    unittest.main()
