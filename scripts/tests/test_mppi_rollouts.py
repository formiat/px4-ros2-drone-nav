#!/usr/bin/env python3
"""Roadmap item 28 step 1: the MPPI controller's rollouts set for one flight
from the environment (MPPI_ROLLOUTS), through the run script and the launch
to the controller's parameters, the open-space count three quarters of it,
and the count recorded in the manifest."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "drone_city_nav" / "launch"))

from launch_overrides import rollouts_overrides  # noqa: E402


class MppiRolloutsContractTest(unittest.TestCase):
    def test_the_count_keeps_the_budgets_proportion(self) -> None:
        self.assertEqual({}, rollouts_overrides(""))
        self.assertEqual({"rollouts": 8192, "open_static_rollouts": 6144}, rollouts_overrides("8192"))
        self.assertEqual({"rollouts": 1024, "open_static_rollouts": 768}, rollouts_overrides(" 1024 "))
        with self.assertRaises(RuntimeError):
            rollouts_overrides("0")
        with self.assertRaises(ValueError):
            rollouts_overrides("many")

    def test_the_variable_reaches_the_controller_and_the_manifest(self) -> None:
        self.assertIn('[[ -n "${MPPI_ROLLOUTS:-}" ]] && ros_launch_args+=(mppi_rollouts:="${MPPI_ROLLOUTS}")',
                      (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text())
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text()
        self.assertIn('LaunchConfiguration("mppi_rollouts")', launch)
        self.assertIn("production_mppi_parameters.append(rollouts_overrides(", launch)
        self.assertIn("  MPPI_ROLLOUTS\n", (REPOSITORY / "scripts/container_run.sh").read_text())
        self.assertIn("SMOKE_SEED MPPI_ROLLOUTS SMOKE_FACE FLIGHT_CPUSET; do",
                      (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text())


if __name__ == "__main__":
    unittest.main()
