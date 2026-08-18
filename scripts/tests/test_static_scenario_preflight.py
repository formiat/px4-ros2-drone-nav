#!/usr/bin/env python3
"""Contract tests for optional static scenario preflight execution."""

from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1] / "run_static_scenario_preflight.sh"
)


class StaticScenarioPreflightTest(unittest.TestCase):
    def run_helper(self, enabled: str | None, *command: str) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        if enabled is None:
            environment.pop("ENABLE_STATIC_SCENARIO_PREFLIGHT", None)
        else:
            environment["ENABLE_STATIC_SCENARIO_PREFLIGHT"] = enabled
        return subprocess.run(
            [str(SCRIPT), *command],
            check=False,
            capture_output=True,
            encoding="utf-8",
            env=environment,
        )

    def test_default_skips_the_preflight_command(self) -> None:
        result = self.run_helper(None, "bash", "-c", "exit 19")

        self.assertEqual(result.returncode, 0)
        self.assertIn("status=skipped enabled=false", result.stdout)

    def test_enabled_runs_and_propagates_the_command_status(self) -> None:
        result = self.run_helper("true", "bash", "-c", "exit 19")

        self.assertEqual(result.returncode, 19)
        self.assertIn("status=running enabled=true", result.stdout)

    def test_invalid_value_is_rejected(self) -> None:
        result = self.run_helper("sometimes", "bash", "-c", "exit 0")

        self.assertEqual(result.returncode, 2)
        self.assertIn("must be a boolean", result.stderr)


if __name__ == "__main__":
    unittest.main()
