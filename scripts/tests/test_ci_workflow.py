#!/usr/bin/env python3
"""The CI jobs (roadmap item 46): pinned, read-only, and running the workstation's own targets."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
WORKFLOW = REPOSITORY / ".github" / "workflows" / "ci.yml"
MAKEFILE = REPOSITORY / "Makefile"


class CiWorkflowTest(unittest.TestCase):
    def setUp(self) -> None:
        self.text = WORKFLOW.read_text(encoding="utf-8")
        self.workflow = yaml.safe_load(self.text)

    def test_every_action_is_pinned_by_its_commit(self) -> None:
        uses = re.findall(r"^\s*-?\s*uses:\s*(\S+)", self.text, flags=re.MULTILINE)
        self.assertTrue(uses)
        for action in uses:
            with self.subTest(action=action):
                self.assertRegex(action, r"^[\w.-]+/[\w.-]+@[0-9a-f]{40}$")

    def test_the_jobs_read_the_repository_and_nothing_more(self) -> None:
        self.assertEqual({"contents": "read"}, self.workflow["permissions"])
        self.assertNotIn("secrets.", self.text)
        for name, job in self.workflow["jobs"].items():
            with self.subTest(job=name):
                self.assertNotIn("permissions", job)

    def test_the_jobs_run_on_push_and_pull_request_one_run_a_branch(self) -> None:
        # PyYAML reads the key "on" as the boolean true.
        triggers = self.workflow.get("on", self.workflow.get(True))
        self.assertEqual({"push", "pull_request"}, set(triggers))
        self.assertTrue(self.workflow["concurrency"]["cancel-in-progress"])
        self.assertIn("github.ref", self.workflow["concurrency"]["group"])

    def test_the_x86_64_job_runs_the_gates_targets_and_the_arm64_job_the_onboard_build(self) -> None:
        targets = set(re.findall(r"^\.PHONY:\s*(\S+)", MAKEFILE.read_text(encoding="utf-8"), re.MULTILINE))
        jobs = self.workflow["jobs"]
        self.assertEqual({"x86-64", "arm64"}, set(jobs))
        self.assertEqual("ubuntu-24.04", jobs["x86-64"]["runs-on"])
        self.assertEqual("ubuntu-24.04-arm", jobs["arm64"]["runs-on"])
        called = [target for step in jobs["x86-64"]["steps"]
                  for target in re.findall(r"make ([\w-]+)", step.get("run", ""))]
        self.assertEqual(["build", "test", "test-scripts"], called)
        runs = "".join(step.get("run", "") for step in jobs["x86-64"]["steps"])
        # The format of the whole tree, the quality checks scoped by the push's or the pull request's base.
        self.assertIn("./scripts/check_cpp_quality.sh --all --format --no-build --no-test", runs)
        self.assertIn('--env QUALITY_BASE="$BASE"', runs)
        self.assertIn("./scripts/check_cpp_quality.sh --tidy --cppcheck", runs)
        self.assertEqual(0, jobs["x86-64"]["steps"][0]["with"]["fetch-depth"])
        # The two unit tests that need a CUDA device, and no others, are left out on a runner without one.
        self.assertIn(
            'make test CTEST_EXCLUDE="^mppi_control_sequence_test_current|^mppi_engine_parity_test"', runs
        )
        self.assertIn("$(if $(CTEST_EXCLUDE),--exclude-regex '$(CTEST_EXCLUDE)')",
                      MAKEFILE.read_text(encoding="utf-8"))
        for target in called:
            with self.subTest(target=target):
                self.assertIn(target, targets)
        arm64 = [step.get("run", "") for step in jobs["arm64"]["steps"]]
        self.assertIn("tools/onboard_image.sh --arm64", arm64)
        for name, job in jobs.items():
            with self.subTest(job=name):
                runs = [step.get("run", "") for step in job["steps"]]
                self.assertIn("./scripts/setup_px4_ros2_interface_lib.sh", runs)
                self.assertLessEqual(job["timeout-minutes"], 360)


if __name__ == "__main__":
    unittest.main()
