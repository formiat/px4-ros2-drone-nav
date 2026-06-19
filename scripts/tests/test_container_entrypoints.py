#!/usr/bin/env python3
"""Static tests for container entrypoint scripts."""

from __future__ import annotations

import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1]


class ContainerEntrypointTest(unittest.TestCase):
    def read_script(self, name: str) -> str:
        return (SCRIPTS_DIR / name).read_text(encoding="utf-8")

    def test_dev_shell_uses_shared_container_runner(self) -> None:
        text = self.read_script("dev_shell.sh")

        self.assertIn('exec "${repo_root}/scripts/container_run.sh" "$@"', text)
        self.assertNotIn("docker run", text)

    def test_common_wrappers_use_shared_container_runner(self) -> None:
        expected_targets = {
            "build.sh": "make build",
            "test.sh": "make test",
            "sim_gui.sh": "make sim-gui",
            "sim_headless.sh": "make sim-headless",
        }

        for script_name, make_target in expected_targets.items():
            with self.subTest(script_name=script_name):
                text = self.read_script(script_name)
                self.assertIn(
                    f'exec "${{repo_root}}/scripts/container_run.sh" {make_target}',
                    text,
                )
                self.assertNotIn("docker run", text)

    def test_container_runner_owns_docker_invocation(self) -> None:
        text = self.read_script("container_run.sh")

        self.assertIn("docker run", text)
        self.assertIn("--volume \"${repo_root}:/workspace:rw\"", text)
        self.assertIn('bash -lc "${container_command}" bash "$@"', text)


if __name__ == "__main__":
    unittest.main()
