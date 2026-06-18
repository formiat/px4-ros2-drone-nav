#!/usr/bin/env python3
"""Tests for Gazebo stale process cleanup selection."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "gazebo_process_cleanup.py"
SPEC = importlib.util.spec_from_file_location("gazebo_process_cleanup", SCRIPT_PATH)
assert SPEC is not None
cleanup = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = cleanup
assert SPEC.loader is not None
SPEC.loader.exec_module(cleanup)


class GazeboProcessCleanupTest(unittest.TestCase):
    def test_selects_stale_server_and_gui_processes(self) -> None:
        processes = cleanup.parse_ps_output(
            "\n".join(
                [
                    "100 1 100 /usr/bin/gz sim -r -s generated_city.sdf",
                    "101 1 101 gz sim -g",
                    "102 1 102 python3 unrelated.py --arg gazebo",
                ]
            )
        )

        candidates = cleanup.select_conflicting_processes(processes, {999})

        self.assertEqual([process.pid for process in candidates], [100, 101])

    def test_protects_current_process_and_ancestors(self) -> None:
        processes = cleanup.parse_ps_output(
            "\n".join(
                [
                    "10 1 10 /bin/bash parent",
                    "20 10 20 /bin/bash runner",
                    "30 20 30 /usr/bin/gz sim -g",
                    "40 1 40 /usr/bin/gz sim -s old_world.sdf",
                ]
            )
        )

        candidates = cleanup.select_conflicting_processes(processes, {30})

        self.assertEqual([process.pid for process in candidates], [40])

    def test_ignores_unrelated_commands_with_gazebo_words(self) -> None:
        self.assertFalse(
            cleanup.is_conflicting_gazebo_process(
                "python3 note.py --message 'please run gz sim later'"
            )
        )
        self.assertFalse(cleanup.is_conflicting_gazebo_process("gazebo-helper"))
        self.assertTrue(cleanup.is_conflicting_gazebo_process("/opt/gz sim -g"))

    def test_parse_invalid_lines_is_tolerant(self) -> None:
        processes = cleanup.parse_ps_output(
            "\n".join(
                [
                    "not-a-process",
                    "200 1 200 /usr/bin/gz sim -g",
                ]
            )
        )

        self.assertEqual(len(processes), 1)
        self.assertEqual(processes[0].pid, 200)


if __name__ == "__main__":
    unittest.main()
