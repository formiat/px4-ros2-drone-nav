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
                    "103 1 103 python3 note.py --message please run gz sim later",
                    "104 1 104 rg gz sim scripts/run_city_mvp.sh",
                    "105 1 105 bash -lc echo gz sim",
                ]
            )
        )

        candidates = cleanup.select_conflicting_processes(processes, {999})

        self.assertEqual([process.pid for process in candidates], [100, 101])

    def test_selects_project_simulation_stack_and_descendants(self) -> None:
        processes = cleanup.parse_ps_output(
            "\n".join(
                [
                    "100 1 100 bash ./scripts/run_city_mvp.sh",
                    (
                        "101 100 100 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 "
                        "launch drone_city_nav city_nav.launch.py"
                    ),
                    (
                        "102 101 100 /workspace/install/drone_city_nav/lib/"
                        "drone_city_nav/lidar_debug_node --ros-args"
                    ),
                    "103 100 100 MicroXRCEAgent udp4 -p 8888",
                    (
                        "104 100 100 make -C /workspace/external/PX4-Autopilot "
                        "px4_sitl gz_x500_lidar_2d"
                    ),
                    (
                        "105 104 100 /workspace/external/PX4-Autopilot/build/"
                        "px4_sitl_default/bin/px4"
                    ),
                    (
                        "106 1 106 rviz2 -d "
                        "/workspace/drone_city_nav/rviz/city_nav_debug.rviz"
                    ),
                    "107 1 107 python3 unrelated.py",
                ]
            )
        )

        candidates = cleanup.select_conflicting_processes(
            processes,
            {999},
            project_markers=["/workspace", "drone_city_nav"],
        )

        self.assertEqual(
            [process.pid for process in candidates],
            [100, 101, 102, 103, 104, 105, 106],
        )

    def test_protects_current_run_script_and_children(self) -> None:
        processes = cleanup.parse_ps_output(
            "\n".join(
                [
                    "10 1 10 /bin/bash parent",
                    "20 10 20 bash ./scripts/run_city_mvp.sh",
                    (
                        "30 20 20 /workspace/install/drone_city_nav/lib/"
                        "drone_city_nav/planner_node"
                    ),
                    "40 1 40 bash ./scripts/run_city_mvp.sh",
                    (
                        "50 40 40 /workspace/install/drone_city_nav/lib/"
                        "drone_city_nav/planner_node"
                    ),
                ]
            )
        )

        candidates = cleanup.select_conflicting_processes(
            processes,
            {20},
            project_markers=["/workspace", "drone_city_nav"],
        )

        self.assertEqual([process.pid for process in candidates], [40, 50])

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
        self.assertFalse(cleanup.is_conflicting_gazebo_process("gazebo-helper"))
        self.assertFalse(
            cleanup.is_conflicting_gazebo_process(
                "python3 note.py --message please run gz sim later"
            )
        )
        self.assertFalse(
            cleanup.is_conflicting_gazebo_process("rg gz sim scripts/run_city_mvp.sh")
        )
        self.assertFalse(
            cleanup.is_conflicting_gazebo_process("bash -lc echo gz sim")
        )
        self.assertFalse(
            cleanup.is_conflicting_gazebo_process(
                "python3 note.py --message 'please run gz sim later'"
            )
        )

    def test_matches_gazebo_executable_invocation_shapes(self) -> None:
        self.assertTrue(cleanup.is_conflicting_gazebo_process("gz sim -g"))
        self.assertTrue(cleanup.is_conflicting_gazebo_process("/opt/gz sim -g"))
        self.assertTrue(
            cleanup.is_conflicting_gazebo_process(
                "/usr/bin/env GZ_SIM_RESOURCE_PATH=/tmp /usr/bin/gz sim -s old.sdf"
            )
        )

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
