#!/usr/bin/env python3
"""Static contracts for the finite intercept mission CPU budget."""

from __future__ import annotations

import ast
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
LAUNCH = PACKAGE / "launch" / "intercept.launch.py"


def _load_budget_allocator():
    module = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    allocator = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_allocate_planner_workers"
    )
    namespace: dict[str, object] = {}
    exec(compile(ast.Module(body=[allocator], type_ignores=[]), LAUNCH, "exec"), namespace)
    return namespace["_allocate_planner_workers"]


class InterceptResourceBudgetContractTest(unittest.TestCase):
    def test_default_budget_is_fairly_partitioned(self) -> None:
        allocate = _load_budget_allocator()

        self.assertEqual([2, 2, 2, 2], allocate(8, 4))
        self.assertEqual([3, 2, 2, 2], allocate(9, 4))

    def test_budget_rejects_invalid_per_vehicle_counts(self) -> None:
        allocate = _load_budget_allocator()

        with self.assertRaises(RuntimeError):
            allocate(3, 4)
        with self.assertRaises(RuntimeError):
            allocate(33, 4)

    def test_launch_wires_worker_counts_and_tick_phases(self) -> None:
        source = LAUNCH.read_text(encoding="utf-8")

        self.assertIn(
            'DeclareLaunchArgument("planner_worker_budget", default_value="8")',
            source,
        )
        self.assertIn('"planner_worker_count": planner_worker_counts[role_index]', source)
        self.assertIn('"planning_tick_phase_offset_s": (', source)

    def test_planners_share_one_multithreaded_component_process(self) -> None:
        source = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("ComposableNodeContainer(", source)
        self.assertIn('executable="component_container_mt"', source)
        self.assertIn('namespace=""', source)
        self.assertIn('plugin="drone_city_nav::ProductionMppiNode"', source)
        self.assertIn('"thread_num": len(role_names)', source)
        self.assertNotIn('executable="production_mppi_node"', source)

    def test_planner_validates_and_applies_tick_phase(self) -> None:
        source = (PACKAGE / "src" / "production_mppi_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn('"planning_tick_phase_offset_s"', source)
        self.assertIn("planning tick phase offset must be in", source)
        self.assertIn("planning_start_timer_->cancel()", source)
        self.assertIn("startPlanningTimer();", source)


if __name__ == "__main__":
    unittest.main()
