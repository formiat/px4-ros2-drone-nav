#!/usr/bin/env python3
"""Static contracts for the finite intercept mission CPU budget."""

from __future__ import annotations

import ast
import re
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
LAUNCH = PACKAGE / "launch" / "multi_vehicle.launch.py"
DIAGNOSTICS_LAUNCH = PACKAGE / "launch" / "intercept_diagnostics_launch.py"
TRACKING_LAUNCH = PACKAGE / "launch" / "intercept_tracking_launch.py"
RUN_SCRIPT = REPOSITORY / "scripts" / "run_drone_nav_sim.sh"


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


def _load_cpu_affinity_prefix():
    module = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    function = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_cpu_affinity_prefix"
    )
    namespace: dict[str, object] = {"re": re}
    exec(compile(ast.Module(body=[function], type_ignores=[]), LAUNCH, "exec"), namespace)
    return namespace["_cpu_affinity_prefix"]


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
        self.assertIn('"thread_num": len(role_names) + 2', source)
        self.assertNotIn('executable="production_mppi_node"', source)

    def test_tracking_pairs_share_an_intra_process_component_container(self) -> None:
        source = TRACKING_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('name="interceptor_tracking_container"', source)
        self.assertIn('plugin="drone_city_nav::RadarTargetTrackerNode"', source)
        self.assertIn('plugin="drone_city_nav::InterceptorGuidanceNode"', source)
        self.assertIn(
            'plugin="drone_city_nav::TargetAssignmentCoordinatorNode"', source
        )
        self.assertIn('{"use_intra_process_comms": True}', source)
        self.assertNotIn('executable="radar_target_tracker_node"', source)
        self.assertNotIn('executable="interceptor_guidance_node"', source)

    def test_diagnostics_share_an_intra_process_component_container(self) -> None:
        source = LAUNCH.read_text(encoding="utf-8")
        diagnostics_source = DIAGNOSTICS_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('name="intercept_diagnostics_container"', diagnostics_source)
        for plugin in (
            "WorldVisualizationNode",
            "InterceptSpectatorNode",
            "InterceptDiagnosticsMuxNode",
            "LidarDebugNode",
        ):
            with self.subTest(plugin=plugin):
                self.assertIn(
                    f'plugin="drone_city_nav::{plugin}"', diagnostics_source
                )
        for executable in (
            "world_visualization_node",
            "intercept_spectator_node",
            "intercept_diagnostics_mux_node",
            "lidar_debug_node",
        ):
            with self.subTest(executable=executable):
                self.assertNotIn(f'executable="{executable}"', source)
        self.assertIn("if lidar_debug_enabled:", source)

    def test_subsystem_affinity_is_validated_and_wired_by_role(self) -> None:
        prefix = _load_cpu_affinity_prefix()
        launch_source = LAUNCH.read_text(encoding="utf-8")
        run_source = RUN_SCRIPT.read_text(encoding="utf-8")

        self.assertEqual("", prefix(""))
        self.assertEqual("taskset --cpu-list 4-15", prefix("4-15"))
        with self.assertRaises(RuntimeError):
            prefix("4-15;false")
        self.assertIn('ENABLE_SUBSYSTEM_CPU_AFFINITY:-true', run_source)
        self.assertIn('control_cpu_list:="${control_cpu_list}"', run_source)
        self.assertIn('planning_cpu_list:="${planning_cpu_list}"', run_source)
        self.assertIn('diagnostics_cpu_list:="${diagnostics_cpu_list}"', run_source)
        self.assertIn("prefix=control_prefix", launch_source)
        self.assertIn("prefix=planning_prefix", launch_source)
        self.assertIn("prefix=diagnostics_prefix", launch_source)

    def test_planner_validates_and_applies_tick_phase(self) -> None:
        source = (PACKAGE / "src" / "production_mppi_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn('"planning_tick_phase_offset_s"', source)
        self.assertIn("planning tick phase offset must be in", source)
        self.assertIn("planning_start_timer_->cancel()", source)
        self.assertIn("startPlanningTimer();", source)

    def test_planning_timer_cannot_starve_input_callbacks(self) -> None:
        header = (PACKAGE / "src" / "production_mppi_node.hpp").read_text(
            encoding="utf-8"
        )
        source = (PACKAGE / "src" / "production_mppi_node.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("input_callback_group_", header)
        self.assertIn("planning_callback_group_", header)
        self.assertIn(
            "input_subscription_options.callback_group = input_callback_group_",
            source,
        )
        self.assertIn("planning_callback_group_);", source)

    def test_adaptive_rollout_budget_preserves_uncertain_and_risky_work(self) -> None:
        config = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")
        planning_tick = (
            PACKAGE / "src" / "production_mppi_node_planning_tick.cpp"
        ).read_text(encoding="utf-8")
        policy = (PACKAGE / "src" / "mppi_rollout_budget.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("rollouts: 8192", config)
        self.assertIn("open_static_rollouts: 6144", config)
        self.assertIn("direct_tracking_rollouts: 4096", config)
        self.assertIn("selectMppiRolloutBudget", planning_tick)
        self.assertIn(".active_rollouts = rollout_budget.active_rollouts", planning_tick)
        self.assertIn("kFullWorldUncertain", policy)
        self.assertIn("kFullElevatedRisk", policy)
        self.assertIn("kFullLowClearance", policy)
        self.assertIn("kFullNoStaticExploration", policy)


if __name__ == "__main__":
    unittest.main()
