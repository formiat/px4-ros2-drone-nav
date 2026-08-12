#!/usr/bin/env python3
"""Cross-file contracts for the cooperative traffic mission."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SCENARIO = PACKAGE / "config" / "cooperative_traffic_scenario.json"
LOADER = PACKAGE / "launch" / "intercept_scenario.py"
WRAPPER = PACKAGE / "launch" / "cooperative_traffic.launch.py"
SHARED_LAUNCH = PACKAGE / "launch" / "multi_vehicle.launch.py"
MISSION_LAUNCH = PACKAGE / "launch" / "multi_vehicle_mission_launch.py"
RUNNER = REPOSITORY / "scripts" / "run_drone_nav_sim.sh"
RUNTIME = REPOSITORY / "scripts" / "multi_vehicle_sim_runtime.sh"
CONTAINER_RUNNER = REPOSITORY / "scripts" / "container_run.sh"
MAKEFILE = REPOSITORY / "Makefile"
GUI_WRAPPER = REPOSITORY / "scripts" / "sim_cooperative_traffic_gui.sh"
HEADLESS_WRAPPER = (
    REPOSITORY / "scripts" / "sim_cooperative_traffic_headless.sh"
)

SPEC = importlib.util.spec_from_file_location("multi_vehicle_scenario", LOADER)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class CooperativeTrafficLaunchContractTest(unittest.TestCase):
    def test_scenario_contains_four_equal_altitude_crossing_routes(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO)

        self.assertEqual(scenario["mission_name"], "cooperative_traffic")
        self.assertEqual(scenario["civilian_ids"], [f"civilian_{i}" for i in range(4)])
        self.assertEqual(len(scenario["vehicles"]), 4)
        self.assertTrue(
            all(vehicle["role"] == "civilian" for vehicle in scenario["vehicles"])
        )
        self.assertEqual(
            {vehicle["map_start_m"][2] for vehicle in scenario["vehicles"]},
            {0.3},
        )
        self.assertEqual(
            {goal["goal_m"][2] for goal in scenario["vehicle_goals"]},
            {18.0},
        )
        starts = {vehicle["map_start_m"][:2] for vehicle in scenario["vehicles"]}
        goals = {goal["goal_m"][:2] for goal in scenario["vehicle_goals"]}
        self.assertEqual(starts, goals)

    def test_launch_reuses_generic_navigation_stack_and_enables_cooperation(self) -> None:
        wrapper = WRAPPER.read_text(encoding="utf-8")
        shared = SHARED_LAUNCH.read_text(encoding="utf-8")
        mission = MISSION_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('"cooperative_traffic"', wrapper)
        self.assertIn('plugin="drone_city_nav::ProductionMppiNode"', shared)
        self.assertIn('"cooperative_traffic_enabled": cooperative_traffic', shared)
        self.assertIn('"vehicle_role": config["role_code"]', shared)
        self.assertIn('"civilian": 3', shared)
        self.assertIn(
            'plugin="drone_city_nav::CooperativeTrafficAgentNode"', mission
        )
        self.assertIn('executable="cooperative_traffic_referee_node"', mission)
        cooperative_body = mission.split(
            "def make_cooperative_mission_nodes(", maxsplit=1
        )[1]
        self.assertNotIn("RadarSimulatorNode", cooperative_body)
        self.assertNotIn("RadarTargetTrackerNode", cooperative_body)
        self.assertNotIn("InterceptorGuidanceNode", cooperative_body)

    def test_runner_and_container_expose_a_separate_finite_mission(self) -> None:
        runner = RUNNER.read_text(encoding="utf-8")
        runtime = RUNTIME.read_text(encoding="utf-8")
        container = CONTAINER_RUNNER.read_text(encoding="utf-8")
        makefile = MAKEFILE.read_text(encoding="utf-8")

        self.assertIn("cooperative_traffic)", runtime)
        self.assertIn("cooperative_traffic.launch.py", runner)
        self.assertIn("cooperative_traffic_scenario.json", runtime)
        self.assertIn("MULTI_VEHICLE_SCENARIO_PATH", container)
        self.assertIn("sim-cooperative-traffic-gui:", makefile)
        self.assertIn("sim-cooperative-traffic-headless:", makefile)
        self.assertIn(
            "make sim-cooperative-traffic-gui",
            GUI_WRAPPER.read_text(encoding="utf-8"),
        )
        self.assertIn(
            "make sim-cooperative-traffic-headless",
            HEADLESS_WRAPPER.read_text(encoding="utf-8"),
        )

    def test_cooperative_truth_never_enters_the_vehicle_agents(self) -> None:
        agent = (PACKAGE / "src" / "cooperative_traffic_agent_node.cpp").read_text(
            encoding="utf-8"
        )
        referee = (
            PACKAGE / "src" / "cooperative_traffic_referee_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertNotIn("SimulationTruthState", agent)
        self.assertIn("makeExclusiveGroundTruthBoundary", referee)
        self.assertNotIn("create_publisher<msg::VehicleDestroyed>", referee)


if __name__ == "__main__":
    unittest.main()
