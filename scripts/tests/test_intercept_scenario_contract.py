#!/usr/bin/env python3
"""Cross-file contracts for canonical intercept starts and Gazebo spawns."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = REPOSITORY / "drone_city_nav" / "config" / "intercept_scenario.json"
MULTI_SCENARIO_PATH = (
    REPOSITORY
    / "drone_city_nav"
    / "config"
    / "multi_intercept_2v2_scenario.json"
)
LOADER_PATH = REPOSITORY / "drone_city_nav" / "launch" / "intercept_scenario.py"
LAUNCH_PATH = REPOSITORY / "drone_city_nav" / "launch" / "intercept.launch.py"
RUNNER_PATH = REPOSITORY / "scripts" / "run_drone_nav_sim.sh"
RUNTIME_HELPER_PATH = REPOSITORY / "scripts" / "intercept_sim_runtime.sh"
MULTI_HEADLESS_PATH = REPOSITORY / "scripts" / "sim_multi_intercept_headless.sh"
MULTI_GUI_PATH = REPOSITORY / "scripts" / "sim_multi_intercept_gui.sh"
MAKEFILE_PATH = REPOSITORY / "Makefile"

SPEC = importlib.util.spec_from_file_location("intercept_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class InterceptScenarioContractTest(unittest.TestCase):
    def test_all_map_starts_derive_expected_gazebo_spawns(self) -> None:
        scenario = SCENARIO_MODULE.load_intercept_scenario(SCENARIO_PATH)
        expected = {
            "interceptor_0": ((54.0, 54.0, 0.3), (-171.0, -81.0, 0.3)),
            "interceptor_1": ((54.0, 378.0, 0.3), (153.0, -81.0, 0.3)),
            "interceptor_2": ((270.0, 378.0, 0.3), (153.0, 135.0, 0.3)),
            "evader": ((216.0, 54.0, 0.3), (-171.0, 81.0, 0.3)),
        }
        self.assertEqual(tuple(expected), tuple(v["id"] for v in scenario["vehicles"]))
        for vehicle in scenario["vehicles"]:
            with self.subTest(vehicle=vehicle["id"]):
                map_start, gazebo_spawn = expected[vehicle["id"]]
                self.assertEqual(vehicle["map_start_m"], map_start)
                self.assertEqual(vehicle["gazebo_spawn_m"], gazebo_spawn)
        self.assertEqual(scenario["evader_goal_m"], (54.0, 378.0, 18.0))

    def test_vehicle_order_is_part_of_the_px4_instance_contract(self) -> None:
        document = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))
        document["canonical_world"] = str(
            (SCENARIO_PATH.parent / document["canonical_world"]).resolve()
        )
        document["vehicles"][0], document["vehicles"][1] = (
            document["vehicles"][1],
            document["vehicles"][0],
        )
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "vehicle order"):
                SCENARIO_MODULE.load_intercept_scenario(malformed)

    def test_generic_2v2_scenario_has_stable_targets_and_spawns(self) -> None:
        scenario = SCENARIO_MODULE.load_intercept_scenario(MULTI_SCENARIO_PATH)
        self.assertEqual(scenario["mission_name"], "multi_intercept")
        self.assertEqual(
            [vehicle["id"] for vehicle in scenario["vehicles"]],
            ["interceptor_0", "interceptor_1", "evader_0", "evader_1"],
        )
        self.assertEqual(
            scenario["interceptor_ids"], ["interceptor_0", "interceptor_1"]
        )
        self.assertEqual(
            [(target["id"], target["detection_id"]) for target in scenario["evaders"]],
            [("evader_0", 1), ("evader_1", 2)],
        )
        self.assertEqual(
            [target["goal_m"] for target in scenario["evaders"]],
            [(162.0, 216.0, 18.0), (162.0, 216.0, 18.0)],
        )
        self.assertEqual(
            [vehicle["gazebo_spawn_m"] for vehicle in scenario["vehicles"]],
            [
                (-171.0, -81.0, 0.3),
                (153.0, 135.0, 0.3),
                (-171.0, 135.0, 0.3),
                (153.0, -81.0, 0.3),
            ],
        )
        makefile = MAKEFILE_PATH.read_text(encoding="utf-8")
        for wrapper, target in (
            (MULTI_HEADLESS_PATH, "sim-multi-intercept-headless"),
            (MULTI_GUI_PATH, "sim-multi-intercept-gui"),
        ):
            with self.subTest(wrapper=wrapper.name):
                text = wrapper.read_text(encoding="utf-8")
                self.assertIn(f"make {target}", text)
                recipe = makefile.split(f"{target}:", maxsplit=1)[1].split(
                    "\n\n", maxsplit=1
                )[0]
                self.assertIn("MISSION_TYPE=multi_intercept", recipe)
                self.assertIn("INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED=false", recipe)
        runner = RUNNER_PATH.read_text(encoding="utf-8") + RUNTIME_HELPER_PATH.read_text(
            encoding="utf-8"
        )
        self.assertIn("multi_intercept_2v2_scenario.json", runner)

    def test_v2_rejects_duplicate_detection_ids(self) -> None:
        document = json.loads(MULTI_SCENARIO_PATH.read_text(encoding="utf-8"))
        document["canonical_world"] = str(
            (MULTI_SCENARIO_PATH.parent / document["canonical_world"]).resolve()
        )
        document["vehicles"][-1]["detection_id"] = 1
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "positive and unique"):
                SCENARIO_MODULE.load_intercept_scenario(malformed)

    def test_launch_and_runner_have_no_independent_intercept_coordinates(self) -> None:
        launch = LAUNCH_PATH.read_text(encoding="utf-8")
        runner = RUNNER_PATH.read_text(encoding="utf-8") + RUNTIME_HELPER_PATH.read_text(
            encoding="utf-8"
        )
        self.assertIn("load_intercept_scenario", launch)
        self.assertIn('intercept_scenario_path:="${intercept_scenario_path}"', runner)
        self.assertIn("intercept_scenario.py", runner)
        self.assertIn("intercept_gazebo_spawn_poses", runner)
        self.assertIn(
            'PX4_GZ_MODEL_POSE="${intercept_gazebo_spawn_poses[instance]}"',
            runner,
        )
        self.assertNotIn("intercept_px4_spawn_poses", runner)
        for legacy in (
            "interceptor_0_origin_x_m",
            "interceptor_1_origin_x_m",
            "interceptor_2_origin_x_m",
            "evader_origin_x_m",
            "evader_goal_x_m\", default_value",
            "INTERCEPTOR_1_SIM_START",
            "INTERCEPTOR_2_SIM_START",
            "EVADER_SIM_START",
        ):
            with self.subTest(legacy=legacy):
                self.assertNotIn(legacy, launch + runner)


if __name__ == "__main__":
    unittest.main()
