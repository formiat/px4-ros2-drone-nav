#!/usr/bin/env python3
"""Roadmap item 29: the large dark hall, a world the repository carries, flown
by the point-to-point mission as Urban is."""

from __future__ import annotations

import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = REPOSITORY / "drone_city_nav/config/dark_hall_point_to_point_scenario.json"
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/dark_hall.world3d.json"
HALL = REPOSITORY / "environments/fixtures/dark_hall"
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class DarkHallScenarioTest(unittest.TestCase):
    def test_the_mission_crosses_the_hall_at_the_lights_edge(self) -> None:
        scenario = SCENARIO_MODULE.load_point_to_point_scenario(SCENARIO_PATH)
        self.assertEqual(scenario["canonical_world_path"], WORLD_PATH.resolve())
        self.assertEqual(scenario["gazebo_world_name"], "dark_hall_collisions")
        self.assertFalse(scenario["gazebo_axes_swapped"])
        # The body the legs' 0.23 m above the pad's top at 0.5 m, 4 m from the
        # west wall and facing it: the estimator starts on the wall's texture.
        self.assertEqual(scenario["map_start_m"], (4.0, 30.0, 0.73))
        self.assertAlmostEqual(3.141592653589793, scenario["yaw_rad"])
        goal = scenario["mission_goal_sequence_m"][-1]
        self.assertAlmostEqual(64.0, goal[0] - scenario["map_start_m"][0])
        # Flown 6 m over the floor: the pair's lowest rays meet it 7.6 m away,
        # at the edge of what the carried light lights.
        self.assertEqual(6.0, goal[2])
        self.assertEqual(6.0, scenario["initial_altitude_m"])

    def test_the_world_has_no_light_and_a_textured_floor(self) -> None:
        world = ET.parse(HALL / "world.sdf").getroot().find("world")
        assert world is not None
        self.assertEqual([], world.findall("light"))
        maps = {element.text for element in world.iter("albedo_map")}
        self.assertEqual({"texture.png"}, maps)
        self.assertTrue((HALL / "texture.png").is_file())

    def test_the_named_scripts_run_the_hall_targets(self) -> None:
        makefile = (REPOSITORY / "Makefile").read_text()
        for kind in ("headless", "gui"):
            self.assertIn(f"sim-dark-hall-{kind}:", makefile)
            script = (REPOSITORY / f"scripts/sim_dark_hall_{kind}.sh").read_text()
            self.assertIn(f"make sim-dark-hall-{kind}", script)
        self.assertIn("--environment dark_hall_fixture", makefile)
        render = (REPOSITORY / "tools/render_flight.sh").read_text()
        self.assertIn('dark-hall) target="sim-dark-hall-headless"', render)


if __name__ == "__main__":
    unittest.main()
