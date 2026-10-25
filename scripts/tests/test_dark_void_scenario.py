#!/usr/bin/env python3
"""Roadmap item 31: the dark void, the hall of item 29 crossed at 12 m, by the
point-to-point mission; the goal by the east wall or in the hall's middle."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
WALL_SCENARIO = REPOSITORY / "drone_city_nav/config/dark_void_point_to_point_scenario.json"
MID_SCENARIO = REPOSITORY / "drone_city_nav/config/dark_void_mid_point_to_point_scenario.json"
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/dark_hall.world3d.json"
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class DarkVoidScenarioTest(unittest.TestCase):
    def test_the_hall_is_crossed_at_twelve_metres(self) -> None:
        for path, goal_x in ((WALL_SCENARIO, 75.5), (MID_SCENARIO, 40.0)):
            scenario = SCENARIO_MODULE.load_point_to_point_scenario(path)
            self.assertEqual(scenario["canonical_world_path"], WORLD_PATH.resolve())
            self.assertEqual(scenario["map_start_m"], (4.0, 30.0, 0.73))
            goal = scenario["mission_goal_sequence_m"][-1]
            self.assertEqual((goal_x, 30.0, 12.0), tuple(goal))
            # Twelve metres: the tracker holds 20 features over the floor at 11 m
            # and none at 13 (K46); the ceiling 8 m above is out of reach.
            self.assertEqual(12.0, scenario["initial_altitude_m"])
        # The wall's goal is within the light's reach of the east wall at x = 80.
        self.assertLess(80.0 - 75.5, 5.2)

    def test_the_named_scripts_run_the_void_targets(self) -> None:
        makefile = (REPOSITORY / "Makefile").read_text()
        for kind in ("headless", "gui"):
            self.assertIn(f"sim-dark-void-{kind}: sim-dark-hall-{kind}", makefile)
            script = (REPOSITORY / f"scripts/sim_dark_void_{kind}.sh").read_text()
            self.assertIn(f"make sim-dark-void-{kind}", script)
        self.assertIn("DARK_VOID_SCENARIO_PATH ?= drone_city_nav/config/dark_void_point_to_point_scenario.json",
                      makefile)
        render = (REPOSITORY / "tools/render_flight.sh").read_text()
        self.assertIn('dark-void) target="sim-dark-void-headless"', render)
        self.assertIn("**The dark void**", (REPOSITORY / "docs/scenarios.md").read_text())


if __name__ == "__main__":
    unittest.main()
