#!/usr/bin/env python3
"""The evaluation overlay of RViz (specification K26): what it draws is read from
the stack's events and the evaluation's knowledge, and nothing of the stack
reads it back."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
PATH = REPOSITORY / "scripts" / "evaluation_overlay.py"
SPEC = importlib.util.spec_from_file_location("evaluation_overlay", PATH)
assert SPEC is not None and SPEC.loader is not None
OVERLAY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = OVERLAY
SPEC.loader.exec_module(OVERLAY)


class StateWordsTest(unittest.TestCase):
    def test_the_words_follow_the_vehicle_s_decisions(self) -> None:
        text = OVERLAY.state_text
        self.assertEqual("ON THE PAD", text([], False, False, False, False)[0])
        self.assertEqual("TAKEOFF", text([], False, False, True, False)[0])
        self.assertEqual("TO GOAL", text(["MISSION_READINESS ready=true"], False, False,
                                         True, True)[0])
        self.assertEqual("BATTERY -> HOME",
                         text(["MISSION_READINESS ready=true", "GOAL_UNREACHABLE trigger=battery"],
                              False, False, True, True)[0])
        self.assertEqual("LIGHT UNRELIABLE -> HOME",
                         text(["GOAL_UNREACHABLE trigger=unreliable_light"], False, False,
                              True, True)[0])
        self.assertEqual("GOAL UNREACHABLE -> HOME",
                         text(["GOAL_UNREACHABLE trigger=topological"], False, False,
                              True, True)[0])
        # The sight gone overrides the way home; the descent overrides the reckoning.
        self.assertEqual("DEAD RECKONING",
                         text(["GOAL_UNREACHABLE trigger=unreliable_light"], True, False,
                              True, True)[0])
        self.assertEqual("LEVEL DESCENT", text([], True, True, True, True)[0])
        # On the pad the estimator declares dead reckoning too: not the flight's.
        self.assertEqual("TAKEOFF", text([], True, False, True, False)[0])
        self.assertEqual("LANDED", text(["VEHICLE_LANDED"], True, True, False, True)[0])
        self.assertEqual("GOAL REACHED",
                         text(["MISSION_RESULT success=true reason=goal"], False, False,
                              True, True)[0])
        self.assertEqual("CRASH", text(["VEHICLE_LANDED", "VEHICLE_DESTROYED"], False, False, False, True)[0])

    def test_the_words_sit_in_the_top_left_corner_of_each_view(self) -> None:
        # Straight down with yaw 0, RViz's camera has +Y to its right and -X up.
        x, y, z = OVERLAY.corner_offset(0.0, 1.5707, 50.0, -0.55, 0.86)
        self.assertLess(y, -8.0)
        self.assertLess(x, -15.0)
        self.assertAlmostEqual(z, 0.0, places=1)
        # The follow view looks down at 54 degrees: the offset climbs with the view's up.
        _, _, z = OVERLAY.corner_offset(0.65, 0.95, 45.0, -0.55, 0.86)
        self.assertGreater(z, 5.0)

    def test_a_blank_line_is_never_drawn(self) -> None:
        # RViz dies on a text marker of one space (r1125, r1153, r1154): a
        # line with nothing to say is deleted instead.
        source = PATH.read_text()
        self.assertNotIn('light or " "', source)
        self.assertIn("words.action = Marker.DELETE", source)

    def test_the_light_s_line_is_a_bar_and_the_charge_only_with_a_battery(self) -> None:
        self.assertEqual("", OVERLAY.light_text(None, None))
        self.assertEqual("LIGHT |||||||||| 100 %", OVERLAY.light_text(1.0, None))
        self.assertEqual("LIGHT ||||......  42 %   CHARGE  183 s", OVERLAY.light_text(0.42, 183.2))
        self.assertEqual("LIGHT ..........   0 %", OVERLAY.light_text(0.0, float("inf")))


class EvaluationNamespaceContractTest(unittest.TestCase):
    def test_nothing_of_the_stack_reads_the_evaluation_namespace(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for folder in ("src", "include", "launch", "config"):
            for path in (production / folder).rglob("*"):
                if path.suffix in {".cpp", ".hpp", ".py", ".yaml"}:
                    self.assertNotIn("/evaluation", path.read_text(errors="ignore"), str(path))

    def test_the_events_the_overlay_reads_are_published_by_the_nodes_that_decide(self) -> None:
        sources = {name: (REPOSITORY / "drone_city_nav/src" / name).read_text() for name in (
            "mission_monitor_node.cpp", "collision_crash_node.cpp",
            "visual_inertial_odometry_node.cpp")}
        for text in sources.values():
            self.assertIn('"/drone_city_nav/mission_events"', text)
        self.assertIn('"MISSION_READINESS ready=true"', sources["mission_monitor_node.cpp"])
        self.assertIn('"GOAL_UNREACHABLE trigger="', sources["mission_monitor_node.cpp"])
        self.assertIn('"MISSION_RESULT success="', sources["mission_monitor_node.cpp"])
        self.assertIn('"VEHICLE_LANDED"', sources["collision_crash_node.cpp"])
        self.assertIn('"VEHICLE_DESTROYED"', sources["collision_crash_node.cpp"])
        self.assertIn('"VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started="',
                      sources["visual_inertial_odometry_node.cpp"])

    def test_both_rviz_views_show_the_overlay_and_rviz_starts_it(self) -> None:
        for name, view in (("city_nav_debug.rviz", "follow"),
                           ("city_nav_debug_top_down.rviz", "top")):
            config = (REPOSITORY / "drone_city_nav/rviz" / name).read_text()
            self.assertIn("Name: Evaluation Overlay", config)
            self.assertIn(f"Value: /evaluation/markers_{view}", config)
            # The view the words are placed for is the view of the configuration.
            yaw, pitch, distance = OVERLAY.VIEWS[view]
            self.assertIn(f"Yaw: {yaw:g}", config)
            self.assertIn(f"Pitch: {pitch:g}", config)
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text()
        self.assertIn("scripts/evaluation_overlay.py", runtime)
        self.assertIn('bool_is_true "${enable_rviz}"', runtime)


if __name__ == "__main__":
    unittest.main()
