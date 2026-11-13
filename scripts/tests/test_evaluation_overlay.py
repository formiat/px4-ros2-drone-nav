#!/usr/bin/env python3
"""The evaluation overlay of RViz (specification K26): what it draws is read from
the stack's events and the evaluation's knowledge, and nothing of the stack
reads it back."""

from __future__ import annotations

import importlib.util
import math
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
        # LANDED while the vehicle stays where it touched down; one that flew
        # on (r1184 touched a floor in flight) shows what it does now.
        self.assertEqual("LANDED", text(["VEHICLE_LANDED"], True, True, False, True, True)[0])
        self.assertEqual("TO GOAL", text(["VEHICLE_LANDED"], False, False, True, True, False)[0])
        # By the truth: the estimate drifts on the ground without vision.
        self.assertIn("math.dist(truth, landed_at)", PATH.read_text())
        self.assertIn('state["landed_at"] = state["truth"]', PATH.read_text())
        self.assertEqual("GOAL REACHED",
                         text(["MISSION_RESULT success=true reason=goal"], False, False,
                              True, True)[0])
        self.assertEqual("CRASH", text(["VEHICLE_LANDED", "VEHICLE_DESTROYED"], False, False, False, True)[0])
        # No word of smoke: the vehicle has no rung, no hold and no return
        # of its own for a plume (the owner, 2026-10-18); a goal given up by
        # the proof reads as any other.
        ready = ["MISSION_READINESS ready=true"]
        self.assertEqual("GOAL UNREACHABLE -> HOME",
                         text(ready + ["GOAL_UNREACHABLE trigger=topological"], False,
                              False, True, True)[0])
        self.assertNotIn("SMOKE", PATH.read_text())

    def test_a_warm_body_and_a_heat_source_are_told_apart(self) -> None:
        # Roadmap items 21 and 32: the heat source's line, and the same line with a class for a body.
        text = OVERLAY.state_text
        ready = ["MISSION_READINESS ready=true"]
        self.assertEqual("WARM BODY", text(ready + ["HEAT_SOURCE class=body seen=true bodies=1 range_m=6.2"],
                                           False, False, True, True)[0])
        self.assertEqual("TO GOAL", text(ready + ["HEAT_SOURCE class=body seen=true bodies=1 range_m=6.2",
                                                  "HEAT_SOURCE class=body seen=false"],
                                         False, False, True, True)[0])
        self.assertEqual("HEAT SOURCE", text(ready + ["HEAT_SOURCE seen=true range_m=8.0 pixels=30",
                                                      "HEAT_SOURCE class=body seen=false"],
                                             False, False, True, True)[0])
        self.assertEqual("WARM BODY", text(ready + ["HEAT_SOURCE seen=true range_m=8.0 pixels=30",
                                                    "HEAT_SOURCE seen=false",
                                                    "HEAT_SOURCE class=body seen=true bodies=2 range_m=4.0"],
                                           False, False, True, True)[0])

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
        self.assertIn("words.action = backdrop.action = Marker.DELETE", source)

    def test_the_light_s_line_shows_what_was_set_beside_what_is_seen(self) -> None:
        self.assertEqual("", OVERLAY.light_text(None, None))
        self.assertEqual("LIGHT |||||||||| 100 %", OVERLAY.light_text(1.0, None))
        self.assertEqual("LIGHT ||||......  42 %   SEES |||.......  31 %",
                         OVERLAY.light_text(0.42, 0.31))
        self.assertEqual("SEES ..........   0 %", OVERLAY.light_text(None, 0.0))
        # The charge only in the scenarios that give the light a battery.
        self.assertEqual("", OVERLAY.charge_text(None))
        self.assertEqual("", OVERLAY.charge_text(float("inf")))
        self.assertEqual("CHARGE  183 s", OVERLAY.charge_text(183.2))

    def test_the_satellite_signal_s_line_follows_the_estimator_s_trust(self) -> None:
        # Roadmap item 39 (K56): nothing without a declaration; the words follow
        # the last declaration and tell a return from a first trust.
        self.assertEqual("", OVERLAY.gnss_text([]))
        self.assertEqual("", OVERLAY.gnss_text(["MISSION_READINESS ready=true"]))
        self.assertEqual("GNSS TRUSTED", OVERLAY.gnss_text(["GNSS_TRUST trusted=true"]))
        self.assertEqual("GNSS DISTRUSTED", OVERLAY.gnss_text(
            ["GNSS_TRUST trusted=true", "GNSS_TRUST trusted=false"]))
        self.assertEqual("GNSS BACK", OVERLAY.gnss_text(
            ["GNSS_TRUST trusted=true", "GNSS_TRUST trusted=false", "GNSS_TRUST trusted=true"]))
        self.assertIn('"GNSS_TRUST trusted="', (REPOSITORY / "drone_city_nav/src"
                                                / "visual_inertial_odometry_node_internal.hpp").read_text())

    def test_the_survey_mission_reads_as_a_survey(self) -> None:
        # Roadmap item 34: the survey gives its goals as the operator gives one and ends by the way home; the
        # words say what the vehicle is doing, not TO GOAL and OPERATOR -> HOME.
        text = OVERLAY.state_text
        ready = ["MISSION_READINESS ready=true"]
        goals = ready + ["OPERATOR_COMMAND goal 0.375 22.125 9.875", "SURVEY_GOAL_REACHED",
                         "OPERATOR_COMMAND goal 1.625 26.375 12.375"]
        self.assertEqual("SURVEY   GOAL 0", text(ready, False, False, True, True, False, True)[0])
        self.assertEqual("SURVEY   GOAL 2", text(goals, False, False, True, True, False, True)[0])
        home = goals + ["OPERATOR_COMMAND home", "GOAL_UNREACHABLE trigger=operator"]
        self.assertEqual("SURVEY DONE -> HOME", text(home, False, False, True, True, False, True)[0])
        self.assertEqual("OPERATOR -> HOME", text(home, False, False, True, True, False, False)[0])
        self.assertEqual("SURVEY COMPLETE",
                         text(home + ["MISSION_RESULT success=true reason="], False, False, True, True, False, True)[0])
        # The battery's way home is the battery's on a survey too.
        self.assertEqual("BATTERY -> HOME",
                         text(goals + ["GOAL_UNREACHABLE trigger=battery"], False, False, True, True, False, True)[0])
        runtime = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn('--sdf-to-map-offset="$(sdf_to_map_offset)" --mission "${mission_type}" \\\n', runtime)
        render = (REPOSITORY / "tools" / "render_flight.sh").read_text()
        self.assertIn('survey) target="sim-urban-survey-headless" ;;', render)
        # The surveys of the start building and of the held-out worlds are recorded as the urban one is.
        self.assertIn('finals-survey) target="sim-finals-survey-headless"', render)
        self.assertIn('cave-survey) target="sim-cave-survey-headless"', render)
        self.assertIn("urban_circuit_practice_01_survey_building_scenario.json SURVEY_VOLUME_M=-30,3.5,0,28,46,18",
                      render)

    def test_the_injected_satellite_fault_stands_beside_the_verdict(self) -> None:
        # The owner's request of 2026-11-05: the viewer sees the fault begin and the verdict follow. The variant's
        # name is the relay's knowledge, on /evaluation, which no production node reads.
        trusted = ["GNSS_TRUST trusted=true"]
        self.assertEqual("GNSS TRUSTED", OVERLAY.gnss_text(trusted, "none"))
        self.assertEqual("GNSS TRUSTED   FAULT WALK", OVERLAY.gnss_text(trusted, "walk"))
        self.assertEqual("GNSS DISTRUSTED   FAULT RETURN OFFSET",
                         OVERLAY.gnss_text(trusted + ["GNSS_TRUST trusted=false"], "return_offset"))
        self.assertEqual("GNSS FAULT JAM+BAD TIME", OVERLAY.gnss_text([], "jam+bad_time"))
        overlay = (REPOSITORY / "scripts" / "evaluation_overlay.py").read_text()
        self.assertIn('create_subscription(String, "/evaluation/gnss_fault", on_gnss_fault, 10)', overlay)
        relay = (REPOSITORY / "scripts" / "gnss_fault_relay.py").read_text()
        self.assertIn('fault_pub = node.create_publisher(String, "/evaluation/gnss_fault", 1)', relay)
        self.assertIn("fault_pub.publish(String(data=player.active_names(t_s)))", relay)

    def test_the_truth_is_drawn_in_the_map_frame(self) -> None:
        # The held-out worlds (roadmap item 33) have a map frame moved from the
        # SDF frame; the overlay draws in the map frame and moves the
        # simulator's truth into it by the offset the true pose record takes.
        self.assertEqual((1.0, 2.0, 3.0), OVERLAY.sdf_to_map((1.0, 2.0, 3.0), (0.0, 0.0, 0.0)))
        self.assertEqual((73.75, 296.25, 21.23),
                         tuple(round(v, 2) for v in OVERLAY.sdf_to_map(
                             (80.25, 63.25, -16.27), (-6.5, 233.0, 37.5))))
        source = PATH.read_text()
        self.assertIn('made.header.frame_id = "map"', source)
        self.assertNotIn('"gazebo_map"', source)
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text()
        self.assertIn('--sdf-to-map-offset="$(sdf_to_map_offset)"', runtime)
        self.assertEqual(2, runtime.count('--sdf-to-map-offset="$(sdf_to_map_offset)"'))
        offboard = (REPOSITORY / "drone_city_nav/src/mppi_offboard_node_state.cpp").read_text()
        self.assertIn('gazebo_aligned_rviz_axes_swapped_ ? rviz_drone_follow_parent_frame_ : "map"',
                      offboard)

    def test_the_backdrop_faces_the_camera(self) -> None:
        # Straight down: the backdrop's face looks up, its orientation a unit quaternion.
        x, y, z, w = OVERLAY.facing(0.0, 1.5707)
        self.assertAlmostEqual(1.0, x * x + y * y + z * z + w * w, places=6)
        # A unit quaternion rotating (0, 0, 1) to the camera's direction (up, for pitch pi/2).
        tz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
        self.assertAlmostEqual(1.0, tz[2], places=3)
        x, y, z, w = OVERLAY.facing(0.65, 0.95)
        tz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
        self.assertAlmostEqual(math.cos(0.65) * math.cos(0.95), tz[0], places=3)
        self.assertAlmostEqual(math.sin(0.95), tz[2], places=3)
        source = PATH.read_text()
        self.assertIn("Marker.CUBE", source)
        self.assertIn('"/drone_city_nav/mppi/navigation_progress"', source)
        # The frame's share, not the measured range over the guaranteed one,
        # which has a floor at the 2 m margin (31 % in the dark, r1175).
        self.assertIn('state["seen"] = message.sensor_frame_share', source)
        self.assertNotIn("sensor_measured_range_m", source)


class EvaluationNamespaceContractTest(unittest.TestCase):
    def test_nothing_of_the_stack_reads_the_evaluation_namespace(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for folder in ("src", "include", "launch", "config"):
            for path in (production / folder).rglob("*"):
                # The smoke model is an evaluation component itself (roadmap
                # item 18 stage 1): it reads the light's share as the overlay
                # does, and test_simulation_smoke.py holds the vehicle away
                # from it.
                if path.name.startswith("simulation_smoke_"):
                    continue
                if path.suffix in {".cpp", ".hpp", ".py", ".yaml"}:
                    self.assertNotIn("/evaluation", path.read_text(errors="ignore"), str(path))

    def test_the_events_the_overlay_reads_are_published_by_the_nodes_that_decide(self) -> None:
        sources = {name: (REPOSITORY / "drone_city_nav/src" / name).read_text() for name in (
            "mission_monitor_node.cpp", "collision_crash_node.cpp",
            "visual_inertial_odometry_node.cpp")}
        # The monitor's sources are three since 2026-11-02 (the source size contract).
        sources["mission_monitor_node.cpp"] = "".join(
            path.read_text() for path in sorted((REPOSITORY / "drone_city_nav/src").glob("mission_monitor_node*.cpp")))
        # The camera estimator's node is three since 2026-11-04 (the class, its constructor, its depth).
        sources["visual_inertial_odometry_node.cpp"] = "".join(
            path.read_text() for path in sorted((REPOSITORY / "drone_city_nav/src").glob("visual_inertial_odometry_node*")))
        for text in sources.values():
            self.assertIn('"/drone_city_nav/mission_events"', text)
        self.assertIn('"MISSION_READINESS ready=true"', sources["mission_monitor_node.cpp"])
        self.assertIn('"GOAL_UNREACHABLE trigger="', sources["mission_monitor_node.cpp"])
        self.assertIn('"MISSION_RESULT success="', sources["mission_monitor_node.cpp"])
        self.assertIn('"VEHICLE_LANDED"', sources["collision_crash_node.cpp"])
        self.assertIn('"VEHICLE_DESTROYED"', sources["collision_crash_node.cpp"])
        self.assertIn('"VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started="',
                      sources["visual_inertial_odometry_node.cpp"])
        offboard = "\n".join(path.read_text() for path in sorted(
            (REPOSITORY / "drone_city_nav/src").glob("mppi_offboard_node*.cpp")))
        self.assertIn('"/drone_city_nav/mission_events"', offboard)
        self.assertIn('"AUTOPILOT_CONTROL state=', offboard)
        self.assertIn('"MISSION_LANDING started=true"', offboard)
        self.assertNotIn('"SMOKE_', offboard + sources["mission_monitor_node.cpp"])
        self.assertIn('"MISSION_LANDED disarmed="', sources["mission_monitor_node.cpp"])
        self.assertIn('"OPERATOR_COMMAND "', sources["mission_monitor_node.cpp"])

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
