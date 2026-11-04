#!/usr/bin/env python3
"""Roadmap item 34: the survey mission (specification F20) — its scenario, its targets, its node beside the
mission monitor, the operator's channel it commands on, the mission check's survey branch and the
coverage read from a flight's records."""

from __future__ import annotations

import importlib.util
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))
import survey_coverage  # noqa: E402
from validate_static_cooperative_scenario import OCCUPANCY_MAGIC  # noqa: E402

LOADER = importlib.util.spec_from_file_location(
    "point_to_point_scenario", REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py")
assert LOADER is not None and LOADER.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(LOADER)
LOADER.loader.exec_module(SCENARIO_MODULE)

CHUNK = 16


def words_of(block) -> list:
    """A 16^3 boolean block (x, y, z) as the memory's 64 words: bit x + 16 y + 256 z."""
    bits = np.packbits(block.transpose(2, 1, 0).reshape(-1).astype(np.uint8), bitorder="little")
    return np.frombuffer(bits.tobytes(), dtype="<u8").tolist()


def write_truth(path: Path, occupied, resolution: float, origin) -> None:
    """A truth grid in the project's Occupancy3D format (version 5)."""
    shape = occupied.shape
    chunks = []
    for cx in range((shape[0] + CHUNK - 1) // CHUNK):
        for cy in range((shape[1] + CHUNK - 1) // CHUNK):
            for cz in range((shape[2] + CHUNK - 1) // CHUNK):
                block = np.zeros((CHUNK, CHUNK, CHUNK), dtype=bool)
                sub = occupied[cx * CHUNK:(cx + 1) * CHUNK, cy * CHUNK:(cy + 1) * CHUNK, cz * CHUNK:(cz + 1) * CHUNK]
                block[:sub.shape[0], :sub.shape[1], :sub.shape[2]] = sub
                if block.any():
                    chunks.append(((cx, cy, cz), words_of(block)))
    data = bytearray(OCCUPANCY_MAGIC)
    data += struct.pack("<II", 5, CHUNK)
    data += struct.pack("<ffff", resolution, *origin)
    data += struct.pack("<III", *shape)
    data += struct.pack("<Q", 0)
    data += struct.pack("<I", len(chunks))
    for index, words in chunks:
        data += struct.pack("<iii", *index)
        data += struct.pack("<64Q", *words)
    path.write_bytes(bytes(data))


class SurveyScenarioTest(unittest.TestCase):
    def test_the_scenario_orders_the_volume_the_targets_snapshot(self) -> None:
        scenario = SCENARIO_MODULE.load_point_to_point_scenario(
            REPOSITORY / "drone_city_nav/config/urban_circuit_practice_01_survey_scenario.json")
        self.assertEqual(scenario["survey_volume_m"], (-32.0, 0.0, 0.0, 20.0, 46.0, 16.0))
        # The first goal is the takeoff point: the planner needs a goal before the survey's first.
        self.assertEqual(scenario["mission_goal_sequence_m"],
                         ((0.749319792, 21.246976852, 9.74),))
        self.assertEqual(scenario["map_start_m"], (0.749319792, 21.246976852, 7.74))
        goal = SCENARIO_MODULE.load_point_to_point_scenario(
            REPOSITORY / "drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json")
        self.assertIsNone(goal["survey_volume_m"])
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        self.assertIn("SURVEY_VOLUME_M := -32,0,0,20,46,16\n", makefile)
        recipe = makefile[makefile.index("sim-urban-survey-headless: build"):]
        recipe = recipe[:recipe.index("\n\n")]
        for text in ("MISSION_TYPE=survey", "TRUTH_OCCUPANCY_3D_PATH=$(URBAN_TRUTH_OCCUPANCY_3D)",
                     'RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:-$(SURVEY_VOLUME_M)}"',
                     "SURVEY_COVERAGE_MINIMUM=", "HEADLESS=1 MISSION_CHECK=1", "$(SURVEY_SCENARIO_PATH)"):
            self.assertIn(text, recipe)
        self.assertNotIn("REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING", recipe)
        for name in ("survey_headless", "survey_gui", "survey_lidar_headless", "survey_lidar_gui"):
            target = "sim-urban-" + name.replace("_", "-")
            self.assertIn(f"make {target}\n",
                          (REPOSITORY / f"scripts/sim_urban_{name}.sh").read_text(encoding="utf-8"))
            self.assertIn(f"{target}:", makefile)
        self.assertIn("$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-survey-headless\n", makefile)

    def test_the_survey_node_commands_on_the_operators_channel_beside_the_monitor(self) -> None:
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn('executable="survey_mission_node"', launch)
        self.assertIn('mission_monitor_parameters.append({"mission_type": "survey"})', launch)
        self.assertIn('survey_parameters.append({"survey_volume_m": list(survey_volume)})', launch)
        self.assertIn('"enable_survey_mission"',
                      (REPOSITORY / "drone_city_nav/launch/launch_arguments.py").read_text(encoding="utf-8"))
        runner = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn('enable_survey_mission:="$([[ "${mission_type}" == survey ]] && echo true || echo false)"',
                      runner)
        self.assertIn("point_to_point | survey) ;;",
                      (REPOSITORY / "scripts/multi_vehicle_sim_runtime.sh").read_text(encoding="utf-8"))
        node = (REPOSITORY / "drone_city_nav/src/survey_mission_node.cpp").read_text(encoding="utf-8")
        # One channel every goal enters by (K31); the monitor stays the objective's publisher (item 12).
        self.assertIn('"operator_command_topic"', node)
        self.assertIn('"goal %.3f %.3f %.3f"', node)
        self.assertIn('send("home")', node)
        self.assertNotIn("NavigationObjective", node)
        monitor = (REPOSITORY / "drone_city_nav/src/mission_monitor_node.cpp").read_text(encoding="utf-8")
        self.assertIn('survey_ = mission_type == "survey";', monitor)
        self.assertIn('"SURVEY_GOAL_REACHED', monitor)
        # The survey's first goal, the takeoff's point, captured before the planner declares the mission
        # ready, is the navigation ready (r2813: the survey's commands unanswered for 320 s).
        self.assertIn('MISSION_READINESS ready=true reason=survey_first_goal', monitor)
        self.assertIn('survey_ ? "survey_home" : "none"', monitor)
        self.assertIn("const bool moved = moved_ || survey_;", monitor)
        # The survey starts with the mission (r2826: done on the pad before the takeoff's goal).
        self.assertIn('event.starts_with("MISSION_READINESS ready=true")', node)
        self.assertIn("if (!ready_) {", node)
        core = "".join((REPOSITORY / f"drone_city_nav/src/{name}").read_text(encoding="utf-8")
                       for name in ("survey_mission.cpp", "survey_reading.cpp"))
        # A frontier disappears by measurement alone (F20, I3): no blacklist of tried frontiers.
        for word in ("blacklist", "tried", "attempts"):
            self.assertNotIn(word, core)
        self.assertIn("within its radius, K16", core)

    def test_the_mission_check_reads_the_survey(self) -> None:
        validator = (REPOSITORY / "scripts/validate_drone_nav_headless.py").read_text(encoding="utf-8")
        self.assertIn('choices=("point_to_point", "survey", "cooperative_traffic")', validator)
        self.assertIn("def validate_survey(", validator)
        # The truth flooded as the survey node floods, in the scenario's band and frame (r2810: flooded inside the
        # volume alone, the coverage read 100 % by construction).
        self.assertIn("band=band, truth_offset=offset", validator)
        self.assertEqual(survey_coverage.scenario_frame(
            REPOSITORY / "drone_city_nav/config/urban_circuit_practice_01_survey_scenario.json"),
            ((1.0, 20.0), (0.0, 0.0, 0.0)))
        self.assertEqual(survey_coverage.FLOOD_MARGIN_M, 40.0)
        self.assertIn(r"MISSION_RESULT success=true reason='survey_home'", validator)
        self.assertIn('or args.mission_type == "survey" else errors', validator)
        # A survey inside a building sees no ray without a return (r2820).
        self.assertIn('misses_required=args.mission_type != "survey"', validator)
        self.assertIn("SURVEY_COVERAGE_MINIMUM\n", (REPOSITORY / "scripts/container_run.sh").read_text(encoding="utf-8"))
        self.assertIn("RAW_SNAPSHOT_BOUNDS_M SURVEY_COVERAGE_MINIMUM",
                      (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text(encoding="utf-8"))

    def test_the_coverage_of_a_room_read_from_the_records(self) -> None:
        # A 16 x 12 x 6 m box at 0.25 m: a room of 8 x 8 x 4 m observed whole with a door in the east wall;
        # the truth at 0.5 m holds the same walls; the path crosses the room.
        shape = (64, 48, 24)
        observed = np.zeros(shape, dtype=bool)
        occupied = np.zeros(shape, dtype=bool)
        observed[8:40, 8:40, 4:20] = True
        occupied[8:40, 8:40, 4:20] = True
        occupied[9:39, 9:39, 5:19] = False
        occupied[39, 20:26, 5:13] = False
        truth = np.zeros((32, 24, 12), dtype=bool)
        truth[4:20, 4:20, 2:10] = True
        truth[5:19, 5:19, 3:9] = False
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "r1"
            run.mkdir()
            chunks = []
            for cx in range(4):
                for cy in range(3):
                    for cz in range(2):
                        sl = (slice(cx * CHUNK, (cx + 1) * CHUNK), slice(cy * CHUNK, (cy + 1) * CHUNK),
                              slice(cz * CHUNK, (cz + 1) * CHUNK))
                        if observed[sl].any():
                            full_observed = np.zeros((CHUNK, CHUNK, CHUNK), dtype=bool)
                            full_occupied = np.zeros((CHUNK, CHUNK, CHUNK), dtype=bool)
                            sub = observed[sl]
                            full_observed[:sub.shape[0], :sub.shape[1], :sub.shape[2]] = sub
                            sub = occupied[sl]
                            full_occupied[:sub.shape[0], :sub.shape[1], :sub.shape[2]] = sub
                            chunks.append({"index": [cx, cy, cz],
                                           "observed_words_hex": [f"{w:016x}" for w in words_of(full_observed)],
                                           "occupied_words_hex": [f"{w:016x}" for w in words_of(full_occupied)]})
            (run / "raw_snapshot_3d_revision_3.json").write_text(json.dumps({
                "revision": 3, "grid": {"resolution_m": 0.25, "origin_m": [0.0, 0.0, 0.0],
                                        "chunk_size_cells": CHUNK, "shape_cells": list(shape)},
                "selection": {"bounds_m": [0.0, 0.0, 0.0, 16.0, 12.0, 6.0]}, "chunks": chunks}))
            (run / "gz_pose.csv").write_text("t,x,y,z\n" + "".join(
                f"{i},{3.0 + 0.5 * i},{6.0},{2.5}\n" for i in range(12)))
            truth_path = Path(directory) / "truth.occupancy3d"
            write_truth(truth_path, truth, 0.5, (0.0, 0.0, 0.0))
            figures = survey_coverage.survey_figures(run, truth_path, None, 6.4)
        self.assertEqual(figures["frontier_cells"], 24)
        self.assertEqual(figures["clusters"], 1)
        self.assertEqual(figures["clusters_with_goal"], 1)
        self.assertGreater(figures["truth_target_cells"], 1000)
        self.assertGreaterEqual(figures["coverage"], 0.99)
        self.assertGreaterEqual(figures["volume_coverage"], 0.99)
        self.assertGreaterEqual(figures["occupied_agreeing_with_truth"], 0.99)
        self.assertAlmostEqual(figures["path_m"], 5.5)


if __name__ == "__main__":
    unittest.main()
