#!/usr/bin/env python3
"""Roadmap item 18 stage 1: smoke in the simulator stays an evaluation
component. The vehicle's drivers read every sensor through the smoke model's
relay when the location has smoke, and nothing of the vehicle reads the
zones; the overlay draws the zones as the model reads them."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "drone_city_nav" / "launch"))

from sensor_profile import SMOKE_TOPIC_SUFFIX, stereo_tof_topics  # noqa: E402

OVERLAY_PATH = REPOSITORY / "scripts" / "evaluation_overlay.py"
SPEC = importlib.util.spec_from_file_location("evaluation_overlay", OVERLAY_PATH)
assert SPEC is not None and SPEC.loader is not None
OVERLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OVERLAY)


class SimulationSmokeContractTest(unittest.TestCase):
    def test_no_vehicle_source_reads_the_smoke(self) -> None:
        sources = REPOSITORY / "drone_city_nav" / "src"
        for path in sources.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".cu", ".cuh"}:
                continue
            if path.name.startswith("simulation_smoke_"):
                continue
            text = path.read_text(errors="ignore")
            for word in ("simulation_smoke", "SmokeField", "smoke_zones", "SMOKE_ZONES"):
                self.assertNotIn(word, text, f"{path} reads the smoke model")

    def test_the_smoke_model_is_the_only_simulation_node_reading_the_evaluation(self) -> None:
        node = (REPOSITORY / "drone_city_nav" / "src" / "simulation_smoke_sensor_node.cpp").read_text()
        self.assertIn('"/evaluation/light_share"', node)
        self.assertNotIn("/drone_city_nav/", node)

    def test_with_smoke_every_sensor_comes_through_the_relay(self) -> None:
        bridge, remaps, depth = stereo_tof_topics(
            "world", "model", image_topic_suffix=SMOKE_TOPIC_SUFFIX,
            tof_topic_suffix=SMOKE_TOPIC_SUFFIX)
        for side in ("left", "right"):
            self.assertTrue(depth[f"{side}_gazebo_image_topic"].endswith(
                f"/stereo_{side}/image{SMOKE_TOPIC_SUFFIX}"))
        self.assertEqual(6, len(bridge))
        for argument in bridge:
            self.assertIn(f"/scan/points{SMOKE_TOPIC_SUFFIX}@", argument)
        clean, _, clean_depth = stereo_tof_topics("world", "model")
        for argument in clean:
            self.assertNotIn(SMOKE_TOPIC_SUFFIX, argument)
        self.assertNotIn(SMOKE_TOPIC_SUFFIX, clean_depth["left_gazebo_image_topic"])
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('f"{lidar_gz_topic}{sensor_suffix}@sensor_msgs/msg/PointCloud2"', launch)
        self.assertIn('executable="simulation_smoke_sensor_node"', launch)
        runtime = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text()
        self.assertIn('smoke_zones:="${SMOKE_ZONES}"', runtime)
        # The smoke face (K44) is off unless a flight asks for it (the owner,
        # 2026-10-24): SMOKE_FACE reaches the depth node's parameter.
        self.assertIn('smoke_face:="${SMOKE_FACE}"', runtime)
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('"smoke_face": smoke_face', launch)

    def test_the_heat_sources_ride_with_the_smoke_and_the_core_reads_them(self) -> None:
        # Roadmap item 32: HEAT_SOURCES reach the smoke model's relay, which
        # creates the bodies in the world; the thermal core is back on the
        # stereo set, its frame relayed as the pair's without the smoke's
        # suffix (the model leaves it untouched, K39); the depth node reads
        # the frame alone, never the sources; the hot scenarios are their own
        # targets and the ordinary flight sets no source.
        runtime = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text()
        self.assertIn('[[ -n "${HEAT_SOURCES:-}" ]] && ros_launch_args+=(heat_sources:="${HEAT_SOURCES}")', runtime)
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('"heat_sources": heat_sources.perform(context).strip()', launch)
        self.assertIn("  HEAT_SOURCES\n", (REPOSITORY / "scripts" / "container_run.sh").read_text())
        self.assertIn("HEAT_SOURCES SMOKE_ZONES SMOKE_SEED",
                      (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text())
        relay = (REPOSITORY / "drone_city_nav" / "src" / "simulation_smoke_sensor_node.cpp").read_text()
        self.assertIn('declare_parameter<std::string>("heat_sources", "")', relay)
        self.assertIn('"/create"', relay)
        self.assertIn("gz::sim::systems::Thermal", relay)
        self.assertNotIn("thermal/image", relay)
        model = (REPOSITORY / "drone_city_nav" / "models" / "stereo_tof_v1" / "model.sdf").read_text()
        self.assertIn('<sensor name="thermal" type="thermal_camera">', model)
        self.assertIn("<max_temp>650</max_temp>", model)
        _, _, depth = stereo_tof_topics(
            "world", "model", image_topic_suffix=SMOKE_TOPIC_SUFFIX,
            tof_topic_suffix=SMOKE_TOPIC_SUFFIX)
        self.assertEqual(depth["thermal_gazebo_image_topic"],
                         "/world/world/model/model/link/stereo_tof_link/sensor/thermal/image")
        self.assertEqual(depth["thermal_image_topic"], "/thermal/image")
        node = (REPOSITORY / "drone_city_nav" / "src" / "stereo_depth_node.cpp").read_text()
        self.assertIn("readThermalImage(", node)
        self.assertIn("heatStandoffCap(", node)
        self.assertIn("heatMarksText(", node)
        self.assertIn("HEAT_SOURCE_MARKS", (REPOSITORY / "drone_city_nav" / "src" / "stereo_depth_node_internal.hpp").read_text())
        self.assertNotIn("HEAT_SOURCES", node)
        makefile = (REPOSITORY / "Makefile").read_text()
        self.assertIn('HOT_PLUME_SCENARIO := SMOKE_ZONES="12,22,11,3,3,3,1.0" '
                      'HEAT_SOURCES="12,22,11,0.3,500"', makefile)
        self.assertIn('HEAT_SOURCES="-26.6,24.5,5,0.3,500"', makefile)
        self.assertNotIn('HEAT_SOURCES="$${HEAT_SOURCES-', makefile)
        self.assertEqual(makefile.count("HEAT_SOURCES="), 2)
        scenarios = (REPOSITORY / "docs" / "scenarios.md").read_text()
        self.assertIn("sim_urban_hot_smoke_plume_headless.sh", scenarios)
        self.assertIn("sim_urban_hot_smoke_room_headless.sh", scenarios)
        evidence = (REPOSITORY / "scripts" / "smoke_truth_evidence.py").read_text()
        self.assertIn("HEAT_STANDOFF_M = 4.0", evidence)
        # The lidar set (K49): the core on the lidar model, its frame over the bridge, the
        # thermal node and the memory's heat marks input on the lidar profile alone.
        lidar_model = (REPOSITORY / "drone_city_nav" / "models" / "lidar_3d_v1" / "model.sdf").read_text()
        self.assertIn('<sensor name="thermal" type="thermal_camera">', lidar_model)
        self.assertIn('executable="thermal_core_node"', launch)
        self.assertIn('sensor/thermal/image"', launch)
        self.assertIn('obstacle_memory_overrides["heat_marks_topic"] = "/drone_city_nav/heat_marks"', launch)
        memory = (REPOSITORY / "drone_city_nav" / "src" / "obstacle_memory_3d_node.cpp").read_text()
        self.assertIn('declare_parameter<std::string>("heat_marks_topic", "")', memory)
        self.assertIn("thermal_core_node", (REPOSITORY / "drone_city_nav" / "CMakeLists.txt").read_text())

    def test_the_smoke_face_detector_reads_the_frame_alone(self) -> None:
        # The smoke face detector (K44) is the vehicle's: it reads the pair's
        # frame, never the location's zones or the run's manifest, and the
        # depth node reports its faces.
        node = (REPOSITORY / "drone_city_nav" / "src" / "stereo_depth_node.cpp").read_text()
        patches = (REPOSITORY / "drone_city_nav" / "src" / "stereo_silent_patches.cpp").read_text()
        for text in (node, patches):
            self.assertNotIn("SMOKE_ZONES", text)
            self.assertNotIn("smoke_zones", text)
            self.assertNotIn("manifest", text)
        self.assertIn("smokeFaces(", node)
        self.assertIn('declare_parameter<bool>("smoke_face", false)', node)
        self.assertIn("rawCells(", node)
        self.assertIn("smoke_faces=%zu face_range_m=%.1f", node)

    def test_the_replayed_world_reads_the_zones_as_the_model_does(self) -> None:
        # The zones are drawn in a recording's world half alone: RViz shows no
        # obstacle of the truth (the owner, 2026-10-18), so the overlay reads
        # no zones and draws no box.
        sys.path.insert(0, str(REPOSITORY / "scripts"))
        import render_world_replay as replay
        self.assertFalse(hasattr(OVERLAY, "smoke_zones"))
        self.assertNotIn("smoke_alpha", OVERLAY_PATH.read_text())
        zones = replay.smoke_zones("1,2,3,4,5,6,0.5; 7,8,9,1,1,1,2")
        self.assertEqual(
            [((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.5), ((7.0, 8.0, 9.0), (1.0, 1.0, 1.0), 2.0)],
            zones)
        self.assertEqual([], replay.smoke_zones(""))
        with self.assertRaises(ValueError):
            replay.smoke_zones("1,2,3,4,5,6")
        thin = replay.smoke_alpha(0.05)
        dense = replay.smoke_alpha(4.0)
        self.assertLess(thin, dense)
        self.assertGreaterEqual(thin, 0.12)
        self.assertLessEqual(dense, 0.55)


if __name__ == "__main__":
    unittest.main()
