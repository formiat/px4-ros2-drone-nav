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

    def test_the_overlay_reads_the_zones_as_the_model_does(self) -> None:
        zones = OVERLAY.smoke_zones("1,2,3,4,5,6,0.5; 7,8,9,1,1,1,2")
        self.assertEqual(
            [((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.5), ((7.0, 8.0, 9.0), (1.0, 1.0, 1.0), 2.0)],
            zones)
        self.assertEqual([], OVERLAY.smoke_zones(""))
        with self.assertRaises(ValueError):
            OVERLAY.smoke_zones("1,2,3,4,5,6")
        thin = OVERLAY.smoke_alpha(0.05)
        dense = OVERLAY.smoke_alpha(4.0)
        self.assertLess(thin, dense)
        self.assertGreaterEqual(thin, 0.12)
        self.assertLessEqual(dense, 0.55)


if __name__ == "__main__":
    unittest.main()
