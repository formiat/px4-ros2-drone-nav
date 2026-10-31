#!/usr/bin/env python3
"""Roadmap item 39: the hybrid localization profile, the cave's visual-inertial
estimator with the satellite fixes read through the simulation's relay, their
trust a measurement; the cave profiles untouched by it."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]


class HybridProfileTest(unittest.TestCase):
    def test_the_profile_is_wired_through_every_layer(self) -> None:
        run = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn("visual_inertial_shadow | visual_inertial | hybrid)", run)
        self.assertIn('[[ "${localization_profile}" == "hybrid" ]] && ros_launch_args+=(gnss_topic:=/drone_city_nav/simulation/gnss)', run)
        px4 = (REPOSITORY / "scripts/px4_parameter_runtime.sh").read_text(encoding="utf-8")
        self.assertIn('"${localization_profile}" == "visual_inertial" || "${localization_profile}" == "hybrid"', px4)
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn("from localization_profile import", launch)
        self.assertIn('"gnss_topic",', launch)
        profiles = (REPOSITORY / "drone_city_nav/launch/localization_profile.py").read_text(encoding="utf-8")
        self.assertIn('"hybrid",', profiles)
        self.assertIn('overrides["gnss_topic"] = gnss_topic.strip()', profiles)
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text(encoding="utf-8")
        self.assertIn('if [[ "${LOCALIZATION_PROFILE:-}" == hybrid ]]; then', runtime)
        self.assertIn("scripts/gnss_fault_relay.py", runtime)
        self.assertIn("GNSS_FAULTS GNSS_FAULT_SEED", runtime)
        whitelist = (REPOSITORY / "scripts/container_run.sh").read_text(encoding="utf-8")
        self.assertIn("\n  GNSS_FAULTS\n  GNSS_FAULT_SEED\n", whitelist)
        check = (REPOSITORY / "scripts/headless_runtime_evidence.py").read_text(encoding="utf-8")
        self.assertIn('if profile in ("visual_inertial", "hybrid"):', check)
        self.assertIn("from gnss_trust_evidence import report_gnss_trust", check)
        self.assertIn("def report_gnss_trust", (REPOSITORY / "scripts/gnss_trust_evidence.py").read_text(encoding="utf-8"))

    def test_the_cave_profiles_subscribe_no_signal(self) -> None:
        # The topic is empty in the configuration: the estimator subscribes
        # nothing and pays nothing; only the hybrid launch names one.
        configuration = yaml.safe_load((REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text(encoding="utf-8"))
        parameters = configuration["visual_inertial_odometry_node"]["ros__parameters"]
        self.assertEqual(parameters["gnss_topic"], "")
        self.assertEqual(parameters["gnss_window_s"], 5.0)
        for gate in ("gnss_position_gate", "gnss_velocity_gate", "gnss_self_gate_mps", "gnss_jump_gate_m",
                     "gnss_clock_step_gate_s"):
            self.assertGreaterEqual(parameters[gate], 1.0e300, gate)
        node = (REPOSITORY / "drone_city_nav/src/visual_inertial_odometry_node.cpp").read_text(encoding="utf-8")
        self.assertIn('declare_parameter<std::string>("gnss_topic", "")', node)
        self.assertIn("if (!gnss_topic.empty()) {", node)
        # No production node reads the relay's schedule.
        for path in (REPOSITORY / "drone_city_nav/src").rglob("*.cpp"):
            self.assertNotIn("GNSS_FAULTS", path.read_text(encoding="utf-8"), path.name)

    def test_the_trust_is_a_measurement_without_a_latch(self) -> None:
        trust = (REPOSITORY / "drone_city_nav/src/gnss_trust.cpp").read_text(encoding="utf-8")
        # A disagreeing fix resets the agreement; nothing else is remembered.
        self.assertIn("agreeing_ = false;\n    trusted_ = false;", trust)
        self.assertNotIn("lied", trust)
        header = (REPOSITORY / "drone_city_nav/include/drone_city_nav/gnss_trust.hpp").read_text(encoding="utf-8")
        self.assertEqual(len(re.findall(r"^\s+(bool|double|std::int64_t) \w+_\{", header, re.M)), 3)


if __name__ == "__main__":
    unittest.main()
