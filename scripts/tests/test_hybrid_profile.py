#!/usr/bin/env python3
"""Roadmap item 39: the hybrid localization profile, the cave's visual-inertial
estimator with the satellite fixes read through the simulation's relay, their
trust a measurement; the cave profiles untouched by it."""

from __future__ import annotations

import io
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
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
        check = (REPOSITORY / "scripts/localization_evidence.py").read_text(encoding="utf-8")
        self.assertIn('if profile in ("visual_inertial", "hybrid"):', check)
        self.assertIn("from gnss_trust_evidence import report_gnss_faults, report_gnss_trust", check)
        self.assertIn("def report_gnss_trust", (REPOSITORY / "scripts/gnss_trust_evidence.py").read_text(encoding="utf-8"))

    def test_the_map_registration_is_measured_on_request(self) -> None:
        # Roadmap item 40: MAP_REGISTRATION=measure reaches the estimator's
        # parameter through the launch, on every profile that runs it.
        import importlib.util  # noqa: PLC0415
        spec = importlib.util.spec_from_file_location(
            "localization_profile", REPOSITORY / "drone_city_nav/launch/localization_profile.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.visual_inertial_overrides("visual_inertial", "", "measure"),
                         {"publish_to_autopilot": True, "map_registration": "measure"})
        self.assertNotIn("map_registration", module.visual_inertial_overrides("hybrid", "/g", ""))
        self.assertIsNone(module.visual_inertial_overrides("lidar_inertial", "", "measure"))
        run = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn('ros_launch_args+=(map_registration:="${MAP_REGISTRATION}")', run)
        self.assertIn("  MAP_REGISTRATION\n", (REPOSITORY / "scripts/container_run.sh").read_text(encoding="utf-8"))
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn('"map_registration",', launch)
        node = (REPOSITORY / "drone_city_nav/src/visual_inertial_odometry_node.cpp").read_text(encoding="utf-8")
        self.assertIn('declare_parameter<std::string>("map_registration", "act") != "measure"', node)

    def test_the_cave_profiles_subscribe_no_signal(self) -> None:
        # The topic is empty in the configuration: the estimator subscribes
        # nothing and pays nothing; only the hybrid launch names one.
        configuration = yaml.safe_load((REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text(encoding="utf-8"))
        parameters = configuration["visual_inertial_odometry_node"]["ros__parameters"]
        self.assertEqual(parameters["gnss_topic"], "")
        self.assertEqual(parameters["gnss_window_s"], 5.0)
        # The gates are twice the 99th percentile measured over r2626 to r2628
        # (tools/gnss_gates.py); they change only with a new measurement.
        self.assertEqual(parameters["gnss_position_gate"], 0.70)
        self.assertEqual(parameters["gnss_velocity_gate"], 1.26)
        self.assertEqual(parameters["gnss_self_gate_mps"], 2.0)
        self.assertEqual(parameters["gnss_jump_gate_m"], 0.16)
        self.assertEqual(parameters["gnss_clock_step_gate_s"], 0.072)
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


def write_run(directory: Path, active: list[str], changes: list[tuple[float, str]]) -> str:
    """A run's records: the relay's row per airborne second (`active`), its log
    with the airborne moment at 30 s of simulation, the true pose with the wall
    clock at 1000 s plus the simulation second, and the estimator's trust
    changes on that wall clock."""
    with (directory / "gnss_relay.csv").open("w", encoding="utf-8") as stream:
        stream.write("t_airborne_s,active,relayed,dropped,latitude_deg,longitude_deg,altitude_m,fix_type,eph_m\n")
        for second, name in enumerate(active):
            stream.write(f"{second}.0,{name},30,0,37.4,-121.9,39.0,3,0.9\n")
    (directory / "gnss_relay.log").write_text("GNSS_FAULT_RELAY airborne_at_us=30000000 faults=''\n",
                                              encoding="utf-8")
    with (directory / "gz_pose.csv").open("w", encoding="utf-8") as stream:
        for second in range(0, 400):
            stream.write(f"{second}.0,1,2,3,0,0,0,1,{1000 + second}.0\n")
    return "".join(f"[INFO] [{stamp:.3f}] [visual_inertial_odometry_node]: GNSS_TRUST changed "
                   f"trusted={state} reason=x\n" for stamp, state in changes)


class GnssFaultEvidenceTest(unittest.TestCase):
    def setUp(self) -> None:
        sys.path.insert(0, str(REPOSITORY / "scripts"))
        import gnss_trust_evidence  # noqa: PLC0415

        self.evidence = gnss_trust_evidence

    def report(self, active: list[str], changes: list[tuple[float, str]]) -> tuple[list[str], str]:
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            log = write_run(run, active, changes)
            errors: list[str] = []
            out = io.StringIO()
            with redirect_stdout(out):
                self.evidence.report_gnss_faults(run, log, errors, [])
            return errors, out.getvalue()

    def test_a_fault_met_in_time_and_held_to_its_end_passes(self) -> None:
        # lost from airborne second 100 to 160: simulation 130 to 190, wall 1130 to 1190.
        active = ["none"] * 100 + ["lost"] * 60 + ["none"] * 40
        errors, out = self.report(active, [(1040.0, "true"), (1130.5, "false"), (1196.0, "true")])
        self.assertEqual(errors, [])
        self.assertIn("OK: under the fault lost from 1130.0 to 1190.0: lost 0.5 s after the start, "
                      "back 6.0 s after the end", out)

    def test_a_late_loss_and_an_early_return_fail(self) -> None:
        active = ["none"] * 100 + ["walk"] * 60 + ["none"] * 40
        errors, _ = self.report(active, [(1040.0, "true"), (1135.0, "false"), (1196.0, "true")])
        self.assertEqual(len(errors), 1)
        self.assertIn("FAIL: the trust is lost within 2 s of the fault walk from 1130.0 to 1190.0 "
                      "(lost 5.0 s after)", errors[0])
        errors, _ = self.report(active, [(1040.0, "true"), (1130.2, "false"), (1170.0, "true")])
        self.assertEqual(len(errors), 1)
        self.assertIn("FAIL: the trust stays lost until the fault walk from 1130.0 to 1190.0 ends "
                      "(back 20.0 s before the end)", errors[0])

    def test_the_offset_a_fault_leaves_behind_keeps_the_trust_away(self) -> None:
        active = ["none"] * 100 + ["return_offset"] * 60 + ["returned_offset"] * 40
        errors, out = self.report(active, [(1040.0, "true"), (1130.3, "false")])
        self.assertEqual(errors, [])
        self.assertIn("OK: under the fault return_offset from 1130.0 on: lost 0.3 s after the start, "
                      "not back, as the fault stays", out)
        errors, _ = self.report(active, [(1040.0, "true"), (1130.3, "false"), (1200.0, "true")])
        self.assertEqual(len(errors), 1)
        self.assertIn("back although the fault stays", errors[0])

    def test_the_returns_the_law_allows_are_notes(self) -> None:
        # A velocity lie while the vehicle is slow is under the gate; an offset
        # the estimator's own drift since the outage could explain is taken
        # back by the law's limit (r2659, r2661).
        active = ["none"] * 100 + ["bad_velocity"] * 60 + ["none"] * 40
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            log = write_run(run, active, [(1040.0, "true"), (1130.1, "false"), (1185.0, "true")])
            # The truth stands still: the written record moves nowhere.
            errors: list[str] = []
            notes: list[str] = []
            with redirect_stdout(io.StringIO()):
                self.evidence.report_gnss_faults(run, log, errors, notes, "bad_velocity@100:60")
            self.assertEqual(errors, [])
            self.assertEqual(len(notes), 1)
            self.assertIn("lies about a motion of 0.00 m/s", notes[0])
        active = ["none"] * 100 + ["return_offset"] * 60 + ["returned_offset"] * 40
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            log = write_run(run, active, [(1040.0, "true"), (1130.3, "false"), (1190.0, "true")])
            log += ("[INFO] [1189.000] [visual_inertial_odometry_node]: GNSS_TRUST trusted=false agreeing=true "
                    "reason=agreeing drift_m=45.00\n")
            errors, notes = [], []
            with redirect_stdout(io.StringIO()):
                self.evidence.report_gnss_faults(run, log, errors, notes, "return_offset@100:60")
            self.assertEqual(errors, [])
            self.assertIn("offset of 50 m is within the drift", notes[0])
            log = log.replace("drift_m=45.00", "drift_m=10.00")
            errors, notes = [], []
            with redirect_stdout(io.StringIO()):
                self.evidence.report_gnss_faults(run, log, errors, notes, "return_offset@100:60")
            self.assertEqual(len(errors), 1)
            self.assertIn("back although the fault stays", errors[0])

    def test_a_faulted_flight_arrives_within_the_published_uncertainty(self) -> None:
        import headless_runtime_evidence as check  # noqa: PLC0415

        with tempfile.TemporaryDirectory() as directory:
            truth = Path(directory) / "gz_pose.csv"
            truth.write_text("10.0,0,0,0,0,0,0,1,1010.0\n11.0,3,0,0,0,0,0,1,1011.0\n", encoding="utf-8")
            log = ("[INFO] [1010.500] [visual_inertial_odometry_node]: VISUAL_INERTIAL_ODOMETRY healthy=true "
                   "position_sigma_m=(0.60,0.80,0.10)\n"
                   "[INFO] [1011.000] [production_mppi_node]: MISSION_WAYPOINT_ACKNOWLEDGED index=0 "
                   "goal=(0.0,0.0,0.0)\n")
            errors: list[str] = []
            with redirect_stdout(io.StringIO()):
                check.validate_goal_reached_in_truth(truth, log, errors)
            self.assertEqual(len(errors), 1)
            self.assertIn("inside the goal's capture radius", errors[0])
            errors = []
            out = io.StringIO()
            with redirect_stdout(out):
                check.validate_goal_reached_in_truth(truth, log, errors, faulted_signal=True)
            self.assertEqual(errors, [])
            self.assertIn("OK: the true position is 3.00 m from the goal under the signal's faults at its "
                          "acknowledgement (allowed two sigma 1.00 plus 2.0 m = 4.00 m)", out.getvalue())
