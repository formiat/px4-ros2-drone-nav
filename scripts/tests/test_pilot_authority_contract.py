"""Roadmap item 27 block 1 (specification K29): the pilot arms the vehicle and chooses its mode.
No production source sends PX4 an arming or a mode switch; the only command to the autopilot's
arming is the disarm of a destroyed vehicle. The evaluation pilot does the arming, is started
with every flight, and its schedule parses."""
import importlib.util
import re
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
PILOT_PATH = REPOSITORY / "scripts" / "sim_pilot.py"
SPEC = importlib.util.spec_from_file_location("sim_pilot", PILOT_PATH)
assert SPEC is not None and SPEC.loader is not None
PILOT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PILOT
SPEC.loader.exec_module(PILOT)


class PilotAuthorityContractTest(unittest.TestCase):
    def production_sources(self):
        for folder in ("src", "include"):
            for path in (REPOSITORY / "drone_city_nav" / folder).rglob("*"):
                if path.suffix in {".cpp", ".hpp"}:
                    yield path, path.read_text(encoding="utf-8")

    def test_no_production_source_switches_a_mode(self) -> None:
        for path, text in self.production_sources():
            if path.name == "px4_offboard_setpoint_io.cpp":
                continue  # the command names for logs
            self.assertNotIn("VEHICLE_CMD_DO_SET_MODE", text, str(path))

    def test_the_only_arming_command_is_the_disarm_of_a_destroyed_vehicle(self) -> None:
        for path, text in self.production_sources():
            for match in re.finditer(r"VEHICLE_CMD_COMPONENT_ARM_DISARM,\s*([^,)]+)", text):
                self.assertEqual(match.group(1).strip(), "0.0F", f"{path}: arms the vehicle")

    def test_the_pilot_flies_with_every_flight(self) -> None:
        runtime = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        start = runtime.index("start_runtime_evidence_capture() {")
        camera_branch = runtime.index('if [[ "${CAMERA_PROFILE:-}" == "stereo_tof" ]]')
        self.assertIn("scripts/sim_pilot.py", runtime[start:camera_branch])

    def test_the_schedule_parses_and_refuses_what_it_does_not_know(self) -> None:
        events = PILOT.parse_events("land@90;hold@70;cmd:goal:1,2,3@50;cmd:stop@40;rc_loss@60")
        self.assertEqual([event.at_s for event in events], [40.0, 50.0, 60.0, 70.0, 90.0])
        self.assertEqual(PILOT.operator_text("cmd:goal:1,2,3"), "goal 1 2 3")
        for wrong in ("arm@10", "cmd:fly@10", "cmd:goal:1,2@10", "hold"):
            with self.assertRaises(ValueError):
                PILOT.parse_events(wrong)

    def test_the_failsafe_ladder_commands_no_mode_and_lands_by_itself(self) -> None:
        # Specification K90 and K29: a planner lost for five seconds is answered
        # by the mission's own landing, given up if the planner is heard again
        # before the ground; a position source that says nothing is dead
        # reckoning. The only command the offboard sends the autopilot is the
        # disarm of a vehicle already destroyed.
        source = REPOSITORY / "drone_city_nav" / "src"
        offboard = "".join(
            path.read_text(encoding="utf-8")
            for path in sorted(source.glob("mppi_offboard_node*.cpp"))
        )
        self.assertNotIn("VEHICLE_CMD_NAV_LAND", offboard)
        self.assertNotIn("VEHICLE_CMD_DO_SET_MODE", offboard)
        self.assertEqual(offboard.count("publishCommand("), 2)
        self.assertIn(
            "    if (!emergency_landing_ && loss_s >= planner_health_land_after_s_) {\n",
            offboard,
        )
        self.assertIn(
            "    if (emergency_landing_) {\n"
            "      publishMissionLandingSetpoint();\n"
            "    } else {\n"
            "      publishUnavailablePathHoldSetpoint();\n"
            "    }\n",
            offboard,
        )
        self.assertIn(
            "    emergency_landing_ = false;\n"
            "    mission_landing_descent_flowing_ = false;\n",
            offboard,
        )
        self.assertIn(
            "  state.landing = blind_landing_ || mission_landing_ || emergency_landing_ ||\n",
            offboard,
        )
        self.assertIn(
            "  dead_reckoning_landing_.observeSilence(now().nanoseconds());\n", offboard
        )
        self.assertIn("dead_reckoning_landing_.silent())));\n", offboard)

    def test_the_stacks_own_mode_is_registered_on_the_launchs_word_and_selected_by_the_pilot(
        self,
    ) -> None:
        # Specification K91: the stack registers a mode of PX4's own where the
        # launch asks (offboard otherwise), still commands no mode (the test
        # above), and the evaluation pilot selects it as it selects offboard.
        launch_arguments = (REPOSITORY / "drone_city_nav" / "launch" / "launch_arguments.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"px4_external_mode",\n            default_value="false",', launch_arguments)
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"px4_external_mode": LaunchConfiguration("px4_external_mode")', launch)
        flight = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn(
            'ros_launch_args+=(px4_external_mode:="$(normalize_bool "${PX4_EXTERNAL_MODE:-false}")")',
            flight,
        )
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text(encoding="utf-8")
        self.assertIn('bool_is_true "${PX4_EXTERNAL_MODE:-false}" && echo --external-mode', evidence)
        container = (REPOSITORY / "scripts" / "container_run.sh").read_text(encoding="utf-8")
        self.assertIn("  PX4_EXTERNAL_MODE\n", container)
        offboard = (REPOSITORY / "drone_city_nav" / "src" / "mppi_offboard_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('declare_parameter<bool>("px4_external_mode", false)', offboard)
        mode = (REPOSITORY / "drone_city_nav" / "src" / "mppi_offboard_node_external_mode.hpp").read_text(
            encoding="utf-8"
        )
        # The flags PX4 derives for offboard: position for the hold and the
        # trajectory, velocity for the cruise, the climb rate in both.
        self.assertIn(
            "configuration.position_enabled = mode_ != OffboardSetpointMode::kVelocityCruise;", mode
        )
        self.assertIn("configuration.climb_rate_enabled = true;", mode)
        self.assertIn("return 0.0F;", mode)
        # The pilot finds the stack's mode among the external modes PX4 lists.
        self.assertIsNone(PILOT.external_nav_state(1 << 14))
        self.assertEqual(23, PILOT.external_nav_state((1 << 14) | (1 << 23)))
        self.assertEqual(25, PILOT.external_nav_state((1 << 25) | (1 << 31)))
        self.assertEqual(11, PILOT.FIRST_EXTERNAL_SUB_MODE)
        # An external mode cannot take off: the pilot arms in Takeoff and
        # selects the stack's mode once the autopilot's takeoff has ended.
        pilot = PILOT_PATH.read_text(encoding="utf-8")
        self.assertEqual((4, 2), PILOT.MODES["takeoff"])
        self.assertEqual(17, PILOT.TAKEOFF_NAV_STATE)
        self.assertEqual((1.5, 0.1), (PILOT.TAKEN_OFF_M, PILOT.CLIMB_STOPPED_MPS))
        # Through Hold: a mode selected while the takeoff wishes to take off
        # keeps the wish, and the autopilot never disarms on the ground.
        self.assertEqual((4, 2.0), (PILOT.LOITER_NAV_STATE, PILOT.HOLD_BEFORE_THE_MODE_S))
        self.assertIn(
            "                elif status.nav_state == LOITER_NAV_STATE:\n"
            "                    vehicle.hold_since_s = vehicle.hold_since_s or clock_s()\n"
            "                    if clock_s() - vehicle.hold_since_s >= HOLD_BEFORE_THE_MODE_S:\n"
            '                        vehicle.set_mode("external")\n'
            "                elif vehicle.taken_off:\n"
            '                    vehicle.set_mode("hold")\n',
            pilot,
        )
        self.assertIn(
            "                              and position.z < self.ground_z - TAKEN_OFF_M\n"
            "                              and abs(position.vz) < CLIMB_STOPPED_MPS)\n",
            pilot,
        )

    def test_an_injected_fault_stops_and_resumes_the_named_process_alone(self) -> None:
        # Specification K90: the failsafe ladder's rungs are entered by a hung
        # planner or estimator, which the schedule stops and lets go on.
        import os
        import shutil
        import signal
        import subprocess
        import sys
        import tempfile
        import time

        events = PILOT.parse_events("fault:resume:probe@9;fault:stop:probe@5")
        self.assertEqual([event.action for event in events],
                         ["fault:stop:probe", "fault:resume:probe"])
        for wrong in ("fault:kill:probe@5", "fault:stop@5", "fault:stop:@5"):
            with self.assertRaises(ValueError):
                PILOT.parse_events(wrong)

        def state(pid: int) -> str:
            return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]

        with tempfile.TemporaryDirectory() as directory:
            probe = Path(directory) / "k90_fault_probe"
            other = Path(directory) / "k90_fault_other"
            # A copy of the interpreter under each name: a multi-call
            # `sleep` reads its own name and refuses one it does not know.
            interpreter = os.path.realpath(sys.executable)
            shutil.copy(interpreter, probe)
            shutil.copy(interpreter, other)
            wait = ["-c", "import time; time.sleep(30)"]
            named = subprocess.Popen([str(probe), *wait])
            beside = subprocess.Popen([str(other), *wait])
            try:
                time.sleep(0.5)
                self.assertEqual(PILOT.signal_processes("k90_fault_probe", signal.SIGSTOP),
                                 [named.pid])
                time.sleep(0.1)
                self.assertEqual(state(named.pid), "T")
                self.assertNotEqual(state(beside.pid), "T")
                self.assertEqual(PILOT.signal_processes("k90_fault_probe", signal.SIGCONT),
                                 [named.pid])
                time.sleep(0.1)
                self.assertNotEqual(state(named.pid), "T")
                self.assertEqual(PILOT.signal_processes("k90_no_such_process", signal.SIGSTOP), [])
            finally:
                named.kill()
                beside.kill()
                named.wait()
                beside.wait()


if __name__ == "__main__":
    unittest.main()
