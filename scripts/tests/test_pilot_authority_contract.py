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


if __name__ == "__main__":
    unittest.main()
