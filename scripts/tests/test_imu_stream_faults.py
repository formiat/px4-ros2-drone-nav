"""The IMU stream's failures stay an evaluation component: no production source reads
them, the runtime points the camera estimator at the relay only when they are on, and
the schedule is the two holes the entry describes."""
import importlib.util
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
INJECTOR_PATH = REPOSITORY / "scripts" / "imu_stream_faults.py"
SPEC = importlib.util.spec_from_file_location("imu_stream_faults", INJECTOR_PATH)
assert SPEC is not None and SPEC.loader is not None
INJECTOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = INJECTOR
SPEC.loader.exec_module(INJECTOR)


class ImuStreamFaultContractTest(unittest.TestCase):
    def test_no_production_source_reads_the_injection(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for path in production.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".yaml", ".msg"}:
                continue
            text = path.read_text(errors="ignore")
            for word in ("imu_stream_fault", "IMU_FAULT"):
                self.assertNotIn(word, text, f"{path} reads the injection")

    def test_the_runtime_points_the_estimator_at_the_relay(self) -> None:
        runtime = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text()
        self.assertIn(f"imu_topic_suffix:={INJECTOR.RELAY_SUFFIX}", runtime)
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('"px4_sensor_combined_topic"', launch)
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn("imu_stream_faults.py", evidence)

    def test_the_schedule_is_the_two_holes(self) -> None:
        holes = INJECTOR.schedule("holes")
        self.assertEqual([(h.start_s, h.length_s) for h in holes], [(60.0, 2.0), (120.0, 4.0)])
        self.assertIsNone(INJECTOR.hole_at(holes, 59.9))
        self.assertIs(INJECTOR.hole_at(holes, 61.0), holes[0])
        self.assertIsNone(INJECTOR.hole_at(holes, 62.0))
        with self.assertRaises(ValueError):
            INJECTOR.schedule("unknown")


if __name__ == "__main__":
    unittest.main()
