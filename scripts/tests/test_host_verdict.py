from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

PATH = Path(__file__).resolve().parents[2] / "tools" / "host_verdict.py"
SPEC = importlib.util.spec_from_file_location("host_verdict", PATH)
assert SPEC is not None and SPEC.loader is not None
HOST = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HOST)


class HostVerdictTest(unittest.TestCase):
    def test_a_flight_on_a_host_that_kept_real_time_counts(self) -> None:
        self.assertEqual([], HOST.verdict("stereo_tof", 0.89, 1.0))
        self.assertEqual([], HOST.verdict("lidar", 1.0, 1.0))

    def test_a_slow_simulator_voids_the_flight_by_its_profiles_floor(self) -> None:
        # The stereo set renders the pair and holds 0.82 to 1.00 on a quiet
        # host; the lidar holds 1.00 (specification A7).
        self.assertTrue(HOST.verdict("stereo_tof", 0.78, 1.0))
        self.assertEqual([], HOST.verdict("stereo_tof", 0.82, 1.0))
        self.assertTrue(HOST.verdict("lidar", 0.9, 1.0))

    def test_a_frozen_host_voids_the_flight(self) -> None:
        # r720, r723: every process frozen for 3.7 and 7.3 s (A6).
        self.assertTrue(HOST.verdict("stereo_tof", 1.0, 3.7))
        self.assertTrue(HOST.verdict("stereo_tof", None, 0.0))

    def test_a_slowed_flight_is_judged_against_the_factor_it_asked_for(self) -> None:
        self.assertEqual([], HOST.verdict("stereo_tof", 0.5, 1.0, 0.5))
        self.assertTrue(HOST.verdict("stereo_tof", 0.40, 1.0, 0.5))
        self.assertTrue(HOST.verdict("lidar", 0.47, 1.0, 0.5))

    def test_a_starved_estimator_voids_a_flight_at_real_time(self) -> None:
        self.assertEqual([], HOST.verdict("lidar", 1.0, 1.0, 1.0, 180.0))
        self.assertTrue(HOST.verdict("lidar", 1.0, 1.0, 1.0, 900.0))
        line = ("[lidar_inertial_odometry_node]: LIDAR_INERTIAL_ODOMETRY healthy=true "
                "published=true scan_ms=70.1 imu_lag_ms=0.0 imu_gap_max_ms=12.0 "
                "pose_age_max_ms=164.5 imu_samples=7")
        self.assertEqual(["164.5"], HOST.POSE_AGE_PATTERN.findall(line))

    def test_the_watchdog_judges_the_last_window_after_the_flight_got_going(self) -> None:
        import importlib.util as util
        spec = util.spec_from_file_location("rtf_watchdog", PATH.parent / "rtf_watchdog.py")
        watchdog = util.module_from_spec(spec)
        spec.loader.exec_module(watchdog)
        startup = [0.03, 0.2, 0.5]
        self.assertEqual(("waiting", None), watchdog.judge(startup + [0.99] * 10, 0.82))
        self.assertEqual("ok", watchdog.judge(startup + [0.99] * 30, 0.82)[0])
        # Thirty seconds of a loaded host after a sound start.
        self.assertEqual("slow", watchdog.judge(startup + [0.99] * 40 + [0.6] * 30, 0.82)[0])
        # A slowed run that asked for 0.5 is judged against half the floor.
        self.assertEqual("ok", watchdog.judge([0.5] * 40, 0.41)[0])
        # The watch stops a flight at 0.85 of the verdict's floor, not at the floor.
        self.assertEqual(0.85, watchdog.WATCH_SHARE_OF_FLOOR)

    def test_every_launcher_under_load_is_tracked_and_documented(self) -> None:
        readme = (PATH.parents[1] / "tools/README.md").read_text()
        for tool in ("rtf_watchdog.py", "fly_until_valid.sh", "record_until_pass.sh"):
            self.assertTrue((PATH.parent / tool).is_file(), tool)
            self.assertIn(f"`{tool}`", readme)
        for launcher in ("fly_until_valid.sh", "record_until_pass.sh"):
            text = (PATH.parent / launcher).read_text()
            self.assertIn("tools/rtf_watchdog.py", text)
            self.assertIn("tools/host_verdict.py", text)

    def test_the_series_launcher_records_the_verdict(self) -> None:
        launcher = (PATH.parent / "series2.sh").read_text(encoding="utf-8")
        self.assertIn("tools/host_verdict.py", launcher)


if __name__ == "__main__":
    unittest.main()
