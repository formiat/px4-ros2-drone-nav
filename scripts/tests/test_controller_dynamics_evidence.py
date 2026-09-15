from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import controller_dynamics_evidence as evidence  # noqa: E402


def straight_flight(lateral_offset_m: float, speed_mps: float = 3.0, seconds: float = 20.0):
    """Setpoints along +x at `speed_mps` from t=0 (setpoint clock), the local
    position following 0.3 s later on its own clock, offset laterally."""
    setpoint_time = np.arange(0.0, seconds, 0.02)
    setpoints = np.column_stack([
        setpoint_time * 1e6, setpoint_time * speed_mps, np.zeros_like(setpoint_time),
        -5.0 * np.ones_like(setpoint_time), speed_mps * np.ones_like(setpoint_time),
        np.zeros_like(setpoint_time), np.zeros_like(setpoint_time)])
    clock_shift_s = 100.0
    position_time = setpoint_time + clock_shift_s
    positions = np.column_stack([
        position_time * 1e6, position_time * 1e6, setpoint_time * speed_mps,
        lateral_offset_m * np.ones_like(setpoint_time), -5.0 * np.ones_like(setpoint_time),
        speed_mps * np.ones_like(setpoint_time), np.zeros_like(setpoint_time),
        np.zeros_like(setpoint_time)])
    return setpoints, positions


class LateralTrackingTest(unittest.TestCase):
    def test_reads_the_lateral_offset_across_clocks(self) -> None:
        setpoints, positions = straight_flight(0.12)
        measured = evidence.lateral_tracking_error_p99_m(setpoints, positions)
        self.assertGreater(measured.samples, 500)
        self.assertAlmostEqual(measured.value, 0.12, places=2)

    def test_ignores_hold_setpoints_and_slow_flight(self) -> None:
        setpoints, positions = straight_flight(0.12, speed_mps=1.0)
        self.assertEqual(evidence.lateral_tracking_error_p99_m(setpoints, positions).samples, 0)
        setpoints[:, 4:7] = np.nan
        self.assertEqual(evidence.lateral_tracking_error_p99_m(setpoints, positions).samples, 0)


class DescentArrestTest(unittest.TestCase):
    def test_measures_the_arrest_of_a_descent(self) -> None:
        time_s = np.arange(0.0, 6.0, 0.02)
        # Descending at 3 m/s for 2 s, arrested at 2.0 m/s^2 over 1.5 s, then level.
        vertical_up = np.where(time_s < 2.0, -3.0, np.minimum(-3.0 + 2.0 * (time_s - 2.0), 0.0))
        positions = np.column_stack([time_s * 1e6, time_s * 1e6, np.zeros_like(time_s),
                                     np.zeros_like(time_s), np.zeros_like(time_s),
                                     np.zeros_like(time_s), np.zeros_like(time_s),
                                     -vertical_up])
        measured = evidence.descent_arrest_median_mps2(positions)
        self.assertGreaterEqual(measured.samples, evidence.MINIMUM_DESCENT_ARREST_SAMPLES)
        self.assertAlmostEqual(measured.value, 2.0, places=1)

    def test_a_flight_without_descents_measures_nothing(self) -> None:
        time_s = np.arange(0.0, 6.0, 0.02)
        positions = np.zeros((len(time_s), 8))
        positions[:, 0] = time_s * 1e6
        self.assertEqual(evidence.descent_arrest_median_mps2(positions).samples, 0)


class PositionEstimateTest(unittest.TestCase):
    def test_aligns_the_clocks_and_reads_the_horizontal_error(self) -> None:
        # A straight flight along x whose speed breathes between 1.8 and 2.2 m/s,
        # so the speed profile has something to align on.
        sim_time = np.arange(0.0, 40.0, 0.02)
        truth = np.column_stack([sim_time, 2.0 * sim_time + 0.2 * np.sin(sim_time),
                                 np.zeros_like(sim_time), 7.0 * np.ones_like(sim_time)])
        log_time = np.arange(0.0, 40.0, 0.2) + 1.7e9
        flight_time = log_time - 1.7e9
        estimate = np.column_stack([log_time, 2.0 * flight_time + 0.2 * np.sin(flight_time) + 0.1,
                                    np.zeros_like(flight_time),
                                    2.0 + 0.2 * np.cos(flight_time)])
        # The estimate sits 0.1 m ahead along a 2 m/s motion: an along-track
        # offset of about +0.05 s, no cross-track error, 0.1 m in total.
        measured = evidence.position_estimate_error(estimate, truth)
        self.assertGreater(measured.samples, 100)
        self.assertAlmostEqual(measured.along_track_offset_s, 0.05, delta=0.01)
        self.assertLess(measured.cross_track_p95_m, 0.02)
        self.assertAlmostEqual(measured.total_p95_m, 0.1, places=1)


class LidarAgeAndValidationTest(unittest.TestCase):
    def test_reads_the_largest_lidar_age(self) -> None:
        log = ("[1.0] [production_mppi_node]: PRODUCTION_MPPI_TICK a=1 latest_lidar_obstacle_age_ms=120 b=2\n"
               "[1.2] [production_mppi_node]: PRODUCTION_MPPI_TICK a=1 latest_lidar_obstacle_age_ms=-1 b=2\n"
               "[1.4] [production_mppi_node]: PRODUCTION_MPPI_TICK a=1 latest_lidar_obstacle_age_ms=340 b=2\n")
        measured = evidence.lidar_evidence_age_max_ms(log)
        self.assertEqual(measured.samples, 2)
        self.assertEqual(measured.value, 340.0)

    def test_validation_names_every_missing_record(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            errors: list[str] = []
            evidence.validate_controller_dynamics(Path(directory), "", errors)
        self.assertEqual(len(errors), 3)
        self.assertTrue(any("tracking.npz" in error for error in errors))
        self.assertTrue(any("gz_pose.csv" in error for error in errors))
        self.assertTrue(any("lidar evidence age" in error for error in errors))

    def test_validation_passes_a_clean_flight(self) -> None:
        setpoints, positions = straight_flight(0.05)
        sim_time = np.arange(0.0, 20.0, 0.02)
        truth = np.column_stack([sim_time, 3.0 * sim_time + 0.2 * np.sin(sim_time),
                                 0.05 * np.ones_like(sim_time), 5.0 * np.ones_like(sim_time)])
        log_time = np.arange(0.0, 20.0, 0.2) + 1.7e9
        log = "".join(
            f"[{stamp:.6f}] [production_mppi_node]: PRODUCTION_MPPI_TICK tick=1 "
            f"state_position=({3.0 * (stamp - 1.7e9) + 0.2 * np.sin(stamp - 1.7e9):.3f},0.100,5.000) "
            f"state_velocity=({3.0 + 0.2 * np.cos(stamp - 1.7e9):.3f},0.000,0.000) "
            "latest_lidar_obstacle_age_ms=150\n"
            for stamp in log_time)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            np.savez_compressed(run / "tracking.npz", sp=setpoints, lp=positions)
            np.savetxt(run / "gz_pose.csv", truth, delimiter=",")
            errors: list[str] = []
            evidence.validate_controller_dynamics(run, log, errors)
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
