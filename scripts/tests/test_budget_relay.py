"""The budget relay (roadmap item 36, specification K53): the stamps read off serialized bytes, the launch's wiring of
the loop's consumers, and the contract that carries BUDGET_RELAY to the container, the manifest and the relay with
no production source reading it."""
from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
REPOSITORY = SCRIPTS_DIR.parent
LAUNCH_DIR = REPOSITORY / "drone_city_nav" / "launch"


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


RELAY = load("budget_relay_script", SCRIPTS_DIR / "budget_relay.py")
WIRING = load("budget_relay_wiring", LAUNCH_DIR / "budget_relay.py")


class StampTest(unittest.TestCase):
    def test_the_stamps_are_read_off_real_serializations(self) -> None:
        from rclpy.serialization import serialize_message
        from geometry_msgs.msg import PoseStamped
        from px4_msgs.msg import VehicleOdometry
        from sensor_msgs.msg import PointCloud2

        # The package's own messages lead with the same std_msgs/Header (RawObstacleDelta3D, RawObstacleSnapshot3D,
        # LatestSensorObstacleScan, MppiTrajectoryHorizon: the first field of each .msg), so the layout is one.
        for message in (PointCloud2(), PoseStamped()):
            message.header.stamp.sec = 1234
            message.header.stamp.nanosec = 567_000_001
            message.header.frame_id = "map"
            self.assertEqual(RELAY.stamp_ns_of(serialize_message(message), True), 1234_567_000_001)
        odometry = VehicleOdometry()
        odometry.timestamp = 99
        odometry.timestamp_sample = 1_234_567
        self.assertEqual(RELAY.stamp_ns_of(serialize_message(odometry), False), 1_234_567_000)

    def test_every_relayed_output_has_its_profile_ages(self) -> None:
        names = {r.name for r in RELAY.RELAYED}
        self.assertEqual(names, {"depth_points", "odometry", "memory_delta", "memory_snapshot", "latest_scan",
                                 "horizon"})
        for relayed in RELAY.RELAYED:
            self.assertTrue(relayed.target.endswith(RELAY.BUDGETED_SUFFIX) or relayed.target == RELAY.ODOMETRY)
            for age in relayed.age_ms.values():
                self.assertGreater(age, 0.0)
        self.assertEqual(RELAY.percentiles([3.0, 1.0, 2.0]), (2.0, 3.0, 3.0))


class WiringTest(unittest.TestCase):
    def test_off_changes_nothing(self) -> None:
        planner, offboard, vio, lio, memory = [], {}, {}, {}, []
        self.assertEqual(WIRING.apply_budget_relay("false", "stereo_tof", planner, offboard, vio, lio, memory), [])
        self.assertEqual((planner, offboard, vio, lio, memory), ([], {}, {}, {}, []))

    def test_on_rewires_the_loops_consumers(self) -> None:
        planner, offboard, vio, lio, memory = [], {}, {}, None, []
        arguments = WIRING.apply_budget_relay("true", "stereo_tof", planner, offboard, vio, lio, memory)
        self.assertEqual(planner, [{
            "raw_obstacle_delta_3d_topic": "/drone_city_nav/raw_obstacle_delta_3d/budgeted",
            "raw_obstacle_snapshot_3d_topic": "/drone_city_nav/raw_obstacle_snapshot_3d/budgeted",
            "latest_sensor_obstacle_scan_topic": "/drone_city_nav/latest_sensor_obstacle_scan/budgeted"}])
        self.assertEqual(offboard, {"mppi_execution_horizon_topic": "/drone_city_nav/mppi/execution_horizon/budgeted"})
        self.assertEqual(vio, {"px4_visual_odometry_topic": RELAY.UNBUDGETED_ODOMETRY})
        self.assertEqual(memory, [{"lidar_3d_topic": "/stereo_depth/points/budgeted"}])
        self.assertEqual(arguments, ["--ros-args", "-r",
                                     "visual_inertial_odometry_node:/stereo_depth/points:=/stereo_depth/points/budgeted"])
        # The relay's sources and targets are the wiring's names.
        by_name = {r.name: r for r in RELAY.RELAYED}
        self.assertEqual(by_name["horizon"].target, offboard["mppi_execution_horizon_topic"])
        self.assertEqual(by_name["memory_delta"].target, planner[0]["raw_obstacle_delta_3d_topic"])
        self.assertEqual(by_name["depth_points"].target, memory[0]["lidar_3d_topic"])
        planner, offboard, vio, lio, memory = [], {}, None, {}, []
        self.assertEqual(WIRING.apply_budget_relay("true", "lidar", planner, offboard, vio, lio, memory), [])
        self.assertEqual(lio, {"px4_visual_odometry_topic": RELAY.UNBUDGETED_ODOMETRY})
        self.assertEqual(memory, [])


class ContractTest(unittest.TestCase):
    def test_the_variable_reaches_the_container_the_manifest_the_launch_and_the_relay(self) -> None:
        self.assertIn("  BUDGET_RELAY\n", (REPOSITORY / "scripts" / "container_run.sh").read_text())
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn("IMU_FAULT_SEED BUDGET_RELAY \\", evidence)
        self.assertIn('budget_relay.py" --mode "${BUDGET_RELAY}"', evidence)
        resources = (REPOSITORY / "scripts" / "simulation_resource_runtime.sh").read_text()
        self.assertIn('[[ "${BUDGET_RELAY:-off}" == on ]] && ros_launch_args+=(budget_relay:=true)', resources)
        launch = (LAUNCH_DIR / "city_nav.launch.py").read_text()
        self.assertIn('"budget_relay"', launch)
        self.assertIn("apply_budget_relay(", launch)

    def test_no_production_source_reads_the_relay(self) -> None:
        for path in (REPOSITORY / "drone_city_nav").rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".yaml"}:
                continue
            text = path.read_text(errors="ignore")
            for word in ("BUDGET_RELAY", "/budgeted", "unbudgeted"):
                self.assertNotIn(word, text, f"{path} reads the relay")


if __name__ == "__main__":
    unittest.main()
