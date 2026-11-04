#!/usr/bin/env python3
"""The stack bag and its replay (roadmap item 27 block 6): the topics the recorder takes cover what the stack reads
and what it writes, the flight writes its launch arguments and stops the recorder before its processes, the launch
runs the stack without Gazebo on request, and the comparison's measures read as they should."""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import stack_bag_topics as topics  # noqa: E402


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compare = load_module("replay_compare", REPOSITORY / "tools" / "replay_compare.py")


class StackBagTopicsTest(unittest.TestCase):
    def test_the_inputs_follow_the_profiles_and_never_meet_the_outputs(self) -> None:
        cameras = topics.inputs("stereo_tof", "stereo_tof")
        lidar = topics.inputs("lidar", "none")
        for topic in ("/stereo/left/image", "/tof/down/points", "/magnetometer/magnetic_field"):
            self.assertIn(topic, cameras)
            self.assertNotIn(topic, lidar)
        self.assertIn("/lidar_3d/points", lidar)
        self.assertNotIn("/lidar_3d/points", cameras)
        for inputs in (cameras, lidar):
            self.assertIn("/fmu/out/sensor_combined", inputs)
            self.assertIn("/drone_city_nav/operator_command", inputs)
            self.assertNotIn(topics.CLOCK, inputs)
            self.assertEqual([], [t for t in inputs if t in topics.OUTPUTS])

    def test_the_autopilot_topics_the_configuration_names_are_inputs(self) -> None:
        import re
        configuration = (REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text(encoding="utf-8")
        named = set(re.findall(r"(/fmu/out/[a-z_0-9]+)", configuration))
        self.assertTrue(named)
        self.assertEqual(set(), named - set(topics.INPUTS_COMMON), "an autopilot topic the stack reads is not recorded")

    def test_a_bag_filters_the_topics_it_holds(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            (Path(temp) / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n  topics_with_message_count:\n"
                "    - topic_metadata:\n        name: /stereo/left/image\n        type: sensor_msgs/msg/Image\n"
                "    - topic_metadata:\n        name: /clock\n        type: rosgraph_msgs/msg/Clock\n")
            self.assertEqual({"/stereo/left/image", "/clock"}, topics.bag_topics(Path(temp)))


class ReplayContractTest(unittest.TestCase):
    def test_the_flight_records_on_request_and_stops_the_recorder_first(self) -> None:
        evidence = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text(encoding="utf-8")
        self.assertIn('if bool_is_true "${RECORD_STACK_BAG:-false}"; then', evidence)
        self.assertIn("stack_bag_topics.py\" record", evidence)
        self.assertIn("RECORD_STACK_BAG \\\n", evidence)
        self.assertIn("stop_stack_bag_recorder() {", evidence)
        runner = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn("  trap - EXIT INT TERM\n  stop_stack_bag_recorder\n", runner)
        self.assertIn('printf \'%s\\n\' "${ros_launch_args[@]}" > "${runtime_artifact_dir}/ros_launch_args.txt"', runner)
        self.assertIn("  RECORD_STACK_BAG\n", (REPOSITORY / "scripts/container_run.sh").read_text(encoding="utf-8"))

    def test_the_launch_runs_the_stack_without_gazebo(self) -> None:
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn('executable="gazebo_stereo_depth_node" if gazebo else "stereo_depth_node"', launch)
        self.assertIn("] if gazebo else []", launch)
        self.assertIn("gazebo=gazebo_bridge_enabled,", launch)
        self.assertIn('if gazebo_bridge_enabled and navigation_sensors == "lidar":', launch)
        replay = (REPOSITORY / "tools/replay_stack.sh").read_text(encoding="utf-8")
        self.assertIn('enable_gazebo_bridge:=false', replay)
        self.assertIn("stack_bag_topics.py inputs", replay)
        self.assertIn("replay_compare.py", replay)
        self.assertIn("--clock 250", replay)


class Stamped:
    def __init__(self, sec: int, nanosec: int, x: float, y: float, z: float) -> None:
        self.header = type("Header", (), {})()
        self.header.stamp = type("Stamp", (), {"sec": sec, "nanosec": nanosec})()
        self.pose = type("Pose", (), {})()
        self.pose.position = type("Point", (), {"x": x, "y": y, "z": z})()


class ReplayCompareTest(unittest.TestCase):
    def test_the_poses_match_by_stamp_and_the_distance_is_judged_at_p95(self) -> None:
        flight = [(b"", Stamped(1, k, 0.0, 0.0, 0.0)) for k in range(100)]
        replay = [(b"", Stamped(1, k, 0.0, 0.0, 0.0)) for k in range(5, 105)]
        line, ok = compare.compare_poses(flight, replay)
        self.assertTrue(ok)
        self.assertIn("unmatched flight 5 replay 5", line)
        replay = [(b"", Stamped(1, k, 0.1 if k >= 90 else 0.0, 0.0, 0.0)) for k in range(100)]
        line, ok = compare.compare_poses(flight, replay)
        self.assertFalse(ok)
        self.assertIn("max 0.100", line)
        replay = [(b"", Stamped(1, k, 0.1 if k >= 96 else 0.0, 0.0, 0.0)) for k in range(100)]
        self.assertTrue(compare.compare_poses(flight, replay)[1], "four of a hundred beyond: p95 within")

    def test_the_setpoints_compare_bytes_and_finite_fields(self) -> None:
        def setpoint(stamp: int, x: float, vx: float, raw: bytes):
            return (raw, type("S", (), {"timestamp": stamp, "position": [x, 0.0, math.nan],
                                        "velocity": [vx, math.nan, math.nan]})())
        flight = [setpoint(k, 1.0, 0.5, b"a") for k in range(10)]
        replay = [setpoint(k, 1.0, 0.5, b"a") for k in range(9)] + [setpoint(9, 1.5, 0.5, b"b")]
        line = compare.compare_setpoints(flight, replay)
        self.assertIn("9 of 10 matched setpoints byte-equal (90.0 percent)", line)
        self.assertIn("max 0.500 m", line)
        self.assertEqual(math.inf, compare.vector_difference([math.nan, 1.0], [0.0, 1.0]))

    def test_the_events_name_the_first_difference(self) -> None:
        self.assertEqual("3 events, the same in order", compare.compare_events(["a", "b", "c"], ["a", "b", "c"]))
        self.assertIn("event 2 differs", compare.compare_events(["a", "b"], ["a", "c"]))
        self.assertIn("then the flight has 3 and the replay 2", compare.compare_events(["a", "b", "c"], ["a", "b"]))


if __name__ == "__main__":
    unittest.main()
