"""The budget relay (roadmap item 36, specification K53): the launch's wiring of the loop's consumers against the
relay's table (`drone_city_nav/src/budget_relay.hpp`, whose stamp reading the C++ unit test covers), and the
contract that carries BUDGET_RELAY to the container, the manifest and the relay with no production source reading
it."""
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


WIRING = load("budget_relay_wiring", LAUNCH_DIR / "budget_relay.py")
HEADER = (REPOSITORY / "drone_city_nav" / "src" / "budget_relay.hpp").read_text()


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
        self.assertEqual(vio, {"px4_visual_odometry_topic": WIRING.UNBUDGETED_ODOMETRY})
        self.assertEqual(memory, [{"lidar_3d_topic": "/stereo_depth/points/budgeted"}])
        self.assertEqual(arguments, ["--ros-args", "-r",
                                     "visual_inertial_odometry_node:/stereo_depth/points:=/stereo_depth/points/budgeted"])
        # The relay's targets (the C++ table) are the wiring's names.
        for target in (offboard["mppi_execution_horizon_topic"], *planner[0].values(), memory[0]["lidar_3d_topic"],
                       WIRING.UNBUDGETED_ODOMETRY):
            self.assertIn(f'"{target}"', HEADER, target)
        planner, offboard, vio, lio, memory = [], {}, None, {}, []
        self.assertEqual(WIRING.apply_budget_relay("true", "lidar", planner, offboard, vio, lio, memory), [])
        self.assertEqual(lio, {"px4_visual_odometry_topic": WIRING.UNBUDGETED_ODOMETRY})
        self.assertEqual(memory, [])


class ContractTest(unittest.TestCase):
    def test_the_variable_reaches_the_container_the_manifest_the_launch_and_the_relay(self) -> None:
        self.assertIn("  BUDGET_RELAY\n", (REPOSITORY / "scripts" / "container_run.sh").read_text())
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn("IMU_FAULT_SEED BUDGET_RELAY \\", evidence)
        self.assertIn('ros2 run drone_city_nav budget_relay_node --ros-args -p "mode:=${BUDGET_RELAY}"', evidence)
        resources = (REPOSITORY / "scripts" / "simulation_resource_runtime.sh").read_text()
        self.assertIn("ros_launch_args+=(budget_relay:=true)", resources)
        launch = (LAUNCH_DIR / "city_nav.launch.py").read_text()
        self.assertIn('"budget_relay"', launch)
        self.assertIn("apply_budget_relay(", launch)

    def test_no_production_source_reads_the_relay(self) -> None:
        for path in (REPOSITORY / "drone_city_nav").rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".yaml"} or path.stem in ("budget_relay", "budget_relay_node",
                                                                             "budget_relay_test"):
                continue
            text = path.read_text(errors="ignore")
            for word in ("BUDGET_RELAY", "/budgeted", "unbudgeted"):
                self.assertNotIn(word, text, f"{path} reads the relay")


if __name__ == "__main__":
    unittest.main()
