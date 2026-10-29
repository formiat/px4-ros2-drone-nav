"""The step controller (roadmap item 36): the launch's wiring of the acknowledgement into the onboard nodes, and
the contract that carries STEP_CONTROLLER to the container, the manifest, the launch and the controller, with no
production source reading it (the nodes know only their parameter and their acknowledgement)."""
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


WIRING = load("budget_relay_wiring_for_steps", LAUNCH_DIR / "budget_relay.py")


class WiringTest(unittest.TestCase):
    def test_off_changes_nothing(self) -> None:
        planner, memory, offboard, monitor, vio, lio = [], [], {}, [], {}, None
        WIRING.apply_step_ack("false", planner, memory, offboard, monitor, vio, lio)
        self.assertEqual((planner, memory, offboard, monitor, vio), ([], [], {}, [], {}))

    def test_on_turns_the_acknowledgement_on_in_every_onboard_node(self) -> None:
        planner, memory, offboard, monitor, vio, lio = [], [], {}, [], {}, {}
        WIRING.apply_step_ack("true", planner, memory, offboard, monitor, vio, lio)
        for parameters in (planner, memory, monitor):
            self.assertEqual(parameters, [{"step_ack_enabled": True}])
        for overrides in (offboard, vio, lio):
            self.assertEqual(overrides, {"step_ack_enabled": True})


class ContractTest(unittest.TestCase):
    def test_the_variable_reaches_the_container_the_manifest_the_launch_and_the_controller(self) -> None:
        container = (REPOSITORY / "scripts" / "container_run.sh").read_text()
        self.assertIn("  STEP_CONTROLLER\n", container)
        self.assertIn("  STEP_MS\n", container)
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn("BUDGET_RELAY STEP_CONTROLLER \\", evidence)
        self.assertIn('step_controller.py" --world "${world_name}"', evidence)
        self.assertIn("step_nodes=offboard,mission_monitor,memory,planner,visual_inertial", evidence)
        self.assertIn("step_nodes=offboard,mission_monitor,memory,planner,lidar_inertial", evidence)
        resources = (REPOSITORY / "scripts" / "simulation_resource_runtime.sh").read_text()
        self.assertIn("ros_launch_args+=(step_ack:=true)", resources)
        launch = (LAUNCH_DIR / "city_nav.launch.py").read_text()
        self.assertIn('"step_ack"', launch)
        self.assertIn("apply_step_ack(", launch)
        header = (REPOSITORY / "drone_city_nav" / "include" / "drone_city_nav" / "step_acknowledger.hpp").read_text()
        self.assertIn('declare_parameter<bool>("step_ack_enabled", false)', header)
        self.assertIn('"/drone_city_nav/step_ack/" + name', header)
        for name in ("offboard", "mission_monitor", "memory", "planner", "visual_inertial", "lidar_inertial"):
            self.assertGreaterEqual(
                sum(f'"{name}"' in path.read_text(errors="ignore")
                    for path in (REPOSITORY / "drone_city_nav" / "src").rglob("*.cpp")), 1, name)

    def test_no_production_source_reads_the_controller(self) -> None:
        for path in (REPOSITORY / "drone_city_nav").rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".yaml"}:
                continue
            text = path.read_text(errors="ignore")
            for word in ("STEP_CONTROLLER", "step_controller", "multi_step"):
                self.assertNotIn(word, text, f"{path} reads the controller")


if __name__ == "__main__":
    unittest.main()
