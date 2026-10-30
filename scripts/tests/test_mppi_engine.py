"""The MPPI engine's choice (roadmap item 28): MPPI_ENGINE reaches the container, the manifest and the launch, the
node reads mppi_engine as cuda or cpu, and the CUDA engine is a build option the controller honours."""
from __future__ import annotations

import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]


class MppiEngineContractTest(unittest.TestCase):
    def test_the_variable_reaches_the_container_the_manifest_and_the_launch(self) -> None:
        self.assertIn("  MPPI_ENGINE\n", (REPOSITORY / "scripts" / "container_run.sh").read_text())
        self.assertIn("MPPI_ROLLOUTS MPPI_ENGINE SMOKE_FACE", (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text())
        self.assertIn('ros_launch_args+=(mppi_engine:="${MPPI_ENGINE}")',
                      (REPOSITORY / "scripts" / "simulation_resource_runtime.sh").read_text())
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('"mppi_engine"', launch)
        self.assertIn("engine_overrides(mppi_engine.perform(context))", launch)

    def test_the_launch_override_accepts_cuda_and_cpu_only(self) -> None:
        import importlib.util
        import sys
        path = REPOSITORY / "drone_city_nav" / "launch" / "launch_overrides.py"
        spec = importlib.util.spec_from_file_location("launch_overrides_for_engine", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules["launch_overrides_for_engine"] = module
        spec.loader.exec_module(module)
        self.assertEqual(module.engine_overrides(""), {})
        self.assertEqual(module.engine_overrides(" cpu "), {"mppi_engine": "cpu"})
        self.assertEqual(module.engine_overrides("cuda"), {"mppi_engine": "cuda"})
        with self.assertRaises(RuntimeError):
            module.engine_overrides("vulkan")

    def test_the_node_reads_the_engine_and_the_build_gates_the_cuda_one(self) -> None:
        loader = (REPOSITORY / "drone_city_nav" / "src" / "runtime" / "ros" /
                  "production_mppi_config_ros_control.cpp").read_text()
        self.assertIn('declare<std::string>("mppi_engine", "cuda")', loader)
        controller = (REPOSITORY / "drone_city_nav" / "src" / "runtime" / "mppi_controller_3d.cpp").read_text()
        self.assertIn("#if DRONE_CITY_NAV_HAS_CUDA_MPPI", controller)
        self.assertIn("std::make_unique<mppi::MppiCpuEngine>(config)", controller)
        cmake = (REPOSITORY / "drone_city_nav" / "CMakeLists.txt").read_text()
        self.assertIn("src/mppi/mppi_cpu_engine.cpp", cmake)
        self.assertIn("DRONE_CITY_NAV_HAS_CUDA_MPPI=1", cmake)


if __name__ == "__main__":
    unittest.main()
