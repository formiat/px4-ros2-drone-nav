"""The real vehicle's imperfections profile (roadmap item 35, specification K52): the model's part written by the
preparation, the relays' seeded schedules and stamps, and the contract that carries IMPERFECTIONS to the container,
the manifest, the launch and the relays, with no production source reading it."""
from __future__ import annotations

import importlib.util
import random
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
REPOSITORY = SCRIPTS_DIR.parent
sys.path.insert(0, str(SCRIPTS_DIR))

import configure_imperfections as imperfections  # noqa: E402


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # the relays' dataclasses resolve their annotations by module
    spec.loader.exec_module(module)
    return module


IMU = load("imu_stream_faults")
CAMERA = load("camera_stream_faults")
LIDAR = load("lidar_sweep_jitter")

BASE_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="x500_base"><link name="base_link">
<sensor name="imu_sensor" type="imu"><imu>
<angular_velocity><x><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></x>
<y><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></y>
<z><noise type="gaussian"><mean>0</mean><stddev>0.0008726646</stddev></noise></z></angular_velocity>
<linear_acceleration><x><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></x>
<y><noise type="gaussian"><mean>0</mean><stddev>0.00637</stddev></noise></y>
<z><noise type="gaussian"><mean>0</mean><stddev>0.00686</stddev></noise></z></linear_acceleration>
</imu></sensor></link></model></sdf>
"""


class ConfigureImperfectionsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = Path(tempfile.mkdtemp())
        self.px4_models = self.temp / "px4_models"
        (self.px4_models / "x500_base").mkdir(parents=True)
        (self.px4_models / "x500_base" / "model.sdf").write_text(BASE_MODEL_SDF)
        self.runtime = self.temp / "runtime"
        self.runtime.mkdir()
        (self.runtime / "x500_base").symlink_to(self.px4_models / "x500_base")
        shutil.copytree(REPOSITORY / "drone_city_nav" / "models" / "x500_lidar_3d", self.runtime / "x500_lidar_3d")

    def tearDown(self) -> None:
        shutil.rmtree(self.temp, ignore_errors=True)

    def test_the_base_model_gets_the_bias_walk_in_a_copy(self) -> None:
        done = imperfections.configure(self.runtime, 3, self.px4_models)
        self.assertEqual(done["imu_axes_with_bias"], 6)
        base = self.runtime / "x500_base"
        self.assertFalse(base.is_symlink(), "the symlink into PX4's tree must become a copy")
        self.assertNotIn("dynamic_bias", (self.px4_models / "x500_base" / "model.sdf").read_text())
        root = ET.parse(base / "model.sdf").getroot()
        gyro = root.find(".//angular_velocity/x/noise")
        accel = root.find(".//linear_acceleration/z/noise")
        self.assertAlmostEqual(float(gyro.findtext("dynamic_bias_stddev")), 2.74e-4, delta=1e-5)
        self.assertAlmostEqual(float(accel.findtext("dynamic_bias_stddev")), 0.0424, delta=5e-4)
        self.assertEqual(gyro.findtext("dynamic_bias_correlation_time"), "400")
        self.assertEqual(gyro.findtext("stddev"), "0.0008726646", "the white noise stays the model's")

    def test_the_vehicle_model_gets_the_vibration_and_the_turned_sensor_pose(self) -> None:
        done = imperfections.configure(self.runtime, 0, self.px4_models)
        self.assertEqual(done["sensor_poses_turned"], 1)
        root = ET.parse(self.runtime / "x500_lidar_3d" / "model.sdf").getroot()
        model = root.find("model")
        plugins = [p for p in model.findall("plugin") if p.get("name") == imperfections.PLUGIN_NAME]
        self.assertEqual(len(plugins), 1)
        self.assertEqual(plugins[0].get("filename"), "libdrone_vibration_system.so")
        self.assertEqual(plugins[0].findtext("frequency_hz"), "55")
        self.assertEqual(plugins[0].findtext("acceleration_amplitude_mps2"), "0.192 0.552 0.38")
        self.assertEqual(plugins[0].findtext("seed"), "0")
        lidar = [i for i in model.findall("include") if "lidar_3d_v1" in (i.findtext("uri") or "")][0]
        values = [float(v) for v in lidar.findtext("pose").split()]
        self.assertAlmostEqual(values[4], imperfections.EXTRINSIC_ANGLE_RAD, places=7)
        self.assertAlmostEqual(values[5], imperfections.EXTRINSIC_ANGLE_RAD, places=7)
        self.assertEqual(values[:3], [0.12, 0.0, 0.5], "the position stays nominal")
        # Written twice, the model carries one plugin and one turn.
        imperfections.configure(self.runtime, 0, self.px4_models)
        root = ET.parse(self.runtime / "x500_lidar_3d" / "model.sdf").getroot()
        self.assertEqual(len([p for p in root.find("model").findall("plugin")
                              if p.get("name") == imperfections.PLUGIN_NAME]), 1)

    def test_the_turn_takes_its_signs_from_the_seed(self) -> None:
        self.assertEqual(imperfections.pose_turned("0 0 0.24 0 0 0", 0).split()[4:],
                         [f"{imperfections.EXTRINSIC_ANGLE_RAD:.6g}"] * 2)
        turned = [float(v) for v in imperfections.pose_turned("0 0 0.24 0 0 0", 1).split()]
        self.assertLess(turned[4], 0.0)
        self.assertGreater(turned[5], 0.0)
        turned = [float(v) for v in imperfections.pose_turned("0 0 0.24 0 0 0", 2).split()]
        self.assertGreater(turned[4], 0.0)
        self.assertLess(turned[5], 0.0)


class RelaysTest(unittest.TestCase):
    def test_the_imu_jitter_profile_is_seeded_and_at_the_recordings_rate(self) -> None:
        self.assertEqual(IMU.schedule("holes"), [IMU.ImuHole(60.0, 2.0), IMU.ImuHole(120.0, 4.0)])
        gaps = IMU.schedule("jitter", 5, 600.0)
        self.assertEqual(gaps, IMU.schedule("jitter", 5, 600.0))
        self.assertNotEqual(gaps, IMU.schedule("jitter", 6, 600.0))
        # 2.4 gaps a minute over ten minutes, within the draw's spread.
        self.assertGreater(len(gaps), 12)
        self.assertLess(len(gaps), 36)
        for gap in gaps:
            self.assertGreaterEqual(gap.length_s, IMU.GAP_SHORTEST_S)
            self.assertLessEqual(gap.length_s, IMU.GAP_LONGEST_S)
        rng = random.Random("test")
        previous = 1_000_000
        deviations = []
        for i in range(2000):
            stamp = 1_000_000 + 5_000 * (i + 1)
            jittered = IMU.jittered_stamp_us(stamp, previous, rng)
            self.assertGreater(jittered, previous)
            self.assertLessEqual(abs(jittered - stamp), IMU.JITTER_CLIP_S * 1e6 + 1)
            deviations.append(jittered - stamp)
            previous = jittered
        spread = (sum(d * d for d in deviations) / len(deviations)) ** 0.5
        self.assertGreater(spread, 1500.0)
        self.assertLess(spread, 3500.0)

    def test_the_frames_stamp_moves_by_the_offset_and_the_bytes_stay(self) -> None:
        from gz.msgs10.image_pb2 import Image

        frame = Image()
        frame.header.stamp.sec = 12
        frame.header.stamp.nsec = 999_000_000
        frame.width = 4
        frame.height = 2
        frame.data = bytes(range(24))
        moved = Image.FromString(CAMERA.with_stamp_offset(frame.SerializeToString(), 2_000_000))
        self.assertEqual((moved.header.stamp.sec, moved.header.stamp.nsec), (13, 1_000_000))
        self.assertEqual(moved.data, frame.data)
        back = Image.FromString(CAMERA.with_stamp_offset(frame.SerializeToString(), -2_000_000))
        self.assertEqual((back.header.stamp.sec, back.header.stamp.nsec), (12, 997_000_000))
        self.assertEqual(CAMERA.schedule("none", 1, 600.0), [])

    def test_the_sweeps_delay_stays_within_the_recordings_spread(self) -> None:
        rng = random.Random("sweeps")
        delays = [LIDAR.delay_s(rng) for _ in range(5000)]
        self.assertTrue(all(0.0 <= d <= LIDAR.DELAY_MAX_S for d in delays))
        self.assertAlmostEqual(sum(delays) / len(delays), 0.0105, delta=0.002)


class ContractTest(unittest.TestCase):
    def test_the_profile_reaches_the_container_the_manifest_and_the_relays(self) -> None:
        container = (REPOSITORY / "scripts" / "container_run.sh").read_text()
        for name in ("IMPERFECTIONS", "IMPERFECTION_SEED", "IMU_FAULT_SEED"):
            self.assertIn(f"  {name}\n", container)
        evidence = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn("IMPERFECTIONS IMPERFECTION_SEED IMU_FAULTS IMU_FAULT_SEED \\", evidence)
        self.assertIn("lidar_sweep_jitter.py", evidence)
        self.assertIn('--stamp-offset-ms "${frame_offset_ms}"', evidence)
        self.assertIn('--profile "${IMU_FAULTS}" --seed "${IMU_FAULT_SEED:-0}"', evidence)
        runtime = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text()
        self.assertIn("append_sensor_relay_launch_args", runtime)
        resources = (REPOSITORY / "scripts" / "simulation_resource_runtime.sh").read_text()
        self.assertIn('export IMU_FAULTS="${IMU_FAULTS:-jitter}"', resources)
        self.assertIn("lidar_scan_topic_suffix:=/relayed", resources)
        self.assertIn("configure_imperfections.py", resources)
        launch = (REPOSITORY / "drone_city_nav" / "launch" / "city_nav.launch.py").read_text()
        self.assertIn('"lidar_scan_topic_suffix"', launch)
        self.assertIn('lidar_inertial_overrides["px4_sensor_combined_topic"]', launch)
        cmake = (REPOSITORY / "drone_city_nav" / "CMakeLists.txt").read_text()
        self.assertIn("add_library(drone_vibration_system SHARED src/drone_vibration_system.cpp)", cmake)

    def test_no_production_source_reads_the_profile(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for path in production.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".yaml", ".msg"} or path.name == "drone_vibration_system.cpp":
                continue
            text = path.read_text(errors="ignore")
            for word in ("IMPERFECTION", "lidar_sweep_jitter", "configure_imperfections"):
                self.assertNotIn(word, text, f"{path} reads the profile")


if __name__ == "__main__":
    unittest.main()
