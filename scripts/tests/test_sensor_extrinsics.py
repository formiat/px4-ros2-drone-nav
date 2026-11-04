#!/usr/bin/env python3
"""The sensor set's mounts held to their single source (roadmap item 27 block 3): every place that states a
sensor's pose (the models in the repository, the defaults written in the nodes' code, the launch's parameters, the
offline tools) agrees with drone_city_nav/config/sensor_extrinsics.yaml within 1e-6, and the simulator's runtime
model is written from it."""

from __future__ import annotations

import importlib.util
import math
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
MODELS = REPOSITORY / "drone_city_nav" / "models"
SOURCE = REPOSITORY / "drone_city_nav" / "config" / "sensor_extrinsics.yaml"
X500_BASE = REPOSITORY / "external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf"
TOLERANCE = 1e-6
sys.path.insert(0, str(REPOSITORY / "scripts"))
sys.path.insert(0, str(REPOSITORY / "drone_city_nav" / "launch"))

import configure_drone_lidar_model as generator  # noqa: E402
import sensor_extrinsics  # noqa: E402


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(element) -> list[float]:
    return generator.pose_values(element)


def numbers(text: str) -> list[float]:
    """The literals of a C++ initializer, std::numbers::pi read as itself."""
    values = []
    for token in text.replace("{", "").replace("}", "").split(","):
        token = token.strip()
        if not token:
            continue
        if "std::numbers::pi" in token:
            coefficient = token.replace("std::numbers::pi", "").replace("*", "").strip()
            values.append({"": 1.0, "-": -1.0}.get(coefficient, None) or float(coefficient))
            values[-1] *= math.pi
        else:
            values.append(float(token))
    return values


class SensorExtrinsicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.extrinsics = sensor_extrinsics.load(SOURCE)
        self.camera = sensor_extrinsics.camera_parameters(self.extrinsics)
        self.lidar = sensor_extrinsics.lidar_parameters(self.extrinsics)

    def assert_close(self, expected, actual, what: str) -> None:
        self.assertEqual(len(expected), len(actual), what)
        for a, b in zip(expected, actual):
            self.assertAlmostEqual(a, b, delta=TOLERANCE, msg=what)

    def test_the_models_in_the_repository_hold_the_mounts(self) -> None:
        stereo = ET.parse(MODELS / "stereo_tof_v1" / "model.sdf").getroot().find("model/link[@name='stereo_tof_link']")
        block = pose(stereo)
        self.assert_close(self.extrinsics["stereo_tof_link"]["position_m"], block[:3], "stereo_tof_link")
        for sensor in stereo.findall("sensor"):
            mount = generator.STEREO_SENSOR_MOUNTS.get(sensor.get("name"))
            if mount is None:
                continue
            values = pose(sensor)
            self.assert_close(self.extrinsics[mount]["position_m"], [a + b for a, b in zip(values[:3], block[:3])],
                              sensor.get("name"))
            self.assert_close(self.extrinsics[mount]["rpy_rad"], values[3:], sensor.get("name"))
        wrapper = ET.parse(MODELS / "x500_lidar_3d" / "model.sdf").getroot()
        include = next(e for e in wrapper.iter("include") if e.findtext("uri") == "model://lidar_3d_v1")
        self.assertEqual(generator.lidar_include_pose(MODELS / "lidar_3d_v1" / "model.sdf", self.extrinsics),
                         generator.pose_text(pose(include)[:3], pose(include)[3:]))
        if X500_BASE.exists():
            base = ET.parse(X500_BASE).getroot().find("model")
            self.assertAlmostEqual(self.extrinsics["base_link_height_m"], pose(base)[2], delta=TOLERANCE)
            for kind in ("imu", "air_pressure", "magnetometer"):
                sensor = base.find(f"link[@name='base_link']/sensor[@type='{kind}']")
                self.assertEqual([0.0] * 6, pose(sensor), kind)

    def test_the_code_defaults_hold_the_mounts(self) -> None:
        source = (REPOSITORY / "drone_city_nav/src/stereo_depth_node.hpp").read_text()
        ring = source[source.index("kTofMounts = {{"):]
        ring = ring[: ring.index("}};")]
        for side in sensor_extrinsics.TOF_SIDES:
            line = re.search(rf'\{{"{side}", ([^\n]*)\}},', ring).group(1)
            values = numbers(line)
            self.assert_close(self.camera[f"tof_{side}_pitch_yaw_rad"], values[:2], f"kTofMounts {side}")
            self.assert_close(self.camera[f"tof_{side}_position_left_camera_flu_m"], values[2:], f"kTofMounts {side}")
        node = (REPOSITORY / "drone_city_nav/src/visual_inertial_odometry_node.cpp").read_text()
        default = re.search(r'"left_camera_position_body_frd_m", \{([^}]*)\}', node).group(1)
        self.assert_close(self.camera["left_camera_position_body_frd_m"], numbers(default), "the estimator's camera")
        calibration = (REPOSITORY / "drone_city_nav/src/stereo_calibration_ros.hpp").read_text()
        default = re.search(r'"right_from_left_translation_m", \{([^}]*)\}', calibration).group(1)
        self.assert_close(self.camera["right_from_left_translation_m"], numbers(default), "the pair's baseline")
        for name in ("obstacle_memory_3d_node.cpp", "lidar_inertial_odometry_node.cpp"):
            text = (REPOSITORY / "drone_city_nav/src" / name).read_text()
            translation = re.search(r'"lidar_extrinsic_translation_body_frd_m", \{([^}]*)\}', text).group(1)
            quaternion = re.search(r'"lidar_extrinsic_quaternion_lidar_flu_to_body_frd", \{([^}]*)\}', text).group(1)
            self.assert_close(self.lidar["lidar_extrinsic_translation_body_frd_m"], numbers(translation), name)
            self.assert_close(self.lidar["lidar_extrinsic_quaternion_lidar_flu_to_body_frd"], numbers(quaternion), name)

    def test_the_launch_hands_the_nodes_the_mounts(self) -> None:
        profile = load_module("sensor_profile", REPOSITORY / "drone_city_nav/launch/sensor_profile.py")
        _, _, depth = profile.stereo_tof_topics("world", "model")
        for name, value in self.camera.items():
            self.assert_close(value, depth[name], name)
        self.assert_close(sensor_extrinsics.vision_memory_parameters(self.extrinsics)["lidar_extrinsic_translation_body_frd_m"],
                          profile.VISION_MEMORY_OVERRIDES["lidar_extrinsic_translation_body_frd_m"], "the vision memory")
        self.assertEqual(self.lidar, profile.LIDAR_PARAMETERS)
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text()
        self.assertIn("            **LIDAR_PARAMETERS,", launch)
        self.assertIn("obstacle_memory_parameters = [params_file.perform(context), LIDAR_PARAMETERS]", launch)
        multi = (REPOSITORY / "drone_city_nav/launch/multi_vehicle.launch.py").read_text()
        self.assertIn('memory_params.update(_SENSOR_PROFILE_SUPPORT["LIDAR_PARAMETERS"])', multi)
        configuration = (REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text()
        for name in ("lidar_extrinsic_translation_body_frd_m", "lidar_extrinsic_quaternion_lidar_flu_to_body_frd",
                     "right_from_left_translation_m", "right_from_left_rotation", "left_camera_position_body_frd_m"):
            self.assertNotIn(name, configuration, "the mounts' one source is config/sensor_extrinsics.yaml")

    def test_the_runtime_model_is_written_from_the_source(self) -> None:
        moved = yaml.safe_load(SOURCE.read_text())
        moved["lidar"]["position_m"][0] += 0.05
        moved["tof_up"]["position_m"][2] += 0.01
        moved["base_link_height_m"] = 0.30
        with tempfile.TemporaryDirectory() as temp:
            for name in ("x500_lidar_3d", "lidar_3d_v1", "stereo_tof_v1"):
                shutil.copytree(MODELS / name, Path(temp) / name)
            source = Path(temp) / "sensor_extrinsics.yaml"
            source.write_text(yaml.safe_dump(moved))
            generator.configure_model(Path(temp) / "x500_lidar_3d", "x500_lidar_3d", "stereo_tof",
                                      extrinsics_path=source)
            wrapper = ET.parse(Path(temp) / "x500_lidar_3d" / "model.sdf").getroot()
            lidar = next(e for e in wrapper.iter("include") if e.findtext("uri") == "model://lidar_3d_v1")
            self.assert_close([0.17, 0.0, 0.56], pose(lidar)[:3], "the lidar's include")
            stereo = next(e for e in wrapper.iter("include") if e.findtext("uri") == "model://stereo_tof_v1")
            self.assert_close([0.0, 0.0, 0.30], pose(stereo)[:3], "the stereo block's include")
            block = ET.parse(Path(temp) / "stereo_tof_v1" / "model.sdf").getroot().find(
                "model/link[@name='stereo_tof_link']")
            up = block.find("sensor[@name='tof_up']")
            self.assert_close([-0.42, 0.0, 0.11], pose(up)[:3], "tof_up")
            # Every sensor of the block but the thermal camera takes a mount.
            self.assertEqual(set(generator.STEREO_SENSOR_MOUNTS) | {"thermal"},
                             {sensor.get("name") for sensor in block.findall("sensor")})
        moved["imu"]["position_m"][0] = 0.01
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "sensor_extrinsics.yaml"
            source.write_text(yaml.safe_dump(moved))
            with self.assertRaises(ValueError):
                generator.load_extrinsics(source)

    def test_the_offline_tools_hold_the_mounts(self) -> None:
        bag = (REPOSITORY / "tools/lio/extract_bag.py").read_text()
        values = numbers(re.search(r"t_body = np\.array\(\[([^\]]*)\]", bag).group(1))
        self.assert_close(self.lidar["lidar_extrinsic_translation_body_frd_m"], values, "tools/lio/extract_bag.py")
        error = (REPOSITORY / "tools/estimate_error.py").read_text()
        self.assertAlmostEqual(self.extrinsics["base_link_height_m"],
                               float(re.search(r"REST_OFFSET_M = ([0-9.]+)", error).group(1)), delta=TOLERANCE)


if __name__ == "__main__":
    unittest.main()
