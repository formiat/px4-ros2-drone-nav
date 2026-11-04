#!/usr/bin/env python3
"""Materialize one profile-specific X500 lidar wrapper in a runtime directory, its sensors' poses written from the
sensor set's single source (drone_city_nav/config/sensor_extrinsics.yaml, roadmap item 27 block 3)."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

EXTRINSICS_PATH = Path(__file__).resolve().parents[1] / "drone_city_nav" / "config" / "sensor_extrinsics.yaml"
# The stereo block's sensors and the mounts they take (the evaluation depth cameras sit on the pair).
STEREO_SENSOR_MOUNTS = {
    "stereo_left": "stereo_left", "stereo_right": "stereo_right",
    "evaluation_depth_left": "stereo_left", "evaluation_depth_right": "stereo_right",
    **{f"tof_{side}": f"tof_{side}" for side in ("up", "down", "front", "back", "left", "right")},
}


NAVIGATION_LIDAR_MODEL = "lidar_3d_v1"
CAMERA_PROFILE_MODELS = {
    "stereo_tof": ("stereo_tof_v1", "stereo_tof_link"),
}
# The height of the x500's base_link, the body the autopilot localizes, over
# the vehicle model's origin (x500_base's own pose): a sensor set whose poses
# are stated against the body is included this much above the origin
# (`base_link_height_m` of the single source). Without it every sensor sat
# 0.24 m lower than the stack's extrinsics say, and the obstacle memory stood
# a voxel above the world (r979).


def load_extrinsics(path: Path = EXTRINSICS_PATH) -> dict:
    with path.open(encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream)
    for name in ("imu", "barometer", "magnetometer"):
        if any(extrinsics[name]["position_m"]) or any(extrinsics[name]["rpy_rad"]):
            raise ValueError(f"{name}: the body frame is the autopilot's IMU's, at base_link")
    return extrinsics


def pose_text(position, rpy) -> str:
    return " ".join(f"{value + 0.0:.12g}" for value in (*position, *rpy))


def pose_values(element) -> list[float]:
    values = [float(v) for v in ((element.findtext("pose") or "0 0 0 0 0 0").split())]
    return values + [0.0] * (6 - len(values))


def write_stereo_poses(model_sdf: Path, extrinsics: dict) -> int:
    """The stereo block's link and sensors at the single source's mounts; returns the sensors written."""
    tree = ET.parse(model_sdf)
    link = tree.getroot().find("model/link[@name='stereo_tof_link']")
    if link is None:
        raise RuntimeError(f"stereo_tof_link is missing in {model_sdf}")
    block = extrinsics["stereo_tof_link"]
    if any(block["rpy_rad"]):
        raise ValueError("stereo_tof_link: the block is stated unturned, its sensors' turns their own")
    link.find("pose").text = pose_text(block["position_m"], block["rpy_rad"])
    written = 0
    for sensor in link.findall("sensor"):
        mount_name = STEREO_SENSOR_MOUNTS.get(sensor.get("name"))
        if mount_name is None:
            continue
        mount = extrinsics[mount_name]
        relative = [a - b for a, b in zip(mount["position_m"], block["position_m"])]
        pose = sensor.find("pose")
        if pose is None:
            pose = ET.SubElement(sensor, "pose")
        pose.text = pose_text(relative, mount["rpy_rad"])
        written += 1
    ET.indent(tree, space="  ")
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    return written


def lidar_include_pose(lidar_model_sdf: Path, extrinsics: dict) -> str:
    """The lidar model's include on the vehicle: the scan's origin less its offset in the lidar's own model."""
    sensor = ET.parse(lidar_model_sdf).getroot().find("model/link/sensor[@type='gpu_lidar']")
    if sensor is None:
        raise RuntimeError(f"no lidar sensor in {lidar_model_sdf}")
    offset = pose_values(sensor)
    if any(offset[3:]):
        raise ValueError("the lidar's own model holds its scan unturned")
    mount = extrinsics["lidar"]
    position = [p - o for p, o in zip(mount["position_m"], offset[:3])]
    position[2] += extrinsics["base_link_height_m"]
    return pose_text(position, mount["rpy_rad"])


def configure_model(
    model_directory: Path,
    model_name: str,
    camera_profile: str = "none",
    lidar_mounted: bool = True,
    extrinsics_path: Path = EXTRINSICS_PATH,
) -> str:
    """The wrapper in `model_directory`; its sensor models are its siblings in the runtime directory."""
    extrinsics = load_extrinsics(extrinsics_path)
    sensor_model = NAVIGATION_LIDAR_MODEL
    model_sdf = model_directory / "model.sdf"
    model_config = model_directory / "model.config"
    sdf_tree = ET.parse(model_sdf)
    sdf_root = sdf_tree.getroot()
    model = sdf_root.find("model")
    if model is None:
        raise RuntimeError(f"model element is missing in {model_sdf}")
    model.attrib["name"] = model_name

    sensor_includes = [
        include
        for include in model.findall("include")
        if (include.findtext("uri") or "").removeprefix("model://") == sensor_model
    ]
    if len(sensor_includes) != 1:
        raise RuntimeError(
            f"expected one navigation lidar include in {model_sdf}, "
            f"found {len(sensor_includes)}"
        )
    sensor_includes[0].find("uri").text = f"model://{sensor_model}"
    sensor_includes[0].find("pose").text = lidar_include_pose(
        model_directory.parent / sensor_model / "model.sdf", extrinsics)
    if not lidar_mounted:
        # The camera profile navigates without the lidar: the sensor leaves
        # the vehicle model, not merely the control path.
        model.remove(sensor_includes[0])
        for joint in model.findall("joint"):
            if joint.attrib.get("name") == "LidarJoint":
                model.remove(joint)
    if camera_profile != "none":
        camera_model, camera_link = CAMERA_PROFILE_MODELS[camera_profile]
        include = ET.SubElement(model, "include", {"merge": "true"})
        ET.SubElement(include, "uri").text = f"model://{camera_model}"
        ET.SubElement(include, "pose").text = pose_text(
            (0.0, 0.0, extrinsics["base_link_height_m"]), (0.0, 0.0, 0.0))
        write_stereo_poses(model_directory.parent / camera_model / "model.sdf", extrinsics)
        joint = ET.SubElement(
            model, "joint", {"name": "CameraProfileJoint", "type": "fixed"}
        )
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = camera_link

    config_tree = ET.parse(model_config)
    config_name = config_tree.getroot().find("name")
    if config_name is None:
        raise RuntimeError(f"model name is missing in {model_config}")
    config_name.text = model_name

    ET.indent(sdf_tree, space="  ")
    sdf_tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    ET.indent(config_tree, space="  ")
    config_tree.write(model_config, encoding="utf-8", xml_declaration=True)
    return sensor_model


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_directory", type=Path)
    parser.add_argument("--model-name", required=True)
    parser.add_argument(
        "--camera-profile",
        choices=("none", *CAMERA_PROFILE_MODELS),
        default="none",
    )
    parser.add_argument(
        "--without-lidar",
        action="store_true",
        help="Leave the navigation lidar out of the vehicle model.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    sensor_model = configure_model(
        args.model_directory,
        args.model_name,
        args.camera_profile,
        not args.without_lidar,
    )
    print(
        "Drone lidar model configured: "
        f"model={args.model_name} sensor={sensor_model} "
        f"camera_profile={args.camera_profile}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
