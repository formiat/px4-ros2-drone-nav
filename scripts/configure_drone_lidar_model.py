#!/usr/bin/env python3
"""Materialize one profile-specific X500 lidar wrapper in a runtime directory."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


PROFILE_SENSOR_MODELS = {
    "2d": "lidar_2d_v2",
    "3d": "lidar_3d_v1",
}


def configure_model(
    model_directory: Path, model_name: str, lidar_profile: str
) -> str:
    sensor_model = PROFILE_SENSOR_MODELS.get(lidar_profile)
    if sensor_model is None:
        raise ValueError(f"unsupported materialized lidar profile: {lidar_profile}")

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
        if (include.findtext("uri") or "").removeprefix("model://")
        in PROFILE_SENSOR_MODELS.values()
    ]
    if len(sensor_includes) != 1:
        raise RuntimeError(
            f"expected one navigation lidar include in {model_sdf}, "
            f"found {len(sensor_includes)}"
        )
    sensor_includes[0].find("uri").text = f"model://{sensor_model}"

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
    parser.add_argument("--lidar-profile", choices=("2d", "3d"), required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    sensor_model = configure_model(
        args.model_directory, args.model_name, args.lidar_profile
    )
    print(
        "Drone lidar model configured: "
        f"model={args.model_name} profile={args.lidar_profile} sensor={sensor_model}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
