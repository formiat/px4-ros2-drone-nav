#!/usr/bin/env python3
"""Configure the runtime GPU lidar visibility mask for one navigation mode."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


GZ_VISIBILITY_ALL = 0x0FFFFFFF
STATIC_PASSAGE_MASS_VISIBILITY_FLAG = 0x08000000
NO_STATIC_OCCLUDER_VISIBILITY_FLAG = 0x04000000
STATIC_VISIBILITY_MASK = GZ_VISIBILITY_ALL & ~(
    STATIC_PASSAGE_MASS_VISIBILITY_FLAG | NO_STATIC_OCCLUDER_VISIBILITY_FLAG
)


def visibility_mask(mode: str) -> int:
    if mode == "static":
        return STATIC_VISIBILITY_MASK
    if mode == "no-static":
        return GZ_VISIBILITY_ALL
    raise ValueError(f"unsupported lidar visibility mode: {mode}")


def configure_model(model_sdf: Path, mode: str) -> int:
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    sensors = [
        sensor
        for sensor in root.iter("sensor")
        if sensor.attrib.get("name") == "lidar_2d_v2"
        and sensor.attrib.get("type") == "gpu_lidar"
    ]
    if len(sensors) != 1:
        raise RuntimeError(
            f"expected one lidar_2d_v2 gpu_lidar in {model_sdf}, found {len(sensors)}"
        )
    mask_element = sensors[0].find("ray/visibility_mask")
    if mask_element is None:
        raise RuntimeError(f"lidar visibility_mask is missing in {model_sdf}")

    mask = visibility_mask(mode)
    mask_element.text = str(mask)
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    return mask


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_sdf", type=Path)
    parser.add_argument("--mode", choices=("static", "no-static"), required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    mask = configure_model(args.model_sdf, args.mode)
    print(f"Lidar visibility configured: mode={args.mode} mask={mask}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
