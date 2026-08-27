#!/usr/bin/env python3
"""Configure the runtime GPU lidar visibility mask for one navigation mode."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from gazebo_visibility import (
    NO_STATIC_3D_VISIBILITY_MASK,
    STATIC_VISIBILITY_MASK,
)


def visibility_mask(mode: str) -> int:
    if mode == "static":
        return STATIC_VISIBILITY_MASK
    if mode == "no-static-3d":
        return NO_STATIC_3D_VISIBILITY_MASK
    raise ValueError(f"unsupported lidar visibility mode: {mode}")


def configure_model(model_sdf: Path, mode: str, enabled: bool = True) -> int:
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    sensors = [
        sensor
        for sensor in root.iter("sensor")
        if sensor.attrib.get("type") == "gpu_lidar"
    ]
    if len(sensors) != 1:
        raise RuntimeError(
            f"expected one gpu_lidar in {model_sdf}, found {len(sensors)}"
        )
    mask_element = sensors[0].find("ray/visibility_mask")
    if mask_element is None:
        raise RuntimeError(f"lidar visibility_mask is missing in {model_sdf}")

    mask = visibility_mask(mode)
    mask_element.text = str(mask)
    always_on = sensors[0].find("always_on")
    if always_on is None:
        always_on = ET.SubElement(sensors[0], "always_on")
    always_on.text = "true" if enabled else "false"
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    return mask


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_sdf", type=Path)
    parser.add_argument(
        "--mode",
        choices=("static", "no-static-3d"),
        required=True,
    )
    parser.add_argument(
        "--enabled",
        choices=("true", "false"),
        default="true",
        help="Enable or disable the runtime GPU lidar sensor.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    enabled = args.enabled == "true"
    mask = configure_model(args.model_sdf, args.mode, enabled)
    print(
        f"Lidar runtime configured: mode={args.mode} enabled={str(enabled).lower()} "
        f"mask={mask}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
