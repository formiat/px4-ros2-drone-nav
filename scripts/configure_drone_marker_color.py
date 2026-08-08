#!/usr/bin/env python3
"""Create a role-specific Gazebo marker variant from the canonical drone model."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


SOURCE_MARKER_PREFIX = "yellow_"
TARGET_MARKER_PREFIX = "red_"
RED_SURFACE_RGB = (1.0, 0.05, 0.02)


def _parse_rgba(element: ET.Element, source: Path) -> list[float]:
    values = [float(value) for value in (element.text or "").split()]
    if len(values) != 4:
        raise RuntimeError(
            f"expected RGBA value in {source} element <{element.tag}>"
        )
    return values


def _format_rgba(rgb: tuple[float, float, float], alpha: float) -> str:
    return f"{rgb[0]:.2f} {rgb[1]:.2f} {rgb[2]:.2f} {alpha:g}"


def _recolor_material(material: ET.Element, source: Path) -> None:
    for tag in ("ambient", "diffuse"):
        element = material.find(tag)
        if element is None:
            raise RuntimeError(f"marker material has no <{tag}> in {source}")
        alpha = _parse_rgba(element, source)[3]
        element.text = _format_rgba(RED_SURFACE_RGB, alpha)

    emissive = material.find("emissive")
    if emissive is None:
        raise RuntimeError(f"marker material has no <emissive> in {source}")
    old_emissive = _parse_rgba(emissive, source)
    red_intensity = max(old_emissive[:3])
    emissive.text = _format_rgba((red_intensity, 0.0, 0.0), old_emissive[3])


def configure_model(model_directory: Path, model_name: str) -> int:
    model_sdf = model_directory / "model.sdf"
    model_config = model_directory / "model.config"

    sdf_tree = ET.parse(model_sdf)
    sdf_root = sdf_tree.getroot()
    model = sdf_root.find("model")
    if model is None:
        raise RuntimeError(f"model element is missing in {model_sdf}")
    model.attrib["name"] = model_name

    marker_link = model.find("./link[@name='visibility_marker_link']")
    if marker_link is None:
        raise RuntimeError(f"visibility_marker_link is missing in {model_sdf}")

    configured_visuals = 0
    for visual in marker_link.findall("visual"):
        visual_name = visual.attrib.get("name", "")
        if not visual_name.startswith(SOURCE_MARKER_PREFIX):
            continue
        material = visual.find("material")
        if material is None:
            raise RuntimeError(f"marker visual {visual_name} has no material")
        visual.attrib["name"] = (
            TARGET_MARKER_PREFIX + visual_name.removeprefix(SOURCE_MARKER_PREFIX)
        )
        _recolor_material(material, model_sdf)
        configured_visuals += 1

    if configured_visuals == 0:
        raise RuntimeError(f"no {SOURCE_MARKER_PREFIX} marker visuals in {model_sdf}")

    config_tree = ET.parse(model_config)
    config_name = config_tree.getroot().find("name")
    if config_name is None:
        raise RuntimeError(f"model name is missing in {model_config}")
    config_name.text = model_name

    ET.indent(sdf_tree, space="  ")
    sdf_tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    ET.indent(config_tree, space="  ")
    config_tree.write(model_config, encoding="utf-8", xml_declaration=True)
    return configured_visuals


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model_directory", type=Path)
    parser.add_argument("--model-name", required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    visual_count = configure_model(args.model_directory, args.model_name)
    print(
        "Drone marker configured: "
        f"model={args.model_name} color=red visuals={visual_count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
