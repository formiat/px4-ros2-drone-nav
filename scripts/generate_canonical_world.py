#!/usr/bin/env python3
"""Generate Gazebo SDF and sparse Occupancy3D from one canonical world spec."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from world_compiler_portal_graph import DerivedPortalGraph, derive_portal_graph


OCCUPANCY_MAGIC = b"DCNOCC3D"
OCCUPANCY_VERSION = 5
FREE_SPACE_TOPOLOGY_MAGIC = b"DCNFTOP3"
FREE_SPACE_TOPOLOGY_VERSION = 1
NO_STATIC_SOLID_VISIBILITY = 0x08000000
NO_STATIC_OCCLUDER_VISIBILITY = 0x04000000
BUILDING_GRID_CENTER_M = 27.0
BUILDING_GRID_SPACING_M = 54.0
BUILDING_PALETTE_RGB = (
    (148, 158, 164),
    (126, 151, 168),
    (140, 157, 145),
    (164, 151, 134),
    (154, 142, 159),
    (132, 158, 158),
    (169, 148, 148),
    (146, 148, 170),
)


@dataclass(frozen=True)
class Box:
    id: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float, float] = (0.48, 0.50, 0.53, 1.0)
    visibility_flags: int | None = None


def building_color(x: float, y: float) -> tuple[float, float, float, float]:
    """Return the deterministic color used by the RViz static-map palette."""
    building_x = round((x - BUILDING_GRID_CENTER_M) / BUILDING_GRID_SPACING_M)
    building_y = round((y - BUILDING_GRID_CENTER_M) / BUILDING_GRID_SPACING_M)
    red, green, blue = BUILDING_PALETTE_RGB[
        (building_x + 3 * building_y) % len(BUILDING_PALETTE_RGB)
    ]
    return red / 255.0, green / 255.0, blue / 255.0, 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--sdf", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--topology", type=Path, required=True)
    return parser.parse_args()


def load_spec(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        spec = json.load(stream)
    if spec.get("schema") != "drone_city_nav_canonical_world_v1":
        raise ValueError("unsupported canonical world schema")
    return spec


def passage_structure_boxes(passage_structure: dict) -> list[Box]:
    height = float(passage_structure["height_m"])
    z_reference = float(passage_structure["opening_center_z_m"])
    kind = passage_structure["kind"]
    boxes: list[Box] = []

    def append_box(suffix: str, cx: float, cy: float, z_min: float, z_max: float,
                   sx: float, sy: float) -> None:
        if sx <= 1.0e-6 or sy <= 1.0e-6 or z_max <= z_min + 1.0e-6:
            return
        boxes.append(Box(
            id=f"{passage_structure['id']}_{suffix}",
            center=(cx, cy, 0.5 * (z_min + z_max)),
            size=(sx, sy, z_max - z_min),
            color=(0.43, 0.47, 0.55, 1.0),
            visibility_flags=NO_STATIC_SOLID_VISIBILITY,
        ))

    if kind == "straight":
        center_x, center_y = map(float, passage_structure["structure_center_m"])
        size_x, size_y, size_z = map(float, passage_structure["structure_size_m"])
        opening_min = z_reference - 0.5 * height
        opening_max = z_reference + 0.5 * height
        append_box("lower", center_x, center_y, 0.0, opening_min, size_x, size_y)
        append_box("upper", center_x, center_y, opening_max, size_z, size_x, size_y)
    elif kind == "intersection":
        center_x, center_y = map(float, passage_structure["intersection_center_m"])
        size_x, size_y, size_z = map(float, passage_structure["intersection_size_m"])
        opening_min = z_reference - 0.5 * height
        opening_max = z_reference + 0.5 * height
        append_box("intersection_lower", center_x, center_y, 0.0, opening_min,
                   size_x, size_y)
        append_box("intersection_upper", center_x, center_y, opening_max, size_z,
                   size_x, size_y)
        for bridge in passage_structure["bridges"]:
            bridge_x, bridge_y = map(float, bridge["center_m"])
            bridge_size_x, bridge_size_y, bridge_size_z = map(
                float, bridge["size_m"])
            bridge_id = bridge["id"]
            append_box(f"{bridge_id}_lower", bridge_x, bridge_y, 0.0,
                       opening_min, bridge_size_x, bridge_size_y)
            append_box(f"{bridge_id}_upper", bridge_x, bridge_y, opening_max,
                       bridge_size_z, bridge_size_x, bridge_size_y)
            if bridge["blocked"]:
                append_box(f"{bridge_id}_middle", bridge_x, bridge_y,
                           opening_min, opening_max, bridge_size_x, bridge_size_y)
    else:
        raise ValueError(f"unsupported passage structure kind: {kind}")
    return boxes


def passage_structure_occluder_boxes(passage_structure: dict) -> list[Box]:
    height = float(passage_structure["height_m"])
    z_reference = float(passage_structure["opening_center_z_m"])
    z_min = z_reference - 0.5 * height
    z_max = z_reference + 0.5 * height
    if passage_structure["kind"] != "intersection":
        center_x, center_y = map(float, passage_structure["structure_center_m"])
        size_x, size_y, _ = map(float, passage_structure["structure_size_m"])
        return [Box(f"{passage_structure['id']}_no_static_occluder",
                    (center_x, center_y, 0.5 * (z_min + z_max)),
                    (size_x, size_y, z_max - z_min))]

    boxes = []
    center_x, center_y = map(float, passage_structure["intersection_center_m"])
    size_x, size_y, _ = map(float, passage_structure["intersection_size_m"])
    boxes.append(Box(f"{passage_structure['id']}_intersection_no_static_occluder",
                     (center_x, center_y, 0.5 * (z_min + z_max)),
                     (size_x, size_y, z_max - z_min)))
    for bridge in passage_structure["bridges"]:
        if bridge["blocked"]:
            continue
        bridge_x, bridge_y = map(float, bridge["center_m"])
        bridge_size_x, bridge_size_y, _ = map(float, bridge["size_m"])
        boxes.append(Box(f"{passage_structure['id']}_{bridge['id']}_no_static_occluder",
                         (bridge_x, bridge_y, 0.5 * (z_min + z_max)),
                         (bridge_size_x, bridge_size_y, z_max - z_min)))
    return boxes


def physical_boxes(spec: dict) -> list[Box]:
    boxes: list[Box] = []
    grid = spec["building_grid"]
    size_x, size_y, size_z = map(float, grid["size_m"])
    index = 1
    for x in map(float, grid["x_centers_m"]):
        for y in map(float, grid["y_centers_m"]):
            boxes.append(Box(f"building_{index:03d}", (x, y, 0.5 * size_z),
                             (size_x, size_y, size_z), building_color(x, y)))
            index += 1
    for passage_structure in spec["passage_structures"]:
        boxes.extend(passage_structure_boxes(passage_structure))

    unique: list[Box] = []
    seen_geometry: set[
        tuple[tuple[float, float, float], tuple[float, float, float], int | None]
    ] = set()
    for box in boxes:
        geometry = (box.center, box.size, box.visibility_flags)
        if geometry in seen_geometry:
            continue
        seen_geometry.add(geometry)
        unique.append(box)
    return unique


def sdf_pose(spec: dict, center: tuple[float, float, float]) -> tuple[float, float, float]:
    transform = spec["map_to_sdf"]
    return (center[1] + float(transform["sdf_x_offset_m"]),
            center[0] + float(transform["sdf_y_offset_m"]), center[2])


def add_text(parent: ET.Element, name: str, text: str, **attributes: str) -> ET.Element:
    element = ET.SubElement(parent, name, attributes)
    element.text = text
    return element


def add_box_model(world: ET.Element, spec: dict, box: Box) -> None:
    model = ET.SubElement(world, "model", {"name": box.id})
    add_text(model, "static", "true")
    px, py, pz = sdf_pose(spec, box.center)
    add_text(model, "pose", f"{px:.3f} {py:.3f} {pz:.3f} 0 0 0")
    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    size = ET.SubElement(ET.SubElement(geometry, "box"), "size")
    sdf_size = (box.size[1], box.size[0], box.size[2])
    size.text = " ".join(f"{value:.3f}" for value in sdf_size)
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    if box.visibility_flags is not None:
        add_text(visual, "visibility_flags", str(box.visibility_flags))
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_size = ET.SubElement(ET.SubElement(visual_geometry, "box"), "size")
    visual_size.text = size.text
    material = ET.SubElement(visual, "material")
    add_text(material, "diffuse", " ".join(str(value) for value in box.color))


def add_visual_box(world: ET.Element, spec: dict, name: str,
                   center: tuple[float, float, float], size: tuple[float, float, float],
                   color: tuple[float, float, float, float]) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    add_text(model, "static", "true")
    px, py, pz = sdf_pose(spec, center)
    add_text(model, "pose", f"{px:.3f} {py:.3f} {pz:.3f} 0 0 0")
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    add_text(box, "size", f"{size[1]:.3f} {size[0]:.3f} {size[2]:.3f}")
    material = ET.SubElement(visual, "material")
    add_text(material, "diffuse", " ".join(str(value) for value in color))


def generate_sdf(spec: dict, boxes: Iterable[Box], output: Path) -> None:
    root = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(root, "world", {"name": "generated_city"})
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
        ("gz-sim-air-speed-system", "gz::sim::systems::AirSpeed"),
        ("gz-sim-apply-link-wrench-system", "gz::sim::systems::ApplyLinkWrench"),
        ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
        ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensors = ET.SubElement(world, "plugin", {
        "filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors"})
    add_text(sensors, "render_engine", "ogre2")
    physics = ET.SubElement(world, "physics", {"type": "ode"})
    add_text(physics, "max_step_size", "0.004")
    add_text(physics, "real_time_factor", "1.0")
    add_text(physics, "real_time_update_rate", "250")
    add_text(world, "gravity", "0 0 -9.8")
    add_text(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
    ET.SubElement(world, "atmosphere", {"type": "adiabatic"})
    spherical = ET.SubElement(world, "spherical_coordinates")
    add_text(spherical, "surface_model", "EARTH_WGS84")
    add_text(spherical, "world_frame_orientation", "ENU")
    add_text(spherical, "latitude_deg", "47.397971057728974")
    add_text(spherical, "longitude_deg", "8.546163739800146")
    add_text(spherical, "elevation", "0")
    scene = ET.SubElement(world, "scene")
    add_text(scene, "ambient", "0.55 0.58 0.62 1")
    add_text(scene, "background", "0.72 0.80 0.92 1")
    add_text(scene, "shadows", "true")
    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    add_text(light, "cast_shadows", "true")
    add_text(light, "pose", "0 0 80 0 0 0")
    add_text(light, "diffuse", "0.9 0.86 0.78 1")
    add_text(light, "specular", "0.25 0.25 0.25 1")
    add_text(light, "direction", "-0.5 0.2 -1")

    ground = spec["ground"]
    add_box_model(world, spec, Box("ground", tuple(map(float, ground["center_m"])),
                                   tuple(map(float, ground["size_m"])),
                                   (0.30, 0.34, 0.34, 1.0)))
    for index, map_x in enumerate((0.0, 54.0, 108.0, 162.0, 216.0, 270.0), 1):
        add_visual_box(world, spec, f"street_{index:02d}", (map_x, 225.0, 0.04),
                       (18.0, 510.0, 0.04), (0.08, 0.08, 0.08, 1.0))
    for index, map_y in enumerate((0.0, 54.0, 108.0, 162.0, 216.0,
                                    270.0, 324.0, 378.0, 432.0, 450.0), 1):
        add_visual_box(world, spec, f"avenue_{index:02d}", (135.0, map_y, 0.05),
                       (330.0, 18.0, 0.04), (0.10, 0.10, 0.10, 1.0))
    for box in boxes:
        add_box_model(world, spec, box)

    for passage_structure in spec["passage_structures"]:
        for occluder in passage_structure_occluder_boxes(passage_structure):
            model = ET.SubElement(world, "model", {"name": occluder.id})
            add_text(model, "static", "true")
            px, py, pz = sdf_pose(spec, occluder.center)
            add_text(model, "pose", f"{px:.3f} {py:.3f} {pz:.3f} 0 0 0")
            link = ET.SubElement(model, "link", {"name": "no_static_lidar_occluder"})
            visual = ET.SubElement(link, "visual", {"name": "visual"})
            add_text(visual, "cast_shadows", "false")
            add_text(visual, "transparency", "0.999")
            add_text(visual, "visibility_flags", str(NO_STATIC_OCCLUDER_VISIBILITY))
            geometry = ET.SubElement(visual, "geometry")
            add_text(ET.SubElement(geometry, "box"), "size",
                     f"{occluder.size[1]:.3f} {occluder.size[0]:.3f} "
                     f"{occluder.size[2]:.3f}")

    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def generate_occupancy(spec: dict, boxes: Iterable[Box], output: Path,
                       topology_output: Path) -> None:
    occupancy = spec["occupancy"]
    origin = tuple(map(float, occupancy["origin_m"]))
    size = tuple(map(float, occupancy["size_m"]))
    resolution = float(occupancy["resolution_m"])
    chunk_size = int(occupancy["chunk_size"])
    dimensions = tuple(int(math.ceil(axis / resolution)) for axis in size)
    chunks: dict[tuple[int, int, int], int] = {}
    column_masks: dict[tuple[int, int], int] = {}
    physical = list(boxes)
    for box in physical:
        minimum = tuple(box.center[axis] - 0.5 * box.size[axis]
                        for axis in range(3))
        maximum = tuple(box.center[axis] + 0.5 * box.size[axis]
                        for axis in range(3))
        starts = tuple(max(0, int(math.ceil((minimum[axis] - origin[axis]) / resolution - 0.5)))
                       for axis in range(3))
        ends = tuple(min(dimensions[axis],
                         int(math.ceil((maximum[axis] - origin[axis]) / resolution - 0.5)))
                     for axis in range(3))
        z_mask = ((1 << (ends[2] - starts[2])) - 1) << starts[2]
        for y in range(starts[1], ends[1]):
            for x in range(starts[0], ends[0]):
                column_masks[(x, y)] = column_masks.get((x, y), 0) | z_mask
        for z in range(starts[2], ends[2]):
            for y in range(starts[1], ends[1]):
                for x in range(starts[0], ends[0]):
                    chunk = (x // chunk_size, y // chunk_size, z // chunk_size)
                    local = ((z % chunk_size) * chunk_size + y % chunk_size) * chunk_size + x % chunk_size
                    chunks[chunk] = chunks.get(chunk, 0) | (1 << local)

    portal_config = occupancy["portal_graph"]
    validation_capsule = portal_config["validation_capsule"]
    portal_graph = derive_portal_graph(
        column_masks, dimensions, origin, resolution,
        minimum_opening_area_m2=float(portal_config["minimum_opening_area_m2"]),
        speed_limit_mps=float(portal_config["speed_limit_mps"]),
        validation_radius_m=float(validation_capsule["radius_m"]),
        validation_lower_extent_m=float(validation_capsule["lower_extent_m"]),
        validation_upper_extent_m=float(validation_capsule["upper_extent_m"]),
        validation_sweep_step_m=float(validation_capsule["sweep_step_m"]),
    )

    canonical_bytes = json.dumps(spec, sort_keys=True, separators=(",", ":")).encode()
    fingerprint = int.from_bytes(hashlib.sha256(canonical_bytes).digest()[:8], "little")
    words_per_chunk = (chunk_size ** 3 + 63) // 64
    header = struct.pack(
        "<8sII4f3IQI",
        OCCUPANCY_MAGIC,
        OCCUPANCY_VERSION,
        chunk_size,
        resolution,
        origin[0], origin[1], origin[2],
        dimensions[0], dimensions[1], dimensions[2],
        fingerprint,
        len(chunks),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(header)
        for chunk, bits in sorted(chunks.items()):
            stream.write(struct.pack("<3i", *chunk))
            stream.write(b"".join(struct.pack("<Q", (bits >> (64 * word)) & ((1 << 64) - 1))
                                  for word in range(words_per_chunk)))
    write_free_space_topology(
        topology_output, fingerprint, origin, resolution, dimensions, portal_graph
    )


def write_string(stream, value: str) -> None:
    encoded = value.encode("utf-8")
    if not encoded or len(encoded) > 0xFFFF:
        raise ValueError("world artifact string must contain 1..65535 UTF-8 bytes")
    stream.write(struct.pack("<H", len(encoded)))
    stream.write(encoded)


def write_points(stream, points: Iterable[tuple[float, float, float]]) -> None:
    materialized = tuple(points)
    stream.write(struct.pack("<I", len(materialized)))
    for point in materialized:
        stream.write(struct.pack("<3f", *point))


def write_topology_data(stream, graph: DerivedPortalGraph) -> None:
    stream.write(struct.pack("<I", len(graph.regions)))
    for region in graph.regions:
        write_string(stream, region.id)
        stream.write(struct.pack("<I", len(region.portals)))
        for portal in region.portals:
            write_string(stream, portal.id)
            stream.write(struct.pack("<3f", *portal.center))
            stream.write(struct.pack("<3f", *portal.outward_normal))
            write_points(stream, portal.opening_polygon)
    stream.write(struct.pack("<I", len(graph.traversal_edges)))
    for edge in graph.traversal_edges:
        write_string(stream, edge.id)
        write_string(stream, edge.region_id)
        write_string(stream, edge.entry_portal_id)
        write_string(stream, edge.exit_portal_id)
        write_points(stream, edge.centerline)
        stream.write(struct.pack(
            "<6f", edge.min_z_m, edge.max_z_m, edge.width_m, edge.height_m,
            edge.minimum_clearance_m, edge.speed_limit_mps
        ))


def write_free_space_topology(output: Path, occupancy_fingerprint: int,
                              origin: tuple[float, float, float], resolution: float,
                              dimensions: tuple[int, int, int],
                              graph: DerivedPortalGraph) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(struct.pack(
            "<8sIQ4f3I",
            FREE_SPACE_TOPOLOGY_MAGIC,
            FREE_SPACE_TOPOLOGY_VERSION,
            occupancy_fingerprint,
            resolution,
            origin[0], origin[1], origin[2],
            dimensions[0], dimensions[1], dimensions[2],
        ))
        write_topology_data(stream, graph)


def main() -> None:
    args = parse_args()
    spec = load_spec(args.spec)
    boxes = physical_boxes(spec)
    generate_sdf(spec, boxes, args.sdf)
    generate_occupancy(spec, boxes, args.occupancy, args.topology)


if __name__ == "__main__":
    main()
