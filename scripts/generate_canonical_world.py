#!/usr/bin/env python3
"""Generate Gazebo SDF and sparse Occupancy3D from one canonical world spec."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


OCCUPANCY_MAGIC = b"DCNOCC3D"
OCCUPANCY_VERSION = 2
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


@dataclass(frozen=True)
class ChannelEdge:
    id: str
    centerline: tuple[tuple[float, float, float], ...]
    width_m: float
    height_m: float
    speed_limit_mps: float


def building_color(x: float, y: float) -> tuple[float, float, float, float]:
    """Return the deterministic color used by the RViz static-map palette."""
    building_x = round((x - BUILDING_GRID_CENTER_M) / BUILDING_GRID_SPACING_M)
    building_y = round((y - BUILDING_GRID_CENTER_M) / BUILDING_GRID_SPACING_M)
    red, green, blue = BUILDING_PALETTE_RGB[
        (building_x + 3 * building_y) % len(BUILDING_PALETTE_RGB)
    ]
    return red / 255.0, green / 255.0, blue / 255.0, 1.0


def external_portal_point(channel: dict,
                          point: tuple[float, float, float]) -> tuple[float, float, float]:
    intersection_x, intersection_y = map(float, channel["intersection_center_m"])
    for bridge in channel["bridges"]:
        bridge_x, bridge_y = map(float, bridge["center_m"])
        if bridge.get("blocked", False) or abs(point[0] - bridge_x) > 1.0e-6 or \
                abs(point[1] - bridge_y) > 1.0e-6:
            continue
        size_x, size_y, _ = map(float, bridge["size_m"])
        delta_x = bridge_x - intersection_x
        delta_y = bridge_y - intersection_y
        if abs(delta_x) > abs(delta_y):
            return (bridge_x + math.copysign(0.5 * size_x, delta_x), bridge_y,
                    point[2])
        if abs(delta_y) > 1.0e-6:
            return (bridge_x, bridge_y + math.copysign(0.5 * size_y, delta_y),
                    point[2])
    raise ValueError(
        f"channel {channel['id']} centerline endpoint {point} must match an open bridge center"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--sdf", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    return parser.parse_args()


def load_spec(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        spec = json.load(stream)
    if spec.get("schema") != "drone_city_nav_canonical_world_v1":
        raise ValueError("unsupported canonical world schema")
    return spec


def channel_edges(channel: dict) -> list[ChannelEdge]:
    edge_specs = channel.get("edges")
    if edge_specs is None:
        edge_specs = [{"id": None, "centerline_m": channel["centerline_m"]}]
    if not edge_specs:
        raise ValueError(f"channel {channel['id']} needs at least one edge")

    width = float(channel["width_m"])
    height = float(channel["height_m"])
    default_speed_limit = float(channel.get("speed_limit_mps", 10.0))
    edges: list[ChannelEdge] = []
    for edge_spec in edge_specs:
        suffix = edge_spec.get("id")
        edge_id = channel["id"] if suffix is None else f"{channel['id']}:{suffix}"
        centerline = tuple(
            tuple(map(float, point)) for point in edge_spec["centerline_m"]
        )
        speed_limit = float(edge_spec.get("speed_limit_mps", default_speed_limit))
        if not edge_id or len(centerline) < 2:
            raise ValueError(f"channel edge {edge_id} needs at least two points")
        if width <= 0.0 or height <= 0.0 or speed_limit <= 0.0:
            raise ValueError(f"channel edge {edge_id} has invalid constraints")
        entry = external_portal_point(channel, centerline[0])
        exit = external_portal_point(channel, centerline[-1])
        generated_centerline = (entry, *centerline, exit)
        edges.append(ChannelEdge(edge_id, generated_centerline, width, height,
                                 speed_limit))
    if len({edge.id for edge in edges}) != len(edges):
        raise ValueError(f"channel {channel['id']} has duplicate edge ids")
    return edges


def channel_points(channel: dict) -> list[tuple[float, float, float]]:
    points = [point for edge in channel_edges(channel) for point in edge.centerline]
    reference_z = points[0][2]
    if any(abs(point[2] - reference_z) > 1.0e-6 for point in points):
        raise ValueError(f"channel {channel['id']} centerlines must have constant Z")
    return points


def channel_boxes(channel: dict) -> list[Box]:
    height = float(channel["height_m"])
    points = channel_points(channel)
    kind = channel["kind"]
    boxes: list[Box] = []

    def append_box(suffix: str, cx: float, cy: float, z_min: float, z_max: float,
                   sx: float, sy: float) -> None:
        if sx <= 1.0e-6 or sy <= 1.0e-6 or z_max <= z_min + 1.0e-6:
            return
        boxes.append(Box(
            id=f"{channel['id']}_{suffix}",
            center=(cx, cy, 0.5 * (z_min + z_max)),
            size=(sx, sy, z_max - z_min),
            color=(0.43, 0.47, 0.55, 1.0),
            visibility_flags=NO_STATIC_SOLID_VISIBILITY,
        ))

    if kind == "straight":
        center_x, center_y = map(float, channel["structure_center_m"])
        size_x, size_y, size_z = map(float, channel["structure_size_m"])
        z_reference = points[len(points) // 2][2]
        opening_min = z_reference - 0.5 * height
        opening_max = z_reference + 0.5 * height
        append_box("lower", center_x, center_y, 0.0, opening_min, size_x, size_y)
        append_box("upper", center_x, center_y, opening_max, size_z, size_x, size_y)
    elif kind == "intersection":
        center_x, center_y = map(float, channel["intersection_center_m"])
        size_x, size_y, size_z = map(float, channel["intersection_size_m"])
        z_reference = points[1][2]
        opening_min = z_reference - 0.5 * height
        opening_max = z_reference + 0.5 * height
        append_box("intersection_lower", center_x, center_y, 0.0, opening_min,
                   size_x, size_y)
        append_box("intersection_upper", center_x, center_y, opening_max, size_z,
                   size_x, size_y)
        for bridge in channel["bridges"]:
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
        raise ValueError(f"unsupported channel kind: {kind}")
    return boxes


def channel_occluder_boxes(channel: dict) -> list[Box]:
    points = channel_points(channel)
    height = float(channel["height_m"])
    z_min = min(point[2] for point in points) - 0.5 * height
    z_max = max(point[2] for point in points) + 0.5 * height
    if channel["kind"] != "intersection":
        center_x, center_y = map(float, channel["structure_center_m"])
        size_x, size_y, _ = map(float, channel["structure_size_m"])
        return [Box(f"{channel['id']}_no_static_occluder",
                    (center_x, center_y, 0.5 * (z_min + z_max)),
                    (size_x, size_y, z_max - z_min))]

    boxes = []
    center_x, center_y = map(float, channel["intersection_center_m"])
    size_x, size_y, _ = map(float, channel["intersection_size_m"])
    boxes.append(Box(f"{channel['id']}_intersection_no_static_occluder",
                     (center_x, center_y, 0.5 * (z_min + z_max)),
                     (size_x, size_y, z_max - z_min)))
    for bridge in channel["bridges"]:
        if bridge["blocked"]:
            continue
        bridge_x, bridge_y = map(float, bridge["center_m"])
        bridge_size_x, bridge_size_y, _ = map(float, bridge["size_m"])
        boxes.append(Box(f"{channel['id']}_{bridge['id']}_no_static_occluder",
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
    for channel in spec["channels"]:
        boxes.extend(channel_boxes(channel))

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

    for channel in spec["channels"]:
        for occluder in channel_occluder_boxes(channel):
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

    mission = spec["mission"]
    for name, point, color in (
        ("start_marker", mission["start_m"], (0.0, 0.55, 0.25, 1.0)),
        ("goal_marker", mission["goal_m"], (0.8, 0.1, 0.1, 1.0)),
    ):
        model = ET.SubElement(world, "model", {"name": name})
        add_text(model, "static", "true")
        px, py, _ = sdf_pose(spec, tuple(map(float, point)))
        add_text(model, "pose", f"{px:.3f} {py:.3f} 0.05 0 0 0")
        link = ET.SubElement(model, "link", {"name": "link"})
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        geometry = ET.SubElement(visual, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        add_text(cylinder, "radius", "4.8")
        add_text(cylinder, "length", "0.08")
        material = ET.SubElement(visual, "material")
        add_text(material, "diffuse", " ".join(str(value) for value in color))

    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def generate_occupancy(spec: dict, boxes: Iterable[Box], output: Path) -> None:
    occupancy = spec["occupancy"]
    origin = tuple(map(float, occupancy["origin_m"]))
    size = tuple(map(float, occupancy["size_m"]))
    resolution = float(occupancy["resolution_m"])
    chunk_size = int(occupancy["chunk_size"])
    dimensions = tuple(int(math.ceil(axis / resolution)) for axis in size)
    chunks: dict[tuple[int, int, int], int] = {}
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
        for z in range(starts[2], ends[2]):
            for y in range(starts[1], ends[1]):
                for x in range(starts[0], ends[0]):
                    chunk = (x // chunk_size, y // chunk_size, z // chunk_size)
                    local = ((z % chunk_size) * chunk_size + y % chunk_size) * chunk_size + x % chunk_size
                    chunks[chunk] = chunks.get(chunk, 0) | (1 << local)

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
        generated_edges = [
            edge for channel in spec["channels"] for edge in channel_edges(channel)
        ]
        stream.write(struct.pack("<I", len(generated_edges)))
        for edge in generated_edges:
            channel_id = edge.id.encode("utf-8")
            if not channel_id or len(channel_id) > 0xFFFF:
                raise ValueError("channel id must contain 1..65535 UTF-8 bytes")
            centerline = edge.centerline
            reference_z = centerline[len(centerline) // 2][2]
            min_z = reference_z - 0.5 * edge.height_m
            max_z = reference_z + 0.5 * edge.height_m
            minimum_clearance = 0.5 * min(edge.width_m, edge.height_m)
            stream.write(struct.pack("<H", len(channel_id)))
            stream.write(channel_id)
            stream.write(struct.pack("<I", len(centerline)))
            for point in centerline:
                stream.write(struct.pack("<3f", *point))
            stream.write(struct.pack("<4f", min_z, max_z,
                                     minimum_clearance, edge.speed_limit_mps))


def main() -> None:
    args = parse_args()
    spec = load_spec(args.spec)
    boxes = physical_boxes(spec)
    generate_sdf(spec, boxes, args.sdf)
    generate_occupancy(spec, boxes, args.occupancy)


if __name__ == "__main__":
    main()
