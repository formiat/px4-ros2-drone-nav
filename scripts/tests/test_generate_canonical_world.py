from __future__ import annotations

import importlib.util
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PATH = REPO_ROOT / "scripts/generate_canonical_world.py"
SPEC_PATH = REPO_ROOT / "drone_city_nav/worlds/canonical_city.world3d.json"
COMMITTED_SDF = REPO_ROOT / "drone_city_nav/worlds/generated_city.sdf"
COMMITTED_OCCUPANCY = REPO_ROOT / "drone_city_nav/worlds/generated_city.occupancy3d"

SPEC = importlib.util.spec_from_file_location("generate_canonical_world", GENERATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
generator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = generator
SPEC.loader.exec_module(generator)


class CanonicalWorldGeneratorTest(unittest.TestCase):
    def test_generates_matching_sdf_and_sparse_occupancy(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        boxes = generator.physical_boxes(spec)
        with tempfile.TemporaryDirectory() as directory:
            sdf_path = Path(directory) / "city.sdf"
            occupancy_path = Path(directory) / "city.occupancy3d"
            generator.generate_sdf(spec, boxes, sdf_path)
            generator.generate_occupancy(spec, boxes, occupancy_path)

            root = ET.parse(sdf_path).getroot()
            model_names = {
                model.attrib["name"] for model in root.findall("./world/model")
            }
            self.assertNotIn("start_marker", model_names)
            self.assertNotIn("goal_marker", model_names)
            collision_models = {
                model.attrib["name"]
                for model in root.findall("./world/model")
                if model.find("./link/collision") is not None
            }
            self.assertIn("building_001", collision_models)
            self.assertIn("building_040", collision_models)
            self.assertIn("channel_11_19_l_intersection_lower", collision_models)
            self.assertIn("channel_11_19_l_west_middle", collision_models)
            self.assertIn("channel_11_19_l_north_middle", collision_models)
            self.assertNotIn("channel_11_19_l_east_middle", collision_models)
            self.assertNotIn("channel_11_19_l_south_middle", collision_models)
            self.assertIn("channel_108_108_t_south_middle", collision_models)
            self.assertNotIn("channel_108_108_t_west_middle", collision_models)
            self.assertNotIn("channel_108_108_t_east_middle", collision_models)
            self.assertNotIn("channel_108_108_t_north_middle", collision_models)

            building_diffuse = {
                model.attrib["name"]: tuple(
                    map(float, model.findtext("./link/visual/material/diffuse").split())
                )
                for model in root.findall("./world/model")
                if model.attrib["name"].startswith("building_")
            }
            self.assertEqual(generator.building_color(27.0, 27.0),
                             building_diffuse["building_001"])
            self.assertEqual(generator.building_color(81.0, 27.0),
                             building_diffuse["building_009"])
            self.assertNotEqual(
                building_diffuse["building_001"], building_diffuse["building_009"]
            )

            header_format = "<8sII4f3IQI"
            with occupancy_path.open("rb") as stream:
                header = struct.unpack(
                    header_format, stream.read(struct.calcsize(header_format))
                )
            self.assertEqual(b"DCNOCC3D", header[0])
            self.assertEqual(2, header[1])
            self.assertEqual(16, header[2])
            self.assertEqual((690, 1050, 80), header[7:10])
            self.assertGreater(header[11], 0)

            chunk_payload_size = 12 + (16 ** 3 // 64) * 8
            with occupancy_path.open("rb") as stream:
                stream.seek(struct.calcsize(header_format) +
                            header[11] * chunk_payload_size)
                channel_count = struct.unpack("<I", stream.read(4))[0]
                channel_ids = []
                for _ in range(channel_count):
                    id_size = struct.unpack("<H", stream.read(2))[0]
                    channel_ids.append(stream.read(id_size).decode("utf-8"))
                    point_count = struct.unpack("<I", stream.read(4))[0]
                    stream.seek(point_count * struct.calcsize("<3f"), 1)
                    min_z, max_z, clearance, speed_limit = struct.unpack(
                        "<4f", stream.read(struct.calcsize("<4f"))
                    )
                    self.assertAlmostEqual(1.5, min_z)
                    self.assertAlmostEqual(8.5, max_z)
                    self.assertAlmostEqual(3.5, clearance)
                    self.assertAlmostEqual(10.0, speed_limit)
                expected_edge_ids = [
                    edge.id
                    for channel in spec["channels"]
                    for edge in generator.channel_edges(channel)
                ]
                self.assertEqual(expected_edge_ids, channel_ids)
                self.assertEqual(b"", stream.read())

    def test_spec_describes_left_straight_and_t_channels(self) -> None:
        with SPEC_PATH.open(encoding="utf-8") as stream:
            spec = json.load(stream)
        self.assertEqual(5, len(spec["building_grid"]["x_centers_m"]))
        self.assertEqual(8, len(spec["building_grid"]["y_centers_m"]))
        self.assertEqual(4, len(spec["channels"]))
        channels = {channel["id"]: channel for channel in spec["channels"]}
        self.assertTrue(
            all(channel["kind"] == "intersection" for channel in channels.values())
        )
        self.assertEqual(
            {"west", "north"},
            {
                bridge["id"]
                for bridge in channels["channel_11_19_l"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"west", "east"},
            {
                bridge["id"]
                for bridge in channels["channel_54_162_straight"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"east", "south"},
            {
                bridge["id"]
                for bridge in channels["channel_108_216_l"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"south"},
            {
                bridge["id"]
                for bridge in channels["channel_108_108_t"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {
                "channel_108_108_t:west_east",
                "channel_108_108_t:west_north",
                "channel_108_108_t:east_north",
            },
            {edge.id for edge in generator.channel_edges(channels["channel_108_108_t"])},
        )
        straight_edge = generator.channel_edges(
            channels["channel_54_162_straight"]
        )[0]
        self.assertEqual((54.0, 123.0, 5.0), straight_edge.centerline[0])
        self.assertEqual((54.0, 201.0, 5.0), straight_edge.centerline[-1])
        t_edges = {
            edge.id: edge for edge in generator.channel_edges(
                channels["channel_108_108_t"]
            )
        }
        self.assertEqual(
            (69.0, 108.0, 5.0),
            t_edges["channel_108_108_t:west_north"].centerline[0],
        )
        self.assertEqual(
            (108.0, 147.0, 5.0),
            t_edges["channel_108_108_t:west_north"].centerline[-1],
        )

        boxes = generator.physical_boxes(spec)
        self.assertEqual(79, len(boxes))
        self.assertEqual(40, sum(box.id.startswith("building_") for box in boxes))
        geometry = {(box.center, box.size, box.visibility_flags) for box in boxes}
        self.assertEqual(len(boxes), len(geometry))

    def test_building_palette_matches_rviz_coordinate_rule(self) -> None:
        self.assertEqual(
            (148 / 255.0, 158 / 255.0, 164 / 255.0, 1.0),
            generator.building_color(27.0, 27.0),
        )
        self.assertEqual(
            generator.building_color(27.0, 27.0),
            generator.building_color(27.0 + 8 * 54.0, 27.0),
        )
        self.assertNotEqual(
            generator.building_color(27.0, 27.0),
            generator.building_color(81.0, 27.0),
        )

    def test_l_channel_cross_sections_match_left_turn_as_seen_from_start(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        boxes = generator.physical_boxes(spec)

        def occupied(x: float, y: float, z: float) -> bool:
            return any(
                abs(x - box.center[0]) < 0.5 * box.size[0]
                and abs(y - box.center[1]) < 0.5 * box.size[1]
                and abs(z - box.center[2]) < 0.5 * box.size[2]
                for box in boxes
            )

        # RViz left-to-right is descending map X; top-to-bottom is descending map Y.
        sample_xy = (
            ((135.0, 189.0), (108.0, 189.0), (81.0, 189.0)),
            ((135.0, 162.0), (108.0, 162.0), (81.0, 162.0)),
            ((135.0, 135.0), (108.0, 135.0), (81.0, 135.0)),
        )
        self.assertEqual(
            ((True, True, True), (False, False, True), (True, False, True)),
            tuple(
                tuple(occupied(x, y, 5.0) for x, y in row)
                for row in sample_xy
            ),
        )
        for z in (1.0, 10.0):
            self.assertTrue(
                all(occupied(x, y, z) for row in sample_xy for x, y in row)
            )

    def test_new_channel_cross_sections_match_rviz_annotations(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        boxes = generator.physical_boxes(spec)

        def occupied(x: float, y: float, z: float) -> bool:
            return any(
                abs(x - box.center[0]) < 0.5 * box.size[0]
                and abs(y - box.center[1]) < 0.5 * box.size[1]
                and abs(z - box.center[2]) < 0.5 * box.size[2]
                for box in boxes
            )

        def cross_section(
            center_x: float, center_y: float
        ) -> tuple[tuple[bool, ...], ...]:
            return tuple(
                tuple(
                    occupied(center_x + dx, center_y + dy, 5.0)
                    for dx in (27.0, 0.0, -27.0)
                )
                for dy in (27.0, 0.0, -27.0)
            )

        self.assertEqual(
            ((True, False, True), (True, False, True), (True, False, True)),
            cross_section(54.0, 162.0),
        )
        self.assertEqual(
            ((True, False, True), (True, False, False), (True, True, True)),
            cross_section(108.0, 216.0),
        )
        self.assertEqual(
            ((True, False, True), (False, False, False), (True, True, True)),
            cross_section(108.0, 108.0),
        )

    def test_committed_artifacts_are_deterministic_and_current(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        boxes = generator.physical_boxes(spec)
        with tempfile.TemporaryDirectory() as directory:
            generated_sdf = Path(directory) / "city.sdf"
            generated_occupancy = Path(directory) / "city.occupancy3d"
            generator.generate_sdf(spec, boxes, generated_sdf)
            generator.generate_occupancy(spec, boxes, generated_occupancy)
            self.assertEqual(COMMITTED_SDF.read_bytes(), generated_sdf.read_bytes())
            self.assertEqual(
                COMMITTED_OCCUPANCY.read_bytes(), generated_occupancy.read_bytes()
            )


if __name__ == "__main__":
    unittest.main()
