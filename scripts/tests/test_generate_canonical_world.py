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
            collision_models = {
                model.attrib["name"]
                for model in root.findall("./world/model")
                if model.find("./link/collision") is not None
            }
            self.assertIn("channel_11_19_l_intersection_lower", collision_models)
            self.assertIn("channel_11_19_l_east_middle", collision_models)
            self.assertIn("channel_11_19_l_north_middle", collision_models)
            self.assertNotIn("channel_11_19_l_west_middle", collision_models)
            self.assertNotIn("channel_11_19_l_south_middle", collision_models)
            self.assertIn("channel_22_30_lower", collision_models)
            self.assertIn("channel_22_30_upper", collision_models)
            channel_poses = {
                model.attrib["name"]: tuple(
                    map(float, model.findtext("pose", default="").split())
                )
                for model in root.findall("./world/model")
                if model.attrib["name"].startswith("channel_")
                and model.find("./link/collision") is not None
            }
            self.assertGreater(len(channel_poses), 2)
            for pose in channel_poses.values():
                self.assertEqual((0.0, 0.0, 0.0), pose[3:6])
            self.assertIn("building_001", collision_models)

            header_format = "<8sII4f3IQI"
            with occupancy_path.open("rb") as stream:
                header = struct.unpack(
                    header_format, stream.read(struct.calcsize(header_format))
                )
            self.assertEqual(b"DCNOCC3D", header[0])
            self.assertEqual(1, header[1])
            self.assertEqual(16, header[2])
            self.assertEqual((690, 1050, 80), header[7:10])
            self.assertGreater(header[11], 0)

    def test_spec_contains_only_straight_and_l_shaped_channels(self) -> None:
        with SPEC_PATH.open(encoding="utf-8") as stream:
            spec = json.load(stream)
        kinds = {channel["kind"] for channel in spec["channels"]}
        self.assertEqual({"straight", "l_shaped"}, kinds)

    def test_l_channel_has_four_bridges_and_two_physical_middle_masses(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        channel = next(
            channel for channel in spec["channels"]
            if channel["kind"] == "l_shaped"
        )
        self.assertEqual(4, len(channel["bridges"]))
        self.assertEqual(
            {"east", "north"},
            {bridge["id"] for bridge in channel["bridges"] if bridge["blocked"]},
        )
        boxes = {box.id for box in generator.channel_boxes(channel)}
        self.assertEqual(
            {
                "channel_11_19_l_intersection_lower",
                "channel_11_19_l_intersection_upper",
                "channel_11_19_l_west_lower",
                "channel_11_19_l_west_upper",
                "channel_11_19_l_east_lower",
                "channel_11_19_l_east_upper",
                "channel_11_19_l_east_middle",
                "channel_11_19_l_south_lower",
                "channel_11_19_l_south_upper",
                "channel_11_19_l_north_lower",
                "channel_11_19_l_north_upper",
                "channel_11_19_l_north_middle",
            },
            boxes,
        )

    def test_l_channel_cross_sections_match_left_turn_volume(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        boxes = generator.physical_boxes(spec)

        def occupied(x: float, y: float, z: float) -> bool:
            return any(
                abs(x - box.center[0]) < 0.5 * box.size[0]
                and abs(y - box.center[1]) < 0.5 * box.size[1]
                and abs(z - box.center[2]) < 0.5 * box.size[2]
                for box in boxes
            )

        sample_xy = (
            ((81.0, 189.0), (108.0, 189.0), (135.0, 189.0)),
            ((81.0, 162.0), (108.0, 162.0), (135.0, 162.0)),
            ((81.0, 135.0), (108.0, 135.0), (135.0, 135.0)),
        )
        self.assertEqual(
            ((True, True, True), (False, False, True), (True, False, True)),
            tuple(tuple(occupied(x, y, 5.0) for x, y in row)
                  for row in sample_xy),
        )
        for z in (1.0, 10.0):
            self.assertTrue(
                all(occupied(x, y, z) for row in sample_xy for x, y in row)
            )

    def test_all_channels_are_horizontal(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        for channel in spec["channels"]:
            with self.subTest(channel=channel["id"]):
                reference_heights = {
                    float(point[2]) for point in channel["centerline_m"]
                }
                self.assertEqual(1, len(reference_heights))

        converted = next(
            channel for channel in spec["channels"]
            if channel["id"] == "channel_22_30"
        )
        self.assertEqual("straight", converted["kind"])
        boxes = generator.channel_boxes(converted)
        self.assertEqual(
            {"channel_22_30_lower", "channel_22_30_upper"},
            {box.id for box in boxes},
        )
        self.assertEqual((162.0, 297.0, 10.75), boxes[0].center)
        self.assertEqual((162.0, 297.0, 30.25), boxes[1].center)

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
