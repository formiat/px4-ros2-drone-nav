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
            self.assertIn("building_001", collision_models)
            self.assertIn("building_040", collision_models)
            self.assertIn("channel_11_19_l_intersection_lower", collision_models)
            self.assertIn("channel_11_19_l_west_middle", collision_models)
            self.assertIn("channel_11_19_l_north_middle", collision_models)
            self.assertNotIn("channel_11_19_l_east_middle", collision_models)
            self.assertNotIn("channel_11_19_l_south_middle", collision_models)

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

    def test_spec_describes_one_left_turn_channel(self) -> None:
        with SPEC_PATH.open(encoding="utf-8") as stream:
            spec = json.load(stream)
        self.assertEqual(5, len(spec["building_grid"]["x_centers_m"]))
        self.assertEqual(8, len(spec["building_grid"]["y_centers_m"]))
        self.assertEqual(1, len(spec["channels"]))
        channel = spec["channels"][0]
        self.assertEqual("channel_11_19_l", channel["id"])
        self.assertEqual("l_shaped", channel["kind"])
        self.assertEqual(
            {"west", "north"},
            {bridge["id"] for bridge in channel["bridges"] if bridge["blocked"]},
        )

        boxes = generator.physical_boxes(spec)
        self.assertEqual(52, len(boxes))
        self.assertEqual(
            40, sum(box.id.startswith("building_") for box in boxes)
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
