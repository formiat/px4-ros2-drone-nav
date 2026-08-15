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
COMMITTED_TOPOLOGY = REPO_ROOT / "drone_city_nav/worlds/generated_city.topology3d"
COMMITTED_ESDF = REPO_ROOT / "drone_city_nav/worlds/generated_city.esdf3d"
PLANNER_CONFIG = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"

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
            self.assertIn("passage_structure_11_19_l_intersection_lower", collision_models)
            self.assertIn("passage_structure_11_19_l_west_middle", collision_models)
            self.assertIn("passage_structure_11_19_l_north_middle", collision_models)
            self.assertNotIn("passage_structure_11_19_l_east_middle", collision_models)
            self.assertNotIn("passage_structure_11_19_l_south_middle", collision_models)
            self.assertIn("passage_structure_108_108_t_south_middle", collision_models)
            self.assertNotIn("passage_structure_108_108_t_west_middle", collision_models)
            self.assertNotIn("passage_structure_108_108_t_east_middle", collision_models)
            self.assertNotIn("passage_structure_108_108_t_north_middle", collision_models)

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
            self.assertEqual(5, header[1])
            self.assertEqual(16, header[2])
            self.assertEqual((690, 1050, 80), header[7:10])
            self.assertGreater(header[11], 0)

            chunk_payload_size = 12 + (16 ** 3 // 64) * 8
            with occupancy_path.open("rb") as stream:
                stream.seek(struct.calcsize(header_format) +
                            header[11] * chunk_payload_size)
                self.assertEqual(b"", stream.read())


    def test_builds_typed_derived_compiler_commands(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        esdf_command = generator.static_esdf_command(
            spec,
            Path("/tools/generate_static_esdf_cache"),
            Path("city.occupancy3d"),
            Path("city.esdf3d"),
            6,
        )
        self.assertEqual("/tools/generate_static_esdf_cache", esdf_command[0])
        self.assertEqual("26.0", esdf_command[esdf_command.index(
            "--maximum-distance-m") + 1])
        self.assertEqual("6", esdf_command[esdf_command.index("--workers") + 1])

        topology_command = generator.topology_compiler_command(
            spec,
            Path("/tools/free_space_topology_compiler"),
            Path("city.occupancy3d"),
            Path("city.esdf3d"),
            Path("city.topology3d"),
        )
        expected_options = {
            "--occupancy": "city.occupancy3d",
            "--esdf": "city.esdf3d",
            "--output": "city.topology3d",
            "--open-space-clearance-m": "4.0",
            "--minimum-center-z-m": "1.0",
            "--maximum-center-z-m": "32.0",
            "--minimum-segments": "1",
            "--footprint-radius-m": "0.82",
        }
        for option, value in expected_options.items():
            self.assertEqual(value, topology_command[topology_command.index(option) + 1])

    def test_spec_describes_left_straight_and_t_passage_structures(self) -> None:
        with SPEC_PATH.open(encoding="utf-8") as stream:
            spec = json.load(stream)
        self.assertEqual(5, len(spec["building_grid"]["x_centers_m"]))
        self.assertEqual(8, len(spec["building_grid"]["y_centers_m"]))
        self.assertEqual(4, len(spec["passage_structures"]))
        passage_structures = {
            passage_structure["id"]: passage_structure
            for passage_structure in spec["passage_structures"]
        }
        self.assertTrue(
            all(
                passage_structure["kind"] == "intersection"
                for passage_structure in passage_structures.values()
            )
        )
        self.assertEqual(
            {"west", "north"},
            {
                bridge["id"]
                for bridge in passage_structures["passage_structure_11_19_l"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"west", "east"},
            {
                bridge["id"]
                for bridge in passage_structures["passage_structure_54_162_straight"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"east", "south"},
            {
                bridge["id"]
                for bridge in passage_structures["passage_structure_108_216_l"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertEqual(
            {"south"},
            {
                bridge["id"]
                for bridge in passage_structures["passage_structure_108_108_t"]["bridges"]
                if bridge["blocked"]
            },
        )
        self.assertTrue(all(passage_structure["opening_center_z_m"] == 5.0
                            for passage_structure in passage_structures.values()))
        self.assertTrue(all("centerline_m" not in passage_structure
                            and "edges" not in passage_structure
                            for passage_structure in passage_structures.values()))

        boxes = generator.physical_boxes(spec)
        self.assertEqual(79, len(boxes))
        self.assertEqual(40, sum(box.id.startswith("building_") for box in boxes))
        geometry = {(box.center, box.size, box.visibility_flags) for box in boxes}
        self.assertEqual(len(boxes), len(geometry))

    def test_portal_validation_capsule_matches_runtime_vehicle_geometry(self) -> None:
        spec = generator.load_spec(SPEC_PATH)
        capsule = spec["free_space_topology"]["validation_capsule"]
        planner_text = PLANNER_CONFIG.read_text(encoding="utf-8")

        self.assertIn(
            f"physical_footprint_radius_m: {capsule['radius_m']}", planner_text
        )
        self.assertIn(
            "physical_footprint_lower_extent_m: "
            f"{capsule['lower_extent_m']}",
            planner_text,
        )
        self.assertIn(
            "physical_footprint_upper_extent_m: "
            f"{capsule['upper_extent_m']}",
            planner_text,
        )
        self.assertIn(
            f"physical_footprint_sweep_step_m: {capsule['sweep_step_m']}",
            planner_text,
        )

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

    def test_l_passage_cross_sections_match_left_turn_as_seen_from_start(self) -> None:
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

    def test_new_passage_cross_sections_match_rviz_annotations(self) -> None:
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

    def test_derived_artifacts_match_committed_occupancy(self) -> None:
        occupancy_header_format = "<8sII4f3IQI"
        esdf_header_format = "<8sII4f3IQfI"
        topology_header_format = "<8sIQ4f3I"
        with COMMITTED_OCCUPANCY.open("rb") as stream:
            occupancy = struct.unpack(
                occupancy_header_format,
                stream.read(struct.calcsize(occupancy_header_format)),
            )
        with COMMITTED_ESDF.open("rb") as stream:
            esdf = struct.unpack(
                esdf_header_format,
                stream.read(struct.calcsize(esdf_header_format)),
            )
        with COMMITTED_TOPOLOGY.open("rb") as stream:
            topology = struct.unpack(
                topology_header_format,
                stream.read(struct.calcsize(topology_header_format)),
            )
        self.assertEqual(b"DCNESF3D", esdf[0])
        self.assertEqual(1, esdf[1])
        self.assertEqual(16, esdf[2])
        self.assertEqual(occupancy[3:11], esdf[3:11])
        self.assertEqual(occupancy[10], esdf[10])
        self.assertAlmostEqual(26.0, esdf[11])
        self.assertGreater(esdf[12], 0)
        self.assertEqual(b"DCNFTOP3", topology[0])
        self.assertEqual(2, topology[1])
        self.assertEqual(occupancy[10], topology[2])
        self.assertEqual(occupancy[3:10], topology[3:10])


if __name__ == "__main__":
    unittest.main()
