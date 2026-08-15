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
COMMITTED_ESDF = REPO_ROOT / "drone_city_nav/worlds/generated_city.esdf3d"
PLANNER_CONFIG = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"

SPEC = importlib.util.spec_from_file_location("generate_canonical_world", GENERATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
generator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = generator
SPEC.loader.exec_module(generator)


def read_string(stream) -> str:
    size = struct.unpack("<H", stream.read(2))[0]
    return stream.read(size).decode("utf-8")


def read_points(stream) -> tuple[tuple[float, float, float], ...]:
    count = struct.unpack("<I", stream.read(4))[0]
    return tuple(struct.unpack("<3f", stream.read(12)) for _ in range(count))


def read_portal_graph(stream) -> tuple[list[dict], list[dict]]:
    regions = []
    region_count = struct.unpack("<I", stream.read(4))[0]
    for _ in range(region_count):
        region = {"id": read_string(stream), "portals": []}
        portal_count = struct.unpack("<I", stream.read(4))[0]
        for _ in range(portal_count):
            region["portals"].append({
                "id": read_string(stream),
                "center": struct.unpack("<3f", stream.read(12)),
                "normal": struct.unpack("<3f", stream.read(12)),
                "polygon": read_points(stream),
            })
        regions.append(region)
    edges = []
    edge_count = struct.unpack("<I", stream.read(4))[0]
    for _ in range(edge_count):
        edges.append({
            "id": read_string(stream),
            "region_id": read_string(stream),
            "entry_portal_id": read_string(stream),
            "exit_portal_id": read_string(stream),
            "centerline": read_points(stream),
            "geometry": struct.unpack("<6f", stream.read(24)),
        })
    return regions, edges


class CanonicalWorldGeneratorTest(unittest.TestCase):
    def test_portal_graph_is_derived_from_voxel_geometry(self) -> None:
        dimensions = (12, 12, 8)
        columns: dict[tuple[int, int], int] = {}
        full_column = (1 << dimensions[2]) - 1
        opening_column = (1 << 0) | (1 << 6) | (1 << 7)
        for x in range(2, 10):
            columns[(x, 3)] = full_column
            columns[(x, 8)] = full_column
            for y in range(4, 8):
                columns[(x, y)] = opening_column

        graph = generator.derive_portal_graph(
            columns, dimensions, (0.0, 0.0, 0.0), 1.0,
            minimum_opening_area_m2=4.0,
        )

        self.assertEqual(1, len(graph.regions))
        self.assertEqual(2, len(graph.regions[0].portals))
        self.assertEqual(1, len(graph.traversal_edges))
        edge = graph.traversal_edges[0]
        self.assertEqual((2.0, 10.0),
                         tuple(sorted((edge.centerline[0][0],
                                       edge.centerline[-1][0]))))
        self.assertEqual((1.0, 6.0), (edge.min_z_m, edge.max_z_m))

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
            self.assertEqual(4, header[1])
            self.assertEqual(16, header[2])
            self.assertEqual((690, 1050, 80), header[7:10])
            self.assertGreater(header[11], 0)

            chunk_payload_size = 12 + (16 ** 3 // 64) * 8
            with occupancy_path.open("rb") as stream:
                stream.seek(struct.calcsize(header_format) +
                            header[11] * chunk_payload_size)
                regions, edges = read_portal_graph(stream)
                self.assertEqual(3, len(regions))
                self.assertEqual([2, 2, 3],
                                 sorted(len(region["portals"])
                                        for region in regions))
                self.assertEqual(5, len(edges))
                physical_structure_ids = {
                    structure["id"] for structure in spec["passage_structures"]
                }
                self.assertTrue(
                    all(
                        all(
                            structure_id not in edge["id"]
                            for structure_id in physical_structure_ids
                        )
                        for edge in edges
                    )
                )
                for edge in edges:
                    min_z, max_z, width, height, clearance, speed_limit = \
                        edge["geometry"]
                    self.assertAlmostEqual(1.5, min_z)
                    self.assertAlmostEqual(8.5, max_z)
                    self.assertAlmostEqual(30.0, width)
                    self.assertAlmostEqual(7.0, height)
                    self.assertAlmostEqual(3.5, clearance)
                    self.assertAlmostEqual(10.0, speed_limit)
                straight = next(
                    edge for edge in edges
                    if edge["centerline"][0] == (54.0, 123.0, 5.0)
                    and edge["centerline"][-1] == (54.0, 201.0, 5.0)
                )
                self.assertGreaterEqual(len(straight["centerline"]), 4)
                self.assertEqual(b"", stream.read())

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
        capsule = spec["occupancy"]["portal_graph"]["validation_capsule"]
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

    def test_precomputed_esdf_matches_committed_occupancy(self) -> None:
        occupancy_header_format = "<8sII4f3IQI"
        esdf_header_format = "<8sII4f3IQfI"
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
        self.assertEqual(b"DCNESF3D", esdf[0])
        self.assertEqual(1, esdf[1])
        self.assertEqual(16, esdf[2])
        self.assertEqual(occupancy[3:11], esdf[3:11])
        self.assertEqual(occupancy[10], esdf[10])
        self.assertAlmostEqual(26.0, esdf[11])
        self.assertGreater(esdf[12], 0)


if __name__ == "__main__":
    unittest.main()
