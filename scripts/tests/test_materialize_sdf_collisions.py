from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from sdf_collision_materializer import (  # noqa: E402
    CollisionWorldMaterializer,
    MaterializationError,
    ResourceResolver,
    write_materialized_world,
    write_report,
)


class SdfCollisionMaterializerTest(unittest.TestCase):
    def test_preserves_physics_and_adds_px4_world_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            world.write_text(
                """<sdf version="1.6"><world name="candidate">
  <physics name="source" type="ode"><max_step_size>0.004</max_step_size></physics>
  <scene><ambient>0.1 0.2 0.3 1</ambient></scene>
  <plugin filename="legacy.so" name="legacy"/>
</world></sdf>
""",
                encoding="utf-8",
            )
            materializer = CollisionWorldMaterializer(ResourceResolver([], []))

            tree, _ = materializer.materialize(world)

            output_world = tree.getroot().find("world")
            self.assertIsNotNone(output_world)
            self.assertEqual("0.004", output_world.findtext("./physics/max_step_size"))
            self.assertEqual("0.1 0.2 0.3 1", output_world.findtext("./scene/ambient"))
            self.assertEqual("0 0 -9.8", output_world.findtext("gravity"))
            self.assertEqual(
                "EARTH_WGS84",
                output_world.findtext("./spherical_coordinates/surface_model"),
            )
            self.assertIsNone(output_world.find("plugin"))

    def test_resolves_classic_file_uri_against_explicit_model_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            models = root / "models"
            model = models / "warehouse"
            mesh = model / "meshes" / "collision.dae"
            mesh.parent.mkdir(parents=True)
            mesh.write_text("mesh placeholder", encoding="utf-8")
            referring_file = model / "model.sdf"
            resolver = ResourceResolver([], [models])

            resolved = resolver.resolve_mesh(
                "file://models/warehouse/meshes/collision.dae", referring_file
            )

            self.assertEqual(mesh.resolve(), resolved)

    def test_resolved_light_resource_is_skipped_without_guessing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            models = root / "models"
            light = models / "sun"
            light.mkdir(parents=True)
            (light / "model.sdf").write_text(
                '<sdf version="1.6"><light name="sun" type="directional"/></sdf>',
                encoding="utf-8",
            )
            world = root / "world.sdf"
            world.write_text(
                '<sdf version="1.6"><world name="candidate">'
                '<include><uri>model://sun</uri></include>'
                "</world></sdf>",
                encoding="utf-8",
            )
            materializer = CollisionWorldMaterializer(
                ResourceResolver([], [models])
            )

            _, report = materializer.materialize(world)

            self.assertEqual(0, report.collision_instances)
            self.assertEqual(1, report.non_collision_resources_skipped)

    def test_static_include_inside_default_dynamic_wrapper_is_materialized(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            models = root / "models"
            building = models / "building"
            building.mkdir(parents=True)
            (building / "model.sdf").write_text(
                '<sdf version="1.6"><model name="building"><link name="link">'
                '<collision name="wall"><geometry><box><size>1 1 1</size></box>'
                "</geometry></collision></link></model></sdf>",
                encoding="utf-8",
            )
            world = root / "world.sdf"
            world.write_text(
                '<sdf version="1.6"><world name="candidate">'
                '<model name="wrapper"><pose frame="">1 0 0 0 0 0</pose>'
                '<include><uri>model://building</uri>'
                '<static>true</static></include></model></world></sdf>',
                encoding="utf-8",
            )
            materializer = CollisionWorldMaterializer(
                ResourceResolver([], [models])
            )

            _, report = materializer.materialize(world)

            self.assertEqual(1, report.collision_instances)
            self.assertEqual(0, report.dynamic_models_skipped)

    def test_nested_include_keeps_included_model_static_state(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            models = root / "models"
            building = models / "building"
            building.mkdir(parents=True)
            (building / "model.sdf").write_text(
                '<sdf version="1.6"><model name="building"><link name="link">'
                '<collision name="wall"><geometry><box><size>1 1 1</size></box>'
                "</geometry></collision></link></model></sdf>",
                encoding="utf-8",
            )
            world = root / "world.sdf"
            world.write_text(
                '<sdf version="1.6"><world name="candidate">'
                '<model name="static_wrapper"><static>true</static>'
                '<include><uri>model://building</uri></include>'
                "</model></world></sdf>",
                encoding="utf-8",
            )
            materializer = CollisionWorldMaterializer(
                ResourceResolver([], [models])
            )

            _, report = materializer.materialize(world)

            self.assertEqual(0, report.collision_instances)
            self.assertEqual(1, report.dynamic_models_skipped)

    def test_resolves_percent_encoded_model_from_legacy_fuel_cache(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / "cache"
            model = (
                cache
                / "fuel.ignitionrobotics.org"
                / "openrobotics"
                / "models"
                / "test%20tunnel"
                / "3"
            )
            model.mkdir(parents=True)
            expected = model / "model.sdf"
            expected.write_text("<sdf version=\"1.6\"/>", encoding="utf-8")
            resolver = ResourceResolver([cache], [])

            resolved = resolver.resolve_model(
                "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Test Tunnel",
                Path(directory) / "world.sdf",
            )

            self.assertEqual(expected.resolve(), resolved)

    def test_include_pose_overrides_model_pose_and_composes_child_poses(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / "cache"
            model = (
                cache
                / "fuel.gazebosim.org"
                / "openrobotics"
                / "models"
                / "test tunnel"
                / "2"
            )
            mesh = model / "meshes" / "collision.dae"
            mesh.parent.mkdir(parents=True)
            mesh.write_text("mesh placeholder", encoding="utf-8")
            (model / "model.sdf").write_text(
                """<sdf version="1.6">
  <model name="test_tunnel">
    <static>true</static>
    <pose>1 0 0 0 0 0</pose>
    <link name="shell">
      <pose>0 2 0 0 0 0</pose>
      <collision name="physical">
        <pose>0 0 3 0 0 0</pose>
        <geometry><mesh>
          <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Test%20Tunnel/2/files/meshes/collision.dae</uri>
        </mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
""",
                encoding="utf-8",
            )
            world = root / "world.sdf"
            world.write_text(
                """<sdf version="1.6"><world name="candidate">
  <include>
    <name>placed_tunnel</name>
    <pose>10 0 0 0 0 1.5707963267948966</pose>
    <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Test Tunnel</uri>
  </include>
  <model name="moving_prop">
    <static>false</static>
    <link name="link"><collision name="collision">
      <geometry><box><size>1 1 1</size></box></geometry>
    </collision></link>
  </model>
</world></sdf>
""",
                encoding="utf-8",
            )
            output_sdf = root / "materialized.sdf"
            output_report = root / "report.json"
            materializer = CollisionWorldMaterializer(ResourceResolver([cache], []))

            tree, report = materializer.materialize(world)
            fingerprint = write_materialized_world(tree, output_sdf)
            write_report(report, output_sdf, fingerprint, output_report)

            output_root = ET.parse(output_sdf).getroot()
            output_model = output_root.find("./world/model")
            self.assertIsNotNone(output_model)
            pose = tuple(map(float, output_model.findtext("pose", "").split()))
            self.assertAlmostEqual(8.0, pose[0])
            self.assertAlmostEqual(0.0, pose[1])
            self.assertAlmostEqual(3.0, pose[2])
            output_mesh = Path(
                output_model.findtext("./link/collision/geometry/mesh/uri", "")
            )
            self.assertFalse(output_mesh.is_absolute())
            self.assertEqual(mesh.resolve(), (output_sdf.parent / output_mesh).resolve())
            self.assertEqual(1, report.collision_instances)
            self.assertEqual(1, report.dynamic_models_skipped)
            saved_report = json.loads(output_report.read_text(encoding="utf-8"))
            self.assertEqual(fingerprint, saved_report["output_sha256"])

    def test_rejects_unresolved_model_instead_of_dropping_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            world.write_text(
                """<sdf version="1.6"><world name="candidate">
  <include><uri>model://missing</uri></include>
</world></sdf>
""",
                encoding="utf-8",
            )
            materializer = CollisionWorldMaterializer(ResourceResolver([], []))

            with self.assertRaisesRegex(MaterializationError, "not present"):
                materializer.materialize(world)


if __name__ == "__main__":
    unittest.main()
