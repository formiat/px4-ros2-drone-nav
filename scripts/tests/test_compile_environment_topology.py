from __future__ import annotations

import hashlib
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from compile_environment_topology import (  # noqa: E402
    StaticMapInputs,
    TopologyCounts,
    compiled_counts,
    compiler_command,
    expected_counts,
    resolve_static_map_inputs,
    select_static_map,
)
from environment_manifest import find_environment, load_manifest  # noqa: E402


MANIFEST_PATH = REPOSITORY / "environments/environment_manifest.yaml"


def _file_contract(path: str, file_path: Path) -> dict[str, object]:
    payload = file_path.read_bytes()
    return {
        "path": path,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
    }


class CompileEnvironmentTopologyTest(unittest.TestCase):
    def test_every_static_map_pins_a_deterministic_topology_profile(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)

        counts = {
            environment["id"]: expected_counts(static_map["topology_compilation"])
            for environment in manifest["environments"]
            for static_map in environment["static_maps"]
        }

        self.assertEqual(
            {
                "finals_prize_round_world_07": TopologyCounts(5, 676, 1321),
                "cave_circuit_practice_01": TopologyCounts(3, 20, 33),
                "urban_circuit_practice_01": TopologyCounts(3, 248, 458),
                "compact_3d_passage_fixture": TopologyCounts(2, 2, 1),
            },
            counts,
        )

    def test_repository_fixture_inputs_are_resolved_without_installation(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)
        environment = find_environment(manifest, "compact_3d_passage_fixture")
        static_map = select_static_map(environment, "r025")

        inputs = resolve_static_map_inputs(
            REPOSITORY, environment, static_map, REPOSITORY / "unused"
        )

        self.assertEqual(
            REPOSITORY / "environments/fixtures/compact_3d_passage/world.occupancy3d",
            inputs.occupancy,
        )
        self.assertTrue(inputs.esdf.is_file())

    def test_compiler_command_forwards_the_complete_profile(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)
        environment = find_environment(manifest, "compact_3d_passage_fixture")
        static_map = select_static_map(environment, None)

        command = compiler_command(
            Path("compiler"),
            StaticMapInputs(Path("world.occupancy3d"), Path("world.esdf3d")),
            Path("world.topology3d"),
            static_map["topology_compilation"],
        )

        self.assertIn("--maximum-portal-voxels", command)
        self.assertIn("--footprint-sweep-step-m", command)
        self.assertEqual("1", command[command.index("--minimum-segments") + 1])

    def test_installed_static_map_must_match_manifest_file_contracts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact_root = root / "environment" / "static"
            occupancy = artifact_root / "maps/world.occupancy3d"
            esdf = artifact_root / "maps/world.esdf3d"
            occupancy.parent.mkdir(parents=True)
            occupancy.write_bytes(b"expected occupancy")
            esdf.write_bytes(b"expected esdf")
            environment = {"id": "environment", "distribution": "release"}
            static_map = {
                "artifact_id": "static",
                "occupancy": _file_contract("maps/world.occupancy3d", occupancy),
                "esdf": _file_contract("maps/world.esdf3d", esdf),
            }
            occupancy.write_bytes(b"tampered occupancy")

            with self.assertRaisesRegex(RuntimeError, "SHA-256 mismatch"):
                resolve_static_map_inputs(
                    REPOSITORY, environment, static_map, root
                )

    def test_compiler_summary_parser_requires_one_typed_count_triplet(self) -> None:
        output = (
            "FREE_SPACE_TOPOLOGY_COMPILED occupancy=world "
            "regions=3 portals=20 segments=33 classification_ms=10\n"
        )

        self.assertEqual(TopologyCounts(3, 20, 33), compiled_counts(output))


if __name__ == "__main__":
    unittest.main()
