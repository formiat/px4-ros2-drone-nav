from __future__ import annotations

import copy
import hashlib
import io
import json
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from environment_artifacts import (  # noqa: E402
    ArtifactError,
    fetch_artifact,
    install_artifact,
    verify_bundle_contents,
    verify_file_contract,
    verify_repository_files,
    write_deterministic_tar_gz,
)
from environment_manifest import (  # noqa: E402
    ManifestError,
    load_manifest,
    validate_manifest,
)
from fuel_license_inventory import (  # noqa: E402
    FuelResourceRequest,
    requests_from_materialization_report,
    summarize_licenses,
)


MANIFEST_PATH = REPOSITORY / "environments/environment_manifest.yaml"


class EnvironmentManifestTest(unittest.TestCase):
    def test_committed_manifest_has_only_distributed_environment_set(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)

        self.assertEqual(
            {
                "finals_prize_round_world_07",
                "cave_circuit_practice_01",
                "compact_3d_passage_fixture",
            },
            {environment["id"] for environment in manifest["environments"]},
        )
        self.assertTrue(manifest["artifact_release"]["published"])
        release_environments = [
            environment
            for environment in manifest["environments"]
            if environment["distribution"] == "release"
        ]
        self.assertEqual(2, len(release_environments))
        for environment in release_environments:
            self.assertEqual(
                {"source_bundle", "static_map_bundle"},
                {artifact["kind"] for artifact in environment["artifacts"]},
            )
            for artifact in environment["artifacts"]:
                self.assertNotEqual("0" * 64, artifact["sha256"])
                self.assertGreater(artifact["size_bytes"], 1)
            inventory_contract = environment["source"]["license_inventory"]
            self.assertNotEqual("0" * 64, inventory_contract["sha256"])
            inventory = json.loads(
                (REPOSITORY / inventory_contract["path"]).read_text(encoding="utf-8")
            )
            self.assertEqual(environment["id"], inventory["environment_id"])
            self.assertEqual(
                1,
                sum(
                    resource["kind"] == "world"
                    for resource in inventory["resources"]
                ),
            )
            self.assertTrue(
                set(summarize_licenses(inventory)).issubset(
                    {"CC-BY-4.0", "CC0-1.0"}
                )
            )

    def test_committed_repository_fixture_matches_manifest(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)

        verified = verify_repository_files(manifest, MANIFEST_PATH)

        self.assertEqual(7, len(verified))

    def test_duplicate_environment_id_is_rejected(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)
        duplicate = copy.deepcopy(manifest["environments"][0])
        manifest["environments"].append(duplicate)

        with self.assertRaisesRegex(ManifestError, "duplicate environment id"):
            validate_manifest(manifest)

    def test_duplicate_artifact_kind_is_rejected(self) -> None:
        manifest = load_manifest(MANIFEST_PATH)
        environment = manifest["environments"][0]
        duplicate = copy.deepcopy(environment["artifacts"][0])
        duplicate["id"] = "alternate_source"
        duplicate["filename"] = "alternate-source.tar.gz"
        environment["artifacts"].append(duplicate)

        with self.assertRaisesRegex(ManifestError, "one source and one static-map"):
            validate_manifest(manifest)


class EnvironmentArtifactTest(unittest.TestCase):
    def test_archive_generation_is_byte_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.txt"
            source.write_text("physical geometry\n", encoding="utf-8")
            metadata = _metadata_bytes(
                "test_environment", "source", "payload/source.txt", source
            )
            first = root / "first.tar.gz"
            second = root / "second.tar.gz"
            entries = [(source, "payload/source.txt")]
            virtual = [(metadata, "payload/ARTIFACT_METADATA.json")]

            write_deterministic_tar_gz(first, entries, virtual)
            write_deterministic_tar_gz(second, entries, virtual)

            self.assertEqual(first.read_bytes(), second.read_bytes())
            verify_bundle_contents(first, "test_environment", "source")

    def test_file_contract_detects_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "artifact.tar.gz"
            path.write_bytes(b"expected")
            contract = {
                "size_bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            verify_file_contract(path, contract)
            path.write_bytes(b"tampered")

            with self.assertRaisesRegex(ArtifactError, "SHA-256 mismatch"):
                verify_file_contract(path, contract)

    def test_fetch_prefers_verified_local_release_and_installs_safely(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "release"
            release.mkdir()
            payload = root / "world.sdf"
            payload.write_text("<sdf version=\"1.10\"/>\n", encoding="utf-8")
            archive = release / "world-source-v1.tar.gz"
            metadata = _metadata_bytes(
                "test_environment", "source", "test/source/world.sdf", payload
            )
            write_deterministic_tar_gz(
                archive,
                [(payload, "test/source/world.sdf")],
                [(metadata, "test/ARTIFACT_METADATA.json")],
            )
            artifact = {
                "id": "source",
                "filename": archive.name,
                "size_bytes": archive.stat().st_size,
                "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            }
            manifest = {
                "artifact_release": {"published": False, "base_urls": []}
            }
            environment = {"id": "test_environment"}

            fetched = fetch_artifact(
                manifest,
                environment,
                artifact,
                release,
                root / "cache",
            )
            installed = install_artifact(fetched, root / "installed")

            self.assertEqual(archive, fetched)
            self.assertEqual(
                '<sdf version="1.10"/>\n',
                (installed / "test/source/world.sdf").read_text(encoding="utf-8"),
            )

    def test_bundle_with_parent_traversal_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "unsafe.tar.gz"
            with tarfile.open(archive, mode="w:gz") as bundle:
                info = tarfile.TarInfo("../outside")
                info.size = 0
                bundle.addfile(info)

            with self.assertRaisesRegex(ArtifactError, "unsafe archive path"):
                verify_bundle_contents(archive)

    def test_bundle_with_undeclared_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = root / "payload"
            payload.write_text("declared", encoding="utf-8")
            extra = root / "extra"
            extra.write_text("not declared", encoding="utf-8")
            metadata = _metadata_bytes("test", "source", "payload", payload)
            archive = root / "extra.tar.gz"
            write_deterministic_tar_gz(
                archive,
                [(payload, "payload"), (extra, "extra")],
                [(metadata, "ARTIFACT_METADATA.json")],
            )

            with self.assertRaisesRegex(ArtifactError, "undeclared"):
                verify_bundle_contents(archive)

    def test_bundle_with_duplicate_path_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "duplicate.tar.gz"
            metadata = {
                "schema": "drone_city_nav_environment_bundle_v1",
                "environment_id": "test",
                "artifact_id": "source",
                "files": [],
            }
            metadata_bytes = (json.dumps(metadata) + "\n").encode("utf-8")
            with tarfile.open(archive, mode="w:gz") as bundle:
                for _ in range(2):
                    info = tarfile.TarInfo("ARTIFACT_METADATA.json")
                    info.size = len(metadata_bytes)
                    bundle.addfile(info, fileobj=io.BytesIO(metadata_bytes))

            with self.assertRaisesRegex(ArtifactError, "duplicate paths"):
                verify_bundle_contents(archive)


class FuelLicenseInventoryTest(unittest.TestCase):
    def test_materialization_paths_become_exact_version_requests(self) -> None:
        report = {
            "model_files": [
                "/cache/fuel.gazebosim.org/openrobotics/models/"
                "Cave%20Straight/7/model.sdf",
                "/cache/fuel.ignitionrobotics.org/openrobotics/models/"
                "Artifact/3/model.sdf",
            ]
        }

        requests = requests_from_materialization_report(
            "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Test%20World",
            2,
            report,
        )

        self.assertEqual(
            {
                FuelResourceRequest("world", "OpenRobotics", "Test World", 2),
                FuelResourceRequest("model", "openrobotics", "Cave Straight", 7),
                FuelResourceRequest("model", "openrobotics", "Artifact", 3),
            },
            set(requests),
        )


def _metadata_bytes(
    environment_id: str, artifact_id: str, member_name: str, member_path: Path
) -> bytes:
    metadata = {
        "schema": "drone_city_nav_environment_bundle_v1",
        "environment_id": environment_id,
        "artifact_id": artifact_id,
        "files": [
            {
                "path": member_name,
                "size_bytes": member_path.stat().st_size,
                "sha256": hashlib.sha256(member_path.read_bytes()).hexdigest(),
            }
        ],
    }
    return (json.dumps(metadata, sort_keys=True) + "\n").encode("utf-8")


if __name__ == "__main__":
    unittest.main()
