from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from prepare_environment_demo import DemoPreparationError, load_demo_catalog  # noqa: E402


class EnvironmentDemoCatalogTest(unittest.TestCase):
    def test_catalog_covers_downloaded_environment_candidates(self) -> None:
        catalog = load_demo_catalog(
            REPOSITORY / "environments/environment_demo_catalog.json"
        )

        self.assertEqual(
            {
                "finals_prize_round_world_07",
                "cave_circuit_practice_01",
                "urban_circuit_practice_01",
                "tunnel_circuit_practice_01",
                "cave_world",
                "industrial_warehouse",
                "aws_robomaker_small_warehouse",
                "aws_robomaker_hospital",
            },
            set(catalog),
        )
        self.assertEqual(
            {
                "finals_prize_round_world_07",
                "cave_circuit_practice_01",
                "urban_circuit_practice_01",
            },
            {
                environment_id
                for environment_id, environment in catalog.items()
                if environment.source_kind == "release"
            },
        )

    def test_local_entry_requires_a_world_and_resource_arrays(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "catalog.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "drone_city_nav_environment_demo_catalog_v1",
                        "environments": [
                            {
                                "id": "candidate",
                                "display_name": "Candidate",
                                "source_kind": "local",
                                "fuel_caches": [],
                                "model_paths": [],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(DemoPreparationError, "needs source_world"):
                load_demo_catalog(path)

    def test_release_entry_cannot_override_the_manifest_source(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "catalog.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "drone_city_nav_environment_demo_catalog_v1",
                        "environments": [
                            {
                                "id": "release",
                                "display_name": "Release",
                                "source_kind": "release",
                                "resource_paths": ["external/resources"],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(DemoPreparationError, "must use the manifest"):
                load_demo_catalog(path)


if __name__ == "__main__":
    unittest.main()
