#!/usr/bin/env python3
"""Source contracts for bounded no-static ESDF updates and latest-raw safety."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE = REPO_ROOT / "drone_city_nav"


class NoStaticLocalEsdfContractTest(unittest.TestCase):
    def test_local_esdf_budget_is_explicit_and_has_recenter_hysteresis(self) -> None:
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        parameters = config["production_mppi_node"]["ros__parameters"]

        self.assertGreater(parameters["no_static_esdf_update_rate_hz"], 0.0)
        self.assertGreater(parameters["no_static_esdf_half_extent_m"], 0.0)
        self.assertGreaterEqual(parameters["no_static_esdf_recenter_margin_m"], 0.0)
        self.assertLess(
            parameters["no_static_esdf_recenter_margin_m"],
            parameters["no_static_esdf_half_extent_m"],
        )

    def test_no_static_build_crops_before_distance_transform(self) -> None:
        source = (PACKAGE / "src/production_mppi_node_esdf.cpp").read_text()

        crop = source.index("cropOccupancyGrid")
        distance_field = source.index("DistanceField2D::build", crop)
        self.assertLess(crop, distance_field)
        self.assertIn("localEsdfNeedsRecenter", source)
        self.assertIn("source_occupied_fingerprint", source)

    def test_execution_validates_latest_raw_snapshot_independently(self) -> None:
        inputs = (PACKAGE / "src/production_mppi_node_inputs.cpp").read_text()
        execution = (PACKAGE / "src/production_mppi_node_execution.cpp").read_text()

        self.assertIn("latest_raw_snapshot_.store", inputs)
        self.assertIn("rawOccupancyGridViewFromRos", execution)
        self.assertIn("&*latest_raw_view", execution)


if __name__ == "__main__":
    unittest.main()
