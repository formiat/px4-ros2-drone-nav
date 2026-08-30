#!/usr/bin/env python3
"""Static regressions for sparse geometry and the shared 3D time model."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage7GeometryVerticalCostContractTest(unittest.TestCase):
    def test_geometry_uses_sparse_bounded_farthest_first_batches(self) -> None:
        header = (INCLUDE / "static_route_geometry.hpp").read_text(encoding="utf-8")
        implementation = (SOURCE / "static_route_geometry.cpp").read_text(
            encoding="utf-8"
        )

        for field in (
            "sparse_deviation_tolerance_m",
            "shortcut_validation_batch_size",
            "maximum_shortcut_turn_increase_rad",
        ):
            self.assertIn(field, header)
        sparse = implementation.index("sparseRouteIndices(")
        candidates = implementation.index("shortcutCandidateIndices(", sparse)
        batches = implementation.index("batch_begin", candidates)
        raw_validation = implementation.index("segmentValid(", batches)
        segment_validator = implementation.split(
            "segmentValid", maxsplit=1
        )[1].split("sparseRouteIndices", maxsplit=1)[0]
        self.assertLess(sparse, candidates)
        self.assertLess(candidates, batches)
        self.assertLess(batches, raw_validation)
        self.assertIn("const OccupiedCollisionOracle3D& oracle", segment_validator)
        self.assertIn("oracle.validateSegment", segment_validator)
        self.assertNotIn("validateRawSweptFootprint", implementation)
        self.assertIn("std::ranges::sort(candidates, std::greater<>{})", implementation)

    def test_sparse_shortcuts_remain_curvature_and_corridor_aware(self) -> None:
        implementation = (SOURCE / "static_route_geometry.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("protectedStation", implementation)
        self.assertIn("shortcutWithinTurnBudget", implementation)
        self.assertIn("maximum_shortcut_turn_increase_rad", implementation)
        self.assertIn("pointSegmentDistance", implementation)

    def test_vertical_motion_is_part_of_the_persistent_search_time_model(self) -> None:
        model = (INCLUDE / "flight_time_model_3d.hpp").read_text(encoding="utf-8")
        implementation = (SOURCE / "flight_time_model_3d.cpp").read_text(
            encoding="utf-8"
        )
        search = (SOURCE / "persistent_dstar_lite_planner_3d_search.cpp").read_text(
            encoding="utf-8"
        )
        yaml = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")

        self.assertIn("maximum_vertical_speed_mps", model)
        self.assertIn("maximum_vertical_acceleration_mps2", model)
        self.assertIn("vertical / model.maximum_vertical_speed_mps", implementation)
        self.assertIn("minimumFlightTranslationTime3D", search)
        self.assertIn("parameterizeFlightPathTime3D", search)
        self.assertIn("persistent_planner_minimum_horizontal_step_m: 2.0", yaml)
        self.assertIn("persistent_planner_minimum_vertical_step_m: 1.0", yaml)
        self.assertIn("persistent_planner_maximum_adaptive_lattice_level: 2", yaml)
        self.assertNotIn("global_lattice_3d_", yaml)


if __name__ == "__main__":
    unittest.main()
