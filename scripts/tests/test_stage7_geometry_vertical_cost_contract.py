#!/usr/bin/env python3
"""Static regressions for sparse geometry and vertical route costs."""

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
        self.assertIn("validateSweptFootprint", segment_validator)
        self.assertIn("validateObservedSweptFootprint", segment_validator)
        self.assertIn("std::ranges::sort(candidates, std::greater<>{})", implementation)

    def test_sparse_shortcuts_remain_curvature_and_corridor_aware(self) -> None:
        implementation = (SOURCE / "static_route_geometry.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("protectedStation", implementation)
        self.assertIn("shortcutWithinTurnBudget", implementation)
        self.assertIn("maximum_shortcut_turn_increase_rad", implementation)
        self.assertIn("pointSegmentDistance", implementation)

    def test_vertical_cost_is_nonzero_and_part_of_the_search_heuristic(self) -> None:
        config = (INCLUDE / "risk_aware_lattice_3d.hpp").read_text(encoding="utf-8")
        cost = (SOURCE / "risk_aware_lattice_3d_cost.cpp").read_text(encoding="utf-8")
        yaml = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")

        self.assertIn("vertical_alignment_cost_weight{0.35}", config)
        self.assertIn("route_shape_vertical_turn_cost_per_rad{0.20}", config)
        self.assertIn(
            "config.vertical_alignment_cost_weight * result.vertical_alignment_time_s",
            cost,
        )
        self.assertIn("verticalFlightPathAngle", cost)
        self.assertIn(
            "config.vertical_alignment_cost_weight * vertical_time_s", cost
        )
        self.assertIn("persistent_planner_horizontal_step_m: 2.0", yaml)
        self.assertIn("persistent_planner_vertical_step_m: 1.0", yaml)
        self.assertNotIn("global_lattice_3d_", yaml)


if __name__ == "__main__":
    unittest.main()
