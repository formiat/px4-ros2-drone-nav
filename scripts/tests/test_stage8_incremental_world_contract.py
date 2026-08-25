#!/usr/bin/env python3
"""Static regressions for incremental ESDF and topology catch-up scheduling."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage8IncrementalWorldContractTest(unittest.TestCase):
    def test_observed_esdf_has_exact_incremental_and_reuse_modes(self) -> None:
        header = (INCLUDE / "observed_esdf_3d.hpp").read_text(encoding="utf-8")
        implementation = (SOURCE / "observed_esdf_3d.cpp").read_text(
            encoding="utf-8"
        )
        production = (SOURCE / "production_mppi_node_observed_esdf.cpp").read_text(
            encoding="utf-8"
        )
        pose_recenter = (SOURCE / "production_mppi_node_inputs_3d.cpp").read_text(
            encoding="utf-8"
        )

        for token in (
            "ObservedEsdf3DBuildMode",
            "PreviousObservedEsdf3D",
            "ObservedEsdfCoverage3D",
            "updateObservedEsdf3D",
        ):
            self.assertIn(token, header)
        target_halo = implementation.index(
            "expandedRegion(source_region, radius_cells"
        )
        source_halo = implementation.index(
            "expandedRegion(target_region, radius_cells", target_halo
        )
        patch_build = implementation.index("DistanceField3D::buildLocal", source_halo)
        self.assertLess(target_halo, source_halo)
        self.assertLess(source_halo, patch_build)
        self.assertIn("rawChangesCoveredByDirtyChunks", implementation)
        self.assertIn("source_occupancy", header)
        self.assertIn("field.stats.mode != ObservedEsdf3DBuildMode::kReused", production)
        parent_validation = production.index("resident_parent_valid")
        gpu_upload = production.index("engine_->updateEsdf", parent_validation)
        self.assertLess(parent_validation, gpu_upload)
        self.assertIn("reason=superseded_esdf_parent", production)
        evidence_change = production.index(
            "if (active_prepared && !execution_evidence_unchanged)"
        )
        parent_selection = production.index(
            "active_incremental_parent_available", evidence_change
        )
        evidence_refresh = production[evidence_change:parent_selection]
        self.assertIn("incremental_refresh=true", evidence_refresh)
        self.assertNotIn("prepared_esdf_.reset()", evidence_refresh)
        self.assertNotIn("preferred_distance_m) + 20.0", production)
        self.assertIn(
            "observed_esdf_resource.local_occupancy->bounds()", pose_recenter
        )

    def test_topology_prioritizes_corridor_and_bounds_backlog_catchup(self) -> None:
        graph = (INCLUDE / "incremental_topology_graph_3d.hpp").read_text(
            encoding="utf-8"
        )
        scheduler = (SOURCE / "incremental_topology_block_scheduler_3d.cpp").read_text(
            encoding="utf-8"
        )
        lifecycle = (SOURCE / "incremental_topology_observed_blocks_3d.cpp").read_text(
            encoding="utf-8"
        )
        yaml = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")

        for token in (
            "maximum_backlog_blocks_per_update",
            "backlog_boost_threshold_blocks",
            "local_priority_radius_m",
            "forward_corridor_radius_m",
            "forward_corridor_lookahead_m",
        ):
            self.assertIn(token, graph)
        self.assertIn("kLocalSafety", scheduler)
        self.assertIn("kForwardCorridor", scheduler)
        self.assertIn("minimum_oldest_blocks_per_update", lifecycle)
        self.assertIn("backlog_boosted", lifecycle)
        self.assertIn(
            "topological_graph_3d_maximum_backlog_blocks_per_update: 64", yaml
        )
        self.assertIn("topological_graph_3d_forward_corridor_lookahead_m: 60.0", yaml)


if __name__ == "__main__":
    unittest.main()
