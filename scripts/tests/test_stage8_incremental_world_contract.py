#!/usr/bin/env python3
"""Static regressions for bounded incremental world updates."""

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
        window = (SOURCE / "observed_esdf_3d_window.cpp").read_text(
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
            "nearest_obstacle_indices",
            "classification_override_cells",
        ):
            self.assertIn(token, header)
        for token in (
            "classifyObservedGrid3DIncremental",
            "rawChangesCoveredByDirtyChunks",
            "inserted_sources",
            "removed_sources",
            "nearest_obstacle_indices",
            "dependency_invalidated_voxels",
            "maximum_rebuild_ratio",
        ):
            self.assertIn(token, implementation)
        self.assertNotIn("DistanceField3D::buildLocal", implementation)
        self.assertIn("source_occupancy", header)
        self.assertIn("field.stats.mode != ObservedEsdf3DBuildMode::kReused", production)
        parent_validation = production.index("resident_parent_valid")
        gpu_upload = production.index("engine_->updateEsdf", parent_validation)
        self.assertLess(parent_validation, gpu_upload)
        self.assertIn("reason=superseded_esdf_parent", production)
        evidence_change = production.index("if (active_prepared && !launch_support_unchanged)")
        parent_selection = production.index(
            "active_incremental_parent_available", evidence_change
        )
        evidence_refresh = production[evidence_change:parent_selection]
        self.assertIn("incremental_refresh=true", evidence_refresh)
        self.assertNotIn("prepared_esdf_.reset()", evidence_refresh)
        self.assertIn("TRANSIENT_EXECUTION_EVIDENCE_CHANGED", production)
        self.assertIn("TRANSIENT_EXECUTION_EVIDENCE_REFRESHED", production)
        self.assertIn("observedEsdfFullAuditDue", production)
        self.assertNotIn("preferred_distance_m) + 20.0", production)
        self.assertNotIn("planLaunchSupportDeparture3D", window)
        self.assertIn(
            "observed_esdf_resource.local_occupancy->bounds()", pose_recenter
        )

    def test_online_topology_library_is_removed_from_production(self) -> None:
        yaml = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")
        node = (SOURCE / "production_mppi_node.hpp").read_text(encoding="utf-8")
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

        for retired_path in (
            INCLUDE / "incremental_topology_graph_3d.hpp",
            INCLUDE / "incremental_topological_planner_3d.hpp",
            SOURCE / "incremental_topology_block_scheduler_3d.cpp",
            SOURCE / "incremental_topology_observed_blocks_3d.cpp",
        ):
            self.assertFalse(retired_path.exists())
        self.assertNotIn("topological_graph_3d_", yaml)
        self.assertNotIn("topologyWorker", node)
        self.assertNotIn("incremental_topology", cmake)
        self.assertNotIn("production_mppi_node_topology.cpp", cmake)


if __name__ == "__main__":
    unittest.main()
