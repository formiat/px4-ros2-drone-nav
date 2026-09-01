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
        distance_header = (INCLUDE / "known_obstacle_distance_3d.hpp").read_text(
            encoding="utf-8"
        )
        distance_update = (
            SOURCE / "known_obstacle_distance_3d_update.cpp"
        ).read_text(encoding="utf-8")
        window = (SOURCE / "observed_esdf_3d_window.cpp").read_text(
            encoding="utf-8"
        )

        for token in (
            "ObservedEsdf3DBuildMode",
            "PreviousObservedEsdf3D",
            "ObservedEsdfCoverage3D",
            "updateObservedEsdf3D",
            "KnownObstacleDistance3D",
            "known_obstacle_distance",
            "classification_override_cells",
        ):
            self.assertIn(token, header)
        for token in (
            "classifyObservedGrid3DIncremental",
            "rawClassificationChangesCoveredByDirtyChunks",
            "inserted_sources",
            "removed_sources",
            "maximum_rebuild_ratio",
        ):
            self.assertIn(token, implementation)
        self.assertNotIn("DistanceField3D::buildLocal", implementation)
        self.assertNotIn("nearest_obstacle_indices", header)
        self.assertIn("immutable", distance_header.lower())
        for token in (
            "knownObstacleRawChangesCovered3D",
            "affected_output_chunks",
            "reused_chunks",
            "maximum_rebuild_ratio",
        ):
            self.assertIn(token, distance_update)
        self.assertIn("source_occupancy", header)
        self.assertNotIn("planLaunchSupportDeparture3D", window)

    def test_online_topology_library_is_removed_from_production(self) -> None:
        yaml = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")
        node = (
            SOURCE / "runtime" / "ros" / "production_mppi_node.hpp"
        ).read_text(encoding="utf-8")
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
