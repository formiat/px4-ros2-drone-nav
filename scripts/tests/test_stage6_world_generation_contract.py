#!/usr/bin/env python3
"""Static regressions for prioritized workers and atomic world generations."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage6WorldGenerationContractTest(unittest.TestCase):
    def test_worker_pool_has_reserved_fair_priority_lanes(self) -> None:
        header = (INCLUDE / "bounded_worker_pool.hpp").read_text(encoding="utf-8")
        implementation = (SOURCE / "bounded_worker_pool.cpp").read_text(
            encoding="utf-8"
        )

        for lane in ("kRouteCritical", "kWorldUpdate", "kBackground"):
            self.assertIn(lane, header)
        self.assertIn("route_reserved_tasks_", header)
        self.assertIn("world_reserved_tasks_", header)
        self.assertIn("kMaximumConsecutiveRouteTasks", implementation)
        self.assertIn("kMaximumConsecutiveNonBackgroundTasks", implementation)
        self.assertIn("space_available_.notify_all()", implementation)

    def test_expensive_call_sites_declare_their_lane(self) -> None:
        distance_2d = (SOURCE / "distance_field.cpp").read_text(encoding="utf-8")
        distance_3d = (SOURCE / "distance_field_3d.cpp").read_text(encoding="utf-8")
        geometry = (SOURCE / "static_route_geometry.cpp").read_text(encoding="utf-8")

        self.assertIn("WorkerTaskLane::kWorldUpdate", distance_2d)
        self.assertIn("WorkerTaskLane::kWorldUpdate", distance_3d)
        self.assertIn("WorkerTaskLane::kRouteCritical", geometry)

    def test_cpu_world_and_gpu_esdf_share_one_publication_gate(self) -> None:
        node_header = (SOURCE / "production_mppi_node.hpp").read_text(
            encoding="utf-8"
        )
        esdf = (SOURCE / "production_mppi_node_esdf.cpp").read_text(
            encoding="utf-8"
        )
        observed_esdf = (SOURCE / "production_mppi_node_observed_esdf.cpp").read_text(
            encoding="utf-8"
        )
        planning = (SOURCE / "production_mppi_node_world_generation.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("world_generation_publication_mutex_", node_header)
        self.assertIn("world_generation_publication_mutex_", esdf)
        self.assertIn("world_generation_publication_mutex_", observed_esdf)
        gate = planning.index("world_generation_publication_mutex_")
        resident = planning.index("captured_generation_is_resident", gate)
        gpu_plan = planning.index("engine_->plan(input)", resident)
        self.assertLess(gate, resident)
        self.assertLess(resident, gpu_plan)

    def test_mixed_generation_resources_fail_closed(self) -> None:
        world = (SOURCE / "production_mppi_route_world.cpp").read_text(
            encoding="utf-8"
        )
        runtime = (SOURCE / "production_mppi_node_runtime.cpp").read_text(encoding="utf-8")
        generation = (SOURCE / "production_mppi_node_world_generation.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("generation.gpu_esdf_revision != world.revision", world)
        self.assertNotIn("topological_graph", world)
        self.assertIn("owner->version().base_snapshot_revision", world)
        self.assertIn("productionWorldGenerationCoherent", runtime)
        self.assertIn("sameSnapshot", generation)
        self.assertIn("action=retry_next_tick", generation)

    def test_lane_and_generation_backpressure_are_diagnostic(self) -> None:
        diagnostics = (SOURCE / "production_mppi_node_summary.cpp").read_text(
            encoding="utf-8"
        )

        for field in (
            "worker_route_pending",
            "worker_world_pending",
            "worker_background_pending",
            "worker_route_capacity_waits",
            "world_generation_superseded_ticks",
            "world_generation_rejected_publications",
        ):
            self.assertIn(field, diagnostics)


if __name__ == "__main__":
    unittest.main()
