#!/usr/bin/env python3
"""Static regressions for prioritized workers and atomic world generations."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"
WORLD_RUNTIME = SOURCE / "world"
ROS_RUNTIME = SOURCE / "runtime" / "ros"


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

    def test_world_generation_has_one_private_runtime_owner(self) -> None:
        node_header = (ROS_RUNTIME / "production_mppi_node.hpp").read_text(
            encoding="utf-8"
        )
        pipeline_header = (WORLD_RUNTIME / "world_pipeline_3d.hpp").read_text(
            encoding="utf-8"
        )
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertIn("src/world/world_pipeline_3d.cpp", cmake)
        self.assertIn("std::unique_ptr<WorldPipeline3D> world_pipeline_", node_header)
        for retired_owner in (
            "world_generation_publication_mutex_",
            "local_world_generation_counter_",
            "resident_world_",
        ):
            self.assertNotIn(retired_owner, node_header)
        self.assertIn("std::mutex publication_mutex_", pipeline_header)
        self.assertIn("LocalWorldGenerationCounter", pipeline_header)
        self.assertIn("std::shared_ptr<const WorldSnapshot3D> resident_world_", pipeline_header)

    def test_mixed_generation_resources_fail_closed(self) -> None:
        world = (WORLD_RUNTIME / "production_mppi_route_world.cpp").read_text(
            encoding="utf-8"
        )
        controller_adapter = (
            ROS_RUNTIME / "production_mppi_node_planning_controller.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("generation.gpu_esdf_revision != world.revision", world)
        self.assertNotIn("topological_graph", world)
        self.assertIn("owner->version().base_snapshot_revision", world)
        self.assertIn("action=retry_next_tick", controller_adapter)

    def test_lane_and_generation_backpressure_are_diagnostic(self) -> None:
        diagnostics = (ROS_RUNTIME / "production_mppi_node_summary.cpp").read_text(
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
