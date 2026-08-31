#!/usr/bin/env python3
"""Static regressions for early successor preparation and certified route splice."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage4RouteSpliceContractTest(unittest.TestCase):
    def test_extension_trigger_uses_measured_tail_latency_braking_and_overlap(self) -> None:
        header = (INCLUDE / "static_route_extension.hpp").read_text(encoding="utf-8")
        implementation = (SOURCE / "static_route_extension.cpp").read_text(
            encoding="utf-8"
        )
        planning = (SOURCE / "production_mppi_node_route_planning.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("StaticRoutePlanningLatencyTracker", header)
        self.assertIn("planning_p95_ms", header)
        self.assertIn("planning_p99_ms", header)
        self.assertIn("build_and_planning_p99_ms", header)
        self.assertIn("jerkLimitedHorizontalStoppingDistanceM", implementation)
        self.assertIn("decision.required_certified_overlap_m", implementation)
        self.assertIn("protected_distance_m + speed_mps * planning_p99_s", implementation)
        self.assertIn(
            "protected_distance_m + speed_mps * build_and_planning_p99_s",
            implementation,
        )
        self.assertIn(
            "static_route_planning_latency_tracker_.record(route_planning_ms,",
            planning,
        )
        self.assertIn("world_telemetry.build_ms", planning)
        self.assertNotIn("maximum_trigger_fraction_of_route", header + implementation)

    def test_pending_route_and_atomic_replacement_require_the_same_splice_proof(self) -> None:
        pending_header = (INCLUDE / "pending_certified_route_3d.hpp").read_text(
            encoding="utf-8"
        )
        pending = (SOURCE / "pending_certified_route_3d.cpp").read_text(
            encoding="utf-8"
        )
        transitions = (
            SOURCE / "execution_route_snapshot_3d_transitions.cpp"
        ).read_text(encoding="utf-8")
        activation = "".join(
            path.read_text(encoding="utf-8")
            for path in (
                SOURCE / "route_activation_coordinator_3d.cpp",
                SOURCE / "route_activation_preparation_3d.cpp",
            )
        )

        self.assertIn("std::optional<CertifiedRouteSplice3D> route_splice", pending_header)
        route_validity = pending.split(
            "case PendingExecutionBaseKind3D::kRoute:", maxsplit=1
        )[1].split("case PendingExecutionBaseKind3D::kRouteHandoff:", maxsplit=1)[0]
        self.assertIn("route_splice.has_value()", route_validity)
        self.assertIn("route_splice->validFor", pending)
        replacement = transitions.split("replaceCertifiedRouteImpl", maxsplit=1)[1]
        self.assertIn("const CertifiedRouteSplice3D& splice", replacement)
        self.assertIn("assessRouteSpliceReadiness3D", replacement)
        self.assertIn("certifyRouteSplice3D", activation)
        self.assertIn(".route_splice =", activation)
        self.assertIn(
            "state.overlap_search ? report.splice.splice : std::nullopt",
            activation,
        )

    def test_runtime_waits_for_the_proof_window_and_keeps_pending_on_cas_loss(self) -> None:
        execution = (SOURCE / "production_mppi_route_execution.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("assessRouteSpliceReadiness3D", execution)
        self.assertIn("routeSpliceWindowExpired3D", execution)
        self.assertIn("execution_supervisor_.acknowledgePendingIfSame", execution)
        self.assertIn("result.pending_route = execution_supervisor_.pending()", execution)

    def test_extension_and_roi_refresh_are_bound_to_execution_authority(self) -> None:
        planning = (SOURCE / "production_mppi_node_planning_tick.cpp").read_text(
            encoding="utf-8"
        )
        extension = (SOURCE / "production_mppi_node_static_extension.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("maybeRequestStaticRouteExtensionFromExecution", planning)
        self.assertIn(
            "const ExecutionPlan3D& source = "
            "*route_execution.source_snapshot",
            extension,
        )
        self.assertIn(
            "const CertifiedRouteSuffix3D* const active_route = source.route();",
            extension,
        )
        self.assertIn("const CertifiedRouteSuffix3D& active_route", extension)
        self.assertIn("maybeRequestStaticRouteExtension(", extension)
        self.assertIn("PlannerSearchContinuityBase3D", extension)
        self.assertIn("makePlannerSearchTransaction3D", extension)
        self.assertIn(
            "std::make_shared<const CertifiedRouteSuffix3D>(active_route)",
            extension,
        )
        self.assertIn("execution_supervisor_.plan()", extension)


if __name__ == "__main__":
    unittest.main()
