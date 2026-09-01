#!/usr/bin/env python3
"""Architectural include bans for controller-neutral navigation contracts."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
INCLUDE = REPOSITORY / "drone_city_nav" / "include" / "drone_city_nav"
SOURCE = REPOSITORY / "drone_city_nav" / "src"
CMAKE = PACKAGE / "CMakeLists.txt"
CONTROLLER_NEUTRAL_HEADERS = (
    "compiled_trajectory_3d.hpp",
    "control_contracts_3d.hpp",
    "control_route_projection_3d.hpp",
    "derived_clearance_3d.hpp",
    "dynamic_handoff_validator_3d.hpp",
    "esdf_query.hpp",
    "finite_execution_path_3d.hpp",
    "finite_motion_horizon_3d.hpp",
    "motion_altitude_envelope_3d.hpp",
    "motion_dynamics_3d.hpp",
    "navigation_state_prediction.hpp",
    "observed_esdf_3d.hpp",
    "persistent_dstar_lite_planner_3d.hpp",
    "route_3d.hpp",
    "route_planning_3d.hpp",
    "route_risk_annotation_3d.hpp",
    "route_risk_policy_3d.hpp",
    "static_route_extension.hpp",
    "swept_footprint.hpp",
    "tracking_error_tube_handoff_3d.hpp",
    "trajectory_control_reference_3d.hpp",
    "trajectory_compiler_3d.hpp",
    "world_snapshot_3d.hpp",
)
EXECUTION_HEADERS = tuple(sorted(INCLUDE.glob("execution*.hpp"))) + (
    INCLUDE / "committed_execution_authority_3d.hpp",
)
# This ROS-free use case is intentionally the MPPI-side adapter that turns a
# controller result into neutral execution-plan transitions.
MPPI_EXECUTION_ADAPTERS = {
    SOURCE / "runtime" / "execution_horizon_assembler_3d.cpp",
    SOURCE / "runtime" / "execution_horizon_assembler_3d.hpp",
}
EXECUTION_IMPLEMENTATION = tuple(
    path
    for path in (
        *sorted(SOURCE.glob("execution*.cpp")),
        *sorted(SOURCE.glob("execution*.hpp")),
        *sorted((SOURCE / "route_application").glob("execution*.cpp")),
        *sorted((SOURCE / "route_application").glob("execution*.hpp")),
        SOURCE / "committed_execution_authority_3d.cpp",
    )
    if path not in MPPI_EXECUTION_ADAPTERS
)
LOCAL_INCLUDE_PATTERN = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.MULTILINE)
PURE_RUNTIME_BANNED_TOKENS = (
    "production_mppi_node.hpp",
    "rclcpp",
    "/msg/",
)
CONTROLLER_BANNED_TOKENS = (
    "drone_city_nav/mppi/",
    "mppi::",
)
ROUTE_PRIVATE_HEADERS = {
    "execution_publication_navigation_rebase_3d.hpp",
    "production_mppi_execution_control.hpp",
    "production_mppi_node_types.hpp",
    "production_mppi_route_helpers.hpp",
    "production_planner_search_transaction_3d.hpp",
    "production_route_pipeline_artifacts_3d.hpp",
    "route_activation_coordinator_3d.hpp",
    "route_activation_preparation_3d.hpp",
    "route_execution_selection_3d.hpp",
    "route_execution_selector_3d.hpp",
    "route_lifecycle_coordinator_3d.hpp",
    "route_materializer_3d.hpp",
    "route_planner_3d.hpp",
    "route_planning_coordinator_3d.hpp",
    "route_trajectory_compiler_3d.hpp",
}
LEGACY_ORCHESTRATION_CONTRACT_TESTS = (
    "test_execution_input_contract.py",
    "test_planner_readiness_contract.py",
    "test_runtime_transport_budget_contract.py",
    "test_stage2_execution_transport_contract.py",
    "test_stage3_endpoint_execution_contract.py",
    "test_stage7_geometry_vertical_cost_contract.py",
)
WORLD_RUNTIME_INCLUDE_ROOTS = (SOURCE / "world",)
ROUTE_RUNTIME_INCLUDE_ROOTS = (
    SOURCE / "route_application",
    *WORLD_RUNTIME_INCLUDE_ROOTS,
)
MPPI_RUNTIME_INCLUDE_ROOTS = (SOURCE / "runtime", *ROUTE_RUNTIME_INCLUDE_ROOTS)
RUNTIME_INCLUDE_ROOTS = {
    "DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES": WORLD_RUNTIME_INCLUDE_ROOTS,
    "DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES": ROUTE_RUNTIME_INCLUDE_ROOTS,
    "DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES": MPPI_RUNTIME_INCLUDE_ROOTS,
}


# Declared layer graph. A layer's headers may include only headers owned by that
# layer or by one of its transitive predecessors here. This mirrors the
# assert_drone_city_nav_layer_dependency() calls in CMakeLists.txt; the CMake
# guard covers link edges, this covers include edges, which link edges alone
# cannot see because every header shares one include root.
LAYER_PREDECESSORS = {
    "model": (),
    "control_contracts": ("model",),
    "route_contracts": ("model",),
    "world": ("model", "route_contracts"),
    "collision": ("world",),
    "finite_execution": ("collision", "control_contracts"),
    "planning": ("collision",),
    "trajectory": ("planning",),
    "execution": ("trajectory", "finite_execution"),
    "control": ("execution",),
    "runtime": ("control",),
    # ROS adapters and node headers sit above every domain layer.
    "ros": ("runtime",),
}
PACKAGE_INCLUDE_PATTERN = re.compile(r'#\s*include\s*"drone_city_nav/([A-Za-z0-9_./+-]+\.hpp)"')


def layer_closure(layer: str) -> frozenset[str]:
    reached = {layer}
    pending = [layer]
    while pending:
        current = pending.pop()
        for predecessor in LAYER_PREDECESSORS[current]:
            if predecessor not in reached:
                reached.add(predecessor)
                pending.append(predecessor)
    return frozenset(reached)


def cmake_header_manifest(layer: str) -> tuple[str, ...]:
    text = CMAKE.read_text(encoding="utf-8")
    variable = f"DRONE_CITY_NAV_{layer.upper()}_HEADERS"
    match = re.search(rf"set\({re.escape(variable)}\s+(.*?)\)", text, re.DOTALL)
    if match is None:
        raise AssertionError(f"missing CMake header manifest {variable}")
    return tuple(
        Path(relative).name
        for relative in re.findall(
            r"include/drone_city_nav/[A-Za-z0-9_./+-]+\.hpp", match.group(1)
        )
    )


def header_owners() -> dict[str, str]:
    owners: dict[str, str] = {}
    for layer in LAYER_PREDECESSORS:
        for header in cmake_header_manifest(layer):
            if header in owners:
                raise AssertionError(
                    f"{header} is owned by both {owners[header]} and {layer}"
                )
            owners[header] = layer
    return owners


def cmake_source_manifest(variable: str) -> tuple[Path, ...]:
    text = CMAKE.read_text(encoding="utf-8")
    match = re.search(rf"set\({re.escape(variable)}\s+(.*?)\)", text, re.DOTALL)
    if match is None:
        raise AssertionError(f"missing CMake source manifest {variable}")
    return tuple(
        PACKAGE / relative
        for relative in re.findall(r"src/[A-Za-z0-9_./+-]+\.(?:cpp|cu)", match.group(1))
    )


def component_sources() -> tuple[Path, ...]:
    text = CMAKE.read_text(encoding="utf-8")
    match = re.search(
        r"add_library\(\s*drone_city_nav_production_mppi_component\s+SHARED(.*?)\)",
        text,
        re.DOTALL,
    )
    if match is None:
        raise AssertionError("missing production MPPI component source list")
    return tuple(
        PACKAGE / relative
        for relative in re.findall(r"src/[A-Za-z0-9_./+-]+\.(?:cpp|cu)", match.group(1))
    )


def resolve_local_include(
    owner: Path, include: str, include_roots: tuple[Path, ...]
) -> Path | None:
    candidates = [owner.parent / include]
    if include.startswith("drone_city_nav/"):
        candidates.append(PACKAGE / "include" / include)
    else:
        candidates.extend(root / include for root in include_roots)
        candidates.append(INCLUDE / include)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def local_dependency_closure(
    roots: tuple[Path, ...], include_roots: tuple[Path, ...]
) -> set[Path]:
    pending = [path.resolve() for path in roots]
    visited: set[Path] = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        if not path.is_file():
            raise AssertionError(f"runtime source does not exist: {path}")
        visited.add(path)
        text = path.read_text(encoding="utf-8")
        for include in LOCAL_INCLUDE_PATTERN.findall(text):
            dependency = resolve_local_include(path, include, include_roots)
            if dependency is not None and dependency not in visited:
                pending.append(dependency)
    return visited


class NavigationDependencyContractTest(unittest.TestCase):
    def test_every_public_header_has_exactly_one_owning_target(self) -> None:
        owners = header_owners()
        declared = set(owners)
        present = {path.name for path in INCLUDE.glob("*.hpp")}
        self.assertEqual(
            present - declared,
            set(),
            "public headers missing from every DRONE_CITY_NAV_*_HEADERS manifest",
        )
        self.assertEqual(
            declared - present,
            set(),
            "header manifests name files that do not exist",
        )

    def test_public_headers_never_include_a_higher_layer(self) -> None:
        owners = header_owners()
        for header, layer in sorted(owners.items()):
            allowed = layer_closure(layer)
            text = (INCLUDE / header).read_text(encoding="utf-8")
            for included in PACKAGE_INCLUDE_PATTERN.findall(text):
                included_name = Path(included).name
                included_layer = owners.get(included_name)
                if included_layer is None:
                    continue
                with self.subTest(header=header, includes=included_name):
                    self.assertIn(
                        included_layer,
                        allowed,
                        f"{header} ({layer}) includes {included_name} "
                        f"({included_layer}), which is not below {layer}",
                    )

    def test_layer_sources_never_include_a_higher_layer(self) -> None:
        owners = header_owners()
        for layer in LAYER_PREDECESSORS:
            if layer == "ros":
                continue
            allowed = layer_closure(layer)
            match = re.search(
                rf"add_drone_city_nav_layer\(\s*drone_city_nav_{layer}\b(.*?)\)\n",
                CMAKE.read_text(encoding="utf-8"),
                re.DOTALL,
            )
            self.assertIsNotNone(match, f"missing layer target drone_city_nav_{layer}")
            for relative in re.findall(r"src/[A-Za-z0-9_./+-]+\.cpp", match.group(1)):
                text = (PACKAGE / relative).read_text(encoding="utf-8")
                for included in PACKAGE_INCLUDE_PATTERN.findall(text):
                    included_layer = owners.get(Path(included).name)
                    if included_layer is None:
                        continue
                    with self.subTest(source=relative, includes=included):
                        self.assertIn(
                            included_layer,
                            allowed,
                            f"{relative} ({layer}) includes {included} "
                            f"({included_layer}), which is not below {layer}",
                        )

    def test_orchestration_contracts_do_not_parse_source_order(self) -> None:
        test_directory = REPOSITORY / "scripts" / "tests"
        for name in LEGACY_ORCHESTRATION_CONTRACT_TESTS:
            text = (test_directory / name).read_text(encoding="utf-8")
            with self.subTest(test=name):
                self.assertNotIn(".split(", text)
                self.assertNotIn(".index(", text)

    def test_world_planning_and_trajectory_contracts_do_not_depend_on_mppi(self) -> None:
        for name in CONTROLLER_NEUTRAL_HEADERS:
            with self.subTest(header=name):
                text = (INCLUDE / name).read_text(encoding="utf-8")
                self.assertNotIn("drone_city_nav/mppi/", text)
                self.assertNotIn("mppi::", text)

    def test_private_runtime_manifests_are_disjoint_from_the_ros_component(
        self,
    ) -> None:
        manifests = {
            "world": set(cmake_source_manifest("DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES")),
            "route": set(cmake_source_manifest("DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES")),
            "mppi": set(cmake_source_manifest("DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES")),
            "component": set(component_sources()),
        }
        for name, paths in manifests.items():
            with self.subTest(manifest=name):
                self.assertTrue(paths)
                self.assertTrue(all(path.is_file() for path in paths))
        names = tuple(manifests)
        for index, first in enumerate(names):
            for second in names[index + 1 :]:
                with self.subTest(first=first, second=second):
                    self.assertFalse(manifests[first] & manifests[second])

    def test_private_runtime_sources_and_include_roots_follow_layers(self) -> None:
        expected_source_roots = {
            "DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES": (SOURCE / "world",),
            "DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES": (SOURCE / "route_application",),
            "DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES": (SOURCE / "runtime",),
        }
        for variable, allowed_roots in expected_source_roots.items():
            for path in cmake_source_manifest(variable):
                with self.subTest(manifest=variable, path=path.name):
                    self.assertTrue(
                        any(path.is_relative_to(root) for root in allowed_roots)
                    )
        for path in component_sources():
            with self.subTest(component_source=path.name):
                self.assertTrue(path.is_relative_to(SOURCE / "runtime" / "ros"))

        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertNotIn(
            'PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"', cmake
        )
        for relative_root in (
            "src/world",
            "src/route_application",
            "src/runtime",
        ):
            with self.subTest(include_root=relative_root):
                self.assertIn(
                    f"${{CMAKE_CURRENT_SOURCE_DIR}}/{relative_root}", cmake
                )

    def test_private_runtime_header_graphs_are_ros_free(self) -> None:
        for variable in (
            "DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES",
            "DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES",
            "DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES",
        ):
            dependencies = local_dependency_closure(
                cmake_source_manifest(variable), RUNTIME_INCLUDE_ROOTS[variable]
            )
            for path in dependencies:
                text = path.read_text(encoding="utf-8")
                for token in PURE_RUNTIME_BANNED_TOKENS:
                    with self.subTest(
                        manifest=variable,
                        path=path.relative_to(REPOSITORY),
                        token=token,
                    ):
                        self.assertNotIn(token, text)

    def test_world_runtime_does_not_reach_route_or_controller_headers(self) -> None:
        dependencies = local_dependency_closure(
            cmake_source_manifest("DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES"),
            WORLD_RUNTIME_INCLUDE_ROOTS,
        )
        for path in dependencies:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.relative_to(REPOSITORY)):
                self.assertNotIn(path.name, ROUTE_PRIVATE_HEADERS)
                for token in CONTROLLER_BANNED_TOKENS:
                    self.assertNotIn(token, text)

    def test_route_runtime_does_not_reach_controller_headers(self) -> None:
        dependencies = local_dependency_closure(
            cmake_source_manifest("DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES"),
            ROUTE_RUNTIME_INCLUDE_ROOTS,
        )
        for path in dependencies:
            text = path.read_text(encoding="utf-8")
            for token in CONTROLLER_BANNED_TOKENS:
                with self.subTest(path=path.relative_to(REPOSITORY), token=token):
                    self.assertNotIn(token, text)

    def test_execution_contracts_and_implementation_do_not_depend_on_mppi(
        self,
    ) -> None:
        for path in EXECUTION_HEADERS + EXECUTION_IMPLEMENTATION:
            with self.subTest(path=path.relative_to(REPOSITORY)):
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("drone_city_nav/mppi/", text)
                self.assertNotIn("mppi::", text)


if __name__ == "__main__":
    unittest.main()
