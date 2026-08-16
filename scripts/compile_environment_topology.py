#!/usr/bin/env python3
"""Compile and verify FreeSpaceTopology3D for a manifest static map."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from environment_artifacts import verify_file_contract
from environment_manifest import (
    ManifestError,
    find_environment,
    load_manifest,
    repository_root,
)


DEFAULT_MANIFEST = Path("environments/environment_manifest.yaml")
DEFAULT_COMPILER = Path("build/drone_city_nav/free_space_topology_compiler")
DEFAULT_INSTALL_ROOT = Path("external/environment-artifacts/installed")
_PARAMETER_FLAGS = (
    ("maximum_clearance_m", "--maximum-clearance-m"),
    ("open_space_clearance_m", "--open-space-clearance-m"),
    ("speed_limit_mps", "--speed-limit-mps"),
    ("medial_clearance_weight", "--medial-clearance-weight"),
    ("medial_ridge_prominence_m", "--medial-ridge-prominence-m"),
    ("medial_band_radius_cells", "--medial-band-radius-cells"),
    ("chunk_size_cells", "--chunk-size-cells"),
    ("minimum_open_region_voxels", "--minimum-open-region-voxels"),
    (
        "minimum_constrained_component_voxels",
        "--minimum-constrained-component-voxels",
    ),
    ("minimum_portal_voxels", "--minimum-portal-voxels"),
    ("maximum_portal_voxels", "--maximum-portal-voxels"),
    ("footprint_radius_m", "--footprint-radius-m"),
    ("footprint_lower_extent_m", "--footprint-lower-extent-m"),
    ("footprint_upper_extent_m", "--footprint-upper-extent-m"),
    ("footprint_sweep_step_m", "--footprint-sweep-step-m"),
    ("minimum_segments", "--minimum-segments"),
)
_COUNT_PATTERN = re.compile(r"\bregions=(\d+) portals=(\d+) segments=(\d+)\b")


class TopologyCompilationError(RuntimeError):
    """Raised when topology compilation or its deterministic contract fails."""


@dataclass(frozen=True)
class StaticMapInputs:
    occupancy: Path
    esdf: Path


@dataclass(frozen=True)
class TopologyCounts:
    regions: int
    portals: int
    segments: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--static-map")
    parser.add_argument("--compiler", type=Path, default=DEFAULT_COMPILER)
    parser.add_argument("--asset-install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-count-drift", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def select_static_map(environment: dict[str, Any], map_id: str | None) -> dict[str, Any]:
    maps = environment["static_maps"]
    if map_id is None:
        if len(maps) != 1:
            raise TopologyCompilationError("--static-map is required for this environment")
        return maps[0]
    for static_map in maps:
        if static_map["id"] == map_id:
            return static_map
    raise TopologyCompilationError(
        f"unknown static map for {environment['id']}: {map_id}"
    )


def resolve_static_map_inputs(
    repository: Path,
    environment: dict[str, Any],
    static_map: dict[str, Any],
    install_root: Path,
) -> StaticMapInputs:
    if environment["distribution"] == "repository":
        root = repository
    else:
        root = install_root / environment["id"] / static_map["artifact_id"]
    inputs = StaticMapInputs(
        occupancy=root / static_map["occupancy"]["path"],
        esdf=root / static_map["esdf"]["path"],
    )
    missing = [str(path) for path in (inputs.occupancy, inputs.esdf) if not path.is_file()]
    if missing:
        raise TopologyCompilationError(
            "static map is not installed: "
            + ", ".join(missing)
            + f"; fetch artifact {static_map.get('artifact_id', 'repository')} first"
        )
    verify_file_contract(inputs.occupancy, static_map["occupancy"])
    verify_file_contract(inputs.esdf, static_map["esdf"])
    return inputs


def default_output_path(
    repository: Path, environment: dict[str, Any], static_map: dict[str, Any]
) -> Path:
    if "topology" in static_map:
        return repository / static_map["topology"]["path"]
    return (
        repository
        / "external/environment-artifacts/derived"
        / environment["id"]
        / static_map["id"]
        / "world.topology3d"
    )


def compiler_command(
    compiler: Path,
    inputs: StaticMapInputs,
    output: Path,
    profile: dict[str, Any],
    verbose: bool = False,
) -> list[str]:
    command = [
        str(compiler),
        "--occupancy",
        str(inputs.occupancy),
        "--esdf",
        str(inputs.esdf),
        "--output",
        str(output),
    ]
    for name, flag in _PARAMETER_FLAGS:
        command.extend((flag, str(profile[name])))
    if verbose:
        command.append("--verbose")
    return command


def compiled_counts(output: str) -> TopologyCounts:
    matches = _COUNT_PATTERN.findall(output)
    if len(matches) != 1:
        raise TopologyCompilationError("compiler did not emit one topology summary")
    regions, portals, segments = (int(value) for value in matches[0])
    return TopologyCounts(regions=regions, portals=portals, segments=segments)


def expected_counts(profile: dict[str, Any]) -> TopologyCounts:
    expected = profile["expected_counts"]
    return TopologyCounts(
        regions=expected["regions"],
        portals=expected["portals"],
        segments=expected["segments"],
    )


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = repository_root(manifest_path)
    environment = find_environment(manifest, args.environment)
    static_map = select_static_map(environment, args.static_map)
    install_root = (
        args.asset_install_root
        if args.asset_install_root.is_absolute()
        else repository / args.asset_install_root
    )
    inputs = resolve_static_map_inputs(
        repository, environment, static_map, install_root.resolve()
    )
    output = args.output or default_output_path(repository, environment, static_map)
    if not output.is_absolute():
        output = repository / output
    output.parent.mkdir(parents=True, exist_ok=True)
    compiler = args.compiler if args.compiler.is_absolute() else repository / args.compiler
    if not compiler.is_file():
        raise TopologyCompilationError(
            f"topology compiler is missing: {compiler}; run ./scripts/build.sh"
        )
    profile = static_map["topology_compilation"]
    completed = subprocess.run(
        compiler_command(compiler, inputs, output, profile, args.verbose),
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    sys.stdout.write(completed.stdout)
    sys.stderr.write(completed.stderr)
    if completed.returncode != 0:
        raise TopologyCompilationError(
            f"topology compiler exited with status {completed.returncode}"
        )
    actual = compiled_counts(completed.stdout)
    expected = expected_counts(profile)
    if actual != expected and not args.allow_count_drift:
        output.unlink(missing_ok=True)
        raise TopologyCompilationError(
            f"topology count drift: expected={expected} actual={actual}"
        )
    print(
        "ENVIRONMENT_TOPOLOGY_VERIFIED"
        f" environment={environment['id']} static_map={static_map['id']}"
        f" output={output} regions={actual.regions} portals={actual.portals}"
        f" segments={actual.segments} count_drift={str(actual != expected).lower()}"
    )


if __name__ == "__main__":
    try:
        main()
    except (ManifestError, TopologyCompilationError, OSError) as error:
        raise SystemExit(f"environment topology error: {error}") from error
