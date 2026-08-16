#!/usr/bin/env python3
"""Prepare one distributed environment for the shared Gazebo simulation runtime."""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

from compile_environment_topology import (
    default_output_path,
    resolve_static_map_inputs,
    select_static_map,
)
from environment_manifest import (
    ManifestError,
    find_environment,
    load_manifest,
    repository_root,
)
from sdf_collision_materializer import (
    CollisionWorldMaterializer,
    MaterializationError,
    ResourceResolver,
    write_materialized_world,
    write_report,
)


DEFAULT_MANIFEST = Path("environments/environment_manifest.yaml")
DEFAULT_INSTALL_ROOT = Path("external/environment-artifacts/installed")


class EnvironmentPreparationError(RuntimeError):
    """Raised when an environment cannot satisfy the simulation contract."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--static-map")
    parser.add_argument("--asset-install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--rebuild-topology", action="store_true")
    return parser.parse_args()


def ensure_release_artifact(
    repository: Path,
    manifest_path: Path,
    environment: dict,
    artifact_id: str,
    install_root: Path,
) -> Path:
    destination = install_root / environment["id"] / artifact_id
    if destination.is_dir():
        return destination
    completed = subprocess.run(
        [
            sys.executable,
            str(repository / "scripts/manage_environment_assets.py"),
            "--manifest",
            str(manifest_path),
            "fetch",
            "--environment",
            environment["id"],
            "--artifact",
            artifact_id,
            "--install-dir",
            str(install_root),
        ],
        cwd=repository,
        check=False,
    )
    if completed.returncode != 0 or not destination.is_dir():
        raise EnvironmentPreparationError(
            f"failed to install {environment['id']} artifact {artifact_id}"
        )
    return destination


def source_model_paths(source_root: Path) -> list[Path]:
    paths = sorted(source_root.glob("fuel/*/*/models"))
    return [path for path in paths if path.is_dir()]


def repository_path(repository: Path, path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(repository).as_posix()
    except ValueError:
        return str(resolved)


def write_runtime_environment(
    path: Path,
    repository: Path,
    world_name: str,
    world_sdf: Path,
    source_root: Path,
    occupancy: Path,
    esdf: Path,
    topology: Path,
) -> None:
    values = {
        "SIM_WORLD_NAME": world_name,
        "SIM_WORLD_SDF_PATH": repository_path(repository, world_sdf),
        "SIM_WORLD_RESOURCE_PATH": repository_path(repository, source_root / "fuel"),
        "STATIC_OCCUPANCY_3D_PATH": repository_path(repository, occupancy),
        "STATIC_ESDF_3D_CACHE_PATH": repository_path(repository, esdf),
        "STATIC_FREE_SPACE_TOPOLOGY_3D_PATH": repository_path(repository, topology),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(
            f"export {name}={shlex.quote(value)}\n" for name, value in values.items()
        ),
        encoding="utf-8",
    )


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = repository_root(manifest_path)
    environment = find_environment(manifest, args.environment)
    if environment["distribution"] != "release":
        raise EnvironmentPreparationError(
            "simulation preparation currently expects a release environment"
        )
    install_root = args.asset_install_root
    if not install_root.is_absolute():
        install_root = repository / install_root
    install_root = install_root.resolve()

    source_artifact_id = environment["source"]["bundle_artifact_id"]
    source_root = ensure_release_artifact(
        repository, manifest_path, environment, source_artifact_id, install_root
    )
    static_map = select_static_map(environment, args.static_map)
    ensure_release_artifact(
        repository,
        manifest_path,
        environment,
        static_map["artifact_id"],
        install_root,
    )
    inputs = resolve_static_map_inputs(
        repository, environment, static_map, install_root
    )

    topology = default_output_path(repository, environment, static_map)
    if args.rebuild_topology or not topology.is_file():
        completed = subprocess.run(
            [
                sys.executable,
                str(repository / "scripts/compile_environment_topology.py"),
                "--manifest",
                str(manifest_path),
                "--environment",
                environment["id"],
                "--static-map",
                static_map["id"],
                "--asset-install-root",
                str(install_root),
                "--output",
                str(topology),
            ],
            cwd=repository,
            check=False,
        )
        if completed.returncode != 0:
            raise EnvironmentPreparationError("topology compilation failed")
    if topology.stat().st_size == 0:
        raise EnvironmentPreparationError("compiled topology is empty")

    source_world = source_root / environment["source"]["entrypoint"]
    if not source_world.is_file():
        raise EnvironmentPreparationError(f"source world is missing: {source_world}")
    runtime_root = (
        repository
        / "external/environment-artifacts/derived"
        / environment["id"]
        / "runtime"
    )
    world_sdf = runtime_root / "world.sdf"
    report_path = runtime_root / "materialization.json"
    materializer = CollisionWorldMaterializer(
        ResourceResolver([source_root / "fuel"], source_model_paths(source_root)),
        preview_visuals=True,
    )
    tree, report = materializer.materialize(source_world)
    fingerprint = write_materialized_world(tree, world_sdf)
    write_report(report, world_sdf, fingerprint, report_path)
    world = ET.parse(world_sdf).getroot().find("world")
    if world is None or not world.attrib.get("name"):
        raise EnvironmentPreparationError("materialized SDF has no named world")
    world_name = world.attrib["name"]
    environment_file = runtime_root / "environment.env"
    write_runtime_environment(
        environment_file,
        repository,
        world_name,
        world_sdf,
        source_root,
        inputs.occupancy,
        inputs.esdf,
        topology,
    )
    print(
        "ENVIRONMENT_SIMULATION_READY"
        f" environment={environment['id']} static_map={static_map['id']}"
        f" world={world_name} collisions={report.collision_instances}"
        f" occupancy={inputs.occupancy} esdf={inputs.esdf} topology={topology}"
        f" env_file={environment_file}"
    )


if __name__ == "__main__":
    try:
        main()
    except (
        EnvironmentPreparationError,
        ManifestError,
        MaterializationError,
        OSError,
    ) as error:
        raise SystemExit(f"environment preparation error: {error}") from error
