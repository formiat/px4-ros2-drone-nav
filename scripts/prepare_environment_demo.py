#!/usr/bin/env python3
"""Materialize a local, GUI-only spectator world for one environment catalog entry."""

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

from environment_manifest import ManifestError, find_environment, load_manifest
from prepare_environment_simulation import (
    DEFAULT_INSTALL_ROOT,
    DEFAULT_MANIFEST,
    EnvironmentPreparationError,
    configure_gui_lighting,
    ensure_release_artifact,
    ensure_versioned_visual_resources,
    repository_path,
    source_model_paths,
)
from sdf_collision_materializer import (
    CollisionWorldMaterializer,
    MaterializationError,
    ResourceResolver,
    validate_visual_resource_uris,
    write_materialized_world,
    write_report,
)


DEFAULT_CATALOG = Path("environments/environment_demo_catalog.json")
DEFAULT_DERIVED_ROOT = Path("external/environment-artifacts/derived")
_CATALOG_SCHEMA = "drone_city_nav_environment_demo_catalog_v1"


class DemoPreparationError(RuntimeError):
    """Raised when a spectator world cannot be prepared deterministically."""


@dataclass(frozen=True)
class LocalWorldSource:
    world: Path
    fuel_caches: tuple[Path, ...]
    model_paths: tuple[Path, ...]
    resource_paths: tuple[Path, ...] = ()
    downloaded_visuals: int = 0
    visual_resources: int = 0


@dataclass(frozen=True)
class DemoEnvironment:
    environment_id: str
    display_name: str
    source_kind: str
    source_world: str | None = None
    fuel_caches: tuple[str, ...] = ()
    model_paths: tuple[str, ...] = ()
    resource_paths: tuple[str, ...] = ()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--asset-install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--derived-root", type=Path, default=DEFAULT_DERIVED_ROOT)
    return parser.parse_args()


def _identifier(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise DemoPreparationError(f"{label} must be a non-empty string")
    if not value.replace("_", "a").isalnum():
        raise DemoPreparationError(f"{label} must use letters, digits, and underscores")
    return value


def _string_sequence(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or any(not isinstance(item, str) or not item for item in value):
        raise DemoPreparationError(f"{label} must be an array of non-empty strings")
    return tuple(value)


def load_demo_catalog(path: Path) -> dict[str, DemoEnvironment]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise DemoPreparationError(f"cannot load demo catalog {path}: {error}") from error
    if not isinstance(raw, dict) or raw.get("schema") != _CATALOG_SCHEMA:
        raise DemoPreparationError("unsupported environment demo catalog schema")
    entries = raw.get("environments")
    if not isinstance(entries, list) or not entries:
        raise DemoPreparationError("demo catalog must contain environments")

    result: dict[str, DemoEnvironment] = {}
    for index, entry in enumerate(entries):
        label = f"environments[{index}]"
        if not isinstance(entry, dict):
            raise DemoPreparationError(f"{label} must be an object")
        environment_id = _identifier(entry.get("id"), f"{label}.id")
        if environment_id in result:
            raise DemoPreparationError(f"duplicate demo environment id: {environment_id}")
        display_name = entry.get("display_name")
        if not isinstance(display_name, str) or not display_name:
            raise DemoPreparationError(f"{label}.display_name must be a non-empty string")
        source_kind = entry.get("source_kind")
        if source_kind == "release":
            if any(
                key in entry
                for key in ("source_world", "fuel_caches", "model_paths", "resource_paths")
            ):
                raise DemoPreparationError(
                    f"release demo environment {environment_id} must use the manifest source"
                )
            result[environment_id] = DemoEnvironment(
                environment_id=environment_id,
                display_name=display_name,
                source_kind=source_kind,
            )
            continue
        if source_kind != "local":
            raise DemoPreparationError(
                f"{label}.source_kind must be 'release' or 'local'"
            )
        source_world = entry.get("source_world")
        if not isinstance(source_world, str) or not source_world:
            raise DemoPreparationError(
                f"local demo environment {environment_id} needs source_world"
            )
        result[environment_id] = DemoEnvironment(
            environment_id=environment_id,
            display_name=display_name,
            source_kind=source_kind,
            source_world=source_world,
            fuel_caches=_string_sequence(entry.get("fuel_caches"), f"{label}.fuel_caches"),
            model_paths=_string_sequence(entry.get("model_paths"), f"{label}.model_paths"),
            resource_paths=_string_sequence(
                entry.get("resource_paths", []), f"{label}.resource_paths"
            ),
        )
    return result


def _repository_relative_path(repository: Path, raw_path: str, label: str) -> Path:
    candidate = (repository / raw_path).resolve()
    try:
        candidate.relative_to(repository.resolve())
    except ValueError as error:
        raise DemoPreparationError(f"{label} must stay within the repository") from error
    return candidate


def _locate_release_source_root(artifact_root: Path, entrypoint: str) -> Path:
    for candidate in (artifact_root, artifact_root / "source"):
        if (candidate / entrypoint).is_file():
            return candidate
    raise DemoPreparationError(
        f"release source has no world entrypoint {entrypoint}: {artifact_root}"
    )


def resolve_release_source(
    repository: Path,
    manifest_path: Path,
    install_root: Path,
    environment_id: str,
) -> LocalWorldSource:
    manifest = load_manifest(manifest_path)
    environment = find_environment(manifest, environment_id)
    if environment["distribution"] != "release":
        raise DemoPreparationError(
            f"demo catalog declares release source for non-release environment {environment_id}"
        )
    artifact_root = ensure_release_artifact(
        repository,
        manifest_path,
        environment,
        environment["source"]["bundle_artifact_id"],
        install_root,
    )
    source_root = _locate_release_source_root(
        artifact_root, environment["source"]["entrypoint"]
    )
    visual_cache, downloaded_visuals, visual_resources = ensure_versioned_visual_resources(
        source_root,
        install_root / environment_id / "visual_resources",
        environment["source"].get("visual_dependencies", []),
    )
    return LocalWorldSource(
        world=source_root / environment["source"]["entrypoint"],
        fuel_caches=(source_root / "fuel", visual_cache),
        model_paths=tuple(source_model_paths(source_root)),
        downloaded_visuals=downloaded_visuals,
        visual_resources=len(visual_resources),
    )


def resolve_local_source(repository: Path, environment: DemoEnvironment) -> LocalWorldSource:
    assert environment.source_world is not None
    world = _repository_relative_path(
        repository, environment.source_world, f"{environment.environment_id}.source_world"
    )
    if not world.is_file():
        raise DemoPreparationError(
            f"local demo source is not available: {world}. "
            "Fetch or restore the candidate assets before launching this demo."
        )
    fuel_caches = tuple(
        _repository_relative_path(
            repository, value, f"{environment.environment_id}.fuel_caches"
        )
        for value in environment.fuel_caches
    )
    model_paths = tuple(
        _repository_relative_path(
            repository, value, f"{environment.environment_id}.model_paths"
        )
        for value in environment.model_paths
    )
    resource_paths = tuple(
        _repository_relative_path(
            repository, value, f"{environment.environment_id}.resource_paths"
        )
        for value in environment.resource_paths
    )
    missing = [
        path for path in (*fuel_caches, *model_paths, *resource_paths) if not path.is_dir()
    ]
    if missing:
        raise DemoPreparationError(
            "local demo resources are not available: "
            + ", ".join(str(path) for path in missing)
        )
    return LocalWorldSource(
        world=world,
        fuel_caches=fuel_caches,
        model_paths=model_paths,
        resource_paths=resource_paths,
    )


def write_demo_environment(
    output: Path,
    repository: Path,
    world_name: str,
    world_sdf: Path,
    resource_paths: tuple[Path, ...],
) -> None:
    values = {
        "SIM_DEMO_WORLD_NAME": world_name,
        "SIM_DEMO_WORLD_SDF_PATH": repository_path(repository, world_sdf),
        "SIM_DEMO_RESOURCE_PATH": ":".join(
            repository_path(repository, path) for path in resource_paths
        ),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "".join(f"export {name}={shlex.quote(value)}\n" for name, value in values.items()),
        encoding="utf-8",
    )


def prepare_demo(
    repository: Path,
    environment: DemoEnvironment,
    manifest_path: Path,
    install_root: Path,
    derived_root: Path,
) -> Path:
    source = (
        resolve_release_source(
            repository, manifest_path, install_root, environment.environment_id
        )
        if environment.source_kind == "release"
        else resolve_local_source(repository, environment)
    )
    runtime_root = derived_root / environment.environment_id / "demo"
    shutil.rmtree(runtime_root / "assets", ignore_errors=True)
    runtime_root.mkdir(parents=True, exist_ok=True)
    world_sdf = runtime_root / "world_gui.sdf"
    report_path = runtime_root / "materialization_gui.json"
    tree, report = CollisionWorldMaterializer(
        ResourceResolver(source.fuel_caches, source.model_paths, source.resource_paths),
        preserve_visuals=True,
        localized_mesh_root=runtime_root / "assets" / "meshes",
    ).materialize(source.world)
    report.light_instances = configure_gui_lighting(tree)
    fingerprint = write_materialized_world(tree, world_sdf)
    visual_uris = validate_visual_resource_uris(world_sdf)
    write_report(report, world_sdf, fingerprint, report_path)
    world = ET.parse(world_sdf).getroot().find("world")
    if world is None or not world.attrib.get("name"):
        raise DemoPreparationError("materialized demo world has no name")
    environment_file = runtime_root / "environment.env"
    write_demo_environment(
        environment_file,
        repository,
        world.attrib["name"],
        world_sdf,
        tuple(
            path
            for path in (*source.fuel_caches, *source.model_paths, *source.resource_paths)
            if path.is_dir()
        ),
    )
    print(
        "ENVIRONMENT_DEMO_READY"
        f" environment={environment.environment_id}"
        f" display_name={environment.display_name!r}"
        f" world={world.attrib['name']}"
        f" collisions={report.collision_instances} visuals={report.visual_instances}"
        f" lights={report.light_instances} visual_uris={visual_uris}"
        f" visual_resources_verified={source.visual_resources}"
        f" visual_resources_downloaded={source.downloaded_visuals}"
        f" env_file={environment_file}"
    )
    return environment_file


def main() -> None:
    args = parse_args()
    catalog_path = args.catalog.resolve()
    manifest_path = args.manifest.resolve()
    catalog = load_demo_catalog(catalog_path)
    try:
        environment = catalog[args.environment]
    except KeyError as error:
        available = ", ".join(sorted(catalog))
        raise DemoPreparationError(
            f"unknown demo environment {args.environment!r}; available: {available}"
        ) from error
    repository = manifest_path.parent.parent
    install_root = args.asset_install_root
    if not install_root.is_absolute():
        install_root = repository / install_root
    derived_root = args.derived_root
    if not derived_root.is_absolute():
        derived_root = repository / derived_root
    prepare_demo(
        repository.resolve(),
        environment,
        manifest_path,
        install_root.resolve(),
        derived_root.resolve(),
    )


if __name__ == "__main__":
    try:
        main()
    except (
        DemoPreparationError,
        EnvironmentPreparationError,
        ManifestError,
        MaterializationError,
        OSError,
    ) as error:
        raise SystemExit(f"environment demo preparation error: {error}") from error
