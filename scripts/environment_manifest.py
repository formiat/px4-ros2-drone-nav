"""Typed access to the external environment artifact manifest."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any, Iterator

import yaml


class ManifestError(RuntimeError):
    """Raised when the environment distribution contract is invalid."""


_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_CLASSIFICATIONS = {"confirmed_fit", "test_fixture"}
_DISTRIBUTIONS = {"release", "repository"}
_ARTIFACT_KINDS = {"source_bundle", "static_map_bundle"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError(f"cannot load manifest {path}: {exc}") from exc
    manifest = _mapping(data, "manifest")
    validate_manifest(manifest)
    return manifest


def write_manifest(path: Path, manifest: dict[str, Any]) -> None:
    validate_manifest(manifest)
    path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=False),
        encoding="utf-8",
    )


def validate_manifest(manifest: dict[str, Any]) -> None:
    if _string(manifest.get("schema"), "schema") != "drone_city_nav_environment_manifest_v1":
        raise ManifestError("unsupported environment manifest schema")

    release = _mapping(manifest.get("artifact_release"), "artifact_release")
    _string(release.get("tag"), "artifact_release.tag")
    if not isinstance(release.get("published"), bool):
        raise ManifestError("artifact_release.published must be a boolean")
    base_urls = _sequence(release.get("base_urls"), "artifact_release.base_urls")
    if not base_urls:
        raise ManifestError("artifact_release.base_urls must not be empty")
    for index, url in enumerate(base_urls):
        value = _string(url, f"artifact_release.base_urls[{index}]")
        if not value.startswith(("https://", "http://", "file://")):
            raise ManifestError(f"unsupported artifact base URL: {value}")
    _relative_path(release.get("local_mirror"), "artifact_release.local_mirror")

    environments = _sequence(manifest.get("environments"), "environments")
    if not environments:
        raise ManifestError("environments must not be empty")
    environment_ids: set[str] = set()
    release_count = 0
    repository_count = 0
    for index, raw_environment in enumerate(environments):
        environment = _mapping(raw_environment, f"environments[{index}]")
        environment_id = _identifier(
            environment.get("id"), f"environments[{index}].id"
        )
        if environment_id in environment_ids:
            raise ManifestError(f"duplicate environment id: {environment_id}")
        environment_ids.add(environment_id)
        _string(environment.get("display_name"), f"{environment_id}.display_name")
        _string(environment.get("role"), f"{environment_id}.role")
        classification = _string(
            environment.get("classification"), f"{environment_id}.classification"
        )
        if classification not in _CLASSIFICATIONS:
            raise ManifestError(
                f"unsupported classification for {environment_id}: {classification}"
            )
        distribution = _string(
            environment.get("distribution"), f"{environment_id}.distribution"
        )
        if distribution not in _DISTRIBUTIONS:
            raise ManifestError(
                f"unsupported distribution for {environment_id}: {distribution}"
            )
        _validate_source(environment_id, environment.get("source"))
        _validate_static_maps(environment_id, environment.get("static_maps"))
        if distribution == "release":
            release_count += 1
            _validate_release_environment(environment_id, environment)
        else:
            repository_count += 1
            _validate_repository_environment(environment_id, environment)
    if release_count != 2 or repository_count != 1:
        raise ManifestError(
            "distribution contract must contain two release environments and one fixture"
        )


def iter_release_artifacts(
    manifest: dict[str, Any], environment_id: str | None = None
) -> Iterator[tuple[dict[str, Any], dict[str, Any]]]:
    for environment in manifest["environments"]:
        if environment["distribution"] != "release":
            continue
        if environment_id is not None and environment["id"] != environment_id:
            continue
        for artifact in environment["artifacts"]:
            yield environment, artifact


def find_environment(manifest: dict[str, Any], environment_id: str) -> dict[str, Any]:
    for environment in manifest["environments"]:
        if environment["id"] == environment_id:
            return environment
    raise ManifestError(f"unknown environment id: {environment_id}")


def repository_root(manifest_path: Path) -> Path:
    resolved = manifest_path.resolve()
    if resolved.parent.name != "environments":
        raise ManifestError("manifest must be stored in the repository environments directory")
    return resolved.parent.parent


def _validate_source(environment_id: str, raw_source: Any) -> None:
    source = _mapping(raw_source, f"{environment_id}.source")
    _string(source.get("provider"), f"{environment_id}.source.provider")
    _string(source.get("url"), f"{environment_id}.source.url")
    version = source.get("version")
    if not isinstance(version, (int, str)) or isinstance(version, bool):
        raise ManifestError(f"{environment_id}.source.version must be an integer or string")
    license_data = _mapping(source.get("license"), f"{environment_id}.source.license")
    _string(license_data.get("spdx"), f"{environment_id}.source.license.spdx")
    _string(license_data.get("url"), f"{environment_id}.source.license.url")
    _string(
        license_data.get("attribution"),
        f"{environment_id}.source.license.attribution",
    )
    if source["provider"] == "gazebo_fuel":
        if not _positive_int(version):
            raise ManifestError(
                f"{environment_id}.source.version must be a positive Fuel version"
            )
        inventory = _mapping(
            source.get("license_inventory"),
            f"{environment_id}.source.license_inventory",
        )
        _relative_path(
            inventory.get("path"),
            f"{environment_id}.source.license_inventory.path",
        )
        _sha256(
            inventory.get("sha256"),
            f"{environment_id}.source.license_inventory.sha256",
        )
        if not _positive_int(inventory.get("size_bytes")):
            raise ManifestError(
                f"{environment_id}.source.license_inventory.size_bytes must be positive"
            )


def _validate_static_maps(environment_id: str, raw_maps: Any) -> None:
    maps = _sequence(raw_maps, f"{environment_id}.static_maps")
    if not maps:
        raise ManifestError(f"{environment_id}.static_maps must not be empty")
    map_ids: set[str] = set()
    for index, raw_map in enumerate(maps):
        static_map = _mapping(raw_map, f"{environment_id}.static_maps[{index}]")
        map_id = _identifier(static_map.get("id"), f"{environment_id}.static_maps.id")
        if map_id in map_ids:
            raise ManifestError(f"duplicate static map id for {environment_id}: {map_id}")
        map_ids.add(map_id)
        resolution = static_map.get("resolution_m")
        if not isinstance(resolution, (int, float)) or isinstance(resolution, bool):
            raise ManifestError(f"{environment_id}.{map_id}.resolution_m must be numeric")
        if float(resolution) <= 0.0:
            raise ManifestError(f"{environment_id}.{map_id}.resolution_m must be positive")
        origin = _sequence(static_map.get("origin_m"), f"{environment_id}.{map_id}.origin_m")
        dimensions = _sequence(
            static_map.get("dimensions"), f"{environment_id}.{map_id}.dimensions"
        )
        if len(origin) != 3 or not all(_number(value) for value in origin):
            raise ManifestError(f"{environment_id}.{map_id}.origin_m must have three numbers")
        if len(dimensions) != 3 or not all(_positive_int(value) for value in dimensions):
            raise ManifestError(
                f"{environment_id}.{map_id}.dimensions must have three positive integers"
            )
        for field in (
            "dense_voxel_count",
            "collision_triangles",
            "occupied_voxels",
            "occupied_chunks",
        ):
            if not _nonnegative_int(static_map.get(field)):
                raise ManifestError(f"{environment_id}.{map_id}.{field} must be non-negative")
        dense_voxel_count = dimensions[0] * dimensions[1] * dimensions[2]
        if static_map["dense_voxel_count"] != dense_voxel_count:
            raise ManifestError(
                f"{environment_id}.{map_id}.dense_voxel_count does not match dimensions"
            )
        maximum_distance = static_map.get("maximum_distance_m")
        if not _number(maximum_distance) or float(maximum_distance) <= 0.0:
            raise ManifestError(
                f"{environment_id}.{map_id}.maximum_distance_m must be positive"
            )
        for role in ("occupancy", "esdf"):
            file_data = _mapping(
                static_map.get(role), f"{environment_id}.{map_id}.{role}"
            )
            _relative_path(file_data.get("path"), f"{environment_id}.{map_id}.{role}.path")
            _sha256(file_data.get("sha256"), f"{environment_id}.{map_id}.{role}.sha256")
            if not _positive_int(file_data.get("size_bytes")):
                raise ManifestError(
                    f"{environment_id}.{map_id}.{role}.size_bytes must be positive"
                )


def _validate_release_environment(
    environment_id: str, environment: dict[str, Any]
) -> None:
    artifacts = _sequence(environment.get("artifacts"), f"{environment_id}.artifacts")
    artifact_ids: set[str] = set()
    artifact_kinds: dict[str, str] = {}
    kinds: set[str] = set()
    for index, raw_artifact in enumerate(artifacts):
        artifact = _mapping(raw_artifact, f"{environment_id}.artifacts[{index}]")
        artifact_id = _identifier(
            artifact.get("id"), f"{environment_id}.artifacts.id"
        )
        if artifact_id in artifact_ids:
            raise ManifestError(f"duplicate artifact id for {environment_id}: {artifact_id}")
        artifact_ids.add(artifact_id)
        kind = _string(artifact.get("kind"), f"{environment_id}.{artifact_id}.kind")
        if kind not in _ARTIFACT_KINDS:
            raise ManifestError(f"unsupported artifact kind: {kind}")
        kinds.add(kind)
        artifact_kinds[artifact_id] = kind
        filename = _string(
            artifact.get("filename"), f"{environment_id}.{artifact_id}.filename"
        )
        if Path(filename).name != filename or not filename.endswith(".tar.gz"):
            raise ManifestError(f"artifact filename must be a plain .tar.gz name: {filename}")
        _sha256(artifact.get("sha256"), f"{environment_id}.{artifact_id}.sha256")
        if not _positive_int(artifact.get("size_bytes")):
            raise ManifestError(f"{environment_id}.{artifact_id}.size_bytes must be positive")
        build = _mapping(artifact.get("build"), f"{environment_id}.{artifact_id}.build")
        if kind == "source_bundle":
            _relative_path(build.get("world"), f"{environment_id}.{artifact_id}.build.world")
            _relative_path(
                build.get("materialization_report"),
                f"{environment_id}.{artifact_id}.build.materialization_report",
            )
        else:
            files = _sequence(
                build.get("files"), f"{environment_id}.{artifact_id}.build.files"
            )
            if not files:
                raise ManifestError(f"{environment_id}.{artifact_id}.build.files is empty")
            for file_index, raw_file in enumerate(files):
                file_data = _mapping(raw_file, f"{environment_id}.{artifact_id}.build.files")
                _relative_path(
                    file_data.get("source"),
                    f"{environment_id}.{artifact_id}.build.files[{file_index}].source",
                )
                _relative_path(
                    file_data.get("destination"),
                    f"{environment_id}.{artifact_id}.build.files[{file_index}].destination",
                )
    if kinds != _ARTIFACT_KINDS or len(artifacts) != len(_ARTIFACT_KINDS):
        raise ManifestError(
            f"{environment_id} must have one source and one static-map bundle"
        )
    source = environment["source"]
    source_artifact_id = _identifier(
        source.get("bundle_artifact_id"),
        f"{environment_id}.source.bundle_artifact_id",
    )
    if artifact_kinds.get(source_artifact_id) != "source_bundle":
        raise ManifestError(f"{environment_id} source does not reference a source bundle")
    _relative_path(source.get("entrypoint"), f"{environment_id}.source.entrypoint")
    for static_map in environment["static_maps"]:
        artifact_id = _identifier(
            static_map.get("artifact_id"),
            f"{environment_id}.{static_map['id']}.artifact_id",
        )
        if artifact_id not in artifact_ids:
            raise ManifestError(
                f"{environment_id}.{static_map['id']} references unknown artifact {artifact_id}"
            )
        if artifact_kinds[artifact_id] != "static_map_bundle":
            raise ManifestError(
                f"{environment_id}.{static_map['id']} does not reference a map bundle"
            )


def _validate_repository_environment(
    environment_id: str, environment: dict[str, Any]
) -> None:
    files = _sequence(
        environment.get("repository_files"), f"{environment_id}.repository_files"
    )
    paths: set[str] = set()
    for index, raw_file in enumerate(files):
        file_data = _mapping(raw_file, f"{environment_id}.repository_files[{index}]")
        path = _relative_path(
            file_data.get("path"), f"{environment_id}.repository_files[{index}].path"
        )
        if path in paths:
            raise ManifestError(f"duplicate repository file for {environment_id}: {path}")
        paths.add(path)
        _sha256(
            file_data.get("sha256"),
            f"{environment_id}.repository_files[{index}].sha256",
        )
        if not _positive_int(file_data.get("size_bytes")):
            raise ManifestError(
                f"{environment_id}.repository_files[{index}].size_bytes must be positive"
            )
    for static_map in environment["static_maps"]:
        for role in ("occupancy", "esdf"):
            if static_map[role]["path"] not in paths:
                raise ManifestError(
                    f"{environment_id}.{static_map['id']}.{role} is not a repository file"
                )


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ManifestError(f"{field} must be a mapping")
    return value


def _sequence(value: Any, field: str) -> list[Any]:
    if not isinstance(value, list):
        raise ManifestError(f"{field} must be a sequence")
    return value


def _string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{field} must be a non-empty string")
    return value


def _identifier(value: Any, field: str) -> str:
    text = _string(value, field)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", text):
        raise ManifestError(f"{field} is not a valid identifier: {text}")
    return text


def _relative_path(value: Any, field: str) -> str:
    text = _string(value, field)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts:
        raise ManifestError(f"{field} must be a safe relative path")
    return text


def _sha256(value: Any, field: str) -> str:
    text = _string(value, field)
    if _SHA256_PATTERN.fullmatch(text) is None:
        raise ManifestError(f"{field} must be a lowercase SHA-256 digest")
    return text


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _nonnegative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
