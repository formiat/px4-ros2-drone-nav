"""Build, verify, download, and install versioned environment artifacts."""

from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
import shutil
import tarfile
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Any, Iterable
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import urlopen

from environment_manifest import artifact_release_for_environment, sha256_file
from fuel_license_inventory import summarize_licenses


class ArtifactError(RuntimeError):
    """Raised when an environment artifact cannot be reproduced safely."""


def build_release_artifacts(
    manifest: dict[str, Any],
    repository_root: Path,
    candidate_root: Path,
    release_directory_override: Path | None = None,
    selected_environment_ids: set[str] | None = None,
) -> list[Path]:
    repository_root = repository_root.resolve()
    candidate_root = candidate_root.resolve()
    environments = _selected_release_environments(
        manifest, selected_environment_ids
    )
    release_ids = {
        environment["artifact_release_id"] for environment in environments
    }
    if release_directory_override is not None and len(release_ids) != 1:
        raise ArtifactError(
            "--release-dir requires environments from exactly one artifact release"
        )
    override = (
        release_directory_override.resolve()
        if release_directory_override is not None
        else None
    )
    built: list[Path] = []
    touched_releases: dict[str, Path] = {}
    for environment in environments:
        release_directory = override or _release_directory(
            manifest, environment, repository_root
        )
        release_directory.mkdir(parents=True, exist_ok=True)
        touched_releases[environment["artifact_release_id"]] = release_directory
        for artifact in environment["artifacts"]:
            output = release_directory / artifact["filename"]
            if artifact["kind"] == "source_bundle":
                _build_source_bundle(
                    environment, artifact, repository_root, candidate_root, output
                )
            elif artifact["kind"] == "static_map_bundle":
                _build_static_map_bundle(
                    environment, artifact, repository_root, candidate_root, output
                )
                _update_static_map_file_contracts(environment, artifact, candidate_root)
            else:
                raise ArtifactError(f"unsupported artifact kind: {artifact['kind']}")
            artifact["sha256"] = sha256_file(output)
            artifact["size_bytes"] = output.stat().st_size
            verify_bundle_contents(output, environment["id"], artifact["id"])
            built.append(output)
    for release_id, release_directory in touched_releases.items():
        write_release_checksums(manifest, release_directory, release_id)
    return built


def verify_repository_files(
    manifest: dict[str, Any], manifest_path: Path
) -> list[Path]:
    repository = manifest_path.resolve().parent.parent
    verified: list[Path] = []
    for environment in manifest["environments"]:
        inventory = environment["source"].get("license_inventory")
        if inventory is not None:
            path = repository / inventory["path"]
            verify_file_contract(path, inventory)
            verified.append(path)
        if environment["distribution"] != "repository":
            continue
        for file_contract in environment["repository_files"]:
            path = repository / file_contract["path"]
            verify_file_contract(path, file_contract)
            verified.append(path)
    return verified


def update_repository_file_contracts(
    manifest: dict[str, Any], manifest_path: Path
) -> list[Path]:
    repository = manifest_path.resolve().parent.parent
    updated: list[Path] = []
    for environment in manifest["environments"]:
        inventory = environment["source"].get("license_inventory")
        if inventory is not None:
            path = repository / inventory["path"]
            if not path.is_file():
                raise ArtifactError(f"license inventory is missing: {path}")
            inventory["sha256"] = sha256_file(path)
            inventory["size_bytes"] = path.stat().st_size
            updated.append(path)
        if environment["distribution"] != "repository":
            continue
        contracts_by_path = {
            contract["path"]: contract
            for contract in environment["repository_files"]
        }
        for path_text, contract in contracts_by_path.items():
            path = repository / path_text
            if not path.is_file():
                raise ArtifactError(f"repository environment file is missing: {path}")
            contract["sha256"] = sha256_file(path)
            contract["size_bytes"] = path.stat().st_size
            updated.append(path)
        for static_map in environment["static_maps"]:
            for role in ("occupancy", "esdf"):
                file_contract = static_map[role]
                repository_contract = contracts_by_path[file_contract["path"]]
                file_contract["sha256"] = repository_contract["sha256"]
                file_contract["size_bytes"] = repository_contract["size_bytes"]
    return updated


def verify_release_artifacts(
    manifest: dict[str, Any],
    repository_root: Path,
    environment_id: str | None = None,
    inspect_contents: bool = True,
    release_directory_override: Path | None = None,
) -> list[Path]:
    environments = _selected_release_environments(
        manifest, {environment_id} if environment_id is not None else None
    )
    release_ids = {
        environment["artifact_release_id"] for environment in environments
    }
    if release_directory_override is not None and len(release_ids) != 1:
        raise ArtifactError(
            "--release-dir requires --environment when multiple releases exist"
        )
    verified: list[Path] = []
    for environment in environments:
        release_directory = (
            release_directory_override.resolve()
            if release_directory_override is not None
            else _release_directory(manifest, environment, repository_root)
        )
        for artifact in environment["artifacts"]:
            path = release_directory / artifact["filename"]
            verify_file_contract(path, artifact)
            if inspect_contents:
                verify_bundle_contents(path, environment["id"], artifact["id"])
            verified.append(path)
    return verified


def fetch_artifact(
    manifest: dict[str, Any],
    environment: dict[str, Any],
    artifact: dict[str, Any],
    release_directory: Path,
    cache_directory: Path,
    base_url_override: str | None = None,
) -> Path:
    local_path = release_directory / artifact["filename"]
    if local_path.is_file():
        verify_file_contract(local_path, artifact)
        return local_path

    cache_directory.mkdir(parents=True, exist_ok=True)
    cached_path = cache_directory / artifact["filename"]
    if cached_path.is_file():
        try:
            verify_file_contract(cached_path, artifact)
            return cached_path
        except ArtifactError:
            cached_path.unlink()

    if base_url_override is not None:
        base_urls = [base_url_override]
    else:
        release = artifact_release_for_environment(manifest, environment)
        if not release["published"]:
            raise ArtifactError(
                "artifact release is staged locally but not published; provide "
                "--base-url or restore the local release mirror"
            )
        base_urls = release["base_urls"]

    errors: list[str] = []
    for base_url in base_urls:
        url = f"{base_url.rstrip('/')}/{quote(artifact['filename'])}"
        try:
            _download_verified(url, cached_path, artifact)
            return cached_path
        except ArtifactError as exc:
            errors.append(str(exc))
    raise ArtifactError(
        f"cannot fetch {environment['id']}/{artifact['id']}: " + "; ".join(errors)
    )


def install_artifact(archive: Path, destination: Path) -> Path:
    verify_bundle_contents(archive)
    destination_parent = destination.resolve().parent
    destination_parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise ArtifactError(f"installation destination already exists: {destination}")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.", dir=destination_parent)
    )
    try:
        with tarfile.open(archive, mode="r:gz") as bundle:
            _safe_extract(bundle, temporary)
        os.replace(temporary, destination)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return destination


def write_release_checksums(
    manifest: dict[str, Any], release_directory: Path, release_id: str
) -> Path:
    lines: list[str] = []
    for environment in manifest["environments"]:
        if environment.get("artifact_release_id") != release_id:
            continue
        for artifact in environment.get("artifacts", []):
            verify_file_contract(release_directory / artifact["filename"], artifact)
            lines.append(f"{artifact['sha256']}  {artifact['filename']}")
    if not lines:
        raise ArtifactError(f"artifact release has no files: {release_id}")
    output = release_directory / "SHA256SUMS"
    output.write_text("\n".join(sorted(lines)) + "\n", encoding="ascii")
    return output


def _selected_release_environments(
    manifest: dict[str, Any], selected_environment_ids: set[str] | None
) -> list[dict[str, Any]]:
    release_environments = [
        environment
        for environment in manifest["environments"]
        if environment["distribution"] == "release"
    ]
    if selected_environment_ids is None:
        return release_environments
    known_ids = {environment["id"] for environment in release_environments}
    unknown_ids = selected_environment_ids - known_ids
    if unknown_ids:
        raise ArtifactError(
            "unknown release environment(s): " + ", ".join(sorted(unknown_ids))
        )
    return [
        environment
        for environment in release_environments
        if environment["id"] in selected_environment_ids
    ]


def _release_directory(
    manifest: dict[str, Any], environment: dict[str, Any], repository_root: Path
) -> Path:
    release = artifact_release_for_environment(manifest, environment)
    return (repository_root / release["local_mirror"]).resolve()


def verify_file_contract(path: Path, contract: dict[str, Any]) -> None:
    if not path.is_file():
        raise ArtifactError(f"artifact file is missing: {path}")
    actual_size = path.stat().st_size
    if actual_size != contract["size_bytes"]:
        raise ArtifactError(
            f"size mismatch for {path}: expected {contract['size_bytes']}, got {actual_size}"
        )
    actual_sha256 = sha256_file(path)
    if actual_sha256 != contract["sha256"]:
        raise ArtifactError(
            f"SHA-256 mismatch for {path}: expected {contract['sha256']}, "
            f"got {actual_sha256}"
        )


def verify_bundle_contents(
    path: Path,
    expected_environment_id: str | None = None,
    expected_artifact_id: str | None = None,
) -> None:
    try:
        with tarfile.open(path, mode="r:gz") as bundle:
            members = bundle.getmembers()
            _validate_members(members)
            member_names = [member.name for member in members]
            if len(member_names) != len(set(member_names)):
                raise ArtifactError(f"bundle contains duplicate paths: {path}")
            metadata_members = [
                member
                for member in members
                if member.name == "ARTIFACT_METADATA.json"
                or member.name.endswith("/ARTIFACT_METADATA.json")
            ]
            if len(metadata_members) != 1:
                raise ArtifactError(f"bundle must contain one metadata file: {path}")
            stream = bundle.extractfile(metadata_members[0])
            if stream is None:
                raise ArtifactError(f"cannot read bundle metadata: {path}")
            metadata = json.load(stream)
            if metadata.get("schema") != "drone_city_nav_environment_bundle_v1":
                raise ArtifactError(f"unsupported bundle metadata schema: {path}")
            if (
                expected_environment_id is not None
                and metadata.get("environment_id") != expected_environment_id
            ):
                raise ArtifactError(f"bundle environment identity mismatch: {path}")
            if (
                expected_artifact_id is not None
                and metadata.get("artifact_id") != expected_artifact_id
            ):
                raise ArtifactError(f"bundle artifact identity mismatch: {path}")
            members_by_name = {member.name: member for member in members}
            file_contracts = metadata.get("files", [])
            declared_files = {contract["path"] for contract in file_contracts}
            actual_files = {member.name for member in members if member.isfile()}
            if actual_files != declared_files | {metadata_members[0].name}:
                raise ArtifactError(f"bundle contains undeclared or omitted files: {path}")
            for file_contract in file_contracts:
                member = members_by_name.get(file_contract["path"])
                if member is None or not member.isfile():
                    raise ArtifactError(
                        f"bundle member is missing: {file_contract['path']}"
                    )
                if member.size != file_contract["size_bytes"]:
                    raise ArtifactError(
                        f"bundle member size mismatch: {file_contract['path']}"
                    )
                member_stream = bundle.extractfile(member)
                if member_stream is None:
                    raise ArtifactError(
                        f"cannot read bundle member: {file_contract['path']}"
                    )
                if _sha256_stream(member_stream) != file_contract["sha256"]:
                    raise ArtifactError(
                        f"bundle member hash mismatch: {file_contract['path']}"
                    )
    except (OSError, tarfile.TarError, json.JSONDecodeError) as exc:
        raise ArtifactError(f"cannot verify bundle {path}: {exc}") from exc


def write_deterministic_tar_gz(
    output: Path,
    file_entries: Iterable[tuple[Path, str]],
    byte_entries: Iterable[tuple[bytes, str]],
) -> None:
    normalized_files = sorted(
        ((path.resolve(), _safe_archive_path(name)) for path, name in file_entries),
        key=lambda entry: entry[1],
    )
    normalized_bytes = sorted(
        ((data, _safe_archive_path(name)) for data, name in byte_entries),
        key=lambda entry: entry[1],
    )
    names = [name for _, name in normalized_files] + [
        name for _, name in normalized_bytes
    ]
    if len(names) != len(set(names)):
        raise ArtifactError("archive contains duplicate paths")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    try:
        with temporary.open("wb") as raw_stream:
            with gzip.GzipFile(
                filename="", mode="wb", fileobj=raw_stream, compresslevel=6, mtime=0
            ) as compressed:
                with tarfile.open(
                    fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT
                ) as bundle:
                    for source, name in normalized_files:
                        _add_file(bundle, source, name)
                    for data, name in normalized_bytes:
                        _add_bytes(bundle, data, name)
        os.replace(temporary, output)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def _build_source_bundle(
    environment: dict[str, Any],
    artifact: dict[str, Any],
    repository_root: Path,
    candidate_root: Path,
    output: Path,
) -> None:
    build = artifact["build"]
    world = candidate_root / build["world"]
    report_path = candidate_root / build["materialization_report"]
    if not world.is_file() or not report_path.is_file():
        raise ArtifactError(f"source build inputs are missing for {environment['id']}")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    roots = {world.parent}
    for recorded_model in report.get("model_files", []):
        roots.add(_resolve_recorded_path(recorded_model, candidate_root).parent)

    files: dict[str, Path] = {}
    for root in roots:
        _require_within(root, candidate_root)
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            relative = path.relative_to(candidate_root).as_posix()
            archive_path = relative
            files[archive_path] = path

    normalized_report = _normalize_recorded_paths(report, candidate_root)
    report_bytes = _json_bytes(normalized_report)
    inventory_bytes, inventory = _license_inventory(environment, repository_root)
    attribution = _attribution_bytes(environment, inventory, modified=False)
    virtual_files = {
        "reports/materialization.json": report_bytes,
        "ATTRIBUTION.md": attribution,
        "LICENSES.json": inventory_bytes,
    }
    metadata = _bundle_metadata(environment, artifact, files, virtual_files)
    virtual_files["ARTIFACT_METADATA.json"] = _json_bytes(metadata)
    write_deterministic_tar_gz(
        output,
        ((path, name) for name, path in files.items()),
        ((data, name) for name, data in virtual_files.items()),
    )


def _build_static_map_bundle(
    environment: dict[str, Any],
    artifact: dict[str, Any],
    repository_root: Path,
    candidate_root: Path,
    output: Path,
) -> None:
    files: dict[str, Path] = {}
    virtual_files: dict[str, bytes] = {}
    for file_spec in artifact["build"]["files"]:
        source = candidate_root / file_spec["source"]
        if not source.is_file():
            raise ArtifactError(f"static-map build input is missing: {source}")
        destination = file_spec["destination"]
        if source.suffix == ".json":
            data = json.loads(source.read_text(encoding="utf-8"))
            virtual_files[destination] = _json_bytes(
                _normalize_recorded_paths(data, candidate_root)
            )
        else:
            files[destination] = source
    inventory_bytes, inventory = _license_inventory(environment, repository_root)
    virtual_files["ATTRIBUTION.md"] = _attribution_bytes(
        environment, inventory, modified=True
    )
    virtual_files["LICENSES.json"] = inventory_bytes
    metadata = _bundle_metadata(environment, artifact, files, virtual_files)
    virtual_files["ARTIFACT_METADATA.json"] = _json_bytes(metadata)
    write_deterministic_tar_gz(
        output,
        ((path, name) for name, path in files.items()),
        ((data, name) for name, data in virtual_files.items()),
    )


def _update_static_map_file_contracts(
    environment: dict[str, Any], artifact: dict[str, Any], candidate_root: Path
) -> None:
    source_by_destination = {
        file_spec["destination"]: candidate_root / file_spec["source"]
        for file_spec in artifact["build"]["files"]
    }
    for static_map in environment["static_maps"]:
        if static_map.get("artifact_id") != artifact["id"]:
            continue
        for role in ("occupancy", "esdf"):
            contract = static_map[role]
            path = source_by_destination.get(contract["path"])
            if path is None:
                raise ArtifactError(
                    f"{environment['id']} {role} is not part of {artifact['id']}"
                )
            contract["sha256"] = sha256_file(path)
            contract["size_bytes"] = path.stat().st_size


def _bundle_metadata(
    environment: dict[str, Any],
    artifact: dict[str, Any],
    files: dict[str, Path],
    virtual_files: dict[str, bytes],
) -> dict[str, Any]:
    contracts = [
        {"path": name, "size_bytes": path.stat().st_size, "sha256": sha256_file(path)}
        for name, path in files.items()
    ]
    contracts.extend(
        {
            "path": name,
            "size_bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
        }
        for name, data in virtual_files.items()
    )
    return {
        "schema": "drone_city_nav_environment_bundle_v1",
        "environment_id": environment["id"],
        "artifact_id": artifact["id"],
        "artifact_kind": artifact["kind"],
        "source_url": environment["source"]["url"],
        "source_version": environment["source"]["version"],
        "license": environment["source"]["license"],
        "files": sorted(contracts, key=lambda contract: contract["path"]),
    }


def _attribution_bytes(
    environment: dict[str, Any], inventory: dict[str, Any], modified: bool
) -> bytes:
    source = environment["source"]
    license_data = source["license"]
    modification = (
        "This bundle contains project-generated static-map derivatives."
        if modified
        else "The source files are redistributed without intentional geometry changes."
    )
    license_summary = ", ".join(
        f"{spdx}: {count} resource(s)"
        for spdx, count in summarize_licenses(inventory).items()
    )
    text = (
        f"# {environment['display_name']}\n\n"
        f"Source: {source['url']}\n\n"
        f"Version: {source['version']}\n\n"
        f"Attribution: {license_data['attribution']}\n\n"
        f"License: {license_data['spdx']} ({license_data['url']})\n\n"
        f"Bundle license inventory: {license_summary}.\n\n"
        "See LICENSES.json for the pinned license of every bundled Fuel resource.\n\n"
        f"{modification}\n"
    )
    return text.encode("utf-8")


def _license_inventory(
    environment: dict[str, Any], repository_root: Path
) -> tuple[bytes, dict[str, Any]]:
    contract = environment["source"]["license_inventory"]
    path = repository_root / contract["path"]
    verify_file_contract(path, contract)
    data = path.read_bytes()
    try:
        inventory = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ArtifactError(f"invalid license inventory {path}: {exc}") from exc
    if (
        inventory.get("schema") != "drone_city_nav_fuel_license_inventory_v1"
        or inventory.get("environment_id") != environment["id"]
        or not inventory.get("resources")
    ):
        raise ArtifactError(f"license inventory identity is invalid: {path}")
    return data, inventory


def _resolve_recorded_path(recorded: str, candidate_root: Path) -> Path:
    direct = Path(recorded)
    if direct.exists():
        return direct.resolve()
    parts = direct.parts
    marker = ("external", "environment-candidates")
    for index in range(len(parts) - 1):
        if tuple(parts[index : index + 2]) == marker:
            candidate = candidate_root.joinpath(*parts[index + 2 :])
            if candidate.exists():
                return candidate.resolve()
    raise ArtifactError(f"recorded source path cannot be resolved: {recorded}")


def _normalize_recorded_paths(value: Any, candidate_root: Path) -> Any:
    if isinstance(value, dict):
        return {
            key: _normalize_recorded_paths(child, candidate_root)
            for key, child in value.items()
        }
    if isinstance(value, list):
        return [_normalize_recorded_paths(child, candidate_root) for child in value]
    if isinstance(value, str) and "external/environment-candidates" in value:
        try:
            path = _resolve_recorded_path(value, candidate_root)
        except ArtifactError:
            return value
        return path.relative_to(candidate_root).as_posix()
    return value


def _download_verified(url: str, destination: Path, contract: dict[str, Any]) -> None:
    temporary = destination.with_name(f".{destination.name}.part")
    last_error: Exception | None = None
    for attempt in range(3):
        temporary.unlink(missing_ok=True)
        try:
            with urlopen(url, timeout=120) as response, temporary.open("wb") as output:
                shutil.copyfileobj(response, output, length=1024 * 1024)
            verify_file_contract(temporary, contract)
            os.replace(temporary, destination)
            return
        except (HTTPError, URLError, TimeoutError, OSError, ArtifactError) as exc:
            last_error = exc
            temporary.unlink(missing_ok=True)
            if attempt < 2:
                time.sleep(float(attempt + 1))
    raise ArtifactError(f"download failed for {url}: {last_error}") from last_error


def _safe_extract(bundle: tarfile.TarFile, destination: Path) -> None:
    members = bundle.getmembers()
    _validate_members(members)
    root = destination.resolve()
    for member in members:
        target = (root / member.name).resolve()
        if not target.is_relative_to(root):
            raise ArtifactError(f"bundle path escapes destination: {member.name}")
    bundle.extractall(destination, members=members, filter="data")


def _validate_members(members: Iterable[tarfile.TarInfo]) -> None:
    for member in members:
        _safe_archive_path(member.name)
        if not (member.isfile() or member.isdir()):
            raise ArtifactError(f"unsupported bundle member type: {member.name}")


def _safe_archive_path(name: str) -> str:
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ArtifactError(f"unsafe archive path: {name}")
    normalized = path.as_posix()
    if normalized in {"", "."}:
        raise ArtifactError(f"unsafe archive path: {name}")
    return normalized


def _add_file(bundle: tarfile.TarFile, source: Path, name: str) -> None:
    if not source.is_file():
        raise ArtifactError(f"archive source is not a regular file: {source}")
    info = _tar_info(name, source.stat().st_size)
    with source.open("rb") as stream:
        bundle.addfile(info, stream)


def _add_bytes(bundle: tarfile.TarFile, data: bytes, name: str) -> None:
    bundle.addfile(_tar_info(name, len(data)), io.BytesIO(data))


def _tar_info(name: str, size: int) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name)
    info.size = size
    info.mode = 0o644
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    return info


def _sha256_stream(stream: Any) -> str:
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _require_within(path: Path, root: Path) -> None:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError as exc:
        raise ArtifactError(f"source path escapes candidate root: {path}") from exc
