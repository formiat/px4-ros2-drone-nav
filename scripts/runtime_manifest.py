#!/usr/bin/env python3
"""Create and atomically update reproducible navigation-run manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA = "drone_city_nav_runtime_manifest_v1"
REPOSITORY = Path(__file__).resolve().parents[1]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def portable_path(path: Path, repository: Path = REPOSITORY) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(repository.resolve()).as_posix()
    except ValueError:
        return str(resolved)


def resolve_portable_path(value: str, repository: Path = REPOSITORY) -> Path:
    path = Path(value)
    return path if path.is_absolute() else repository / path


def file_record(path: Path, repository: Path = REPOSITORY) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return {
        "path": portable_path(resolved, repository),
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def read_manifest(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema") != SCHEMA:
        raise ValueError(f"unsupported runtime manifest: {path}")
    return document


def update_raw_snapshot_record(
    manifest_path: Path, record: dict[str, Any]
) -> None:
    document = read_manifest(manifest_path)
    current = document.get("raw_snapshot")
    current_revision = (
        int(current.get("revision", 0)) if isinstance(current, dict) else 0
    )
    new_revision = int(record.get("revision", 0))
    if new_revision < current_revision:
        return
    document["raw_snapshot"] = record
    atomic_write_json(manifest_path, document)


def _git_output(repository: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def parse_bounds(value: str) -> list[float]:
    components = [component.strip() for component in value.split(",")]
    if len(components) != 6:
        raise argparse.ArgumentTypeError("bounds require six comma-separated values")
    try:
        bounds = [float(component) for component in components]
    except ValueError as error:
        raise argparse.ArgumentTypeError("bounds must be numeric") from error
    if any(bounds[axis] >= bounds[axis + 3] for axis in range(3)):
        raise argparse.ArgumentTypeError("bounds minima must be below maxima")
    return bounds


def create_manifest(args: argparse.Namespace) -> dict[str, Any]:
    repository = args.repository.resolve()
    commit = _git_output(repository, "rev-parse", "HEAD")
    status = _git_output(repository, "status", "--porcelain=v1", "--untracked-files=all")
    document: dict[str, Any] = {
        "schema": SCHEMA,
        "run_id": args.run_id,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository": {
            "commit": commit,
            "dirty": bool(status),
            "status_sha256": hashlib.sha256(status.encode("utf-8")).hexdigest(),
        },
        "configuration": file_record(args.config, repository),
        "world": file_record(args.world, repository),
        "mission": {
            "type": args.mission_type,
            "goal_sequence_xyz_m": args.mission_goals,
        },
        "runtime_profile": {
            "lidar_profile": args.lidar_profile,
            "static_map_enabled": args.static_map_enabled,
        },
        "constrained_route_volume_bounds_m": args.route_volume_bounds,
        "raw_snapshot": {"status": "pending"},
    }
    if args.scenario is not None:
        document["mission"]["scenario"] = file_record(args.scenario, repository)
    return document


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repository", type=Path, default=REPOSITORY)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--mission-type", required=True)
    parser.add_argument("--mission-goals", default="")
    parser.add_argument("--lidar-profile", required=True)
    parser.add_argument(
        "--static-map-enabled", required=True, choices=("true", "false")
    )
    parser.add_argument("--route-volume-bounds", type=parse_bounds)
    args = parser.parse_args()
    args.static_map_enabled = args.static_map_enabled == "true"
    atomic_write_json(args.output, create_manifest(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
