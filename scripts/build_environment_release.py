#!/usr/bin/env python3
"""Build deterministic environment release artifacts from the local candidate set."""

from __future__ import annotations

import argparse
from pathlib import Path

from environment_artifacts import (
    build_release_artifacts,
    update_repository_file_contracts,
)
from environment_manifest import load_manifest, write_manifest


DEFAULT_MANIFEST = Path("environments/environment_manifest.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--candidate-root",
        type=Path,
        default=Path("external/environment-candidates"),
    )
    parser.add_argument("--release-dir", type=Path)
    parser.add_argument("--environment", action="append", default=[])
    parser.add_argument("--update-manifest", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = manifest_path.parent.parent
    release_directory = (
        args.release_dir
        or repository / manifest["artifact_release"]["local_mirror"]
    ).resolve()
    selected = set(args.environment) or None
    update_repository_file_contracts(manifest, manifest_path)
    built = build_release_artifacts(
        manifest,
        repository,
        args.candidate_root.resolve(),
        release_directory,
        selected,
    )
    if args.update_manifest:
        write_manifest(manifest_path, manifest)
    snapshot = release_directory / "environment_manifest.yaml"
    write_manifest(snapshot, manifest)
    for path in built:
        print(f"ENVIRONMENT_ARTIFACT_BUILT path={path}")
    print(f"ENVIRONMENT_RELEASE_STAGED path={release_directory}")


if __name__ == "__main__":
    main()
