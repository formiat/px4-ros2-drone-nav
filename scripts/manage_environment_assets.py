#!/usr/bin/env python3
"""List, verify, fetch, and install versioned environment artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from environment_artifacts import (
    ArtifactError,
    fetch_artifact,
    install_artifact,
    verify_release_artifacts,
    verify_repository_files,
)
from environment_manifest import (
    artifact_release_for_environment,
    find_environment,
    load_manifest,
    repository_root,
)


DEFAULT_MANIFEST = Path("environments/environment_manifest.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="list distributed environments")

    verify_parser = subparsers.add_parser("verify", help="verify local contracts")
    verify_parser.add_argument("--environment")
    verify_parser.add_argument("--release-dir", type=Path)
    verify_parser.add_argument("--skip-bundle-contents", action="store_true")

    fetch_parser = subparsers.add_parser("fetch", help="fetch and install an artifact")
    fetch_parser.add_argument("--environment", required=True)
    fetch_parser.add_argument("--artifact", required=True)
    fetch_parser.add_argument("--release-dir", type=Path)
    fetch_parser.add_argument("--cache-dir", type=Path)
    fetch_parser.add_argument("--install-dir", type=Path)
    fetch_parser.add_argument("--base-url")
    fetch_parser.add_argument("--no-extract", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = repository_root(manifest_path)
    if args.command == "list":
        _list_environments(manifest)
        return

    if args.command == "verify":
        repository_files = verify_repository_files(manifest, manifest_path)
        release_files = verify_release_artifacts(
            manifest,
            repository,
            environment_id=args.environment,
            inspect_contents=not args.skip_bundle_contents,
            release_directory_override=args.release_dir,
        )
        print(
            "ENVIRONMENT_ASSETS_VERIFIED"
            f" repository_files={len(repository_files)}"
            f" release_artifacts={len(release_files)}"
        )
        return

    environment = find_environment(manifest, args.environment)
    if environment["distribution"] != "release":
        raise ArtifactError(
            f"{environment['id']} is already distributed in the repository"
        )
    artifact = next(
        (
            candidate
            for candidate in environment["artifacts"]
            if candidate["id"] == args.artifact
        ),
        None,
    )
    if artifact is None:
        raise ArtifactError(
            f"unknown artifact for {environment['id']}: {args.artifact}"
        )
    release = artifact_release_for_environment(manifest, environment)
    default_release = repository / release["local_mirror"]
    release_directory = (args.release_dir or default_release).resolve()
    cache_directory = (
        args.cache_dir
        or repository / "external/environment-artifacts/download-cache"
    ).resolve()
    archive = fetch_artifact(
        manifest,
        environment,
        artifact,
        release_directory,
        cache_directory,
        args.base_url,
    )
    if args.no_extract:
        print(f"ENVIRONMENT_ARTIFACT_FETCHED path={archive}")
        return
    install_root = (
        args.install_dir or repository / "external/environment-artifacts/installed"
    ).resolve()
    destination = install_root / environment["id"] / artifact["id"]
    install_artifact(archive, destination)
    print(f"ENVIRONMENT_ARTIFACT_INSTALLED path={destination}")


def _list_environments(manifest: dict) -> None:
    for environment in manifest["environments"]:
        artifacts = ",".join(
            artifact["id"] for artifact in environment.get("artifacts", [])
        )
        release = (
            artifact_release_for_environment(manifest, environment)["tag"]
            if environment["distribution"] == "release"
            else "repository"
        )
        print(
            f"{environment['id']}"
            f" role={environment['role']}"
            f" classification={environment['classification']}"
            f" distribution={environment['distribution']}"
            f" release={release}"
            f" artifacts={artifacts or 'repository'}"
        )


if __name__ == "__main__":
    try:
        main()
    except (ArtifactError, RuntimeError) as error:
        raise SystemExit(f"environment asset error: {error}") from error
