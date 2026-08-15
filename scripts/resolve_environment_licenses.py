#!/usr/bin/env python3
"""Pin per-resource Gazebo Fuel licenses for a distributed environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from environment_manifest import find_environment, load_manifest, repository_root
from fuel_license_inventory import (
    requests_from_materialization_report,
    resolve_inventory,
    summarize_licenses,
    write_inventory,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("environments/environment_manifest.yaml"),
    )
    parser.add_argument("--environment", required=True)
    parser.add_argument(
        "--candidate-root",
        type=Path,
        default=Path("external/environment-candidates"),
    )
    parser.add_argument("--workers", type=int, default=8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = repository_root(manifest_path)
    environment = find_environment(manifest, args.environment)
    if environment["source"]["provider"] != "gazebo_fuel":
        raise RuntimeError(f"{environment['id']} is not a Gazebo Fuel environment")
    source_artifact = next(
        artifact
        for artifact in environment["artifacts"]
        if artifact["kind"] == "source_bundle"
    )
    report_path = (
        args.candidate_root.resolve()
        / source_artifact["build"]["materialization_report"]
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    requests = requests_from_materialization_report(
        environment["source"]["url"],
        environment["source"]["version"],
        report,
    )
    inventory = resolve_inventory(environment["id"], requests, args.workers)
    output = repository / environment["source"]["license_inventory"]["path"]
    write_inventory(output, inventory)
    summary = ",".join(
        f"{spdx}:{count}" for spdx, count in summarize_licenses(inventory).items()
    )
    print(
        "FUEL_LICENSE_INVENTORY_RESOLVED"
        f" environment={environment['id']}"
        f" resources={len(inventory['resources'])}"
        f" licenses={summary}"
        f" output={output}"
    )


if __name__ == "__main__":
    main()
