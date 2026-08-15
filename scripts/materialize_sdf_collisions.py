#!/usr/bin/env python3
"""Create a local collision-only SDF from a world and cached model assets."""

from __future__ import annotations

import argparse
from pathlib import Path

from sdf_collision_materializer import (
    CollisionWorldMaterializer,
    ResourceResolver,
    write_materialized_world,
    write_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output-sdf", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--fuel-cache", action="append", type=Path, default=[])
    parser.add_argument("--model-path", action="append", type=Path, default=[])
    parser.add_argument("--preview-visuals", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    resolver = ResourceResolver(args.fuel_cache, args.model_path)
    materializer = CollisionWorldMaterializer(
        resolver, preview_visuals=args.preview_visuals
    )
    tree, report = materializer.materialize(args.world)
    fingerprint = write_materialized_world(tree, args.output_sdf)
    write_report(report, args.output_sdf, fingerprint, args.report)
    print(
        "SDF_COLLISIONS_MATERIALIZED"
        f" collisions={report.collision_instances}"
        f" meshes={len(report.mesh_files)}"
        f" dynamic_skipped={report.dynamic_models_skipped}"
        f" non_collision_resources_skipped={report.non_collision_resources_skipped}"
        f" output={args.output_sdf}"
    )


if __name__ == "__main__":
    main()
