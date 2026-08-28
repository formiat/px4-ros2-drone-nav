#!/usr/bin/env python3
"""Persist exact raw Occupancy3D bits intersecting an acceptance volume."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Sequence


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from runtime_manifest import (  # noqa: E402
    REPOSITORY,
    atomic_write_json,
    file_record,
    parse_bounds,
    update_raw_snapshot_record,
)


SCHEMA = "drone_city_nav_raw_snapshot_3d_v1"


def _cell_selected(
    cell_x: int,
    cell_y: int,
    cell_z: int,
    origin: Sequence[float],
    resolution_m: float,
    bounds: Sequence[float] | None,
) -> bool:
    if bounds is None:
        return True
    center = (
        origin[0] + (cell_x + 0.5) * resolution_m,
        origin[1] + (cell_y + 0.5) * resolution_m,
        origin[2] + (cell_z + 0.5) * resolution_m,
    )
    return all(bounds[axis] <= center[axis] <= bounds[axis + 3] for axis in range(3))


def _masked_words(
    chunk: Any,
    chunk_size: int,
    origin: Sequence[float],
    resolution_m: float,
    grid_shape: Sequence[int],
    bounds: Sequence[float] | None,
) -> tuple[list[int], list[int]]:
    observed = [0] * len(chunk.observed_words)
    occupied = [0] * len(chunk.occupied_words)
    for word_index, raw_word in enumerate(chunk.observed_words):
        pending = int(raw_word)
        while pending:
            lowest = pending & -pending
            offset = lowest.bit_length() - 1
            local_index = word_index * 64 + offset
            local_x = local_index % chunk_size
            local_y = (local_index // chunk_size) % chunk_size
            local_z = local_index // (chunk_size * chunk_size)
            cell_x = int(chunk.x) * chunk_size + local_x
            cell_y = int(chunk.y) * chunk_size + local_y
            cell_z = int(chunk.z) * chunk_size + local_z
            inside_grid = (
                0 <= cell_x < grid_shape[0]
                and 0 <= cell_y < grid_shape[1]
                and 0 <= cell_z < grid_shape[2]
            )
            if inside_grid and _cell_selected(
                cell_x,
                cell_y,
                cell_z,
                origin,
                resolution_m,
                bounds,
            ):
                observed[word_index] |= lowest
                if int(chunk.occupied_words[word_index]) & lowest:
                    occupied[word_index] |= lowest
            pending ^= lowest
    return observed, occupied


def snapshot_document(
    message: Any, bounds: Sequence[float] | None
) -> dict[str, Any] | None:
    chunk_size = int(message.chunk_size_cells)
    if chunk_size <= 0:
        raise ValueError("raw snapshot has an invalid chunk size")
    origin = (message.origin_x_m, message.origin_y_m, message.origin_z_m)
    grid_shape = (message.width_cells, message.height_cells, message.depth_cells)
    chunks: list[dict[str, Any]] = []
    observed_voxels = 0
    occupied_voxels = 0
    for chunk in message.chunks:
        observed, occupied = _masked_words(
            chunk,
            chunk_size,
            origin,
            float(message.resolution_m),
            grid_shape,
            bounds,
        )
        chunk_observed = sum(word.bit_count() for word in observed)
        if chunk_observed == 0:
            continue
        chunk_occupied = sum(word.bit_count() for word in occupied)
        observed_voxels += chunk_observed
        occupied_voxels += chunk_occupied
        chunks.append(
            {
                "index": [int(chunk.x), int(chunk.y), int(chunk.z)],
                "observed_words_hex": [f"{word:016x}" for word in observed],
                "occupied_words_hex": [f"{word:016x}" for word in occupied],
            }
        )
    if observed_voxels == 0 or occupied_voxels == 0:
        return None
    chunks.sort(key=lambda chunk: (chunk["index"][2], chunk["index"][1], chunk["index"][0]))
    return {
        "schema": SCHEMA,
        "message_type": "drone_city_nav/msg/RawObstacleSnapshot3D",
        "header": {
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frame_id": message.header.frame_id,
        },
        "producer_instance_id": int(message.producer_instance_id),
        "revision": int(message.obstacle_snapshot_revision),
        "grid": {
            "origin_m": [float(value) for value in origin],
            "resolution_m": float(message.resolution_m),
            "shape_cells": [int(value) for value in grid_shape],
            "chunk_size_cells": chunk_size,
        },
        "selection": {
            "bounds_m": list(bounds) if bounds is not None else None,
            "rule": "cell_center_inside_closed_bounds",
            "chunk_count": len(chunks),
            "observed_voxel_count": observed_voxels,
            "occupied_voxel_count": occupied_voxels,
        },
        "chunks": chunks,
    }


def artifact_manifest_record(
    artifact_path: Path, document: dict[str, Any]
) -> dict[str, Any]:
    record = file_record(artifact_path, REPOSITORY)
    selection = document["selection"]
    record.update(
        {
            "status": "captured",
            "producer_instance_id": document["producer_instance_id"],
            "revision": document["revision"],
            "bounds_m": selection["bounds_m"],
            "chunk_count": selection["chunk_count"],
            "observed_voxel_count": selection["observed_voxel_count"],
            "occupied_voxel_count": selection["occupied_voxel_count"],
        }
    )
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--bounds", type=parse_bounds)
    args = parser.parse_args()

    import rclpy
    from drone_city_nav.msg import RawObstacleSnapshot3D
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class SnapshotCaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("raw_snapshot_3d_artifact_capture")
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                RawObstacleSnapshot3D, args.topic, self._capture, qos
            )

        def _capture(self, message: RawObstacleSnapshot3D) -> None:
            document = snapshot_document(message, args.bounds)
            if document is None:
                return
            artifact_path = args.output_directory / (
                f"raw_snapshot_3d_revision_{document['revision']}.json"
            )
            atomic_write_json(artifact_path, document)
            update_raw_snapshot_record(
                args.manifest, artifact_manifest_record(artifact_path, document)
            )
            self.get_logger().info(
                "RAW_SNAPSHOT_ARTIFACT captured=true "
                f"revision={document['revision']} path='{artifact_path}' "
                f"observed={document['selection']['observed_voxel_count']} "
                f"occupied={document['selection']['occupied_voxel_count']}"
            )

    rclpy.init()
    node = SnapshotCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
