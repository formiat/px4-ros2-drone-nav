#!/usr/bin/env python3
"""Tests for the lidar projection snapshot analyzer."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "analyze_lidar_projection_snapshots.py"
)
SPEC = importlib.util.spec_from_file_location("lidar_snapshot_analyzer", SCRIPT_PATH)
assert SPEC is not None
analyzer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analyzer
assert SPEC.loader is not None
SPEC.loader.exec_module(analyzer)


def make_record(
    *,
    altitude_m: float,
    processed: int = 10,
    accepted: int = 8,
    hits: int = 2,
    altitude_rejected: int = 1,
    remembered_hits: int = 3,
    image_ok: bool = True,
    config_yaw: float = 0.0,
    projection_yaw_delta: float = 0.0,
    hit_points: list[dict[str, float]] | None = None,
) -> dict:
    if hit_points is None:
        hit_points = [{"x": 1.0, "y": 0.0}, {"x": 0.0, "y": 1.0}]
    return {
        "snapshot": "snapshot_000001",
        "pose": {"altitude_m": altitude_m},
        "scan": {
            "processed": processed,
            "hits": hits,
            "altitude_rejected": altitude_rejected,
            "projection_rejected": 0,
        },
        "projection_stats": {
            "accepted": accepted,
            "hit": hits,
            "altitude_rejected": altitude_rejected,
            "invalid_range": 0,
            "invalid_scan": 0,
        },
        "projection_config": {
            "compensate_attitude": True,
            "use_px4_heading_for_scan": True,
            "initial_heading_rad": 0.0,
            "scan_yaw_offset_rad": 0.0,
            "lidar_mount_roll_rad": 0.0,
            "lidar_mount_pitch_rad": 0.0,
            "lidar_mount_yaw_rad": config_yaw,
            "min_projected_altitude_m": 1.0,
            "max_projected_altitude_m": 40.0,
        },
        "projection": {
            "yaw_source": "px4_heading",
            "yaw_rad": 0.0,
            "px4_heading_valid": True,
            "yaw_delta_to_attitude_rad": projection_yaw_delta,
        },
        "grid": {"seen": True},
        "image_ok": image_ok,
        "remembered_hits": remembered_hits,
        "hit_points": hit_points,
    }


class LidarProjectionSnapshotAnalyzerTest(unittest.TestCase):
    def test_happy_path_accepts_cruise_hits_and_memory(self) -> None:
        records = [
            make_record(altitude_m=1.0, processed=10, accepted=0, hits=0),
            make_record(altitude_m=18.0, processed=10, accepted=8, hits=2),
        ]

        result = analyzer.analyze_snapshots(records)

        self.assertTrue(result.ok, result.errors)
        self.assertEqual(result.cruise_snapshot_count, 1)
        self.assertEqual(result.max_cruise_current_hits, 2)

    def test_empty_jsonl_fails(self) -> None:
        result = analyzer.analyze_snapshots([])

        self.assertFalse(result.ok)
        self.assertIn("no snapshots were loaded", result.errors)

    def test_cruise_altitude_rejection_dominates_fails(self) -> None:
        records = [
            make_record(
                altitude_m=18.0,
                processed=10,
                accepted=0,
                hits=0,
                altitude_rejected=10,
            )
        ]

        result = analyzer.analyze_snapshots(records)

        self.assertFalse(result.ok)
        self.assertTrue(
            any("altitude rejection dominates" in error for error in result.errors)
        )
        self.assertTrue(
            any("no current lidar hits" in error for error in result.errors)
        )

    def test_missing_remembered_hits_fails(self) -> None:
        records = [
            make_record(
                altitude_m=18.0,
                processed=10,
                accepted=8,
                hits=2,
                remembered_hits=0,
            )
        ]

        result = analyzer.analyze_snapshots(records)

        self.assertFalse(result.ok)
        self.assertIn(
            "remembered lidar hits are absent in the final snapshot", result.errors
        )

    def test_malformed_json_line_fails_to_load(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "snapshots.jsonl"
            path.write_text("{not json}\n", encoding="utf-8")

            with self.assertRaises(analyzer.SnapshotLoadError):
                analyzer.load_snapshots(path)

    def test_projection_config_changes_fail(self) -> None:
        records = [
            make_record(altitude_m=18.0, config_yaw=0.0),
            make_record(altitude_m=19.0, config_yaw=1.0),
        ]

        result = analyzer.analyze_snapshots(records)

        self.assertFalse(result.ok)
        self.assertTrue(
            any("projection_config changed" in error for error in result.errors)
        )

    def test_large_projection_yaw_delta_fails(self) -> None:
        records = [make_record(altitude_m=18.0, projection_yaw_delta=0.6)]

        result = analyzer.analyze_snapshots(records)

        self.assertFalse(result.ok)
        self.assertTrue(
            any("projection yaw diverges" in error for error in result.errors)
        )

    def test_static_map_hit_alignment_fails_for_rotated_hits(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            static_map_path = Path(temp_dir) / "generated_city.map2d"
            static_map_path.write_text(
                "drone_city_nav_static_map_v1\n"
                "rect building_001 0 0 2 2 20\n",
                encoding="utf-8",
            )
            records = [
                make_record(
                    altitude_m=18.0,
                    hit_points=[{"x": 10.0, "y": 10.0}, {"x": 11.0, "y": 10.0}],
                )
            ]

            result = analyzer.analyze_snapshots(
                records, static_map_path=static_map_path
            )

        self.assertFalse(result.ok)
        self.assertTrue(
            any("lidar hit points do not align" in error for error in result.errors)
        )

    def test_cli_accepts_static_map_rectangles(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            snapshots_path = temp_path / "snapshots.jsonl"
            static_map_path = temp_path / "generated_city.map2d"
            snapshots_path.write_text(
                json.dumps(make_record(altitude_m=18.0)) + "\n", encoding="utf-8"
            )
            static_map_path.write_text(
                "drone_city_nav_static_map_v1\n"
                "rect building_001 0 0 1 1 20\n",
                encoding="utf-8",
            )

            with redirect_stdout(StringIO()):
                exit_code = analyzer.main(
                    [str(snapshots_path), "--static-map", str(static_map_path)]
                )

        self.assertEqual(exit_code, 0)


if __name__ == "__main__":
    unittest.main()
