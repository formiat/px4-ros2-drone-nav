#!/usr/bin/env python3
"""Validate lidar projection debug snapshots from a headless simulation run."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class SnapshotLoadError(ValueError):
    """Raised when a snapshots JSONL file cannot be parsed."""


@dataclass
class BeamCounts:
    processed: int = 0
    accepted: int = 0
    hit: int = 0
    altitude_rejected: int = 0
    invalid_range: int = 0
    invalid_scan: int = 0


@dataclass
class AnalysisResult:
    snapshot_count: int = 0
    cruise_snapshot_count: int = 0
    max_cruise_current_hits: int = 0
    max_cruise_accepted_beams: int = 0
    max_cruise_altitude_rejection_ratio: float = 0.0
    final_remembered_hits: int = 0
    static_map_rectangles: int | None = None
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def load_snapshots(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as file:
            for line_number, line in enumerate(file, start=1):
                stripped = line.strip()
                if not stripped:
                    continue
                try:
                    record = json.loads(stripped)
                except json.JSONDecodeError as exc:
                    raise SnapshotLoadError(
                        f"{path}:{line_number}: malformed JSON: {exc.msg}"
                    ) from exc
                if not isinstance(record, dict):
                    raise SnapshotLoadError(
                        f"{path}:{line_number}: snapshot record must be a JSON object"
                    )
                records.append(record)
    except FileNotFoundError as exc:
        raise SnapshotLoadError(f"{path}: file does not exist") from exc
    return records


def count_static_map_rectangles(path: Path) -> int:
    try:
        with path.open("r", encoding="utf-8") as file:
            return sum(1 for line in file if line.lstrip().startswith("rect "))
    except FileNotFoundError as exc:
        raise SnapshotLoadError(f"{path}: static map file does not exist") from exc


def _nested(record: dict[str, Any], *keys: str) -> Any:
    value: Any = record
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _finite_float(value: Any, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _non_negative_int(value: Any, default: int = 0) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError):
        return default
    return max(0, result)


def _beam_counts(record: dict[str, Any]) -> BeamCounts:
    projection_stats = record.get("projection_stats")
    scan = record.get("scan")
    projection_stats = projection_stats if isinstance(projection_stats, dict) else {}
    scan = scan if isinstance(scan, dict) else {}

    accepted = _non_negative_int(projection_stats.get("accepted"))
    hit = _non_negative_int(projection_stats.get("hit"), _non_negative_int(scan.get("hits")))
    altitude_rejected = _non_negative_int(
        projection_stats.get("altitude_rejected"),
        _non_negative_int(scan.get("altitude_rejected")),
    )
    invalid_range = _non_negative_int(projection_stats.get("invalid_range"))
    invalid_scan = _non_negative_int(projection_stats.get("invalid_scan"))
    processed = _non_negative_int(scan.get("processed"))
    if processed <= 0:
        processed = accepted + altitude_rejected + invalid_range + invalid_scan
    if processed <= 0:
        processed = _non_negative_int(scan.get("beams"))

    if accepted <= 0 and processed > 0:
        rejected = altitude_rejected + _non_negative_int(scan.get("projection_rejected"))
        accepted = max(0, processed - rejected)

    return BeamCounts(
        processed=processed,
        accepted=accepted,
        hit=hit,
        altitude_rejected=altitude_rejected,
        invalid_range=invalid_range,
        invalid_scan=invalid_scan,
    )


def _projection_config_key(record: dict[str, Any]) -> tuple[Any, ...] | None:
    config = record.get("projection_config")
    if not isinstance(config, dict):
        return None
    keys = (
        "compensate_attitude",
        "swap_lidar_xy_to_local_frame",
        "scan_yaw_offset_rad",
        "lidar_mount_roll_rad",
        "lidar_mount_pitch_rad",
        "lidar_mount_yaw_rad",
        "min_projected_altitude_m",
        "max_projected_altitude_m",
    )
    return tuple(config.get(key) for key in keys)


def analyze_snapshots(
    records: list[dict[str, Any]],
    *,
    cruise_altitude_min_m: float = 15.0,
    altitude_rejection_threshold: float = 0.75,
    static_map_path: Path | None = None,
) -> AnalysisResult:
    result = AnalysisResult(snapshot_count=len(records))

    if static_map_path is not None:
        result.static_map_rectangles = count_static_map_rectangles(static_map_path)
        if result.static_map_rectangles <= 0:
            result.errors.append("static map has no rect entries")

    if not records:
        result.errors.append("no snapshots were loaded")
        return result

    cruise_records = [
        record
        for record in records
        if _finite_float(_nested(record, "pose", "altitude_m"), default=-math.inf)
        >= cruise_altitude_min_m
    ]
    result.cruise_snapshot_count = len(cruise_records)
    if not cruise_records:
        result.errors.append(
            f"no cruise snapshots with altitude >= {cruise_altitude_min_m:.2f} m"
        )
    else:
        cruise_counts = [_beam_counts(record) for record in cruise_records]
        result.max_cruise_current_hits = max(count.hit for count in cruise_counts)
        result.max_cruise_accepted_beams = max(count.accepted for count in cruise_counts)
        ratios = [
            count.altitude_rejected / count.processed
            for count in cruise_counts
            if count.processed > 0
        ]
        result.max_cruise_altitude_rejection_ratio = max(ratios, default=1.0)
        if result.max_cruise_accepted_beams <= 0:
            result.errors.append("no accepted projected beams at cruise altitude")
        if result.max_cruise_current_hits <= 0:
            result.errors.append("no current lidar hits at cruise altitude")
        if result.max_cruise_altitude_rejection_ratio >= altitude_rejection_threshold:
            result.errors.append(
                "altitude rejection dominates at cruise altitude: "
                f"{result.max_cruise_altitude_rejection_ratio:.3f} >= "
                f"{altitude_rejection_threshold:.3f}"
            )

    result.final_remembered_hits = _non_negative_int(records[-1].get("remembered_hits"))
    if result.final_remembered_hits <= 0:
        result.errors.append("remembered lidar hits are absent in the final snapshot")

    bad_image_snapshots = [
        str(record.get("snapshot", f"#{index}"))
        for index, record in enumerate(records, start=1)
        if bool(_nested(record, "grid", "seen")) and not bool(record.get("image_ok", False))
    ]
    if bad_image_snapshots:
        result.errors.append(
            "image_ok is false for grid snapshots: " + ", ".join(bad_image_snapshots)
        )

    config_keys = [
        config for config in (_projection_config_key(record) for record in records) if config
    ]
    if config_keys:
        distinct_configs = {config for config in config_keys}
        if len(distinct_configs) > 1:
            result.errors.append(
                f"projection_config changed across snapshots ({len(distinct_configs)} variants)"
            )
        if len(config_keys) != len(records):
            result.warnings.append("some snapshots do not include projection_config")
    else:
        result.warnings.append("snapshots do not include projection_config")

    return result


def format_result(result: AnalysisResult) -> str:
    lines = [
        "Lidar projection snapshot analysis",
        f"snapshots: {result.snapshot_count}",
        f"cruise snapshots: {result.cruise_snapshot_count}",
        f"max cruise current hits: {result.max_cruise_current_hits}",
        f"max cruise accepted beams: {result.max_cruise_accepted_beams}",
        "max cruise altitude rejection ratio: "
        f"{result.max_cruise_altitude_rejection_ratio:.3f}",
        f"final remembered hits: {result.final_remembered_hits}",
    ]
    if result.static_map_rectangles is not None:
        lines.append(f"static map rectangles: {result.static_map_rectangles}")
    if result.warnings:
        lines.append("warnings:")
        lines.extend(f"- {warning}" for warning in result.warnings)
    if result.errors:
        lines.append("errors:")
        lines.extend(f"- {error}" for error in result.errors)
    lines.append(f"result: {'PASS' if result.ok else 'FAIL'}")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate lidar debug snapshots from a headless simulation run."
    )
    parser.add_argument("snapshots_jsonl", type=Path)
    parser.add_argument("--static-map", type=Path, default=None)
    parser.add_argument("--cruise-altitude-min", type=float, default=15.0)
    parser.add_argument("--altitude-rejection-threshold", type=float, default=0.75)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        records = load_snapshots(args.snapshots_jsonl)
        result = analyze_snapshots(
            records,
            cruise_altitude_min_m=args.cruise_altitude_min,
            altitude_rejection_threshold=args.altitude_rejection_threshold,
            static_map_path=args.static_map,
        )
    except SnapshotLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(format_result(result))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
