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
    max_cruise_projection_attitude_yaw_delta_rad: float = 0.0
    final_remembered_hits: int = 0
    static_map_rectangles: int | None = None
    static_map_lidar_hit_count: int = 0
    static_map_aligned_lidar_hit_count: int = 0
    static_map_lidar_hit_alignment_ratio: float | None = None
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


@dataclass(frozen=True)
class StaticMapRect:
    center_x_m: float
    center_y_m: float
    size_x_m: float
    size_y_m: float


def load_static_map_rectangles(path: Path) -> list[StaticMapRect]:
    rectangles: list[StaticMapRect] = []
    try:
        with path.open("r", encoding="utf-8") as file:
            for line_number, line in enumerate(file, start=1):
                parts = line.split()
                if not parts or parts[0] != "rect":
                    continue
                if len(parts) != 7:
                    raise SnapshotLoadError(
                        f"{path}:{line_number}: malformed rect entry"
                    )
                try:
                    center_x_m = float(parts[2])
                    center_y_m = float(parts[3])
                    size_x_m = float(parts[4])
                    size_y_m = float(parts[5])
                except ValueError as exc:
                    raise SnapshotLoadError(
                        f"{path}:{line_number}: rect geometry must be numeric"
                    ) from exc
                if not (
                    math.isfinite(center_x_m)
                    and math.isfinite(center_y_m)
                    and size_x_m > 0.0
                    and size_y_m > 0.0
                ):
                    raise SnapshotLoadError(
                        f"{path}:{line_number}: rect geometry is invalid"
                    )
                rectangles.append(
                    StaticMapRect(center_x_m, center_y_m, size_x_m, size_y_m)
                )
    except FileNotFoundError as exc:
        raise SnapshotLoadError(f"{path}: static map file does not exist") from exc
    return rectangles


def count_static_map_rectangles(path: Path) -> int:
    return len(load_static_map_rectangles(path))


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
        "use_px4_heading_for_scan",
        "initial_heading_rad",
        "swap_lidar_xy_to_local_frame",
        "scan_yaw_offset_rad",
        "lidar_mount_roll_rad",
        "lidar_mount_pitch_rad",
        "lidar_mount_yaw_rad",
        "min_projected_altitude_m",
        "max_projected_altitude_m",
    )
    return tuple(config.get(key) for key in keys)


def _point_to_rect_boundary_distance_m(
    x_m: float, y_m: float, rect: StaticMapRect
) -> float:
    half_x_m = rect.size_x_m / 2.0
    half_y_m = rect.size_y_m / 2.0
    dx_m = abs(x_m - rect.center_x_m)
    dy_m = abs(y_m - rect.center_y_m)

    if dx_m <= half_x_m and dy_m <= half_y_m:
        return min(half_x_m - dx_m, half_y_m - dy_m)

    return math.hypot(max(dx_m - half_x_m, 0.0), max(dy_m - half_y_m, 0.0))


def _hit_points(record: dict[str, Any]) -> list[tuple[float, float]]:
    points = record.get("hit_points")
    if not isinstance(points, list):
        return []

    parsed_points: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, dict):
            continue
        x_m = _finite_float(point.get("x"))
        y_m = _finite_float(point.get("y"))
        if math.isfinite(x_m) and math.isfinite(y_m):
            parsed_points.append((x_m, y_m))
    return parsed_points


def _static_map_lidar_alignment_ratio(
    records: list[dict[str, Any]],
    rectangles: list[StaticMapRect],
    max_hit_distance_m: float,
) -> tuple[int, int, float | None]:
    if not rectangles:
        return 0, 0, None

    hit_count = 0
    aligned_count = 0
    for record in records:
        for x_m, y_m in _hit_points(record):
            hit_count += 1
            distance_m = min(
                _point_to_rect_boundary_distance_m(x_m, y_m, rect)
                for rect in rectangles
            )
            if distance_m <= max_hit_distance_m:
                aligned_count += 1

    if hit_count <= 0:
        return 0, 0, None
    return hit_count, aligned_count, aligned_count / hit_count


def analyze_snapshots(
    records: list[dict[str, Any]],
    *,
    cruise_altitude_min_m: float = 15.0,
    altitude_rejection_threshold: float = 0.75,
    yaw_delta_threshold_rad: float = 0.35,
    static_map_path: Path | None = None,
    static_map_hit_distance_m: float = 1.5,
    static_map_hit_alignment_threshold: float = 0.5,
) -> AnalysisResult:
    result = AnalysisResult(snapshot_count=len(records))
    static_map_rectangles: list[StaticMapRect] = []

    if static_map_path is not None:
        static_map_rectangles = load_static_map_rectangles(static_map_path)
        result.static_map_rectangles = len(static_map_rectangles)
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
        yaw_deltas = [
            abs(_finite_float(_nested(record, "projection", "yaw_delta_to_attitude_rad")))
            for record in cruise_records
        ]
        yaw_deltas = [delta for delta in yaw_deltas if math.isfinite(delta)]
        result.max_cruise_projection_attitude_yaw_delta_rad = max(
            yaw_deltas, default=0.0
        )
        if (
            yaw_deltas
            and result.max_cruise_projection_attitude_yaw_delta_rad
            >= yaw_delta_threshold_rad
        ):
            result.errors.append(
                "projection yaw diverges from attitude yaw at cruise altitude: "
                f"{result.max_cruise_projection_attitude_yaw_delta_rad:.3f} >= "
                f"{yaw_delta_threshold_rad:.3f}"
            )

        if static_map_rectangles:
            (
                result.static_map_lidar_hit_count,
                result.static_map_aligned_lidar_hit_count,
                result.static_map_lidar_hit_alignment_ratio,
            ) = _static_map_lidar_alignment_ratio(
                cruise_records, static_map_rectangles, static_map_hit_distance_m
            )
            if result.static_map_lidar_hit_alignment_ratio is None:
                result.errors.append(
                    "static map alignment cannot be checked: no logged lidar hit_points "
                    "at cruise altitude"
                )
            elif (
                result.static_map_lidar_hit_alignment_ratio
                < static_map_hit_alignment_threshold
            ):
                result.errors.append(
                    "lidar hit points do not align with static map walls: "
                    f"{result.static_map_lidar_hit_alignment_ratio:.3f} < "
                    f"{static_map_hit_alignment_threshold:.3f} within "
                    f"{static_map_hit_distance_m:.2f} m"
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
        "max cruise projection-attitude yaw delta: "
        f"{result.max_cruise_projection_attitude_yaw_delta_rad:.3f} rad",
        f"final remembered hits: {result.final_remembered_hits}",
    ]
    if result.static_map_rectangles is not None:
        lines.append(f"static map rectangles: {result.static_map_rectangles}")
    if result.static_map_lidar_hit_alignment_ratio is not None:
        lines.append(
            "static map lidar-hit alignment: "
            f"{result.static_map_lidar_hit_alignment_ratio:.3f} "
            f"({result.static_map_aligned_lidar_hit_count}/"
            f"{result.static_map_lidar_hit_count})"
        )
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
    parser.add_argument("--yaw-delta-threshold-rad", type=float, default=0.35)
    parser.add_argument("--static-map-hit-distance-m", type=float, default=1.5)
    parser.add_argument("--static-map-hit-alignment-threshold", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        records = load_snapshots(args.snapshots_jsonl)
        result = analyze_snapshots(
            records,
            cruise_altitude_min_m=args.cruise_altitude_min,
            altitude_rejection_threshold=args.altitude_rejection_threshold,
            yaw_delta_threshold_rad=args.yaw_delta_threshold_rad,
            static_map_path=args.static_map,
            static_map_hit_distance_m=args.static_map_hit_distance_m,
            static_map_hit_alignment_threshold=args.static_map_hit_alignment_threshold,
        )
    except SnapshotLoadError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(format_result(result))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
