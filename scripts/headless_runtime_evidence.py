"""Validate reproducible artifacts and persistent-3D runtime gate metrics."""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_manifest_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPOSITORY / path


def validate_manifest_file_record(
    label: str, record: object, errors: list[str]
) -> Path | None:
    if not isinstance(record, dict):
        errors.append(f"FAIL: runtime manifest has a {label} file record")
        return None
    path_value = record.get("path")
    digest = record.get("sha256")
    size = record.get("size_bytes")
    if not isinstance(path_value, str) or not re.fullmatch(
        r"[0-9a-f]{64}", str(digest)
    ):
        errors.append(f"FAIL: runtime manifest has a valid {label} identity")
        return None
    path = resolve_manifest_path(path_value)
    if not path.is_file():
        errors.append(f"FAIL: runtime manifest {label} exists: {path}")
        return None
    if path.stat().st_size != size or sha256_file(path) != digest:
        errors.append(f"FAIL: runtime manifest {label} hash matches its artifact")
        return None
    print(f"OK: runtime manifest binds the exact {label}")
    return path


def validate_raw_snapshot_artifact(
    record: object,
    expected_bounds: tuple[float, float, float, float, float, float] | None,
    errors: list[str],
) -> None:
    artifact_path = validate_manifest_file_record("raw snapshot", record, errors)
    if artifact_path is None or not isinstance(record, dict):
        return
    try:
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        errors.append(f"FAIL: raw snapshot artifact is readable JSON ({error})")
        return
    selection = artifact.get("selection")
    if artifact.get("schema") != "drone_city_nav_raw_snapshot_3d_v1" or not isinstance(
        selection, dict
    ):
        errors.append("FAIL: raw snapshot artifact has the supported schema")
        return
    artifact_bounds = selection.get("bounds_m")
    if expected_bounds is not None and artifact_bounds != list(expected_bounds):
        errors.append("FAIL: raw snapshot artifact matches the constrained route volume")
    observed = selection.get("observed_voxel_count", 0)
    occupied = selection.get("occupied_voxel_count", 0)
    chunks = artifact.get("chunks")
    if (
        not isinstance(observed, int)
        or not isinstance(occupied, int)
        or observed <= 0
        or occupied <= 0
        or not isinstance(chunks, list)
        or not chunks
    ):
        errors.append("FAIL: raw snapshot contains observed and occupied problem geometry")
        return
    counted_observed = 0
    counted_occupied = 0
    for chunk in chunks:
        if not isinstance(chunk, dict):
            errors.append("FAIL: raw snapshot chunks preserve exact bit words")
            return
        observed_words = chunk.get("observed_words_hex")
        occupied_words = chunk.get("occupied_words_hex")
        if (
            not isinstance(observed_words, list)
            or not isinstance(occupied_words, list)
            or len(observed_words) != len(occupied_words)
        ):
            errors.append("FAIL: raw snapshot chunks preserve exact bit words")
            return
        try:
            word_pairs = [
                (int(observed_word, 16), int(occupied_word, 16))
                for observed_word, occupied_word in zip(
                    observed_words, occupied_words, strict=True
                )
            ]
        except (TypeError, ValueError):
            errors.append("FAIL: raw snapshot chunks preserve exact bit words")
            return
        if any(
            occupied_word & ~observed_word
            for observed_word, occupied_word in word_pairs
        ):
            errors.append("FAIL: raw snapshot occupied bits are a subset of observed bits")
            return
        counted_observed += sum(word.bit_count() for word, _ in word_pairs)
        counted_occupied += sum(word.bit_count() for _, word in word_pairs)
    if counted_observed != observed or counted_occupied != occupied:
        errors.append("FAIL: raw snapshot voxel counts match its exact bit payload")
        return
    artifact_identity = {
        "producer_instance_id": artifact.get("producer_instance_id"),
        "revision": artifact.get("revision"),
        "bounds_m": artifact_bounds,
        "chunk_count": len(chunks),
        "observed_voxel_count": observed,
        "occupied_voxel_count": occupied,
    }
    if any(record.get(key) != value for key, value in artifact_identity.items()):
        errors.append("FAIL: runtime manifest raw metadata matches its exact artifact")
        return
    print(
        "OK: raw snapshot preserves constrained problem geometry "
        f"({observed} observed, {occupied} occupied voxels)"
    )


def validate_runtime_manifest(
    manifest_path: Path,
    expected_bounds: tuple[float, float, float, float, float, float] | None,
    require_raw_snapshot: bool,
    errors: list[str],
    *,
    expected_mission_type: str | None = None,
    expected_lidar_profile: str | None = None,
    expected_static_map: bool | None = None,
) -> None:
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        errors.append(f"FAIL: runtime manifest is readable JSON ({error})")
        return
    if manifest.get("schema") != "drone_city_nav_runtime_manifest_v1":
        errors.append("FAIL: runtime manifest has the supported schema")
        return
    repository = manifest.get("repository")
    commit = repository.get("commit") if isinstance(repository, dict) else None
    current_status = ""
    if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40,64}", commit):
        errors.append("FAIL: runtime manifest records a Git commit")
    else:
        try:
            git_identity = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=REPOSITORY,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            current_status = subprocess.run(
                ["git", "status", "--porcelain=v1", "--untracked-files=all"],
                cwd=REPOSITORY,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            git_identity = ""
            current_status = ""
        if git_identity != commit:
            errors.append("FAIL: runtime manifest commit matches the running checkout")
        else:
            print(f"OK: runtime manifest binds commit {commit[:12]}")
    if require_raw_snapshot and (
        not isinstance(repository, dict) or repository.get("dirty") is not False
    ):
        errors.append("FAIL: acceptance run uses a clean committed checkout")
    elif isinstance(repository, dict) and repository.get("dirty") is False:
        print("OK: runtime manifest records a clean checkout")
    if require_raw_snapshot and current_status:
        errors.append("FAIL: acceptance checkout remains clean through validation")
    validate_manifest_file_record("configuration", manifest.get("configuration"), errors)
    validate_manifest_file_record("world", manifest.get("world"), errors)
    mission = manifest.get("mission")
    if not isinstance(mission, dict):
        errors.append("FAIL: runtime manifest records the mission")
    else:
        if (
            expected_mission_type is not None
            and mission.get("type") != expected_mission_type
        ):
            errors.append("FAIL: runtime manifest records the active mission type")
        scenario = mission.get("scenario")
        if scenario is not None:
            validate_manifest_file_record("mission scenario", scenario, errors)
    runtime_profile = manifest.get("runtime_profile")
    if not isinstance(runtime_profile, dict):
        errors.append("FAIL: runtime manifest records the runtime profile")
    else:
        if (
            expected_lidar_profile is not None
            and runtime_profile.get("lidar_profile") != expected_lidar_profile
        ):
            errors.append("FAIL: runtime manifest records the active lidar profile")
        if (
            expected_static_map is not None
            and runtime_profile.get("static_map_enabled") != expected_static_map
        ):
            errors.append("FAIL: runtime manifest records the active static-map mode")
    manifest_bounds = manifest.get("constrained_route_volume_bounds_m")
    if expected_bounds is not None and manifest_bounds != list(expected_bounds):
        errors.append("FAIL: runtime manifest records the constrained route volume")
    if require_raw_snapshot:
        validate_raw_snapshot_artifact(
            manifest.get("raw_snapshot"), expected_bounds, errors
        )


def parse_latest_production_summary(ros_log: str) -> dict[str, str] | None:
    summaries = re.findall(r"PRODUCTION_MPPI_SUMMARY ([^\n]+)", ros_log)
    if not summaries:
        return None
    return dict(re.findall(r"([a-zA-Z0-9_]+)=([^\s]+)", summaries[-1]))


def validate_persistent_3d_acceptance_metrics(
    ros_log: str, errors: list[str]
) -> None:
    summary = parse_latest_production_summary(ros_log)
    if summary is None:
        errors.append("FAIL: persistent 3D acceptance has a production summary")
        return
    try:
        post_bootstrap_observations = int(summary["post_bootstrap_route_observations"])
        route_available_ticks = int(summary["post_bootstrap_route_available_ticks"])
        reported_availability = float(
            summary["post_bootstrap_route_availability_ratio"]
        )
        route_holds = int(
            summary["post_bootstrap_no_executable_route_hold_ticks"]
        )
        ownership_gaps = int(summary["ownership_gap_ticks"])
        planner_samples = int(summary["planner_latency_samples"])
        planner_p95_ms = float(summary["planner_p95_ms"])
        planner_p99_ms = float(summary["planner_p99_ms"])
        build_and_planning_p99_ms = float(
            summary["planner_build_and_planning_p99_ms"]
        )
    except (KeyError, ValueError):
        errors.append("FAIL: production summary contains persistent 3D gate metrics")
        return
    valid_availability_counters = (
        post_bootstrap_observations > 0
        and 0 <= route_available_ticks <= post_bootstrap_observations
    )
    availability = (
        route_available_ticks / post_bootstrap_observations
        if valid_availability_counters
        else 0.0
    )
    if not valid_availability_counters:
        errors.append("FAIL: route availability is measured after bootstrap")
    elif (
        not math.isfinite(reported_availability)
        or abs(reported_availability - availability) > 5.1e-7
    ):
        errors.append("FAIL: route availability ratio matches its tick counters")
    elif availability <= 0.99:
        errors.append(
            "FAIL: post-bootstrap route availability exceeds 99 percent "
            f"({availability:.4%})"
        )
    else:
        print(f"OK: post-bootstrap route availability is {availability:.4%}")
    valid_hold_counter = 0 <= route_holds <= max(post_bootstrap_observations, 0)
    hold_ratio = (
        route_holds / post_bootstrap_observations
        if post_bootstrap_observations > 0 and valid_hold_counter
        else math.inf
    )
    if hold_ratio >= 0.01:
        errors.append(
            "FAIL: ordinary post-bootstrap no-route holds stay below one percent "
            f"({hold_ratio:.4%})"
        )
    else:
        print(f"OK: post-bootstrap no-route hold ratio is {hold_ratio:.4%}")
    if ownership_gaps != 0:
        errors.append(f"FAIL: execution ownership has zero gaps ({ownership_gaps})")
    else:
        print("OK: execution ownership has zero gaps")
    if (
        planner_samples <= 0
        or not math.isfinite(planner_p95_ms)
        or planner_p95_ms < 0.0
    ):
        errors.append("FAIL: persistent planner latency has measured samples")
    elif planner_p95_ms >= 200.0:
        errors.append(
            f"FAIL: persistent planner p95 is below 200 ms ({planner_p95_ms:.3f} ms)"
        )
    else:
        print(f"OK: persistent planner p95 is {planner_p95_ms:.3f} ms")
    if (
        not math.isfinite(planner_p99_ms)
        or not math.isfinite(build_and_planning_p99_ms)
        or planner_p99_ms + 1.0e-9 < planner_p95_ms
        or build_and_planning_p99_ms + 1.0e-9 < planner_p99_ms
    ):
        errors.append("FAIL: persistent planner p95/p99 latency ordering is valid")

    admitted_routes = [
        line
        for line in re.findall(r"PRODUCTION_MPPI_GUIDE3D [^\n]+", ros_log)
        if "certified_pending=true" in line
    ]
    if not admitted_routes:
        errors.append("FAIL: runtime admits at least one certified persistent route")
        return
    invalid_reserve = []
    for line in admitted_routes:
        status = re.search(r"certified_reserve=([a-z_]+)", line)
        available = re.search(r"reserve_available_m=([0-9.eE+-]+)", line)
        required = re.search(r"reserve_required_m=([0-9.eE+-]+)", line)
        if status is None or status.group(1) not in {"sufficient", "terminal_exempt"}:
            invalid_reserve.append(line)
            continue
        if (
            status.group(1) == "sufficient"
            and (available is None or required is None)
        ):
            invalid_reserve.append(line)
            continue
        if status.group(1) == "sufficient":
            try:
                available_m = float(available.group(1))
                required_m = float(required.group(1))
            except ValueError:
                invalid_reserve.append(line)
                continue
            if (
                not math.isfinite(available_m)
                or not math.isfinite(required_m)
                or available_m < 0.0
                or required_m < 0.0
                or available_m + 1.0e-6 < required_m
            ):
                invalid_reserve.append(line)
    if invalid_reserve:
        errors.append(
            "FAIL: every admitted route satisfies latency, stopping, and overlap reserve"
        )
    else:
        print(
            "OK: every admitted route satisfies latency, stopping, and overlap reserve "
            f"({len(admitted_routes)})"
        )
