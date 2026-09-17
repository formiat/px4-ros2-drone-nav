"""Validate reproducible artifacts and persistent-3D runtime gate metrics."""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]

# The acceptance thresholds the urban navigation programme runs to. A route
# the vehicle can execute on all but three percent of its post-bootstrap
# ticks, and ordinary holds without one under three percent of them. Measured
# flights of the current stack sit at one and a half to three percent of
# holds against ninety-eight percent availability, and the remaining gap is
# the recovery after a physical block: a fresh occupied cell a metre ahead
# costs the vehicle its route for a third of a second to a second, five to
# fourteen times a flight.
MINIMUM_POST_BOOTSTRAP_ROUTE_AVAILABILITY = 0.97
MAXIMUM_POST_BOOTSTRAP_NO_ROUTE_HOLD_RATIO = 0.03
# The production tick's wall time, snapshot to publication, from the
# PRODUCTION_MPPI_SUMMARY percentiles. The tick is scheduled at 50 Hz
# (deadline_ms 20); the vehicle receives a fresh horizon at the rate the
# whole tick allows, not at the rate the CUDA controller alone would. Measured
# on r314 (4b3dac94, the collision oracle built once per path): p50 22.7 ms,
# p95 30.8 ms, with the controller at 10.7 ms of it; before that fix the
# flights r303 to r313 sat at p50 52 to 55 ms and p95 70 to 98 ms. The bounds
# hold the regression, not the deadline: 71 percent of r314's ticks still
# overran 20 ms.
MAXIMUM_TICK_TOTAL_P50_MS = 30.0
MAXIMUM_TICK_TOTAL_P95_MS = 45.0

# The mean flight speed the programme runs to: the path the vehicle flew,
# divided by the time from mission readiness to the successful mission
# result. The climb, every hold and stop, every replan and the approach to
# the goal all count against it. The five accepting flights of the 97/3
# thresholds flew 430 to 573 m in 275 to 367 s: 1.55 to 1.63 m/s; the five
# flights that accepted the 2.0 m/s threshold (r217 to r221, commit
# 3a565f24) flew 385 to 428 m in 161 to 190 s: 2.25 to 2.40 m/s; the five
# that accepted 2.5 m/s (r261 to r265, commit 5d5dd9ea) flew 398 to 435 m
# in 144 to 172 s: 2.52 to 2.83 m/s; five on cb45e5fb (r268 to r272) flew
# 3.07 to 3.19 m/s and carried the 3.0 m/s threshold, until the body's own
# clearance was made to bound the progress floor (9ab94040, after the
# crashes r281 and r286): the five flights r288 to r292 on it flew 372 to
# 417 m in 124 to 149 s, 2.58 to 3.10 m/s, one of them above 3.0.
MINIMUM_MEAN_FLIGHT_SPEED_MPS = 2.5

MISSION_READINESS_PATTERN = (
    r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_READINESS ready=true"
)
MISSION_SUCCESS_PATTERN = (
    r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_RESULT success=true"
)
TICK_POSITION_PATTERN = (
    r"\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK [^\n]*?"
    r"state_position=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"
)


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


def validate_mean_flight_speed(ros_log: str, errors: list[str]) -> None:
    readiness = re.search(MISSION_READINESS_PATTERN, ros_log)
    result = re.search(MISSION_SUCCESS_PATTERN, ros_log)
    if readiness is None or result is None:
        errors.append(
            "FAIL: mean flight speed spans mission readiness to a successful result"
        )
        return
    started_s = float(readiness.group(1))
    finished_s = float(result.group(1))
    duration_s = finished_s - started_s
    if duration_s <= 0.0:
        errors.append("FAIL: the mission result follows mission readiness")
        return
    path_m = 0.0
    previous: tuple[float, float, float] | None = None
    samples = 0
    for tick in re.finditer(TICK_POSITION_PATTERN, ros_log):
        stamp_s = float(tick.group(1))
        if stamp_s < started_s or stamp_s > finished_s:
            continue
        position = tuple(float(tick.group(index)) for index in (2, 3, 4))
        if not all(math.isfinite(value) for value in position):
            continue
        if previous is not None:
            path_m += math.dist(previous, position)
        previous = position
        samples += 1
    if samples < 2:
        errors.append(
            "FAIL: mean flight speed has vehicle positions between readiness and result"
        )
        return
    speed_mps = path_m / duration_s
    detail = f"{speed_mps:.3f} m/s: {path_m:.1f} m in {duration_s:.1f} s"
    if speed_mps < MINIMUM_MEAN_FLIGHT_SPEED_MPS:
        errors.append(
            "FAIL: mean flight speed reaches "
            f"{MINIMUM_MEAN_FLIGHT_SPEED_MPS:.1f} m/s ({detail})"
        )
    else:
        print(f"OK: mean flight speed is {detail}")


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
        ticks = int(summary["ticks"])
        deadline_misses = int(summary["deadline_misses"])
        tick_total_p50_ms = float(summary["tick_total_p50_ms"])
        tick_total_p95_ms = float(summary["tick_total_p95_ms"])
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
    elif availability <= MINIMUM_POST_BOOTSTRAP_ROUTE_AVAILABILITY:
        errors.append(
            "FAIL: post-bootstrap route availability exceeds "
            f"{MINIMUM_POST_BOOTSTRAP_ROUTE_AVAILABILITY:.0%} "
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
    if hold_ratio >= MAXIMUM_POST_BOOTSTRAP_NO_ROUTE_HOLD_RATIO:
        errors.append(
            "FAIL: ordinary post-bootstrap no-route holds stay below "
            f"{MAXIMUM_POST_BOOTSTRAP_NO_ROUTE_HOLD_RATIO:.0%} "
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
    deadline_miss_share = deadline_misses / ticks if ticks > 0 else math.inf
    if (
        ticks <= 0
        or not math.isfinite(tick_total_p50_ms)
        or not math.isfinite(tick_total_p95_ms)
        or tick_total_p50_ms < 0.0
        or tick_total_p95_ms < tick_total_p50_ms
    ):
        errors.append("FAIL: production tick wall time has measured percentiles")
    elif (
        tick_total_p50_ms >= MAXIMUM_TICK_TOTAL_P50_MS
        or tick_total_p95_ms >= MAXIMUM_TICK_TOTAL_P95_MS
    ):
        errors.append(
            "FAIL: production tick wall time stays below "
            f"{MAXIMUM_TICK_TOTAL_P50_MS:.0f} ms at p50 and "
            f"{MAXIMUM_TICK_TOTAL_P95_MS:.0f} ms at p95 "
            f"({tick_total_p50_ms:.1f} / {tick_total_p95_ms:.1f} ms, "
            f"{deadline_miss_share:.0%} of {ticks} ticks over the 20 ms deadline)"
        )
    else:
        print(
            f"OK: production tick wall time is {tick_total_p50_ms:.1f} ms at p50 and "
            f"{tick_total_p95_ms:.1f} ms at p95 ({deadline_miss_share:.0%} of {ticks} "
            "ticks over the 20 ms deadline)"
        )
    if (
        not math.isfinite(planner_p99_ms)
        or not math.isfinite(build_and_planning_p99_ms)
        or planner_p99_ms + 1.0e-9 < planner_p95_ms
        or build_and_planning_p99_ms + 1.0e-9 < planner_p99_ms
    ):
        errors.append("FAIL: persistent planner p95/p99 latency ordering is valid")

    admitted_routes = [
        line
        for line in re.findall(r"PRODUCTION_MPPI_ROUTE3D [^\n]+", ros_log)
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


# The localization profile a flight declared and what the logs show it flew.
# lidar_inertial is the roadmap's item 13: the estimator alone, with the
# autopilot's GNSS and magnetometer fusion off and the simulation heading
# source, which is the simulator's truth, not run.
PX4_PARAMETER_SHOWN_PATTERN = r"{name} \[\d+,\d+\] : (-?[\d.]+)"
LIDAR_INERTIAL_PUBLISHED_PATTERN = re.compile(
    r"LIDAR_INERTIAL_ODOMETRY .*?published_scans=(\d+)")
MINIMUM_LIDAR_INERTIAL_PUBLISH_HZ = 5.0


def shown_px4_parameter(px4_log: str, name: str) -> float | None:
    match = None
    for match in re.finditer(PX4_PARAMETER_SHOWN_PATTERN.format(name=name), px4_log):
        pass
    return float(match.group(1)) if match is not None else None


def validate_localization_profile(manifest_path: Path, ros_log: str, px4_log: str,
                                  errors: list[str]) -> None:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = manifest.get("effective_overrides", {}).get("LOCALIZATION_PROFILE", "gnss")
    if profile == "gnss":
        print("OK: localization profile is gnss (the autopilot's GNSS and the simulated "
              "heading)")
        return
    if profile == "gnss_shadow":
        print("OK: localization profile is gnss_shadow (the lidar-inertial estimator "
              "beside GNSS, compared and not flown)")
        return
    if profile != "lidar_inertial":
        errors.append(f"FAIL: localization profile is known ({profile})")
        return
    for name, expected in (("EKF2_GPS_CTRL", 0.0), ("EKF2_MAG_TYPE", 5.0),
                           ("EKF2_EV_CTRL", 11.0)):
        shown = shown_px4_parameter(px4_log, name)
        if shown is None:
            errors.append(f"FAIL: lidar_inertial profile shows {name} in the autopilot log")
        elif shown != expected:
            errors.append(f"FAIL: lidar_inertial profile sets {name} to {expected:g} "
                          f"({shown:g})")
    if "[simulation_heading_source_node]" in ros_log:
        errors.append("FAIL: lidar_inertial profile runs without the simulation heading "
                      "source")
    published = None
    for published in LIDAR_INERTIAL_PUBLISHED_PATTERN.finditer(ros_log):
        pass
    readiness = re.search(MISSION_READINESS_PATTERN, ros_log)
    result = re.search(MISSION_SUCCESS_PATTERN, ros_log)
    if published is None:
        errors.append("FAIL: lidar_inertial profile publishes the estimator's odometry")
    elif readiness is not None and result is not None:
        span_s = float(result.group(1)) - float(readiness.group(1))
        rate_hz = int(published.group(1)) / span_s if span_s > 0.0 else 0.0
        if rate_hz < MINIMUM_LIDAR_INERTIAL_PUBLISH_HZ:
            errors.append(
                "FAIL: lidar_inertial profile publishes the estimator's odometry at "
                f"{MINIMUM_LIDAR_INERTIAL_PUBLISH_HZ:.0f} Hz or more ({rate_hz:.1f} Hz)")
        else:
            print(f"OK: localization profile is lidar_inertial: GNSS and magnetometer "
                  f"fusion off, no simulation heading source, the estimator's odometry "
                  f"at {rate_hz:.1f} Hz over the flight")
    else:
        print("OK: localization profile is lidar_inertial (the odometry rate is not "
              "measured without a successful flight)")

