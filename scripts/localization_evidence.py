#!/usr/bin/env python3
"""The estimators' checks of a run (headless_runtime_evidence.py re-exports
them): the localization profile the manifest names against what the
autopilot and the estimator logged, the visual-inertial and lidar-inertial
estimators' publication and health, the published position uncertainty a goal
in the void is judged by, and the mission's readiness and success lines every
check reads. Moved whole from headless_runtime_evidence.py on 2026-11-02 (the
source size contract)."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path


MISSION_READINESS_PATTERN = (
    r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_READINESS ready=true"
)


MISSION_SUCCESS_PATTERN = (
    r"\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_RESULT success=true"
)


# The localization profile a flight declared and what the logs show it flew.
# lidar_inertial is the roadmap's item 13: the estimator alone, with the
# autopilot's GNSS and magnetometer fusion off and the simulation heading
# source, which is the simulator's truth, not run.
PX4_PARAMETER_SHOWN_PATTERN = r"{name} \[\d+,\d+\] : (-?[\d.]+)"


LIDAR_INERTIAL_PUBLISHED_PATTERN = re.compile(
    r"LIDAR_INERTIAL_ODOMETRY .*?published_scans=(\d+)")


MINIMUM_LIDAR_INERTIAL_PUBLISH_HZ = 5.0


# The estimator's health once a second: the share of the scan that matched
# the submap, the registration's residual, the information along its weakest
# axis per matched point, and the running counts of scans and healthy ones.
LIDAR_INERTIAL_HEALTH_PATTERN = re.compile(
    r"\[(\d+\.\d+)\] \[lidar_inertial_odometry_node\]: LIDAR_INERTIAL_ODOMETRY "
    r".*?matched=([\d.]+) residual_m=([\d.]+) information=([\d.]+) "
    r".*? scans=(\d+) healthy_scans=(\d+)")


def shown_px4_parameter(px4_log: str, name: str) -> float | None:
    match = None
    for match in re.finditer(PX4_PARAMETER_SHOWN_PATTERN.format(name=name), px4_log):
        pass
    return float(match.group(1)) if match is not None else None


def validate_localization_profile(manifest_path: Path, ros_log: str, px4_log: str,
                                  errors: list[str],
                                  notes: list[str] | None = None) -> None:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = manifest.get("effective_overrides", {}).get("LOCALIZATION_PROFILE",
                                                          "lidar_inertial")
    if profile == "gnss":
        print("OK: localization profile is gnss (the autopilot's GNSS and the simulated "
              "heading)")
        return
    if profile == "gnss_shadow":
        print("OK: localization profile is gnss_shadow (the lidar-inertial estimator "
              "beside GNSS, compared and not flown)")
        return
    if profile == "visual_inertial_shadow":
        print("OK: localization profile is visual_inertial_shadow (the visual-inertial "
              "estimator beside GNSS, compared and not flown)")
        readiness = re.search(MISSION_READINESS_PATTERN, ros_log)
        result = re.search(MISSION_SUCCESS_PATTERN, ros_log)
        if readiness is not None and result is not None and notes is not None:
            report_visual_inertial_health(ros_log, float(readiness.group(1)),
                                          float(result.group(1)), notes)
        return
    # The log is stamped on the wall clock; a flight slowed against it
    # (REAL_TIME_FACTOR) publishes at its rate per second of simulation.
    real_time_factor = float(manifest.get("effective_overrides", {}).get("REAL_TIME_FACTOR",
                                                                          1.0))
    if profile in ("visual_inertial", "hybrid"):
        validate_visual_inertial_profile(ros_log, px4_log, errors, notes, real_time_factor)
        if profile == "hybrid":
            from gnss_trust_evidence import report_gnss_faults, report_gnss_trust  # noqa: PLC0415

            report_gnss_trust(ros_log, notes)
            faults = manifest.get("effective_overrides", {}).get("GNSS_FAULTS", "")
            if faults:
                report_gnss_faults(manifest_path.parent, ros_log, errors, notes, faults)
        return
    if profile != "lidar_inertial":
        errors.append(f"FAIL: localization profile is known ({profile})")
        return
    for name, expected in (("EKF2_GPS_CTRL", 0.0), ("EKF2_MAG_TYPE", 5.0),
                           ("EKF2_EV_CTRL", 11.0), ("EKF2_HGT_REF", 3.0)):
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
        span_s = (float(result.group(1)) - float(readiness.group(1))) * real_time_factor
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
    if readiness is not None and result is not None:
        validate_lidar_inertial_health(ros_log, float(readiness.group(1)),
                                       float(result.group(1)), errors)


VISUAL_INERTIAL_PUBLISHED_PATTERN = re.compile(
    r"VISUAL_INERTIAL_ODOMETRY .*?published_poses=(\d+)")


def validate_visual_inertial_profile(ros_log: str, px4_log: str, errors: list[str],
                                     notes: list[str] | None,
                                     real_time_factor: float = 1.0) -> None:
    """That the flight flew what the profile says: the visual-inertial estimator
    in the autopilot's place for GNSS and compass, and no simulator truth as the
    heading. The estimator's quality is a note; what it answers for is the goal
    reached in truth."""
    for name, expected in (("EKF2_GPS_CTRL", 0.0), ("EKF2_MAG_TYPE", 5.0),
                           ("EKF2_EV_CTRL", 11.0), ("EKF2_HGT_REF", 3.0)):
        shown = shown_px4_parameter(px4_log, name)
        if shown is None:
            errors.append(f"FAIL: visual_inertial profile shows {name} in the autopilot "
                          "log")
        elif shown != expected:
            errors.append(f"FAIL: visual_inertial profile sets {name} to {expected:g} "
                          f"({shown:g})")
    if "[simulation_heading_source_node]" in ros_log:
        errors.append("FAIL: visual_inertial profile runs without the simulation heading "
                      "source")
    if "[lidar_inertial_odometry_node]" in ros_log:
        errors.append("FAIL: visual_inertial profile runs without the lidar-inertial "
                      "estimator")
    published = None
    for published in VISUAL_INERTIAL_PUBLISHED_PATTERN.finditer(ros_log):
        pass
    readiness = re.search(MISSION_READINESS_PATTERN, ros_log)
    result = re.search(MISSION_SUCCESS_PATTERN, ros_log)
    if published is None or int(published.group(1)) == 0:
        errors.append("FAIL: visual_inertial profile publishes the estimator's odometry")
    elif readiness is not None and result is not None:
        span_s = (float(result.group(1)) - float(readiness.group(1))) * real_time_factor
        rate_hz = int(published.group(1)) / span_s if span_s > 0.0 else 0.0
        print(f"OK: localization profile is visual_inertial: GNSS and magnetometer "
              f"fusion off, no simulation heading source, {published.group(1)} poses "
              f"to the autopilot ({rate_hz:.1f} per second of flight)")
        if notes is not None:
            report_visual_inertial_health(ros_log, float(readiness.group(1)),
                                          float(result.group(1)), notes)
    else:
        print("OK: localization profile is visual_inertial (no successful flight to "
              "measure the odometry over)")


# The visual-inertial estimator's health once a second.
VISUAL_INERTIAL_HEALTH_PATTERN = re.compile(
    r"\[(\d+\.\d+)\] \[visual_inertial_odometry_node\]: VISUAL_INERTIAL_ODOMETRY "
    r"healthy=(\w+) tracked=(\d+) candidates=(\d+) used=(\d+) gated=(\d+) "
    r"untriangulated=(\d+) residual_sigma=([\d.]+) weakest_velocity_sigma_mps=([\d.]+) "
    r".*? frame_ms=([\d.]+) imu_lag_ms=([\d.]+) imu_gap_max_ms=([\d.]+) "
    r".*? frames=(\d+) frames_without_estimate=(\d+)")


def report_visual_inertial_health(ros_log: str, start_s: float, end_s: float,
                                  notes: list[str]) -> None:
    """What the estimator says of itself over the flight. Nothing here fails a
    flight: the requirement the estimate answers for is the goal reached in
    truth."""
    samples = [match for match in VISUAL_INERTIAL_HEALTH_PATTERN.finditer(ros_log)
               if start_s <= float(match.group(1)) <= end_s]
    if len(samples) < 2:
        notes.append("NOTE: visual-inertial estimator health is reported over the flight "
                     f"({len(samples)} reports)")
        return
    column = lambda index: sorted(float(match.group(index)) for match in samples)
    median = lambda values: values[len(values) // 2]
    tracked, residual, information = column(3), column(8), column(9)
    frame_ms, gap_ms = column(10), column(12)
    candidates = sum(int(match.group(4)) for match in samples)
    gated = sum(int(match.group(6)) for match in samples)
    untriangulated = sum(int(match.group(7)) for match in samples)
    frames = int(samples[-1].group(13)) - int(samples[0].group(13))
    without = int(samples[-1].group(14)) - int(samples[0].group(14))
    summary = (f"tracked features p50 {median(tracked):.0f} min {tracked[0]:.0f}, residual "
               f"p50 {median(residual):.2f} sigma, least certain velocity direction "
               f"at most {information[-1]:.3f} m/s, gated {gated} and untriangulated "
               f"{untriangulated} of {candidates} sampled features, frame cost p50 "
               f"{median(frame_ms):.1f} "
               f"ms max {frame_ms[-1]:.1f} ms, IMU gap max {gap_ms[-1]:.0f} ms, "
               f"{without} of {frames} frames without a healthy estimate")
    if without > 0:
        notes.append(f"NOTE: visual-inertial estimate stays healthy through the flight "
                     f"({summary})")
    else:
        print(f"OK: visual-inertial estimator health: {summary}")


POSITION_SIGMA_PATTERN = re.compile(
    r"\[(\d+\.\d+)\] \[visual_inertial_odometry_node\]: VISUAL_INERTIAL_ODOMETRY "
    r"[^\n]*? position_sigma_m=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)")


def published_position_sigma_m(ros_log: str, at_s: float) -> float | None:
    """The estimator's horizontal position sigma from its latest status line
    at or before `at_s` (the lines come once a second)."""
    sigma = None
    for match in POSITION_SIGMA_PATTERN.finditer(ros_log):
        if float(match.group(1)) > at_s:
            break
        sigma = math.hypot(float(match.group(2)), float(match.group(3)))
    return sigma


def validate_lidar_inertial_health(ros_log: str, start_s: float, end_s: float,
                                   errors: list[str]) -> None:
    """The estimator's quality over the flight, and the scans that left the
    autopilot without an estimate: an unhealthy scan is not published, so in
    a clean flight there are none."""
    samples = [match for match in LIDAR_INERTIAL_HEALTH_PATTERN.finditer(ros_log)
               if start_s <= float(match.group(1)) <= end_s]
    if len(samples) < 2:
        errors.append("FAIL: lidar-inertial estimator health is reported over the flight "
                      f"({len(samples)} reports)")
        return
    column = lambda index: sorted(float(match.group(index)) for match in samples)
    matched, residual, information = column(2), column(3), column(4)
    scans = int(samples[-1].group(5)) - int(samples[0].group(5))
    unhealthy = scans - (int(samples[-1].group(6)) - int(samples[0].group(6)))
    median = lambda values: values[len(values) // 2]
    summary = (f"matched share p50 {median(matched):.2f} min {matched[0]:.2f}, residual "
               f"p50 {median(residual):.3f} m max {residual[-1]:.3f} m, weakest-axis "
               f"information p50 {median(information):.3f} min {information[0]:.3f}, "
               f"{unhealthy} of {scans} scans without a valid estimate")
    if unhealthy > 0:
        errors.append(f"FAIL: lidar-inertial estimate stays valid through the flight "
                      f"({summary})")
    else:
        print(f"OK: lidar-inertial estimator health: {summary}")
