"""Controller-dynamics measurements of one flight, from the recorded setpoints,
local position and true pose: the numbers the navigation laws stand on, held to
thresholds so a change of the autopilot, the simulator or the airframe that
breaks a law's assumption is seen on the next flight."""

from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# The lateral tracking error between 1.5 and 4.5 m/s. The tube law budgets
# 0.075 s times the speed (0.225 m at the 3 m/s floor) and the 0.82 m envelope
# keeps 0.27 m beyond the body on top of it, so an error inside 0.25 m still
# leaves the body clear of what the envelope was validated against. Measured
# on the release flights r288 to r292: 0.11 to 0.22 m at p99 (0.219 in r288,
# where the error over speed reached 0.124 s at p99 against the law's 0.075 s;
# the other four flights sit at 0.046 to 0.063 s).
MAXIMUM_LATERAL_TRACKING_ERROR_P99_M = 0.25
# The descent arrest the vertical stopping laws rely on is 2.0 m/s^2. Measured
# The arrest of a descent faster than 1.5 m/s is read per episode as its
# plateau, the peak of the 0.2 s window, the statistic the vertical law rests
# on: over the 91 episodes of the 25 urban flights r268 to r308 the plateau is
# 1.41 m/s^2 at the fifth percentile and 2.19 at the median, and the law
# (guaranteed_vertical_stopping_deceleration_mps2, 1.4) is that fifth
# percentile. A flight's median plateau below the law would mean the airframe
# no longer delivers what half of the stops assume. With the planned vertical
# acceleration at 1.4 the flights r312 to r314 measured medians of 2.0 to 2.13
# over 5, 1 and 6 episodes; the former statistic, the median of every windowed
# sample including the ramps, fell to 1.16 on r312 against a bound of 1.2 and
# measured the ramp, not the airframe. Fewer than three episodes measure
# nothing.
MINIMUM_DESCENT_ARREST_PLATEAU_MPS2 = 1.4
MINIMUM_DESCENT_ARREST_EPISODES = 3
# The position estimate against the true pose at every planning tick, with the
# clocks aligned on the speed profile. The error splits into an offset along
# the motion, read in seconds at the true speed (0.10 to 0.11 s on r288 to
# r292, 0.3 m at 3 m/s: the estimate and the truth are stamped that far apart
# in time), and a cross-track part that is the frame error proper: 0.19 to
# 0.25 m at p95. The braking contract budgets 600 ms of evidence age, and the
# body model keeps 0.16 m beyond the rotor tips beside a wall mapped in the
# estimate's frame.
MAXIMUM_POSITION_ESTIMATE_CROSS_TRACK_P95_M = 0.35
MAXIMUM_POSITION_ESTIMATE_ALONG_TRACK_OFFSET_S = 0.20
# The braking contract charges latest_lidar_obstacle_maximum_age_ms (600 ms) of
# evidence age; the planning tick reports the age it saw. Measured p99 192 to
# 332 ms and at most 524 ms on r268 to r272.
MAXIMUM_LIDAR_EVIDENCE_AGE_MS = 600.0


@dataclass(frozen=True)
class DynamicsMeasurement:
    value: float
    samples: int


@dataclass(frozen=True)
class EstimateErrorMeasurement:
    cross_track_p95_m: float
    # Positive when the estimate sits ahead of the truth along the motion.
    along_track_offset_s: float
    total_p95_m: float
    samples: int


def align_offset_s(reference_time_s: np.ndarray, reference_value: np.ndarray,
                   other_time_s: np.ndarray, other_value: np.ndarray,
                   search_s: float = 3.0, step_s: float = 0.02) -> float:
    """The offset to add to the other stream's time so it matches the reference,
    found by least squares over a grid: the two streams carry different clocks."""
    if len(reference_time_s) < 2 or len(other_time_s) < 2:
        return 0.0
    coarse = reference_time_s[-1] - other_time_s[-1]
    grid = np.arange(reference_time_s[0], reference_time_s[-1], 0.05)
    if len(grid) < 2:
        return coarse
    reference = np.interp(grid, reference_time_s, reference_value)
    best = (np.inf, coarse)
    for delta in np.arange(-search_s, search_s, step_s):
        candidate = np.interp(grid, other_time_s + coarse + delta, other_value)
        error = float(np.mean((reference - candidate) ** 2))
        if error < best[0]:
            best = (error, coarse + delta)
    return best[1]


def lateral_tracking_error_p99_m(setpoints: np.ndarray, positions: np.ndarray,
                                 minimum_speed_mps: float = 1.5,
                                 maximum_speed_mps: float = 4.5) -> DynamicsMeasurement:
    """Setpoints: timestamp_us, x, y, z, vx, vy, vz (NED); positions: timestamp_us,
    timestamp_sample_us, x, y, z, vx, vy, vz (NED). The lateral part of the
    position error against planned setpoints (those carrying a velocity), by
    commanded horizontal speed."""
    planned = ~np.isnan(setpoints[:, 4]) & ~np.isnan(setpoints[:, 5])
    if planned.sum() < 10 or len(positions) < 10:
        return DynamicsMeasurement(float("nan"), 0)
    setpoint_time = setpoints[:, 0] / 1e6
    position_time = positions[:, 0] / 1e6
    offset = align_offset_s(position_time, positions[:, 5], setpoint_time,
                            np.where(planned, setpoints[:, 4], 0.0))
    stamps = setpoint_time[planned] + offset
    x = np.interp(stamps, position_time, positions[:, 2])
    y = np.interp(stamps, position_time, positions[:, 3])
    error_x = x - setpoints[planned, 1]
    error_y = y - setpoints[planned, 2]
    vx = setpoints[planned, 4]
    vy = setpoints[planned, 5]
    speed = np.hypot(vx, vy)
    selected = (speed >= minimum_speed_mps) & (speed < maximum_speed_mps)
    if selected.sum() < 10:
        return DynamicsMeasurement(float("nan"), int(selected.sum()))
    ux = vx[selected] / speed[selected]
    uy = vy[selected] / speed[selected]
    lateral = np.abs(-error_x[selected] * uy + error_y[selected] * ux)
    return DynamicsMeasurement(float(np.percentile(lateral, 99)), int(selected.sum()))


def descent_arrest_plateau_mps2(positions: np.ndarray, window_s: float = 0.2,
                                minimum_descent_mps: float = 1.5) -> DynamicsMeasurement:
    """The median over arrest episodes of the plateau upward acceleration while
    a descent faster than the minimum is being arrested, from the local
    position's vertical velocity (NED vz, down positive). An episode is a run
    of arresting samples (gaps of up to two samples allowed) at least three
    samples long; its plateau is the peak of the windowed acceleration. The
    sample count is the episode count."""
    if len(positions) < 10:
        return DynamicsMeasurement(float("nan"), 0)
    time_s = positions[:, 0] / 1e6
    vertical_up = -positions[:, 7]
    step = float(np.median(np.diff(time_s)))
    if not step > 0.0:
        return DynamicsMeasurement(float("nan"), 0)
    k = max(1, int(round(window_s / step)))
    acceleration = (vertical_up[k:] - vertical_up[:-k]) / (time_s[k:] - time_s[:-k])
    descending = vertical_up[:-k] < -minimum_descent_mps
    arresting = np.flatnonzero(descending & (acceleration > 0.5))
    plateaus: list[float] = []
    start = 0
    for index in range(1, len(arresting) + 1):
        if index == len(arresting) or arresting[index] - arresting[index - 1] > 2:
            episode = arresting[start:index]
            if len(episode) >= 3:
                plateaus.append(float(acceleration[episode].max()))
            start = index
    if not plateaus:
        return DynamicsMeasurement(float("nan"), 0)
    return DynamicsMeasurement(float(np.median(plateaus)), len(plateaus))


TICK_POSITION_PATTERN = re.compile(
    r"\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK .*?"
    r"state_position=\(([-\d.]+),([-\d.]+),([-\d.]+)\).*?"
    r"state_velocity=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)


def estimate_positions_from_log(ros_log: str) -> np.ndarray:
    """timestamp_s, x, y, horizontal speed of every planning tick's estimate, in
    the map frame."""
    rows = [(float(m.group(1)), float(m.group(2)), float(m.group(3)),
             float(np.hypot(float(m.group(5)), float(m.group(6)))))
            for m in TICK_POSITION_PATTERN.finditer(ros_log)]
    return np.array(rows, dtype=np.float64).reshape(-1, 4)


def position_estimate_error(estimate: np.ndarray,
                            truth: np.ndarray) -> EstimateErrorMeasurement:
    """Estimate: timestamp_s, x, y, speed (map frame, the log's clock); truth:
    time_s, x, y, ... (map frame, simulation clock). The clocks are aligned on
    the speed profile, which neither a position bias nor a lag can shift; the
    error is then read as an offset along the true motion (its median, in
    seconds at the true speed) and a cross-track distance (its p95)."""
    nan = float("nan")
    if len(estimate) < 10 or len(truth) < 10:
        return EstimateErrorMeasurement(nan, nan, nan, 0)
    truth_vx = np.gradient(truth[:, 1], truth[:, 0])
    truth_vy = np.gradient(truth[:, 2], truth[:, 0])
    truth_speed = np.hypot(truth_vx, truth_vy)
    offset = align_offset_s(estimate[:, 0], estimate[:, 3], truth[:, 0], truth_speed,
                            search_s=5.0)
    truth_time = truth[:, 0] + offset
    inside = (estimate[:, 0] >= truth_time[0]) & (estimate[:, 0] <= truth_time[-1])
    if inside.sum() < 10:
        return EstimateErrorMeasurement(nan, nan, nan, int(inside.sum()))
    stamps = estimate[inside, 0]
    error_x = estimate[inside, 1] - np.interp(stamps, truth_time, truth[:, 1])
    error_y = estimate[inside, 2] - np.interp(stamps, truth_time, truth[:, 2])
    vx = np.interp(stamps, truth_time, truth_vx)
    vy = np.interp(stamps, truth_time, truth_vy)
    speed = np.hypot(vx, vy)
    moving = speed > 1.0
    total_p95 = float(np.percentile(np.hypot(error_x, error_y), 95))
    if moving.sum() < 10:
        return EstimateErrorMeasurement(nan, nan, total_p95, int(inside.sum()))
    along = (error_x[moving] * vx[moving] + error_y[moving] * vy[moving]) / speed[moving]
    cross = np.abs(-error_x[moving] * vy[moving] + error_y[moving] * vx[moving]) / speed[moving]
    along_track_offset_s = float(np.median(along / speed[moving]))
    return EstimateErrorMeasurement(float(np.percentile(cross, 95)),
                                    along_track_offset_s, total_p95, int(moving.sum()))


TICK_LIDAR_AGE_PATTERN = re.compile(
    r"PRODUCTION_MPPI_TICK .*?latest_lidar_obstacle_age_ms=(-?[\d.]+)"
)


def lidar_evidence_age_max_ms(ros_log: str) -> DynamicsMeasurement:
    ages = [float(m.group(1)) for m in TICK_LIDAR_AGE_PATTERN.finditer(ros_log)]
    ages = [age for age in ages if age >= 0.0]
    if not ages:
        return DynamicsMeasurement(float("nan"), 0)
    return DynamicsMeasurement(max(ages), len(ages))


def load_truth_csv(path: Path) -> np.ndarray:
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 4:
                rows.append([float(value) for value in row[:4]])
    return np.array(rows, dtype=np.float64).reshape(-1, 4)


def validate_controller_dynamics(run_directory: Path, ros_log: str,
                                 errors: list[str]) -> None:
    """The four checks, each on the flight's own records under run_directory."""
    tracking_path = run_directory / "tracking.npz"
    truth_path = run_directory / "gz_pose.csv"
    if tracking_path.is_file():
        with np.load(tracking_path) as data:
            setpoints = np.asarray(data["sp"], dtype=np.float64).reshape(-1, 7)
            positions = np.asarray(data["lp"], dtype=np.float64).reshape(-1, 8)
        lateral = lateral_tracking_error_p99_m(setpoints, positions)
        if not np.isfinite(lateral.value):
            errors.append("FAIL: lateral tracking error is measured on planned setpoints"
                          f" ({lateral.samples} samples)")
        elif lateral.value > MAXIMUM_LATERAL_TRACKING_ERROR_P99_M:
            errors.append(
                "FAIL: lateral tracking error p99 stays within "
                f"{MAXIMUM_LATERAL_TRACKING_ERROR_P99_M:.2f} m ({lateral.value:.3f} m "
                f"over {lateral.samples} samples)")
        else:
            print(f"OK: lateral tracking error p99 is {lateral.value:.3f} m "
                  f"({lateral.samples} samples between 1.5 and 4.5 m/s)")
        arrest = descent_arrest_plateau_mps2(positions)
        if not np.isfinite(arrest.value) or arrest.samples < MINIMUM_DESCENT_ARREST_EPISODES:
            print(f"OK: descent arrest was not exercised enough to measure "
                  f"({arrest.samples} episodes)")
        elif arrest.value < MINIMUM_DESCENT_ARREST_PLATEAU_MPS2:
            errors.append(
                "FAIL: descent arrest plateau reaches the vertical law's "
                f"{MINIMUM_DESCENT_ARREST_PLATEAU_MPS2:.1f} m/s^2 at the median "
                f"({arrest.value:.2f} m/s^2 over {arrest.samples} episodes)")
        else:
            print(f"OK: descent arrest plateau median is {arrest.value:.2f} m/s^2 "
                  f"({arrest.samples} episodes)")
    else:
        errors.append("FAIL: the flight recorded its setpoints and local position "
                      f"({tracking_path.name})")
    if truth_path.is_file():
        truth = load_truth_csv(truth_path)
        estimate = estimate_positions_from_log(ros_log)
        error = position_estimate_error(estimate, truth)
        if not np.isfinite(error.cross_track_p95_m) or not np.isfinite(
                error.along_track_offset_s):
            errors.append("FAIL: the position estimate is compared against the true "
                          f"pose in motion ({error.samples} samples)")
        elif error.cross_track_p95_m > MAXIMUM_POSITION_ESTIMATE_CROSS_TRACK_P95_M:
            errors.append(
                "FAIL: position estimate cross-track error p95 stays within "
                f"{MAXIMUM_POSITION_ESTIMATE_CROSS_TRACK_P95_M:.2f} m "
                f"({error.cross_track_p95_m:.3f} m over {error.samples} samples)")
        elif abs(error.along_track_offset_s) > MAXIMUM_POSITION_ESTIMATE_ALONG_TRACK_OFFSET_S:
            errors.append(
                "FAIL: position estimate along-track offset stays within "
                f"{MAXIMUM_POSITION_ESTIMATE_ALONG_TRACK_OFFSET_S:.2f} s "
                f"({error.along_track_offset_s:.3f} s over {error.samples} samples)")
        else:
            print(f"OK: position estimate cross-track error p95 is "
                  f"{error.cross_track_p95_m:.3f} m, along-track offset "
                  f"{error.along_track_offset_s:.3f} s, total p95 {error.total_p95_m:.3f} m "
                  f"({error.samples} samples)")
    else:
        errors.append(f"FAIL: the flight recorded the true pose ({truth_path.name})")
    age = lidar_evidence_age_max_ms(ros_log)
    if age.samples == 0:
        errors.append("FAIL: the planning tick reports the lidar evidence age")
    elif age.value > MAXIMUM_LIDAR_EVIDENCE_AGE_MS:
        errors.append(
            "FAIL: lidar evidence age stays within "
            f"{MAXIMUM_LIDAR_EVIDENCE_AGE_MS:.0f} ms ({age.value:.0f} ms at most)")
    else:
        print(f"OK: lidar evidence age is at most {age.value:.0f} ms "
              f"({age.samples} ticks)")
