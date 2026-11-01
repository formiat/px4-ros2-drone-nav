#!/usr/bin/env python3
"""What the hybrid profile's trust in the satellite signal did over a flight
(roadmap item 39): read from the estimator's GNSS_TRUST lines by the mission
check (headless_runtime_evidence.py)."""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path

# The trust law reads a disagreeing fix on the next fix (33 ms at 30 Hz);
# the record of the trust comes once a second and the relay's record of the
# fault once a second of airborne time: the loss of trust is expected within
# this many seconds of the fault's start (T of stage 1.2a: 0.3 to 0.7 s on
# r2626 to r2628, replayed; plus the two records' granularity).
TRUST_LOSS_ALLOWANCE_S = 2.0
AIRBORNE_PATTERN = re.compile(r"GNSS_FAULT_RELAY airborne_at_us=(\d+)")
# A fix lying about its velocity lies by the vehicle's speed: under this speed
# the lie stays inside the velocity gate (1.26 normalised over the fix's and
# the estimator's uncertainties, 0.4 to 1.1 m/s), and the law has nothing to
# read (r2659: the trust returned at 0.6 to 0.8 m/s and left again at 1.1).
BAD_VELOCITY_READABLE_MPS = 1.0
# A fix back at an offset is taken back once the estimator's own drift since
# the outage could explain it: the residual read against the position gate
# (0.70) over the three axes' mean (one axis counts 1/sqrt(3)) of the drift
# allowance the estimator lends itself, and the estimator may have drifted
# toward the offset by that allowance — an offset of up to (0.70 sqrt(3) + 1)
# allowances is the law's own limit (r2661: 50 m taken back at 33 m of
# allowance, the residual 40 m).
POSITION_GATE = 0.70
DRIFT_PATTERN = re.compile(r"\[(\d+\.\d+)\].*GNSS_TRUST trusted=.* drift_m=([0-9.]+)")

GNSS_TRUST_PATTERN = re.compile(
    r"GNSS_TRUST trusted=(true|false) agreeing=(true|false) agreeing_s=([0-9.]+) reason=(\w+) "
    r"residual_m=([0-9.]+) normalised=([0-9.]+) velocity_mps=([0-9.]+) "
    r"velocity_normalised=([0-9.]+) self_mps=([0-9.]+) step_m=([0-9.]+) clock_s=([-0-9.]+) "
    r"fix_type=(\d+) eph=([0-9.]+) fused=(\d+)")
GNSS_TRUST_CHANGE_PATTERN = re.compile(
    r"\[(\d+\.\d+)\].*GNSS_TRUST changed trusted=(true|false) reason=(\w+)")


def report_gnss_trust(ros_log: str, notes: list[str] | None) -> None:
    """What the hybrid profile's trust in the satellite signal did over the
    flight (roadmap item 39): the fixes' residuals against the estimator, the
    moments the trust changed, the fixes fused. A record, not a gate; the
    gates themselves are the scenario's (stage 2)."""
    lines = GNSS_TRUST_PATTERN.findall(ros_log)
    changes = GNSS_TRUST_CHANGE_PATTERN.findall(ros_log)
    if not lines:
        if notes is not None:
            notes.append("NOTE: the hybrid profile logged no GNSS_TRUST line")
        return
    residuals = sorted(float(line[4]) for line in lines)
    normalised = sorted(float(line[5]) for line in lines)
    velocities = sorted(float(line[6]) for line in lines)
    selfs = sorted(float(line[8]) for line in lines)
    steps = sorted(float(line[9]) for line in lines)
    trusted = sum(1 for line in lines if line[0] == "true")
    fused = int(lines[-1][13])

    def quantile(values: list[float], share: float) -> float:
        return values[min(len(values) - 1, int(share * (len(values) - 1)))]

    print(f"OK: GNSS trust over {len(lines)} seconds: trusted {trusted}, residual m p50 "
          f"{quantile(residuals, 0.5):.2f} p99 {quantile(residuals, 0.99):.2f} max "
          f"{residuals[-1]:.2f}, normalised p50 {quantile(normalised, 0.5):.2f} p99 "
          f"{quantile(normalised, 0.99):.2f} max {normalised[-1]:.2f}, velocity mps p99 "
          f"{quantile(velocities, 0.99):.2f}, self mps p99 {quantile(selfs, 0.99):.2f}, step m p99 "
          f"{quantile(steps, 0.99):.2f}, fixes fused {fused}")
    for stamp, state, reason in changes:
        print(f"OK: GNSS trust changed to {state} ({reason}) at {float(stamp):.1f}")


def fault_windows(run_directory: Path) -> list[tuple[str, float, float, bool]] | None:
    """The faults the relay played, on the wall clock of the estimator's log:
    (variant, start, end, stays) from the relay's record (one row per second
    of airborne time, the active variants' names joined by "+"), the airborne
    moment from its log, the simulation clock read against the wall from the
    true pose's record. `stays`: the fault's effect outlives its window (the
    offset return_offset leaves behind, which the relay records as
    "returned_offset" to the flight's end). None when a record is missing."""
    relay = run_directory / "gnss_relay.csv"
    relay_log = run_directory / "gnss_relay.log"
    truth = run_directory / "gz_pose.csv"
    if not (relay.is_file() and relay_log.is_file() and truth.is_file()):
        return None
    airborne = AIRBORNE_PATTERN.search(relay_log.read_text(encoding="utf-8", errors="replace"))
    if airborne is None:
        return None
    airborne_s = int(airborne.group(1)) * 1.0e-6
    clock: list[tuple[float, float]] = []
    with truth.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 9:
                clock.append((float(row[0]), float(row[8])))
    if len(clock) < 2:
        return None

    def wall(sim_s: float) -> float:
        nearest = min(clock, key=lambda pair: abs(pair[0] - sim_s))
        return nearest[1] + (sim_s - nearest[0])

    spans: dict[str, tuple[float, float | None]] = {}
    with relay.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            t_s = float(row["t_airborne_s"])
            names = set(row["active"].split("+")) - {"none", ""}
            for name in names:
                spans.setdefault(name, (t_s, None))
            for name, (begun_s, ended_s) in spans.items():
                if ended_s is None and name not in names:
                    spans[name] = (begun_s, t_s)
    windows = []
    for name, (begun_s, ended_s) in spans.items():
        if name == "returned_offset":
            continue
        stays = ended_s is None or (name == "return_offset" and "returned_offset" in spans)
        windows.append((name, wall(airborne_s + begun_s),
                        float("inf") if stays else wall(airborne_s + ended_s), stays))
    return windows


def truth_speed_mps(run_directory: Path, wall_s: float) -> float | None:
    """The vehicle's true speed over the second before `wall_s` (gz_pose.csv)."""
    rows = []
    with (run_directory / "gz_pose.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 9 and wall_s - 1.2 <= float(row[8]) <= wall_s:
                rows.append((float(row[8]), float(row[1]), float(row[2])))
    if len(rows) < 2:
        return None
    (t0, x0, y0), (t1, x1, y1) = rows[0], rows[-1]
    return math.hypot(x1 - x0, y1 - y0) / (t1 - t0) if t1 > t0 else None


def fault_magnitude(faults: str, variant: str) -> float:
    """The variant's magnitude in the schedule (its default when not given)."""
    defaults = {"return_offset": 50.0, "walk": 1.0, "jump": 100.0, "degraded": 10.0, "replay": 10.0}
    for item in faults.split(";"):
        name, _, rest = item.strip().partition("@")
        fields = rest.split(":")
        if name == variant:
            return float(fields[2]) if len(fields) > 2 else defaults.get(variant, 0.0)
    return defaults.get(variant, 0.0)


def allowed_return(run_directory: Path, ros_log: str, variant: str, at_s: float,
                   faults: str) -> str | None:
    """Why the law may take the signal back before the fault ends, or None."""
    if variant == "bad_velocity":
        speed = truth_speed_mps(run_directory, at_s)
        if speed is not None and speed < BAD_VELOCITY_READABLE_MPS:
            return (f"the fix lies about a motion of {speed:.2f} m/s, under the "
                    f"{BAD_VELOCITY_READABLE_MPS:.1f} m/s the velocity gate reads")
    if variant == "return_offset":
        drift = [float(d) for stamp, d in DRIFT_PATTERN.findall(ros_log) if float(stamp) <= at_s]
        offset = fault_magnitude(faults, variant)
        if drift and offset <= (POSITION_GATE * math.sqrt(3.0) + 1.0) * drift[-1]:
            return (f"the offset of {offset:.0f} m is within the drift the estimator could have accrued "
                    f"since the outage ({drift[-1]:.1f} m of allowance, the gate {POSITION_GATE} over "
                    f"the three axes plus the allowance itself): no second absolute source tells them apart")
    return None


def report_gnss_faults(run_directory: Path, ros_log: str, errors: list[str],
                       notes: list[str] | None, faults: str = "") -> None:
    """Р1.6 of roadmap item 39: under every fault the relay played, the trust
    is lost no later than the allowance after the fault's start and not
    regained before its end (never, after a fault whose effect stays). Two
    returns the law allows are notes, not failures: a velocity lie under the
    gate while the vehicle is slow, and an offset the estimator's own drift
    since the outage could explain."""
    windows = fault_windows(run_directory)
    if windows is None:
        errors.append("FAIL: the relay's record, its log and the true pose's record name the "
                      "faults' moments on the wall clock")
        return
    if not windows:
        if notes is not None:
            notes.append("NOTE: the relay played no fault")
        return
    changes = sorted((float(stamp), state == "true") for stamp, state, _ in
                     GNSS_TRUST_CHANGE_PATTERN.findall(ros_log))
    for name, start_s, end_s, stays in windows:
        before = [state for stamp, state in changes if stamp <= start_s]
        trusted_at_start = bool(before) and before[-1]
        losses = [stamp for stamp, state in changes if not state and stamp >= start_s]
        returns = [stamp for stamp, state in changes if state and stamp > start_s]
        span = f"{name} from {start_s:.1f}" + (" on" if stays else f" to {end_s:.1f}")
        if trusted_at_start and (not losses or losses[0] - start_s > TRUST_LOSS_ALLOWANCE_S):
            errors.append(f"FAIL: the trust is lost within {TRUST_LOSS_ALLOWANCE_S:.0f} s of the fault "
                          f"{span} (" + (f"lost {losses[0] - start_s:.1f} s after" if losses
                                        else "never lost") + ")")
            continue
        early = [stamp for stamp in returns if stamp < end_s]
        if early:
            allowed = allowed_return(run_directory, ros_log, name, early[0], faults)
            if allowed is None:
                errors.append(f"FAIL: the trust stays lost until the fault {span} ends (back "
                              + ("although the fault stays" if stays else
                                 f"{end_s - early[0]:.1f} s before the end") + ")")
                continue
            if notes is not None:
                notes.append(f"NOTE: under the fault {span} the trust came back at {early[0]:.1f}: {allowed}")
        lost = (f"lost {losses[0] - start_s:.1f} s after the start" if trusted_at_start
                else "untrusted already at the start")
        back = ("not back, as the fault stays" if stays and not early else
                f"back {end_s - early[0]:.1f} s before the end, as the law allows" if early and not stays else
                "back although the fault stays, as the law allows" if early else
                f"back {returns[0] - end_s:.1f} s after the end" if returns else
                "not back by the flight's end")
        print(f"OK: under the fault {span}: {lost}, {back}")
