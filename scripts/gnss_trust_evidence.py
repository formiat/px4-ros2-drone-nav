#!/usr/bin/env python3
"""What the hybrid profile's trust in the satellite signal did over a flight
(roadmap item 39): read from the estimator's GNSS_TRUST lines by the mission
check (headless_runtime_evidence.py)."""

from __future__ import annotations

import csv
import re
from pathlib import Path

# The trust law reads a disagreeing fix on the next fix (33 ms at 30 Hz);
# the record of the trust comes once a second and the relay's record of the
# fault once a second of airborne time: the loss of trust is expected within
# this many seconds of the fault's start (T of stage 1.2a: 0.3 to 0.7 s on
# r2626 to r2628, replayed; plus the two records' granularity).
TRUST_LOSS_ALLOWANCE_S = 2.0
AIRBORNE_PATTERN = re.compile(r"GNSS_FAULT_RELAY airborne_at_us=(\d+)")

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


def report_gnss_faults(run_directory: Path, ros_log: str, errors: list[str],
                       notes: list[str] | None) -> None:
    """Р1.6 of roadmap item 39: under every fault the relay played, the trust
    is lost no later than the allowance after the fault's start and not
    regained before its end (never, after a fault whose effect stays)."""
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
            errors.append(f"FAIL: the trust stays lost until the fault {span} ends (back "
                          + ("although the fault stays" if stays else
                             f"{end_s - early[0]:.1f} s before the end") + ")")
            continue
        lost = (f"lost {losses[0] - start_s:.1f} s after the start" if trusted_at_start
                else "untrusted already at the start")
        back = ("not back, as the fault stays" if stays else
                f"back {returns[0] - end_s:.1f} s after the end" if returns else
                "not back by the flight's end")
        print(f"OK: under the fault {span}: {lost}, {back}")
