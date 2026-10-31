#!/usr/bin/env python3
"""What the hybrid profile's trust in the satellite signal did over a flight
(roadmap item 39): read from the estimator's GNSS_TRUST lines by the mission
check (headless_runtime_evidence.py)."""

from __future__ import annotations

import re

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
