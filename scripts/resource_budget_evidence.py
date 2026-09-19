#!/usr/bin/env python3
"""The resource record of one flight (resources.csv, resources_host.json, from
scripts/capture_process_resources.py) held against the headless mission check:
the record has to cover the flight, and what the onboard processes consumed
is reported so a change of their cost is seen on the next flight.
"""

from __future__ import annotations

import csv
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from headless_runtime_evidence import MISSION_READINESS_PATTERN, MISSION_SUCCESS_PATTERN

# The transport hops: what each consumer measured of its deliveries. The
# controller's tick line splits the observation age it reports into the
# producer's period and build, the delivery and the wait for the tick.
TICK_TRANSPORT_PATTERN = re.compile(
    r"\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK .*?"
    r"observation_age_ms=(-?[\d.]+) raw_delivery_ms=(-?[\d.]+|nan) "
    r"raw_receive_age_ms=(-?[\d.]+) lidar_delivery_ms=(-?[\d.]+|nan)")
SUMMARY_TRANSPORT_PATTERN = re.compile(
    r"PRODUCTION_MPPI_SUMMARY .*?raw_delivery_samples=(\d+) raw_delivery_p50_ms=([\d.]+|nan) "
    r"raw_delivery_p95_ms=([\d.]+|nan) raw_delivery_max_ms=([\d.]+|nan) "
    r"lidar_delivery_samples=(\d+) lidar_delivery_p50_ms=([\d.]+|nan) "
    r"lidar_delivery_p95_ms=([\d.]+|nan) lidar_delivery_max_ms=([\d.]+|nan)")
HOP_REPORT_PATTERN = (
    r"delivery_ms=(?:[\d.]+|nan) delivery_p50_ms=([\d.]+|nan) delivery_p95_ms=([\d.]+|nan) "
    r"delivery_max_ms=([\d.]+|nan) delivery_samples=(\d+)")
CLOUD_TRANSPORT_PATTERN = re.compile(r"LIDAR3D_ALIGNMENT dropped=false .*?" + HOP_REPORT_PATTERN)
HORIZON_TRANSPORT_PATTERN = re.compile(
    r"OFFBOARD_PLANNED_HORIZON_APPLIED .*?" + HOP_REPORT_PATTERN)

# The processes that fly on the aircraft: the navigation nodes and the DDS
# agent. PX4 lives on the flight controller, the simulator and the
# visualisation do not exist there, and the captures are the harness.
# The depth node is the stereo profile's perception and flies with it: left
# with the simulator it hid 1.4 cores of the onboard budget (r506).
ONBOARD_PROCESSES = ("production_mppi_node", "obstacle_memory_3d_node",
                     "stereo_depth_node", "lidar_inertial_odometry_node",
                     "mppi_offboard_node", "MicroXRCEAgent")
MINIMUM_RECORD_COVERAGE = 0.90
SAMPLE_PERIOD_S = 1.0
# The resident memory an onboard process may gain over a flight, the last
# tenth against the first. Measured on r345 (392.8 m, 142 s): the controller
# gained 128 MiB and the obstacle memory 53 MiB, both holding a map that grows
# with the volume the flight observes; the offboard node and the agent gained
# nothing. Twice the largest of those is the bound. A leak at the tick rate
# would cross it fast: 5392 ticks losing 50 KiB each are 263 MiB.
MAXIMUM_ONBOARD_RSS_GROWTH_BYTES = 256 * 1024 * 1024
# The delivery of the obstacle memory's snapshots and deltas to the
# controller at p95, the hop the observation age depends on. Measured
# 0.25 to 0.30 ms on r346 and r348 to r351, 1.24 ms on r347 under a foreign
# build on the host; twice the worst is the bound, and a hop that turns
# into milliseconds is what it exists to catch.
MAXIMUM_RAW_DELIVERY_P95_MS = 2.5


@dataclass(frozen=True)
class ProcessUsage:
    name: str
    cpu_cores_p50: float
    cpu_cores_p95: float
    cpu_cores_max: float
    rss_p95_bytes: float
    rss_growth_bytes: float
    threads_max: int
    samples: int


@dataclass(frozen=True)
class ResourceRecord:
    stamps_s: np.ndarray
    names: list[str]
    cpu_cores: np.ndarray
    rss_bytes: np.ndarray
    threads: np.ndarray
    gpu_utilization_percent: np.ndarray
    gpu_memory_used_bytes: np.ndarray
    gpu_process_memory_bytes: np.ndarray
    real_time_factor: np.ndarray


def load_resource_record(path: Path) -> ResourceRecord:
    columns: dict[str, list] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            for key, value in row.items():
                columns[key].append(value)

    def numbers(key: str) -> np.ndarray:
        return np.array([float(value) for value in columns.get(key, [])], dtype=np.float64)

    return ResourceRecord(
        stamps_s=numbers("stamp_s"),
        names=list(columns.get("name", [])),
        cpu_cores=numbers("cpu_cores"),
        rss_bytes=numbers("rss_bytes"),
        threads=numbers("threads"),
        gpu_utilization_percent=numbers("gpu_utilization_percent"),
        gpu_memory_used_bytes=numbers("gpu_memory_used_bytes"),
        gpu_process_memory_bytes=numbers("gpu_process_memory_bytes"),
        real_time_factor=numbers("real_time_factor"),
    )


def flight_span_s(ros_log: str) -> tuple[float, float] | None:
    """Mission readiness to the successful result, the span the speed check
    measures over; None when the flight has no such span."""
    readiness = re.search(MISSION_READINESS_PATTERN, ros_log)
    result = re.search(MISSION_SUCCESS_PATTERN, ros_log)
    if readiness is None or result is None:
        return None
    started_s, finished_s = float(readiness.group(1)), float(result.group(1))
    return (started_s, finished_s) if finished_s > started_s else None


def record_coverage(record: ResourceRecord, span: tuple[float, float]) -> float:
    """The share of the flight's seconds that have a sample."""
    started_s, finished_s = span
    inside = (record.stamps_s >= started_s) & (record.stamps_s <= finished_s)
    seconds = np.unique(np.floor(record.stamps_s[inside]))
    return min(1.0, len(seconds) * SAMPLE_PERIOD_S / (finished_s - started_s))


def process_usage(record: ResourceRecord, name: str,
                  span: tuple[float, float]) -> ProcessUsage | None:
    """One process over the flight; growth is the resident memory over the
    last tenth of the flight against the first tenth, at the median of each,
    so a leak shows as a positive number and a transient does not."""
    started_s, finished_s = span
    selected = np.array([n == name for n in record.names]) & \
        (record.stamps_s >= started_s) & (record.stamps_s <= finished_s)
    if not selected.any():
        return None
    stamps = record.stamps_s[selected]
    cpu = record.cpu_cores[selected]
    rss = record.rss_bytes[selected]
    tenth = (finished_s - started_s) / 10.0
    first = rss[stamps <= started_s + tenth]
    last = rss[stamps >= finished_s - tenth]
    growth = float(np.median(last) - np.median(first)) if len(first) and len(last) else 0.0
    return ProcessUsage(
        name=name,
        cpu_cores_p50=float(np.median(cpu)),
        cpu_cores_p95=float(np.percentile(cpu, 95)),
        cpu_cores_max=float(cpu.max()),
        rss_p95_bytes=float(np.percentile(rss, 95)),
        rss_growth_bytes=growth,
        threads_max=int(record.threads[selected].max()),
        samples=int(selected.sum()),
    )


def group_totals(record: ResourceRecord, names: set[str] | None,
                 span: tuple[float, float], exclude: set[str] = frozenset()
                 ) -> tuple[float, float, float]:
    """CPU cores at p50 and p95 and resident memory at p95 of the per-second
    sums over a group of processes: the named ones, or every process not
    excluded when names is None."""
    started_s, finished_s = span
    per_second_cpu: dict[float, float] = defaultdict(float)
    per_second_rss: dict[float, float] = defaultdict(float)
    for stamp, name, cpu, rss in zip(record.stamps_s, record.names, record.cpu_cores,
                                     record.rss_bytes):
        if stamp < started_s or stamp > finished_s:
            continue
        if names is not None and name not in names:
            continue
        if names is None and name in exclude:
            continue
        per_second_cpu[stamp] += cpu
        per_second_rss[stamp] += rss
    if not per_second_cpu:
        return 0.0, 0.0, 0.0
    cpu = np.array(list(per_second_cpu.values()))
    rss = np.array(list(per_second_rss.values()))
    return float(np.median(cpu)), float(np.percentile(cpu, 95)), float(np.percentile(rss, 95))


def mib(value: float) -> float:
    return value / (1024.0 * 1024.0)


@dataclass(frozen=True)
class HopLatency:
    name: str
    samples: int
    p50_ms: float
    p95_ms: float
    max_ms: float


def transport_hops(ros_log: str) -> list[HopLatency]:
    """Every hop's delivery latency as its consumer summarised it: the
    controller's two hops from its summary, the obstacle memory's and the
    offboard node's from the last of their periodic reports."""
    hops: list[HopLatency] = []
    summary = None
    for summary in SUMMARY_TRANSPORT_PATTERN.finditer(ros_log):
        pass
    if summary is not None:
        hops.append(HopLatency("memory to controller (raw snapshots and deltas)",
                               int(summary.group(1)), float(summary.group(2)),
                               float(summary.group(3)), float(summary.group(4))))
        hops.append(HopLatency("memory to controller (latest sensor scan)",
                               int(summary.group(5)), float(summary.group(6)),
                               float(summary.group(7)), float(summary.group(8))))
    for name, pattern in (("bridge to memory (point cloud)", CLOUD_TRANSPORT_PATTERN),
                          ("controller to offboard (horizon)", HORIZON_TRANSPORT_PATTERN)):
        last = None
        for last in pattern.finditer(ros_log):
            pass
        if last is not None:
            hops.append(HopLatency(name, int(last.group(4)), float(last.group(1)),
                                   float(last.group(2)), float(last.group(3))))
    return hops


@dataclass(frozen=True)
class ObservationAgeSplit:
    ticks: int
    age_p50_ms: float
    age_p95_ms: float
    producer_p50_ms: float
    delivery_p50_ms: float
    wait_p50_ms: float


def observation_age_split(ros_log: str, span: tuple[float, float]) -> ObservationAgeSplit | None:
    """The observation age the tick reports, split at the median over the
    flight: what the producer's period and build took before publication,
    what the delivery took, and how long the update waited for the tick."""
    ages, producer, delivery, wait = [], [], [], []
    for match in TICK_TRANSPORT_PATTERN.finditer(ros_log):
        stamp = float(match.group(1))
        if stamp < span[0] or stamp > span[1]:
            continue
        age = float(match.group(2))
        delivered = float(match.group(3))
        waited = float(match.group(4))
        if age < 0.0 or not math.isfinite(delivered) or waited < 0.0:
            continue
        ages.append(age)
        delivery.append(delivered)
        wait.append(waited)
        producer.append(max(0.0, age - delivered - waited))
    if not ages:
        return None
    return ObservationAgeSplit(
        ticks=len(ages),
        age_p50_ms=float(np.median(ages)),
        age_p95_ms=float(np.percentile(ages, 95)),
        producer_p50_ms=float(np.median(producer)),
        delivery_p50_ms=float(np.median(delivery)),
        wait_p50_ms=float(np.median(wait)),
    )


def report_transport(ros_log: str, span: tuple[float, float],
                     errors: list[str]) -> None:
    hops = transport_hops(ros_log)
    if not hops:
        print("OK: no transport hop reported its delivery")
    for hop in hops:
        if hop.name.startswith("memory to controller (raw") and \
                hop.p95_ms > MAXIMUM_RAW_DELIVERY_P95_MS:
            errors.append(
                f"FAIL: transport {hop.name} delivers within "
                f"{MAXIMUM_RAW_DELIVERY_P95_MS:.1f} ms at p95 ({hop.p95_ms:.2f} ms)")
            continue
        print(f"OK: transport {hop.name} delivers in {hop.p50_ms:.2f} ms at p50, "
              f"{hop.p95_ms:.2f} at p95, {hop.max_ms:.2f} at most ({hop.samples} messages)")
    split = observation_age_split(ros_log, span)
    if split is None:
        print("OK: the tick does not split the observation age")
        return
    print(f"OK: observation age is {split.age_p50_ms:.0f} ms at p50 and "
          f"{split.age_p95_ms:.0f} at p95: {split.producer_p50_ms:.0f} ms producer period "
          f"and build, {split.delivery_p50_ms:.2f} ms delivery, {split.wait_p50_ms:.0f} ms "
          f"waiting for the tick, at the median over {split.ticks} ticks")


def validate_resource_budget(run_directory: Path, ros_log: str,
                             errors: list[str]) -> None:
    """The coverage check and the usage report, on the flight's own record."""
    record_path = run_directory / "resources.csv"
    host_path = run_directory / "resources_host.json"
    if not record_path.is_file() or not host_path.is_file():
        errors.append("FAIL: the flight records its process resources "
                      f"({record_path.name}, {host_path.name})")
        return
    record = load_resource_record(record_path)
    host = json.loads(host_path.read_text(encoding="utf-8"))
    span = flight_span_s(ros_log)
    if span is None:
        print("OK: resource record coverage is not measured without a successful flight")
        return
    coverage = record_coverage(record, span)
    if coverage < MINIMUM_RECORD_COVERAGE:
        errors.append(
            "FAIL: the resource record covers at least "
            f"{MINIMUM_RECORD_COVERAGE:.0%} of the flight ({coverage:.0%})")
    else:
        print(f"OK: the resource record covers {coverage:.0%} of the flight "
              f"({span[1] - span[0]:.0f} s on {host.get('cpu_model', '?')}, "
              f"{host.get('gpu_name', '?')})")

    onboard = set(ONBOARD_PROCESSES)
    captures = {name for name in set(record.names) if name.startswith("capture_")}
    for name in ONBOARD_PROCESSES:
        usage = process_usage(record, name, span)
        if usage is None:
            print(f"OK: {name} is not in the resource record")
            continue
        if usage.rss_growth_bytes > MAXIMUM_ONBOARD_RSS_GROWTH_BYTES:
            errors.append(
                f"FAIL: {name} gains at most {mib(MAXIMUM_ONBOARD_RSS_GROWTH_BYTES):.0f} MiB "
                f"of resident memory over the flight ({mib(usage.rss_growth_bytes):+.0f} MiB)")
            continue
        print(f"OK: {name} uses {usage.cpu_cores_p50:.2f} cores at p50, "
              f"{usage.cpu_cores_p95:.2f} at p95, {usage.cpu_cores_max:.2f} at most; "
              f"RSS {mib(usage.rss_p95_bytes):.0f} MiB at p95, "
              f"{mib(usage.rss_growth_bytes):+.0f} MiB over the flight; "
              f"{usage.threads_max} threads ({usage.samples} samples)")
    for label, names, exclude in (("onboard processes", onboard, frozenset()),
                                  ("captures", captures, frozenset()),
                                  ("simulator and harness", None, onboard | captures)):
        cpu_p50, cpu_p95, rss_p95 = group_totals(record, names, span, exclude)
        print(f"OK: {label} use {cpu_p50:.2f} cores at p50, {cpu_p95:.2f} at p95; "
              f"RSS {mib(rss_p95):.0f} MiB at p95")

    inside = (record.stamps_s >= span[0]) & (record.stamps_s <= span[1])
    utilization = record.gpu_utilization_percent[inside]
    utilization = utilization[np.isfinite(utilization)]
    controller = np.array([n == "production_mppi_node" for n in record.names]) & inside
    controller_memory = record.gpu_process_memory_bytes[controller]
    if len(utilization):
        print(f"OK: GPU utilisation is {np.median(utilization):.0f}% at p50, "
              f"{np.percentile(utilization, 95):.0f}% at p95; memory used "
              f"{mib(np.percentile(record.gpu_memory_used_bytes[inside], 95)):.0f} MiB at p95, "
              f"of which production_mppi_node "
              f"{mib(np.percentile(controller_memory, 95)) if len(controller_memory) else 0:.0f} MiB")
    rtf = record.real_time_factor[inside]
    rtf = rtf[np.isfinite(rtf)]
    if len(rtf):
        print(f"OK: real-time factor is {np.median(rtf):.2f} at p50, {rtf.min():.2f} at least")
    else:
        print("OK: real-time factor was not published")
    report_transport(ros_log, span, errors)
