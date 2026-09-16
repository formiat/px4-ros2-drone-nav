#!/usr/bin/env python3
"""Record what every process of one flight consumes, once a second, for the
resource-budget checks of the headless mission check: CPU in cores, resident
memory and threads per process, the GPU's utilisation and memory, the
container's cgroup totals, and the simulator's real-time factor. The host is
described once. The record is rewritten atomically every few seconds so an
interrupted flight still leaves a usable one. The capture records itself too,
so its own cost is in the record.

  capture_process_resources.py OUTPUT.csv HOST.json --world WORLD
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import re
import subprocess
import threading
import time
from pathlib import Path

import psutil

CGROUP = Path("/sys/fs/cgroup")
COLUMNS = ("stamp_s", "pid", "name", "cpu_cores", "rss_bytes", "threads",
           "gpu_utilization_percent", "gpu_memory_used_bytes", "gpu_process_memory_bytes",
           "cgroup_cpu_usec", "cgroup_memory_bytes", "real_time_factor")


def nvidia_smi(query: str, fields: str) -> list[list[str]]:
    """Rows of one nvidia-smi query, or none when there is no GPU to ask."""
    try:
        completed = subprocess.run(
            ["nvidia-smi", f"--query-{query}={fields}", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2.0, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return []
    if completed.returncode != 0:
        return []
    return [[cell.strip() for cell in line.split(",")]
            for line in completed.stdout.splitlines() if line.strip()]


def parse_gpu_sample(gpu_rows: list[list[str]],
                     app_rows: list[list[str]]) -> tuple[float, int, dict[str, int]]:
    """Utilisation in percent, memory used in bytes, and memory by process
    name from the two nvidia-smi queries. The names are the key because a
    container sees nvidia-smi's host pids, which match nothing in its own
    namespace."""
    utilization, used = float("nan"), 0
    for row in gpu_rows:
        if len(row) == 2 and row[1].isdigit():
            utilization, used = float(row[0]), int(row[1]) * 1024 * 1024
            break
    by_name: dict[str, int] = {}
    for row in app_rows:
        if len(row) == 2 and row[1].isdigit():
            name = os.path.basename(row[0])
            by_name[name] = by_name.get(name, 0) + int(row[1]) * 1024 * 1024
    return utilization, used, by_name


def gpu_sample() -> tuple[float, int, dict[str, int]]:
    return parse_gpu_sample(nvidia_smi("gpu", "utilization.gpu,memory.used"),
                            nvidia_smi("compute-apps", "process_name,used_memory"))


INTERPRETERS = {"python3", "python", "bash", "sh", "ruby"}


def process_label(name: str, cmdline: list[str]) -> str:
    """The process by what it runs: an interpreter is named after its script,
    so the three python captures of a flight are told apart, and a compiled
    node keeps its own name."""
    if name not in INTERPRETERS:
        return name
    for argument in cmdline[1:]:
        if not argument.startswith("-"):
            # An inline command (bash -c "...") is not a script; keep the shell.
            return name if " " in argument else os.path.basename(argument)
    return name


def cgroup_sample() -> tuple[int, int]:
    """The container's own cpu.stat usage and memory.current, zero when absent."""
    cpu_usec, memory = 0, 0
    try:
        match = re.search(r"usage_usec (\d+)", (CGROUP / "cpu.stat").read_text())
        cpu_usec = int(match.group(1)) if match else 0
        memory = int((CGROUP / "memory.current").read_text().strip())
    except (OSError, ValueError):
        pass
    return cpu_usec, memory


def host_description(world: str) -> dict[str, object]:
    model = ""
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    gpu = nvidia_smi("gpu", "name,driver_version,memory.total")
    cuda = ""
    try:
        version = subprocess.run(["nvcc", "--version"], capture_output=True, text=True,
                                 timeout=5.0, check=False).stdout
        match = re.search(r"release (\d+\.\d+)", version)
        cuda = match.group(1) if match else ""
    except (OSError, subprocess.TimeoutExpired):
        pass
    return {
        "cpu_model": model,
        "cpu_cores_physical": psutil.cpu_count(logical=False),
        "cpu_cores_logical": psutil.cpu_count(logical=True),
        "memory_total_bytes": psutil.virtual_memory().total,
        "gpu_name": gpu[0][0] if gpu and len(gpu[0]) == 3 else "",
        "gpu_driver_version": gpu[0][1] if gpu and len(gpu[0]) == 3 else "",
        "gpu_memory_total_bytes": int(gpu[0][2]) * 1024 * 1024
        if gpu and len(gpu[0]) == 3 and gpu[0][2].isdigit() else 0,
        "cuda_version": cuda,
        "kernel": platform.release(),
        "container_image": os.environ.get("DRONE_GAZEBO_DEV_IMAGE", ""),
        "world": world,
        "sample_period_s": 1.0,
    }


class RealTimeFactor:
    """The latest real-time factor Gazebo published for the world, NaN until
    the first statistics message; a missing simulator leaves it NaN."""

    def __init__(self, world: str) -> None:
        self.value = float("nan")
        try:
            from gz.msgs10.world_stats_pb2 import WorldStatistics
            from gz.transport13 import Node
        except ImportError:
            return
        self._node = Node()
        self._node.subscribe(WorldStatistics, f"/world/{world}/stats", self._on_stats)

    def _on_stats(self, message) -> None:
        self.value = float(message.real_time_factor)


def write_atomically(output: str, rows: list[tuple[object, ...]]) -> None:
    temporary = output + ".tmp"
    with open(temporary, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    os.replace(temporary, output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("host_output")
    parser.add_argument("--world", required=True)
    parser.add_argument("--save-period-s", type=float, default=5.0)
    args = parser.parse_args()

    Path(args.host_output).write_text(json.dumps(host_description(args.world), indent=2),
                                      encoding="utf-8")
    rtf = RealTimeFactor(args.world)
    rows: list[tuple[object, ...]] = []
    lock = threading.Lock()
    processes: dict[int, psutil.Process] = {}
    # The first cpu_percent of a process is always zero; priming it here makes
    # the first recorded second real.
    for process in psutil.process_iter(["pid"]):
        try:
            process.cpu_percent()
            processes[process.pid] = process
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    print(f"recording {len(processes)} processes to {args.output}", flush=True)

    last_save = time.monotonic()
    next_sample = time.monotonic() + 1.0
    while True:
        time.sleep(max(0.0, next_sample - time.monotonic()))
        next_sample += 1.0
        stamp = time.time()
        utilization, gpu_used, gpu_by_name = gpu_sample()
        cgroup_cpu, cgroup_memory = cgroup_sample()
        seen = set()
        for process in psutil.process_iter(["pid"]):
            seen.add(process.pid)
            if process.pid not in processes:
                try:
                    process.cpu_percent()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                processes[process.pid] = process
                continue
        sample: list[tuple[object, ...]] = []
        for pid in list(processes):
            if pid not in seen:
                del processes[pid]
                continue
            process = processes[pid]
            try:
                with process.oneshot():
                    name = process_label(process.name(), process.cmdline())
                    cpu = process.cpu_percent() / 100.0
                    rss = process.memory_info().rss
                    threads = process.num_threads()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                del processes[pid]
                continue
            sample.append((f"{stamp:.3f}", pid, name, f"{cpu:.3f}", rss, threads,
                           f"{utilization:.1f}", gpu_used, gpu_by_name.get(name, 0),
                           cgroup_cpu, cgroup_memory, f"{rtf.value:.4f}"))
        with lock:
            rows.extend(sample)
        if time.monotonic() - last_save >= args.save_period_s:
            with lock:
                snapshot = list(rows)
            write_atomically(args.output, snapshot)
            last_save = time.monotonic()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
