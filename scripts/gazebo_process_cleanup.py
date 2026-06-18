#!/usr/bin/env python3
"""Clean conflicting Gazebo simulator processes before a new run."""

from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    ppid: int
    pgid: int
    cmd: str


def _split_command(cmd: str) -> list[str]:
    try:
        return shlex.split(cmd)
    except ValueError:
        return cmd.split()


def is_conflicting_gazebo_process(cmd: str) -> bool:
    tokens = _split_command(cmd)
    for index, token in enumerate(tokens[:-1]):
        if os.path.basename(token) == "gz" and tokens[index + 1] == "sim":
            return True
    return False


def parse_ps_line(line: str) -> ProcessInfo | None:
    parts = line.strip().split(None, 3)
    if len(parts) != 4:
        return None
    try:
        return ProcessInfo(
            pid=int(parts[0]),
            ppid=int(parts[1]),
            pgid=int(parts[2]),
            cmd=parts[3],
        )
    except ValueError:
        return None


def parse_ps_output(text: str) -> list[ProcessInfo]:
    processes: list[ProcessInfo] = []
    for line in text.splitlines():
        process = parse_ps_line(line)
        if process is not None:
            processes.append(process)
    return processes


def collect_ancestor_pids(
    processes: list[ProcessInfo], seed_pids: set[int]
) -> set[int]:
    by_pid = {process.pid: process for process in processes}
    protected = set(seed_pids)
    stack = list(seed_pids)
    while stack:
        pid = stack.pop()
        process = by_pid.get(pid)
        if process is None or process.ppid <= 0 or process.ppid in protected:
            continue
        protected.add(process.ppid)
        stack.append(process.ppid)
    return protected


def select_conflicting_processes(
    processes: list[ProcessInfo], protected_pids: set[int]
) -> list[ProcessInfo]:
    protected = collect_ancestor_pids(processes, protected_pids)
    candidates = [
        process
        for process in processes
        if process.pid not in protected and is_conflicting_gazebo_process(process.cmd)
    ]
    return sorted(candidates, key=lambda process: process.pid)


def read_process_table() -> list[ProcessInfo]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,pgid=,cmd="],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "ps command failed")
    return parse_ps_output(result.stdout)


def _is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def stop_processes(
    processes: list[ProcessInfo],
    *,
    dry_run: bool,
    term_timeout_s: float,
) -> int:
    if not processes:
        print("Gazebo stale cleanup: no conflicting Gazebo processes found")
        return 0

    print(
        "Gazebo stale cleanup: "
        f"candidates={len(processes)} dry_run={str(dry_run).lower()}"
    )
    for process in processes:
        print(
            "Gazebo stale cleanup candidate: "
            f"pid={process.pid} ppid={process.ppid} pgid={process.pgid} "
            f"cmd={process.cmd}"
        )

    if dry_run:
        return 0

    for process in processes:
        try:
            os.kill(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        except PermissionError as exc:
            print(
                "WARNING: could not SIGTERM Gazebo process "
                f"pid={process.pid}: {exc}",
                file=sys.stderr,
            )

    deadline = time.monotonic() + max(term_timeout_s, 0.0)
    remaining = [process for process in processes if _is_alive(process.pid)]
    while remaining and time.monotonic() < deadline:
        time.sleep(0.25)
        remaining = [process for process in remaining if _is_alive(process.pid)]

    for process in remaining:
        try:
            os.kill(process.pid, signal.SIGKILL)
            print(f"Gazebo stale cleanup: SIGKILL pid={process.pid}")
        except ProcessLookupError:
            continue
        except PermissionError as exc:
            print(
                "WARNING: could not SIGKILL Gazebo process "
                f"pid={process.pid}: {exc}",
                file=sys.stderr,
            )

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stop stale/conflicting Gazebo simulator processes."
    )
    parser.add_argument("--self-pid", type=int, action="append", default=[])
    parser.add_argument("--protect-pid", type=int, action="append", default=[])
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--term-timeout-s", type=float, default=5.0)
    parser.add_argument("--ps-file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.ps_file:
            processes = parse_ps_output(open(args.ps_file, encoding="utf-8").read())
        else:
            processes = read_process_table()
    except OSError as exc:
        print(f"ERROR: failed to read process table: {exc}", file=sys.stderr)
        return 2
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    protected = {os.getpid(), os.getppid(), *args.self_pid, *args.protect_pid}
    candidates = select_conflicting_processes(processes, protected)
    return stop_processes(
        candidates,
        dry_run=args.dry_run,
        term_timeout_s=args.term_timeout_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
