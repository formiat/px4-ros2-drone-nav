#!/usr/bin/env python3
"""Clean conflicting simulator processes before a new run."""

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


PROJECT_NODE_EXECUTABLES = {
    "lidar_debug_node",
    "mission_monitor_node",
    "obstacle_memory_node",
    "planner_node",
    "px4_offboard_node",
}


def default_project_markers() -> list[str]:
    return [
        "/workspace",
        "drone_city_nav",
        "drone-gazebo",
    ]


def normalized_project_markers(values: list[str]) -> list[str]:
    markers = default_project_markers()
    for value in values:
        if value:
            markers.append(os.path.realpath(value))
            markers.append(value)
    return sorted({marker for marker in markers if marker}, key=len, reverse=True)


def _split_command(cmd: str) -> list[str]:
    try:
        return shlex.split(cmd)
    except ValueError:
        return cmd.split()


def _command_index_after_env_wrapper(tokens: list[str]) -> int:
    if not tokens or os.path.basename(tokens[0]) != "env":
        return 0

    index = 1
    while index < len(tokens):
        token = tokens[index]
        if token == "--":
            return index + 1
        if token.startswith("-"):
            if token in {"-i", "-0"}:
                index += 1
                continue
            if token in {"-u", "-C", "-S"}:
                index += 2
                continue
            if token.startswith(("-u", "-C", "-S")):
                index += 1
                continue
            index += 1
            continue
        if "=" in token and not token.startswith("="):
            index += 1
            continue
        break
    return index


def is_conflicting_gazebo_process(cmd: str) -> bool:
    tokens = _split_command(cmd)
    command_index = _command_index_after_env_wrapper(tokens)
    if command_index + 1 >= len(tokens):
        return False
    return (
        os.path.basename(tokens[command_index]) == "gz"
        and tokens[command_index + 1] == "sim"
    )


def command_has_marker(cmd: str, markers: list[str]) -> bool:
    return any(marker in cmd for marker in markers)


def is_project_run_script(cmd: str) -> bool:
    tokens = _split_command(cmd)
    command_index = _command_index_after_env_wrapper(tokens)
    if command_index >= len(tokens):
        return False

    executable = os.path.basename(tokens[command_index])
    has_runner_arg = any(
        token.endswith("scripts/run_drone_nav_sim.sh")
        or token.endswith("./scripts/run_drone_nav_sim.sh")
        for token in tokens[command_index + 1 :]
    )
    return executable in {"bash", "sh", "run_drone_nav_sim.sh"} and (
        executable == "run_drone_nav_sim.sh" or has_runner_arg
    )


def is_project_ros_launch(cmd: str) -> bool:
    return "ros2 launch drone_city_nav city_nav.launch.py" in cmd


def is_project_ros_node(cmd: str, markers: list[str]) -> bool:
    if not command_has_marker(cmd, markers):
        return False
    return any(
        f"/{name}" in cmd or cmd.endswith(name) for name in PROJECT_NODE_EXECUTABLES
    )


def is_project_px4_process(cmd: str, markers: list[str]) -> bool:
    if "PX4-Autopilot" not in cmd and "px4_sitl" not in cmd:
        return False
    if not command_has_marker(cmd, markers):
        return False
    return (
        "px4_sitl" in cmd
        or "gz_x500_lidar_2d" in cmd
        or "PX4_SIM_MODEL=gz_x500_lidar_2d" in cmd
        or "/bin/px4" in cmd
    )


def is_project_micro_xrce_agent(cmd: str) -> bool:
    tokens = _split_command(cmd)
    if not tokens or os.path.basename(tokens[0]) != "MicroXRCEAgent":
        return False
    return "udp4" in tokens and "-p" in tokens and "8888" in tokens


def is_project_rviz_process(cmd: str, markers: list[str]) -> bool:
    if "rviz2" not in cmd:
        return False
    return (
        "city_nav_debug.rviz" in cmd
        or "city_nav_debug_top_down.rviz" in cmd
        or command_has_marker(cmd, markers)
    )


def is_conflicting_simulation_process(cmd: str, markers: list[str]) -> bool:
    return (
        is_conflicting_gazebo_process(cmd)
        or is_project_run_script(cmd)
        or is_project_ros_launch(cmd)
        or is_project_ros_node(cmd, markers)
        or is_project_px4_process(cmd, markers)
        or is_project_micro_xrce_agent(cmd)
        or is_project_rviz_process(cmd, markers)
    )


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


def collect_descendant_pids(
    processes: list[ProcessInfo], seed_pids: set[int]
) -> set[int]:
    children_by_parent: dict[int, list[int]] = {}
    for process in processes:
        children_by_parent.setdefault(process.ppid, []).append(process.pid)

    descendants: set[int] = set()
    stack = list(seed_pids)
    while stack:
        pid = stack.pop()
        for child_pid in children_by_parent.get(pid, []):
            if child_pid in descendants:
                continue
            descendants.add(child_pid)
            stack.append(child_pid)
    return descendants


def select_conflicting_processes(
    processes: list[ProcessInfo],
    protected_pids: set[int],
    *,
    project_markers: list[str] | None = None,
) -> list[ProcessInfo]:
    protected = collect_ancestor_pids(processes, protected_pids)
    protected.update(collect_descendant_pids(processes, protected_pids))
    markers = (
        project_markers if project_markers is not None else default_project_markers()
    )
    directly_selected = [
        process
        for process in processes
        if process.pid not in protected
        and is_conflicting_simulation_process(process.cmd, markers)
    ]
    selected_pids = {process.pid for process in directly_selected}
    selected_pids.update(collect_descendant_pids(processes, selected_pids))
    candidates = [
        process
        for process in processes
        if process.pid in selected_pids and process.pid not in protected
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
        description="Stop stale/conflicting simulator processes."
    )
    parser.add_argument("--self-pid", type=int, action="append", default=[])
    parser.add_argument("--protect-pid", type=int, action="append", default=[])
    parser.add_argument("--repo-root", action="append", default=[])
    parser.add_argument("--project-marker", action="append", default=[])
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
    project_markers = normalized_project_markers(
        [*args.repo_root, *args.project_marker]
    )
    candidates = select_conflicting_processes(
        processes,
        protected,
        project_markers=project_markers,
    )
    return stop_processes(
        candidates,
        dry_run=args.dry_run,
        term_timeout_s=args.term_timeout_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
