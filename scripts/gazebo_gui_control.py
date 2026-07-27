#!/usr/bin/env python3
"""Gazebo GUI and world-control commands for the drone navigation launcher."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""

    @property
    def combined_output(self) -> str:
        return "\n".join(part for part in (self.stdout, self.stderr) if part)


class CommandRunner(Protocol):
    def __call__(self, args: list[str], timeout_s: float) -> CommandResult:
        ...


@dataclass(frozen=True)
class TrackingConfirmation:
    confirmed: bool
    available: bool
    output: str = ""


def _timeout_output_to_text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode(errors="replace")
    return value


def default_runner(args: list[str], timeout_s: float) -> CommandResult:
    gz_command = shlex.split(os.environ.get("GZ_BIN", "gz"))
    command = [*gz_command, *args]
    try:
        result = subprocess.run(
            command,
            check=False,
            text=True,
            capture_output=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = _timeout_output_to_text(exc.stdout)
        stderr = _timeout_output_to_text(exc.stderr)
        stderr_parts = [part for part in (stderr, str(exc)) if part]
        return CommandResult(124, stdout, "\n".join(stderr_parts))
    return CommandResult(result.returncode, result.stdout, result.stderr)


def response_is_true(result: CommandResult) -> bool:
    return result.returncode == 0 and "data: true" in result.combined_output


def parse_wait_s(value: str, *, default: int, label: str) -> int:
    try:
        wait_s = int(value)
    except ValueError:
        print(f"WARNING: invalid {label} wait '{value}', using {default}s.")
        return default
    if wait_s <= 0:
        print(f"WARNING: invalid {label} wait '{value}', using {default}s.")
        return default
    return wait_s


def parse_offset(value: str) -> tuple[float, float, float] | None:
    pieces = value.split()
    if len(pieces) != 3:
        return None
    try:
        return (float(pieces[0]), float(pieces[1]), float(pieces[2]))
    except ValueError:
        return None


def _format_float(value: float) -> str:
    return f"{value:g}"


def _run_service(
    runner: CommandRunner,
    *,
    service: str,
    reqtype: str,
    reptype: str,
    request: str,
) -> CommandResult:
    return runner(
        [
            "service",
            "-s",
            service,
            "--reqtype",
            reqtype,
            "--reptype",
            reptype,
            "--timeout",
            "1000",
            "--req",
            request,
        ],
        2.0,
    )


def _publish_track(
    runner: CommandRunner,
    *,
    target: str,
    offset: tuple[float, float, float],
) -> CommandResult:
    offset_x, offset_y, offset_z = offset
    payload = (
        "track_mode: FOLLOW "
        f'follow_target {{ name: "{target}" }} '
        "follow_offset { "
        f"x: {_format_float(offset_x)} "
        f"y: {_format_float(offset_y)} "
        f"z: {_format_float(offset_z)} "
        "} "
        "follow_pgain: 1.0 "
        "track_pgain: 1.0"
    )
    return runner(
        [
            "topic",
            "-t",
            "/gui/track",
            "-m",
            "gz.msgs.CameraTrack",
            "-p",
            payload,
        ],
        2.0,
    )


def _confirm_tracking(
    runner: CommandRunner,
    *,
    target: str,
    sample_duration_s: float = 1.0,
    attempts: int = 2,
) -> TrackingConfirmation:
    last_output = ""
    observed_output = False
    for _ in range(max(1, attempts)):
        result = runner(
            [
                "topic",
                "-e",
                "-t",
                "/gui/currently_tracked",
                "-d",
                _format_float(sample_duration_s),
            ],
            sample_duration_s + 1.0,
        )
        output = result.combined_output
        last_output = output
        if result.returncode != 0 or not output.strip():
            continue
        observed_output = True
        if target in output:
            return TrackingConfirmation(True, True, output)
    if observed_output:
        return TrackingConfirmation(False, True, last_output)
    return TrackingConfirmation(False, False, last_output)


def _compact_log_excerpt(text: str, *, limit: int = 240) -> str:
    compact = " ".join(text.split())
    if len(compact) <= limit:
        return compact
    return compact[: limit - 3] + "..."


def configure_follow_camera(
    *,
    target: str,
    offset_text: str,
    wait_s: int,
    runner: CommandRunner = default_runner,
    tracking_sample_duration_s: float = 1.0,
    tracking_confirmation_attempts: int = 2,
) -> int:
    offset = parse_offset(offset_text)
    if offset is None:
        print(
            f"WARNING: invalid Gazebo GUI follow offset '{offset_text}', "
            "expected 'x y z'."
        )
        return 0

    published_attempts = 0
    attempted_state_confirmation = False
    last_response = ""
    for attempt in range(1, wait_s + 1):
        track_response = _publish_track(runner, target=target, offset=offset)
        last_response = track_response.combined_output
        if track_response.returncode == 0:
            published_attempts += 1
            tracking_state = _confirm_tracking(
                runner,
                target=target,
                sample_duration_s=tracking_sample_duration_s,
                attempts=tracking_confirmation_attempts,
            )
            attempted_state_confirmation = True
            if tracking_state.confirmed:
                print(
                    "Gazebo GUI follow camera state confirmed: "
                    f"target={target} offset=({_format_float(offset[0])}, "
                    f"{_format_float(offset[1])}, {_format_float(offset[2])}) "
                    f"published_attempts={published_attempts}"
                )
                return 0
            if tracking_state.available and (
                attempt == 1 or attempt % 5 == 0 or attempt == wait_s
            ):
                print(
                    "WARNING: Gazebo GUI follow camera state did not mention "
                    f"target '{target}': "
                    f"{_compact_log_excerpt(tracking_state.output)}"
                )
        elif attempt == 1 or attempt % 5 == 0:
            print(
                f"Waiting to publish Gazebo GUI follow target '{target}' "
                f"({attempt}/{wait_s}): {last_response}"
            )
        time.sleep(1)

    if attempted_state_confirmation and published_attempts > 0:
        print(
            "WARNING: Gazebo GUI follow camera command was published "
            f"{published_attempts} times but state was not confirmed for "
            f"target '{target}' after {wait_s}s."
        )
        return 0
    print(
        f"WARNING: Gazebo GUI follow camera was not configured for target "
        f"'{target}' after {wait_s}s. Last response: {last_response}"
    )
    return 0


def configure_world_running(
    *,
    world: str,
    wait_s: int,
    runner: CommandRunner = default_runner,
    required_confirmations: int = 3,
) -> int:
    confirmed_attempts = 0
    last_response = ""
    for attempt in range(1, wait_s + 1):
        response = _run_service(
            runner,
            service=f"/world/{world}/control",
            reqtype="gz.msgs.WorldControl",
            reptype="gz.msgs.Boolean",
            request="pause: false",
        )
        last_response = response.combined_output
        if response_is_true(response):
            confirmed_attempts += 1
            if confirmed_attempts == 1:
                print(f"Gazebo world unpause command accepted: world={world}")
            if confirmed_attempts >= required_confirmations:
                print(f"Gazebo world running command confirmed: world={world}")
                return 0
        else:
            confirmed_attempts = 0
            if attempt == 1 or attempt % 5 == 0:
                print(
                    f"Waiting to unpause Gazebo world '{world}' "
                    f"({attempt}/{wait_s}): {last_response}"
                )
        time.sleep(1)
    print(
        f"WARNING: Gazebo world '{world}' was not confirmed running after "
        f"{wait_s}s. Last response: {last_response}"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control Gazebo GUI/world state.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    follow = subparsers.add_parser("follow-camera")
    follow.add_argument("--target", required=True)
    follow.add_argument("--offset", required=True)
    follow.add_argument("--wait-s", default="60")

    world = subparsers.add_parser("world-running")
    world.add_argument("--world", required=True)
    world.add_argument("--wait-s", default="60")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "follow-camera":
        return configure_follow_camera(
            target=args.target,
            offset_text=args.offset,
            wait_s=parse_wait_s(args.wait_s, default=60, label="Gazebo GUI follow"),
        )
    if args.command == "world-running":
        return configure_world_running(
            world=args.world,
            wait_s=parse_wait_s(args.wait_s, default=60, label="Gazebo world unpause"),
        )
    print(f"ERROR: unsupported command: {args.command}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
