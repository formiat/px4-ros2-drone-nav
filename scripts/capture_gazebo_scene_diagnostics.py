#!/usr/bin/env python3
"""Capture bounded Gazebo scene diagnostics for the drone navigation launcher."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
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
class TopicCapture:
    label: str
    topic: str
    file_name: str
    result: CommandResult

    @property
    def text(self) -> str:
        return self.result.combined_output

    @property
    def status(self) -> str:
        if self.result.returncode != 0:
            return "failed"
        if not self.text.strip():
            return "empty"
        return "ok"


EXPECTED_X500_VISUAL_TOKENS = (
    "base_link_visual",
    "rotor_0_visual",
    "rotor_1_visual",
    "rotor_2_visual",
    "rotor_3_visual",
)
EXPECTED_YELLOW_VISUAL_TOKENS = (
    "yellow_body_plate",
    "yellow_arm_x",
    "yellow_arm_y",
    "yellow_rotor_front_left",
    "yellow_rotor_front_right",
    "yellow_rotor_back_left",
    "yellow_rotor_back_right",
    "yellow_drone_locator_core",
    "yellow_ground_projection_beam",
    "yellow_ground_projection_disc",
)


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


def format_bool(value: bool) -> str:
    return "true" if value else "false"


def format_float(value: float) -> str:
    return f"{value:g}"


def capture_topic(
    *,
    label: str,
    topic: str,
    file_name: str,
    duration_s: float,
    timeout_s: float,
    output_dir: Path,
    runner: CommandRunner,
) -> TopicCapture:
    result = runner(
        [
            "topic",
            "-e",
            "-t",
            topic,
            "-d",
            format_float(duration_s),
        ],
        timeout_s,
    )
    capture = TopicCapture(label, topic, file_name, result)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / file_name).write_text(capture.text, encoding="utf-8")
    return capture


def text_contains_any(text: str, tokens: tuple[str, ...]) -> bool:
    return any(token in text for token in tokens)


def build_summary(
    *,
    target: str,
    captures: list[TopicCapture],
) -> list[str]:
    capture_by_label = {capture.label: capture for capture in captures}
    pose_text = capture_by_label["pose_info"].text
    scene_text = capture_by_label["scene_info"].text
    tracked_text = capture_by_label["currently_tracked"].text
    combined_scene_text = "\n".join((pose_text, scene_text))

    target_model_seen = target in combined_scene_text
    target_visual_seen = text_contains_any(
        combined_scene_text, EXPECTED_X500_VISUAL_TOKENS
    )
    yellow_visual_seen = text_contains_any(
        combined_scene_text, EXPECTED_YELLOW_VISUAL_TOKENS
    )
    gui_tracking_target_seen = target in tracked_text

    lines = [
        "Gazebo scene diagnostics summary:",
        f"target={target}",
        f"target_model_seen={format_bool(target_model_seen)}",
        f"target_visual_seen={format_bool(target_visual_seen)}",
        f"yellow_visual_seen={format_bool(yellow_visual_seen)}",
        f"gui_tracking_target_seen={format_bool(gui_tracking_target_seen)}",
    ]
    for capture in captures:
        lines.append(f"{capture.label}_topic={capture.topic}")
        lines.append(f"{capture.label}_status={capture.status}")
        if capture.result.returncode != 0:
            lines.append(
                f"WARNING: {capture.label} capture failed: "
                f"{capture.result.combined_output}"
            )
        elif not capture.text.strip():
            lines.append(f"WARNING: {capture.label} capture was empty")
    if not target_model_seen and pose_text.strip():
        lines.append(
            "WARNING: target model was not found in non-empty Gazebo pose/scene data"
        )
    if not yellow_visual_seen and combined_scene_text.strip():
        lines.append("WARNING: yellow visibility marker visuals were not observed")
    return lines


def capture_diagnostics(
    *,
    world: str,
    target: str,
    output_dir: Path,
    topic_duration_s: float,
    command_timeout_s: float,
    runner: CommandRunner = default_runner,
) -> list[str]:
    topics = [
        (
            "pose_info",
            f"/world/{world}/pose/info",
            "pose_info.txt",
        ),
        (
            "scene_info",
            f"/world/{world}/scene/info",
            "scene_info.txt",
        ),
        (
            "currently_tracked",
            "/gui/currently_tracked",
            "currently_tracked.txt",
        ),
    ]
    captures = [
        capture_topic(
            label=label,
            topic=topic,
            file_name=file_name,
            duration_s=topic_duration_s,
            timeout_s=command_timeout_s,
            output_dir=output_dir,
            runner=runner,
        )
        for label, topic, file_name in topics
    ]
    summary = build_summary(target=target, captures=captures)
    (output_dir / "summary.txt").write_text(
        "\n".join(summary) + "\n", encoding="utf-8"
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="generated_city")
    parser.add_argument("--target", default="x500_lidar_2d_0")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--topic-duration-s", default=0.8, type=float)
    parser.add_argument("--command-timeout-s", default=3.0, type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        summary = capture_diagnostics(
            world=args.world,
            target=args.target,
            output_dir=args.output_dir,
            topic_duration_s=args.topic_duration_s,
            command_timeout_s=args.command_timeout_s,
        )
    except OSError as exc:
        print(f"ERROR: failed to write Gazebo scene diagnostics: {exc}", file=sys.stderr)
        return 1

    for line in summary:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
