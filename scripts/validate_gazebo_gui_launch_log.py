#!/usr/bin/env python3
"""Validate Gazebo GUI launch diagnostics emitted by run_city_mvp.sh."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


CRITICAL_GUI_PATTERN = re.compile(
    r"Segmentation fault"
    r"|Aborted"
    r"|Failed to load plugin"
    r"|Unable to find file"
    r"|No such file or directory"
    r"|could not find mesh"
    r"|failed to load .*mesh",
    re.IGNORECASE,
)
RENDER_WARNING_PATTERN = re.compile(
    r"libEGL warning|egl: failed to create|failed to create dri2 screen",
    re.IGNORECASE,
)


@dataclass
class ValidationResult:
    messages: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def require(self, label: str, condition: bool) -> None:
        if condition:
            self.messages.append(f"OK: {label}")
        else:
            self.errors.append(f"FAIL: {label}")

    def warn(self, message: str) -> None:
        self.messages.append(f"WARN: {message}")


def _read_summary_bool(summary: str, key: str) -> bool | None:
    match = re.search(rf"^{re.escape(key)}=(true|false)$", summary, re.MULTILINE)
    if match is None:
        return None
    return match.group(1) == "true"


def _read_summary_value(summary: str, key: str) -> str | None:
    match = re.search(rf"^{re.escape(key)}=([^\n]+)$", summary, re.MULTILINE)
    if match is None:
        return None
    return match.group(1).strip()


def _warn_on_capture_status(
    result: ValidationResult,
    *,
    label: str,
    status: str | None,
) -> None:
    if status is None:
        result.warn(f"Gazebo scene diagnostics did not report {label}_status")
    elif status != "ok":
        result.warn(f"Gazebo scene diagnostics {label} capture status={status}")


def _validate_gui_log(result: ValidationResult, text: str | None) -> None:
    if text is None:
        return
    result.require("Gazebo GUI log is captured", text is not None)
    if not text.strip():
        result.warn("Gazebo GUI log is empty")
        return
    if RENDER_WARNING_PATTERN.search(text):
        result.warn("Gazebo GUI log contains render-stack warnings")
    result.require(
        "Gazebo GUI log has no critical launch/resource errors",
        CRITICAL_GUI_PATTERN.search(text) is None,
    )


def _validate_scene_diagnostics(
    result: ValidationResult,
    scene_diagnostics_dir: Path | None,
) -> None:
    if scene_diagnostics_dir is None:
        return
    result.require("Gazebo scene diagnostics dir exists", scene_diagnostics_dir.is_dir())
    if not scene_diagnostics_dir.is_dir():
        return
    summary_path = scene_diagnostics_dir / "summary.txt"
    result.require("Gazebo scene diagnostics summary exists", summary_path.is_file())
    if not summary_path.is_file():
        return
    summary = summary_path.read_text(encoding="utf-8", errors="replace")
    target_model_seen = _read_summary_bool(summary, "target_model_seen")
    target_visual_seen = _read_summary_bool(summary, "target_visual_seen")
    yellow_visual_seen = _read_summary_bool(summary, "yellow_visual_seen")
    pose_info_status = _read_summary_value(summary, "pose_info_status")
    scene_info_status = _read_summary_value(summary, "scene_info_status")
    _warn_on_capture_status(result, label="pose_info", status=pose_info_status)
    _warn_on_capture_status(result, label="scene_info", status=scene_info_status)
    if target_model_seen is False and pose_info_status == "ok":
        result.require("Gazebo scene diagnostics target model is present", False)
    elif target_model_seen is False:
        result.warn(
            "Gazebo scene diagnostics did not confirm the target model because "
            "pose diagnostics were unavailable or incomplete"
        )
    else:
        result.require("Gazebo scene diagnostics target model is present", True)
    if target_model_seen is None:
        result.warn("Gazebo scene diagnostics did not report target_model_seen")
    if target_visual_seen is False:
        result.warn("Gazebo scene diagnostics did not observe x500 visual tokens")
    if yellow_visual_seen is False:
        result.warn("Gazebo scene diagnostics did not observe yellow marker visuals")


def validate_log(
    text: str,
    *,
    gui_log_text: str | None = None,
    scene_diagnostics_dir: Path | None = None,
) -> ValidationResult:
    result = ValidationResult()
    cleanup_seen = (
        "Gazebo stale cleanup: no conflicting Gazebo processes found" in text
        or "Gazebo stale cleanup: candidates=" in text
        or "WARNING: stale Gazebo process cleanup is disabled" in text
    )
    result.require("stale Gazebo cleanup status is logged", cleanup_seen)
    result.require(
        "Gazebo world unpause is confirmed",
        "Gazebo world running command confirmed" in text,
    )
    follow_best_effort = (
        "Gazebo GUI follow camera state confirmed" in text
        or "Gazebo GUI follow camera command accepted but state confirmation is unavailable"
        in text
        or "Gazebo GUI follow camera configured" in text
    )
    result.require("Gazebo GUI follow camera status is logged", follow_best_effort)
    if "WARNING: Gazebo GUI follow camera" in text:
        result.warn("Gazebo GUI follow camera emitted a warning")
    result.require("Gazebo GUI config override is absent", "--gui-config" not in text)
    _validate_gui_log(result, gui_log_text)
    _validate_scene_diagnostics(result, scene_diagnostics_dir)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--gui-log", type=Path)
    parser.add_argument("--scene-diagnostics-dir", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        text = args.log_file.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        print(f"FAIL: could not read Gazebo GUI log: {exc}", file=sys.stderr)
        return 1
    gui_log_text = None
    if args.gui_log is not None:
        try:
            gui_log_text = args.gui_log.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            print(f"FAIL: could not read Gazebo GUI client log: {exc}", file=sys.stderr)
            return 1
    result = validate_log(
        text,
        gui_log_text=gui_log_text,
        scene_diagnostics_dir=args.scene_diagnostics_dir,
    )
    for message in result.messages:
        print(message)
    for error in result.errors:
        print(error, file=sys.stderr)
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
