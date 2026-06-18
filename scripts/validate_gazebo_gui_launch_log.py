#!/usr/bin/env python3
"""Validate Gazebo GUI launch diagnostics emitted by run_city_mvp.sh."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path


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


def validate_log(text: str) -> ValidationResult:
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
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_file", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        text = args.log_file.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        print(f"FAIL: could not read Gazebo GUI log: {exc}", file=sys.stderr)
        return 1
    result = validate_log(text)
    for message in result.messages:
        print(message)
    for error in result.errors:
        print(error, file=sys.stderr)
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
