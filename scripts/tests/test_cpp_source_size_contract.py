#!/usr/bin/env python3
"""Contract tests for keeping C++ source files reviewable."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


MAX_SOURCE_LINES = 1000
SOURCE_SUFFIXES = {".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
IGNORED_PATH_PARTS = {"build", "install", "log"}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def tracked_source_files() -> list[Path]:
    root = repo_root()
    result = subprocess.run(
        ["git", "ls-files"],
        cwd=root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    files: list[Path] = []
    for line in result.stdout.splitlines():
        path = Path(line)
        if path.suffix not in SOURCE_SUFFIXES:
            continue
        if any(part in IGNORED_PATH_PARTS for part in path.parts):
            continue
        files.append(root / path)
    return files


class CppSourceSizeContractTest(unittest.TestCase):
    def test_tracked_cpp_sources_stay_under_size_limit(self) -> None:
        oversized: list[str] = []
        root = repo_root()
        for path in tracked_source_files():
            line_count = len(path.read_text(encoding="utf-8").splitlines())
            if line_count > MAX_SOURCE_LINES:
                relative = path.relative_to(root)
                oversized.append(f"{relative}: {line_count} lines")

        self.assertEqual(
            [],
            oversized,
            "Split oversized C++ source/header files; limit is "
            f"{MAX_SOURCE_LINES} lines.",
        )


if __name__ == "__main__":
    unittest.main()
