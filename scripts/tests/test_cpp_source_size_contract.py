#!/usr/bin/env python3
"""The source size contract: a hand-written source stays reviewable.

A production source (C++, CUDA, Python, shell, the launch files among them)
holds at most 1000 non-blank lines, a test source 1500. From 90 percent of
its limit a file is in the band: the test names it, and the file may only
shrink or be split from there, never grow, until it is back under the band
(the hysteresis the owner set on 2026-11-02: a file that reached the limit
is refactored by theme at once, not squeezed by a few lines to fit). The
previous size is the file's at HEAD; a file new to the repository has none.
"""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

PRODUCTION_LIMIT = 1000
TEST_LIMIT = 1500
BAND_SHARE = 0.9
SOURCE_SUFFIXES = {
    ".cc",
    ".cpp",
    ".cu",
    ".cuh",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".py",
    ".sh",
}
IGNORED_PATH_PARTS = {"build", "install", "log"}
TEST_PATH_PARTS = {"tests"}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def repository_source_files() -> list[Path]:
    root = repo_root()
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
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
        files.append(path)
    return files


def non_blank_lines(text: str) -> int:
    return sum(bool(line.strip()) for line in text.splitlines())


def limit_of(path: Path) -> int:
    return TEST_LIMIT if any(part in TEST_PATH_PARTS for part in path.parts) else PRODUCTION_LIMIT


def lines_at_head(path: Path) -> int | None:
    result = subprocess.run(
        ["git", "show", f"HEAD:{path.as_posix()}"],
        cwd=repo_root(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        return None
    return non_blank_lines(result.stdout)


class SourceSizeContractTest(unittest.TestCase):
    def test_tracked_sources_stay_under_their_limit(self) -> None:
        oversized: list[str] = []
        root = repo_root()
        for path in repository_source_files():
            if not (root / path).exists():
                continue
            count = non_blank_lines((root / path).read_text(encoding="utf-8"))
            limit = limit_of(path)
            if count > limit:
                oversized.append(f"{path}: {count} lines, limit {limit}")
        self.assertEqual(
            [],
            oversized,
            "Refactor the source by theme, in its own commit, to under "
            f"{BAND_SHARE:.0%} of its limit (production {PRODUCTION_LIMIT}, "
            f"tests {TEST_LIMIT} non-blank lines); never squeeze lines to fit.",
        )

    def test_a_source_in_the_band_only_shrinks(self) -> None:
        grown: list[str] = []
        in_band: list[str] = []
        root = repo_root()
        for path in repository_source_files():
            if not (root / path).exists():
                continue
            count = non_blank_lines((root / path).read_text(encoding="utf-8"))
            limit = limit_of(path)
            band = int(limit * BAND_SHARE)
            if count < band:
                continue
            in_band.append(f"{path}: {count} of {limit}")
            before = lines_at_head(path)
            if before is not None and before >= band and count > before:
                grown.append(f"{path}: {before} -> {count} lines (band from {band})")
        if in_band:
            print(
                "\nSources in the band (from 90 percent of their limit; they may only "
                "shrink or be split until under it):\n  " + "\n  ".join(in_band)
            )
        self.assertEqual(
            [],
            grown,
            "A source in the band grew: refactor it by theme, in its own commit, "
            "to under the band first.",
        )


if __name__ == "__main__":
    unittest.main()
