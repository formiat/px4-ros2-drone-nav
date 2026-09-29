#!/usr/bin/env python3
"""Every launch file of the package parses. A launch file is read only when a
flight starts, so a syntax error in one passed every other gate and ended a
flight at its launch (r786)."""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[2] / "drone_city_nav" / "launch"


class LaunchFilesParseTest(unittest.TestCase):
    def test_every_launch_file_parses(self) -> None:
        paths = sorted(LAUNCH.glob("*.py"))
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(path=path.name):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


if __name__ == "__main__":
    unittest.main()
