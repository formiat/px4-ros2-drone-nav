#!/usr/bin/env python3
"""Keep ROS launch modules small enough to review and maintain."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
LAUNCH_DIRECTORY = REPOSITORY / "drone_city_nav" / "launch"
MAX_SOURCE_LINES = 1000


class LaunchSourceSizeContractTest(unittest.TestCase):
    def test_launch_sources_stay_within_limit(self) -> None:
        oversized = []
        for source in sorted(LAUNCH_DIRECTORY.rglob("*.py")):
            line_count = len(source.read_text(encoding="utf-8").splitlines())
            if line_count > MAX_SOURCE_LINES:
                oversized.append(
                    f"{source.relative_to(REPOSITORY)}: {line_count} lines"
                )

        self.assertEqual(
            [],
            oversized,
            f"Refactor launch sources larger than {MAX_SOURCE_LINES} lines.",
        )


if __name__ == "__main__":
    unittest.main()
