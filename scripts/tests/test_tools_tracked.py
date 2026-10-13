#!/usr/bin/env python3
"""Every tool in tools/ is named on its page and parses: a figure a tracked
document cites has to come from a tool the repository carries and describes."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS = REPO_ROOT / "tools"


def tracked_tools() -> list[Path]:
    listed = subprocess.run(["git", "ls-files", "tools"], cwd=REPO_ROOT, check=True,
                            capture_output=True, text=True).stdout.split()
    return [REPO_ROOT / name for name in listed
            if Path(name).suffix in (".py", ".sh", ".cpp") and Path(name).name != "README.md"]


class ToolsTrackedTest(unittest.TestCase):
    def test_every_tool_has_a_line_on_its_page(self) -> None:
        page = (TOOLS / "README.md").read_text()
        missing = [str(path.relative_to(TOOLS)) for path in tracked_tools()
                   if str(path.relative_to(TOOLS)) not in page]
        self.assertEqual([], missing)

    def test_every_python_and_shell_tool_parses(self) -> None:
        for path in tracked_tools():
            with self.subTest(tool=str(path.relative_to(REPO_ROOT))):
                if path.suffix == ".py":
                    compile(path.read_text(), str(path), "exec")
                elif path.suffix == ".sh":
                    result = subprocess.run(["bash", "-n", str(path)], capture_output=True,
                                            text=True, check=False)
                    self.assertEqual(0, result.returncode, result.stderr)


if __name__ == "__main__":
    unittest.main()
