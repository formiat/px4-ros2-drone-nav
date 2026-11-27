#!/usr/bin/env python3
"""The guide under docs/guide/ (roadmap item 49) points at what exists: every relative link of its pages leads to a
file or a directory of the repository, every row of the specification it names (K92, R1, A14, ...) is a row of
docs/specification.md, and the architecture map names every executable of the package and every node the
single-vehicle launch starts."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
GUIDE = REPOSITORY / "docs" / "guide"
LINK = re.compile(r"\]\(([^)\s]+)\)")
ROW = re.compile(r"(?<![\w/.-])([KFRAVI]\d{1,3})(?![\w-])")


class GuideLinksTest(unittest.TestCase):
    def pages(self) -> list[Path]:
        pages = sorted(GUIDE.glob("*.md"))
        self.assertTrue(pages, "docs/guide holds no page")
        return pages

    def test_every_relative_link_leads_to_something_in_the_repository(self) -> None:
        for page in self.pages():
            for target in LINK.findall(page.read_text("utf-8")):
                if target.startswith(("http://", "https://", "#", "mailto:")):
                    continue
                path = (page.parent / target.split("#", 1)[0]).resolve()
                with self.subTest(page=page.name, target=target):
                    self.assertTrue(path.exists(), f"{page.name}: {target} leads nowhere")
                    self.assertIn(REPOSITORY, [path, *path.parents], f"{page.name}: {target} leaves the repository")

    def test_every_specification_row_named_is_a_row(self) -> None:
        rows = set(re.findall(r"^\| ([KFRAVI]\d{1,3}) \|", (REPOSITORY / "docs/specification.md").read_text("utf-8"),
                              re.MULTILINE))
        for page in self.pages():
            for row in sorted(set(ROW.findall(page.read_text("utf-8")))):
                with self.subTest(page=page.name, row=row):
                    self.assertIn(row, rows, f"{page.name} names {row}, which the specification does not hold")

    def test_the_map_names_every_executable_and_every_launched_node(self) -> None:
        text = (GUIDE / "architecture_map.md").read_text("utf-8")
        cmake = (REPOSITORY / "drone_city_nav/CMakeLists.txt").read_text("utf-8")
        executables = set(re.findall(r"add_executable\(\s*([a-z][a-z_0-9]+)", cmake))
        executables.add("gazebo_stereo_depth_node")  # added through a variable (stereo_depth_processes)
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text("utf-8")
        launched = set(re.findall(r"executable=[\"']([a-z_0-9.]+)[\"']", launch))
        self.assertGreater(len(executables), 20)
        self.assertGreater(len(launched), 8)
        for name in sorted(executables | launched):
            with self.subTest(name=name):
                self.assertIn(f"`{name}`", text, f"the architecture map does not name {name}")


if __name__ == "__main__":
    unittest.main()
