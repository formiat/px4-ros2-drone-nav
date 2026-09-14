from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from runtime_manifest import package_version  # noqa: E402


class PackageVersionTest(unittest.TestCase):
    def test_reads_the_package_version(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "drone_city_nav" / "package.xml"
            package.parent.mkdir()
            package.write_text(
                "<package>\n  <name>x</name>\n  <version>1.2.3</version>\n</package>\n",
                encoding="utf-8",
            )
            self.assertEqual(package_version(Path(directory)), "1.2.3")

    def test_reports_unknown_without_a_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(package_version(Path(directory)), "unknown")

    def test_the_repository_declares_a_release_version(self) -> None:
        self.assertRegex(package_version(REPOSITORY), r"^\d+\.\d+\.\d+$")


if __name__ == "__main__":
    unittest.main()
