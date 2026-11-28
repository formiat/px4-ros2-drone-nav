#!/usr/bin/env python3
"""What the repository says of itself to those who index it (roadmap item 62): DeepWiki's configuration within the
limits its generator enforces, and the citation file in step with the package."""
import json
import re
import unittest
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]


class DiscoverabilityTest(unittest.TestCase):
    def test_the_wiki_configuration_is_one_the_generator_accepts(self) -> None:
        # .devin/wiki.json: at most 30 pages with unique titles, a parent that is a page, at most 100 notes of at
        # most 10 000 characters; a file outside the limits is rejected and the wiki is planned without it.
        wiki = json.loads((REPOSITORY / ".devin/wiki.json").read_text(encoding="utf-8"))
        titles = [page["title"] for page in wiki["pages"]]
        self.assertTrue(1 <= len(titles) <= 30)
        self.assertEqual(len(titles), len(set(titles)))
        notes = list(wiki["repo_notes"])
        for page in wiki["pages"]:
            self.assertTrue(page["purpose"])
            self.assertIn(page.get("parent", titles[0]), titles)
            notes += page.get("page_notes", [])
        self.assertLessEqual(len(notes), 100)
        self.assertTrue(all(0 < len(note["content"]) <= 10000 for note in notes))
        # Every document a note or a page names exists.
        text = json.dumps(wiki)
        for path in set(re.findall(r"(?:docs|tools|scripts|drone_city_nav)/[\w/.]+\.(?:md|yaml|sh)", text)):
            self.assertTrue((REPOSITORY / path).is_file(), path)

    def test_the_citation_carries_the_package_version(self) -> None:
        citation = yaml.safe_load((REPOSITORY / "CITATION.cff").read_text(encoding="utf-8"))
        package = (REPOSITORY / "drone_city_nav/package.xml").read_text(encoding="utf-8")
        self.assertIn(f"<version>{citation['version']}</version>", package)
        self.assertIn(f"## v{citation['version']} ({citation['date-released']})",
                      (REPOSITORY / "CHANGELOG.md").read_text(encoding="utf-8"))
        self.assertIn('grep -q "^version: $version$" CITATION.cff',
                      (REPOSITORY / "tools/release_check.sh").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
