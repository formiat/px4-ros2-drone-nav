#!/usr/bin/env python3
"""tools/fix_series.sh names the flights specification A14 owes after a change, and no others."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

TOOL = Path(__file__).resolve().parents[2] / "tools" / "fix_series.sh"
ACCEPTANCE = "tools/world_series.sh FIRST urban-cameras:5 urban-lidar:5"


def chain(*scopes: str) -> list[str]:
    result = subprocess.run(["bash", str(TOOL), "-n", "5000", *scopes], check=True,
                            capture_output=True, text=True)
    return result.stdout.splitlines()


class FixSeriesTest(unittest.TestCase):
    def test_without_a_scope_it_is_urban_five_and_five_alone(self) -> None:
        self.assertEqual(chain(), [ACCEPTANCE])

    def test_a_scope_is_flown_before_the_acceptance_and_adds_no_other_world(self) -> None:
        bodies = chain("bodies")
        self.assertEqual(bodies[-1], ACCEPTANCE)
        self.assertEqual(len(bodies), 2)
        self.assertEqual(bodies[0].count("walking-body"), 6)
        for world in ("cave", "finals", "open-sky"):
            self.assertNotIn(world, " ".join(bodies))
        self.assertEqual(chain("tunnels"),
                         ["tools/world_series.sh FIRST finals-lidar:5", ACCEPTANCE])
        self.assertEqual(chain("signal"),
                         ["tools/world_series.sh FIRST open-sky-full:5",
                          'EXTRA="GNSS_FAULTS=lost@30:90" tools/world_series.sh FIRST open-sky-full:1',
                          ACCEPTANCE])

    def test_the_whole_set_is_the_release_scope_alone(self) -> None:
        flights = 0
        for line in chain("release"):
            flights += sum(int(group.rsplit(":", 1)[1]) for group in line.split()
                           if ":" in group and group.rsplit(":", 1)[1].isdigit())
        self.assertEqual(flights, 57)

    def test_an_unknown_scope_is_refused_before_any_flight(self) -> None:
        result = subprocess.run(["bash", str(TOOL), "-n", "5000", "everything"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
