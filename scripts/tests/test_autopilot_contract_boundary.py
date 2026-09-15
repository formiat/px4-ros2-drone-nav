"""The navigation stack reads the autopilot through its contract, not through PX4's messages."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]

# The PX4 adapter (state and commands) and the simulation's PX4 heading feed:
# the only sources allowed to include px4_msgs.
PX4_ADAPTER_SOURCES = {
    "drone_city_nav/include/drone_city_nav/px4_autopilot_adapter.hpp",
    "drone_city_nav/include/drone_city_nav/px4_offboard_setpoint_io.hpp",
    "drone_city_nav/src/px4_autopilot_adapter.cpp",
    "drone_city_nav/src/px4_offboard_setpoint_io.cpp",
    "drone_city_nav/src/mppi_offboard_node.cpp",
    "drone_city_nav/src/simulation_heading_source_node.cpp",
    "drone_city_nav/tests/px4_autopilot_adapter_test.cpp",
    "drone_city_nav/tests/px4_offboard_setpoint_io_test.cpp",
}


class AutopilotContractBoundaryTest(unittest.TestCase):
    def test_only_the_px4_adapter_includes_px4_messages(self) -> None:
        listed = subprocess.run(
            ["git", "ls-files", "drone_city_nav/include", "drone_city_nav/src",
             "drone_city_nav/tests"],
            cwd=REPOSITORY, check=True, capture_output=True, text=True,
        ).stdout.split()
        offenders = []
        for relative in listed:
            if not relative.endswith((".cpp", ".hpp")):
                continue
            text = (REPOSITORY / relative).read_text(encoding="utf-8")
            if "px4_msgs" in text and relative not in PX4_ADAPTER_SOURCES:
                offenders.append(relative)
        self.assertEqual(offenders, [], "sources reading PX4 messages outside the adapter")
        for relative in PX4_ADAPTER_SOURCES:
            self.assertTrue((REPOSITORY / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()
