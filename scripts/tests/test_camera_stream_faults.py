#!/usr/bin/env python3
"""Roadmap item 17 stage 5: the camera stream's failures stay an evaluation
component. The camera driver reads the pair from the relay's topics and is
never told when or how the stream fails."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
INJECTOR_PATH = REPOSITORY / "scripts" / "camera_stream_faults.py"
SPEC = importlib.util.spec_from_file_location("camera_stream_faults", INJECTOR_PATH)
assert SPEC is not None and SPEC.loader is not None
INJECTOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = INJECTOR  # its dataclass looks its module up
SPEC.loader.exec_module(INJECTOR)


class CameraStreamFaultContractTest(unittest.TestCase):
    def test_no_production_source_reads_the_injection(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for path in production.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".py", ".yaml", ".msg"}:
                continue
            text = path.read_text(errors="ignore")
            for word in ("stream_fault", "STREAM_FAULT", "camera_stream"):
                self.assertNotIn(word, text, f"{path} reads the injection")

    def test_the_runtime_points_the_driver_at_the_relay(self) -> None:
        runtime = (REPOSITORY / "scripts" / "run_drone_nav_sim.sh").read_text()
        self.assertIn(
            f"stereo_image_topic_suffix:={INJECTOR.RELAY_SUFFIX}", runtime)

    def test_schedules_are_seeded_and_of_the_two_kinds(self) -> None:
        first = INJECTOR.schedule("moderate", 3, 600.0)
        self.assertEqual(first, INJECTOR.schedule("moderate", 3, 600.0))
        self.assertNotEqual(first, INJECTOR.schedule("moderate", 4, 600.0))
        self.assertGreater(len(first), 10)
        for earlier, later in zip(first, first[1:]):
            self.assertGreaterEqual(later.start_s - earlier.end_s, 5.0)
        for fault in first:
            if fault.delay_s == 0.0:
                self.assertTrue(0.2 <= fault.length_s <= 1.0)
            else:
                self.assertTrue(1.0 <= fault.length_s <= 3.0)
                self.assertTrue(0.1 <= fault.delay_s <= 0.5)
        with self.assertRaises(ValueError):
            INJECTOR.schedule("unknown", 0, 60.0)

    def test_a_frame_is_placed_by_its_own_stamp(self) -> None:
        faults = [INJECTOR.StreamFault(10.0, 1.0, 0.0)]
        self.assertIsNone(INJECTOR.fault_at(faults, 9.9))
        self.assertIs(INJECTOR.fault_at(faults, 10.5), faults[0])
        self.assertIsNone(INJECTOR.fault_at(faults, 11.0))
        # gz.msgs.Image with a header stamp of 123.456 s and a width.
        stamp = bytes([0x08, 123, 0x10, 0x80, 0x84, 0xB8, 0xD9, 0x01])
        header = bytes([0x0A, len(stamp)]) + stamp
        message = bytes([0x0A, len(header)]) + header + bytes([0x10, 4])
        self.assertAlmostEqual(123.456, INJECTOR.stamp_of(message), places=6)


if __name__ == "__main__":
    unittest.main()
