#!/usr/bin/env python3
"""Roadmap item 17 stage 5: the carried light's failures stay an evaluation
component. The vehicle is told its battery's charge and never when or how its
light fails."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

REPOSITORY = Path(__file__).resolve().parents[2]
INJECTOR_PATH = REPOSITORY / "scripts" / "carried_light.py"
SPEC = importlib.util.spec_from_file_location("carried_light", INJECTOR_PATH)
assert SPEC is not None and SPEC.loader is not None
INJECTOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = INJECTOR  # its dataclass looks its module up
SPEC.loader.exec_module(INJECTOR)


class LightFaultInjectorContractTest(unittest.TestCase):
    def test_the_charge_is_all_it_publishes_on_ros(self) -> None:
        source = INJECTOR_PATH.read_text()
        self.assertEqual(1, source.count("create_publisher"))
        self.assertIn('create_publisher(Float64, "/carried_light/charge_s", 1)', source)

    def test_no_production_source_reads_the_injection(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for path in production.rglob("*"):
            if path.suffix not in {".cpp", ".hpp", ".py", ".yaml", ".msg"}:
                continue
            text = path.read_text(errors="ignore")
            for word in ("light_fault", "LIGHT_FAULT", "light_config"):
                self.assertNotIn(word, text, f"{path} reads the injection")

    def test_the_light_it_sends_is_the_models(self) -> None:
        model = ET.parse(REPOSITORY / "drone_city_nav/models/stereo_tof_v1/model.sdf")
        light = model.getroot().find(".//light[@name='carried_light']")
        self.assertIsNotNone(light)
        assert light is not None
        self.assertEqual(INJECTOR.NOMINAL_INTENSITY, float(light.findtext("intensity")))
        self.assertEqual(INJECTOR.LIGHT_RANGE_M, float(light.findtext("attenuation/range")))
        self.assertEqual(INJECTOR.ATTENUATION_CONSTANT,
                         float(light.findtext("attenuation/constant")))
        self.assertEqual(INJECTOR.ATTENUATION_QUADRATIC,
                         float(light.findtext("attenuation/quadratic")))
        self.assertEqual(INJECTOR.SPOT_INNER_ANGLE_RAD,
                         float(light.findtext("spot/inner_angle")))
        self.assertEqual(INJECTOR.SPOT_OUTER_ANGLE_RAD,
                         float(light.findtext("spot/outer_angle")))

    def test_schedules_are_seeded_and_inside_the_owners_range(self) -> None:
        for profile in ("moderate", "severe"):
            first = INJECTOR.schedule(profile, 7, 1800.0)
            self.assertEqual(first, INJECTOR.schedule(profile, 7, 1800.0))
            self.assertNotEqual(first, INJECTOR.schedule(profile, 8, 1800.0))
            previous_end = 0.0
            for outage in first:
                self.assertGreaterEqual(outage.start_s - previous_end, 1.0)
                self.assertLessEqual(outage.start_s - previous_end, 60.0)
                self.assertGreaterEqual(outage.dark_s, 1.0)
                self.assertLessEqual(outage.dark_s, 60.0)
                self.assertGreaterEqual(min(outage.ramp_down_s, outage.ramp_up_s),
                                        INJECTOR.SHORTEST_RAMP_S)
                previous_end = outage.end_s

    def test_an_outage_ramps_down_and_back(self) -> None:
        outage = INJECTOR.Outage(start_s=10.0, ramp_down_s=1.0, dark_s=2.0,
                                 ramp_up_s=1.0, floor=0.0)
        self.assertEqual(1.0, outage.share(9.9))
        self.assertAlmostEqual(0.5, outage.share(10.5))
        self.assertEqual(0.0, outage.share(12.0))
        self.assertAlmostEqual(0.5, outage.share(13.5))
        self.assertEqual(1.0, outage.share(14.0))


if __name__ == "__main__":
    unittest.main()
