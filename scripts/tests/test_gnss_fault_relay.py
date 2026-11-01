#!/usr/bin/env python3
"""Roadmap item 39: the satellite signal's fault relay plays every variant of
the table on the autopilot's fixes, seeded, counted from the airborne moment."""

from __future__ import annotations

import importlib.util
import math
import random
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("gnss_fault_relay", REPOSITORY / "scripts/gnss_fault_relay.py")
assert SPEC is not None and SPEC.loader is not None
RELAY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = RELAY  # the dataclasses resolve their annotations through sys.modules
SPEC.loader.exec_module(RELAY)


def fix(t_s: float, lat: float = 37.4, lon: float = -122.0) -> "RELAY.Fix":
    return RELAY.Fix(timestamp_us=int(t_s * 1e6), latitude_deg=lat, longitude_deg=lon, altitude_m=40.0,
                     vel_n_mps=1.0, vel_e_mps=0.5, vel_d_mps=0.0, eph_m=0.3, epv_m=0.5, fix_type=3)


def metres(a: "RELAY.Fix", b: "RELAY.Fix") -> tuple[float, float]:
    to_m = math.pi / 180.0 * RELAY.EARTH_RADIUS_M
    return ((b.latitude_deg - a.latitude_deg) * to_m,
            (b.longitude_deg - a.longitude_deg) * to_m * math.cos(math.radians(a.latitude_deg)))


class GnssFaultRelayTest(unittest.TestCase):
    def test_the_schedule_is_parsed_with_its_defaults(self) -> None:
        faults = RELAY.parse_faults("lost@60:30; walk@100:40:2.5 ;jump@150:20")
        self.assertEqual([f.variant for f in faults], ["lost", "walk", "jump"])
        self.assertEqual(faults[1].magnitude, 2.5)
        self.assertEqual(faults[2].magnitude, 100.0)
        self.assertEqual(RELAY.parse_faults(""), [])
        with self.assertRaises(ValueError):
            RELAY.parse_faults("spoof@1:2")
        with self.assertRaises(ValueError):
            RELAY.parse_faults("lost@1")

    def test_every_variant_does_what_the_table_says(self) -> None:
        player = RELAY.FaultPlayer(RELAY.parse_faults(
            "lost@10:5; flap@20:8; jam@30:5; degraded@40:5:10; walk@50:10:2; jump@70:5:100; "
            "replay@80:10:5; bad_velocity@100:5; bad_time@110:5; return_offset@120:5:50"), random.Random(7))
        self.assertFalse(player.play(5.0, fix(5.0)).dropped)  # before every fault
        self.assertTrue(player.play(12.0, fix(12.0)).dropped)  # lost
        self.assertTrue(player.play(21.0, fix(21.0)).dropped)  # flap: off for two seconds first
        self.assertFalse(player.play(23.0, fix(23.0)).dropped)  # then on for two
        jammed = player.play(31.0, fix(31.0))
        self.assertEqual(jammed.fix_type, RELAY.FIX_TYPE_NONE)
        self.assertEqual(jammed.jamming_state, RELAY.JAMMING_STATE_DETECTED)
        self.assertGreaterEqual(jammed.eph_m, 50.0)
        degraded = player.play(41.0, fix(41.0))
        self.assertEqual(degraded.fix_type, 3)
        self.assertEqual(degraded.eph_m, 10.0)
        walked_early = player.play(51.0, fix(51.0))
        walked_late = player.play(59.0, fix(59.0))
        self.assertAlmostEqual(math.hypot(*metres(fix(51.0), walked_early)), 2.0, delta=0.05)
        self.assertAlmostEqual(math.hypot(*metres(fix(59.0), walked_late)), 18.0, delta=0.2)
        jumped = player.play(71.0, fix(71.0))
        self.assertAlmostEqual(math.hypot(*metres(fix(71.0), jumped)), 100.0, delta=0.5)
        for t in range(75, 84):
            player.play(float(t), fix(float(t), lat=37.4 + t * 1e-5))
        replayed = player.play(84.0, fix(84.0, lat=37.4 + 84 * 1e-5))
        self.assertAlmostEqual(replayed.latitude_deg, 37.4 + 79 * 1e-5, places=9)  # five seconds late
        self.assertEqual(replayed.timestamp_us, int(84.0 * 1e6))  # with the clock of now
        slow = player.play(101.0, fix(101.0))
        self.assertEqual((slow.vel_n_mps, slow.vel_e_mps), (0.0, 0.0))
        frozen = player.play(111.0, fix(111.0))
        self.assertEqual(player.play(113.0, fix(113.0)).timestamp_us, frozen.timestamp_us)
        self.assertTrue(player.play(122.0, fix(122.0)).dropped)  # the outage
        returned = player.play(130.0, fix(130.0))
        self.assertFalse(returned.dropped)
        self.assertAlmostEqual(math.hypot(*metres(fix(130.0), returned)), 50.0, delta=0.5)
        self.assertAlmostEqual(math.hypot(*metres(fix(200.0), player.play(200.0, fix(200.0)))), 50.0, delta=0.5)

    def test_the_seed_fixes_the_directions(self) -> None:
        a = RELAY.FaultPlayer(RELAY.parse_faults("jump@0:5:100"), random.Random(3)).play(1.0, fix(1.0))
        b = RELAY.FaultPlayer(RELAY.parse_faults("jump@0:5:100"), random.Random(3)).play(1.0, fix(1.0))
        c = RELAY.FaultPlayer(RELAY.parse_faults("jump@0:5:100"), random.Random(4)).play(1.0, fix(1.0))
        self.assertEqual((a.latitude_deg, a.longitude_deg), (b.latitude_deg, b.longitude_deg))
        self.assertNotEqual((a.latitude_deg, a.longitude_deg), (c.latitude_deg, c.longitude_deg))


if __name__ == "__main__":
    unittest.main()
