"""Roadmap item 21: the warm bodies that walk through the location (scripts/moving_bodies.py)."""

import math
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import moving_bodies as bodies  # noqa: E402


class MovingBodiesTest(unittest.TestCase):
    def test_a_body_walks_back_and_forth_at_its_speed(self) -> None:
        walk = bodies.Walk(0.0, 0.0, 4.0, 3.0, 1.0, 1.25, phase=0.0)
        self.assertEqual(5.0, walk.length_m)
        self.assertEqual((0.0, 0.0), walk.state(0.0)[:2])
        x, y, vx, vy = walk.state(2.0)
        self.assertAlmostEqual(2.0, x)
        self.assertAlmostEqual(1.5, y)
        self.assertAlmostEqual(1.25, math.hypot(vx, vy))
        self.assertGreater(vx, 0.0)
        x, y, vx, vy = walk.state(6.0)  # 7.5 m walked: 2.5 m back from the far end
        self.assertAlmostEqual(2.0, x)
        self.assertAlmostEqual(1.5, y)
        self.assertLess(vx, 0.0)
        self.assertEqual(walk.state(1.0), walk.state(1.0 + 8.0), "one round takes 8 s")

    def test_the_seed_draws_where_each_walk_starts_and_a_standing_body_stands(self) -> None:
        text = "0,0,10,0,1,1.2; 5,5,5,9,2,1.0"
        first, second = bodies.parse_walks(text, 7), bodies.parse_walks(text, 7)
        self.assertEqual(first, second)
        self.assertNotEqual([w.phase for w in first], [w.phase for w in bodies.parse_walks(text, 8)])
        self.assertTrue(all(0.0 <= walk.phase < 1.0 for walk in first))
        standing = bodies.parse_walks("3,4,3,4,1,0", 1)[0]
        self.assertEqual((3.0, 4.0, 0.0, 0.0), standing.state(12.0))
        with self.assertRaises(ValueError):
            bodies.parse_walks("0,0,10,0,1", 1)
        with self.assertRaises(ValueError):
            bodies.parse_walks("0,0,10,0,1,9", 1)

    def test_the_body_is_a_person_sized_warm_dynamic_model(self) -> None:
        sdf = bodies.body_sdf(2, 1.0, 2.0, 3.0, Path("/tmp/texture.png"))
        self.assertIn('<model name="warm_body_2"><static>false</static>', sdf)
        self.assertIn("<pose>1.0 2.0 3.85 0 0 0</pose>", sdf)
        self.assertIn("<cylinder><radius>0.25</radius><length>1.7</length></cylinder>", sdf)
        self.assertIn("<temperature>307.0</temperature>", sdf)
        self.assertIn('<collision name="body">', sdf)

    def test_the_walks_stay_in_the_evaluation_component(self) -> None:
        # The walks never leave the process: the runtime starts it on MOVING_BODIES, the variable reaches the
        # container and the manifest, and no production source or launch file names it.
        runtime = (REPOSITORY / "scripts" / "runtime_evidence_runtime.sh").read_text()
        self.assertIn('if [[ -n "${MOVING_BODIES:-}" ]]; then', runtime)
        # With the equals sign: a walk that begins at a negative x is no option.
        self.assertIn('--bodies="${MOVING_BODIES}"', runtime)
        self.assertIn('--seed "${MOVING_BODY_SEED:-${IMPERFECTION_SEED:-0}}"', runtime)
        self.assertIn("    MOVING_BODIES MOVING_BODY_SEED \\\n    HEAT_SOURCES", runtime)
        container = (REPOSITORY / "scripts" / "container_run.sh").read_text()
        self.assertIn("  MOVING_BODIES\n  MOVING_BODY_SEED\n", container)
        production = [*(REPOSITORY / "drone_city_nav" / "src").rglob("*.[ch]pp"),
                      *(REPOSITORY / "drone_city_nav" / "include").rglob("*.hpp"),
                      *(REPOSITORY / "drone_city_nav" / "launch").glob("*.py")]
        self.assertGreater(len(production), 100)
        for source in production:
            text = source.read_text()
            self.assertNotIn("MOVING_BODIES", text, source.name)
            self.assertNotIn("moving_bodies", text, source.name)
            self.assertNotIn("warm_body", text, source.name)


class BodyEncountersTest(unittest.TestCase):
    def setUp(self) -> None:
        sys.path.insert(0, str(REPOSITORY / "tools"))
        import body_encounters  # noqa: PLC0415
        self.tool = body_encounters

    def test_the_distance_is_to_the_bodys_surface(self) -> None:
        body = (10.0, 5.0, 1.0)  # its axis and the floor it stands on
        distance, gap, over = self.tool.body_distance((12.25, 5.0, 2.0), body)
        self.assertAlmostEqual(2.0, distance)
        self.assertAlmostEqual(2.0, gap)
        self.assertLess(over, 0.0)
        distance, gap, over = self.tool.body_distance((10.1, 5.0, 3.3), body)
        self.assertAlmostEqual(0.6, distance)
        self.assertEqual(0.0, gap)
        self.assertAlmostEqual(0.6, over)

    def test_the_stop_way_is_the_reaction_and_the_braking(self) -> None:
        self.assertEqual(0.0, self.tool.stop_way_m(-1.0))
        self.assertAlmostEqual(4.0 * 0.3 + 16.0 / 8.0, self.tool.stop_way_m(4.0))

    def test_a_meeting_is_measured_at_the_closest_approach(self) -> None:
        # The vehicle flies along x at 2 m/s, 2.3 m over the floor; the body stands at x = 20 beside its line.
        truth = [(0.02 * k, 0.04 * k, 1.0, 3.3) for k in range(1000)]
        body = [(0.1 * k, 20.0, 0.0, 1.0) for k in range(200)]
        met = self.tool.meeting(truth, body)
        self.assertAlmostEqual(10.0, met["t_s"], delta=0.05)
        self.assertAlmostEqual(0.75, met["gap_m"], delta=0.01)
        self.assertAlmostEqual(2.0, met["speed_before_mps"], delta=0.05)
        self.assertGreater(met["closing_mps"], 1.5)
        self.assertLess(met["margin_m"], met["distance_m"])

    def test_the_tool_is_recorded(self) -> None:
        readme = (REPOSITORY / "tools/README.md").read_text(encoding="utf-8")
        self.assertIn("| `body_encounters.py` |", readme)
        self.assertIn("| `body_contact_rig.py` |", readme)


if __name__ == "__main__":
    unittest.main()
