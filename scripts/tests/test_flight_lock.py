#!/usr/bin/env python3
"""One flight chain a checkout at a time (roadmap item 43 point 7): scripts/flight_lock_runtime.sh."""

from __future__ import annotations

import shutil
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
LOCK_RUNTIME = REPOSITORY / "scripts" / "flight_lock_runtime.sh"
# The tools that clean the checkout's simulation round their flights, and the wrapper every sim_*.sh runs.
CLEANING_TOOLS = (
    "tools/series2.sh",
    "tools/fly_until_valid.sh",
    "tools/cores_series.sh",
    "tools/imperfections_series.sh",
    "tools/record_until_pass.sh",
    "tools/stereo_depth_accuracy.sh",
)


class FlightLockTest(unittest.TestCase):
    def setUp(self) -> None:
        # A checkout of its own: the lock lives under the log/ beside the script that is sourced.
        self.checkout = Path(tempfile.mkdtemp(prefix="flight_lock_"))
        self.addCleanup(shutil.rmtree, self.checkout, ignore_errors=True)
        (self.checkout / "scripts").mkdir()
        shutil.copy(LOCK_RUNTIME, self.checkout / "scripts")
        self.owner = self.checkout / "log" / ".flight-lock" / "owner"

    def fly(self, body: str, **popen: object) -> subprocess.Popen:
        return subprocess.Popen(
            ["bash", "-c", f"source scripts/flight_lock_runtime.sh; {body}", "flight", "r1"],
            cwd=self.checkout, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **popen)

    def hold(self) -> subprocess.Popen:
        holder = self.fly("echo held; exec sleep 60")
        self.addCleanup(self.end, holder)
        self.assertEqual("held\n", holder.stdout.readline())
        return holder

    @staticmethod
    def end(process: subprocess.Popen) -> None:
        process.kill()
        process.communicate()

    def lock_is_free(self) -> bool:
        return subprocess.run(["flock", "--nonblock", str(self.owner), "true"]).returncode == 0

    def test_a_second_flight_is_refused_with_status_3_and_the_owner_named(self) -> None:
        holder = self.hold()
        second = self.fly("echo flew")
        out, err = second.communicate(timeout=10)
        self.assertEqual(3, second.returncode)
        self.assertEqual("", out, "nothing of the second flight ran")
        self.assertIn("Refusing to fly", err)
        self.assertIn(f"pid={holder.pid} ", err)
        self.assertIn("command=flight r1", err)
        # The refusal changed nothing: the first flight still holds the lock under its own name.
        self.assertFalse(self.lock_is_free())
        self.assertIn(f"pid={holder.pid} ", self.owner.read_text())

    def test_a_dead_owners_lock_is_taken_by_the_next_flight(self) -> None:
        holder = self.hold()
        holder.send_signal(signal.SIGKILL)
        holder.wait(timeout=10)
        second = self.fly("echo flew")
        out, _ = second.communicate(timeout=10)
        self.assertEqual(0, second.returncode)
        self.assertEqual("flew\n", out)
        self.assertIn(f"pid={second.pid} ", self.owner.read_text())

    def test_no_lock_is_left_after_a_flight_or_a_refusal(self) -> None:
        first = self.fly("echo flew")
        first.communicate(timeout=10)
        self.assertEqual(0, first.returncode)
        self.assertTrue(self.lock_is_free(), "after a flight's ordinary end")
        holder = self.hold()
        refused = self.fly("echo flew")
        refused.communicate(timeout=10)
        self.assertEqual(3, refused.returncode)
        holder.terminate()
        holder.wait(timeout=10)
        deadline = time.monotonic() + 5.0
        while not self.lock_is_free() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(self.lock_is_free(), "after the holder's end, with a refusal in between")
        # The weekly pruning of log/ never deletes the lock from under a long chain.
        self.assertTrue((self.owner.parent / ".keep").exists())

    def test_what_a_holder_starts_is_its_own_and_is_not_refused(self) -> None:
        chain = self.fly("bash -c 'source scripts/flight_lock_runtime.sh; echo inner flew'")
        out, _ = chain.communicate(timeout=10)
        self.assertEqual(0, chain.returncode)
        self.assertEqual("inner flew\n", out)

    def test_whatever_cleans_or_flies_takes_the_lock_first(self) -> None:
        # A tool that could not take the lock must not reach its cleanup: the cleanup stops this checkout's
        # simulation, which is the other chain's flight.
        for name in CLEANING_TOOLS:
            text = (REPOSITORY / name).read_text()
            self.assertLess(text.index(". scripts/flight_lock_runtime.sh"), text.index("stop_sim.sh >"), name)
        wrapper = (REPOSITORY / "scripts/run_sim_wrapped.sh").read_text()
        self.assertLess(wrapper.index("flight_lock_runtime.sh\""), wrapper.index("cleanup_sim_processes.sh\""))
        series = (REPOSITORY / "tools/world_series.sh").read_text()
        self.assertLess(series.index(". scripts/flight_lock_runtime.sh"), series.index("tools/fly_until_valid.sh \"$"))
        # A flight started by make alone, inside any container, takes it before it writes a run's record; one
        # started by a holder on the host is told so through the container's environment.
        runner = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text()
        self.assertLess(runner.index("flight_lock_runtime.sh\""), runner.index("initialize_runtime_evidence_paths"))
        self.assertIn("  DRONE_GAZEBO_FLIGHT_LOCK_HELD\n", (REPOSITORY / "scripts/container_run.sh").read_text())


if __name__ == "__main__":
    unittest.main()
