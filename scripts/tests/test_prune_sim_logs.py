from __future__ import annotations

import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "prune_sim_logs.sh"


def age(path: Path, days: float) -> None:
    stamp = time.time() - days * 86400.0
    os.utime(path, (stamp, stamp))


def write(path: Path, days_old: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("x", encoding="utf-8")
    age(path, days_old)


class PruneSimLogsTest(unittest.TestCase):
    def layout(self, root: Path) -> None:
        # Old, prunable entries.
        write(root / "log/runs/r001/ros.log", 10)
        write(root / "log/experiment_old/notes.txt", 30)
        write(root / "external/PX4-Autopilot/build/px4_sitl_default/rootfs/0/log/old.ulg", 12)
        # Recent entries and entries that are never pruned.
        write(root / "log/runs/r002/ros.log", 1)
        write(root / "log/runs/r003/ros.log", 20)
        write(root / "log/runs/r003/.keep", 20)
        write(root / "log/runs/r004/old.log", 20)
        write(root / "log/runs/r004/fresh.log", 0.1)
        write(root / "log/tools/tool.py", 40)
        write(root / "external/PX4-Autopilot/build/px4_sitl_default/rootfs/0/log/new.ulg", 0.5)
        # Directories carry their own old mtimes.
        for directory in (
            "log/runs/r001", "log/experiment_old", "log/runs/r003", "log/runs/r004",
            "log/tools", "log/runs", "log",
        ):
            age(root / directory, 30)
        # A symbolic link out of the repository is left alone.
        outside = root.parent / "outside"
        outside.mkdir(exist_ok=True)
        write(outside / "secret.log", 30)
        (root / "log/outside_link").symlink_to(outside)

    def run_script(self, root: Path, *arguments: str) -> str:
        completed = subprocess.run(
            [str(SCRIPT), "--root", str(root), *arguments],
            check=True,
            capture_output=True,
            text=True,
        )
        return completed.stdout

    def test_prunes_only_old_entries_of_the_known_directories(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "repo"
            root.mkdir()
            self.layout(root)

            preview = self.run_script(root, "--dry-run")
            self.assertIn("would delete 3 entries", preview)
            self.assertTrue((root / "log/runs/r001").exists())

            result = self.run_script(root)
            self.assertIn("deleted 3 entries older than 7 days", result)
            self.assertFalse((root / "log/runs/r001").exists())
            self.assertFalse((root / "log/experiment_old").exists())
            px4_log = root / "external/PX4-Autopilot/build/px4_sitl_default/rootfs/0/log"
            self.assertFalse((px4_log / "old.ulg").exists())
            self.assertTrue((px4_log / "new.ulg").exists())
            self.assertTrue((root / "log/runs/r002/ros.log").exists())
            self.assertTrue((root / "log/runs/r003/ros.log").exists())
            self.assertTrue((root / "log/runs/r004/old.log").exists())
            self.assertTrue((root / "log/tools/tool.py").exists())
            self.assertTrue((root / "log/outside_link").is_symlink())
            self.assertTrue((root.parent / "outside/secret.log").exists())

    def test_window_and_switch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "repo"
            root.mkdir()
            self.layout(root)

            disabled = subprocess.run(
                [str(SCRIPT), "--root", str(root)],
                check=True,
                capture_output=True,
                text=True,
                env={**os.environ, "DRONE_GAZEBO_PRUNE_LOGS": "false"},
            ).stdout
            self.assertIn("disabled", disabled)
            self.assertTrue((root / "log/runs/r001").exists())

            wide = self.run_script(root, "--dry-run", "--older-than-days", "60")
            self.assertIn("would delete 0 entries", wide)
            with self.assertRaises(subprocess.CalledProcessError):
                self.run_script(root, "--older-than-days", "0")


if __name__ == "__main__":
    unittest.main()
