"""The cleaner of the shared-memory files stopped Fast DDS processes leave (scripts/fastdds_shm_cleanup.py)."""

import fcntl
import importlib.util
import tempfile
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fastdds_shm_cleanup", REPOSITORY / "scripts" / "fastdds_shm_cleanup.py")
cleanup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cleanup)


class FastDdsShmCleanupTest(unittest.TestCase):
    def test_what_no_living_process_holds_goes_and_what_one_holds_stays(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            dead, alive = "fastrtps_0123456789abcdef", "fastrtps_fedcba9876543210"
            names = [dead, dead + "_el", alive, alive + "_el", "fastrtps_port7411", "fastrtps_port7411_el",
                     "sem.fastrtps_port7411_mutex", "fastrtps_port7413", "fastrtps_port7413_sl",
                     "sem.fastrtps_port7413_mutex", "lttng-ust-wait-8", "fastrtps_note.txt"]
            for name in names:
                (directory / name).write_bytes(b"x")
            with (directory / (alive + "_el")).open("ab") as held, \
                    (directory / "fastrtps_port7413_sl").open("ab") as held_port:
                fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(held_port, fcntl.LOCK_SH | fcntl.LOCK_NB)
                self.assertEqual(0, cleanup.main(["--directory", temp, "--dry-run"]))
                self.assertEqual(sorted(names), sorted(path.name for path in directory.iterdir()), "a dry run")
                self.assertEqual(0, cleanup.main(["--directory", temp]))
            left = sorted(path.name for path in directory.iterdir())
            self.assertEqual(sorted([alive, alive + "_el", "fastrtps_port7413", "fastrtps_port7413_sl",
                                     "sem.fastrtps_port7413_mutex", "lttng-ust-wait-8", "fastrtps_note.txt"]), left)

    def test_a_missing_directory_is_nothing_to_clean(self) -> None:
        self.assertEqual(0, cleanup.main(["--directory", "/nonexistent/shm"]))


if __name__ == "__main__":
    unittest.main()
