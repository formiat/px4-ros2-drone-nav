"""Every message of the package imports with its C type support in the installed workspace: a
message removed from the package leaves stale generated sources behind in a build tree that is
not cleaned, and the Python type support built from them fails to load with an undefined symbol
(r1212: the raw snapshot capturer could not import RawObstacleSnapshot3D after two messages
went). The check runs in a shell that sources ROS and the install tree, as the runtime scripts
do; it is skipped where either is missing, which is outside the container."""
import subprocess
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
INSTALL_SETUP = REPOSITORY / "install" / "setup.bash"
PROBE = """
import importlib, sys
messages = importlib.import_module("drone_city_nav.msg")
for name in sys.argv[1:]:
    getattr(messages, name).__class__.__import_type_support__()
print("ok")
"""


class MessageTypeSupportTest(unittest.TestCase):
    def test_every_message_imports_its_type_support(self) -> None:
        if not ROS_SETUP.is_file() or not INSTALL_SETUP.is_file():
            self.skipTest("ROS or the install tree is not available")
        names = sorted(path.stem for path in (REPOSITORY / "drone_city_nav" / "msg").glob("*.msg"))
        self.assertTrue(names)
        command = (
            f"source {ROS_SETUP} && source {INSTALL_SETUP} && cd / && "
            f"python3 -c '{PROBE}' {' '.join(names)}"
        )
        completed = subprocess.run(
            ["bash", "-c", command], capture_output=True, text=True, check=False
        )
        self.assertEqual(completed.returncode, 0, completed.stderr[-2000:])
        self.assertIn("ok", completed.stdout)
