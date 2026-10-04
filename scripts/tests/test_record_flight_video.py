from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import record_flight_video as recorder  # noqa: E402
import rviz_recording_view as views  # noqa: E402


class RecordingCommandsTest(unittest.TestCase):
    SIZES = {"world": (854, 962), "follow": (820, 924), "top": (820, 924)}

    def test_the_flight_stores_three_raw_streams_and_joins_nothing(self) -> None:
        command = recorder.capture_command(
            self.SIZES, Path("log/runs/r1"),
            {name: Path(f"{name}.mkv") for name in recorder.SOURCES})
        text = " ".join(command)
        self.assertEqual(3, command.count("rawvideo"))
        self.assertIn("-video_size 854x962 -framerate 24 -i log/runs/r1/capture_world.bgra", text)
        self.assertIn("-video_size 820x924 -framerate 15 -i log/runs/r1/capture_top.bgra", text)
        # Whatever the host computes beside a flight slows the simulator:
        # no filter, and the GPU's encoder.
        self.assertNotIn("-filter_complex", command)
        self.assertEqual(3, command.count("h264_nvenc"))

    def test_the_split_picture_is_the_world_left_and_the_view_right(self) -> None:
        command = recorder.compose_command(Path("w.mkv"), Path("v.mkv"), 12.0, 100.0,
                                           Path("out.mp4"))
        graph = command[command.index("-filter_complex") + 1]
        self.assertIn("[0:v]vflip,fps=24,scale=960:1080", graph)
        self.assertIn("[1:v]vflip,fps=24,scale=960:1080", graph)
        self.assertIn("[w][v]hstack=inputs=2", graph)
        self.assertEqual(2, command.count("12.00"))
        self.assertEqual("out.mp4", command[-1])

    def test_the_cut_is_the_mission(self) -> None:
        log = (
            "[mission_monitor_node-7] [INFO] [100.5] [mission_monitor_node]: "
            "MISSION_READINESS ready=true\n"
            "[mission_monitor_node-7] [INFO] [300.0] [mission_monitor_node]: "
            "MISSION_RESULT success=true\n"
        )
        self.assertEqual((100.5, 300.0), recorder.mission_window(log))
        self.assertIsNone(recorder.mission_window("nothing"))

    def test_a_black_or_still_or_short_recording_is_not_one(self) -> None:
        good = {"width": 1920, "height": 1080, "duration_s": 206.0}
        picture = {"readable": True, "world_brightness": 5.0, "rviz_brightness": 40.0,
                   "change": 12.0}
        self.assertEqual([], recorder.verdict(good, picture, 200.0))
        self.assertTrue(recorder.verdict({**good, "duration_s": 90.0}, picture, 200.0))
        self.assertTrue(recorder.verdict(good, {**picture, "rviz_brightness": 1.0}, 200.0))
        self.assertTrue(recorder.verdict(good, {**picture, "change": 0.0}, 200.0))
        self.assertTrue(recorder.verdict({**good, "width": 1280}, picture, 200.0))
        # The world's half may be all but black: the location has no light.
        self.assertEqual([], recorder.verdict(good, {**picture, "world_brightness": 0.5}, 200.0))


class RecordingViewsTest(unittest.TestCase):
    def test_the_views_keep_the_displays_and_lose_the_panels(self) -> None:
        for name, view in (("city_nav_debug.rviz", "follow"),
                           ("city_nav_debug_top_down.rviz", "top")):
            source = (REPOSITORY / "drone_city_nav" / "rviz" / name).read_text()
            made = views.recording_view(source, view)
            self.assertTrue(made.startswith("Panels: []\n"))
            self.assertIn("Hide Left Dock: true", made)
            self.assertIn("Frame Rate: 15", made)
            self.assertEqual(source.count("Class: rviz_default_plugins/PointCloud2"),
                             made.count("Class: rviz_default_plugins/PointCloud2"))

    def test_the_top_view_follows_the_vehicle(self) -> None:
        source = (REPOSITORY / "drone_city_nav/rviz/city_nav_debug_top_down.rviz").read_text()
        current = views.recording_view(source, "top").split("    Current:\n", 1)[1]
        self.assertIn("Target Frame: drone_follow", current.split("    Saved:")[0])
        self.assertIn("Distance: 50", current)
        # The debugging configurations stay as they are.
        self.assertIn("Target Frame: gazebo_map", source)


class RecordingRuntimeContractTest(unittest.TestCase):
    def test_a_recorded_flight_is_the_headless_one_with_its_pictures(self) -> None:
        runner = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text()
        runtime = (REPOSITORY / "scripts/video_recording_runtime.sh").read_text()
        self.assertIn('record_video="$(normalize_bool "${RECORD_VIDEO:-false}")"', runner)
        for call in ("prepare_video_recording", "recorded_gazebo_gui", "recorded_top_view"):
            self.assertIn(call, runner)
            self.assertIn(f"{call}() {{", runtime)
        # Passed only by a recorded flight: an empty launch argument is
        # malformed and no flight starts (r1019).
        self.assertIn('ros_launch_args+=(rviz_environment:="${recording_rviz_environment}")',
                      runner)
        self.assertNotIn('\n    rviz_environment:=', runner)
        self.assertIn("gazebo_gui_recording.config", runtime)
        self.assertIn("frame_pace_shim.c", runtime)
        for view in ("world", "follow", "top"):
            self.assertIn(f"capture_{view}.bgra", runtime.replace("${view}", view))
        self.assertIn("  RECORD_VIDEO\n", (REPOSITORY / "scripts/container_run.sh").read_text())
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text()
        self.assertIn('"rviz_environment"', launch)

    def test_the_gazebo_window_holds_the_scene_alone_and_no_light(self) -> None:
        config = (REPOSITORY / "drone_city_nav/config/gazebo_gui_recording.config").read_text()
        self.assertEqual(3, config.count("<plugin "))
        for plugin in ("MinimalScene", "GzSceneManager", "CameraTracking"):
            self.assertIn(f'filename="{plugin}"', config)
        # The location's own light never comes back (specification A10).
        self.assertIn("<ambient_light>0 0 0</ambient_light>", config)

    def test_no_production_source_knows_of_the_recording(self) -> None:
        production = REPOSITORY / "drone_city_nav"
        for folder in ("src", "include"):
            for path in (production / folder).rglob("*"):
                if path.suffix in {".cpp", ".hpp"}:
                    text = path.read_text(errors="ignore")
                    self.assertNotIn("RECORD_VIDEO", text, str(path))
                    self.assertNotIn("FRAME_CAPTURE", text, str(path))

    def test_the_batch_flies_every_scenario_round_robin(self) -> None:
        batch = (REPOSITORY / "tools/record_batch.sh").read_text()
        for scenario in ("point-to-point", "--lidar point-to-point", "long-failures",
                         "light-lost", "light-failure", "low-battery", "return-home"):
            self.assertIn(f'"{scenario}"', batch)
        self.assertLess(batch.index('for pass in'), batch.index('for scenario in "${scenarios[@]}"'))
        flight = (REPOSITORY / "tools/record_flight.sh").read_text()
        self.assertIn("RECORD_VIDEO=1", flight)
        self.assertIn("gnome-session-inhibit --inhibit idle", flight)


if __name__ == "__main__":
    unittest.main()
