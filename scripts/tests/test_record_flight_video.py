from __future__ import annotations

import importlib.util
import re
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
        # Each window's frames are decoded up to the flight's end, where its
        # clock ends (the world window lives on after the launch's shutdown,
        # r1179), as halves the right way up, and the recording is written
        # from the split pictures at the frame rate.
        decode = recorder.decode_command(Path("w.mkv"), 600.0)
        self.assertEqual(["-t", "600.000", "-i", "w.mkv", "-vf", "vflip,scale=960:1080"],
                         decode[decode.index("-t"):decode.index("-t") + 6])
        self.assertNotIn("-ss", decode)
        encode = recorder.encode_command(Path("out.mp4"))
        self.assertIn("-video_size 1920x1080 -framerate 24 -i -", " ".join(encode))
        self.assertEqual("out.mp4", encode[-1])

    def test_every_frame_shows_its_own_moment_of_the_flight(self) -> None:
        # The owner's rule of 2026-10-08: a second of the recording is a
        # second of the flight at every moment, no frame further than half
        # a second from its moment. The host holds the simulator unevenly:
        # here a flight at a half for twenty wall seconds, at a quarter for
        # the next twenty, 24 frames a wall second. One pace over the whole
        # (r1179) put frames 6.9 s from their moment.
        clock = ([(1000.0 + 0.1 * step, 50.0 + 0.05 * step, 0.0, 0.0, 5.0)
                  for step in range(200)] +
                 [(1020.0 + 0.1 * step, 60.0 + 0.025 * step, 0.0, 0.0, 5.0)
                  for step in range(201)])
        table, offset = recorder.frame_table(clock, 1000.0, 24, 50.0, 15.0)
        self.assertEqual(15 * 24, len(table))
        # Ten seconds of flight at a half are twenty wall seconds, 480
        # frames; the next five at a quarter are twenty wall seconds too:
        # the table doubles its stride where the simulator slowed.
        self.assertEqual(0, table[0])
        self.assertEqual(240, table[5 * 24])
        self.assertEqual(480, table[10 * 24])
        # The last frame, 14.96 s of flight: 19.83 wall seconds at a quarter.
        self.assertEqual(956, table[-1])
        self.assertTrue(all(b >= a for a, b in zip(table, table[1:])))
        # A frame comes every 1/24 wall second, 1/6 flight second at a
        # quarter: no frame further than that from its moment.
        self.assertLess(offset, 1.0 / 6.0 + 1.0e-6)
        self.assertLess(offset, recorder.MAXIMUM_FRAME_OFFSET_S)

    CLOCK = [(1000.0 + 0.1 * step, 50.0 + 0.05 * step, 0.2 * min(step, 300), 0.0, 5.0)
             for step in range(601)]

    def test_the_recording_ends_with_the_missions_result(self) -> None:
        # The pose record goes on while the launch winds down (r1194: 3.2 s
        # after the result, the nodes gone and RViz's clouds with them).
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            self.assertEqual(2000.0, recorder.flight_end_wall_s(run, 2000.0))
            (run / "ros_drone_nav.log").write_text(
                "[mission_monitor_node-8] [ERROR] [1990.250000000] [mission_monitor_node]: "
                "MISSION_RESULT success=false reason='vehicle_landed'\n")
            self.assertAlmostEqual(1990.75, recorder.flight_end_wall_s(run, 2000.0))
            self.assertEqual(1985.0, recorder.flight_end_wall_s(run, 1985.0))

    def test_the_frames_are_timed_by_the_simulation_clock(self) -> None:
        # Sixty wall seconds of a flight slowed to a half are thirty of flight.
        self.assertAlmostEqual(50.0, recorder.simulation_time_at(self.CLOCK, 1000.0))
        self.assertAlmostEqual(65.0, recorder.simulation_time_at(self.CLOCK, 1030.0))
        self.assertAlmostEqual(80.0, recorder.simulation_time_at(self.CLOCK, 2000.0))

    def test_the_slideshow_is_looked_for_where_the_vehicle_moves(self) -> None:
        # 4 m/s for fifteen simulation seconds, then a hover.
        self.assertEqual([0.0, 5.0, 10.0], recorder.moving_windows(self.CLOCK, 50.0, 80.0))

    def test_a_different_frame_is_read_at_the_dark_pictures_scale(self) -> None:
        # The dark hall's world half (roadmap item 29) changes slowly: at the
        # 400/200 the rule was written with it showed 1.2 to 3.6 different
        # frames a second while its window redrew at its rate (r1488); at
        # 100/50, 10 to 13, a still picture 0 and an Urban flight 24.
        self.assertEqual("hi=100:lo=50", recorder.DECIMATE_THRESHOLDS)
        source = (REPOSITORY / "scripts/record_flight_video.py").read_text()
        self.assertIn("mpdecimate={DECIMATE_THRESHOLDS}", source)

    def test_a_black_or_still_or_short_recording_is_not_one(self) -> None:
        good = {"width": 1920, "height": 1080, "duration_s": 200.4}
        picture = {"readable": True, "world_brightness": 5.0, "rviz_brightness": 40.0,
                   "change": 12.0}
        still = {"world": 0.0, "follow": 0.01}
        distinct = {"world": 19.0, "RViz": 14.0}
        self.assertEqual([], recorder.verdict(good, picture, 200.0, still, distinct, 0.07))
        # Shorter than a minute, or off the flight's clock by more than a second.
        self.assertTrue(recorder.verdict({**good, "duration_s": 37.0}, picture, 37.0, still,
                                         distinct, 0.07))
        self.assertTrue(recorder.verdict({**good, "duration_s": 206.0}, picture, 200.0, still,
                                         distinct, 0.07))
        # Or with a frame further than half a second from its moment.
        self.assertTrue(recorder.verdict(good, picture, 200.0, still, distinct, 0.6))
        self.assertTrue(recorder.verdict(good, {**picture, "rviz_brightness": 1.0}, 200.0,
                                         still, distinct, 0.07))
        self.assertTrue(recorder.verdict(good, {**picture, "change": 0.0}, 200.0, still,
                                         distinct, 0.07))
        self.assertTrue(recorder.verdict({**good, "width": 1280}, picture, 200.0, still,
                                         distinct, 0.07))
        # The world's half may be all but black: the location has no light.
        self.assertEqual([], recorder.verdict(good, {**picture, "world_brightness": 0.5},
                                              200.0, still, distinct, 0.07))
        # A window that stood still, or a half that showed one frame a second
        # while the vehicle moved, is a slideshow (r1030 to r1058).
        self.assertTrue(recorder.verdict(good, picture, 200.0, {**still, "world": 0.9},
                                         distinct, 0.07))
        self.assertTrue(recorder.verdict(good, picture, 200.0, still,
                                         {**distinct, "world": 1.0}, 0.07))

    def test_the_redraws_of_a_window_are_read_second_by_second(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "capture_world.bgra.rate").write_text("100 24\n101 24\n103 1\n104 24\n")
            # The second with no line is a second with no redraw.
            self.assertEqual([24, 0, 1], recorder.redraws(run, "world", 100.2, 104.0))
            self.assertEqual([], recorder.redraws(run, "top", 100.2, 104.0))


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
        # The Gazebo window never waits for a blank screen to present a frame.
        self.assertIn("__GL_SYNC_TO_VBLANK=0 vblank_mode=0", runtime)
        self.assertIn(".rate", (REPOSITORY / "scripts/frame_pace_shim.c").read_text())
        # The carried light has no gizmo for the window to draw (r1041, r1057).
        rig = (REPOSITORY / "drone_city_nav/models/stereo_tof_v1/model.sdf").read_text()
        light = rig.split('<light name="carried_light"', 1)[1].split("</light>", 1)[0]
        self.assertIn("<visualize>false</visualize>", light)
        self.assertNotIn("visualize_visual",
                         (REPOSITORY / "scripts/carried_light.py").read_text())
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
        for scenario in ("point-to-point", "light-lost", "light-failure", "low-battery"):
            self.assertIn(f'"{scenario}"', batch)
        # The long flight to B and back and the goal-outside flight are set
        # aside (the owner, 2026-10-08): not in the batch, refused by the
        # flight without the owner's word.
        flight = (REPOSITORY / "tools/record_flight.sh").read_text()
        for scenario in ("long-failures", "return-home"):
            self.assertNotIn(f'"{scenario}"', batch)
            self.assertIn(f'{scenario}) [ "${{FLY_SET_ASIDE:-0}}" = 1 ] ||', flight)
        self.assertLess(batch.index('for pass in'), batch.index('for scenario in "${scenarios[@]}"'))
        # A lidar flight is not recorded (specification A11).
        self.assertNotIn("lidar", batch)
        flight = (REPOSITORY / "tools/record_flight.sh").read_text()
        self.assertIn("RECORD_VIDEO=1", flight)
        self.assertIn("gnome-session-inhibit --inhibit idle", flight)


class RenderedFlightTest(unittest.TestCase):
    """Part 6 of the technical-debt goal: the pictures drawn after the flight."""

    def test_a_rendered_flight_is_the_ordinary_one_with_no_window(self) -> None:
        tool = (REPOSITORY / "tools/render_flight.sh").read_text()
        for setting in ("RENDER_BAG=1", "ENABLE_RVIZ=true", "RVIZ_CONFIG_FILE=none",
                        "fly_until_valid.sh"):
            self.assertIn(setting, tool)
        self.assertNotIn("RECORD_VIDEO", tool)
        self.assertNotIn("REAL_TIME_FACTOR", tool)
        launch = (REPOSITORY / "drone_city_nav/launch/city_nav.launch.py").read_text()
        self.assertIn('rviz_config.perform(context) == "none"', launch)
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text()
        self.assertIn('if bool_is_true "${RENDER_BAG:-false}"; then', runtime)
        self.assertIn("ros2 bag record --use-sim-time", runtime)

    def test_the_vehicle_s_own_frame_is_kept_and_pasted_into_the_world_half(self) -> None:
        # The bag records the pair's left frame after the smoke relay, the
        # replay keeps it a quarter scale beside the windows, and the follow
        # view's world half carries the latest frame at each moment, a quarter
        # of the half's width in the lower right corner with a white border.
        import numpy
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text()
        self.assertIn("/stereo/left/image", runtime)
        render = (REPOSITORY / "scripts/render_flight_runtime.sh").read_text()
        self.assertIn("render_camera_frames.py", render)
        self.assertIn('camera_frames(run_directory / "render_camera") if view == "follow" else None',
                      (REPOSITORY / "scripts/render_flight_video.py").read_text())
        frames = [(10.0, Path("a"), 4, 3), (10.5, Path("b"), 4, 3), (11.0, Path("c"), 4, 3)]
        self.assertEqual(-1, recorder.camera_frame_at(frames, -1, 9.9))
        self.assertEqual(1, recorder.camera_frame_at(frames, -1, 10.7))
        self.assertEqual(2, recorder.camera_frame_at(frames, 1, 12.0))
        picture = numpy.zeros((recorder.HEIGHT, 2 * recorder.HALF_WIDTH, 3), dtype=numpy.uint8)
        frame = bytes(range(0, 256, 256 // 12))[:12]
        recorder.paste_inset(picture, frame, 4, 3)
        right = recorder.HALF_WIDTH - recorder.INSET_MARGIN
        bottom = recorder.HEIGHT - recorder.INSET_MARGIN
        height = recorder.INSET_WIDTH * 3 // 4
        self.assertEqual(255, picture[bottom, right, 0])  # the border
        self.assertEqual(frame[-1], picture[bottom - 1, right - 1, 0])  # the last pixel
        self.assertEqual(frame[0], picture[bottom - height, right - recorder.INSET_WIDTH, 0])
        self.assertEqual(0, picture[bottom - height - 10, right - recorder.INSET_WIDTH - 10, 0])
        self.assertEqual(0, picture[:, recorder.HALF_WIDTH:].max())  # the view half untouched
        import render_camera_frames as dumper
        frame = bytes([10] * 64 + [50] * 64 + [90] * 64 + [130] * 64)  # four bands of 4 rows
        data, width, height = dumper.quarter(frame, 16, 16)
        self.assertEqual((4, 4), (width, height))
        self.assertEqual(bytes([10, 10, 10, 10, 50, 50, 50, 50]), data[:8])  # each block's mean
        self.assertNotIn("--smoke-zones", runtime)  # RViz shows no obstacle of the truth

    def test_the_replayed_world_draws_the_smoke_the_vehicle_cannot_see(self) -> None:
        import render_world_replay as replay

        sdf = replay.smoke_box_sdf(2, ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 1.0))
        self.assertIn('<model name="smoke_zone_2">', sdf)
        self.assertIn("<pose>1.0 2.0 3.0 0 0 0</pose>", sdf)
        self.assertIn("<size>4.0 5.0 6.0</size>", sdf)
        self.assertNotIn("<collision", sdf)
        self.assertIn(f"<visibility_flags>{replay.SMOKE_VISIBILITY_FLAGS}</visibility_flags>", sdf)
        # The vehicle's cameras and lidar render no visual on that layer.
        for model in ("stereo_tof_v1", "lidar_3d_v1"):
            text = (REPOSITORY / "drone_city_nav/models" / model / "model.sdf").read_text()
            for mask in re.findall(r"<visibility_mask>(\d+)</visibility_mask>", text):
                self.assertEqual(0, int(mask) & replay.SMOKE_VISIBILITY_FLAGS, model)
        self.assertIn("<transparency>0.61</transparency>", sdf)  # 1/m: 0.39 opaque
        self.assertIn("<emissive>0.22 0.22 0.23 1</emissive>", sdf)  # 1/m: a glow of 0.1 + 0.3 x 0.39

    def test_the_world_is_replayed_through_the_true_poses_and_the_light(self) -> None:
        import render_world_replay as replay

        poses = [(10.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                 (11.0, 2.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0)]
        times = [pose[0] for pose in poses]
        self.assertEqual(replay.pose_at(poses, times, 10.25), (0.5, 1.0, 1.5, 1.0, 0.0, 0.0, 0.0))
        self.assertEqual(replay.pose_at(poses, times, 10.75)[3:], (0.0, 0.0, 0.0, 1.0))
        shares = [(20.0, 1.0), (50.0, 0.4), (51.0, 1.0)]
        self.assertEqual(replay.share_at(shares, 10.0), 1.0)
        self.assertEqual(replay.share_at(shares, 50.5), 0.4)
        self.assertEqual(replay.share_at(shares, 60.0), 1.0)


class RecordingToolsContractTest(unittest.TestCase):
    def test_every_recording_tool_is_tracked_and_documented(self) -> None:
        readme = (REPOSITORY / "tools/README.md").read_text()
        for tool in ("record_flight.sh", "record_batch.sh", "record_index.py",
                     "record_until_pass.sh", "video_check.py", "window_rig.sh",
                     "render_flight.sh"):
            self.assertTrue((REPOSITORY / "tools" / tool).is_file(), tool)
            self.assertIn(f"`{tool}`", readme)

    def test_the_video_check_judges_by_the_recorder_s_floors(self) -> None:
        check = (REPOSITORY / "tools/video_check.py").read_text()
        for name in ("MINIMUM_DISTINCT_FRAMES_PER_SECOND", "MINIMUM_DURATION_S"):
            self.assertIn(f"recorder.{name}", check)


if __name__ == "__main__":
    unittest.main()
