#!/usr/bin/env python3
"""Roadmap item 39, stage 0: the open-sky world (the autopilot's baylands park as
a fixture of the environment pipeline) flown by the point-to-point mission low
over the ground through a sequence of goals drawn by tools/open_sky_scenario.py."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import runpy
import math
import unittest
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"
SCENARIO_PATH = REPOSITORY / "drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json"
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/open_sky_baylands.world3d.json"
FIXTURE = REPOSITORY / "environments/fixtures/open_sky_baylands"
URBAN_CONSTANTS = ("REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING", "OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M",
                   "RAW_SNAPSHOT_BOUNDS_M", "SMOKE_ZONES", "URBAN_TRUTH_OCCUPANCY_3D", "SIM_DARK_WORLD_SDF_PATH")

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class OpenSkyScenarioTest(unittest.TestCase):
    def test_the_mission_is_a_sequence_of_goals_low_over_the_ground(self) -> None:
        scenario = SCENARIO_MODULE.load_point_to_point_scenario(SCENARIO_PATH)
        self.assertEqual(scenario["gazebo_world_name"], "open_sky_baylands_collisions")
        self.assertEqual(scenario["canonical_world_path"], WORLD_PATH.resolve())
        transform = scenario["map_to_sdf"]
        for axis, key in enumerate(("sdf_x_offset_m", "sdf_y_offset_m", "sdf_z_offset_m")):
            self.assertAlmostEqual(scenario["gazebo_spawn_m"][axis], scenario["map_start_m"][axis] + transform[key])
        selection = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))["selection"]
        self.assertEqual(selection["tool"], "tools/open_sky_scenario.py")
        goals = scenario["mission_goal_sequence_m"]
        self.assertEqual(len(goals), 3)
        self.assertEqual(len(selection["legs_m"]), 3)
        for leg in selection["legs_m"]:
            self.assertGreaterEqual(leg, 500.0)
            self.assertLessEqual(leg, 700.0)
        self.assertGreaterEqual(selection["route_length_m"], 1500.0)
        self.assertLessEqual(selection["route_length_m"], 2100.0)
        self.assertEqual(selection["goal_over_ground_m"], 6.0)
        self.assertEqual(selection["flight_over_ground_m"], [4.0, 8.0])
        (pad,) = scenario["launch_platforms"]
        self.assertAlmostEqual(scenario["map_start_m"][2], pad["top_z_m"] + 0.23)
        points = [scenario["map_start_m"], *goals]
        for straight, (a, b) in zip(selection["straight_distances_m"], zip(points, points[1:])):
            self.assertAlmostEqual(math.dist(a, b), straight, delta=0.1)

    def test_no_world_states_a_kind_of_space_and_the_launch_applies_no_profile(self) -> None:
        # Roadmap item 45 (the owner, 2026-11-17: the vehicle is one, and nobody switches a mode of the
        # environment). The world stated the kind of space it was (flight_profile) and the launch applied a
        # profile by it (roadmap item 39 stage 5, specification K75): every number it chose is now one for every
        # world, the sensor set's, or chosen in flight by what the stack measures (K76, K88).
        world = json.loads(WORLD_PATH.read_text(encoding="utf-8"))
        self.assertNotIn("flight_profile", world)
        launch = REPOSITORY / "drone_city_nav/launch"
        scenario_module = runpy.run_path(str(launch / "point_to_point_scenario.py"))
        load = scenario_module["load_point_to_point_scenario"]
        self.assertNotIn("flight_profile", load(
            REPOSITORY / "drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json"))
        # Nothing the flight runs reads the field, and no target of the Makefile names a number of the flight by
        # a world: the open sky's targets set the world, the scenario and the window.
        for root, patterns in (("drone_city_nav", ("*.py", "*.json", "*.yaml", "*.cpp", "*.hpp")),
                               ("tools", ("*.py", "*.sh")), ("scripts", ("*.py", "*.sh"))):
            for pattern in patterns:
                for path in (REPOSITORY / root).rglob(pattern):
                    if "tests" in path.parts:
                        continue
                    self.assertNotIn("flight_profile", path.read_text(encoding="utf-8", errors="replace"), path)
        makefile_text = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        self.assertNotIn("flight_profile", makefile_text)
        for name in ("sim-open-sky-point-to-point-headless: build", "sim-open-sky-point-to-point-gui: build"):
            target = makefile_text[makefile_text.index(name):]
            target = target[:target.index("./scripts/run_drone_nav_sim.sh")]
            for number in ("_MPS", "_MPS2", "_MPS3", "RANGE_M", "MARGIN_M", "_HZ"):
                self.assertNotIn(number, target, name)
        # No dictionary of the launch is the open sky's any more (roadmap item 45): what the world's
        # flight_profile chose is one number for every world, the sensor set's, or chosen in flight by what the
        # stack measures.
        self.assertNotIn("OPEN_SKY_", (launch / "sensor_profile.py").read_text(encoding="utf-8"))
        self.assertNotIn("OPEN_SKY_", (launch / "city_nav.launch.py").read_text(encoding="utf-8"))
        self.assertNotIn("flight_profile", (launch / "city_nav.launch.py").read_text(encoding="utf-8"))
        # What the lidar set carries on every world (roadmap item 45): the scan's age and the window's margin.
        # A route is released for the leg over the seen only while the planner holds one.
        lidar = runpy.run_path(str(launch / "sensor_profile.py"))["LIDAR_OBSERVABILITY"]
        self.assertAlmostEqual(lidar["latest_sensor_obstacle_maximum_age_ms"], 300.0)
        self.assertAlmostEqual(lidar["no_static_3d_esdf_horizontal_recenter_margin_m"], 17.0)
        # The leg over the seen (K76) is first taken only where the memory has seen the way up from the
        # route's start free, so its margin is the lidar set's on every world.
        self.assertAlmostEqual(lidar["persistent_planner_overflight_margin_m"], 5.0)
        overflight = (REPOSITORY / "drone_city_nav/src/persistent_dstar_lite_planner_3d_overflight.cpp").read_text(
            encoding="utf-8")
        self.assertIn("if (first_taking && !skySeenOver(*world_.observed_occupancy, start, level_m,", overflight)
        # The lidar's promise under a sky its scan saw open (specification K75, K88) is the lidar set's on every
        # world: the planner takes it tick by tick from the frame, and under a roof the configuration's 14 m and
        # the voxel stand.
        self.assertAlmostEqual(lidar["open_sky_lidar_detection_range_m"], 35.0)
        self.assertAlmostEqual(lidar["guaranteed_obstacle_size_m"], 0.75)
        self.assertLessEqual(lidar["open_sky_lidar_detection_range_m"],
                             lidar["guaranteed_obstacle_size_m"] / math.tan(math.radians(1.0)))
        self.assertNotIn("guaranteed_lidar_detection_range_m", lidar)
        finalize = (REPOSITORY / "drone_city_nav/src/runtime/ros/production_mppi_node_planning_tick_finalize.cpp"
                    ).read_text(encoding="utf-8")
        self.assertIn("if (planner_holds_leg_over_seen_.load(std::memory_order_relaxed) &&", finalize)
        self.assertNotIn("overflight_margin_m > 0.0", finalize)
        # The open sky has no roof: its ceiling stands 16 m over the memory's configured box, and the launch
        # stands the box a metre over a world's ceiling. The tracing threads and the publication's rate are
        # the configuration's, one for every world.
        self.assertNotIn("OPEN_SKY_MEMORY", (launch / "sensor_profile.py").read_text(encoding="utf-8"))
        memory_yaml = (REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text(encoding="utf-8")
        for line in ("grid_depth_m: 40.0", "grid_origin_z: 0.0", "integration_threads: 4",
                     "obstacle_memory_3d_transport_rate_hz: 10.0"):
            self.assertEqual(memory_yaml.count(f"    {line}\n"), 1, line)
        box = 56.0
        self.assertIn('world_ceiling_m + 1.0 - float(memory_box["grid_origin_z"])',
                      (launch / "city_nav.launch.py").read_text(encoding="utf-8"))
        # The dynamics are the vehicle's, one for every world (specification V3): no profile names them.
        self.assertNotIn("OPEN_SKY_DYNAMICS", (launch / "sensor_profile.py").read_text(encoding="utf-8"))
        vehicle = (REPOSITORY / "drone_city_nav/config/urban_mvp.yaml").read_text(encoding="utf-8")
        for line in ("cruise_speed_mps: 15.0", "absolute_speed_limit_mps: 15.0",
                     "maximum_horizontal_acceleration_mps2: 5.0", "maximum_vertical_acceleration_mps2: 3.0",
                     "maximum_control_jerk_mps3: 18.0", "guaranteed_vertical_stopping_deceleration_mps2: 1.4",
                     "unavailable_path_braking_acceleration_mps2: 5.0"):
            self.assertEqual(vehicle.count(f"    {line}\n"), 1, line)
        self.assertEqual(vehicle.count("    guaranteed_lidar_detection_range_m: 14.0\n"), 1)
        # The stereo set plans with the profile it has flown, on every world; the environment's overrides win.
        self.assertEqual(runpy.run_path(str(launch / "sensor_profile.py"))["STEREO_TOF_PROFILE"],
                         {"cruise_speed_mps": 6.5, "absolute_speed_limit_mps": 10.0,
                          "maximum_horizontal_acceleration_mps2": 4.0, "maximum_vertical_acceleration_mps2": 1.4,
                          "maximum_control_jerk_mps3": 12.0})
        launch_text = (launch / "city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn("production_mppi_parameters.append(STEREO_TOF_PROFILE)", launch_text)
        self.assertLess(launch_text.index("production_mppi_parameters.append(STEREO_TOF_PROFILE)"),
                        launch_text.index('("cruise_speed_mps", cruise_speed_mps),'))
        self.assertLess(launch_text.index('offboard_overrides["unavailable_path_braking_acceleration_mps2"] = ('),
                        launch_text.index('("maximum_horizontal_acceleration_mps2", maximum_horizontal_acceleration_mps2),'))
        self.assertAlmostEqual(world["navigation"]["maximum_target_z_m"], box - 1.0)
        self.assertIn("box_size[2] + 16.0 - 1.0", (REPOSITORY / "tools/open_sky_scenario.py").read_text(encoding="utf-8"))
        text = (launch / "city_nav.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("OPEN_SKY_DYNAMICS", text)
        # Nothing consumed the pair's shadow memory beside a navigating lidar: it is gone on every world.
        self.assertNotIn("shadow", text.replace("gnss_shadow", "").replace("visual_inertial_shadow", ""))
        # The lidar's memory reads the sky on every scan and places a scan's returns by it (specification K88):
        # the cone and the open sky's obstacle are the lidar set's on every world, and no world names them.
        self.assertEqual(runpy.run_path(str(launch / "sensor_profile.py"))["LIDAR_MEMORY"],
                         {"sky_cone_half_angle_rad": 0.175, "guaranteed_obstacle_size_m": 0.75,
                          "latest_scan_clear_sight_half_angle_rad": 0.105})
        self.assertIn('production_mppi_parameters.append(LIDAR_OBSERVABILITY)\n'
                      '            obstacle_memory_parameters.append(LIDAR_MEMORY)', text)
        self.assertNotIn('"guaranteed_obstacle_size_m": OPEN_SKY_LIDAR_OBSERVABILITY[', text)
        clouds = (REPOSITORY / "drone_city_nav/src/obstacle_memory_3d_node_clouds.cpp").read_text(encoding="utf-8")
        self.assertIn("latest.sky_open = skyOpenAbove(memory_beams, sky_cone_half_angle_rad_);", clouds)
        self.assertIn(".maximum_range_m = latest.sky_open ? 0.0 : roofed_integration_range_m_,", clouds)
        # The tracing threads are for the scans traced to the sensor's range; one taken under a roof has one.
        memory_source = (REPOSITORY / "drone_city_nav/src/obstacle_memory_3d.cpp").read_text(encoding="utf-8")
        self.assertIn("scan.maximum_range_m > 0.0\n          ? 1U\n"
                      "          : static_cast<std::size_t>(config_.integration_threads);", memory_source)
        self.assertNotIn("OPEN_SKY_MEMORY", text)
        coordinator = (REPOSITORY / "drone_city_nav/src/runtime/planning_cycle_coordinator_3d.cpp").read_text(
            encoding="utf-8")
        self.assertIn("speed_policy_input.sky_open = promise.sky_open;", coordinator)
        # The rollouts' speed table is built at the widest promise; the reference is held to the tick's own.
        control = (REPOSITORY / "drone_city_nav/src/runtime/ros/production_mppi_config_ros_control.cpp").read_text(
            encoding="utf-8")
        self.assertIn("const double limit_mps = sensorBrakingMaximumSpeedMps(\n        widest_contract,", control)
        self.assertNotIn("config_.speed_policy.guaranteed_obstacle_size_m,", coordinator)
        self.assertNotIn("OPEN_SKY_TAKEOFF", text)
        self.assertEqual(vehicle.count("    takeoff_hover_s: 0.5\n"), 1)
        memory = (REPOSITORY / "drone_city_nav/src/obstacle_memory_3d_node.cpp").read_text(encoding="utf-8")
        self.assertIn('declare_parameter<double>("guaranteed_obstacle_size_m", 0.0)', memory)
        self.assertIn("2.0 * placement_unit_m / std::tan(heading_uncertainty_rad)", memory)
        # No target names a speed or an acceleration: the configuration is the one place, the environment the
        # override (scripts/run_drone_nav_sim.sh passes one only where it is set).
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        for name in ("CRUISE_SPEED_MPS", "ABSOLUTE_SPEED_LIMIT_MPS", "MAXIMUM_HORIZONTAL_ACCELERATION_MPS2"):
            self.assertNotIn(name, makefile)
        runner = (REPOSITORY / "scripts/run_drone_nav_sim.sh").read_text(encoding="utf-8")
        self.assertIn('cruise_speed_override="${CRUISE_SPEED_MPS:-}"', runner)

    def test_the_fixture_is_the_manifests_and_the_targets_set_nothing_of_urban(self) -> None:
        manifest = yaml.safe_load((REPOSITORY / "environments/environment_manifest.yaml").read_text(encoding="utf-8"))
        (environment,) = [e for e in manifest["environments"] if e["id"] == "open_sky_baylands"]
        self.assertEqual(environment["distribution"], "repository")
        for record in environment["repository_files"]:
            path = REPOSITORY / record["path"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), record["sha256"], record["path"])
            self.assertEqual(path.stat().st_size, record["size_bytes"])
        world = (FIXTURE / "world.sdf").read_text(encoding="utf-8")
        self.assertIn("<spherical_coordinates>", world)
        self.assertNotIn("relative_to=", world)
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        for mode in ("headless", "gui"):
            target = f"sim-open-sky-point-to-point-{mode}"
            self.assertIn(f"{target}: build", makefile)
            recipe = makefile[makefile.index(f"{target}: build"):]
            recipe = recipe[:recipe.index("\n\n")]
            self.assertIn("--environment open_sky_baylands --runtime-map-mode no-static", recipe)
            for constant in URBAN_CONSTANTS:
                self.assertNotIn(constant, recipe)
            # The sun lights the world: no flicker of the carried light (A10 is the dark location's).
            self.assertIn('LIGHT_FAULTS="$${LIGHT_FAULTS-none}"', recipe)
            self.assertNotIn("$(ORDINARY_FLIGHT)", recipe)
            wrapper = REPOSITORY / f"scripts/sim_open_sky_point_to_point_{mode}.sh"
            self.assertIn(f"make {target}\n", wrapper.read_text(encoding="utf-8"))
            lidar = REPOSITORY / f"scripts/sim_open_sky_point_to_point_lidar_{mode}.sh"
            self.assertIn(f"make sim-open-sky-point-to-point-lidar-{mode}\n", lidar.read_text(encoding="utf-8"))
            self.assertIn(f"$(LIDAR_SCENARIO) $(MAKE) --no-print-directory {target}\n", makefile)
        # The cameras fly the sky's world under its sun with its textures (the
        # gui world), the lidar the sensor world; never the dark one.
        headless = makefile[makefile.index("sim-open-sky-point-to-point-headless: build"):]
        headless = headless[:headless.index("\n\n")]
        self.assertIn("SIM_GUI_WORLD_SDF_PATH", headless)
        self.assertIn("SIM_SENSOR_WORLD_SDF_PATH", headless)
        self.assertNotIn("SIM_DARK_WORLD_SDF_PATH", headless)
        scenarios = (REPOSITORY / "docs/scenarios.md").read_text(encoding="utf-8")
        self.assertIn("sim_open_sky_point_to_point_headless.sh", scenarios)


if __name__ == "__main__":
    unittest.main()
