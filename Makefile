SHELL := /usr/bin/env bash

COLCON_BUILD_BASE ?= build
COLCON_INSTALL_BASE ?= install
COLCON_LOG_BASE ?= log

.PHONY: build
build:
	colcon --log-base $(COLCON_LOG_BASE) build --packages-select drone_city_nav --symlink-install --build-base $(COLCON_BUILD_BASE) --install-base $(COLCON_INSTALL_BASE) --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

.PHONY: mppi-benchmark-build
mppi-benchmark-build:
	colcon --log-base $(COLCON_LOG_BASE) build --packages-select drone_city_nav --symlink-install --build-base $(COLCON_BUILD_BASE) --install-base $(COLCON_INSTALL_BASE) --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DDRONE_CITY_NAV_ENABLE_CUDA_MPPI=ON

.PHONY: mppi-benchmark
mppi-benchmark: mppi-benchmark-build
	./$(COLCON_BUILD_BASE)/drone_city_nav/mppi_cuda_benchmark $(MPPI_BENCHMARK_ARGS)

.PHONY: test
test: build
	ctest --test-dir $(COLCON_BUILD_BASE)/drone_city_nav --output-on-failure

.PHONY: test-scripts
test-scripts:
	python3 -m unittest discover scripts/tests

.PHONY: quality
quality:
	./scripts/check_cpp_quality.sh

.PHONY: format-check
format-check:
	./scripts/check_cpp_quality.sh --format --no-build --no-test

.PHONY: format
format:
	./scripts/format_cpp_changed.sh

.PHONY: sim-environment-demo
sim-environment-demo:
	./scripts/run_environment_demo.sh "$${ENVIRONMENT_DEMO_ID:-urban_circuit_practice_01}"

.PHONY: sim-cooperative-traffic-urban-headless
sim-cooperative-traffic-urban-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/cooperative_traffic_urban_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		MISSION_TYPE=cooperative_traffic \
		MULTI_VEHICLE_SCENARIO_PATH=drone_city_nav/config/cooperative_traffic_urban_scenario.json \
		MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID=civilian_0 \
		MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY=next_living \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
		OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="$${OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M:-4,20,9,16,32,18}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		COOPERATIVE_MISSION_TIMEOUT_S="$${COOPERATIVE_MISSION_TIMEOUT_S:-480}" \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-600}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cooperative-traffic-urban-gui
sim-cooperative-traffic-urban-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/cooperative_traffic_urban_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		MISSION_TYPE=cooperative_traffic \
		MULTI_VEHICLE_SCENARIO_PATH=drone_city_nav/config/cooperative_traffic_urban_scenario.json \
		MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID=civilian_0 \
		MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY=next_living \
		COOPERATIVE_MISSION_TIMEOUT_S="$${COOPERATIVE_MISSION_TIMEOUT_S:-480}" \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# The ordinary flight after roadmap item 17 (specification A10): the location
# with no light of its own and the carried light steady (the owner's decision
# of 2026-11-05, F1: the moderate flicker flies only where a scenario names
# it); a named scenario sets the failures.
ORDINARY_FLIGHT := LIGHT_FAULTS="$${LIGHT_FAULTS-none}"

.PHONY: sim-urban-point-to-point-headless
sim-urban-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
		OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="$${OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M:--33,2.5,0,-7,42.5,18}" \
		RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:--33,2.5,0,-7,42.5,18}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}" \
		./scripts/run_drone_nav_sim.sh

# Roadmap item 11: the point-to-point flight inside the urban survey's volume (the goal at
# the west corridor's low end), by a survey map when STATIC_OCCUPANCY_3D_PATH names one (a
# survey map artifact's occupancy.occupancy3d, tools/survey_map.py) and without it
# otherwise. The map is the route's hint; the obstacle memory and the sensors remain the
# law (I2), so the memory stays on.
IN_VOLUME_SCENARIO_PATH := drone_city_nav/config/urban_circuit_practice_01_point_to_point_in_volume_scenario.json
.PHONY: sim-urban-point-to-point-in-volume-headless
sim-urban-point-to-point-in-volume-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(IN_VOLUME_SCENARIO_PATH)
	survey_map="$${STATIC_OCCUPANCY_3D_PATH:-}"; \
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(IN_VOLUME_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP="$$([ -n "$$survey_map" ] && echo true || echo false)" \
		STATIC_OCCUPANCY_3D_PATH="$$survey_map" \
		ENABLE_OBSTACLE_MEMORY=true LIDAR_PROFILE=3d \
		RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:-$(SURVEY_VOLUME_M)}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-point-to-point-gui
sim-urban-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# Roadmap item 34: the survey mission, no goal (specification F20): the
# frontiers of the memory inside the volume the scenario orders, each looked
# at from a vantage, home by the own track when none is left; the snapshot of
# the memory is the volume, which the check's coverage reads against the
# truth grid within the sensor's reach of the flown path.
SURVEY_SCENARIO_PATH := drone_city_nav/config/urban_circuit_practice_01_survey_scenario.json
SURVEY_VOLUME_M := -32,0,0,20,46,16

.PHONY: sim-urban-survey-headless
sim-urban-survey-headless: build urban-truth-occupancy-check
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		TRUTH_OCCUPANCY_3D_PATH=$(URBAN_TRUTH_OCCUPANCY_3D) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:-$(SURVEY_VOLUME_M)}" \
		SURVEY_COVERAGE_MINIMUM="$${SURVEY_COVERAGE_MINIMUM:-0.0}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-1800}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-survey-gui
sim-urban-survey-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-survey-lidar-headless sim-urban-survey-lidar-gui
sim-urban-survey-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-survey-headless

sim-urban-survey-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-survey-gui

# The survey on the held-out finals world (roadmap items 33 and 34), nothing
# adapted: 30 x 60 x 12 m round the point-to-point scenario's start, the long
# side along the route's first leg (the start inside the world since
# 2026-11-05; the volume ordered before it lay in the void round the shells).
FINALS_SURVEY_SCENARIO_PATH := drone_city_nav/config/finals_prize_round_world_07_survey_scenario.json
FINALS_SURVEY_VOLUME_M := 42,225,25,72,285,37
# The cave's survey (the goal of 2026-11-05, part 2): the point-to-point start, a
# volume of 30 x 60 x 12 m round it, its long side along the route's first leg.
CAVE_SURVEY_SCENARIO_PATH := drone_city_nav/config/cave_circuit_practice_01_survey_scenario.json
CAVE_SURVEY_VOLUME_M := -29.25,20.25,-0.75,0.75,80.25,11.25

.PHONY: sim-finals-survey-headless
sim-finals-survey-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment finals_prize_round_world_07 --runtime-map-mode no-static \
		--scenario $(FINALS_SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/finals_prize_round_world_07/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(FINALS_SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		TRUTH_OCCUPANCY_3D_PATH=$(FINALS_TRUTH_OCCUPANCY_3D) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:-$(FINALS_SURVEY_VOLUME_M)}" \
		SURVEY_COVERAGE_MINIMUM="$${SURVEY_COVERAGE_MINIMUM:-0.0}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-1800}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-finals-survey-gui
sim-finals-survey-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment finals_prize_round_world_07 --runtime-map-mode no-static \
		--scenario $(FINALS_SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/finals_prize_round_world_07/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(FINALS_SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-finals-survey-lidar-headless sim-finals-survey-lidar-gui
sim-finals-survey-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-finals-survey-headless

sim-finals-survey-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-finals-survey-gui

.PHONY: sim-cave-survey-headless
sim-cave-survey-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment cave_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(CAVE_SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/cave_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(CAVE_SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		TRUTH_OCCUPANCY_3D_PATH=$(CAVE_TRUTH_OCCUPANCY_3D) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		RAW_SNAPSHOT_BOUNDS_M="$${RAW_SNAPSHOT_BOUNDS_M:-$(CAVE_SURVEY_VOLUME_M)}" \
		SURVEY_COVERAGE_MINIMUM="$${SURVEY_COVERAGE_MINIMUM:-0.0}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-1800}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cave-survey-gui
sim-cave-survey-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment cave_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(CAVE_SURVEY_SCENARIO_PATH)
	. external/environment-artifacts/derived/cave_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(CAVE_SURVEY_SCENARIO_PATH) \
		MISSION_TYPE=survey \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cave-survey-lidar-headless sim-cave-survey-lidar-gui
sim-cave-survey-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-cave-survey-headless

sim-cave-survey-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-cave-survey-gui

# Roadmap item 29: the large dark hall (environments/fixtures/dark_hall), the
# point-to-point mission across it as the ordinary flight flies Urban's.
DARK_HALL_SCENARIO_PATH := drone_city_nav/config/dark_hall_point_to_point_scenario.json

.PHONY: sim-dark-hall-headless
sim-dark-hall-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment dark_hall_fixture --runtime-map-mode no-static \
		--scenario $(DARK_HALL_SCENARIO_PATH)
	. external/environment-artifacts/derived/dark_hall_fixture/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(DARK_HALL_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-600}" \
		./scripts/run_drone_nav_sim.sh

# Roadmap item 31: the dark void, the hall crossed at 14 m after a climb over
# the pad, where the carried light reaches neither the floor nor the ceiling
# (specification K46). The
# scenario file names the goal: by the east wall (within the light's reach at
# the end) unless DARK_VOID_SCENARIO_PATH names the hall's middle
# (dark_void_mid_point_to_point_scenario.json, an arrival on dead reckoning).
DARK_VOID_SCENARIO_PATH ?= drone_city_nav/config/dark_void_point_to_point_scenario.json

.PHONY: sim-dark-void-headless sim-dark-void-gui
sim-dark-void-headless: DARK_HALL_SCENARIO_PATH := $(DARK_VOID_SCENARIO_PATH)
sim-dark-void-headless: sim-dark-hall-headless
sim-dark-void-gui: DARK_HALL_SCENARIO_PATH := $(DARK_VOID_SCENARIO_PATH)
sim-dark-void-gui: sim-dark-hall-gui

.PHONY: sim-dark-hall-gui
sim-dark-hall-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment dark_hall_fixture --runtime-map-mode no-static \
		--scenario $(DARK_HALL_SCENARIO_PATH)
	. external/environment-artifacts/derived/dark_hall_fixture/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(DARK_HALL_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# Roadmap item 33: the held-out worlds, flown by the point-to-point mission with
# nothing adapted to them; the scenarios are drawn from each world's truth grid
# by tools/heldout_scenario.py. The urban targets' route volume and snapshot
# bounds are the urban location's and are not set here; the world's truth grid
# is named for the checks that read one.
CAVE_SCENARIO_PATH := drone_city_nav/config/cave_circuit_practice_01_point_to_point_scenario.json
CAVE_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/cave_practice_01_r050.occupancy3d
FINALS_SCENARIO_PATH := drone_city_nav/config/finals_prize_round_world_07_point_to_point_scenario.json
FINALS_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/final_event_07_r050.occupancy3d

.PHONY: sim-cave-point-to-point-headless
sim-cave-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment cave_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(CAVE_SCENARIO_PATH)
	. external/environment-artifacts/derived/cave_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(CAVE_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		TRUTH_OCCUPANCY_3D_PATH="$${TRUTH_OCCUPANCY_3D_PATH:-$(CAVE_TRUTH_OCCUPANCY_3D)}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cave-point-to-point-gui
sim-cave-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment cave_circuit_practice_01 --runtime-map-mode no-static \
		--scenario $(CAVE_SCENARIO_PATH)
	. external/environment-artifacts/derived/cave_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(CAVE_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-finals-point-to-point-headless
sim-finals-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment finals_prize_round_world_07 --runtime-map-mode no-static \
		--scenario $(FINALS_SCENARIO_PATH)
	. external/environment-artifacts/derived/finals_prize_round_world_07/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_DARK_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(FINALS_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		TRUTH_OCCUPANCY_3D_PATH="$${TRUTH_OCCUPANCY_3D_PATH:-$(FINALS_TRUTH_OCCUPANCY_3D)}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-finals-point-to-point-gui
sim-finals-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment finals_prize_round_world_07 --runtime-map-mode no-static \
		--scenario $(FINALS_SCENARIO_PATH)
	. external/environment-artifacts/derived/finals_prize_round_world_07/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_DARK_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(FINALS_SCENARIO_PATH) \
		$(ORDINARY_FLIGHT) \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# Roadmap item 39 (stage 0): the open-sky world, the autopilot's baylands park
# as a fixture of the environment pipeline, flown by the point-to-point mission
# low over the ground through a sequence of goals drawn by
# tools/open_sky_scenario.py; the cameras fly the world under its sun with its
# textures (the gui world, never the dark one: the sensor world carries no
# texture and is the lidar's), the carried light's flicker off (A10 is the dark
# location's: under the sun the lamp's outage read 3.6 s and K12 sent r2614
# home), the mission window the kilometres' own. Nothing of the urban
# location is set.
OPEN_SKY_SCENARIO_PATH := drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json
OPEN_SKY_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/open_sky_baylands_r050.occupancy3d

.PHONY: sim-open-sky-point-to-point-headless
sim-open-sky-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment open_sky_baylands --runtime-map-mode no-static \
		--scenario $(OPEN_SKY_SCENARIO_PATH)
	. external/environment-artifacts/derived/open_sky_baylands/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_GUI_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=$(OPEN_SKY_SCENARIO_PATH) \
		LIGHT_FAULTS="$${LIGHT_FAULTS-none}" \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		TRUTH_OCCUPANCY_3D_PATH="$${TRUTH_OCCUPANCY_3D_PATH:-$(OPEN_SKY_TRUTH_OCCUPANCY_3D)}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-2400}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-open-sky-point-to-point-gui
sim-open-sky-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment open_sky_baylands --runtime-map-mode no-static \
		--scenario $(OPEN_SKY_SCENARIO_PATH)
	. external/environment-artifacts/derived/open_sky_baylands/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_GUI_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=$(OPEN_SKY_SCENARIO_PATH) \
		LIGHT_FAULTS="$${LIGHT_FAULTS-none}" \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# Named scenarios over the point-to-point mission: each sets what makes it
# that scenario and runs the mission's target, so that every flight the
# repository offers is one command (docs/scenarios.md).
URBAN_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/urban_practice_01_r050.occupancy3d
RETURN_HOME_GOAL_XYZ_M := 30,60,10
LIDAR_SCENARIO := CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar
# Roadmap item 19: the goal behind the location's north wall, 49 m from the
# start, recorded in the manifest as injected, and the truth grid the check
# floods to confirm it. On the stereo set the return is the light's
# battery's: 720 s of light give B up some five minutes in (the owner's rule
# of 2026-09-30, specification A8), not after an hour of exploration.
RETURN_HOME_SCENARIO := MISSION_GOALS_XYZ_M=$(RETURN_HOME_GOAL_XYZ_M) \
	MISSION_GOAL_UNREACHABLE=true LIGHT_BATTERY_S=720 \
	TRUTH_OCCUPANCY_3D_PATH=$(URBAN_TRUTH_OCCUPANCY_3D) \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
# Roadmap item 17 stage 5: the carried light failing until the vehicle judges
# it unreliable, and a battery too low at launch to reach B; each ends at the
# start.
LIGHT_FAILURE_SCENARIO := LIGHT_FAULTS=severe \
	RETURN_HOME_EXPECTED=true
LOW_BATTERY_SCENARIO := LIGHT_BATTERY_S=240 \
	RETURN_HOME_EXPECTED=true
# Roadmap item 17 stage 5: a long flight, B and back, under the moderate
# flicker and the camera stream's failures, which ends at the start whether
# it reaches B or gives it up for an unreliable light; and a short one under
# the harshest failures the owner's range allows, the light all but gone
# from the forty-fifth second, which asks only that the vehicle stay whole.
LONG_FAILURES_SCENARIO := LIGHT_FAULTS=moderate STREAM_FAULTS=moderate RETURN_HOME_ALLOWED=true \
	MISSION_GOALS_XYZ_M="63.009,23.857,12.593;0.749,21.247,9.6"
# Roadmap item 18: smoke in the location (specification A13, F4). The ordinary
# smoke flight carries four plumes beside the route where the location is
# wide (the start hall at (8,27) and (12,22), the tunnel's mouth at (18,-10),
# the east hall at (68,-20); 1/m, one on the route itself to fly round), none
# closing the way; the blocked flight closes the north corridor at (4,43),
# 4.4 m wide, with one plume of 3/m: item 24's proof gives the goal up where
# the plume is in the memory (the lidar returns from it outside, K37; the
# stereo set's smoke face, K44). The vehicle has no rung, hold or return of
# its own for smoke (the owner, 2026-10-18: smoke is what makes space not
# free, nothing more; K40 withdrawn), and never enters dense smoke in truth
# (R7, the mission check). The stereo set cannot prove the way closed: its
# lamp reaches 5 m and the proof's flood runs through the halls it has not
# seen (r1753: budget exhausted every 27 s for 500 s), so, as the
# goal-outside flight does (A8, the owner's rule of 2026-09-30), the blocked
# flight launches with 720 s of light and the stereo set comes home on its
# charge (K11) after searching for a way; the lidar needs no light and
# comes home by the proof.
SMOKE_SCENARIO := SMOKE_ZONES="8,27,10.5,3,3,3,1.0;12,22,11,2,2,2,1.0;18,-10,4,3,3,3,1.0;68,-20,6.5,3,3,3,1.0" \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
SMOKE_BLOCKED_SCENARIO := SMOKE_ZONES="4,43,10,3,6,6,3.0" \
	RETURN_HOME_EXPECTED=true RETURN_TRIGGERS_ALLOWED=topological,battery \
	LIGHT_BATTERY_S=720 \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
# Roadmap item 23: moderate smoke wall to wall in whole rooms, the rest of the
# location clear, one room always on the way: the west hall, which the
# circuit's way to B crosses, and the tunnel's mouth at (18,-6), which the
# way passes by; at 0.1/m (a visibility of about 30 m; the pair matches a
# wall 4 m off through it whole and 8 m off not at all, the lidar reads the
# walls through it as its last return, A13). Two rooms on the way (the east
# hall as well) carried the stereo set's estimator 6 to 7 m off by B
# (r1658, r1664): 77 features tracked there against 158 in the clear. The
# west hall's box is the evaluation's route volume and its memory snapshot
# too, so that the memory of a smoky room is read against the truth
# (tools/room_memory_truth.py).
SMOKE_ROOMS_SCENARIO := SMOKE_ZONES="-20,22.5,9,26,40,18,0.1;18,-6,5,12,10,10,0.1" \
	OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="-33,2.5,0,-7,42.5,18" \
	RAW_SNAPSHOT_BOUNDS_M="-33,2.5,0,-7,42.5,18" \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
# The owner's rule of 2026-10-24 (specification F4): the ordinary flight, and
# so every acceptance flight, is the clear location, no smoke anywhere, and
# no smoke flight is flown at all. From 2026-10-17 to 2026-10-24 the ordinary
# flight carried the location's smoke (the moderate rooms of item 23 and the
# dense plumes of item 18, F4 as then written); the smoke targets below stay
# as the record of items 18, 23 and 24 and of the acceptance of 2026-10-24
# (r2049 to r2065), unflown. The ordinary flight's route volume and its
# memory snapshot stay the west hall's (the validator asks the two to agree,
# and the route crosses the hall either way). sim-urban-clear-* is the
# ordinary flight under its old reference name.
CLEAR_SCENARIO := SMOKE_ZONES=""
# Roadmap item 32: hot smoke, flown only for its acceptance (the owner,
# 2026-10-24: the ordinary flight is clear, and hot smoke joins no ordinary
# scenario). The hot plume: the location's plume at (12,22,11), which the
# ordinary route passes within a metre (r2191), 3 x 3 x 3 at 1/m with a heat
# source at its centre (0.3 m, 500 K); the smoke face (K44) is on, the plume
# being dense. The hot room: the moderate rooms of item 23 with a source on
# the route's crossing of the west hall (its middle in r2191, (-26.6,24.5,5));
# the way round it or the way home are both the rule's (K49). The cold room,
# the control, is sim-urban-smoke-rooms-* as it is.
HOT_PLUME_SCENARIO := SMOKE_ZONES="12,22,11,3,3,3,1.0" HEAT_SOURCES="12,22,11,0.3,500" \
	SMOKE_FACE=true SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
HOT_ROOM_SCENARIO := SMOKE_ZONES="-20,22.5,9,26,40,18,0.1;18,-6,5,12,10,10,0.1" \
	HEAT_SOURCES="-26.6,24.5,5,0.3,500" \
	RETURN_HOME_ALLOWED=true RETURN_TRIGGERS_ALLOWED=topological,battery \
	LIGHT_BATTERY_S=720 \
	OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="-33,2.5,0,-7,42.5,18" \
	RAW_SNAPSHOT_BOUNDS_M="-33,2.5,0,-7,42.5,18" \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-900}"
LIGHT_LOST_SCENARIO := LIGHT_FAULTS=lost STREAM_FAULTS=moderate \
	VEHICLE_INTACT_EXPECTED=true

.PHONY: urban-truth-occupancy-check
urban-truth-occupancy-check:
	@test -f $(URBAN_TRUTH_OCCUPANCY_3D) || \
		(printf '%s\n' 'Missing $(URBAN_TRUTH_OCCUPANCY_3D): voxelize the location as docs/environment_candidates.md describes.' >&2; exit 2)

.PHONY: sim-urban-point-to-point-lidar-headless
sim-urban-point-to-point-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-point-to-point-lidar-gui
sim-urban-point-to-point-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-cave-point-to-point-lidar-headless sim-cave-point-to-point-lidar-gui
sim-cave-point-to-point-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-cave-point-to-point-headless

sim-cave-point-to-point-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-cave-point-to-point-gui

.PHONY: sim-finals-point-to-point-lidar-headless sim-finals-point-to-point-lidar-gui
sim-finals-point-to-point-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-finals-point-to-point-headless

sim-finals-point-to-point-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-finals-point-to-point-gui

.PHONY: sim-open-sky-point-to-point-lidar-headless sim-open-sky-point-to-point-lidar-gui
sim-open-sky-point-to-point-lidar-headless:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-open-sky-point-to-point-headless

sim-open-sky-point-to-point-lidar-gui:
	$(LIDAR_SCENARIO) $(MAKE) --no-print-directory sim-open-sky-point-to-point-gui

.PHONY: sim-urban-point-to-point-gnss-headless
sim-urban-point-to-point-gnss-headless:
	LOCALIZATION_PROFILE=gnss $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-point-to-point-gnss-gui
sim-urban-point-to-point-gnss-gui:
	LOCALIZATION_PROFILE=gnss $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-return-home-headless
sim-urban-return-home-headless: urban-truth-occupancy-check
	$(RETURN_HOME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-return-home-gui
sim-urban-return-home-gui:
	$(RETURN_HOME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-return-home-lidar-headless
sim-urban-return-home-lidar-headless: urban-truth-occupancy-check
	$(LIDAR_SCENARIO) $(RETURN_HOME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-return-home-lidar-gui
sim-urban-return-home-lidar-gui:
	$(LIDAR_SCENARIO) $(RETURN_HOME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-light-failure-headless
sim-urban-light-failure-headless:
	$(LIGHT_FAILURE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-light-failure-gui
sim-urban-light-failure-gui:
	$(LIGHT_FAILURE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-low-battery-headless
sim-urban-low-battery-headless:
	$(LOW_BATTERY_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-low-battery-gui
sim-urban-low-battery-gui:
	$(LOW_BATTERY_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-long-failures-headless
sim-urban-long-failures-headless:
	$(LONG_FAILURES_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-long-failures-gui
sim-urban-long-failures-gui:
	$(LONG_FAILURES_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-light-lost-headless
sim-urban-light-lost-headless:
	$(LIGHT_LOST_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-light-lost-gui
sim-urban-light-lost-gui:
	$(LIGHT_LOST_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-clear-headless
sim-urban-clear-headless:
	$(CLEAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-clear-gui
sim-urban-clear-gui:
	$(CLEAR_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-smoke-headless
sim-urban-smoke-headless:
	$(SMOKE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-smoke-gui
sim-urban-smoke-gui:
	$(SMOKE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-smoke-blocked-headless
sim-urban-smoke-blocked-headless:
	$(SMOKE_BLOCKED_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-smoke-blocked-gui
sim-urban-smoke-blocked-gui:
	$(SMOKE_BLOCKED_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-smoke-rooms-headless
sim-urban-smoke-rooms-headless:
	$(SMOKE_ROOMS_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

# Roadmap item 21: a warm body of a person's size walking back and forth where every flight passes, the long
# corridor at y -16 (its floor at 1.0 m, its walls at y -18.5 and -13.5; MOVING_BODIES, scripts/moving_bodies.py):
# across the route in the vehicle's view; along the corridor, towards the vehicle and away; out of the south room
# into the corridor's mouth, from the vehicle's side. A scenario of its own: the ordinary flight carries no body.
WALKING_BODY_SCENARIO := MOVING_BODIES="40,-18,40,-14,1.0,1.2"
WALKING_BODY_ONCOMING_SCENARIO := MOVING_BODIES="43,-16,33,-16,1.0,1.2"
WALKING_BODY_SIDE_SCENARIO := MOVING_BODIES="29.5,-23.5,29.5,-15,1.0,1.2"

.PHONY: sim-urban-walking-body-headless
sim-urban-walking-body-headless:
	$(WALKING_BODY_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-walking-body-gui
sim-urban-walking-body-gui:
	$(WALKING_BODY_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-walking-body-oncoming-headless
sim-urban-walking-body-oncoming-headless:
	$(WALKING_BODY_ONCOMING_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-walking-body-oncoming-gui
sim-urban-walking-body-oncoming-gui:
	$(WALKING_BODY_ONCOMING_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-walking-body-side-headless
sim-urban-walking-body-side-headless:
	$(WALKING_BODY_SIDE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-walking-body-side-gui
sim-urban-walking-body-side-gui:
	$(WALKING_BODY_SIDE_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-hot-smoke-plume-headless
sim-urban-hot-smoke-plume-headless:
	$(HOT_PLUME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-hot-smoke-plume-gui
sim-urban-hot-smoke-plume-gui:
	$(HOT_PLUME_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-hot-smoke-room-headless
sim-urban-hot-smoke-room-headless:
	$(HOT_ROOM_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-hot-smoke-room-gui
sim-urban-hot-smoke-room-gui:
	$(HOT_ROOM_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui

.PHONY: sim-urban-smoke-rooms-gui
sim-urban-smoke-rooms-gui:
	$(SMOKE_ROOMS_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui
