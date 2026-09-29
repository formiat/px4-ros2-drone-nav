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
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || printf '%s' "$$SIM_GUI_WORLD_SDF_PATH")" \
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
		SIM_WORLD_SDF_PATH="$$SIM_GUI_WORLD_SDF_PATH" \
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

.PHONY: sim-urban-point-to-point-headless
sim-urban-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${CAMERA_PROFILE:-stereo_tof}" = none ] && printf '%s' "$$SIM_SENSOR_WORLD_SDF_PATH" || { [ "$${WORLD_ILLUMINATION:-lit}" = dark ] && printf '%s' "$$SIM_DARK_WORLD_SDF_PATH" || printf '%s' "$$SIM_GUI_WORLD_SDF_PATH"; })" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
		OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="$${OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M:-4,20,9,16,32,18}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-600}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-point-to-point-gui
sim-urban-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$([ "$${WORLD_ILLUMINATION:-lit}" = dark ] && printf '%s' "$$SIM_DARK_WORLD_SDF_PATH" || printf '%s' "$$SIM_GUI_WORLD_SDF_PATH")" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-4}" \
		./scripts/run_drone_nav_sim.sh

# Named scenarios over the point-to-point mission: each sets what makes it
# that scenario and runs the mission's target, so that every flight the
# repository offers is one command (docs/scenarios.md).
URBAN_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/urban_practice_01_r050.occupancy3d
RETURN_HOME_GOAL_XYZ_M := 200,100,10
LIDAR_SCENARIO := CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar
# Roadmap item 19: the goal behind the location's outer walls, recorded in the
# manifest as injected, and the truth grid the check floods to confirm it.
RETURN_HOME_SCENARIO := MISSION_GOALS_XYZ_M=$(RETURN_HOME_GOAL_XYZ_M) \
	MISSION_GOAL_UNREACHABLE=true \
	TRUTH_OCCUPANCY_3D_PATH=$(URBAN_TRUTH_OCCUPANCY_3D) \
	SMOKE_DURATION_S="$${SMOKE_DURATION_S:-1800}"
# Roadmap item 17 stage 5: the carried light failing until the vehicle judges
# it unreliable, and a battery too low at launch to reach B; each ends at the
# start.
LIGHT_FAILURE_SCENARIO := WORLD_ILLUMINATION=dark LIGHT_FAULTS=severe \
	RETURN_HOME_EXPECTED=true
LOW_BATTERY_SCENARIO := WORLD_ILLUMINATION=dark LIGHT_BATTERY_S=240 \
	RETURN_HOME_EXPECTED=true
# Roadmap item 17 stage 7: a zone that fails the light, over B itself, the one
# place a zone closes the way in this location (every zone elsewhere on the
# route leaves a way around); B in its dark is unreachable and the vehicle
# ends at the start.
ZONE_ACROSS_B_SCENARIO := WORLD_ILLUMINATION=dark \
	ANOMALY_ZONES=63.009,23.857,12.593,1.5,8 RETURN_HOME_EXPECTED=true

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

# Roadmap item 17: the location with no light of its own, the vehicle's
# carried light the only one.
.PHONY: sim-urban-point-to-point-dark-headless
sim-urban-point-to-point-dark-headless:
	WORLD_ILLUMINATION=dark $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-point-to-point-dark-gui
sim-urban-point-to-point-dark-gui:
	WORLD_ILLUMINATION=dark $(MAKE) --no-print-directory sim-urban-point-to-point-gui

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

.PHONY: sim-urban-zone-across-b-headless
sim-urban-zone-across-b-headless:
	$(ZONE_ACROSS_B_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-headless

.PHONY: sim-urban-zone-across-b-gui
sim-urban-zone-across-b-gui:
	$(ZONE_ACROSS_B_SCENARIO) $(MAKE) --no-print-directory sim-urban-point-to-point-gui
