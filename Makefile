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
	@test -n "$${ENVIRONMENT_DEMO_ID:-}" || \
		(printf '%s\n' 'Set ENVIRONMENT_DEMO_ID, for example urban_circuit_practice_01.' >&2; exit 2)
	./scripts/run_environment_demo.sh "$${ENVIRONMENT_DEMO_ID}"

.PHONY: sim-gui
sim-gui: build
	@if [[ -z "$${MISSION_GOALS_XYZ_M+x}" && \
		-z "$${POINT_TO_POINT_SCENARIO_PATH:-}" ]]; then \
		export MISSION_GOALS_XYZ_M='216,378,18;216,54,18;54,378,18;54,54,18'; \
	fi; \
	./scripts/run_drone_nav_sim.sh

.PHONY: sim-headless
sim-headless: build
	@if [[ -z "$${MISSION_GOALS_XYZ_M+x}" && \
		-z "$${POINT_TO_POINT_SCENARIO_PATH:-}" ]]; then \
		export MISSION_GOALS_XYZ_M='216,378,18;216,54,18;54,378,18;54,54,18'; \
	fi; \
	HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S="$${SMOKE_DURATION_S:-300}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-intercept-gui
sim-intercept-gui: build
	MISSION_TYPE=intercept \
		INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID="$${INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-evader}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-intercept-headless
sim-intercept-headless: build
	MISSION_TYPE=intercept HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S="$${SMOKE_DURATION_S:-120}" ./scripts/run_drone_nav_sim.sh

.PHONY: sim-multi-intercept-gui
sim-multi-intercept-gui: build
	MISSION_TYPE=multi_intercept \
		INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED=false \
		INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID="$${INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-evader_0}" \
		INTERCEPT_SPECTATOR_RESELECTION_POLICY="$${INTERCEPT_SPECTATOR_RESELECTION_POLICY:-next_living}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-multi-intercept-headless
sim-multi-intercept-headless: build
	MISSION_TYPE=multi_intercept \
		INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED=false \
		INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID="$${INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-evader_0}" \
		INTERCEPT_SPECTATOR_RESELECTION_POLICY="$${INTERCEPT_SPECTATOR_RESELECTION_POLICY:-next_living}" \
		HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S="$${SMOKE_DURATION_S:-180}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cooperative-traffic-gui
sim-cooperative-traffic-gui: build
	MISSION_TYPE=cooperative_traffic \
		MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID="$${MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID:-civilian_0}" \
		MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY="$${MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY:-next_living}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cooperative-traffic-headless
sim-cooperative-traffic-headless: build
	MISSION_TYPE=cooperative_traffic \
		MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID="$${MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID:-civilian_0}" \
		MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY="$${MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY:-next_living}" \
		HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S="$${SMOKE_DURATION_S:-300}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-cooperative-traffic-urban-headless
sim-cooperative-traffic-urban-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/cooperative_traffic_urban_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_SENSOR_WORLD_SDF_PATH" \
		MISSION_TYPE=cooperative_traffic \
		MULTI_VEHICLE_SCENARIO_PATH=drone_city_nav/config/cooperative_traffic_urban_scenario.json \
		MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID=civilian_0 \
		MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY=next_living \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
		OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="$${OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M:-4,20,9,16,32,18}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-5}" \
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
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-5}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-point-to-point-headless
sim-urban-point-to-point-headless: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_SENSOR_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
		OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M="$${OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M:-4,20,9,16,32,18}" \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-5}" \
		HEADLESS=1 MISSION_CHECK=1 \
		SMOKE_DURATION_S="$${SMOKE_DURATION_S:-300}" \
		./scripts/run_drone_nav_sim.sh

.PHONY: sim-urban-point-to-point-gui
sim-urban-point-to-point-gui: build
	python3 scripts/prepare_environment_simulation.py \
		--environment urban_circuit_practice_01 --runtime-map-mode no-static \
		--scenario drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json
	. external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env; \
		SIM_WORLD_SDF_PATH="$$SIM_GUI_WORLD_SDF_PATH" \
		POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json \
		ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d \
		CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-6.5}" \
		ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-10}" \
		MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-5}" \
		./scripts/run_drone_nav_sim.sh
