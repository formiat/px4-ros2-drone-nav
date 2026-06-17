SHELL := /usr/bin/env bash

.DEFAULT_GOAL := host-build

COLCON_BUILD_BASE ?= build
COLCON_INSTALL_BASE ?= install
COLCON_LOG_BASE ?= log

.PHONY: build
build:
	colcon --log-base $(COLCON_LOG_BASE) build --packages-select drone_city_nav --symlink-install --build-base $(COLCON_BUILD_BASE) --install-base $(COLCON_INSTALL_BASE) --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

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

.PHONY: sim-gui
sim-gui:
	./scripts/run_city_mvp.sh

.PHONY: sim-headless
sim-headless:
	HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh

.PHONY: host-shell
host-shell:
	./scripts/host_shell.sh

.PHONY: host-build
host-build:
	./scripts/host_shell.sh make build

.PHONY: host-test
host-test:
	./scripts/host_shell.sh make test

.PHONY: host-test-scripts
host-test-scripts:
	./scripts/host_shell.sh make test-scripts

.PHONY: host-quality
host-quality:
	./scripts/host_shell.sh make quality

.PHONY: host-format-check
host-format-check:
	./scripts/host_shell.sh make format-check

.PHONY: host-format
host-format:
	./scripts/host_shell.sh make format

.PHONY: host-sim-gui
host-sim-gui:
	./scripts/run_city_mvp_host.sh

.PHONY: host-sim-headless
host-sim-headless:
	HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp_host.sh

.PHONY: host-speed-sweep
host-speed-sweep:
	./scripts/run_speed_sweep_host.sh

.PHONY: host-record-debug-bag
host-record-debug-bag:
	./scripts/record_debug_bag_host.sh
