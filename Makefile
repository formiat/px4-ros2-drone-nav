SHELL := /usr/bin/env bash

.PHONY: build
build:
	colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

.PHONY: test
test: build
	ctest --test-dir build/drone_city_nav --output-on-failure

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
