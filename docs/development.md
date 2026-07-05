# Development Guide

Development should keep planner behavior, runtime control, and diagnostics
separable. Avoid hidden coupling between ROS topics, config defaults, and
offboard runtime decisions.

## Coding Style

Follow:

- `CPP_BEST_PRACTICES.md`
- `AGENTS.md`
- `README.md`
- `CONTRIBUTING.md`

Code comments and repository documentation should be in English.

## Container-Only Workflow

Use the documented scripts and Make targets. Do not run ad-hoc top-level CMake
commands. Do not write workspace files as root unless doing intentional
maintenance with `ALLOW_ROOT_WORKSPACE_WRITE=1`.

## Adding A ROS Node

1. Add source files under `drone_city_nav/src`.
2. Add public interfaces under `drone_city_nav/include/drone_city_nav` only
   when needed.
3. Register the target in `drone_city_nav/CMakeLists.txt`.
4. Add parameters to config with matching C++ defaults.
5. Add launch wiring in `city_nav.launch.py` when the node belongs to the main
   run graph.
6. Add topic/config tests where appropriate.

## Adding A Parameter

When adding a parameter:

1. Add the C++ default.
2. Add the YAML default.
3. Add sanitization/clamping if needed.
4. Add config tests.
5. Add docs in `configuration.md`.
6. Include the parameter in the right fingerprint if it affects a documented
   runtime or planning contract.

Avoid a state where behavior changes depending on whether YAML was loaded.

## Adding A Topic

Define:

- publisher node;
- subscriber node;
- message type;
- QoS;
- frame id and coordinate convention;
- whether the topic is raw input, executable output, or debug-only output.

Debug/prohibited outputs must not be wired back into raw planner inputs.

## Planner / Runtime Separation

Planner owns:

- obstacle merging;
- grid inflation and planning clearance;
- route and trajectory generation;
- planning diagnostics.

Offboard owns:

- accepted executable trajectory state;
- runtime speed profile;
- trajectory continuity handling;
- velocity and position setpoints;
- runtime telemetry.

Do not make planner diagnostics authoritative for runtime control unless the
trajectory artifact explicitly carries that contract.

## Safe Refactoring

Before refactoring:

- identify public headers and tests;
- check size contracts;
- preserve diagnostics unless intentionally changing them;
- add tests for new decision modules;
- avoid changing trajectory/control behavior in an extract-only refactor;
- keep generated files out of commits.

For behavior changes, make the changed contract explicit in tests and logs.
