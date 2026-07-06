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

## Adding Or Changing A Planning Stage

When adding a planning stage, define its contract before writing the code:

- input artifact and owner;
- output artifact and owner;
- hard validation rules;
- diagnostics that prove why it changed the path;
- timing fields;
- tests or script checks;
- how it behaves when it fails.

The stage should not silently change ownership. For example, a geometry cleanup
stage can modify trajectory points if that is its documented job. A
speed-profile diagnostic stage should not secretly change the executable
geometry.

## Adding Or Changing Runtime Control Logic

Runtime control changes should identify which layer is affected:

- projection and prediction;
- projection smoothing;
- P gain schedule;
- D gain schedule;
- curvature feedforward;
- speed policy;
- velocity smoother;
- terminal state machine;
- PX4 setpoint mode.

Do not add a new feature that compensates for another feature without proving
the existing layer cannot be tuned or simplified. Lateral control in particular
should remain understandable: predicted projection, optional smoothing, P plus
D plus feedforward, angle cap, smoother.

## Documentation Requirements For New Features

Every non-trivial feature should update docs in the same change:

- configuration reference for new parameters;
- architecture or pipeline docs for ownership changes;
- diagnostics docs for new log fields;
- troubleshooting docs for common failure modes;
- performance docs if the feature changes timing.

Documentation should explain the reason for the feature, not only its name.
Avoid leaving a parameter in YAML without describing what layer owns it and how
to read its diagnostics.

## Review Checklist

Before committing a planning or control change, check:

- code defaults and YAML defaults match for active features;
- stale diagnostics or docs were removed;
- path publication cannot erase a valid old trajectory on failure;
- offboard continuity behavior is logged;
- new decision modules have direct tests;
- blackbox fields are enough to diagnose the new behavior;
- script contracts still match the current telemetry schema.

This checklist keeps the project from accumulating hidden legacy assumptions as
the system grows.
