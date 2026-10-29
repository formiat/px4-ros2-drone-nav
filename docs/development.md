# Development Guide

Development should keep planner behavior, runtime control, and diagnostics
separable. Avoid hidden coupling between ROS topics, config defaults, and
offboard runtime decisions.

## Coding Style

Follow:

- `cpp_best_practices.md` (this folder)
- `AGENTS.md`
- `README.md`
- `CONTRIBUTING.md`

Code comments and repository documentation should be in English.

## Container-Only Workflow

Use the documented scripts and Make targets. Do not run ad-hoc top-level CMake
commands. Do not write workspace files as root unless doing intentional
maintenance with `ALLOW_ROOT_WORKSPACE_WRITE=1`.

## The Development Host In The Cloud (weighed 2026-10-29)

The owner's question of 2026-10-29: can the builds, the simulations and the
agent's work move to a rented machine, so that they run round the clock
without the owner's computer on. Yes, under one hard condition that rules
most GPU rentals out.

**What the workflow needs of a host.** `scripts/container_run.sh` runs the
flight image with `docker run --privileged --gpus all --cpuset-cpus 0-13`
(the core set of A4) on the NVIDIA container runtime: the host must give a
full virtual machine or bare metal with its own Docker daemon and an NVIDIA
driver, not a container. A GPU "pod" (RunPod, the ordinary Vast.ai
instances) is itself an unprivileged container without nested Docker, and
cannot run the image; Vast.ai's virtual-machine hosts (`vms_enabled`) can.
The rest, by this host's figures: 16 logical cores (the series' set is 14 of
them), about 30 GiB of RAM, a CUDA 12.6 driver, Gazebo's headless rendering
of the stereo pair through EGL (unverified on a data-centre GPU such as the
L4: the first check on any trial machine), about 25 GB of images
(`drone-gazebo-dev` 9.1 GB, the PX4 base 6.1, the tool image 6.0, the
onboard stages 4.1) plus 3.8 GB of environment artifacts and 36 GB of the
datasets of roadmap item 25 under `/data/gazebo/`. The agent (Claude Code)
works on such a host through `ssh` and `tmux` as it does here; the
repository and its logs move with `git` and `rsync`.

**What it costs**, hourly per GPU instance plus storage, as the aggregators
quoted on 2026-10-29 (checked against the providers' own pages before
renting):

| Option | Kind | Price |
|---|---|---|
| AWS g6.4xlarge (L4 24 GB, 16 vCPU, 64 GiB) | virtual machine | about $1.32 an hour on demand (us-east-1); a 1-year plan lower |
| Vast.ai, RTX 4090 | container, or a virtual machine on `vms_enabled` hosts | $0.14 to $0.40 an hour, marketplace |
| RunPod, RTX 4090 | container only (no nested Docker) | $0.34 (community) to $0.89 (secure) an hour |
| Hetzner GEX line (RTX 4000 SFF Ada class and up) | bare metal, monthly | the GEX44 was EUR 184 to 232 a month before the 2026 repricing; the line changed to the GEX45 and GEX63 in autumn 2026, their prices not in the sources found |

A month of round-the-clock work is therefore about $950 on the AWS
instance, $100 to $300 on a Vast.ai machine, and a fixed monthly sum on
Hetzner; a machine switched off between sessions costs its storage alone.
Item 38 (faster than real time) is what a stronger rented host would buy on
top. Not decided; recorded for the choice.

Sources: [Hostinger, cloud GPU pricing](https://www.hostinger.com/tutorials/cloud-gpu-pricing/),
[gpu.ai price index](https://gpu.ai/gpu-price-index), [gpuperhour](https://gpuperhour.com/),
[DoiT, g6.4xlarge us-east-1](https://www.doit.com/compute/spot/us-east-1/g6.4xlarge),
[Holori, g6.4xlarge](https://calculator.holori.com/aws/ec2/g6.4xlarge),
[Vast.ai, virtual machines](https://docs.vast.ai/guides/instances/virtual-machines),
[RunPod, pods](https://docs.runpod.io/pods/overview%20),
[wz-it, Hetzner GPU servers](https://wz-it.com/blog/hetzner-gpu-server-vergleich/),
[achromatic, Hetzner comparison](https://achromatic.dev/blog/hetzner-server-comparison).

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
- ESDF risk-band classification;
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

### Private Production Services

Keep private application services in the directory owned by their narrow CMake
target:

- `src/world/`: raw-world and coherent world-publication services;
- `src/route_application/`: route-runtime use cases — planning, materialization,
  activation, and execution publication — and their build-only contracts. They
  are one application service, so they are one directory rather than three that
  imply independent layers;
- `src/runtime/`: ROS-free MPPI application services and configuration;
- `src/runtime/ros/`: the production node composition root and ROS adapters.

Do not restore the package-wide `src/` include root to a private runtime target.
Expose a new build-only include directory only through the target that owns it,
and preserve the allowed target dependency direction ROS component -> MPPI ->
route -> world.
The component must not link `drone_city_nav_core` or reach around its immediate
runtime dependency.

Domain orchestration behavior belongs in direct C++ API, state-machine, and
concurrency tests. Python source contracts may check ROS messages, QoS, launch
wiring, or explicit dependency bans; they must not infer transaction behavior
by slicing C++ source with expression order or line position.

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
