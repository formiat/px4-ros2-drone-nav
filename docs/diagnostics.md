# Logging And Diagnostics

Production diagnostics are designed to observe control without delaying
execution-horizon publication.

## Main Artifacts

- `log/mppi/`: MPPI JSONL summaries.
- `log/lidar_debug/`: synchronized lidar/grid/path snapshots.
- `log/lidar_memory_hits/`: accepted and classified memory-hit records.
- `log/gz_drone_nav.log`: Gazebo server and orchestration log.
- `log/gz_gui_drone_nav.log`: Gazebo GUI log.
- `log/gazebo_scene_debug/`: bounded GUI scene and camera diagnostics.
- ROS logs under `log/latest/` and timestamped run directories.

The runner may place artifacts in a per-run directory. Use the path printed by
the script rather than assuming that `log/latest` belongs to the intended run.

## MPPI Tick Diagnostics

`PRODUCTION_MPPI_TICK` reports:

- pose, obstacle, memory, and ESDF revisions and ages;
- target source and target position;
- active guide generation, status, and remaining length;
- static/no-static speed-policy limits;
- passage phase and opening id;
- GPU and host stage timings;
- selected tier and risk exposure;
- raw and known-solid collision flags;
- head and terminal progress;
- horizon stability and first-control delta;
- liveness action;
- post-update classification;
- dropped diagnostic snapshots.

The JSONL record carries the same data in machine-readable form.

## Timing

Relevant stages include:

- input snapshot;
- ESDF build and upload latency;
- noise generation;
- rollout simulation;
- risk reduction;
- weight and control update;
- horizon reconstruction;
- safety evaluation;
- total GPU and host tick time;
- asynchronous RViz/JSONL work.

ESDF build latency is asynchronous and must not be interpreted as part of
`gpu_total_ms`.

## Global Guide Diagnostics

Inspect:

- lattice status and termination reason;
- reached-goal/frontier/dead-end classification;
- guide fingerprint and generation;
- sticky-guide retain/release reason;
- current route station and remaining distance;
- blocked, exhausted, cross-track, and stall state;
- heading-source cascade.

Diagnostic lattice classification is observational unless a separate lifecycle
condition explicitly consumes it.

## Liveness Diagnostics

Compare:

- actual displacement;
- predicted head progress;
- predicted terminal progress;
- observation age;
- reseed generation;
- guide stall generation.

High terminal progress with no actual displacement indicates an ineffective
horizon, not successful navigation.

## Passage Diagnostics

Inspect:

- active semantic portal and route event;
- approach/alignment/traversal phase;
- vertical and lateral errors;
- capture and retention counters;
- stationary hold activation and target;
- known-solid collision;
- actual opening entry and minimum margins from the mission monitor.

Planner activation is not proof that the drone entered the opening.

## Offboard Diagnostics

Important events:

- horizon rejection reason;
- `MPPI_HORIZON_DEADLINE_MISSED`;
- dynamic braking activation;
- stationary passage hold activation;
- applied-control feedback age;
- PX4 mode/arming state.

An expired horizon must result in braking or hold, never continued open-loop
execution.

## Lidar Diagnostics

Use lidar snapshots to verify:

- heading was accepted before projection;
- scan and pose timestamps align;
- raw returns and map-frame points agree;
- memory contains plausible retained evidence;
- static/no-static source selection is correct;
- optional known-static classifier decisions are explainable.

## Run Analysis Order

1. Confirm mission/crash result and final pose.
2. Confirm pose, heading, raw snapshot, and ESDF freshness.
3. Inspect active global guide and target source.
4. Inspect selected MPPI tier and collision flags.
5. Inspect head progress, actual motion, and liveness.
6. Inspect passage state if active.
7. Inspect offboard deadline/braking behavior.
8. Only then tune costs or dynamics.
