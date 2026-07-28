# Navigation Pipeline

The production pipeline converts current vehicle state and obstacle evidence
into a timestamped local trajectory horizon.

## 1. Raw World Snapshot

`obstacle_memory_node` combines:

- the optional static city map;
- accumulated lidar obstacle memory;
- current sensor updates and provenance.

It publishes one atomic `/drone_city_nav/raw_obstacle_snapshot`. The matching
RViz occupancy grid is debug-only and is never used as planner input.

## 2. ESDF Preparation

The production MPPI node builds an occupied-distance field for each accepted
raw snapshot. ESDF construction is asynchronous. MPPI continues using the last
complete immutable ESDF until a newer revision is ready.

Distance classifications are:

- raw collision: hard reject;
- critical band: highest non-collision risk;
- planning band: elevated risk;
- preferred space: normal risk.

The current defaults are defined in `config/urban_mvp.yaml`; documentation must
not duplicate YAML as a second parameter source of truth.

## 3. Global Lattice Guide

The risk-aware lattice searches motion primitives over position and heading.
It produces a bounded guide polyline toward the mission goal.

Lattice output is classified as reached-goal, viable frontier, or dead end for
diagnostics. The current classification is observational and does not by itself
change runtime acceptance.

An accepted guide is sticky. It is retained across ESDF revisions while its
remaining portion is valid and useful. Replacement reasons include blocking,
exhaustion, excessive cross-track error, and observed stall.

Initial search heading uses a cascade:

1. velocity heading at normal speed;
2. previous accepted-guide tangent at low speed;
3. yaw or mission direction when no accepted guide exists.

## 4. Target And Speed Policy

The planner selects a lookahead point on the active guide. Static mode uses a
longer dynamic lookahead and higher speed profile; no-static mode uses a
shorter guide, horizon, and speed cap.

Reference speed is bounded by:

- mode cruise and absolute limits;
- curvature preview;
- observed-space/braking constraints;
- goal approach;
- an active passage limit.

When no guide exists in no-static mode, direct flight to the distant mission
goal is forbidden. The planner publishes braking/hold behavior instead.

## 5. Passage Coordination

If the guide crosses a known opening, the passage coordinator can activate an
approach, stationary vertical alignment, traversal, or partial-from-inside
phase. It supplies MPPI with known 3D constraints and can request an explicit
XY position hold while altitude is captured.

See `known_passages.md` for the detailed contract.

## 6. GPU MPPI

Each planning tick:

1. shifts the previous nominal controls by elapsed time;
2. generates CUDA control perturbations;
3. simulates thousands of point-mass rollouts;
4. queries ESDF and known-solid geometry;
5. selects the best eligible risk class;
6. computes the weighted control update;
7. limits the first control relative to applied-control feedback;
8. reconstructs the selected nominal horizon on the host.

MPPI optimizes a short receding horizon. It does not publish a long mission
path for open-loop execution.

## 7. Post-Update Checks And Braking

The reconstructed horizon receives an observational post-update
classification. Raw or known-solid collision causes execution to switch to the
braking fallback. The independent horizon safety check estimates
time-to-collision and stopping capability against the current ESDF.

The liveness monitor compares predicted and actual progress. Persistent
prediction without real movement can reseed the MPPI nominal controls and
release a stalled guide.

## 8. Horizon Publication And Execution

The production node publishes the execution horizon immediately after planning
and safety evaluation. RViz, INFO summaries, and JSONL diagnostics run outside
the control-critical publication path.

Offboard consumes the fresh horizon, interpolates by timestamp, and emits PX4
trajectory setpoints. If the horizon expires, offboard brakes instead of
continuing an old path.

## Removed Legacy Stages

The production runtime no longer contains:

- grid A* path publication;
- corridor construction;
- post-corridor trajectory optimization;
- separate turn smoothing;
- partial-replan races;
- prefix/suffix stitching;
- safe truncation;
- planner/prohibited inflated grids;
- inflation relaxation or escape tunnels;
- the legacy speed planner and terminal-capture path lifecycle.
