# Navigation Pipeline

The production pipeline converts current vehicle state and obstacle evidence
into a timestamped local trajectory horizon.

## 1. Raw World Snapshot

`obstacle_memory_node` integrates accepted lidar returns into a scored 2D
memory grid. `/drone_city_nav/obstacle_memory_snapshot` carries that grid and
its provenance atomically; `/drone_city_nav/raw_obstacle_snapshot` carries the
validated raw runtime grid and risk-policy fingerprint. The matching RViz
occupancy grid is debug-only and is never used as planner input.

In no-static mode this snapshot is the planning world. In static mode it keeps
sensor diagnostics and snapshot freshness alive, but the planner loads
canonical Occupancy3D directly and does not merge this 2D grid into the static
3D map.

## 2. ESDF Preparation

The production MPPI node builds a mode-specific occupied-distance field
asynchronously. Static mode materializes a local dense ESDF3D from canonical
Occupancy3D. No-static mode builds an ESDF2D from the latest raw lidar-memory
snapshot. MPPI continues using the last complete immutable field until a newer
revision is ready.

Distance classifications are:

- raw collision: hard reject;
- critical band: highest non-collision risk;
- planning band: elevated risk;
- preferred space: normal risk.

The current defaults are defined in `config/urban_mvp.yaml`; documentation must
not duplicate YAML as a second parameter source of truth.

## 3. Global Lattice Guide

Static mode uses a 3D lattice over physical free voxels and produces
`RouteSample3D` samples. No-static retains the 2D lidar-driven lattice.

Lattice output is classified as reached-goal, viable frontier, search
incomplete, or exhausted. Only reached-goal and viable-frontier results are
executable. Incomplete search is continued when possible; exhausted output is
not accepted as a guide.

No-static frontier selection keeps candidates from distinct departure
directions before evaluating continuation depth. Temporary zero or negative
Euclidean progress toward the mission goal is allowed when it provides a
locally viable detour. Goal distance is a soft ranking term, not a frontier
eligibility condition.

An accepted guide is sticky. It is retained across ESDF revisions while its
remaining portion is valid and useful. Replacement reasons include blocking,
exhaustion, excessive cross-track error, and observed stall.

Initial search heading uses a cascade:

1. velocity heading at normal speed;
2. previous accepted-guide tangent at low speed;
3. mission direction when no accepted guide exists.

## 4. Target And Speed Policy

The planner selects a lookahead point on the active guide. Static mode uses a
longer dynamic lookahead and higher speed profile; no-static mode uses a
shorter guide, horizon, and speed cap.

Reference speed is bounded by:

- mode cruise and absolute limits;
- curvature preview;
- observed-space/braking constraints;
- goal approach;
- constrained-route span limits.

When no guide exists in no-static mode, direct flight to the distant mission
goal is forbidden. The planner publishes braking/hold behavior instead.

## 5. Constrained Route Spans

Static air channels are ordinary physical free space represented by generated
constrained graph edges. Global search explicitly evaluates
`start -> entry -> channel -> exit -> planning goal` topology candidates. A
selected channel edge creates its constrained station interval directly; local
clearance analysis remains a validation step rather than a nearest-opening
selector or separate passage lifecycle.

## 6. GPU MPPI

Each planning tick:

1. shifts the previous nominal controls by elapsed time;
2. generates CUDA control perturbations;
3. simulates thousands of dynamic-state rollouts;
4. queries the 3D ESDF with the swept oriented physical footprint;
5. selects the best eligible risk class;
6. computes the weighted control update;
7. limits the first control relative to applied-control feedback;
8. reconstructs the selected nominal horizon on the host.

MPPI optimizes a short receding horizon. It does not publish a long mission
path for open-loop execution.

## 7. Post-Update Checks And Braking

The reconstructed horizon receives an observational post-update
classification. Raw physical collision causes execution to switch to the
braking fallback. The independent horizon safety check estimates
time-to-collision and stopping capability against the current ESDF. Targets and
all published horizon states must remain inside the configured flight envelope.
The braking hold cannot latch a ground position outside that envelope; after
takeoff it captures the current admissible low-speed position instead.

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
