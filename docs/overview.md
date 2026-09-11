# Project Overview

This repository is a ROS 2 workspace for PX4/Gazebo drone navigation. Its
production navigation stack uses revisioned raw Occupancy3D, one persistent
full-3D strategic planner, and GPU MPPI with explicit route ownership, safety
evidence, and execution-horizon contracts.

The project is a simulation-oriented research system. It is not certified for
real-aircraft operation.

## Current Capabilities

- Gazebo Harmonic simulation with PX4 SITL and a GPU lidar.
- Static-map and no-static operating modes.
- Map-frame lidar projection and accumulated obstacle memory.
- Atomic raw-obstacle snapshots with provenance and revisions.
- Raw swept-footprint collision validation plus distance-based soft risk bands.
- A persistent sparse D* Lite route repaired across compatible world revisions.
- Full-3D route progress, certified successor reserve, and atomic suffix repair.
- Speed-dependent tracking-error tubes attached to immutable route geometry.
- CUDA MPPI local planning at a receding horizon.
- Map-independent parameterized speed policy.
- Timestamped execution horizons consumed by the MPPI offboard node.
- Typed position hold when no physically executable route is available.
- Terminal-point or current-position hold when no fresh finite path is available.
- Canonical 3D static world with a `5 x 8` Manhattan grid, two L-shaped passages,
  and one straight-through passage.
- Typed vehicle destruction from Gazebo contact or 5 m proximity intercept.
- Swept oriented 3D drone-footprint collision checks.
- A configured half-open flight envelope, currently `1.0 <= z < 32.0 m`.
- RViz, JSONL, lidar snapshots, and mission diagnostics.

## Main Runtime Nodes

- `obstacle_memory_node` owns lidar ingestion, memory, and raw world snapshots.
- `world_visualization_node` publishes static and raw world geometry.
- `production_mppi_node` is the ROS composition root and adapter for immutable
  world, planning, route, execution, controller, and diagnostics services.
- `ExecutionSupervisor3D` owns pending/active execution state and the complete
  currentness, revalidation, owner/control, and atomic horizon-commit transaction;
  the node publishes DDS only after that transaction succeeds.
- `mppi_offboard_node` executes fresh timestamped horizons through PX4.
- `collision_crash_node` converts Gazebo contacts into typed physical-destruction
  events.
- `mission_monitor_node` observes mission completion and physical crashes.
- `lidar_debug_node` records map-frame lidar and navigation snapshots.

## Main Run Modes

```bash
./scripts/sim_headless.sh
./scripts/sim_gui.sh
```

Runs default to `ENABLE_STATIC_MAP=false` and use the selected lidar memory as
the world source. Set `ENABLE_STATIC_MAP=true` explicitly to use the known city
map. Speed is configured explicitly and does not depend on map mode.

No-static navigation requires the 3D lidar profile. It integrates hit and miss
rays into revisioned observed Occupancy3D. Unknown and confirmed-free voxels
have the same strategic traversability and base cost; only raw occupied evidence
and the physical flight envelope can reject a strategic edge.

All build, test, quality, and simulation commands must run through the
repository container workflow.

## Important Terms

- **Raw occupancy**: direct static-map or sensor evidence. Raw collision is a
  hard reject.
- **Occupied-distance field**: derived evidence used for soft clearance ranking
  and controller queries. It cannot create hard occupied geometry.
- **Risk tier**: preferred, planning, critical, or collision.
- **Persistent route**: immutable certified full-3D geometry owned across
  compatible updates and incrementally repaired by the D* Lite planner.
- **Execution plan**: an atomic bundle containing route geometry, tracking tube,
  nominal horizon, braking fallback, and exact evidence revisions.
- **MPPI horizon**: the short dynamically simulated trajectory recomputed on
  every planning tick.
- **Execution horizon**: a timestamped MPPI horizon published to offboard.
- **Constrained route span**: a section of the 3D route whose free-space envelope
  limits altitude or speed. It is derived from occupancy, not annotations.

## Explicit Non-Capabilities

- Offline `FreeSpaceTopology3D` is optional static passage evidence; it is not a
  competing route producer or a source of hard occupancy.
- There is no 2D production navigation branch, online frontier planner,
  direct-versus-topology arbitration, or location-specific opening logic.
- The persistent graph uses a complete minimum-resolution 26-connected lattice
  plus world-aligned multiresolution overlays. Lazy exact raw validation admits
  long open-volume edges and automatically leaves the fine lattice in control
  around occupied geometry.
- Current-position hold is not a substitute for finding a physically executable
  route.

The stack is validated with a heading source of about 1.5 degrees standard
deviation or better (in simulation, the true attitude through PX4's external
vision interface, see `docs/gazebo_simulation.md`, "Heading Source"). It is not
validated with a compass-grade heading of 2 degrees and more.

## Documentation Map

- `roadmap.md`: completed milestones and planned navigation, multi-drone, and 3D
  perception work.
- `architecture.md`: node ownership and data flow.
- `navigation_pipeline.md`: current world-to-control pipeline.
- `world3d.md`: canonical world generation, raw Occupancy3D,
  FreeSpaceTopology3D, and constrained spans.
- `environment_candidates.md`: external world selection, versioned artifact
  distribution, static-map import, and sparse-topology evidence.
- `trajectory_optimization.md`: GPU MPPI optimization.
- `replanning.md`: persistent route repair, atomic execution plans, and liveness.
- `obstacle_mapping.md`: static, lidar, memory, and raw snapshot sources.
- `configuration.md`: parameter groups and source-of-truth guidance.
- `diagnostics.md`: current logs, metrics, and artifacts.
- `rviz.md`: current visualization layers.
- `build_and_run.md`: supported container commands.
