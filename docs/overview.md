# Project Overview

This repository is a ROS 2 workspace for PX4/Gazebo drone navigation. Its
production navigation stack uses raw occupancy, a distance field, a
risk-aware motion-primitive lattice guide, and GPU MPPI with explicit route
availability and execution-horizon contracts.

The project is a simulation-oriented research system. It is not certified for
real-aircraft operation.

## Current Capabilities

- Gazebo Harmonic simulation with PX4 SITL and a GPU lidar.
- Static-map and no-static operating modes.
- Map-frame lidar projection and accumulated obstacle memory.
- Atomic raw-obstacle snapshots with provenance and revisions.
- ESDF-based collision queries and categorical risk bands.
- A sticky global lattice guide for route direction.
- CUDA MPPI local planning at a receding horizon.
- Static and no-static speed policies.
- Timestamped execution horizons consumed by the MPPI offboard node.
- Typed position hold when no physically executable route is available.
- Terminal-point or current-position hold when no fresh finite path is available.
- Canonical 3D static world with a `5 x 8` Manhattan grid, two L-shaped channels,
  and one straight-through channel.
- Typed vehicle destruction from Gazebo contact or 5 m proximity intercept.
- Swept oriented 3D drone-footprint collision checks.
- A configured half-open flight envelope, currently `1.0 <= z < 32.0 m`.
- RViz, JSONL, lidar snapshots, and mission diagnostics.

## Main Runtime Nodes

- `obstacle_memory_node` owns lidar ingestion, memory, and raw world snapshots.
- `world_visualization_node` publishes static and raw world geometry.
- `production_mppi_node` owns ESDF preparation, 3D lattice routes, MPPI, and
  horizon publication.
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

`ENABLE_STATIC_MAP=true` uses the known city map and the long, high-speed static
profile. `ENABLE_STATIC_MAP=false` uses lidar memory as the world source and the
shorter, conservative no-static profile.

Only static mode can execute the canonical air channels. No-static has no
channel semantics or 3D perception; collisionless lidar occluders make every
open bridge and intersection appear occupied.

All build, test, quality, and simulation commands must run through the
repository container workflow.

## Important Terms

- **Raw occupancy**: direct static-map or sensor evidence. Raw collision is a
  hard reject.
- **ESDF**: occupied-distance field used for collision queries and risk-band
  classification.
- **Risk tier**: preferred, planning, critical, or collision.
- **Global lattice guide**: a locally planned route-direction polyline. It is
  not a persistent topological street graph.
- **MPPI horizon**: the short dynamically simulated trajectory recomputed on
  every planning tick.
- **Execution horizon**: a timestamped MPPI horizon published to offboard.
- **Constrained route span**: a section of the 3D route whose free-space envelope
  limits altitude or speed. It is derived from occupancy, not annotations.

## Explicit Non-Capabilities

- The lattice search is recomputed; it is not AD*, LPA*, or D* Lite.
- No persistent no-static topological memory exists yet.
- No-static does not infer 3D channels because the vehicle has a 2D lidar.
- Current-position hold is not a substitute for finding a physically executable
  route.

## Documentation Map

- `roadmap.md`: planned interceptor, radar, multi-drone, and 3D passage work.
- `architecture.md`: node ownership and data flow.
- `navigation_pipeline.md`: current world-to-control pipeline.
- `world3d.md`: canonical world generation, Occupancy3D, and constrained spans.
- `environment_candidates.md`: external world selection, versioned artifact
  distribution, and static-map import evidence.
- `trajectory_optimization.md`: GPU MPPI optimization.
- `replanning.md`: receding-horizon updates, guide replacement, and liveness.
- `obstacle_mapping.md`: static, lidar, memory, and raw snapshot sources.
- `configuration.md`: parameter groups and source-of-truth guidance.
- `diagnostics.md`: current logs, metrics, and artifacts.
- `rviz.md`: current visualization layers.
- `build_and_run.md`: supported container commands.
