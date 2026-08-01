# Project Overview

This repository is a ROS 2 workspace for PX4/Gazebo drone navigation. Its
production navigation stack uses raw occupancy, a distance field, a
risk-aware motion-primitive lattice guide, GPU MPPI, and an independent
braking supervisor.

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
- Braking fallback when the selected horizon is not executable.
- Canonical 3D static world with horizontal straight and L-shaped channels.
- Gazebo contact-based crash detection.
- RViz, JSONL, lidar snapshots, and mission diagnostics.

## Main Runtime Nodes

- `obstacle_memory_node` owns lidar ingestion, memory, and raw world snapshots.
- `world_visualization_node` publishes static and raw world geometry.
- `production_mppi_node` owns ESDF preparation, 3D lattice routes, MPPI, and
  horizon publication.
- `mppi_offboard_node` executes fresh timestamped horizons through PX4.
- `collision_crash_node` converts Gazebo contacts into a latched crash state.
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

Only static mode executes canonical air channels. No-static has no channel
semantics or 3D perception; its 2D lidar visibility profile presents all
connector openings as obstacles.

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
- The braking fallback is an approximate reachable braking trajectory, not a
  full reachable-set solver.

## Documentation Map

- `architecture.md`: node ownership and data flow.
- `navigation_pipeline.md`: current world-to-control pipeline.
- `world3d.md`: canonical world generation, Occupancy3D, and constrained spans.
- `trajectory_optimization.md`: GPU MPPI optimization.
- `replanning.md`: receding-horizon updates, guide replacement, and liveness.
- `obstacle_mapping.md`: static, lidar, memory, and raw snapshot sources.
- `configuration.md`: parameter groups and source-of-truth guidance.
- `diagnostics.md`: current logs, metrics, and artifacts.
- `rviz.md`: current visualization layers.
- `build_and_run.md`: supported container commands.
