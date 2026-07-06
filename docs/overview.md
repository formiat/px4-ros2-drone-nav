# Project Overview

This repository is a ROS 2 workspace for PX4/Gazebo drone navigation. The main
package is `drone_city_nav`, an ament CMake package that builds the planner,
obstacle-memory, offboard-control, diagnostics, and simulation helper nodes.

The project is no longer just a minimal proof of concept. It is a working
navigation stack with a Gazebo/PX4 simulation environment, a planner that
builds obstacle-aware executable trajectories, and an offboard controller that
tracks those trajectories with velocity and terminal position setpoints.

## Goals

The project aims to provide a practical testbed for:

- obstacle-aware drone navigation in a local 2D planning map;
- static-map, lidar-overlay, and obstacle-memory fusion;
- smooth executable trajectory generation after rough A* routing;
- PX4 offboard control through velocity setpoints;
- robust terminal capture through a final position-setpoint mode;
- repeatable diagnostics for planner, trajectory, control, and simulation runs.

## Current Capabilities

The current stack supports:

- Gazebo simulation with PX4 SITL and an `x500_lidar_2d_0` model;
- ROS 2 nodes for obstacle memory, planning, offboard control, lidar debug, and
  mission monitoring;
- a static city map loaded from `drone_city_nav/worlds/generated_city.map2d`;
- a diagnostics-only known passage annotation map loaded from
  `drone_city_nav/worlds/known_passages.passages3d`;
- current lidar obstacle overlay and accumulated obstacle memory;
- runtime prohibited-grid construction with extra planning clearance;
- A* rough route planning;
- corridor construction around the route;
- smooth trajectory optimization inside the corridor;
- turn smoothing and isolated geometry spike cleanup;
- speed-profile construction for trajectory samples;
- accepted executable trajectory publication with per-sample debug altitude;
- offboard velocity following with P/D cross-track control, curvature
  feedforward, projection smoothing, and velocity smoothing;
- a terminal state machine that transitions from cruise to velocity terminal
  capture, position capture, and final hold;
- RViz visualization, trajectory/corridor dumps, lidar snapshots, and
  offboard blackbox telemetry.

## Main Run Modes

- Gazebo simulation: `./scripts/sim_gui.sh`
- Headless smoke simulation: `./scripts/sim_headless.sh`
- Build-only workflow: `./scripts/build.sh`
- Test workflow: `./scripts/test.sh`
- Interactive container workflow: `./scripts/dev_shell.sh`
- RViz debug view: enabled through the `enable_rviz` launch argument or the GUI
  simulation workflow.

The container workflow is the only supported workflow. Do not run ad-hoc
top-level CMake commands from the host.

## Non-Goals

This project is not currently intended to provide:

- a production-certified flight stack;
- real-aircraft safety guarantees;
- GPS/global-map mission planning;
- multi-drone coordination;
- full 3D volumetric planning;
- a general SLAM system;
- support for arbitrary simulator versions outside the repository container.

The planner still performs XY obstacle avoidance and trajectory shaping in a
2D navigation representation. Executable trajectory samples now also carry
`z_m`, currently initialized from `cruise_altitude_m`, so RViz paths, markers,
and dumps can represent the trajectory in 3D. Runtime vertical control still
holds the configured cruise altitude in this stage.

Known 3D passages are currently annotations only. Passage structures,
openings, gate centers, approach arrows, and exit arrows are loaded and
published for RViz/debugging, but they are not added to or removed from
`prohibited_grid` and do not affect A*, corridor construction, trajectory
optimization, speed profile, or offboard control yet.

## Important Terms

- Raw obstacle source: direct obstacle evidence from a static map, lidar
  overlay, or memory grid. Raw sources must not contain safety inflation.
- Prohibited grid: the runtime grid after raw sources are merged and inflated.
  This is the hard safety grid used for validation and replan triggers.
- Planning clearance: additional planner-only clearance applied while building
  paths and trajectories. Entering the planning-clearance margin is not itself
  a replan reason.
- Executable trajectory: the accepted path that the offboard controller tracks.
  Its geometry and speed profile are currently XY-owned, while `z_m` is a
  representation/debug altitude for RViz and diagnostics.
- Known passage: a pre-annotated 3D passage structure and opening that can be
  visualized now and used by future 3D traversal stages.
- Trajectory optimizer: the post-corridor optimizer that improves smoothness
  and radius while staying inside the valid corridor.
- Terminal capture: the final control state sequence that slows down, enters
  position capture, and holds the goal.

## Documentation Map

- `installation.md` explains host setup.
- `build_and_run.md` explains the container commands.
- `gazebo_simulation.md` explains simulator-specific behavior.
- `rviz.md` explains visualization layers.
- `architecture.md` explains nodes and data flow.
- `navigation_pipeline.md` explains planner stages.
- `trajectory_optimization.md` explains smoothing and optimization.
- `drone_control.md` explains offboard trajectory following.
- `terminal_capture.md` explains final-goal behavior.
- `replanning.md` explains path invalidation and replans.
- `obstacle_mapping.md` explains static, lidar, and memory obstacle sources.
- `configuration.md` explains the main configuration groups.
- `diagnostics.md` explains logs, dumps, and blackbox telemetry.
- `testing.md` explains verification commands.
- `development.md` explains repository development rules.
- `troubleshooting.md` lists common failures.
- `performance.md` explains timing diagnostics and bottlenecks.
