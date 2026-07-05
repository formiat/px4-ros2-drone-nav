# Navigation Pipeline

The navigation pipeline turns raw obstacle evidence and a mission goal into an
accepted executable trajectory.

## 1. Raw Obstacle Sources

The planner can use:

- static map cells from `generated_city.map2d`;
- accumulated obstacle memory from `/drone_city_nav/obstacle_memory_grid`;
- current lidar scan overlay from `/scan`.

Raw sources are merged before inflation. Raw sources must not contain safety
inflation.

## 2. Prohibited Grid

The planner grid builder creates a hard-safety prohibited grid. Current
defaults:

```yaml
inflation_radius_m: 1.0
planning_clearance_m: 3.0
```

The prohibited grid is the hard validation grid. The extra planning clearance
is used to bias planning away from obstacles and does not by itself cause a
runtime replan.

The published grid is:

```text
/drone_city_nav/prohibited_grid
```

## 3. A* Rough Route

A* searches the grid from the current or initial position to the goal. Current
important parameters:

- `astar_heuristic_weight`
- `astar_turn_cost_weight`
- `astar_initial_heading_bias_enabled`
- `astar_evasive_maneuvering_enabled`

Evasive maneuvering is disabled by default. Initial heading bias is enabled.

## 4. Path Collapse And Simplification

The rough grid path is collapsed into fewer route points. This route is still
not the final trajectory. It is used as centerline input for corridor and
trajectory generation.

## 5. Corridor Construction

The corridor builder samples the route and finds lateral bounds. It uses the
prohibited/planning grid to avoid invalid space and can reuse clearance fields
and previous samples when safe.

Important concepts:

- sample step;
- left/right clearance;
- lateral limit window;
- center recovery;
- route prohibited samples;
- corridor width and clearance diagnostics.

## 6. Trajectory Optimizer

The trajectory optimizer works inside the corridor. It searches lateral offsets
and prefers smooth geometry with larger radii and lower curvature jumps.

The optimizer is not a "racing line" anymore. Its current goal is smooth,
trackable geometry rather than shortest or fastest traversal.

## 7. Turn Smoothing

Turn smoothing runs after the optimizer and attempts to repair remaining local
kinks. It evaluates candidate arcs/curves around problematic corners and
accepts candidates that improve local geometry while staying traversable.

## 8. Isolated Geometry Spike Smoothing

The shape-cleanup stage handles isolated curvature spikes that survive the
optimizer and turn smoothing. It is a geometry cleanup layer, not a
speed-profile-only patch.

## 9. Speed Profile Construction

A speed profile is built for trajectory samples using curvature, acceleration,
deceleration, minimum turn speed, and lookahead policy. Planner-side speed
profile diagnostics are planning diagnostics; offboard rebuilds the runtime
profile for actual control.

## 10. Publication

The planner publishes:

- `/drone_city_nav/path`
- `/drone_city_nav/path_id`
- `/drone_city_nav/trajectory_diagnostics`
- `/drone_city_nav/current_waypoint`

The offboard node publishes:

- `/drone_city_nav/final_trajectory_path`
- `/drone_city_nav/offboard_debug_markers`

The accepted trajectory is matched to diagnostics by `path_stamp_ns`.
