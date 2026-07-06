# Configuration Reference

The main parameter file is:

```text
drone_city_nav/config/urban_mvp.yaml
```

The file name is legacy. It remains the active default configuration for
`city_nav.launch.py`.

## Main Nodes In The YAML

- `obstacle_memory_node`
- `planner_node`
- `px4_offboard_node`
- `lidar_debug_node`
- `mission_monitor_node`

When adding a new parameter, keep C++ defaults and YAML defaults synchronized.
This repository intentionally avoids hidden behavior that only works when the
YAML file is present.

## Planner Parameters

Mission and map:

- `start_x_m`, `start_y_m`
- `goal_x_m`, `goal_y_m`
- `cruise_altitude_m`
- `px4_local_origin_x_m`, `px4_local_origin_y_m`
- planning grid resolution, size, and origin.

Obstacle/grid:

- `inflation_radius_m`
- `planning_clearance_m`
- `use_static_map`
- `static_map_path`
- lidar overlay and memory input settings.

## A* Parameters

- `astar_heuristic_weight`
- `astar_turn_cost_weight`
- `astar_evasive_maneuvering_enabled`
- `astar_evasive_maneuvering_straight_cost_weight`
- `astar_initial_heading_bias_enabled`
- `astar_initial_heading_bias_min_speed_mps`
- `astar_initial_heading_bias_weight`

Evasive maneuvering is disabled by default.

## Corridor Parameters

- `corridor_max_radius_m`
- `corridor_sample_step_m`
- `corridor_ray_step_m`
- `corridor_center_recovery_max_m`
- `corridor_lateral_limit_window_m`
- `corridor_lateral_limit_ratio`
- `corridor_lateral_limit_margin_m`
- `corridor_parallel_workers`

Corridor diagnostics report width, clearance, reused samples, clearance-field
reuse, and route-prohibited samples.

## Trajectory Optimizer Parameters

Important smoothing and search parameters:

- `trajectory_optimizer_max_iterations`
- `trajectory_optimizer_optimizer_sample_step_m`
- `trajectory_optimizer_initial_offset_step_m`
- `trajectory_optimizer_min_offset_step_m`
- `trajectory_optimizer_cooling_ratio`
- `trajectory_optimizer_parallel_workers`

Smoothness weights:

- `trajectory_optimizer_weight_curvature`
- `trajectory_optimizer_weight_curvature_change`
- `trajectory_optimizer_preferred_min_radius_m`
- `trajectory_optimizer_weight_radius_shortfall`
- `trajectory_optimizer_weight_offset_change`
- `trajectory_optimizer_weight_offset_second_change`
- `trajectory_optimizer_weight_offset_slope`
- `trajectory_optimizer_max_offset_slope_per_m`

Active windows and DP:

- `trajectory_optimizer_window_*`
- `trajectory_optimizer_dp_*`

Async refinement:

- `trajectory_optimizer_async_refinement_workers`

Default is `0`, so async refinement is disabled.

## Turn Smoothing Parameters

- `turn_smoothing_trigger_heading_delta_deg`
- `turn_smoothing_trigger_min_radius_m`
- `turn_smoothing_trigger_speed_limit_mps`
- `turn_smoothing_entry_distance_m`
- `turn_smoothing_exit_distance_m`
- `turn_smoothing_sample_step_m`
- `turn_smoothing_outer_bias_ratio`
- `turn_smoothing_min_outer_shift_m`
- `turn_smoothing_max_outer_shift_m`
- `turn_smoothing_min_heading_improvement_deg`
- `turn_smoothing_max_passes`

## Speed Profile Parameters

Construction:

- `cruise_speed_mps`
- `min_turn_speed_mps`
- `speed_profile_accel_mps2`
- `speed_profile_decel_mps2`
- `turn_speed_lateral_accel_mps2`
- `speed_profile_sample_step_m`

Runtime policy:

- `speed_profile_lookahead_time_s`
- `speed_profile_lookahead_min_m`
- `speed_profile_lookahead_max_m`
- `setpoint_forward_accel_mps2`
- `setpoint_forward_decel_mps2`

Planner and offboard both expose speed-related settings. Construction,
runtime-speed-policy, and runtime-velocity-control fingerprints help detect
configuration drift.

## Offboard Control Parameters

- `cross_track_gain`
- `cross_track_derivative_gain`
- `cross_track_p_gain_schedule_*`
- `cross_track_d_gain_schedule_*`
- `tracking_prediction_horizon_s`
- `max_lateral_control_angle_deg`
- `setpoint_lateral_response_accel_mps2`
- `curvature_feedforward_*`
- `max_velocity_jerk_mps3`
- `max_lateral_velocity_jerk_mps3`
- `control_tangent_smoothing_*`
- `control_curve_smoothing_*`
- `trajectory_update_max_start_cross_track_m`

## Terminal Capture Parameters

- `acceptance_radius_m`
- `final_hold_max_speed_mps`
- `terminal_capture_radius_m`
- `terminal_capture_gain_1ps`
- `terminal_capture_max_speed_mps`
- `terminal_capture_decel_mps2`
- `terminal_capture_braking_margin_m`
- `terminal_position_capture_max_entry_speed_mps`
- `terminal_stuck_speed_mps`

## Diagnostics Parameters

- `telemetry_log_period_s`
- `flight_blackbox_enabled`
- `flight_blackbox_path`
- `final_trajectory_debug_topic`
- `final_trajectory_debug_sample_step_m`
- `offboard_debug_marker_topic`
- `diagnostic_turn_preview_distance_m`

`diagnostic_turn_preview_distance_m` is diagnostics-only.

## Configuration Philosophy

Configuration values are part of the system contract. A parameter should have a
clear owner, a clear default, and the same value in code defaults and the main
YAML unless there is an explicit reason for a launch-specific override.

The project uses this rule because many failures look like algorithmic
problems but are really configuration drift. If a parameter is enabled in YAML
but disabled by code default, a developer running without the YAML can observe
a different controller. For active features, code defaults and the main
configuration should match.

Parameters should be grouped by physical meaning:

- route and grid parameters describe space and obstacles;
- trajectory parameters describe geometry generation;
- speed-profile parameters describe scalar speed along geometry;
- velocity-control parameters describe runtime setpoint behavior;
- terminal parameters describe final state transitions;
- diagnostic parameters describe logging and visualization only.

## Hard Margins And Planning Margins

Hard obstacle inflation and planning clearance must be tuned together but
understood separately.

Hard inflation is the safety margin around raw obstacle evidence. It defines
the prohibited grid and can trigger replans when the accepted trajectory
intersects it.

Planning clearance is extra margin used during planning. It encourages A*,
corridor construction, and trajectory generation to stay farther away from
obstacles. It should not be treated as a runtime replan boundary.

When changing margins, update both code defaults and YAML. Then verify in RViz
that:

- hard prohibited grid matches the intended collision margin;
- planning-generated trajectories keep the intended extra clearance;
- runtime replans still come only from hard prohibited intersections.

## Speed And Control Parameter Boundaries

Several parameters sound similar but belong to different layers.

`turn_speed_lateral_accel_mps2` describes how fast the drone may fly through a
curve based on radius. It belongs to speed profile and turn-speed feasibility.

`setpoint_lateral_response_accel_mps2` describes how quickly the velocity
setpoint can respond laterally. It belongs to the velocity smoother and
setpoint dynamics.

`setpoint_forward_accel_mps2` and `setpoint_forward_decel_mps2` describe
forward setpoint response. They should not be silently limited by a turn-speed
lateral acceleration budget.

Keeping these names separate prevents one parameter from accidentally meaning
both "how fast through a turn" and "how fast the command can change".

## Fingerprints

Configuration fingerprints are diagnostics for cross-node compatibility. They
are split by purpose:

- speed-profile construction fingerprint;
- runtime speed-policy fingerprint;
- runtime velocity-control fingerprint.

The construction fingerprint is the strongest compatibility check between
planner and offboard because both sides can reason about speed-profile sample
construction. Runtime fingerprints are useful context, but some runtime control
parameters are intentionally offboard-owned. A mismatch there should be
reported carefully so logs do not produce noisy false alarms.

## Safe Tuning Workflow

For behavior changes:

1. Change one conceptual layer at a time.
2. Update code default and YAML together.
3. Run tests and quality checks.
4. Run a simulation and compare blackbox metrics.
5. Inspect RViz for visual regressions.
6. Keep notes on which metric improved and which metric worsened.

Examples:

- To make trajectories rounder, tune trajectory optimizer radius and curvature
  weights before changing the controller.
- To reduce left-right oscillation on a straight, inspect projection smoothing,
  P schedule, D schedule, feedforward suppression, and smoother lag.
- To improve final positioning, tune terminal state thresholds rather than
  normal lateral control.

Avoid stacking several fixes for the same symptom at once. If a change helps,
the logs should show why it helped.

## Deprecated Or Diagnostic-Only Parameters

When a parameter no longer affects control, either remove it or rename it so
its diagnostic-only role is obvious. Stale parameters are dangerous because
they invite tuning that cannot affect the run.

Examples of parameters that should be handled carefully:

- old turn-preview values that are route diagnostics only;
- planner-side speed diagnostics that are not the runtime speed profile;
- any field left over from a removed rate limiter or old lateral feature.

The configuration reference should not preserve legacy names merely for
comfort. If the project no longer uses a concept, the docs should say so or the
concept should be removed.
