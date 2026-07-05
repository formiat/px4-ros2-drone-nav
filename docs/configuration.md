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
