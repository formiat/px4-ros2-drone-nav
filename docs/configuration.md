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
- `initial_altitude_m`
- `px4_local_origin_x_m`, `px4_local_origin_y_m`
- planning grid resolution, size, and origin.

`initial_altitude_m` is the startup altitude seed. Before the vehicle has a
valid airborne altitude, it initializes executable trajectory `z_m` samples and
acts as a conservative position-hold fallback. After takeoff, route publication
uses the current vehicle altitude, passage traversal can change altitude, and
terminal position capture holds the current altitude latched at terminal entry.

Obstacle/grid:

- `inflation_radius_m`
- `planning_clearance_m`
- `obstacle_memory_grid_topic`
- `obstacle_memory_provenance_topic`
- `obstacle_memory_snapshot_topic`
- `use_static_map`
- `static_map_path`
- `static_map_grid_topic`
- `static_map_points_topic`
- `static_building_markers_topic`
- lidar overlay and memory input settings.

`obstacle_memory_grid_topic` and `obstacle_memory_provenance_topic` are separate
debug/visualization outputs. The planner does not correlate those topics at
runtime. It consumes `obstacle_memory_snapshot_topic`, whose single typed message
contains both the authoritative raw 2D grid and its exact provenance. The whole
message is rejected unless stamp, frame, geometry, grid content, occupied count,
and provenance records agree. This prevents callback backlog or cross-topic
delivery order from separating a blocker grid from its diagnostic evidence.

Known passages:

- `known_passages_enabled`
- `known_passages_path`
- `known_passage_markers_topic`
- `known_passage_debug_publish_period_s`
- `known_passage_validation_enabled`
- `known_passage_validation_min_opening_overlap_m`
- `known_passage_validation_min_opening_depth_fraction`
- `known_passage_validation_clearance_margin_m`
- `known_passage_validation_max_diagnostics`
- `known_static_lidar_hit_closer_range_tolerance_m`
- `known_static_lidar_hit_farther_range_tolerance_m`
- `ground_lidar_rejection_enabled`
- `ground_lidar_altitude_m`
- `ground_lidar_closer_range_tolerance_m`
- `ground_lidar_farther_range_tolerance_m`
- `obstacle_memory_debug_publish_period_s`
- `obstacle_memory_snapshot_diagnostic_period_s`
- `obstacle_memory_snapshot_max_serialized_bytes`
- `obstacle_memory_snapshot_max_assembly_time_ms`
- `obstacle_memory_snapshot_max_publish_interval_ms`
- `obstacle_memory_snapshot_max_age_ms`
- `obstacle_memory_snapshot_max_callback_time_ms`
- `obstacle_memory_snapshot_max_apply_delay_ms`
- `obstacle_memory_snapshot_min_apply_rate_hz`
- `vertical_profile_preferred_gate_clearance_margin_m`
- `known_passage_traversal_speed_limit_mps`
- `passage_insertion_enabled`
- `passage_insertion_sample_step_m`
- `passage_insertion_min_anchor_margin_m`
- `passage_insertion_max_anchor_margin_m`
- `passage_insertion_opening_lateral_target_margin_m`
- `passage_insertion_repair_clearance_margin_m`
- `passage_insertion_max_lateral_shift_m`
- `passage_insertion_max_join_tangent_delta_deg`
- `passage_insertion_max_join_curvature_jump_1pm`
- `passage_insertion_min_inserted_radius_m`
- `passage_insertion_max_candidates`
- `passage_insertion_max_diagnostics`

The closer and farther tolerances are configured identically for the planner
and obstacle-memory nodes. The always-on 3D classifier compares each measured
hit range with the nearest known passage-building solid. A hit materially
closer than the known surface remains obstacle evidence. A later return can
still match the same known collision surface within the bounded farther
tolerance, accounting for Gazebo collision/projection disagreement. Opening,
boundary, and otherwise ambiguous hits remain obstacles. Missing geometry or
invalid 3D pose is fail-open.

Known passages describe pre-annotated passage structures and openings. They
publish RViz markers, validate whether the final executable trajectory crosses a
known structure footprint through an allowed opening volume, and provide the
known-solid geometry for the lidar classifier.

See `known_passages.md` for the complete physical-world, trajectory, runtime,
and diagnostic contract.

Known passage structures are not encoded into the 2D static obstacle map as hard
blocking cells. The 2D planner must be able to route through the footprint when
a valid 3D opening is annotated; RViz shows those volumes through
`/drone_city_nav/known_passage_markers` instead.

The validation layer does not reject trajectories by itself. The lidar
classifier does not detect passages, does not modify static map cells, and does
not create a trajectory-dependent working copy of lidar or memory data. It
suppresses only a new hit whose measured range confidently matches a known
physical solid. Closer hits, hits through the free opening, and boundary or
otherwise ambiguous hits remain dynamic obstacle evidence.

The default file format is line-based and versioned:

```text
drone_city_nav_known_passages_v1
frame_id map
structure <id> <center_x> <center_y> <size_x> <size_y> <z_min> <z_max>
opening <structure_id> <opening_id> <center_x> <center_y> <center_z> <normal_x> <normal_y> <width> <height> <depth> <min_z> <max_z> <approach_m> <exit_m>
```

The parser rejects unknown keywords, duplicate ids, invalid dimensions,
non-finite values, openings outside their structure footprint, and opening z
ranges outside the structure z range.

The validation layer reports a diagnostics contract for known passages. It is a
planner repair/telemetry signal, not a substitute for the ordinary physical
building collision volumes:

- no structure footprint intersection is valid;
- structure footprint intersection through a matching opening volume is valid;
- structure footprint intersection without a matching opening is reported as a
  violation with `structure_without_opening` or `opening_volume_miss`;
- `known_passage_validation_min_opening_overlap_m` controls the absolute
  minimum station overlap required to count an opening match.
- `known_passage_validation_min_opening_depth_fraction` requires the trajectory
  to cover a configured fraction of the opening depth before a span is treated
  as a confident opening match.
- `known_passage_validation_clearance_margin_m` rejects an opening match as
  `opening_volume_miss` when its lateral/vertical clearance is below this
  margin.
- `known_passage_validation_max_diagnostics` caps per-span JSON/log detail.
- `known_static_lidar_hit_closer_range_tolerance_m` bounds how much closer a
  hit may be before it is retained as an unknown object in front of the known
  solid.
- `known_static_lidar_hit_farther_range_tolerance_m` bounds a later return
  still treated as the known collision surface. It must have the same effective
  value in planner and obstacle-memory node configuration.
- `ground_lidar_rejection_enabled` enables the always-on, per-beam flat-ground
  provider. It does not suspend mapping based on vehicle tilt.
- `ground_lidar_altitude_m` is the map-frame Z of the expected flat ground. The
  generated city default is `0.05` m, matching the top of the physical ground
  collision box.
- `ground_lidar_closer_range_tolerance_m` is the strict allowance before the
  expected ground range. A hit closer by more than this value remains unknown
  obstacle evidence.
- `ground_lidar_farther_range_tolerance_m` is the bounded allowance behind the
  analytic ground intersection. A farther ground-facing return is ambiguous and
  performs no hit or free-space update.

All four ground parameters have identical defaults in `obstacle_memory_node`
and `planner_node`: enabled, ground Z `0.05` m, closer tolerance `0.5` m, and
farther tolerance `1.5` m. Headless validation requires the effective logged
configuration to match. A deliberately disabled provider is reported as
`disabled`; invalid numeric configuration is `unavailable` and does not disable
the independent known-static provider.
- `obstacle_memory_debug_publish_period_s` limits only the standalone raw-grid
  and provenance debug topics. `0` publishes them with every atomic update;
  the default `1.0 s` avoids tripling the large per-scan transport payload.
- `obstacle_memory_snapshot_diagnostic_period_s` controls producer and planner
  transport-budget summaries; per-publication/apply identity logs remain
  available for exact event correlation.
- `obstacle_memory_snapshot_max_serialized_bytes` and
  `obstacle_memory_snapshot_max_assembly_time_ms` are producer warning budgets
  for the complete atomic message and its construction cost.
- `obstacle_memory_snapshot_max_publish_interval_ms` warns when authoritative
  producer cadence falls below its operational budget.
- `obstacle_memory_snapshot_max_age_ms`,
  `obstacle_memory_snapshot_max_callback_time_ms`,
  `obstacle_memory_snapshot_max_apply_delay_ms`, and
  `obstacle_memory_snapshot_min_apply_rate_hz` are planner warning budgets for
  snapshot freshness when adopted, callback parsing, parsed-to-active delay,
  and effective adoption by the 0.5 s planning timer. The defaults are 350 ms
  apply age, 100 ms callback time, 300 ms apply delay, and 1.0 Hz apply rate.
  Apply rate is intentionally lower than producer cadence because planning
  adopts only the newest parsed snapshot, not every intermediate publication.
  These are diagnostic budgets, not rejection thresholds.
- `vertical_profile_preferred_gate_clearance_margin_m` keeps the selected gate
  altitude inside a preferred safe band when possible. It clamps to the nearest
  preferred boundary instead of forcing the opening center.
- `known_passage_traversal_speed_limit_mps` caps speed inside known-passage hard
  altitude windows.
- `passage_insertion_enabled` controls the optional local XY repair stage. It
  is enabled by default so annotated passages can locally align XY trajectory
  geometry before vertical profiling.
- `passage_insertion_sample_step_m` controls inserted segment sampling.
- `passage_insertion_min_anchor_margin_m` and
  `passage_insertion_max_anchor_margin_m` bound the stitch window around the
  missed opening span.
- `passage_insertion_opening_lateral_target_margin_m` keeps the repaired path
  away from the opening side edges.
- `passage_insertion_repair_clearance_margin_m` also repairs already-valid
  opening traversals when their lateral clearance is below the preferred margin.
- `passage_insertion_max_lateral_shift_m` rejects repairs that would require a
  large local shift.
- `passage_insertion_max_join_tangent_delta_deg` and
  `passage_insertion_max_join_curvature_jump_1pm` protect stitch continuity.
- `passage_insertion_min_inserted_radius_m` can require a minimum local radius;
  `0` disables this radius gate.
- `passage_insertion_max_candidates` and `passage_insertion_max_diagnostics`
  bound CPU work and log volume.

`mission_monitor_node` also reads `known_passages_enabled` and
`known_passages_path`. When enabled, it converts each annotated architectural
passage into ordinary solid building volumes around the opening. The opening
itself remains free space; mission failure still comes from collision with
solid volumes, not from a special "passed passage id" rule.

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
- `static_building_markers_topic`
- `known_passage_markers_topic`
- `known_passage_debug_publish_period_s`
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
