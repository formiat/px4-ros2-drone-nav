# Configuration Reference

The authoritative runtime defaults are in:

```text
drone_city_nav/config/urban_mvp.yaml
```

Node constructors declare and validate the same parameters. This document
describes ownership and tuning order instead of duplicating every numeric
default.

## `obstacle_memory_node`

World inputs:

- `use_static_map`, `static_map_path`;
- `grid_*`, `initial_*`, `px4_local_origin_*`;
- `risk_critical_distance_m`, `risk_preferred_distance_m`.

Lidar projection:

- pose and heading topics;
- heading variance and startup alignment gates;
- scan latency and motion compensation;
- lidar mount translation and quaternion;
- accepted projected-altitude and range bounds.

Memory:

- hit/miss weights and score thresholds;
- scan stride;
- debug/snapshot publication periods;
- provenance transport limits.

Known-static filtering:

- `known_passages_enabled`, `known_passages_path`;
- `known_static_lidar_hit_classifier_enabled`;
- endpoint/range tolerances;
- ambiguous-evidence confirmation and retention.

The classifier is disabled by default. Enabling it changes how new lidar
evidence enters memory; it does not change static-map cells.

## `production_mppi_node`

Execution cadence:

- `tick_rate_hz`, `rviz_rate_hz`, `diagnostics_info_rate_hz`;
- `rollouts`, `dt_s`, mode-specific horizon duration;
- deadline and maximum input ages.

Mode policy:

- `use_static_map`;
- static/no-static cruise and absolute speed;
- acceleration, lateral acceleration, braking, and jerk limits;
- mode-specific lookahead and curvature preview;
- mode-specific observation, goal, and passage limits.

Risk:

- `raw_collision_radius_m`;
- `critical_distance_m`;
- `preferred_distance_m`.

Global guide:

- heading bins and primitive length;
- static/no-static lattice window and expansion limits;
- validation sampling;
- remaining-distance replacement thresholds;
- cross-track and stall thresholds;
- velocity/previous-guide heading cascade thresholds.

Passages:

- selection distance, lateral margin, and normal alignment;
- vertical clearance and capture hysteresis;
- capture and retention cycles;
- lateral staging and approach speed;
- stationary-trigger and dynamics estimates.

Safety and liveness:

- reaction latency and braking acceleration;
- fallback duration;
- time-to-collision threshold;
- actual-displacement and predicted-progress thresholds.

## `mppi_offboard_node`

- execution-horizon and PX4 topics;
- maximum receive age and control lookahead;
- fallback braking acceleration;
- takeoff altitude and hover time;
- arm/offboard resend policy;
- map origin;
- RViz drone marker and follow TF.

## Other Nodes

`world_visualization_node` owns world and passage debug topics.
`mission_monitor_node` owns mission success, crash, and actual passage metrics.
`lidar_debug_node` owns snapshot cadence, projection diagnostics, and point
cloud topics.

## Environment Overrides

Simulation scripts translate environment variables such as
`ENABLE_STATIC_MAP`, `ENABLE_RVIZ`, and camera toggles into launch arguments or
temporary parameter overrides. The launch file and `scripts/run_drone_nav_sim.sh`
are the source of truth for supported overrides.

## Tuning Order

1. Verify frame transforms and raw obstacle evidence.
2. Verify mode-specific PX4 limits match MPPI limits.
3. Verify horizon safety and braking behavior.
4. Tune reference speed and lookahead.
5. Tune risk-band exposure.
6. Tune smoothness and control costs.
7. Tune liveness and guide lifecycle only from observed failure cases.

Do not compensate for frame, collision, or stale-input failures by changing
soft MPPI weights.
