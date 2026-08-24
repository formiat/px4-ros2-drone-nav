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

- `grid_*`, `initial_*`, `px4_local_origin_*`;
- `risk_critical_distance_m`, `risk_preferred_distance_m`.

`use_static_map` is still declared on this node for launch compatibility, but
it does not load or merge static geometry. Static Occupancy3D belongs to
`production_mppi_node` and `world_visualization_node`.

Lidar projection:

- pose and heading topics;
- heading variance, consecutive stable-sample count, and maximum sample-delta
  gates;
- scan latency and motion compensation;
- lidar mount translation and quaternion;
- accepted projected-altitude and range bounds.

Memory:

- hit/miss weights and score thresholds;
- scan stride;
- debug/snapshot publication periods;
- provenance transport limits.

`obstacle_memory_3d_node` owns the 3D-profile equivalents: organized beam
geometry, full-6DoF acquisition-pose alignment, sparse Occupancy3D bounds and
chunk size, hit/miss integration, snapshot/delta cadence, self-return filtering,
and selected-spectator current/accumulated point clouds.

## `production_mppi_node`

Execution cadence:

- `tick_rate_hz`, `rviz_rate_hz`, `diagnostics_info_rate_hz`;
- `diagnostics_file_rate_hz`, `diagnostics_flush_period_s`, and
  `diagnostics_error_ring_capacity`;
- `rollouts`, `dt_s`, mode-specific horizon duration;
- deadline and maximum input ages.

No-static direct raw validation:

- `latest_lidar_obstacle_scan_topic` selects the timestamp-aligned raw-hit
  stream produced by obstacle memory;
- `latest_lidar_obstacle_maximum_age_ms` bounds how long that direct evidence
  participates in complete finite-path validation. Both acquisition and local
  receipt must remain within this budget when the producer clock is not ahead.
  An acquisition timestamp ahead of the consumer's simulated clock uses local
  receipt age as the freshness authority so executor load does not discard
  current evidence while `/clock` delivery catches up. Receipt age is evaluated
  after selecting the atomic latest scan rather than against the older planning
  tick start. Stale or missing data does not create an obstacle.

No-static 3D world:

- `raw_obstacle_snapshot_3d_topic` and `raw_obstacle_delta_3d_topic` define the
  revisioned observed-world transport;
- `obstacle_memory_3d_transport_rate_hz` bounds planner-world publication;
- `obstacle_memory_3d_snapshot_minimum_period_s`,
  `obstacle_memory_3d_snapshot_maximum_period_s`, and
  `obstacle_memory_3d_snapshot_rebase_dirty_ratio` control adaptive base
  rebasing without coupling it to lidar cadence;
- local ESDF half extent, recenter margin, and update rate bound the dense GPU
  resource built from sparse observed occupancy;
- outside-ROI and unknown voxels remain unavailable for execution rather than
  becoming occupied.

Mode policy:

- `use_static_map`;
- map-independent cruise and absolute speed;
- acceleration, lateral acceleration, braking, and jerk limits;
- the conservative terminal-path horizontal deceleration limit, independently
  of the larger acceleration available to ordinary manoeuvres;
- mode-specific lookahead and curvature preview;
- mode-specific observation and goal limits.

Risk:

- `critical_distance_m`;
- `preferred_distance_m`.

Raw occupied cells or voxels are the only hard collision geometry. The distance
thresholds classify free space for risk ranking; they do not inflate raw
occupancy.

Global guide:

- heading bins and primitive length;
- static/no-static lattice window and expansion limits;
- frontier endpoint displacement, continuation depth, candidate count, and the
  soft goal-distance ranking weight;
- validation sampling;
- remaining-distance replacement thresholds;
- cross-track and stall thresholds;
- velocity/previous-guide heading cascade thresholds.

Static world:

- `static_occupancy_3d_path` selects the generated Occupancy3D artifact;
- `static_free_space_topology_3d_path` selects its fingerprint-bound derived
  FreeSpaceTopology3D index; an empty value derives `.topology3d` from the raw
  occupancy path;
- `static_esdf_3d_cache_path` selects its fingerprint-bound precomputed ESDF
  artifact; an empty value derives the `.esdf3d` path from Occupancy3D;
- Occupancy3D contains only raw physical occupancy and never embeds topology;
- `global_lattice_3d_nominal_vertical_speed_mps` participates in physical travel
  time estimation;
- `global_lattice_3d_vertical_alignment_cost_weight` is the additional vertical
  travel-time preference; it ranks a level route ahead of an otherwise equal
  climb without making a required certified vertical passage unreachable;
- `global_lattice_3d_route_shape_vertical_turn_cost_per_rad` charges changes in
  flight-path angle, including climb/descent reversals, while the horizontal
  turn parameter continues to charge yaw-plane shape changes;
- static-route sparse-deviation, turn-increase, and validation-batch parameters
  control bounded farthest-first shortcut validation before final resampling;
- planning/critical exposure and turn-cost parameters rank complete route
  candidates with finite costs;
- `global_lattice_3d_passage_connection_distance_m` connects ordinary lattice
  states to derived portal entries; root topology candidates also evaluate a
  collision-free direct connection from the current start to each external
  portal plane;
- frontier continuation-depth parameters reject short endpoints that have no
  locally reachable continuation;
- route-envelope parameters control typed constrained-span execution data;
- constrained-span speed is encoded in 3D route samples.
- `minimum_target_z_m` and `maximum_target_z_m` define the half-open flight
  envelope used by objectives, 3D successors, passage edges, smoothed routes,
  activation, dynamic vertical stopping validation, and offboard publication;
- `physical_footprint_radius_m`, `physical_footprint_lower_extent_m`, and
  `physical_footprint_upper_extent_m` define the actual oriented drone volume;
  radial, axial, and swept sampling parameters control raw-occupancy validation.

Constrained route execution uses 3D route station. Vertical alignment begins at
a distance derived from measured `z`/`vz` and configured vertical dynamics. XY
hold is reserved for the final configured distance before the entry plane when
the measured altitude has not reached the retained capture window.

Speed policy and liveness:

- reaction latency and available stopping acceleration;
- unresolved-frontier and mission-goal stopping margins;
- actual-displacement and predicted-progress thresholds.

## `mppi_offboard_node`

- execution-horizon and PX4 topics;
- maximum receive age and control lookahead;
- finite-path receive age, deadline, and control lookahead;
- takeoff altitude and hover time;
- the same minimum/maximum target altitude contract as the planner;
- expected vehicle role, mission epoch, and destruction topic;
- bounded death force-disarm retry period;
- arm/offboard resend policy;
- map origin;
- RViz drone marker and follow TF.

## Other Nodes

`world_visualization_node` owns static and raw world debug topics.
`static_map_visualization_stride_cells` controls only the density of the RViz
static point cloud. It does not change Occupancy3D or ESDF3D resolution.
`route_constraint_diagnostics_distance_m` controls how far before entry and
after exit constrained-route lifecycle diagnostics report approach/departure.
It is observational and does not alter planning or speed policy.
`mission_monitor_node` owns mission success and crash metrics.
`lidar_debug_node` owns snapshot cadence, projection diagnostics, and point
cloud topics.

## Environment Overrides

Simulation scripts translate environment variables such as
`ENABLE_STATIC_MAP`, `LIDAR_PROFILE=none|2d|3d`, `ENABLE_RVIZ`, and camera
toggles into launch arguments or temporary parameter overrides. No-static mode
rejects the `none` profile; 2D and 3D are alternatives rather than an implicit
fusion mode. All simulation entry points default to 3D; 2D requires an explicit
`LIDAR_PROFILE=2d` selection. Static maps are opt-in: `ENABLE_STATIC_MAP`
defaults to `false`, and a static run requires `ENABLE_STATIC_MAP=true`. No
separate boolean lidar flags are supported.
Intercept spectator selection additionally uses
`INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID` and
`INTERCEPT_SPECTATOR_RESELECTION_POLICY=first_living|next_living`. The launch
file and `scripts/run_drone_nav_sim.sh` are the source of truth for supported
overrides.

## Tuning Order

1. Verify frame transforms and raw obstacle evidence.
2. Verify mode-specific PX4 limits match MPPI limits.
3. Verify physical finite-path validation, route-unavailable hold, and terminal
   path hold behavior.
4. Tune reference speed and lookahead.
5. Tune risk-band exposure and `critical_clearance_proximity_weight`. The latter
   is a bounded soft cost inside the critical band; it must not be used as a
   reachability or hold threshold.
6. Tune smoothness and control costs.
7. Tune liveness and guide lifecycle only from observed failure cases.

Do not compensate for frame, collision, or stale-input failures by changing
soft MPPI weights.
