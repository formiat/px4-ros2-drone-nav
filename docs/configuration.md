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

Beam-adjacent surface reconstruction (`lidar_surface_interpolation_enabled`,
`lidar_surface_interpolation_maximum_incidence_deg`,
`lidar_surface_interpolation_maximum_row_incidence_deg`): two adjacent returns
of the organized scan whose range step fits a single surface viewed within the
incidence limit are joined by occupied surface samples spaced below one voxel
edge. A sparse beam layout maps a wall as rows and columns spaced by range
times beam spacing, so the band the vehicle body occupies between two rows, or
the strip between two columns of a wall seen at a grazing angle, would
otherwise stay unknown until the sensor is a few metres away. Vertical
neighbours use the tight limit and must join through a segment steeper than 30
degrees: a ceiling or floor in front of a wall answers with a much larger range
step than the wall itself, and joining those returns would draw a ramp through
free volume. Horizontal neighbours use the wide limit: a depth edge between two
columns steps by the whole gap behind it, far beyond any incidence. The samples
carry occupied evidence at their endpoint only and no free-space evidence. The
latest-lidar obstacle scan stays the measured returns alone.

## `production_mppi_node`

Execution cadence:

- `tick_rate_hz`, `rviz_rate_hz`, `diagnostics_info_rate_hz`;
- `diagnostics_file_rate_hz`, `diagnostics_flush_period_s`, and
  `diagnostics_error_ring_capacity`;
- `rollouts`, `dt_s`, mode-specific horizon duration;
- deadline and maximum input ages;
- `execution_horizon_acknowledgement_grace_ms` bounds how long a freshly
  published horizon waits for the offboard acknowledgement before a newer plan
  may supersede it. Supersession never waits for the resident horizon to
  expire: an acknowledged predecessor, a witnessed owner, or the elapsed grace
  each admit the replacement, and the deferral is counted in diagnostics.

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
  tick start. Stale or missing data does not create an obstacle; no-static
  execution instead fails closed and publishes no new motion from that evidence.

No-static 3D world:

- `raw_obstacle_snapshot_3d_topic` and `raw_obstacle_delta_3d_topic` define the
  revisioned observed-world transport;
- `obstacle_memory_3d_transport_rate_hz` bounds planner-world publication;
- `obstacle_memory_3d_snapshot_minimum_period_s`,
  `obstacle_memory_3d_snapshot_maximum_period_s`, and
  `obstacle_memory_3d_snapshot_rebase_dirty_ratio` control adaptive base
  rebasing without coupling it to lidar cadence;
- local ESDF half extent, recenter margin, and update rate bound the derived
  controller-distance resource. It is an exact capped dense Euclidean distance
  transform over the chunk-aligned local window; an unchanged occupied source
  reuses the resident field and GPU texture without recomputation;
- `world_worker_count` sizes the dedicated world-build pool, so a distance
  transform never shares planner threads with D* Lite continuations;
- the observed ESDF distance cap is derived from preferred clearance, the full
  oriented-footprint bounding radius, and conservative voxel-query correction;
- missing distance evidence, outside-cache volume, and unknown voxels are neutral
  to strategic traversability. Exact raw occupied evidence remains authoritative.

Mode policy:

- `use_static_map`;
- map-independent cruise and absolute speed;
- acceleration, lateral acceleration, braking, and jerk limits;
- the conservative terminal-path horizontal deceleration limit, independently
  of the larger acceleration available to ordinary manoeuvres;
- route lookahead and curvature preview;
- sensor-braking and goal limits.

The reference speed is also capped by the body clearance of the horizon the
vehicle is executing, with the tracking-error tube law the route
certification applies (`tracking_error_tube_response_time_s`,
`tracking_error_tube_minimum_progress_speed_mps`): the tracking error the
controller can accumulate within its response time must fit inside the
clearance to known occupied evidence (unknown space does not count), whatever
the route promised when it was certified. The minimum progress speed floors
that limit so a tight spot stays leavable; the body validation remains the
only hard authority.

`guaranteed_lidar_detection_range_m` and
`sensor_braking_physical_margin_m` define the physical sensor side of the
speed contract. The guaranteed range must not exceed either the modeled 3D
lidar range or the range admitted by obstacle memory. The organized scan spans
the complete vertical `[-90 deg, +90 deg]` interval so pure climb and descent
do not enter a polar blind cone. The speed policy adds
`latest_lidar_obstacle_maximum_age_ms` to `speed_reaction_latency_s`, then
requires

```text
speed * total_latency + jerk_limited_stopping_distance + physical_margin
  <= guaranteed_lidar_detection_range
```

The stopping term uses the weaker of the guaranteed horizontal and vertical
decelerations, the worst forward 3D acceleration derived from the configured
horizontal and vertical acceleration limits, and
`maximum_control_jerk_mps3`. The same jerk-limited stopping implementation is
used by route-reserve certification. The resulting speed is a hard norm limit
for the complete `(vx, vy, vz)` vector in CPU and CUDA dynamics and is also part
of strategic ETA and route time parameterization. A measured speed above the
contract requests braking instead of new motion. Free and unknown space use the
same limit; freshness changes admission, not occupancy semantics.

Risk:

- `critical_distance_m`;
- `preferred_distance_m`.

Sampler:

- `mppi_temperature` and `mppi_adaptive_temperature_cost_fraction` normalize the
  MPPI weighting temperature to the feasible-cost spread so the update does not
  collapse onto one rollout when costs are large;
- `mppi_body_collision_gate_enabled` optionally excludes rollouts whose body
  enters an occupied ESDF voxel from the weighted update while another
  rollout stays feasible. The device ESDF is coarser than the raw grid and can
  mark a raw-valid passage occupied, which leaves the update hovering in front
  of it, so the gate is off in the production profile; the raw swept footprint
  remains the only hard authority either way;
- `route_directed_candidate_cost_tolerance` accepts the deterministic
  route-directed candidate when its cost is within that fraction of the
  stochastic optimum, which keeps the warm start on the route instead of
  alternating between near-equal candidates.

Raw occupied cells or voxels are the only hard collision geometry. The distance
thresholds classify free space for risk ranking; they do not inflate raw
occupancy.

Persistent 3D planner and route lifecycle:

- persistent graph minimum horizontal/vertical step and maximum power-of-two
  adaptive level;
- per-update expansion, changed-voxel, extracted-path, shortcut, and compute
  budgets;
- connector radius and mission-goal tolerance;
- route sampling and completion tolerances;
- static derived-distance lookahead;
- raw validation sampling;
- remaining-distance successor trigger, certified overlap, measured-latency
  reserve, splice, and retry thresholds;
- cross-track and stall thresholds;
- velocity/previous-route heading cascade thresholds;
- speed-dependent tracking-error response horizon and
  `tracking_error_tube_minimum_progress_speed_mps`, the progress floor that a
  constrained segment keeps wherever the physical body clears raw occupancy;
- `overspeed_weight` charges every rollout state above the MPPI dynamics
  speed caps (the absolute limit horizontally, the sensor-braking limit
  translationally); a rollout that starts above a cap inherits its speed, and
  the term makes shedding the excess worth more than the progress it buys;
- `persistent_planner_clearance_ranking_weight` and
  `persistent_planner_clearance_ranking_distance_m` scale lattice edges near
  raw occupied evidence for ranking only; a low-clearance edge stays traversable
  whenever the raw swept body check accepts it;
  `persistent_planner_feasibility_clearance_ranking_distance_m` is the shorter
  reach within which the feasibility-first search derives the same ranking, so
  the first route already keeps its body out of the critical band;
  `persistent_planner_clearance_ranking_critical_weight` adds a steep band
  below the execution risk model's `critical_distance_m`, so the planner detours
  around a critical metre the way the executor's critical exposure cost would;
- `clearance_costs_enabled` and `static_route_geometry_optimization_enabled`
  are on in the production profile.

Static world:

- `static_occupancy_3d_path` selects the generated Occupancy3D artifact;
- `static_free_space_topology_3d_path` selects its fingerprint-bound derived
  FreeSpaceTopology3D index; an empty value derives `.topology3d` from the raw
  occupancy path;
- `static_esdf_3d_cache_path` selects its fingerprint-bound precomputed ESDF
  artifact; an empty value derives the `.esdf3d` path from Occupancy3D;
- Occupancy3D contains only raw physical occupancy and never embeds topology;
- the persistent planner, route compiler, and controller use the same configured
  horizontal/vertical dynamics and 3D execution-time model;
- static-route sparse-deviation, turn-increase, and validation-batch parameters
  control bounded farthest-first shortcut validation before final resampling;
- optional static topology supplies fingerprint-bound passage metadata for
  cooperative execution evidence; it is not a competing route producer;
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

- lidar evidence age, reaction latency, guaranteed 3D deceleration, worst
  forward acceleration, control jerk, detection range, and physical margin;
- finite-route reserve and mission-goal stopping margins;
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
`ENABLE_STATIC_MAP`, `LIDAR_PROFILE=none|3d`, `ENABLE_RVIZ`, and camera toggles
into launch arguments or temporary parameter overrides. No-static mode requires
the 3D profile and rejects `none`. All simulation entry points default to 3D.
Static maps are opt-in: `ENABLE_STATIC_MAP`
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
7. Tune liveness and route-recovery lifecycle only from observed failure cases.

Do not compensate for frame, collision, or stale-input failures by changing
soft MPPI weights.
