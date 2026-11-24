# Configuration Reference

The authoritative runtime defaults are in:

```text
drone_city_nav/config/urban_mvp.yaml
```

Node constructors declare and validate the same parameters. This document
describes ownership and tuning order instead of duplicating every numeric
default.

## `obstacle_memory_3d_node`

World inputs:

- `grid_*`, `initial_*`, `px4_local_origin_*`;
- `risk_critical_distance_m`, `risk_preferred_distance_m`.

`use_static_map` is declared on this node for launch compatibility, but it
does not load or merge static geometry. Static Occupancy3D belongs to
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
- snapshot/delta cadence and transport limits.

The node owns the organized beam geometry, full-6DoF acquisition-pose
alignment, sparse Occupancy3D bounds and chunk size, hit/miss integration,
snapshot/delta cadence, self-return filtering, and selected-spectator
current/accumulated point clouds.

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
latest-sensor obstacle scan stays the measured returns alone.

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

- `latest_sensor_obstacle_scan_topic` selects the timestamp-aligned raw-hit
  stream produced by obstacle memory;
- `latest_sensor_obstacle_maximum_age_ms` bounds how long that direct evidence
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
- acceleration, braking, and jerk limits, and `maximum_lateral_acceleration_mps2`,
  the acceleration a turn may demand for the curvature limiter and the route
  time profile — no more than the horizontal acceleration, since every
  published control is clamped to that magnitude as a vector and a turn shares
  it with braking; `route_curvature_measurement_window_m` is the length of
  route a corner's turn is read over, the scale of the tightest fillet;
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
only hard authority. Where that horizon enters space the lidar has not
observed, the sensor-braking contract below is read with the observed range
along the motion in place of `guaranteed_lidar_detection_range_m`, floored by
the same minimum progress speed.

`guaranteed_lidar_detection_range_m` and
`sensor_braking_physical_margin_m` define the physical sensor side of the
speed contract. The guaranteed range must not exceed either the modeled 3D
lidar range or the range admitted by obstacle memory. It is the most the
contract grants: since roadmap item 17 the range and the evidence age are read
from the sensor's latest frame, the range falling with the share of the frame
that observes and with its light headroom
([`camera_perception.md`](camera_perception.md), specification K9, K15). The organized scan spans
the complete vertical `[-90 deg, +90 deg]` interval so pure climb and descent
do not enter a polar blind cone, and its row and column spacing is bound to the
guaranteed range: at that range adjacent rows land no farther apart than the
vertical body band (`physical_footprint_lower_extent_m` plus
`physical_footprint_upper_extent_m`) plus one occupancy voxel, and adjacent
columns no farther apart than the body diameter plus one voxel. A wall crossing
the vehicle's path is therefore painted inside the swept footprint by measured
returns alone, whatever the roof or floor next to it does to the neighbouring
rows; the surface joins only densify that evidence, so the 3D profile leaves
`lidar_surface_interpolation_enabled` off: with a dense scan the joins add tens
of thousands of occupied voxels per scan between measured returns, every one
of which the planner has to repair around. The 3D profile samples 121 rows
(1.5 deg) and 240 columns (1.5 deg) for the 30 m guarantee with the 0.58 m body
band and 0.25 m voxels. With 10 deg rows the same wall was a single
row at sensor height until the vehicle was a few metres away: under a passage
roof the neighbouring row hits the ceiling instead of the wall, so no join can
fill the band, and the body band sailed through unknown space between two
rows. The speed policy adds
`latest_sensor_obstacle_maximum_age_ms` to `speed_reaction_latency_s`, then
requires

```text
speed * total_latency + jerk_limited_stopping_distance + physical_margin
  <= guaranteed_lidar_detection_range
```

The stopping term is assessed along the direction of motion: the guaranteed
deceleration and the acceleration the vehicle may still carry are the per-axis
limits (horizontal and vertical) divided by that direction's axis shares, with
`maximum_control_jerk_mps3`. Level flight is therefore bounded by the horizontal
axis alone and a climb by the weaker vertical deceleration; the weakest
deceleration of either axis used to be paired with the vector sum of both
accelerations for every direction, so raising the horizontal acceleration
lowered the speed the contract admitted in level flight. The reference speed
applies the vehicle's velocity direction (the route tangent while it stands);
the worst direction over all of them is the hard norm limit for the complete
`(vx, vy, vz)` vector in CPU and CUDA dynamics and is also part of strategic ETA
and route time parameterization. `maximum_yaw_rate_radps` and
`maximum_yaw_acceleration_radps2` bound the yaw the gaze may command, and
`gaze_survey_yaw_rate_radps` is the rate at which a climb or a descent with no
heading to face surveys the walls it passes (the stereo profile sets it, a
lidar profile leaves it at zero); the
time model's stationary turns and the angular-derivative contract read the
same two values, and the configured ones carry their measurement in
`urban_mvp.yaml`. The same jerk-limited stopping implementation
is used by route-reserve certification. A measured speed above the contract
requests braking instead of new motion. Free and unknown space use the same
limit; freshness changes admission, not occupancy semantics.

Risk:

- `critical_distance_m` and `preferred_distance_m` are risk-tier bands for
  route annotation and diagnostics; the preferred band is also the reach of
  the clearance preference. No law measures a margin from them: the clearance
  laws are the tracking-error tube beside the motion and the stopping
  capability ahead of it, so the reference speed, the rollout cost and the
  route certification cannot disagree about what "too close, too fast" means.

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
- `reference_tracking_lag_s` is the measured delay with which the vehicle
  follows a reference that falls, less the reaction latency; the distance
  laws of the speed policy that face evidence (clearance, route clearance,
  blocked route, route endpoint) charge it as latency, see
  [trajectory_optimization.md](trajectory_optimization.md);
- `overspeed_weight` charges every rollout state above the MPPI dynamics
  speed caps (the absolute limit horizontally, the sensor-braking limit
  translationally) and above the reference speed the speed policy derives
  from its stopping laws (a blocked route, a route endpoint, the goal, a
  turn, the sensor range); a rollout that starts above a cap inherits its
  speed, and the term makes shedding the excess worth more than the progress
  it buys, which the symmetric speed tracking weight alone cannot;
- `persistent_planner_clearance_ranking_weight` and
  `persistent_planner_clearance_ranking_distance_m` scale lattice edges near
  raw occupied evidence for ranking only; a low-clearance edge stays traversable
  whenever the raw swept body check accepts it;
  `persistent_planner_feasibility_clearance_ranking_distance_m` is the shorter
  reach within which the feasibility-first search derives the same ranking, so
  the first route already keeps its body clear of the walls; below cruise times
  `tracking_error_tube_response_time_s` the same ranking scales an edge's
  flight time by the ratio the tube law will impose on execution there, so a
  short tight passage and a longer open detour are compared by the time each
  will actually take — there is no separate weight for this;
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

- sensor evidence age, reaction latency, guaranteed 3D deceleration, worst
  forward acceleration, control jerk, detection range, and physical margin;
- finite-route reserve and mission-goal stopping margins;
- liveness observation window, the along-route displacement that counts as
  progress, the off-route displacement that counts as movement
  (`liveness_minimum_offroute_displacement_m`), and how many consecutive stalled
  windows a reseed needs (`liveness_stalled_windows_before_reseed`).

## `mppi_offboard_node`

- execution-horizon and PX4 topics;
- maximum receive age and control lookahead;
- finite-path receive age, deadline, and control lookahead;
- takeoff climb above the spawn point and hover time;
- `unavailable_path_braking_acceleration_mps2`: the deceleration the local hold
  brakes a still-moving vehicle with when nothing owns its motion; the launch
  passes `maximum_horizontal_acceleration_mps2` through to it so the hold
  brakes as hard as the planner plans against and PX4 is configured to;
- `unavailable_path_braking_response_s`: the time that hold brings a residual
  velocity to zero over; the braking is full above response times acceleration
  and proportional below it, so a vehicle that has all but stopped is settled
  rather than thrown the other way;
- the same minimum/maximum target altitude contract as the planner;
- expected vehicle role, mission epoch, and destruction topic;
- bounded death force-disarm retry period;
- arm/offboard resend policy;
- map origin;
- `px4_visual_odometry_topic`: the external odometry the autopilot is given,
  read for its declared dead reckoning; after 3 s of it the node descends the
  vehicle level at 0.5 m/s (specification K19);
- RViz drone marker and follow TF.

The yaw setpoint is none: the node hands the autopilot the yaw rate the
horizon plans `yaw_rate_lead_s` ahead (0.26 s, specification K17). The lead
is the lag of the autopilot's yaw rate loop on this airframe and must be
fitted again for another vehicle or other yaw gains.

## Other Nodes

`world_visualization_node` owns static and raw world debug topics.
`static_map_visualization_stride_cells` controls only the density of the RViz
static point cloud. It does not change Occupancy3D or ESDF3D resolution.
`route_constraint_diagnostics_distance_m` controls how far before entry and
after exit constrained-route lifecycle diagnostics report approach/departure.
It is observational and does not alter planning or speed policy.
`mission_monitor_node` owns mission success and crash metrics, and the return
home of roadmap item 19: `unreachable_goal_proof_voxel_budget` bounds the
flood of the memory that proves a goal unreachable, beyond which the proof
stays undecided, and `minimum_target_z_m` and `maximum_target_z_m`, the
planner's envelope, bound it in height (specification K16).
It gives the goal up as well for the carried light's battery and for a light
judged unreliable (specification K11, K12; the constants are in
`carried_light_judgment.hpp`): by the light driver's reading of its output
(`carried_light_output_topic`), by the time-of-flight surfaces the frame
shows dark (`unlit_surface_share_topic`, published by `stereo_depth_node`)
and, with no reading, by the frames alone. On the way home it hands the planner
the trail it flew, point by point (`return_trail.hpp`, K20).
`visual_inertial_odometry_node` fuses the rotors' drag: `rotor_drag_1ps`
(0.106, a parameter) is this airframe's and must be fitted again for another
vehicle, as the accelerometer's reading across the rotor axis against the
true body velocity on recorded flights
([`localization.md`](localization.md), K21). A frame under 20 features is
no aid (K13, a constant of the estimator's configuration structure).

## The Numbers The World's Name Chooses

The inventory of roadmap item 45, taken on 2026-11-23 at `0ba4fbbc`: every
number of the flight that depends on the world's `flight_profile` (`cave`,
the default, or `open_sky`; `drone_city_nav/worlds/*.world3d.json`, read by
`launch/point_to_point_scenario.py:190` and applied by
`launch/city_nav.launch.py:314` to `:643` from the dictionaries of
`launch/sensor_profile.py:116` to `:171`) or on the Makefile's open-sky
targets. The vehicle is one and nobody switches a mode of the environment
(the owner, 2026-11-17), so each number becomes one of three things: (a)
one number for every world; (b) a number chosen in flight by what the
stack measures; (c) a property of the airframe or of the host, one for
every world. The item's stages replace the rows one by one; this table is
their plan and their evidence.

| Number | Under a roof | On the open sky | Who reads it | Class | Evidence, and for (b) the measured quantity |
|---|---|---|---|---|---|
| `cruise_speed_mps`, `absolute_speed_limit_mps` | 6.5 and 10 (`config/urban_mvp.yaml:378`, `:379`, every Makefile target) | 15 and 15 (the open-sky targets) | planner, PX4 | (a) 15 and 15, in the configuration alone since stage 2; the stereo set's planner keeps 6.5 and 10 (below the table) | The owner's word of 2026-11-20. Under 15 the caves flew with no contact (r4042 to r4048, r4106 to r4109): the cruise had bound 4 to 7 percent of the lidar's ticks and none of the cameras', the sensor's braking contract binds in its place. |
| `maximum_horizontal_acceleration_mps2` | 4 (`:390`, every target) | 5 (the open-sky targets) | planner, offboard, PX4 | (c) 5, since stage 2; the stereo set's planner keeps 4 (the sensor set's, see below the table) | The airframe brakes at 6.4 to 7.5 m/s² (`tools/airframe_envelope.sh`). 5 flew worse than 4 on Urban in 2026-09 (r224, r225) and does not today: r4106 and r4107 2.830 and 2.853 m/s against 2.26 to 2.55 at 4, the true clearance and the stops within the base's spread; the cave 1.978 (r4108), the stereo set 1.316 (r4109). |
| `maximum_vertical_acceleration_mps2` | 1.4 (`:576`) | 3.0 (`OPEN_SKY_DYNAMICS`) | planner | (c) 3.0, since stage 2; the stereo set keeps 1.4 | The caves' 1.4 kept a dive within its arrest before the route's end bounded a descent by where it stops (K78); the stopping contract keeps its own 1.4 on every world. To be flown by the item's ladder. |
| `maximum_control_jerk_mps3` | 12 (`:419`) | 18 (`OPEN_SKY_DYNAMICS`) | planner, PX4 | (c) 18, since stage 2; the stereo set keeps 12 | The airframe's onset of braking, 23 to 31 m/s³ (specification V3). To be flown by the ladder. |
| `takeoff_hover_s` | 2.0 (`:1055`) | 0.5 (`OPEN_SKY_TAKEOFF`) | offboard | (a) 0.5, since stage 3 | The number is the mission's, not the space's; flown by both sensor sets. |
| `guaranteed_lidar_detection_range_m` with `guaranteed_obstacle_size_m` | 14 m and the 0.25 m voxel (`:483`; no size named) | 35 m and 0.75 m (`OPEN_SKY_LIDAR_OBSERVABILITY`) | planner (the braking contract, the cone of clear sight), memory (how far a return is placed: 19.2 m against 35) | (b); the memory since stage 3 (K88) | A promise of safety (K75): obstacles of 0.75 m are promised to the lidar's whole range because what is thinner is passed over (K76), which a roof forbids. The quantity: **the sky is open over the vehicle**, no beam of the newest scan within a cone of 10 degrees about the vertical returned (K88). The memory node holds every beam of a scan, returned or not; the planner would read the answer with the scan (`LatestSensorObstacleScan`). Measured by the truth below. |
| `latest_sensor_obstacle_maximum_age_ms` | 600 (`:359`) | 300 on the lidar | planner | the sensor set's: 300 on the lidar, 600 on the pair; since stage 3 | The scan's age at a tick (`sensor_evidence_age_s`): the lidar 0.124 s at p50 and 0.192 at most on Urban (r4037), 0.128 and 0.208 in the cave (r3955), 0.112 and 0.180 on the open sky (r3946), no tick over 0.3 s anywhere; the pair 0.230 and 0.414, 3.07 percent of its ticks over 0.3 s (r4033). |
| `no_static_3d_esdf_horizontal_recenter_margin_m` | 12 (`:343`) | 17 on the lidar | planner | (a) 17 on the lidar set, since stage 3 | The window recentred 17 m ahead is the stricter; what it costs the caves' tick, by a flight. |
| `persistent_planner_overflight_margin_m` | 0 (the code's default) | 5 on the lidar | planner | (b) | The leg over everything seen is taken where the body's sweep clears it on the memory (`persistent_dstar_lite_planner_3d_overflight.cpp`), and what the memory has not seen is free: with the margin set under a roof, 95 of the planner's 622 updates in the replay of r2918 were that leg. A roof does not refuse it by itself; the leg has to ask for a sky the scan saw open (the quantity below). On the open sky all 35 routes the planner published were that leg (r3946). |
| `grid_depth_m` | 40 (`:28`) | 56 (`OPEN_SKY_MEMORY`) | memory | the world's data, since stage 3 | The launch stands the memory's box a metre over the world's ceiling where the configuration's 40 m does not reach it; the ceiling (39 and 55 m) stays in the world's file. One box of 56 m for every world would have moved the frames the world tools cut by the box (`tools/heldout_scenario.py`). |
| `integration_threads` | 1 (the code's default) | 4 | memory | (c) 4, since stage 3 | The host's. The threads' evidence is joined by OR before a voxel changes, so the memory is the one a single thread makes (K75). |
| `obstacle_memory_3d_transport_rate_hz` | 10 (`:196`; r346 to r351) | 5 | memory | (a) 10, since stage 3 | The profile's comment and K75 call the caves' rate 2 Hz; 2 is the code's default and the configuration has carried 10 since `4df57c69`: the open sky's 5 halves it (the logs of r4037 and r3946 state 10.00 and 5.00 Hz). By an open-sky flight. |
| `latest_scan_clear_sight_half_angle_rad` | 0, no cone (the code's default) | 0.105 | memory, then planner | (a) 0.105 | The cone is the sensor's own contract read on the newest frame (K85) and reaches no farther than the promise above. By flights. |
| the stereo pair's shadow memory beside a navigating lidar | on | off (`city_nav.launch.py:643`) | launch | gone, since stage 3 | Nothing consumed it; it took a core the planner wanted (r3282). |

The stereo set's planner keeps the profile it has flown (cruise 6.5 m/s,
limit 10, 4 m/s², 1.4 m/s² and 12 m/s³, `STEREO_TOF_PROFILE`) on every world:
a sensor set's numbers, not a space's. Its speed is its sensor's, 2.6 m/s at
the pair's 5.2 m, and the ceiling is to it a scale the planner builds routes
at. Under 15 m/s its true way to Urban's goal was under 510 m in none of
thirteen flights (511 to 639 m) against eleven of thirteen under 6.5 and 10
(442 to 510 m), a mission took 380 s at the median against 339, the estimate
ended 1.56 m off at the mean against 0.86, and three flights acknowledged the
goal over 2 m away against one (specification V3's change log names the
runs). Setting the accelerations back alone did not shorten the way, nor did
PX4's parameters; which of the planner's uses of the ceiling lengthens it is
not found. The lidar set flies the airframe's numbers under a roof at
the base's speed (2.342 m/s against 2.334); with the caves' former vertical
acceleration and jerk beside the acceleration 5 two flights were 0.5 m/s
faster (r4106, r4107), which the open sky does not admit.

The mission check reads the same field to choose the norm of the mean
speed (`scripts/validate_drone_nav_headless.py:421`,
`scripts/headless_runtime_evidence.py:399`): R8 on the open sky, R3
elsewhere. A check may know its scenario; it is not flight code.

**Is the sky open over the vehicle: does it tell the worlds apart?**
Measured by the truth on the kept flights (`tools/sky_above.py`: rays of a
cone about the vertical, from the sensor's place to the lidar's 35 m,
through the world's truth voxelization, every 0.1 s from the climb off the
pad):

| Flight | Samples | Open, cone of 10° | 20° | 30° | 45° | 60° |
|---|---|---|---|---|---|---|
| Urban, lidar (r4037) | 1779 | 0 | 0 | 0 | 0 | 0 |
| Urban, cameras (r4033) | 3165 | 0 | 0 | 0 | 0 | 0 |
| Cave, lidar (r3955) | 4363 | 0 | 0 | 0 | 0 | 0 |
| Cave, lidar (r4045) | 5153 | 0 | 0 | 0 | 0 | 0 |
| Open sky, full set (r3946) | 1673 | 100 % | 97.91 % | 97.19 % | 96.47 % | 88.88 % |

Under a roof no sample of 14 460 reads open, at any cone: the rule would
never give a cave the open sky's numbers. On the open sky a cone of 10
degrees is open throughout, and wider cones meet the park's crowns for 3.5
to 12.3 s at a stretch (one change of the answer at 20 and 30 degrees, five
at 45 and 60), where the rule gives the caves' numbers, the careful side.

The cone is 10 degrees: the crowns the wider cones meet stand over the
takeoff (the first 1.6 to 4.7 s of the flight), where the first leg over the
seen is taken, and a rule that read a roof there would have sent the open
sky's first leg to the searches.

## Environment Overrides

Simulation scripts translate environment variables such as
`ENABLE_STATIC_MAP`, `ENABLE_RVIZ`, and camera toggles into launch arguments
or temporary parameter overrides. The 3D lidar is the one lidar profile
(`LIDAR_PROFILE=3d`); a static-map flight needs `STATIC_OCCUPANCY_3D_PATH`
and refuses to start without it. `MEMORY_PRIOR_OCCUPANCY_3D_PATH` names a
survey's map the obstacle memory starts from (specification K71; the node's
`prior_occupancy_3d_path`).
`CAMERA_PROFILE=none|stereo_tof` and `NAVIGATION_SENSOR_PROFILE=lidar|stereo_tof`
select the sensor set and what navigates on it; both default to `stereo_tof`
everywhere (`launch/sensor_profile.py` holds the defaults, what the set
guarantees to see and the vision memory's overrides, and both launches overlay
them on the YAML, whose `production_mppi_node` and `obstacle_memory_3d_node`
sections stay the lidar's). The final-revalidation evidence parameters are
`latest_sensor_*` (formerly `latest_lidar_*`).
`MISSION_GOALS_XYZ_M` replaces the point-to-point goal sequence;
`MISSION_GOAL_UNREACHABLE=true` records in the manifest that the goal was
placed out of reach on purpose, so that the check counts a return home as the
outcome asked for, and `TRUTH_OCCUPANCY_3D_PATH` names the truth grid the check
floods to confirm it ([scenarios.md](scenarios.md)).
`REAL_TIME_FACTOR` (1.0 unless set, above 0 and at most 1) slows the whole
flight against the wall clock: the preparation writes it into the worlds'
physics, the autopilot follows in lockstep, and the onboard loop keeps the
simulation clock, so every period and freshness watchdog slows with it
(specification K24, K25). The manifest records it, the host's verdict is
read against it, and `SMOKE_DURATION_S` is seconds of simulation: the runner
waits that many divided by the factor on the wall clock. The waits of the
launch itself (`STARTUP_SLEEP_S`, `PX4_PARAM_DELAY_S`, the waits for the
world and the GUI) stay wall-clock seconds: they wait for processes to
start, which the factor does not slow. A slowed flight flies with a faster
computer than the vehicle has, so a change is accepted at 1.0 only
([testing.md](testing.md)).
The flights of roadmap item 17 add their own, all recorded in the manifest:
`BLANK_PANELS="x,y,z,yaw,width,height;..."` (uniform matte panels), `LIGHT_FAULTS=none|moderate|severe|lost` with
`LIGHT_FAULT_SEED` (the carried light's injected failures; moderate is the
default), `LIGHT_BATTERY_S` (seconds of light at launch, only in the scenarios that test a limited charge: 720 in the return-home scenario, 240 in the low-battery one; unset, the light never runs out and the vehicle never turns for it),
`STREAM_FAULTS=none|moderate` with
`STREAM_FAULT_SEED` (frames dropped or delayed by a relay the camera driver
then reads), `SMOKE_ZONES` with `SMOKE_SEED` (boxes of smoke every sensor is
read through, roadmap item 18 stage 1, specification A13), `SMOKE_FACE=true` (the stereo set's smoke face detector on, K44; off by default since 2026-10-24), and the four that tell the mission check what a named
scenario asks for, `RETURN_HOME_EXPECTED`, `RETURN_HOME_ALLOWED`,
`RETURN_TRIGGERS_ALLOWED` and `VEHICLE_INTACT_EXPECTED`
([testing.md](testing.md)). None of them reaches
a production node: the light and its failures and the smoke live in the
simulator, and the vehicle reads its frames and its battery's charge.
The location has no light of its own in any flight: the lit world is no
longer selectable (the owner's rule of 2026-10-03, specification A10).
The memory's decay is a parameter of `obstacle_memory_3d_node`,
`decay_seconds_per_confirmation`: 0 in `urban_mvp.yaml` since 2026-10-03,
which keeps every occupied voxel for the flight; the decay by time stays in
the code switched off and is not to be used again (specification K10,
roadmap item 21).
Static maps are opt-in: `ENABLE_STATIC_MAP`
defaults to `false`, and a static run requires `ENABLE_STATIC_MAP=true`. No
separate boolean lidar flags are supported.
Multi-vehicle spectator selection additionally uses
`MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID` and
`MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY=first_living|next_living`. The launch
file and `scripts/run_drone_nav_sim.sh` are the source of truth for supported
overrides.

## Tuning Order

1. Verify frame transforms and raw obstacle evidence.
2. Verify mode-specific PX4 limits match MPPI limits.
3. Verify physical finite-path validation, route-unavailable hold, and terminal
   path hold behavior.
4. Tune reference speed and lookahead.
5. Tune `clearance_preference_weight` and `obstacle_approach_weight`. The
   first is a bounded soft cost that grows with depth into the preferred band,
   charged per second; the second prices the tracking-error tube law beside
   the motion and the stopping law ahead of it, the laws the route
   certification and the speed policy's `clearance` limiter apply, each read
   from that one configuration. Neither may be used as a reachability or hold
   threshold. Both are monotone in clearance and neither prices motion as
   such: there is deliberately no charge per metre *travelled* inside a band,
   because a corridor narrower than the band would then price every metre
   through it far above the progress it earns, and the weighted update
   converges on standing still; and no braking distance is charged to a wall
   beside the vehicle, because it never gets nearer.
6. Tune `altitude_tracking_weight`: it is the only term holding the vertical
   channel, since no progress term rewards vertical motion and speed tracking
   follows the route tangent.
7. Tune smoothness and control costs.
8. Tune liveness and route-recovery lifecycle only from observed failure cases.

Do not compensate for frame, collision, or stale-input failures by changing
soft MPPI weights.
