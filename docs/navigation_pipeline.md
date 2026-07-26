# Navigation Pipeline

The navigation pipeline turns raw obstacle evidence and a mission goal into an
accepted executable trajectory.

## 1. Raw Obstacle Sources

The planner can use:

- static map cells from `generated_city.map2d`;
- accumulated obstacle memory from the atomic
  `/drone_city_nav/obstacle_memory_snapshot` runtime artifact;
- current lidar scan overlay from `/scan`.

The planner can also load known 3D passage annotations from
`known_passages.passages3d`. These annotations are not raw obstacle sources and
do not make the XY planner search through buildings. After a final trajectory is
built, the planner validates whether it crosses any known structure footprint
through an allowed opening volume and reports the result in logs and trajectory
diagnostics.

See `known_passages.md` for the physical-world, lidar, local XY repair, and
vertical-profile contracts for these annotations.

Known passages can also provide physical-solid geometry to the optional 3D
lidar fallback classifier. It is disabled by default. When enabled, the
classifier combines the range match with signed 3D endpoint distance before
current lidar or accumulated memory records a new hit. Hits attached to a known
mass are suppressed or held as non-mutating pending evidence; hits clearly
detached in front of a mass and hits inside a free opening are retained
immediately. The classifier is independent of trajectory/proximity, never
filters static-map cells, and does not alter A* route preferences.

Raw sources are merged before risk evaluation. Raw sources contain only direct
obstacle evidence.

## 2. Raw Occupancy And Risk Field

The builder creates one raw occupancy snapshot and one occupied-distance field.
Current risk boundaries are:

```yaml
inflation_radius_m: 1.0
planning_clearance_m: 3.0
```

Raw occupied cells and local ROI bounds are hard rejects. Distances below
`inflation_radius_m` form the critical tier; distances below
`inflation_radius_m + planning_clearance_m` form the planning tier. A shared
lexicographic comparator prefers lower tiers and shorter tier exposure before
algorithm-specific geometry cost.

For a confirmed safe-truncation replan, one immutable raw/risk snapshot is
shared by all partial and full race jobs. Local repair replaces
only the geometry between the confirmed truncation point and a reconnect
station after the blocked span; the old suffix geometry remains unchanged.
Vertical, passage, and speed metadata are rebuilt for the complete stitched
trajectory before publication.

The authoritative runtime output is:

```text
/drone_city_nav/raw_obstacle_snapshot
```

`/drone_city_nav/raw_obstacle_grid` mirrors only raw cells for RViz.

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

The corridor builder samples the route and finds lateral bounds to raw occupied
cells. It uses the same immutable raw/risk context as route generation and can
reuse occupied-clearance fields and previous samples when safe.

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

## 9. Optional Local Passage Insertion

Local passage insertion is an optional repair stage for known 3D passages. It
runs after optimizer, turn smoothing, and isolated geometry cleanup, but before
vertical profile and speed profile construction.

The stage is enabled by default. It targets known-passage spans where the
current XY trajectory intersects a known structure but misses the opening
corridor, and can also repair a matched traversal with less than the configured
preferred lateral margin. It builds a local smooth XY segment through the
opening, stitches it back into the original trajectory, recomputes
sample stationing/tangent/curvature, and accepts the result only if:

- the full stitched trajectory avoids raw occupied cells and the ROI boundary;
- mission start and goal endpoints stay anchored;
- join tangent and curvature jumps stay within configured limits;
- the known-passage XY match improves.

This stage must not optimize for shortest or fastest path. Its purpose is to
repair opening alignment while preserving smoothness and safety.

## 10. Speed Profile Construction

A speed profile is built for trajectory samples using curvature, acceleration,
deceleration, minimum turn speed, and lookahead policy. Planner-side speed
profile diagnostics are planning diagnostics; offboard rebuilds the runtime
profile for actual control.

Trajectory sample length, curvature, projection, and speed profile are still
computed in XY. Each executable sample also carries `z_m`. Before takeoff this
altitude is seeded from `initial_altitude_m`; after takeoff it is seeded from the
current vehicle altitude and can be modified by known-passage vertical
profiling. The same samples are used by ROS path publication, RViz, dumps,
diagnostics, and runtime vertical maneuver control.

## 11. Publication

The planner publishes:

- `/drone_city_nav/executable_trajectory`
- `/drone_city_nav/path`
- `/drone_city_nav/path_id`
- `/drone_city_nav/trajectory_diagnostics`
- `/drone_city_nav/current_waypoint`
- `/drone_city_nav/static_building_markers`
- `/drone_city_nav/known_passage_markers`

The offboard node publishes:

- `/drone_city_nav/final_trajectory_path`
- `/drone_city_nav/offboard_debug_markers`
- `/drone_city_nav/truncation_suffix_ack`

The executable command atomically associates path geometry with path id and
safe-truncation metadata. Diagnostics are still matched by `path_stamp_ns`.
The `/drone_city_nav/path` debug mirror and
`/drone_city_nav/final_trajectory_path` messages carry per-sample altitude in
`pose.position.z`.

Known passage markers are RViz/debug artifacts. Architectural structure volumes,
opening frames, opening centers, approach arrows, and exit arrows help verify
annotation geometry. The same annotations feed the no-over-building validator
and, when enabled, the known-static lidar fallback classifier. Known-passage
validation is a planner acceptance guard: a trajectory that intersects an
annotated structure outside its opening is not published. It is not collision
or crash detection; physical contacts remain Gazebo's responsibility.
Classifier decisions apply only to new dynamic lidar evidence and are not tied
to an active trajectory passage span.

## Pipeline Contracts

The pipeline works because each stage has a narrow contract. A later stage can
improve shape or timing, but it should not reinterpret the meaning of earlier
artifacts.

Raw obstacle sources mean "there is evidence of an obstacle here". They do not
carry derived safety state. The obstacle-field builder owns the occupied
distance field and mode-specific risk policy. A* owns connectivity. Corridor
construction owns continuous lateral bounds. The trajectory optimizer owns
smoothness inside those bounds. Turn
smoothing owns local corner repair. Local passage insertion owns optional
known-passage XY repair. The speed profile owns scalar speed along the final
geometry. Offboard owns the actual runtime command.

When a run looks wrong, identify which contract failed before changing
parameters. Examples:

- If A* cannot find a route, inspect hard grid occupancy and start/goal
  validity before changing trajectory weights.
- If the trajectory is angular inside a wide corridor, inspect optimizer active
  windows and radius penalties.
- If the trajectory is smooth but the drone cuts a turn, inspect runtime speed,
  lateral command, smoother lag, and actual velocity.
- If a path disappears, inspect publication and rejection behavior before
  assuming the planner produced no route.

## Hard Safety Versus Planning Preference

The current model materializes only merged raw occupancy. Raw occupied cells
and points outside the evaluation ROI are hard rejects. The occupied distance
field derives a critical band and a planning band. Candidate ordering is
lexicographic: preferred space dominates planning-band exposure, which
dominates critical-band exposure; geometry quality decides only after risk.

Soft-band exposure does not invalidate an accepted trajectory. A newly raw
occupied span does. Diagnostics expose the worst tier, exposure lengths, and
minimum raw clearance so this distinction is observable.

## Stage-By-Stage Failure Reading

A useful analysis order for a failed or ugly run is:

1. Confirm that raw obstacle sources are plausible in RViz.
2. Confirm that raw occupied cells and ROI bounds are correct.
3. Confirm that critical/planning exposure ordering explains route selection.
4. Inspect A* route topology and whether the route is forced through a narrow
   passage.
5. Inspect corridor width and centerline blocked spans.
6. Inspect optimizer active samples and candidate rejection reasons.
7. Inspect turn-smoothing detected corners, attempted corners, and acceptance
   reasons.
8. Inspect final curvature and speed-profile constraints.
9. Inspect offboard cross-track, normal velocity, setpoint lag, and smoother
   limits.

This order prevents tuning late-stage control parameters for an early-stage
geometry problem, or changing planner weights for a runtime tracking problem.

## Why Speed Is Built After Geometry

Speed is a property of the final selected curve. It should not be used to hide
bad geometry. If a turn has a small radius, the speed profile will reduce speed
there, but the preferred fix is usually to increase the radius if the corridor
allows it.

Candidate selection no longer runs a full speed planner for every trajectory
candidate. That was expensive and pushed the optimizer toward "fast" geometry
instead of "smooth and trackable" geometry. The current approach is:

- choose geometry by smoothness, radius, curvature change, and validity;
- build one speed profile for the selected trajectory;
- let offboard runtime speed policy and smoother execute it.

This is why a final trajectory can still be color-coded by speed even though
speed was not the primary criterion for selecting the curve.

## Publication Invariants

A published executable path should satisfy:

- finite points and stable sample spacing;
- start and goal anchoring appropriate for the current plan;
- no hard prohibited intersection;
- known passage footprint intersections are reported with matched-opening or
  violation diagnostics;
- joins without unacceptable tangent or curvature jumps;
- diagnostics with the same path timestamp;
- continuity decision before replacing the accepted runtime trajectory.

If a build fails, the old trajectory should remain active unless a separate
safety condition requires hold. Publication should not use an empty path as an
ordinary "failed optimization" signal.
