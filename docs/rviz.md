# RViz Visualization

RViz is used for navigation debugging. It is not required for headless smoke
runs, but it is the main way to inspect grids, trajectories, and markers.

## Starting RViz

RViz can be enabled through the launch argument:

```bash
ros2 launch drone_city_nav city_nav.launch.py enable_rviz:=true
```

The standard GUI script may also start RViz depending on the chosen run mode
and environment.

Default config:

```text
drone_city_nav/rviz/city_nav_debug.rviz
```

The default config follows the drone through the visualization-only
`drone_follow` TF frame. The GUI runner enables this by default. Use:

```bash
ENABLE_RVIZ_FOLLOW_CAMERA=false ./scripts/sim_gui.sh
```

to open the top-down fallback layout from:

```text
drone_city_nav/rviz/city_nav_debug_top_down.rviz
```

The follow frame is for RViz camera targeting only; it is not consumed by the
planner, controller, or mission monitor.

## Frames

Navigation debug messages are published in the planning/control `map` frame, but
the default RViz config intentionally uses `gazebo_map` as its fixed frame. The
launch file publishes `gazebo_aligned_map_tf`, a fixed compatibility transform
that swaps the horizontal X/Y axes and flips Z for visualization. This is not a
bug and should not be removed casually: it exists so the static map, 3D building
markers, known passage markers, trajectory, and drone marker line up with the
generated city as it is visually inspected in Gazebo. If this transform is ever
removed, the Gazebo world convention, static map coordinates, and debug overlays
must be migrated together.

Because that transform maps positive `map` altitude to negative `gazebo_map` Z,
3D debug publishers intentionally compensate visual Z before publishing RViz
markers/point clouds/debug paths. That compensation is visualization-only and
must not be applied to the control path consumed by offboard flight logic.

## Important Layers

- Static map: raw static obstacle source from the `.map2d` file.
- Static building volumes: semi-transparent 3D CUBE markers built from the
  `.map2d` rectangle heights on `/drone_city_nav/static_building_markers`.
- Prohibited grid: inflated hard-safety planner output on
  `/drone_city_nav/prohibited_grid`.
- Raw Memory Cells: uninflated 2D obstacle-memory cell centers on the ground
  plane.
- Raw Memory Hit Origins 3D: exact lidar trigger endpoints that first made the
  corresponding active memory cells occupied.
- Current lidar hits: current scan projection into the map.
- Raw lidar returns 3D: current scan returns at their transformed 3D endpoints.
- Corridor: left/right/free-space bounds around the rough route.
- Known passages: 3D passage annotations from
  `/drone_city_nav/known_passage_markers`.
- Final optimized trajectory: executable trajectory published by offboard on
  `/drone_city_nav/final_trajectory_path`.
- Drone markers: current drone pose, altitude, heading, and offboard debug
  markers.
- Speed/curvature coloring: path debug colors that help identify slow or tight
  segments.

The final trajectory path and trajectory color markers are displayed at the
sample `z_m` altitude, not on the ground plane. The drone pose marker is a 3D
sphere at the current PX4 local altitude and is deleted when pose or altitude
data is stale/invalid.

## Obstacle Memory Layers

The default RViz configuration shows obstacle memory twice for two different
diagnostic questions:

- `Raw Memory Cells` subscribes to
  `/drone_city_nav/raw_memory_obstacle_points`. Each point is the center of an
  active 2D occupied cell at the fixed visualization altitude `z=0.05 m`. This
  is the planning representation and remains unchanged.
- `Raw Memory Hit Origins 3D` subscribes to
  `/drone_city_nav/raw_memory_obstacle_points_3d`. Each point is the exact
  `occupancy_trigger.endpoint_map_m` observation that first changed the active
  cell to occupied. Its per-point Z is compensated with the same intentional
  Gazebo-aligned RViz convention described above.

The 3D layer contains one small point per active provenance record with valid,
finite XYZ. It does not show inflation, `last_hit`, Z ranges, historical cells,
or a voxel map. When a memory cell is no longer occupied, both its planning
point and its 3D trigger point disappear on their next debug update. This layer
is distinct from `/drone_city_nav/remembered_lidar_points`, which is a separate
lidar debug accumulator rather than authoritative obstacle memory.

## Current Lidar Layers

RViz displays the current scan in two complementary layers:

- `Lidar Hit Points` is the existing 2D debug view on the configured ground
  visualization plane.
- `Raw Lidar Returns 3D` subscribes to
  `/drone_city_nav/raw_lidar_hit_points_3d`. It contains every valid physical
  return from the latest scan at its map-frame XYZ endpoint after pose,
  attitude, and lidar-mount transformation.

The 3D layer is published directly from each scan with volatile QoS. It does
not enter obstacle memory, classifier/ground filtering, trajectory planning, or
the lidar debug accumulator. Mapping-specific projected-altitude limits are
intentionally disabled for this visualization, so a tilted lidar can show the
ground or a passage upper/lower mass. An empty scan or unavailable pose publishes
an empty cloud, clearing the previous 3D scan from RViz. Its Z uses the same
intentional Gazebo-aligned compensation as other 3D debug layers.

## Known Passage Markers

Known passages are pre-annotated 3D passages. The system loads, logs,
visualizes, and validates them against the published trajectory. The same
annotations define known physical solids for the always-on 3D lidar
classifier before dynamic evidence is recorded. Confident matching solid hits
are suppressed; detached and free-opening hits are preserved, while ambiguous
static-attached hits remain pending without changing either 2D grid. The
classifier itself does not change A* route preference or static-map cells.
Separate known-passage stages can locally repair final XY geometry, add a
vertical profile, and cap traversal speed. Validation failures remain
diagnostic/repair input and do not reject a path by themselves.

The RViz `Known Passages` display subscribes to:

```text
/drone_city_nav/known_passage_markers
```

Marker namespaces:

- `known_passage_structure`: visible architectural building pieces around each
  opening.
- `known_passage_opening_frame`: wireframe box for the traversable opening.
- `known_passage_opening_center`: center point of the traversable opening.
- `known_passage_approach`: approach direction into the opening.
- `known_passage_exit`: exit direction after the opening.

If the known passage source is disabled, empty, or fails to load, the planner
publishes delete markers so stale passage objects disappear from RViz.

RViz currently shows the annotated volumes and final trajectory, but it does
not highlight individual lidar hits suppressed by the known-static classifier.
Use trajectory diagnostics JSON and planner/obstacle-memory logs to inspect
`known_passage_validation[...]`, `known_passage_diagN_*`, and
`known_static[...]` details.

## Reading The Colors

The exact color mapping is defined in the debug marker and RViz configuration,
but the practical interpretation is:

- green/blue-like segments usually indicate easier or faster trajectory parts;
- red/purple-like segments usually indicate tighter curvature, lower speed
  limits, or notable constraints;
- abrupt color changes can indicate curvature jumps, isolated geometry spikes,
  or speed-profile constraints.

Use the trajectory dumps and planner diagnostics to confirm the exact reason.
Do not rely on color alone for root-cause analysis.

## Common Visual Artifacts

- A path can appear briefly missing when no valid path is published or when a
  failed rebuild publishes a hold/empty path.
- A trajectory can look slightly wavy on straight-ish sections; this is not
  automatically a bug if curvature and control metrics are acceptable.
- The prohibited grid includes safety inflation. It is expected to look wider
  than raw obstacles.
- The planning clearance is not a separate RViz hard-safety replan trigger; it
  is an additional planner preference margin.

## Useful Cross-Checks

When RViz looks surprising, compare it with:

- `log/offboard_blackbox.jsonl`
- `log/final_trajectory_samples/latest.csv`
- `log/final_trajectory_samples/latest_summary.json`
- `log/corridor_samples/latest.csv`
- `log/lidar_debug/snapshots.jsonl`

For lidar projection snapshots:

```bash
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

## RViz Interpretation Rules

RViz is a diagnostic view, not the source of truth. Use it to identify where to
look in logs. Do not tune a parameter only because a color looks surprising.
The color should be traced to a numeric field such as curvature, speed limit,
clearance, or candidate rejection reason.

Useful interpretation rules:

- raw obstacle layers should align with visible static geometry;
- prohibited grid should be wider than raw obstacles by the hard inflation
  radius;
- planning clearance can influence the route without appearing as a hard
  replan boundary;
- the final trajectory is the executable curve, not the rough route;
- a slightly wavy final trajectory is acceptable if control metrics are stable;
- a visually smooth curve can still be too fast or poorly tracked.

## Layer Correlation Workflow

When a path looks wrong:

1. Hide the final trajectory and inspect raw static and lidar layers.
2. Enable the prohibited grid and confirm hard safety space.
3. Enable the corridor and check whether the desired smooth path had room.
4. Enable the final trajectory and inspect whether it uses available corridor
   width.
5. Check speed/curvature coloring around suspicious segments.
6. Open trajectory diagnostics for the same path stamp.
7. Open offboard blackbox around the matching flight time.

This order separates geometry availability from optimizer choice and runtime
tracking.

## Reading Purple Or Red Segments

Purple or red-like segments usually mean the trajectory is constrained, but the
reason can differ:

- tight radius from actual geometry;
- isolated curvature spike;
- speed-profile braking before a later turn;
- terminal capture slowing near the goal;
- debug coloring threshold rather than a hard problem.

Confirm with top speed constraints and local curvature metrics. If a segment
is slow because radius is small, improve trajectory geometry if the corridor
allows it. If it is slow because the speed profile is braking for a later turn,
the visible slow segment can be upstream of the actual curve.

## RViz Versus Runtime Control

RViz shows the path and debug markers, but the actual drone command is shaped
after that path is accepted. Runtime control also depends on:

- predicted projection;
- projection smoothing mode;
- P and D gain schedules;
- curvature feedforward context;
- speed policy;
- velocity smoother;
- terminal state.

If the drone oscillates on a path that looks acceptable, inspect blackbox
control fields. The trajectory may not be the problem.

## Common RViz Misreads

Common mistakes:

- treating planning clearance as a collision boundary;
- treating the rough route as the executable trajectory;
- assuming path color explains the cause without checking diagnostics;
- judging terminal position capture as normal lateral control;
- ignoring path stamp mismatch between RViz display and diagnostics;
- forgetting that a rejected trajectory may still appear in planner logs.

When in doubt, correlate by path stamp and navigation time.
