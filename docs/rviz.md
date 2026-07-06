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

## Frames

Navigation debug data uses the `map` frame. Gazebo visualization uses its own
visual world convention, so the launch file publishes a fixed transform from
`gazebo_map` to `map` when RViz is enabled.

## Important Layers

- Static map: raw static obstacle source from the `.map2d` file.
- Prohibited grid: inflated hard-safety planner output on
  `/drone_city_nav/prohibited_grid`.
- Raw memory: uninflated obstacle memory evidence.
- Current lidar hits: current scan projection into the map.
- Corridor: left/right/free-space bounds around the rough route.
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
