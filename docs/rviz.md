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
- Drone markers: current drone pose, heading, and offboard debug markers.
- Speed/curvature coloring: path debug colors that help identify slow or tight
  segments.

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
