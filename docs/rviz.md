# RViz Visualization

RViz is a diagnostic view. Its topics do not participate in planning or
offboard control.

## Starting RViz

GUI runs enable RViz by default through the simulation wrapper. The launch
argument is `enable_rviz`. `ENABLE_RVIZ_FOLLOW_CAMERA=false` selects the
top-down configuration instead of the `drone_follow` view.

## Frames

- `map`: planner and mission frame.
- `gazebo_map`: Gazebo-aligned visualization frame.
- `drone_follow`: visualization-only moving target published by offboard.

Do not infer a planner coordinate error until the displayed fixed frame and the
`gazebo_map -> map` transform are verified.

## Current Layers

| Display | Topic |
|---|---|
| Raw Obstacle Grid | `/drone_city_nav/raw_obstacle_grid` |
| Static City Map Points | `/drone_city_nav/static_map_points` |
| Selected MPPI Horizon | `/drone_city_nav/mppi/path` |
| Selected MPPI Markers | `/drone_city_nav/mppi/markers` |
| Interceptor Directions | `/drone_city_nav/interceptor_directions` |
| Drone | `/drone_city_nav/drone_marker` |
| Lidar Hit Points | `/drone_city_nav/lidar_debug_points` |
| Raw Lidar Returns 3D | `/drone_city_nav/raw_lidar_hit_points_3d` |
| Remembered Lidar Hits | `/drone_city_nav/remembered_lidar_points` |
| Raw Memory Cells | `/drone_city_nav/raw_memory_obstacle_points` |
| Raw Memory Hit Origins 3D | `/drone_city_nav/raw_memory_obstacle_points_3d` |
| Raw Occupied Cells | `/drone_city_nav/raw_occupied_cells` |

In the `3x1` intercept mission, the lightweight planner paths are also shown from
`/vehicles/interceptor_0/mppi/path` through
`/vehicles/interceptor_2/mppi/path`, each with a stable color. The global MPPI,
memory, and lidar topics above represent only the vehicle selected by
`/drone_city_nav/spectator_target`. The diagnostics mux clears old marker and
point-cloud state before switching those topics. Optional displays
for each interceptor's complete memory point cloud are present but disabled by
default, so they create no RViz subscription or rendering load until enabled.
The `2x2` script selects `evader_0` initially and changes the same RViz follow
frame and selected topics to `evader_1` after a typed destruction event when it
is still alive.

## MPPI Markers

The marker array includes:

- selected local horizon;
- previous/nominal horizon context;
- active global lattice guide;
- current MPPI target;
- mission start and goal;
- risk and collision annotations.

The global lattice guide is a route-direction polyline, not a complete
topological street graph. The MPPI horizon is the short executable local
trajectory. In static mode the guide is rendered at its planned Z. Constrained
span boundaries are currently exposed through diagnostics rather than separate
RViz markers.

## Reading Lidar Layers

- Raw lidar returns are sensor-frame observations projected into map space.
- Remembered hits persist after the obstacle leaves the current scan.
- Raw occupied cells are the merged planner evidence.
- Static points are a downsampled visualization generated from canonical
  Occupancy3D. The default four-cell stride keeps RViz responsive while the
  planner continues using the full 0.5 m map. A deterministic muted eight-color
  palette separates neighboring city blocks without connected-component or
  adjacency analysis. The translucent square rendering is visualization-only.

The static 3D cloud applies the same debug-only Z sign compensation as the
other navigation markers before the legacy `gazebo_map -> map` visualization
transform flips Z. Removing that compensation places the entire city below the
RViz ground plane even though publication and planning remain healthy.

The static cloud is published once with transient-local durability. Both RViz
profiles request the same durability so a display started after the world
visualization node still receives the retained map sample.

A point visible in one layer but absent in another can be correct because the
layers represent different lifecycle stages.

## Common Misreads

- A short blue horizon is expected; MPPI executes receding horizons.
- A distant global guide is not the command currently sent to PX4.
- The current static city contains four air-passage structures. Generated passage
  graph candidates and selected edges replace legacy semantic passage markers.
- RViz path publication can be throttled below MPPI tick rate.
- A follow-camera failure is a visualization problem unless vehicle state or
  control diagnostics are also stale.
