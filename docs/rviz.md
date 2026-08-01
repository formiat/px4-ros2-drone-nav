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
| MPPI Horizon | `/drone_city_nav/mppi/path` |
| MPPI Markers | `/drone_city_nav/mppi/markers` |
| Drone | `/drone_city_nav/drone_marker` |
| Lidar Hit Points | `/drone_city_nav/lidar_debug_points` |
| Raw Lidar Returns 3D | `/drone_city_nav/raw_lidar_hit_points_3d` |
| Remembered Lidar Hits | `/drone_city_nav/remembered_lidar_points` |
| Raw Memory Cells | `/drone_city_nav/raw_memory_obstacle_points` |
| Raw Memory Hit Origins 3D | `/drone_city_nav/raw_memory_obstacle_points_3d` |
| Raw Occupied Cells | `/drone_city_nav/raw_occupied_cells` |

## MPPI Markers

The marker array includes:

- selected local horizon;
- previous/nominal horizon context;
- active global lattice guide;
- current MPPI target;
- mission start and goal;
- constrained 3D route spans where applicable;
- risk and collision annotations.

The global lattice guide is a route-direction polyline, not a complete
topological street graph. The MPPI horizon is the short executable local
trajectory.

## Reading Lidar Layers

- Raw lidar returns are sensor-frame observations projected into map space.
- Remembered hits persist after the obstacle leaves the current scan.
- Raw occupied cells are the merged planner evidence.
- Static points are a downsampled visualization generated from canonical
  Occupancy3D. The default four-cell stride keeps RViz responsive while the
  planner continues using the full 0.5 m map.

A point visible in one layer but absent in another can be correct because the
layers represent different lifecycle stages.

## Common Misreads

- A short blue horizon is expected; MPPI executes receding horizons.
- A distant global guide is not the command currently sent to PX4.
- Static air channels are free volumes in Occupancy3D and physical Gazebo space.
- RViz path publication can be throttled below MPPI tick rate.
- A follow-camera failure is a visualization problem unless vehicle state or
  control diagnostics are also stale.
