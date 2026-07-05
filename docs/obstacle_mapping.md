# Obstacle Mapping

Obstacle mapping combines static obstacles, current lidar hits, and accumulated
memory into planner inputs. The main rule is that raw sources stay raw; the
planner owns inflation and planning clearance.

## Static Map

The static map is loaded from:

```text
drone_city_nav/worlds/generated_city.map2d
```

Configured by:

```yaml
use_static_map: true
static_map_path: worlds/generated_city.map2d
static_map_min_blocking_height_m: 0.0
```

The static map is cached and reused when possible.

## Current Lidar Overlay

The planner can project the current `/scan` into a raw dynamic overlay. Current
lidar overlay is useful for obstacles that are visible now but not yet stable
in memory.

Important parameters:

- `max_current_lidar_staleness_s`
- `max_lidar_range_m`
- `range_hit_epsilon_m`
- lidar pose latency and attitude compensation settings.

## Obstacle Memory

`obstacle_memory_node` accumulates scan evidence into
`/drone_city_nav/obstacle_memory_grid`.

Memory uses hit/miss scoring:

- `hit_weight`
- `miss_weight`
- `min_score`
- `max_score`
- `occupied_score`
- `free_score`

Mapping starts only above `min_mapping_altitude_m`.

## Motion Compensation

Lidar projection can account for:

- PX4 heading;
- motion-compensated lidar pose;
- lidar pose latency;
- attitude compensation;
- lidar mount roll/pitch/yaw offsets;
- lidar z offset.

The same concepts appear in obstacle memory, planner current lidar overlay, and
lidar debug configuration.

## Raw, Prohibited, And Planning Clearance

Raw obstacles are direct evidence. The planner merges raw sources and produces:

- prohibited grid: hard safety grid;
- planning clearance: extra planner margin.

The current default is:

```yaml
inflation_radius_m: 1.0
planning_clearance_m: 3.0
```

The prohibited grid is published for validation and visualization. Planning
clearance affects route/trajectory construction and should not be interpreted
as a hard runtime failure by itself.

## RViz Outputs

Useful visualization topics:

- `/drone_city_nav/static_map_grid`
- `/drone_city_nav/static_map_points`
- `/drone_city_nav/obstacle_memory_grid`
- `/drone_city_nav/prohibited_grid`
- `/drone_city_nav/lidar_debug_points`
- `/drone_city_nav/remembered_lidar_points`
- `/drone_city_nav/prohibited_obstacle_points`
- `/drone_city_nav/raw_memory_obstacle_points`

## Common Problems

- If lidar hits are shifted, check PX4 origin, heading source, mount yaw, and
  attitude compensation.
- If obstacles appear too wide, check inflation and planning clearance.
- If memory contains stale obstacles, inspect hit/miss scoring and free-space
  clearing.
- If the planner replans unexpectedly, compare raw sources, prohibited grid,
  and the current executable trajectory.
