# Known Architectural Passages

Known passages describe physical 3D openings that require route selection,
altitude alignment, and solid-volume collision checks.

## Sources Of Truth

```text
generated_city.sdf                 physical visuals and collisions
known_passages.passages3d          planner knowledge and passage semantics
generated_city.map2d               ordinary 2D static obstacle evidence
```

The SDF and annotation are intentionally separate files. Any geometry change
must update both. Script-level SDF contract tests verify current connector
positions, dimensions, visibility flags, and collision geometry.

The annotation format is versioned:

```text
drone_city_nav_known_passages_v1
frame_id map
structure <id> <center_x> <center_y> <size_x> <size_y> <z_min> <z_max>
opening <structure_id> <opening_id> <center_x> <center_y> <center_z> \
        <normal_x> <normal_y> <width> <height> <depth> \
        <min_z> <max_z> <approach_m> <exit_m>
```

The parser rejects malformed numeric values, duplicate identifiers, invalid
dimensions, invalid normals, out-of-structure centers, and invalid Z ranges.

## Physical And Sensor Geometry

Connector models contain real Gazebo collision geometry. The drone can
physically collide with lower, upper, and lateral material.

GPU lidar visibility masks exclude connector lower/upper visuals from the
simulated scan while keeping them visible in the GUI and physically collidable.
The planner therefore relies on the explicit known-solid annotation for these
volumes.

An optional endpoint/source classifier remains available through
`known_static_lidar_hit_classifier_enabled` and is disabled by default. It is a
fallback for interpreting lidar evidence near annotated solids; it is not the
primary passage detector and does not modify static-map cells.

## Solid Volumes

Each opening annotation is converted into oriented known-solid volumes around
the free opening:

- left mass;
- right mass;
- lower mass;
- upper mass.

The volumes are uploaded once to the persistent MPPI engine. Every rollout
sample and the reconstructed nominal horizon are checked against them.

Current limitation: known-solid checks use the simulated state point and do not
yet expand geometry by the full drone body or rotor footprint.

## Passage Selection

The runtime considers an opening only when:

- it is within the configured activation distance;
- the remaining global guide crosses the opening plane;
- guide direction has sufficient alignment with the opening normal;
- the crossing lies inside the usable lateral opening width.

Among matching openings, the nearest one is selected. The coordinator then
latches that opening until it exits or invalidates the episode.

This is guide-based passage selection, not a full global lattice portal
primitive.

## Coordinator Phases

`PassageCoordinator` owns:

- `inactive`;
- `approach`;
- `stationary_vertical_alignment`;
- `traversal`;
- `partial_from_inside`;
- `invalid_opening`.

The coordinator computes a safe vertical interval by applying the configured
clearance margin. Capture requires both altitude and vertical-speed conditions
for several stable cycles. Retention hysteresis prevents one noisy sample from
dropping traversal state.

If altitude cannot be captured before the entry boundary, the coordinator
requests:

```text
XY position hold
-> vertical alignment
-> stable altitude capture
-> traversal release
```

If altitude is already valid but the drone is laterally misaligned, it first
uses a low-speed approach staging target. A drone starting inside the opening
keeps its current valid altitude until it leaves the passage footprint.

## MPPI And Offboard Integration

An active passage provides MPPI with:

- opening center and normal;
- depth;
- safe Z range and preferred Z;
- approach and exit distances;
- mode-specific speed threshold;
- current passage phase.

Known-solid intersection and Z-window violation are collision results. Passage
speed above the configured threshold currently contributes a cost and the
separate speed policy lowers the reference speed.

During stationary alignment, the execution horizon carries an explicit
position-hold request. Offboard switches to PX4 position mode at the captured
XY point and preferred Z, then returns to horizon execution after the
coordinator releases hold.

## Visualization And Diagnostics

`world_visualization_node` publishes durable markers for structures, solids,
openings, and approach/exit directions. MPPI markers show the selected opening
and current local target.

Useful diagnostics include:

- selected opening id;
- coordinator phase;
- vertical and lateral error;
- distance to entry;
- capture/retention counters;
- alignment and stopping estimates;
- known-solid collision;
- actual opening-entry metrics from `mission_monitor_node`.

## Current Limitations

- SDF and annotation are manually synchronized.
- Structure orientation is inferred from current axis-aligned city geometry.
- Passage selection is local guide inference rather than a persistent global
  portal choice.
- Preferred altitude is not yet a route-station `z(s)` profile.
- Stationary hold shares the execution-horizon protocol.
- Full drone-footprint and 3D braking-fallback validation remain future safety
  work.
