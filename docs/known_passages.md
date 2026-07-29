# Semantic Portals

Semantic portals describe physical 3D openings that are part of a typed global
route. They combine route-station events, altitude references, speed policy,
and solid-volume collision checks without requiring a dense 3D map.

## Sources Of Truth

```text
generated_city.sdf                 physical visuals and collisions
known_passages.passages3d          semantic portal and solid-volume model
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

Left and right masses are also rasterized into the unified raw 2D world
snapshot before ESDF construction because they are solid at every flight
altitude. Lower and upper masses remain exclusively 3D constraints, leaving
the portal footprint traversable at its valid altitude. This gives the global
lattice the same connector topology in static and no-static modes.

## Typed Global Route

When a sticky global guide generation is accepted, it is converted once into a
`SemanticPortalRoute`. Each opening crossed by the guide creates a
`RoutePassageEvent` containing:

- the directed entry and exit planes;
- the opening polygon and hard Z interval;
- approach, entry, exit, and departure route stations;
- traversal direction;
- preferred altitude;
- mode-specific speed limit.

The complete route is split into typed normal, portal-approach,
portal-traversal, and portal-exit segments. A portal is never selected from
distance to the drone. A new global-route generation invalidates the old event
sequence atomically.

The global lattice treats every portal footprint as unavailable to ordinary
motion primitives. A dedicated bidirectional portal primitive is the only graph
edge allowed to enter through one portal plane and leave through the other.
That edge still checks the raw ESDF, so semantic geometry never hides a live
lidar obstacle. Consequently every accepted route through a connector already
contains a complete entry-to-exit crossing before route events are created.

## Coordinator Phases

`PassageCoordinator` owns:

- `inactive`;
- `upcoming`;
- `vertical_alignment`;
- `ready`;
- `traversal`;
- `cleared`;
- `invalid_route_event`.

The coordinator consumes the next event from the active semantic route and the
current monotonic route station. Capture requires both altitude and
vertical-speed conditions for several stable cycles. Retention hysteresis
prevents one noisy sample from dropping traversal state. Starting inside a
portal initializes the same traversal phase while preserving a valid current
altitude.

If altitude cannot be captured before the entry boundary, the coordinator
requests:

```text
XY position hold
-> vertical alignment
-> stable altitude capture
-> traversal release
```

## MPPI And Offboard Integration

Every MPPI tick receives the full route polyline with cumulative station, not
only one lookahead point. Every rollout sample is projected onto this route;
cross-track error and along-route progress are computed from the projection.

The upcoming route event provides MPPI with:

- opening center and normal;
- depth;
- safe Z range and preferred Z;
- approach, entry, exit, and departure route stations;
- mode-specific speed threshold;
- current passage phase.

The vertical reference is a continuous `z(route_station)` profile: normal
flight altitude before approach, interpolation to the preferred portal
altitude, fixed altitude through the opening, and interpolation back on exit.
Known-solid intersection and opening Z-window violation are collision results.
The passage speed policy is activated only after the current route station
enters the portal approach.

During stationary alignment, the execution horizon carries an explicit
position-hold request. Offboard switches to PX4 position mode at the captured
XY point and preferred Z, then returns to horizon execution after the
coordinator releases hold.

## Visualization And Diagnostics

`world_visualization_node` publishes durable markers for structures, solids,
portals, and approach/exit directions. MPPI markers show the active route event
and current local target.

Useful diagnostics include:

- portal id and route generation;
- route event index and station;
- coordinator phase;
- vertical error and Z reference;
- distance to entry;
- capture/retention counters;
- alignment and stopping estimates;
- known-solid collision;
- actual opening-entry metrics from `mission_monitor_node`.

## Remaining Limitations

- SDF and annotation are manually synchronized.
- Structure orientation is inferred from current axis-aligned city geometry.
- Stationary hold shares the execution-horizon protocol.
- Full drone-footprint and 3D braking-fallback validation remain future safety
  work.
