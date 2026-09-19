# 3D World Artifacts

## Source Of Truth

A World3D specification declares the static world a mission flies in:

- the `map` frame and the map-to-SDF coordinate transform;
- Occupancy3D origin, dimensions, resolution, and chunk size;
- the flight envelope and initial altitude;
- ground and building geometry when the world carries its own geometry.

`drone_city_nav/worlds/urban_circuit_practice_01.world3d.json` is the only
committed specification. It describes the imported Urban Circuit environment;
its Gazebo geometry, collision world, and sensor world come from the environment
artifact pipeline described in `docs/gazebo_simulation.md`, not from a generator
in this repository. There is no hand-authored portal database and no
nearest-opening selector.

Static map artifacts are optional per environment. When an environment ships
them, `scripts/prepare_environment_simulation.py` exports their paths and
`scripts/compile_environment_topology.py` compiles the matching
FreeSpaceTopology3D. Urban Circuit runs no-static, so the exported static paths
are empty and the planner loads no offline map.

Free-space topology is a separately versioned compiled index derived from the
same raw voxels used for collision checks.

## Occupancy3D

The binary map uses schema version 5 with a sparse chunked bitset. Its header
stores grid bounds, resolution, chunk size, a fingerprint of the exact serialized
raw voxel geometry,
and the number of occupied chunks. The file ends after the raw chunk payload.
It contains no regions, portals, traversals, clearance envelopes, or other
derived planning data.

## FreeSpaceTopology3D

The separately versioned topology artifact stores the Occupancy3D fingerprint
and exact grid geometry followed by open-space regions, arbitrary portal voxel
patches, and sparse medial passage segments. A route-specific traversal is
resolved lazily from these segments and cached; it is not materialized for every
portal pair offline. Each segment records a sampled 3D centerline, endpoint
portals and neighbors, minimum clearance, and speed limit. A topology file is
used only when both its fingerprint and bounds match the loaded raw occupancy.

The graph is computed from raw occupancy and the matching ESDF after physical
geometry has been voxelized. World3D passage records do not provide centerlines,
portal IDs, planner edges, or conflict resources. Nearby physical structures
merge naturally when their constrained free volumes are connected. The grid
origin, size, resolution, and chunk size come from the World3D specification of
the environment being compiled.

Only physical collision geometry is voxelized. No clearance inflation,
prohibited grid, portal mask, or no-static lidar occluder is stored in
Occupancy3D.

`production_mppi_node` loads Occupancy3D directly when `use_static_map=true`.
The immutable global ESDF is computed offline and stored as independently
compressed 16-cubed chunks. At runtime the planner decodes only chunks intersecting
the current vehicle-to-planning-goal ROI, materializes that local dense ESDF3D,
and uploads it to MPPI. The cache is accepted only when its grid metadata,
raw-occupancy fingerprint, and maximum distance satisfy the current request.
A missing, corrupt, or incompatible cache is logged and falls back to the exact
runtime EDT builder.

The cache stores exact squared voxel distances rather than an inflated occupancy
layer. Only raw Occupancy3D cells remain hard obstacles. Using the global field
also preserves distance information from physical objects immediately outside a
local computational ROI; the ROI boundary itself is not an obstacle.

## Topology Extractor V2

The world compiler builds the complete static passage index in deterministic
stages:

1. Decode the immutable ESDF chunks covering the configured analysis envelope.
2. Classify raw-footprint-feasible voxels by clearance without modifying raw
   occupancy.
3. Label open-space components and medial constrained-space components.
4. Extract bottleneck surfaces as arbitrary 3D portal voxel patches and derive
   display polygons from those patches.
5. Skeletonize each constrained component into a sparse medial segment graph,
   preserving slopes, shafts, curves, and junctions.
6. Reject components that cannot produce raw swept-footprint-safe segments.
7. Assign deterministic geometry-derived region, portal, and segment IDs.

This pass covers the complete static Occupancy3D artifact, not only the current
mission route. Portal semantics are therefore reusable by every vehicle and
every static mission. The compiler never creates occupied or prohibited cells;
the graph is an immutable evidence index over raw geometry. Runtime route
association and activation still validate physical geometry with the swept
footprint against raw Occupancy3D.

The World3D `free_space_topology.validation_capsule` is the minimum vehicle
profile used to compile useful sparse segments. The configured Z range bounds
only the offline acceleration index. Neither setting inflates or modifies
Occupancy3D, defines a hard clearance envelope, or replaces runtime validation.
Every route-associated traversal is checked again with the current vehicle's
swept footprint against raw Occupancy3D.

## Advanced Passage Fixtures

`drone_city_nav/tests/advanced_passage_fixture.cpp` builds compact deterministic
raw Occupancy3D volumes for the topology compiler. The fixtures contain no
portal IDs, centerlines, or planner masks. Their independent acceptance contract
defines only open-space seeds, physically raw-safe reference paths, and minimum
topology cardinality:

- a straight sloped tunnel with entrances at different heights;
- a vertical shaft with horizontal portal surfaces;
- a non-rectangular arch tunnel;
- a curved tunnel with changing XYZ tangent;
- three-arm T and four-arm X junctions;
- a wide roofed hangar that must not be classified as a constrained passage.

The fixture tests first prove physical swept-footprint feasibility directly
against raw occupancy and verify that positive cases contain a measurable
clearance bottleneck. This keeps geometry-extractor tests independent from the
implementation that they validate.

## Static Planning Contract

Static point-to-point search uses the same persistent sparse D* Lite graph as
no-static navigation. The graph is world-fixed and full-3D; its edges use the
shared flight-time objective and are accepted only after exact swept-footprint
validation against raw Occupancy3D. Physical occupied voxels and the
flight envelope remain the only hard geometry. The ESDF cache supplies derived
distance evidence but cannot create an occupied or prohibited region.

`FreeSpaceTopology3D` is an offline, fingerprint-bound passage evidence index.
It does not add a second strategic search, inject macro-edges into D* Lite, or
arbitrate route ownership. Its candidate traversal geometry remains available
to RViz and physical passage diagnostics. A route is valid regardless of whether
it is represented by that optional index.

The persistent planner returns `RouteSample3D` values containing:

- 3D position;
- route tangent;
- cumulative route station;
- a reference-speed field populated for MPPI route execution.

If certified route geometry is associated with a derived passage traversal, the
association is execution evidence rather than a route choice. Its typed
`ConstrainedRouteSpan` has `approach -> traversal -> departure` station semantics
and is included in the same immutable route geometry. A span contains:

- free distance left and right;
- minimum, maximum, and reference Z;
- constrained reference speed;
- begin and end route stations.

Derived traversal and ordered segment identifiers are retained for lifecycle
diagnostics, cooperative conflict resources, and RViz. Exact raw geometry
validates the route; topology identity cannot weaken or strengthen collision
semantics.

MPPI follows the complete typed route and applies the span speed/reference data.
The observable lifecycle is derived from route station:

```text
approach -> traversal -> departure -> unconstrained
```

Use `route_generation + passage_id + span_index` to correlate a diagnostic event
with a derived `PassageTraversalId`. Region, portal, segment, traversal, and
cooperative conflict-resource IDs are distinct strong types in C++; their ROS
representations are explicitly named strings. Topology IDs intentionally follow
geometry, so a physical topology change may produce a new ID.

## No-Static Contract

No-static production navigation requires `LIDAR_PROFILE=3d`. It builds
revisioned observed Occupancy3D, local occupied-distance evidence, and persistent
`RouteSample3D` routes. Confirmed free and unknown voxels have identical
strategic traversability and base cost. The mode does not divide the world into
open space and semantic passages and does not load the static topology artifact.

A world SDF may mark simulation-only lidar occluders with a dedicated
visibility flag. Before each run, `scripts/configure_lidar_visibility.py`
changes the GPU lidar mask:

- static mode hides those occluders from its optional diagnostic lidar because
  raw Occupancy3D is authoritative;
- no-static 3D mode hides the simulation-only occluders but exposes physical
  geometry, allowing hit and miss rays to establish the real free volume.

Occluders have no Gazebo collision element and are not written to Occupancy3D.
They are simulation-only scene resources and are never planning evidence.
Physical geometry remains real Gazebo collision in every mode.

## Visualization

Planning occupancy remains at `0.5 m`. RViz samples the static map with
`static_map_visualization_stride_cells=4`, producing `2 m` point spacing. This
reduces rendering load only; it does not change Occupancy3D, ESDF3D,
persistent-planner step, or collision resolution.

RViz shows the accepted route at its planned Z through the MPPI marker array.
Derived sparse passage segments are thin translucent blue lines; route-associated
traversal evidence is thicker green. Tick JSONL and route logs include persistent
planner status, route length, execution-time estimate, raw evidence lineage,
reserve, compilation, publication, and activation status.

## Change Checklist

When changing a static world:

1. Edit its World3D specification.
2. Rebuild the environment artifacts and, if the environment ships a static map,
   recompile its FreeSpaceTopology3D.
3. Run `make test-scripts` and `make quality` in the container.
4. Check Occupancy3D points against Gazebo geometry in RViz.
5. Verify the no-static 3D profile ignores simulation-only occluders, observes
   physical geometry, and validates the real free volume from raw sensor evidence.
