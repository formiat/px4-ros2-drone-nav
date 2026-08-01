# Canonical 3D World

## Source Of Truth

`drone_city_nav/worlds/canonical_city.world3d.json` is the only hand-edited
source of static geometry. It declares:

- the `map` frame and the map-to-SDF coordinate transform;
- Occupancy3D origin, dimensions, resolution, and chunk size;
- ground and regular building geometry;
- an optional array of generated air-channel structures;
- mission start and goal positions.

The current city keeps that optional array empty. It is a clean `5 x 8`
Manhattan grid containing 40 ordinary buildings and no connector structures,
openings, channel masses, or lidar occluders.

`scripts/generate_canonical_world.py` deterministically emits two committed
artifacts from that specification:

- `drone_city_nav/worlds/generated_city.sdf` for Gazebo rendering and physics;
- `drone_city_nav/worlds/generated_city.occupancy3d` for static planning.

The old `.map2d` and `.passages3d` sources no longer exist. There is no runtime
portal database, nearest-opening selector, or separate hand-authored passage
lifecycle.

## Regeneration

Run the generator through the repository container workflow:

```bash
./scripts/dev_shell.sh python3 scripts/generate_canonical_world.py \
  --spec drone_city_nav/worlds/canonical_city.world3d.json \
  --sdf drone_city_nav/worlds/generated_city.sdf \
  --occupancy drone_city_nav/worlds/generated_city.occupancy3d
```

`make test-scripts` regenerates both artifacts in a temporary directory and
checks byte-for-byte equality with the committed files. It also verifies that
the current world contains exactly the regular Manhattan buildings and no
channel or occluder models.

## Occupancy3D

The binary map uses schema version 1 with a sparse chunked bitset. Its header
stores grid bounds, resolution, chunk size, a fingerprint of the canonical JSON,
and the number of occupied chunks. The current world uses:

```text
origin:       (-30, -30, 0) m
size:         (345, 525, 40) m
resolution:   0.5 m
dimensions:   690 x 1050 x 80 cells
chunk size:   16 cells
```

Only physical collision geometry is voxelized. No clearance inflation,
prohibited grid, portal mask, or no-static lidar occluder is stored in
Occupancy3D.

`production_mppi_node` loads this file directly when `use_static_map=true`.
Around the current vehicle-to-planning-goal region it materializes an immutable
local dense ESDF3D and uploads that field to MPPI. The full global 3D array is
not rebuilt on every tick.

## Optional Channel Geometry

The canonical `channels` array can describe physical world construction, but it
is empty in the current city. A channel identifier, when present in another
world specification, is a generator identifier rather than a runtime navigation
event id.

### Straight

A straight channel contains one structure volume plus a 3D centerline. The
generator creates a physical lower mass and upper mass, leaving one continuous
free opening around the centerline reference Z.

### L-Shaped

The L-shaped channel is one intersection between four neighboring buildings
plus four bridge volumes. An optional left-turn declaration can use:

- open west and south bridges;
- blocked east and north bridges with physical middle masses;
- lower and upper physical masses on every bridge;
- one lower and one upper physical mass across the central intersection.

This produces a continuous L-shaped free volume. It is not a staircase and is
not two independent openings selected by a passage coordinator.

All channel centerlines have one constant reference Z. Every generated lower,
upper, and middle mass is an axis-aligned box whose floor and roof are parallel
to the ground. The generator does not support inclined slabs, stepped vertical
profiles, or curved vertical passages.

## Static Planning Contract

Static lattice search operates on `(x, y, z)` against the local ESDF3D. It uses
staged preferred, planning, and critical risk admission while physical occupied
voxels remain the only hard geometry. The accepted lattice points are sampled
as `RouteSample3D` values containing:

- 3D position;
- route tangent;
- cumulative route station;
- a reference-speed field populated for MPPI route execution.

The accepted route is then probed laterally and vertically against ESDF3D.
Sections narrower than the configured lateral or vertical thresholds become
`ConstrainedRouteSpan` values. A span contains station-indexed free-space and
reference data, not a semantic portal id:

- free distance left and right;
- minimum, maximum, and reference Z;
- constrained reference speed;
- begin and end route stations.

MPPI follows the complete typed route and applies the span speed/reference data.
The observable lifecycle is derived from route station:

```text
approach -> traversal -> departure -> unconstrained
```

Because classification is geometric, a constrained span can represent an
authored air channel or any other narrow section of the accepted route. Use
`route_generation + span_index` plus entry/exit coordinates to correlate a
diagnostic event with canonical geometry.

## No-Static Contract

No-static mode intentionally remains 2D:

- it does not load Occupancy3D for planning;
- it does not create `RouteSample3D` channel routes or constrained spans;
- it does not identify, infer, or traverse semantic passages;
- it has no 3D lidar or 3D channel perception.

For world specifications that contain channels, the generated SDF gives their
lower/upper/middle masses one dedicated visibility flag and adds transparent,
collisionless lidar occluders across openings. Before each run,
`scripts/configure_lidar_visibility.py` changes the GPU lidar mask. The current
city generates none of these models because its `channels` array is empty:

- static mode hides both channel masses and no-static occluders from the 2D
  lidar because Occupancy3D is authoritative;
- no-static mode exposes both sets, making every connector appear as an ordinary
  obstacle to lidar memory.

Occluders have no Gazebo collision element and are not written to Occupancy3D.
They are a simulation-only sensor contract used to disable channel traversal in
no-static mode. Physical channel masses remain real Gazebo collisions in both
modes.

## Visualization

Planning occupancy remains at `0.5 m`. RViz samples the static map with
`static_map_visualization_stride_cells=4`, producing `2 m` point spacing. This
reduces rendering load only; it does not change Occupancy3D, ESDF3D, lattice, or
collision resolution.

RViz shows the accepted route at its planned Z through the MPPI marker array.
Constrained-span boundaries and envelope values are available in logs, not as a
separate marker layer. RViz does not reconstruct authored channel ids or
publish legacy passage markers.

## Change Checklist

When changing static geometry or optional channels:

1. Edit only `canonical_city.world3d.json`.
2. Regenerate both committed artifacts.
3. Run `make test-scripts` and `make quality` in the container.
4. Check Occupancy3D points against Gazebo geometry in RViz.
5. If channels are present, verify static route Z and constrained-span
   diagnostics through each changed channel.
6. If channels are present, verify no-static lidar sees each opening as blocked
   and routes around it.
