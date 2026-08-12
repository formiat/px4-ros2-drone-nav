# Canonical 3D World

## Source Of Truth

`drone_city_nav/worlds/canonical_city.world3d.json` is the only hand-edited
source of static geometry. It declares:

- the `map` frame and the map-to-SDF coordinate transform;
- Occupancy3D origin, dimensions, resolution, and chunk size;
- ground and regular building geometry;
- an optional array of generated air-channel structures;
- mission start and goal positions.

The current city contains 40 ordinary buildings plus four horizontal channels:

- straight-through at `(54, 162)`, open south/north;
- left turn at `(108, 162)`, open south/east;
- left turn at `(108, 216)`, open west/north.
- T junction at `(108, 108)`, open west/east/north and physically closed south.

RViz reverses the visual X direction relative to map X. The map directions above
therefore produce the screen-space cross-sections requested for a vehicle
approaching from the lower-right mission start.

The canonical-world toolchain deterministically emits three committed artifacts
from that specification:

- `drone_city_nav/worlds/generated_city.sdf` for Gazebo rendering and physics;
- `drone_city_nav/worlds/generated_city.occupancy3d` for static planning;
- `drone_city_nav/worlds/generated_city.esdf3d` for precomputed static distances.

The old `.map2d` and `.passages3d` sources no longer exist. There is no runtime
portal database, nearest-opening selector, or separate hand-authored passage
lifecycle.

Building visuals use the same deterministic eight-color muted palette in Gazebo
and RViz. The palette index is derived from the building grid coordinates, so a
building keeps the same color in both views and across regenerated worlds. RViz
controls its own transparency; Gazebo building materials remain opaque.

## Regeneration

Run the generators through the repository container workflow:

```bash
./scripts/dev_shell.sh
make build
python3 scripts/generate_canonical_world.py \
  --spec drone_city_nav/worlds/canonical_city.world3d.json \
  --sdf drone_city_nav/worlds/generated_city.sdf \
  --occupancy drone_city_nav/worlds/generated_city.occupancy3d
./build/drone_city_nav/generate_static_esdf_cache \
  --occupancy drone_city_nav/worlds/generated_city.occupancy3d \
  --output drone_city_nav/worlds/generated_city.esdf3d \
  --maximum-distance-m 26 --workers 8
```

`make test-scripts` regenerates both artifacts in a temporary directory and
checks the SDF and Occupancy3D byte-for-byte. It also verifies that the committed
ESDF cache carries the same grid metadata and canonical-world fingerprint, plus
all four channel cross-sections, orientation, horizontal geometry, deduplicated
shared bridge masses, and lidar visibility contract. C++ tests verify exact
cache round-tripping, ROI extraction, distance capping, and corruption handling.

## Occupancy3D

The binary map uses schema version 3 with a sparse chunked bitset. Its header
stores grid bounds, resolution, chunk size, a fingerprint of the canonical JSON,
and the number of occupied chunks. The chunk payload is followed by the generated
constrained free-space graph. Each channel edge stores its identifier, sampled 3D
centerline, entry and exit, explicit width and height, vertical window, minimum
clearance, and speed limit. Keeping the cross-section dimensions separate allows
cooperative traffic to derive and validate multiple lateral lanes without confusing
the lower vertical clearance with the available horizontal width.
Entry and exit are generated on the exterior portal planes, not at bridge
centers, so approach and departure route segments remain outside solid geometry.
This metadata is compiled from the same canonical JSON; it is not a separate
hand-authored passage source. The current world uses:

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

`production_mppi_node` loads Occupancy3D directly when `use_static_map=true`.
The immutable global ESDF is computed offline and stored as independently
compressed 16-cubed chunks. At runtime the planner decodes only chunks intersecting
the current vehicle-to-planning-goal ROI, materializes that local dense ESDF3D,
and uploads it to MPPI. The cache is accepted only when its grid metadata,
canonical-world fingerprint, and maximum distance satisfy the current request.
A missing, corrupt, or incompatible cache is logged and falls back to the exact
runtime EDT builder.

The cache stores exact squared voxel distances rather than an inflated occupancy
layer. Only raw Occupancy3D cells remain hard obstacles. Using the global field
also preserves distance information from physical objects immediately outside a
local computational ROI; the ROI boundary itself is not an obstacle.

## Generated Channel Geometry And Graph

The canonical `channels` array describes physical world construction and one or
more corresponding constrained free-space edges. A channel identifier is stable
across the generated SDF and Occupancy3D graph.

### Straight

A straight channel contains one structure volume plus a 3D centerline. The
generator creates a physical lower mass and upper mass, leaving one continuous
free opening around the centerline reference Z.

### Four-Building Intersection

An `intersection` channel contains one center volume plus four bridge volumes
between four neighboring buildings. The `blocked` state of each bridge defines
the route shape:

- opposite open bridges produce a straight-through channel;
- adjacent open bridges produce an L-shaped channel;
- three open bridges produce a T junction;
- blocked bridges receive physical middle masses;
- lower and upper physical masses on every bridge;
- one lower and one upper physical mass across the central intersection.

This produces one continuous free volume. It is not a staircase and is not a
set of independent openings selected by a passage coordinator. Shared bridge
masses between adjacent channel declarations are deduplicated by physical
geometry before SDF and Occupancy3D generation.

A T junction compiles three pairwise graph edges through one physical center:
west/east, west/north, and east/north. This keeps every traversal bidirectional
without duplicating collision geometry.

All channel centerlines have one constant reference Z. Every generated lower,
upper, and middle mass is an axis-aligned box whose floor and roof are parallel
to the ground. The generator does not support inclined slabs, stepped vertical
profiles, or curved vertical passages.

## Static Planning Contract

Static global search operates on a hybrid graph:

```text
ordinary omnidirectional 3D lattice edges
+ generated bidirectional constrained channel edges
```

Every edge is validated against the current raw Occupancy3D/ESDF3D before use.
At the root, search explicitly evaluates collision-free
`start -> entry -> channel -> exit` candidates for every channel direction;
ordinary lattice expansion remains available for entries that cannot be reached
directly. Partial lattice frontiers require measured continuation depth beyond
their endpoint before they can be executed.
Physical occupied voxels remain the only hard geometry. Preferred, planning, and
critical stages all run, and all complete routes are compared by finite objective
cost instead of returning the first preferred route. The objective records travel
time, vertical-alignment time, risk exposure, and turn cost. The extra vertical
alignment weight is intentionally `0.0` during channel development; vertical
motion still contributes through physical travel time.

No channel is mandatory and there is no required channel sequence. A channel is
selected only when its validated route has the better objective cost. The accepted
points are sampled as `RouteSample3D` values containing:

- 3D position;
- route tangent;
- cumulative route station;
- a reference-speed field populated for MPPI route execution.

When the selected graph path traverses a channel edge, that transition directly
creates a typed `ConstrainedRouteSpan` with `approach -> traversal -> departure`
station semantics. The complete route and span are revalidated against the latest
raw ESDF before atomic activation. A span contains:

- free distance left and right;
- minimum, maximum, and reference Z;
- constrained reference speed;
- begin and end route stations.

The channel identifier is retained for lifecycle diagnostics and RViz. Geometric
ESDF queries validate the selected edge; they no longer infer passage lifecycle
postfactum from arbitrary narrow route samples.

MPPI follows the complete typed route and applies the span speed/reference data.
The observable lifecycle is derived from route station:

```text
approach -> traversal -> departure -> unconstrained
```

Use `route_generation + channel_id + span_index` to correlate a diagnostic event
with canonical geometry.

## No-Static Contract

No-static mode intentionally remains 2D:

- it does not load Occupancy3D for planning;
- it does not create `RouteSample3D` channel routes or constrained spans;
- it does not identify, infer, or traverse semantic passages;
- it has no 3D lidar or 3D channel perception.

The generated SDF gives channel lower/upper/middle masses one dedicated
visibility flag and adds transparent, collisionless lidar occluders across each
intersection and open bridge. Before each run,
`scripts/configure_lidar_visibility.py` changes the GPU lidar mask:

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
Generated candidate channel edges are thin translucent blue lines; channel edges
selected by the active route are thicker green lines. Tick JSONL and guide logs
include selected topology, objective cost, route length, travel time, vertical
alignment time, risk exposure, channel id, and acceptance/rejection reason.

## Change Checklist

When changing static geometry or optional channels:

1. Edit only `canonical_city.world3d.json`.
2. Regenerate all three committed artifacts.
3. Run `make test-scripts` and `make quality` in the container.
4. Check Occupancy3D points against Gazebo geometry in RViz.
5. If channels are present, verify static route Z and constrained-span
   diagnostics through each changed channel.
6. If channels are present, verify no-static lidar sees each opening as blocked
   and routes around it.
