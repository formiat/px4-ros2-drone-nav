# Canonical 3D World

`drone_city_nav/worlds/canonical_city.world3d.json` is the source of truth for
static geometry. `scripts/generate_canonical_world.py` deterministically emits:

- `generated_city.sdf` for Gazebo;
- `generated_city.occupancy3d` for static planning.

The binary map is sparse and chunked. The planner materializes an immutable
dense ESDF3D view for GPU collision and risk queries. Occupied voxels represent
physical geometry only; no inflated or prohibited layer is generated.

The static lattice searches `(x, y, z)` and returns `RouteSample3D` values.
Route-envelope analysis derives `ConstrainedRouteSpan` sections from clearance.
Straight openings, L-shaped channels, and altitude-changing channels therefore
use the same route contract. MPPI consumes route position, altitude, tangent,
and reference speed directly.

An L-shaped channel is authored as one intersection between four neighboring
buildings plus four bridge volumes. Every bridge has lower and upper physical
masses; two bridges additionally have a solid middle mass, while the other two
form the entrance and exit. The intersection itself has one lower and one upper
mass. This creates one continuous turn volume rather than carving a bend inside
a single connector.

Altitude-changing channels use continuous inclined lower and upper physical
masses. The free opening and its `RouteSample3D` reference Z rise continuously;
neither Gazebo geometry nor planning occupancy approximates the channel with
stair-step slices.

Planning occupancy remains at `0.5 m`. RViz intentionally samples that map with
`static_map_visualization_stride_cells=4`, producing a `2 m` diagnostic point
spacing. This visualization reduction does not affect planning or collision
queries.

No-static mode intentionally does not load semantic channel metadata. Its 2D
lidar sees connector occluders as ordinary obstacles, so it routes around them.
