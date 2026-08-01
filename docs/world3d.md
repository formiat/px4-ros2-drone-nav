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

No-static mode intentionally does not load semantic channel metadata. Its 2D
lidar sees connector occluders as ordinary obstacles, so it routes around them.
