# Known Architectural Passages

Known architectural passages let the simulation use real three-dimensional
openings in buildings without replacing the established two-dimensional
navigation stack with a general volumetric planner.

The design deliberately separates three concerns:

```text
Gazebo SDF              physical rendering and collision
known_passages.passages3d  drone knowledge, trajectory constraints, and lidar geometry
generated_city.map2d    ordinary 2D static obstacle evidence for A*
```

An opening is ordinary free space. The material around it is ordinary solid
building geometry. There is no special runtime rule that declares a vehicle to
have "passed" or "missed" an opening: Gazebo collision and ordinary mission
collision volumes determine whether the vehicle contacted a solid mass.

## Physical Buildings And The 2D Map

The current world contains three physical passage buildings named
`physical_building_connector_*` in:

```text
drone_city_nav/worlds/generated_city.sdf
```

Each is a real static SDF collision and visual model. The model is split into:

- `lower_mass`, from ground level to the bottom of the opening;
- `upper_mass`, from the top of the opening to the top of the building.

The opening spans the complete lateral width of the connector, so the current
models do not need separate `left_mass` and `right_mass` SDF links. The lower
and upper masses form one wide architectural building between its neighboring
buildings, with a seven-meter-tall free opening rather than a standalone gate.

The three current openings have center altitudes of approximately 5 m, 15 m,
and 25 m. Their physical opening interval is respectively 1.5..8.5 m,
11.5..18.5 m, and 21.5..28.5 m. Their exact positions, dimensions, orientation,
and approach/exit distances are defined in the annotation file described below.

Passage buildings are an intentional exception to normal city consistency:
they are absent from `generated_city.map2d`. Their footprint therefore remains
free in the static 2D map, allowing A* to retain the normal corridor through
that location. Adding the connector footprint to the static map would make A*
route around the whole building and would prevent passage traversal.

All ordinary buildings must still be represented consistently in both the SDF
world and the static map. Do not use this exception for ordinary solid
buildings.

## Annotation File And Synchronization Contract

The drone's explicit knowledge is stored separately in:

```text
drone_city_nav/worlds/known_passages.passages3d
```

The line-based, versioned format is:

```text
drone_city_nav_known_passages_v1
frame_id map
structure <id> <center_x> <center_y> <size_x> <size_y> <z_min> <z_max>
opening <structure_id> <opening_id> <center_x> <center_y> <center_z> <normal_x> <normal_y> <width> <height> <depth> <min_z> <max_z> <approach_m> <exit_m>
```

An annotation contains:

- the physical SDF model identity;
- the structure footprint and full vertical range;
- opening center, XY normal, width, height, and depth;
- opening minimum and maximum altitude;
- approach and exit distances used by vertical planning.

This is currently a deliberately explicit duplication, not a generated
artifact: SDF is the physical-world definition and `.passages3d` is the drone
knowledge definition. Keep them synchronized whenever a passage is edited.
`scripts/tests/test_topic_contract.py` verifies every current connector against
its SDF model, including map-frame position, orientation, opening bounds,
lower/upper collision and visual dimensions, and the absence of connector ids
from `generated_city.map2d`.

The parser rejects malformed and unsafe annotations, including duplicate ids,
non-finite values, invalid dimensions, openings outside their structure
footprint, and opening altitude ranges outside the structure range.

## Planner Pipeline

The baseline navigation pipeline remains XY-owned:

```text
static map + memory + current lidar
-> prohibited grid
-> A*
-> corridor
-> trajectory optimizer
-> turn smoothing
-> isolated-spike geometry cleanup
```

Known-passage logic is layered on top of that established path:

```text
base XY trajectory
-> optional local passage insertion
-> vertical profile
-> XY speed profile with passage constraints
-> offboard velocity and vertical tracking
```

### Local XY Passage Insertion

`trajectory_passage_insertion` runs after the normal XY geometry stages and
before vertical and speed profiling. It is not a new global route planner and
does not make A* prefer openings.

If a final XY trajectory intersects a known building footprint but misses the
opening corridor, or crosses it with less than the configured preferred lateral
margin, the stage may construct a small smooth insertion:

```text
old trajectory
-> approach anchor
-> opening-aligned local segment
-> reconnect anchor
-> old trajectory
```

The candidate is accepted only when it remains traversable on the prohibited
grid, preserves mission endpoints, satisfies join tangent/curvature limits, and
improves opening alignment. It aims for sufficient clearance, not the opening
center. The center is only a fallback reference when no smaller safe shift can
be determined.

### Vertical Profile

`trajectory_vertical_profile` then writes a smooth `z(s)` value into the same
executable trajectory samples. It does not alter A*, the corridor, or the
global XY optimizer.

For each valid opening match it:

1. derives a hard altitude interval by subtracting
   `vertical_profile_gate_clearance_margin_m` from the physical opening bounds;
2. derives a preferred interval using
   `vertical_profile_preferred_gate_clearance_margin_m`;
3. retains the carried/current profile altitude when it is already within the
   preferred interval, otherwise clamps it to the nearest preferred boundary;
4. begins a smooth climb or descent before the opening;
5. holds the gate altitude before entry and through the opening;
6. carries the achieved altitude into the following trajectory rather than
   returning to a fixed cruise altitude after each opening.

The transition is constrained by vertical speed, acceleration, jerk, climb
angle, minimum/maximum transition distance, and a pre-gate hold distance. A
profile that cannot complete its transition before the required hold window is
infeasible and is not accepted as a valid planner result.

The annotation validation itself is diagnostic and repair input only. It reports
whether a planned span intersects a structure footprint through a sufficiently
deep, clear opening volume. It is not a collision engine and does not create a
special mission-failure rule.

### Speed And Runtime Tracking

The final path remains curvature-profiled in XY, but a known-passage hard
vertical window applies `known_passage_traversal_speed_limit_mps` to the scalar
speed. The default is 10 m/s.

At runtime the offboard follower receives the same samples with their `z_m`
values and computes both horizontal velocity and `vz`. If the actual altitude
cannot reach an upcoming hard window in time, vertical trackability temporarily
adds a lower horizontal speed cap. This is a last-resort tracking constraint,
not a replacement for the planner's pre-gate transition and hold.

The vertical follower is separately bounded by vertical speed, acceleration,
jerk, and climb-angle parameters. During terminal position capture, altitude is
latched from the current vehicle altitude; it is not restored to the old
initial altitude.

## Lidar And Obstacle Memory

The lidar is not disabled near an opening. It remains an ordinary dynamic
safety sensor, but known static passage masses are prevented from becoming
false dynamic obstacles.

For every usable lidar return, both ingestion paths perform the same process:

1. project the measurement as a map-frame 3D ray using vehicle position,
   roll, pitch, yaw, lidar mount orientation, and lidar offset;
2. construct known solid volumes from the annotation: `left_mass`,
   `right_mass`, `lower_mass`, and `upper_mass` as applicable;
3. query the nearest expected known solid and flat-ground intersections;
4. compare the measured and nearest expected range before writing the hit to either
   the current lidar overlay or accumulated obstacle memory.

The decision is asymmetric:

- a hit more than `known_static_lidar_hit_closer_range_tolerance_m` closer than
  the expected surface is retained as possible unknown geometry in front of the
  building;
- a confident face-interior hit no farther than
  `known_static_lidar_hit_farther_range_tolerance_m` behind the expected
  surface is an `expected_static` hit and is suppressed;
- a grazing/boundary intersection, a hit through an opening, missing geometry,
  invalid pose, or a return beyond the farther tolerance is ambiguous or
  unexpected and is retained by fail-open policy.

The defaults are 0.5 m for the stricter closer tolerance and 1.5 m for the
farther tolerance. The second allowance compensates for bounded SDF collision,
projection, and simulator timing disagreement without hiding an unknown object
that is actually in front of a known wall.

This classifier operates for every new scan, independent of the active route
or distance to an opening. It never edits static-map cells, never changes A*
preferences, and never removes arbitrary cells from obstacle memory. If known
passage geometry changes, obstacle memory is reset so existing 2D evidence and
its sparse 3D provenance cannot survive under a different geometry contract.

The flat-ground provider is independent of passage annotations. Expected and
ambiguous ground-facing beams create neither occupied evidence nor free-space
clearing. A hit clearly before both a ground and known-solid candidate remains
an obstacle. If the nearest candidates tie and the hit is not clearly before
all of them, the decision suppresses all grid mutation rather than selecting an
arbitrary provider.

## Collision, Mission Outcome, And Diagnostics

Gazebo SDF collision geometry is the physical source of truth: a vehicle can
pass through the empty opening but collides with `upper_mass` or `lower_mass`.
`mission_monitor_node` derives corresponding ordinary solid volumes from the
annotations for mission diagnostics. Its clearance estimate cannot command PX4
or stop the vehicle, and it does not use a special "opening id was passed"
rule to decide success or failure.

The following diagnostics make passage behavior inspectable:

- trajectory diagnostics: opening overlap, depth fraction, lateral/vertical
  clearance, local insertion candidates, and vertical-profile windows;
- trajectory CSV: `z_m`, vertical constraints, safe altitude interval, and
  associated opening id;
- offboard blackbox: target altitude, vertical command, vertical trackability
  cap, and final scalar speed;
- lidar/current-overlay and obstacle-memory logs: expected, unexpected, and
  ambiguous known-static hits;
- prohibited-intersection logs: bounded `known_static_hit` provenance with
  grid cell, endpoint XYZ, measured range, expected range, range delta, and
  matched building part when available.

The RViz `Known Passages` display is an annotation/debug layer, not the Gazebo
world renderer. It shows architectural mass markers, opening frame and center,
and approach/exit direction. RViz uses a visualization-specific `map` to
`gazebo_map` transform; publishers compensate visual Z so 3D annotations and
the executable path line up with the Gazebo scene. Never apply that visual Z
compensation to planner or offboard control coordinates.

## Current Scope And Limitations

- Passages are pre-annotated. The system does not detect arbitrary openings
  from lidar, depth, RGB, or a 3D occupancy map.
- The primary planner is still a 2D XY planner. Passage insertion and vertical
  profiling are constrained local 3D additions, not general 3D free-space
  search.
- Static-map free space through a passage is intentional. Physical collision,
  annotation geometry, lidar filtering, and trajectory constraints must remain
  synchronized for safe simulation behavior.
- When adding or moving a physical connector, update the SDF and
  `.passages3d` together, keep the connector out of `.map2d`, and run the
  passage contract tests before a simulation run.
