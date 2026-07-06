# Context

Планируем этап **Known Passage Annotation Map** для фичи **3D passage traversal**.
На этом этапе дрон только знает о заранее описанных 3D passages и показывает их в
диагностике/RViz. Полёт через passage, vertical profile, validation, lidar policy и
local passage insertion сейчас не реализуются.

Ключевое решение после локального discovery: не добавлять `yaml-cpp` и не
парсить отдельный YAML вручную. В проекте уже есть собственный line-based формат
для static map (`static_city_map.cpp`), а ROS-параметры читаются через
`declare_parameter()`. Поэтому для первого этапа делаем простой версионированный
файл `known_passages.passages3d`, а путь к нему задаём в существующем
`urban_mvp.yaml`.

Поведение planner/control должно остаться прежним:

- passage structures не добавляются в `prohibited_grid`;
- A*/corridor/trajectory optimizer/speed profile не получают новых constraints;
- offboard не меняет управление;
- новый слой только грузится, логируется и публикуется как RViz `MarkerArray`.

# Investigation Context

`INVESTIGATION.md` отсутствует.

Изученные локальные точки:

- `TrajectoryPointSample` уже содержит `z_m`: `drone_city_nav/include/drone_city_nav/trajectory.hpp:54`.
- ROS `Path` уже умеет per-sample altitude через `pathToRos(samples, header)`: `drone_city_nav/src/planner_node_debug_publication.cpp:97`.
- Final trajectory debug markers уже рисуются на `sample.z_m`: `drone_city_nav/src/trajectory_debug_markers.cpp:103`.
- Drone marker уже 3D sphere на текущей высоте: `drone_city_nav/src/offboard_debug_markers.cpp:19`.
- Static map parser уже использует простой версионированный text format: `drone_city_nav/src/static_city_map.cpp:15`.
- Static map debug already publishes grid + point cloud from planner node: `drone_city_nav/src/planner_node_debug_publication.cpp:18`.
- Planner node publishers создаются в lifecycle ctor: `drone_city_nav/src/planner_node_lifecycle.cpp:36`.
- Planner config/topic loading centralised in `loadPlannerNodeConfig()`: `drone_city_nav/src/planner_node_config.cpp:18`.
- RViz config already contains Path/MarkerArray/PointCloud displays: `drone_city_nav/rviz/city_nav_debug.rviz:47`.
- CMake splits pure core and ROS adapters: `drone_city_nav/CMakeLists.txt:29`, `drone_city_nav/CMakeLists.txt:96`.

# Detected Stack/Profiles

- Project stack: ROS 2 workspace, C++20, ament CMake, `colcon`, PX4/Gazebo simulation.
- Applied project profiles:
  - `generic.md`: mandatory for command discovery and reporting.
  - `cpp.md`: selected because the package has `drone_city_nav/CMakeLists.txt`, `.cpp/.hpp` sources, ament CMake targets, and C++ tests.
- Notion policy is `optional`; user prompt does not mention a Notion task id, so Notion is not required.
- GitLab protocol was read; user prompt does not mention MR/GitLab, so no GitLab read is required.

# Repo-Approved Commands Found

Repository instructions require container-only workflow:

- Build: `./scripts/build.sh`
- Full C++ test target: `./scripts/test.sh`
- Interactive container entry: `./scripts/dev_shell.sh`
- Inside container:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
- Before commit after C++ changes:
  1. `./scripts/dev_shell.sh make format`
  2. `./scripts/dev_shell.sh make quality`
  3. local `git commit`

# Affected Components

- New known passage model/parser:
  - `drone_city_nav/include/drone_city_nav/known_passage_map.hpp`
  - `drone_city_nav/src/known_passage_map.cpp`
  - `drone_city_nav/tests/known_passage_map_test.cpp`
- New known passage debug markers:
  - `drone_city_nav/include/drone_city_nav/known_passage_debug_markers.hpp`
  - `drone_city_nav/src/known_passage_debug_markers.cpp`
  - `drone_city_nav/tests/known_passage_debug_markers_test.cpp`
- Planner config and diagnostics-only publication:
  - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:16`
  - `drone_city_nav/src/planner_node_config.cpp:18`
  - `drone_city_nav/src/planner_node.hpp:93`
  - `drone_city_nav/src/planner_node_lifecycle.cpp:5`
  - `drone_city_nav/src/planner_node_debug_publication.cpp:18`
- Default config/artifact/RViz:
  - `drone_city_nav/config/urban_mvp.yaml:42`
  - `drone_city_nav/worlds/known_passages.passages3d`
  - `drone_city_nav/rviz/city_nav_debug.rviz:56`
  - `scripts/record_debug_bag.sh:9`
- Build/test wiring:
  - `drone_city_nav/CMakeLists.txt:29`
  - `drone_city_nav/CMakeLists.txt:96`
  - `drone_city_nav/CMakeLists.txt:320`
- Docs/contracts:
  - `docs/navigation_pipeline.md:97`
  - `docs/rviz.md:34`
  - `docs/configuration.md`
  - `scripts/tests/test_topic_contract.py:42`

# Implementation Steps

1. Add pure C++ known passage data model and parser.

   Files:
   - Add `drone_city_nav/include/drone_city_nav/known_passage_map.hpp`.
   - Add `drone_city_nav/src/known_passage_map.cpp`.
   - Add source to `drone_city_nav_core` in `drone_city_nav/CMakeLists.txt:29`.

   Expected result:
   - A parsed `KnownPassageMap` contains `frame_id`, `structures`, and nested
     `openings`.
   - Parser is deterministic, dependency-free, and follows the existing
     `static_city_map.cpp` style: comments, strict version, strict trailing-token
     rejection, duplicate-id rejection, finite/positive validation.

   Proposed public types:

   ```cpp
   struct PassageOpening {
     std::string id;
     std::string structure_id;
     Point3 center{};
     Point2 normal_xy{1.0, 0.0};
     double width_m{0.0};
     double height_m{0.0};
     double depth_m{0.0};
     double min_z_m{0.0};
     double max_z_m{0.0};
     double approach_distance_m{0.0};
     double exit_distance_m{0.0};
   };

   struct PassageStructure {
     std::string id;
     Point2 center{};
     double size_x_m{0.0};
     double size_y_m{0.0};
     double z_min_m{0.0};
     double z_max_m{0.0};
     std::vector<PassageOpening> openings;
   };

   struct KnownPassageMap {
     std::string frame_id;
     std::vector<PassageStructure> structures;
   };
   ```

   Proposed file format:

   ```text
   drone_city_nav_known_passages_v1
   frame_id map
   structure arch_01 120.0 180.0 14.0 8.0 0.0 40.0
   opening arch_01 main 120.0 180.0 12.0 1.0 0.0 8.0 5.0 4.0 9.5 14.5 18.0 18.0
   ```

   Meaning:
   - `structure <id> <center_x> <center_y> <size_x> <size_y> <z_min> <z_max>`
   - `opening <structure_id> <opening_id> <center_x> <center_y> <center_z> <normal_x> <normal_y> <width> <height> <depth> <min_z> <max_z> <approach_m> <exit_m>`

   Validation rules:
   - `frame_id` required and unique.
   - `structure` id unique.
   - `opening` id unique within its structure.
   - all sizes/distances positive except `z_min` may be `0`;
   - `z_max > z_min`;
   - opening `min_z/max_z` inside structure `z_min/z_max`;
   - `normal_xy` finite and normalizable;
   - opening center XY must be inside structure footprint;
   - unknown keywords throw with line number.

2. Add source/path loader similar to static map source.

   Files:
   - Add `KnownPassageSourceConfig`, `KnownPassageSourceStatus`, and
     `KnownPassageSourceResult` either in `known_passage_map.hpp` or a small
     `known_passage_source.hpp` if the source wrapper grows.
   - Implement `resolveKnownPassageMapPath()` parallel to
     `resolveStaticMapPath()` from `drone_city_nav/src/static_map_source.cpp:10`.
   - Keep this pure C++ if it only uses `std::filesystem`.

   Expected result:
   - Disabled source returns status `kDisabled`.
   - Missing/invalid file returns `kLoadFailed` with error message.
   - Frame mismatch is reported but does not affect planner behavior in this
     diagnostics-only phase.

   Pseudocode:

   ```cpp
   KnownPassageSourceResult loadKnownPassageMapSource(config) {
     result.resolved_path = resolveKnownPassageMapPath(...);
     if (!config.enabled) return disabled;
     try {
       result.map = loadKnownPassageMap(result.resolved_path);
       result.frame_matches = result.map.frame_id == config.expected_frame_id;
       result.status = kLoaded;
     } catch (const std::exception& e) {
       result.status = kLoadFailed;
       result.error_message = e.what();
     }
     return result;
   }
   ```

3. Add planner config parameters and defaults.

   Files:
   - Update `PlannerTopics` in `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:16`.
   - Update `PlannerNodeConfig` in `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:55`.
   - Update `loadPlannerNodeConfig()` in `drone_city_nav/src/planner_node_config.cpp:18`.
   - Update `drone_city_nav/config/urban_mvp.yaml:42`.
   - Update `drone_city_nav/tests/planner_node_config_test.cpp:43`.
   - Update `drone_city_nav/tests/px4_offboard_config_test.cpp:9` to assert YAML contains the new diagnostics-only parameters.

   New ROS params under `planner_node.ros__parameters`:

   ```yaml
   known_passages_enabled: true
   known_passages_path: worlds/known_passages.passages3d
   known_passage_markers_topic: /drone_city_nav/known_passage_markers
   known_passage_debug_publish_period_s: 1.0
   ```

   Expected result:
   - Code defaults and YAML defaults match.
   - Config clamp/sanitize rules:
     - publish period clamped `[0.0, 60.0]`;
     - empty path allowed but will resolve to default `worlds/known_passages.passages3d`;
     - topic string follows the existing topic loading style.

4. Add default known passage annotation fixture.

   Files:
   - Add `drone_city_nav/worlds/known_passages.passages3d`.
   - No CMake install change is needed for this file if it lives under
     `worlds/`, because `drone_city_nav/CMakeLists.txt:223` already installs
     the whole `worlds` directory.

   Expected result:
   - The default file is valid and small.
   - It may contain one example passage or only comments plus a valid empty map.
     Prefer one example only if it matches current/generated world geometry well
     enough for visualization; otherwise use a valid empty file to keep runtime
     diagnostics clean.

   Valid empty file:

   ```text
   drone_city_nav_known_passages_v1
   frame_id map
   ```

5. Add RViz marker builder for passage structures/openings.

   Files:
   - Add `drone_city_nav/include/drone_city_nav/known_passage_debug_markers.hpp`.
   - Add `drone_city_nav/src/known_passage_debug_markers.cpp`.
   - Add source to `drone_city_nav_ros_adapters` in `drone_city_nav/CMakeLists.txt:96`.
   - Add `drone_city_nav/tests/known_passage_debug_markers_test.cpp`.

   Expected result:
   - `buildKnownPassageDebugMarkers(header, map)` returns:
     - transparent structure volume marker, namespace `known_passage_structure`;
     - opening frame `LINE_LIST`, namespace `known_passage_opening_frame`;
     - gate center `SPHERE`, namespace `known_passage_gate_center`;
     - approach/exit `ARROW` markers, namespaces `known_passage_approach` and
       `known_passage_exit`.
   - Empty/disabled map publishes `DELETEALL` or stable delete markers so stale
     RViz objects disappear.
   - Marker ids are stable and deterministic from iteration order.

   Marker geometry contract:

   ```cpp
   // opening local frame
   normal = normalized(opening.normal_xy);
   lateral = {-normal.y, normal.x};
   half_width = opening.width_m / 2;
   z_low = opening.min_z_m;
   z_high = opening.max_z_m;
   depth_half = opening.depth_m / 2;
   entry_center = opening.center.xy - normal * depth_half;
   exit_center = opening.center.xy + normal * depth_half;
   ```

   Unit tests should assert marker count, namespaces, types, header frame, center
   z, frame corners z, and approach/exit direction.

6. Integrate diagnostics-only loading and publication into `PlannerNode`.

   Files:
   - Update `drone_city_nav/src/planner_node.hpp:93` with:
     - loaded `KnownPassageMap` state;
     - resolved path/status counters;
     - `known_passage_markers_pub_`;
     - timer or reuse static-map debug timer.
   - Update publisher creation in `drone_city_nav/src/planner_node_lifecycle.cpp:36`.
   - Add `loadConfiguredKnownPassages()` near `loadConfiguredStaticMap()` in
     `planner_node_lifecycle.cpp` or a new `planner_node_passages.cpp` if this
     keeps file size lower.
   - Add `publishKnownPassageDebug()` near
     `PlannerNode::publishStaticMapDebug()` in
     `drone_city_nav/src/planner_node_debug_publication.cpp:18`.
   - Add the new source file to the `planner_node` executable in
     `drone_city_nav/CMakeLists.txt:123` if a separate `.cpp` is created.

   Expected result:
   - Startup logs are useful in headless mode:

     ```text
     Known passage map: enabled=true path='...' status=loaded frame='map'
     structures=1 openings=1 markers_topic='/drone_city_nav/known_passage_markers'
     ```

   - Load failure warning includes resolved path and parser error.
   - Frame mismatch warning includes map frame and planner frame.
   - This code must not feed passage structures into:
     - `static_grid_`;
     - `PlanningGridBuilder`;
     - `PlannerCore`;
     - `publishPath()` / `publishTrajectoryPath()`.

7. Update RViz, rosbag/debug artifacts, and docs.

   Files:
   - Add MarkerArray display to `drone_city_nav/rviz/city_nav_debug.rviz:56`
     for `/drone_city_nav/known_passage_markers`.
   - Add `/drone_city_nav/known_passage_markers` to
     `scripts/record_debug_bag.sh:9` so headless debug bags capture it.
   - Update `docs/overview.md:71`, `docs/navigation_pipeline.md:97`,
     `docs/rviz.md:34`, and `docs/configuration.md`.
   - Optionally add `docs/known_passages.md` if the format description is too
     large for `configuration.md`; link it from `docs/overview.md`.

   Expected result:
   - Docs explicitly state this phase is diagnostics-only.
   - Use the terms `known passage`, `passage structure`, `opening`, and `gate`.
     Do not use `holes` in code/docs.
   - RViz interpretation tells developers how to distinguish structure volume,
     opening frame, gate center, approach arrow, and exit arrow.

8. Add script-level/topic contract tests where useful.

   Files:
   - Update or add a Python test under `scripts/tests/`, likely
     `scripts/tests/test_topic_contract.py:42` or a new RViz contract test.

   Expected result:
   - The default YAML contains `known_passage_markers_topic`.
   - The RViz config subscribes to `/drone_city_nav/known_passage_markers`.
   - The rosbag script records `/drone_city_nav/known_passage_markers`.

9. Update CMake tests and run targeted checks.

   Files:
   - Add `known_passage_map_test` under the `if(BUILD_TESTING)` block in
     `drone_city_nav/CMakeLists.txt:320`.
   - Add `known_passage_debug_markers_test` near
     `trajectory_debug_markers_test` in `drone_city_nav/CMakeLists.txt:335`.

   Expected result:
   - New tests build in the approved `colcon` workflow.
   - Existing marker/config/static-map tests continue to pass.

# Verification Plan

Minimum targeted verification after implementation:

```bash
./scripts/dev_shell.sh make format
./scripts/build.sh
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R 'known_passage|planner_node_config|trajectory_debug_markers|static_map_debug'
./scripts/dev_shell.sh make test-scripts
./scripts/dev_shell.sh make quality
```

If only `PLAN.md` changes in this planning round, no build/test is required, but
a local commit is still required because `PLAN.md` is git-tracked once created.

No simulation is required for this stage. A later implementation PR may choose a
manual RViz/Gazebo smoke check, but it should not replace unit/script tests.

# Testing Strategy

## Category 1: No Refactoring / Pure Additions

Parser tests in `known_passage_map_test.cpp`:

- happy path: valid file with one structure and one opening;
- empty valid map: version + frame only;
- comments and blank lines;
- duplicate structure id rejected;
- duplicate opening id in same structure rejected;
- opening references missing structure rejected;
- non-finite/negative/zero dimensions rejected;
- opening outside footprint rejected;
- opening z range outside structure rejected;
- unknown keyword/trailing token rejected with exception.

Marker tests in `known_passage_debug_markers_test.cpp`:

- structure volume marker uses expected namespace/type/alpha;
- opening frame has line-list points at expected `z`;
- gate center sphere is at `center_xyz`;
- approach/exit arrows are opposite along `normal_xy`;
- empty/disabled marker output deletes stale markers.

Config tests:

- `planner_node_config_test.cpp` default topic/path/enabled/period;
- clamp test for invalid publish period;
- override test for custom path/topic.

Script tests:

- default YAML contains new params;
- RViz config contains marker display topic;
- bag script records marker topic.

## Category 2: Light Refactoring

Planner node may need a small `planner_node_passages.cpp` to avoid growing
`planner_node_lifecycle.cpp` or `planner_node_debug_publication.cpp` too much.
Tests remain unit/config-level; no integration simulation needed.

Important assertions:

- passage map loading does not mutate `static_grid_`;
- `publishKnownPassageDebug()` is independent from path publication;
- marker publication can be disabled by `known_passages_enabled=false` or
  `known_passage_debug_publish_period_s=0.0`.

## Category 3: Heavy / Deferred

Not in this stage:

- vertical profile generation;
- passage validation against trajectory;
- local XY insertion through gate center;
- speed caps from vertical motion;
- lidar policy changes inside known passage structures;
- executable trajectory artifact changes.

These require separate tests for path validity, control behavior, speed profile,
and replanning. Do not mix them into this diagnostics-only stage.

# Risks and Tradeoffs

- Custom `.passages3d` format is less familiar than YAML, but avoids adding
  parser dependencies and matches the existing static map parser style.
- Passage structures are diagnostics-only; users may expect them to affect
  planning. Logs/docs must explicitly say they do not yet affect planner/control.
- RViz marker geometry can be visually misleading if marker orientation/frame
  math is wrong. Unit tests should validate actual marker coordinates.
- Marker topic adds a small amount of runtime publication work. Use
  transient-local QoS and configurable publish period.
- If the default passage file contains an example that does not match the Gazebo
  world, RViz will confuse debugging. Prefer a valid empty default unless a real
  matching fixture is added.
- Future stages must not treat passage buildings as absent from all collision
  logic. This stage only creates the annotation source of truth for later
  validation.

# Open Questions

- No blocking questions for this stage.
- Non-blocking choice for the implementer: default `known_passages.passages3d`
  can be either a valid empty map or one real example passage. Prefer valid empty
  unless a matching Gazebo/world fixture is created in the same change.
- The exact visual style of structure volume markers can be tuned during
  implementation, but the marker namespaces and geometric contracts above should
  stay stable for tests and RViz debugging.
