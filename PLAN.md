# Context

The task is to plan fixes for three local review reports about the Gazebo + ROS 2
+ PX4 drone navigation MVP. The reviewed issues are concentrated in navigation
safety, planning cost models, obstacle-map handling, validation scripts, and test
coverage. This round is a planning round only: no production code is changed.

The user requires every new or touched feature to be covered by enough automated
tests and logging to support headless debugging. The repository rule also
requires code comments and documentation to stay in English.

# Investigation context

No `INVESTIGATION.md` exists in the workspace at the time of this plan. The plan
is based on the local review text from `.agent-io/inbox.txt`, repository docs,
build files, and direct inspection of the affected source and test files.

Notion access was not used because the orchestration policy is `optional` and the
prompt does not name a Notion task id. GitLab access was not used because the
prompt contains pasted review findings but no GitLab MR id, MR URL, or project
remote target to read. The remote-access policy for this workflow also forbids
SSH and HTTP access to remote targets.

# Detected stack/profiles

- Primary stack: C++ ROS 2 workspace with an ament CMake package
  (`drone_city_nav`) built through `colcon`.
- Simulator/control stack: Gazebo, PX4 SITL, ROS 2 nodes, Python validation
  scripts.
- Relevant orchestrator profiles read and applied:
  - `project_profiles/generic.md`
  - `project_profiles/cpp.md`
- Rust profile was not applied: no workspace `Cargo.toml` or Rust source was
  found in the project scope.
- Build artifacts and compile databases already exist under `build-host/` and
  `build/`; do not introduce ad-hoc top-level CMake commands.

# Repo-approved commands found

Use these commands from the repository root:

- Native host build: `make host-build`
- Native host unit tests: `make host-test`
- Native host script tests: `make host-test-scripts`
- Native host quality gate: `make host-quality`
- Format changed C++ files only: `make host-format`
- Native headless simulation: `make host-sim-headless`
- Native GUI simulation: `make host-sim-gui`
- Native speed sweep: `make host-speed-sweep`

Container compatibility remains supported through `./scripts/dev_shell.sh`, then
`make build`, `make test`, `make test-scripts`, `make quality`,
`make sim-headless`, and `make sim-gui` inside the container. The native host
workflow is the default for agent work.

# Affected components

- Offboard PX4 safety and command generation:
  - `drone_city_nav/src/px4_offboard_node.cpp`
  - `drone_city_nav/src/offboard_speed_controller.cpp`
  - `drone_city_nav/src/offboard_path_follower.cpp`
  - `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
  - related tests under `drone_city_nav/tests/`
- A* and planner core:
  - `drone_city_nav/src/astar_planner.cpp`
  - `drone_city_nav/include/drone_city_nav/astar_planner.hpp`
  - `drone_city_nav/src/planner_core.cpp`
  - `drone_city_nav/src/path_smoothing.cpp`
  - `drone_city_nav/src/planner_node.cpp`
- Occupancy grid and obstacle memory:
  - `drone_city_nav/src/occupancy_grid.cpp`
  - `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`
  - `drone_city_nav/src/obstacle_memory.cpp`
  - `drone_city_nav/src/obstacle_memory_node.cpp`
  - `drone_city_nav/src/planner_node_config.cpp`
- Planning-grid source selection:
  - `drone_city_nav/src/planning_grid_builder.cpp`
  - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
- Lidar projection and debug visualization:
  - `drone_city_nav/src/lidar_projection.cpp`
  - `drone_city_nav/src/lidar_debug_node.cpp`
  - `drone_city_nav/src/lidar_debug_renderer.cpp`
  - `drone_city_nav/include/drone_city_nav/lidar_debug_renderer.hpp`
- Mission validation and monitoring:
  - `scripts/validate_city_mvp_headless.py`
  - `scripts/tests/test_validate_city_mvp_headless.py`
  - `drone_city_nav/src/mission_monitor_node.cpp`
- Pose freshness and transforms:
  - `drone_city_nav/src/navigation_pose.cpp`
  - `drone_city_nav/src/lidar_projection.cpp`
  - `drone_city_nav/launch/city_nav.launch.py`

# Implementation steps

1. Fix the clearance-escape deadlock in offboard target safety.

   Files:
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/include/drone_city_nav/offboard_target_safety.hpp`
   - `drone_city_nav/src/offboard_target_safety.cpp`
   - `drone_city_nav/tests/offboard_target_safety_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Materialized result:
   - Extract target-segment safety policy into a ROS-free core module that can be
     tested directly.
   - Allow escape when the same continuous clearance metric that stopped the
     speed controller indicates a clearance stop, not only when the current grid
     cell is already inflated-blocked.
   - Keep the hard rule that escape must not cross real occupied cells.
   - Add throttled logs for rejected escape candidates with reason,
     `start_clearance_m`, `end_clearance_m`, `start_blocked`,
     `start_occupied`, and whether clearance improved.

2. Fail closed on stale PX4 local position.

   Files:
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/tests/offboard_target_safety_test.cpp` or a new focused
     offboard state helper test if extraction requires it.

   Materialized result:
   - Add `max_pose_staleness_s` with bounded sanitization.
   - Track the last valid PX4 local-position update time.
   - Clear or ignore `local_position_valid_` when updates become stale.
   - Hold position and log a clear warning instead of publishing trajectory
     setpoints from stale coordinates.

3. Make clearance slowdown consistent with inflated safety cells.

   Files:
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/tests/offboard_speed_controller_test.cpp`
   - new or extracted tests for local clearance estimation.

   Materialized result:
   - `estimateLocalClearanceM()` treats inflated cells (`>=80`) as safety
     clearance blockers when feeding speed limits, matching the target-segment
     safety gate and docs.
   - Tests cover occupied cells, inflated cells, unknown/free cells, and the
     boundary where clearance braking starts.

4. Sanitize command resend timing.

   Files:
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - an extracted parameter/config test if the parameter reader is made
     testable.

   Materialized result:
   - Clamp `command_resend_period_s` to a finite positive range consistent with
     other timing parameters.
   - Log when an invalid value is replaced by the default or clamp boundary.

5. Fix `--allow-mission-failure` semantics in the headless validator.

   Files:
   - `scripts/validate_city_mvp_headless.py`
   - `scripts/tests/test_validate_city_mvp_headless.py`

   Materialized result:
   - Mission failure and PX4 critical-error checks fail only when
     `allow_mission_failure` is false.
   - `--mission-check --allow-mission-failure` still parses and reports mission
     status, but returns success for exploratory sweeps that explicitly allow
     mission failure.
   - Missing log files produce a structured validation failure or warning rather
     than an uncaught `FileNotFoundError`.

6. Harden occupancy-grid and obstacle-memory configuration.

   Files:
   - `drone_city_nav/src/occupancy_grid.cpp`
   - `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/src/obstacle_memory.cpp`
   - `drone_city_nav/src/obstacle_memory_node.cpp`
   - `drone_city_nav/tests/obstacle_memory_test.cpp`
   - `drone_city_nav/tests/planner_node_config_test.cpp`

   Materialized result:
   - Add shared validation for finite positive resolution, finite positive
     dimensions, and a documented maximum cell count.
   - Prevent division by zero, invalid casts, and huge accidental allocations.
   - Validate score ordering as
     `min_score <= free_score < occupied_score <= max_score`.
   - Log rejected or clamped grid parameters with exact values.

7. Fix A* state-index overflow and expose budget exhaustion distinctly.

   Files:
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/include/drone_city_nav/astar_planner.hpp`
   - `drone_city_nav/tests/planner_core_test.cpp` or a new
     `drone_city_nav/tests/astar_planner_test.cpp`

   Materialized result:
   - Store parent state indices as `std::size_t` with an explicit sentinel.
   - Guard state-space size before allocation or parent indexing.
   - Add an `AStarStatus` result that distinguishes success, unreachable target,
     invalid start/goal, and expansion-budget exceeded.
   - Update planner logs to include the status.

8. Replace the approximate clearance BFS with a reusable metric clearance field.

   Files:
   - `drone_city_nav/include/drone_city_nav/clearance_field.hpp`
   - `drone_city_nav/src/clearance_field.cpp`
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/src/planner_core.cpp`
   - `drone_city_nav/src/path_smoothing.cpp`
   - `drone_city_nav/tests/clearance_field_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Materialized result:
   - Build one reusable clearance-distance field in meters, using weighted
     8-neighbor propagation or another documented metric approximation.
   - Use the field for A* clearance cost, planner path-clearance diagnostics,
     and smoothing line-of-sight clearance checks.
   - Remove duplicated brute-force nearest-obstacle scans where practical.
   - Add comments explaining the chosen metric and its tradeoff.

9. Make the soft clearance penalty piecewise and length-aware.

   Files:
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/include/drone_city_nav/astar_planner.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/tests/planner_core_test.cpp`

   Materialized result:
   - Keep hard inflation as the blocked safety zone.
   - Apply zero soft penalty beyond a configurable comfort radius.
   - Apply smooth penalty only between safety and comfort radii, based on real
     occupied cells rather than inflated cells.
   - Preserve physical distance as the primary path cost so a much longer outer
     route is not selected only because it is wider.
   - Document the intentional non-admissible heuristic tradeoff when clearance
     or turn penalties are enabled.

10. Make path smoothing respect the same safety model.

    Files:
    - `drone_city_nav/src/path_smoothing.cpp`
    - `drone_city_nav/tests/planner_core_test.cpp`

    Materialized result:
    - Ensure smoothing does not cut across occupied or inflated cells.
    - Use the shared clearance field or shared line-safety helper.
    - Tune `path_smoothing_min_obstacle_clearance_m` so smoothing does not push
      routes unnecessarily toward the map boundary.

11. Prevent path-follower backtracking on self-intersecting paths.

    Files:
    - `drone_city_nav/src/offboard_path_follower.cpp`
    - `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
    - `drone_city_nav/tests/offboard_path_follower_test.cpp`

    Materialized result:
    - Search closest projections from the current waypoint/progress lower bound
      instead of across the whole path.
    - Keep forward progress monotonic unless the planner publishes a new path.
    - Add tests for self-intersecting and looped paths where the nearest segment
      behind the drone is geometrically closer than the correct forward segment.

12. Validate every actual command segment before target publication.

    Files:
    - `drone_city_nav/src/px4_offboard_node.cpp`
    - `drone_city_nav/src/offboard_target_safety.cpp`
    - `drone_city_nav/tests/offboard_target_safety_test.cpp`

    Materialized result:
    - Check `current_position -> target` and target-advance segments against
      occupied and inflated cells on every control update.
    - If the drone starts inside inflated space, allow only escape segments that
      avoid occupied cells and increase clearance or leave the inflated zone.
    - Publish hold-position setpoints and log the rejection reason when no safe
      segment exists.

13. Improve replan diagnostics without making replanning trigger-happy.

    Files:
    - `drone_city_nav/src/planner_node.cpp`
    - possible extracted helper under `drone_city_nav/include/drone_city_nav/`
      and `drone_city_nav/src/`

    Materialized result:
    - Log every path decision as one of: initial plan, keep current path,
      blocked-by-new-obstacle, no-path, fallback, or escape.
    - Include path waypoint count, segment count, total length, direct distance,
      minimum clearance, A* status, expansion count, and obstacle-source flags.
    - Preserve the current policy that a path is reused unless newly discovered
      obstacles intersect it.

14. Include current lidar bounds in planning-grid source selection.

    Files:
    - `drone_city_nav/src/planning_grid_builder.cpp`
    - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
    - `drone_city_nav/tests/planning_grid_builder_test.cpp`

    Materialized result:
    - `selectPlanningGridBounds()` can choose a valid current-lidar-only grid
      when static and memory grids are unavailable.
    - Add a lidar-only test where fallback bounds differ from the lidar grid and
      the lidar source is still used correctly.

15. Render occupancy-grid cells in lidar debug images.

    Files:
    - `drone_city_nav/src/lidar_debug_renderer.cpp`
    - `drone_city_nav/include/drone_city_nav/lidar_debug_renderer.hpp`
    - `drone_city_nav/tests/lidar_debug_renderer_test.cpp`

    Materialized result:
    - Draw occupancy-grid cells from `GridImageView::cells` before lidar hit
      overlays.
    - Use distinct colors for real occupied, inflated, remembered lidar hits,
      current lidar hits, path, and drone marker.
    - Add pixel-level renderer tests proving grid cells are visible in snapshots.

16. Bound lidar debug hit-memory carrier maps.

    Files:
    - `drone_city_nav/src/lidar_debug_node.cpp`
    - focused helper tests if pruning is extracted.

    Materialized result:
    - Enforce `max_remembered_hit_points_` or a separate carrier-map cap on
      `hit_candidates_` and `remembered_hit_cells_`, not only on the published
      vector.
    - Log pruning counts at a throttled rate for long headless runs.

17. Clarify mission-monitor footprint/volume parameter behavior.

    Files:
    - `drone_city_nav/src/mission_monitor_node.cpp`
    - new or existing mission-monitor tests if a pure parser is extracted.

    Materialized result:
    - Avoid warning about trailing `building_footprints` values when
      `building_volumes` intentionally takes precedence.
    - Log the selected source and number of parsed buildings.

18. Bound future timestamp freshness.

    Files:
    - `drone_city_nav/src/navigation_pose.cpp`
    - a new `drone_city_nav/tests/navigation_pose_test.cpp` or an existing
      relevant test file.

    Materialized result:
    - Reject future stamps beyond a small configurable or documented tolerance,
      or explicitly document and test why future stamps are accepted.
    - Log clock-domain mismatches when they affect pose freshness.

19. Add missing comments on robotics-frame and safety math.

    Files:
    - `drone_city_nav/src/astar_planner.cpp`
    - `drone_city_nav/src/offboard_speed_controller.cpp`
    - `drone_city_nav/src/lidar_projection.cpp`
    - `drone_city_nav/src/occupancy_grid.cpp`
    - `drone_city_nav/src/mission_monitor_node.cpp`
    - `drone_city_nav/launch/city_nav.launch.py`

    Materialized result:
    - Add short comments explaining direction-state A*, braking-distance
      limiting, FLU/FRD/NED transform conventions, inflation center-vs-edge
      adjustment, signed clearance semantics, and the Gazebo map transform.
    - Do not add narrative comments where the code is already self-explanatory.

20. Remove or wire dead command-smoothing helpers.

    Files:
    - `drone_city_nav/src/px4_offboard_node.cpp`
    - `drone_city_nav/src/offboard_path_follower.cpp`
    - `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
    - `drone_city_nav/tests/offboard_path_follower_test.cpp`

    Materialized result:
    - Either remove unused wrappers and stale tests, or intentionally wire the
      smoothing path into the command-generation flow with tests that exercise
      the active ROS-node path.
    - Prefer removal if the current segment-safety model supersedes the old
      helpers.

21. Rename misleading stable-path "occupied" terminology.

    Files:
    - `drone_city_nav/src/planner_core.cpp`
    - `drone_city_nav/include/drone_city_nav/planner_core.hpp`
    - `drone_city_nav/src/planner_node.cpp`
    - `drone_city_nav/tests/planner_core_test.cpp`

    Materialized result:
    - Rename functions and parameters that check inflated blocked cells but say
      "occupied" to `blocked` terminology.
    - Preserve behavior and update logs/tests to avoid tuning confusion.

22. Review cppcheck quality-gate behavior.

    Files:
    - `scripts/check_cpp_quality.sh`
    - no C++ files unless the script decision requires local annotations.

    Materialized result:
    - Decide whether informational `normalCheckLevelMaxBranches` should fail the
      whole quality gate.
    - If adjusted, keep actionable warnings as failures and document the reason
      in script comments or output.

# Verification plan

For this planning-only change:

1. `make host-format`
2. `make host-quality`
3. `make host-test-scripts`

For implementation batches, use the smallest safe scope first, then broaden:

1. Build and unit tests after each C++ batch:
   - `make host-build`
   - `ctest --test-dir build-host/drone_city_nav --output-on-failure -R <focused_test_regex>`
   - `make host-test` when shared planner/offboard contracts change.
2. Script validation changes:
   - `make host-test-scripts`
3. Quality gate before each commit:
   - `make host-format`
   - `make host-quality`
4. Headless simulation after safety/planning batches:
   - `make host-sim-headless`
   - no-static-map headless run with the documented environment toggles from
     `docs/MVP_SIMULATION.md`.
   - inspect generated logs for path status, segment count, speed, position,
     clearance, rejected target reasons, and mission result.
5. Optional compatibility check when a batch touches scripts or launch behavior
   used in the container:
   - `./scripts/dev_shell.sh`, then `make build`, `make test`, and
     `make sim-headless` inside the container.

Skipped checks for this planning-only round:

- Headless simulation is not run because no runtime code is changed by this
  plan.
- Container validation is not run because no container-specific behavior is
  changed by this plan.

# Testing strategy

1. No-refactor tests

   Use when the existing function is directly testable or the change is small:

   - Update `scripts/tests/test_validate_city_mvp_headless.py` for
     `mission_check + allow_mission_failure` and missing log files.
   - Add A* tests for `start == goal`, unreachable goal, blocked start/goal,
     budget exceeded, diagonal corner-cut, index-size guard, and status
     reporting.
   - Add `planning_grid_builder_test` coverage for current-lidar-only bounds.
   - Add `lidar_debug_renderer_test` pixel checks for grid-cell rendering.
   - Add obstacle-memory config tests for invalid score ordering, invalid
     dimensions, and cell-count caps.
   - Add speed-controller tests for clearance-tracking overspeed boundaries.

2. Light-refactor tests

   Use when the code is currently hidden inside ROS node methods but can be
   extracted without changing architecture:

   - Extract offboard target-segment safety into pure C++ and test escape,
     blocked, occupied, inflated, and clearance-stop cases.
   - Extract obstacle-memory/grid parameter normalization and test it without
     constructing full ROS nodes.
   - Extract mission-monitor footprint/volume parsing if needed, then test
     source precedence and warnings.
   - Add forward-only path-projection helpers and tests for self-intersecting
     paths.
   - Add a navigation-pose freshness helper test for stale and future stamps.

3. Heavy-refactor tests

   Use only when shared behavior becomes easier and safer after introducing a
   reusable module:

   - Introduce `ClearanceField2D` and test metric distances, diagonal behavior,
     reuse in A*, smoothing, and planner diagnostics.
   - Split large `planner_node.cpp` orchestration into a pure planning-cycle
     runner once safety fixes are stable.
   - Add integration-style headless assertions that parse logs for movement,
     clearance, path segment counts, target rejection reasons, and mission
     completion.

# Risks and tradeoffs

- Tightening stale-pose and segment-safety gates can convert silent unsafe
  behavior into visible holds. That is intended, but the logs must clearly show
  why the drone is holding.
- Treating inflated cells as speed-clearance blockers may reduce speed near
  buildings. The piecewise soft-penalty and escape logic should keep this from
  becoming a permanent deadlock.
- A metric clearance field is more correct than the current cell-count BFS but
  adds implementation complexity. Keep it isolated and benchmark with existing
  headless logs.
- A* with clearance and turn penalties is not strictly optimal under the simple
  Euclidean heuristic. This should be documented as an intentional tradeoff or
  addressed with a more conservative heuristic if optimality becomes necessary.
- Reusing current paths until newly discovered obstacles intersect them reduces
  oscillation, but stale obstacle-memory artifacts can still force conservative
  routes. Debug snapshots and pruning logs are important.
- Large refactors of `planner_node.cpp` should wait until the critical safety
  fixes and tests land, otherwise behavior changes will be difficult to isolate.
- Lower inflation or comfort penalties can make routes more direct, but the hard
  safety radius must remain a true no-fly zone unless a dedicated escape policy
  is active.

# Open questions

- What exact maximum cell count should be accepted for planning and obstacle
  memory grids on the target development machines?
- Should the clearance slowdown use inflated cells only, or the maximum of
  occupied-cell distance and inflated-cell distance with separate thresholds?
- What default future timestamp tolerance should be used for mixed ROS, PX4, bag
  replay, and simulator clock domains?
- Should `--allow-mission-failure` also downgrade PX4 critical errors to
  warnings, or should PX4 critical errors remain fatal even when mission failure
  is allowed?
- Should the old command-smoothing helpers be removed, or is there a future
  target-selection mode that still needs them?
- How strict should the no-static-map headless acceptance threshold be while the
  map is intentionally partial and lidar-driven?
