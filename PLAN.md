# Context

Нужно запланировать следующий этап развития racing trajectory:

- добавить RViz debug для скорости и кривизны финальной trajectory;
- проверить и санитизировать шипы кривизны на финальной trajectory;
- сделать `racing_line` optimizer time-aware через `weight_time`;
- использовать forward/backward speed estimate при оценке candidates;
- расширить diagnostics/CSV/blackbox так, чтобы в headless-режиме было видно, стала ли траектория реально быстрее;
- подготовить tuning-процесс без откладывания discovery на реализацию.

Итоговая цель этапа: optimizer выбирает траекторию по ожидаемому времени прохождения, а не только по геометрической гладкости.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. Существующего `PLAN.md` перед этим файлом не было, поэтому план создан заново.

По текущему коду уже есть базовые элементы, которые нужно развить, а не дублировать:

- `drone_city_nav/src/racing_line.cpp:215` считает geometric cost через length/curvature/smoothness/offset, но не учитывает traversal time.
- `drone_city_nav/src/racing_line.cpp:325` запускает `optimizeRacingLine(...)`; сейчас сигнатура получает только corridor/grid/racing config, без speed-profile config.
- `drone_city_nav/src/offboard_velocity_follower.cpp:180` считает geometric speed limit из curvature.
- `drone_city_nav/src/offboard_velocity_follower.cpp:249` применяет backward/forward pass к `TrajectorySpeedProfile`.
- `drone_city_nav/src/trajectory_planner.cpp:188` вызывает `optimizeRacingLine(...)`, затем `drone_city_nav/src/trajectory_planner.cpp:203` отдельно строит speed profile.
- `drone_city_nav/src/px4_offboard_node.cpp:690` уже публикует corridor markers.
- `drone_city_nav/src/px4_offboard_node.cpp:761` публикует offboard debug markers.
- `drone_city_nav/src/px4_offboard_node.cpp:808` считает `trajectoryShapeDiagnostics(...)`.
- `drone_city_nav/src/px4_offboard_node.cpp:1264` пишет CSV final trajectory samples.
- `drone_city_nav/src/px4_offboard_node.cpp:2728` пишет blackbox trajectory/speed diagnostics.

# Detected stack/profiles

- Стек: ROS 2 workspace, C++20, CMake/ament/colcon, запуск через Docker container workflow.
- Manifest/build files: `CMakeLists.txt`, `Makefile`, `package.xml`, `README.md`, `CONTRIBUTING.md`, `drone_city_nav/*.cpp/*.hpp`.
- Прочитаны mandatory profiles:
  - `generic.md`, потому что он обязателен для любого workspace;
  - `cpp.md`, потому что проект C/C++ и изменения будут в `.cpp/.hpp`.
- Rust profile не применялся: в целевом workspace задача не затрагивает Rust-проект и нет Rust build flow для `drone_city_nav`.
- Notion protocol прочитан; prompt не содержит Notion task id, `notion_policy=optional`, поэтому чтение Notion не требуется.
- GitLab protocol прочитан; prompt не содержит GitLab/MR запроса, поэтому `glab` не используется.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`, `AGENTS.md`:

- Top-level container wrappers:
  - `./scripts/build.sh`
  - `./scripts/test.sh`
  - `./scripts/sim_headless.sh`
  - `./scripts/sim_gui.sh`
  - `./scripts/stop_sim.sh`
- Interactive container shell only when needed:
  - `./scripts/dev_shell.sh`
- Inside container:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
  - `make sim-headless`
  - `make sim-gui`

For implementation commits after C++ changes:

1. Format changed C++ files with `make format` inside the container.
2. Run `make quality`.
3. Commit completed file changes.

Simulation should not be run automatically unless the user explicitly asks for it.

# Affected components

- `drone_city_nav/include/drone_city_nav/racing_line.hpp`
  - `RacingLineConfig`
  - `RacingLineStats`
  - `optimizeRacingLine(...)`
- `drone_city_nav/src/racing_line.cpp`
  - `costForPoints(...)`
  - `scoreForCandidate(...)`
  - `populateResultSamples(...)`
  - `optimizeRacingLine(...)`
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
  - `VelocityFollowerConfig`
  - `TrajectorySpeedSample`
  - `TrajectorySpeedProfile`
  - new traversal-time helper API
- `drone_city_nav/src/offboard_velocity_follower.cpp`
  - `geometricSpeedSampleFromPointSample(...)`
  - `finalizeSpeedProfile(...)`
  - `buildTrajectorySpeedProfile(...)`
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
  - `TrajectoryPlannerStats`
  - `TrajectoryPlannerConfig`
- `drone_city_nav/src/trajectory_planner.cpp`
  - `computeSpeedProfileStats(...)`
  - `planBaselineTrajectory(...)`
  - `planRacingTrajectory(...)`
- `drone_city_nav/src/px4_offboard_node.cpp`
  - parameter declaration around `racing_line_weight_*`
  - final trajectory debug publishing
  - `addCorridorDebugMarkers(...)`
  - `publishOffboardDebugMarkers(...)`
  - `writeFinalTrajectorySamplesCsvFile(...)`
  - blackbox writer
- `drone_city_nav/CMakeLists.txt`
  - `drone_city_nav_core` production source list
  - `px4_offboard_node` executable source list for ROS-message marker helpers
  - `ament_add_gtest(...)` blocks for every new test file
- `drone_city_nav/config/urban_mvp.yaml`
  - racing/time/debug parameters
- Tests:
  - `drone_city_nav/tests/racing_line_test.cpp`
  - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
  - `drone_city_nav/tests/trajectory_planner_test.cpp`
  - `drone_city_nav/tests/planner_node_config_test.cpp`
  - `drone_city_nav/tests/px4_offboard_config_test.cpp`
  - `drone_city_nav/tests/trajectory_diagnostics_test.cpp`
  - `drone_city_nav/tests/trajectory_debug_markers_test.cpp`
  - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`

# Implementation steps

1. Add reusable traversal-time estimation over trajectory samples.

   Files:
   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
   - `drone_city_nav/src/offboard_velocity_follower.cpp`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Code anchors:
   - `buildTrajectorySpeedProfile(std::span<const TrajectoryPointSample>, ...)`
   - `finalizeSpeedProfile(...)`
   - `TrajectorySpeedProfile`

   Materialized result:
   - Add a small result struct, for example:

     ```cpp
     struct TraversalTimeEstimate {
       bool valid{false};
       double estimated_time_s{0.0};
       double min_speed_limit_mps{0.0};
       double max_speed_limit_mps{0.0};
       std::size_t curvature_limited_samples{0U};
     };
     ```

   - Add a helper that reuses existing speed-profile logic:

     ```cpp
     [[nodiscard]] TraversalTimeEstimate estimateTraversalTime(
         std::span<const TrajectoryPointSample> samples,
         const VelocityFollowerConfig& config,
         bool use_forward_backward_profile);
     ```

   - For `use_forward_backward_profile=true`, call `buildTrajectorySpeedProfile(samples, config)` and integrate `dt = ds / max(eps, avg(v_i, v_j))`.
   - For `false`, use geometric limits only. This keeps item 4 explicit: geometric estimate first, profiled estimate available for optimizer.

2. Extend racing line config/stats for time-aware cost.

   Files:
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/tests/px4_offboard_config_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:
   - `RacingLineConfig`
   - `RacingLineStats`
   - `px4_offboard_node.cpp:386` existing `racing_line_weight_*` params
   - `urban_mvp.yaml:144` existing racing line weights

   Materialized result:
   - Add `double weight_time{0.0};` to `RacingLineConfig`.
   - Add final accepted trajectory stats. These are computed after regularization, so they describe the trajectory actually handed to the follower:
     - `estimated_time_s`
     - `min_speed_limit_mps`
     - `max_speed_limit_mps`
     - `curvature_limited_samples`
   - Add centerline baseline stats. These describe the unoptimized corridor centerline built from the same corridor samples:
     - `centerline_estimated_time_s`
     - `centerline_min_speed_limit_mps`
     - `centerline_max_speed_limit_mps`
     - `centerline_curvature_limited_samples`
   - Add best optimizer candidate stats before the regularization pass:
     - `best_candidate_estimated_time_s`
     - `best_candidate_score`
     - `best_candidate_min_speed_limit_mps`
     - `best_candidate_max_speed_limit_mps`
     - `best_candidate_curvature_limited_samples`
   - Add derived comparison stats:
     - `time_gain_s = centerline_estimated_time_s - estimated_time_s`
     - `regularization_time_delta_s = estimated_time_s - best_candidate_estimated_time_s`
   - Add parameter `racing_line_weight_time` with a conservative default. Recommended initial value: `0.05` or `0.1`, then tune from logs. If implementation wants zero-risk rollout, default can be `0.0`, but the planned stage goal is to enable a small nonzero value in `urban_mvp.yaml`.
   - Add `ament_add_gtest(px4_offboard_config_test tests/px4_offboard_config_test.cpp)` in `drone_city_nav/CMakeLists.txt`, with `rclcpp` dependency if the test instantiates parameter loading.

3. Pass speed-profile config into the racing optimizer.

   Files:
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`
   - `drone_city_nav/src/racing_line.cpp`
   - `drone_city_nav/src/trajectory_planner.cpp`
   - `drone_city_nav/tests/racing_line_test.cpp`
   - `drone_city_nav/tests/trajectory_planner_test.cpp`

   Code anchors:
   - `optimizeRacingLine(...)`
   - `planRacingTrajectory(...)`

   Materialized result:
   - Change optimizer signature from:

     ```cpp
     optimizeRacingLine(corridor_samples, grid, racing_config)
     ```

     to:

     ```cpp
     optimizeRacingLine(corridor_samples, grid, racing_config, speed_config)
     ```

   - `TrajectoryPlannerConfig` already has both `racing_line` and `speed_profile`, so `planRacingTrajectory(...)` can pass both without duplicating speed parameters in `RacingLineConfig`.
   - Update all unit tests/call sites to avoid hidden default behavior.

4. Add time-aware cost to candidate scoring.

   Files:
   - `drone_city_nav/src/racing_line.cpp`
   - `drone_city_nav/tests/racing_line_test.cpp`

   Code anchors:
   - `costForPoints(...)`
   - `scoreForCandidate(...)`
   - `evaluatePath(...)`

   Materialized result:
   - Convert candidate `points + offsets` to `TrajectoryPointSample` using the same sample construction rules used for final output.
   - Estimate traversal time for each candidate.
   - Add:

     ```cpp
     score += sanitized_weight_time * estimated_time.estimated_time_s;
     ```

   - Keep safety first: prohibited/outside-grid rejection remains a hard penalty/rejection before time can make a bad candidate look attractive.
   - Add a unit test with two valid candidates where one is slightly longer but has lower curvature; with `weight_time > 0`, optimizer must prefer the faster candidate.

5. Add forward/backward estimate mode to optimizer candidate evaluation.

   Files:
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`
   - `drone_city_nav/src/racing_line.cpp`
   - `drone_city_nav/tests/racing_line_test.cpp`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Code anchors:
   - `TraversalTimeEstimate`
   - `buildTrajectorySpeedProfile(...)`
   - `scoreForCandidate(...)`

   Materialized result:
   - Candidate scoring uses profiled speed by default: curvature limit plus backward/forward accel/decel pass.
   - If runtime cost is too high, add a clearly named config such as `racing_line_time_cost_uses_profiled_speed` only if needed. Preferred first implementation: always use profiled estimate so optimizer and follower evaluate the same physics.
   - Add tests:
     - high curvature reduces speed/time estimate;
     - final stop/backward pass increases estimated time versus geometric-only estimate;
     - acceleration limit reduces speed after a slow section.

6. Move or expose trajectory shape diagnostics so curvature sanity is testable outside `px4_offboard_node`.

   Files:
   - `drone_city_nav/include/drone_city_nav/trajectory_diagnostics.hpp`
   - `drone_city_nav/src/trajectory_diagnostics.cpp`
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:
   - current private `TrajectoryShapeDiagnostics` near `px4_offboard_node.cpp:137`
   - current `trajectoryShapeDiagnostics(...)` call at `px4_offboard_node.cpp:808`

   Materialized result:
   - Extract diagnostics into a reusable function:

     ```cpp
     [[nodiscard]] TrajectoryShapeDiagnostics
     computeTrajectoryShapeDiagnostics(std::span<const TrajectoryPointSample> samples);
     ```

   - Keep existing fields:
     - `max_curvature_jump_1pm`
     - `max_heading_delta_rad`
     - `max_offset_delta_m`
     - segment length stats
   - Add unit tests for a smooth arc and for an artificial curvature spike.
   - Add `src/trajectory_diagnostics.cpp` to `drone_city_nav_core` in `drone_city_nav/CMakeLists.txt`.
   - Add `ament_add_gtest(trajectory_diagnostics_test tests/trajectory_diagnostics_test.cpp)` linked to `drone_city_nav_core`.

7. Add trajectory-sample regularization for curvature spikes.

   Files:
   - `drone_city_nav/src/racing_line.cpp`
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`
   - `drone_city_nav/tests/racing_line_test.cpp`

   Code anchors:
   - `populateResultSamples(...)`
   - optimizer final candidate selection

   Materialized result:
   - Add a final regularization pass over racing offsets/samples, not over speed limits.
   - The pass must:
     - smooth offset second differences or neighboring points;
     - clamp each sample back inside corridor bounds;
     - rerun prohibited/outside-grid validation;
     - accept regularized samples only if they remain valid and do not worsen time/cost beyond a small tolerance.
   - Add stats:
     - `regularization_applied`
     - `regularization_iterations`
     - `pre_regularization_max_curvature_jump_1pm`
     - `post_regularization_max_curvature_jump_1pm`
   - Add a negative test: if regularization would cross prohibited cells, original valid samples are kept and a diagnostic flag explains why.

8. Add RViz final trajectory speed/curvature debug markers.

   Files:
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/include/drone_city_nav/trajectory_debug_markers.hpp`
   - `drone_city_nav/src/trajectory_debug_markers.cpp`
   - `drone_city_nav/tests/trajectory_debug_markers_test.cpp`
   - `drone_city_nav/CMakeLists.txt`
   - `drone_city_nav/config/urban_mvp.yaml`

   Code anchors:
   - `publishFinalTrajectoryDebugPath()`
   - `makeDebugMarker(...)`
   - `addCorridorDebugMarkers(...)`
   - `publishOffboardDebugMarkers()`

   Materialized result:
   - Keep current cyan `nav_msgs::Path` unchanged.
   - Add marker layers:
     - `final_trajectory_speed_colormap`: segment colors by `profiled_limit_mps`.
     - `final_trajectory_curvature_colormap`: segment colors by absolute curvature.
   - Use `visualization_msgs::msg::Marker::LINE_LIST` or segmented `LINE_STRIP` markers with per-point colors.
   - Publish on existing `offboard_debug_marker_topic` unless a separate topic is cleaner.
   - Use stable namespaces and DELETE markers when trajectory is invalid/empty so RViz does not show stale debug.
   - Add `src/trajectory_debug_markers.cpp` to the `px4_offboard_node` executable source list, following the existing `lidar_debug_node src/lidar_radar_markers.cpp` pattern rather than putting ROS-message marker code into `drone_city_nav_core`.
   - Add `ament_add_gtest(trajectory_debug_markers_test tests/trajectory_debug_markers_test.cpp src/trajectory_debug_markers.cpp)` in `drone_city_nav/CMakeLists.txt`, link `drone_city_nav_core`, and add `ament_target_dependencies(... visualization_msgs std_msgs geometry_msgs)` as required by the helper.
   - Add a pure helper test for color mapping:
     - low speed maps to slow color;
     - high speed maps to fast color;
     - high curvature maps to high-curvature color;
     - markers use `map` frame and `kRvizGroundZ`.
     - invalid/empty trajectory emits DELETE markers for `final_trajectory_speed_colormap` and `final_trajectory_curvature_colormap`.

9. Extend CSV final trajectory samples with time/speed diagnostics.

   Files:
   - `drone_city_nav/include/drone_city_nav/trajectory_diagnostics_io.hpp`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp`
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:
   - `writeFinalTrajectorySamplesCsvFile(...)`

   Materialized result:
   - Extract pure formatting helpers so CSV/summary contracts are unit-testable without launching ROS:

     ```cpp
     [[nodiscard]] std::string finalTrajectorySamplesCsvHeader();
     [[nodiscard]] std::string finalTrajectorySamplesCsvRow(
         const TrajectoryPointSample& sample,
         const TrajectorySpeedSample& speed_sample,
         double time_from_start_s,
         double time_to_finish_s);
     [[nodiscard]] std::string finalTrajectoryDiagnosticsSummaryJson(
         const TrajectoryPlannerStats& stats,
         const TrajectoryShapeDiagnostics& shape);
     ```

   - Add per-sample columns:
     - `profiled_time_from_start_s`
     - `profiled_time_to_finish_s`
     - `speed_limit_source`
     - current `curvature_1pm`, `speed_geometric_limit_mps`, `speed_profiled_limit_mps` remain.
   - Add aggregate metadata as a companion summary JSON next to the CSV, not repeated in each row. This avoids breaking row semantics and gives headless runs a stable file for aggregate diagnostics.
   - Add `src/trajectory_diagnostics_io.cpp` to `drone_city_nav_core` in `drone_city_nav/CMakeLists.txt`.
   - Add `ament_add_gtest(trajectory_diagnostics_io_test tests/trajectory_diagnostics_io_test.cpp)` linked to `drone_city_nav_core`.
   - Add deterministic tests:
     - CSV header contains all required old and new columns;
     - CSV row has finite numeric fields for normal samples;
     - summary JSON parses and contains `estimated_time_s`, `centerline_estimated_time_s`, `time_gain_s`, min/max speed limits and curvature-limited counts.

10. Extend blackbox and log summary with time-aware stats.

    Files:
    - `drone_city_nav/src/px4_offboard_node.cpp`
    - `drone_city_nav/src/trajectory_planner.cpp`
    - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
    - `drone_city_nav/include/drone_city_nav/trajectory_diagnostics_io.hpp`
    - `drone_city_nav/src/trajectory_diagnostics_io.cpp`
    - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
    - `drone_city_nav/CMakeLists.txt`

    Code anchors:
    - trajectory log around `px4_offboard_node.cpp:835`
    - blackbox writer around `px4_offboard_node.cpp:2728`
    - `computeSpeedProfileStats(...)`

    Materialized result:
    - Log and blackbox fields for final accepted trajectory:
      - `racing_final_estimated_time_s`
      - `racing_final_min_speed_limit_mps`
      - `racing_final_max_speed_limit_mps`
      - `racing_final_curvature_limited_samples`
    - Log and blackbox fields for centerline baseline:
      - `racing_centerline_estimated_time_s`
      - `racing_centerline_min_speed_limit_mps`
      - `racing_centerline_max_speed_limit_mps`
      - `racing_centerline_curvature_limited_samples`
    - Log and blackbox fields for best optimizer candidate before regularization:
      - `racing_best_candidate_estimated_time_s`
      - `racing_best_candidate_score`
      - `racing_best_candidate_min_speed_limit_mps`
      - `racing_best_candidate_max_speed_limit_mps`
      - `racing_best_candidate_curvature_limited_samples`
    - Log and blackbox derived fields:
      - `racing_time_gain_s`
      - `racing_regularization_time_delta_s`
      - regularization fields from step 7
    - Keep existing `speed_profile_min_mps`, `speed_profile_mean_mps`, `speed_profile_max_mps`, `speed_profile_limited_by_curvature_count`.
    - Add deterministic JSON field tests through `trajectory_diagnostics_io_test`: generate a representative summary/blackbox fragment, parse it with the existing test JSON parser or a lightweight local parser, and assert all required keys exist and contain finite numbers.

11. Add tuning parameters and defaults.

    Files:
    - `drone_city_nav/config/urban_mvp.yaml`
    - `drone_city_nav/src/px4_offboard_node.cpp`
    - `drone_city_nav/tests/px4_offboard_config_test.cpp`
    - `drone_city_nav/CMakeLists.txt`

    Code anchors:
    - parameter declaration around existing racing weights

    Materialized result:
    - Add:
      - `racing_line_weight_time`
      - regularization thresholds if needed, for example:
        - `racing_line_max_curvature_jump_1pm`
        - `racing_line_regularization_iterations`
    - Start with conservative values:
      - `racing_line_weight_time`: small nonzero (`0.05` or `0.1`)
      - regularization iterations: low (`1-3`)
    - Document in config comments only if the file already uses comments nearby; otherwise rely on code/README diagnostics.

12. Update and add tests for full pipeline behavior.

    Files:
    - `drone_city_nav/tests/racing_line_test.cpp`
    - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
    - `drone_city_nav/tests/trajectory_planner_test.cpp`
    - `drone_city_nav/tests/trajectory_diagnostics_test.cpp`
    - `drone_city_nav/tests/trajectory_debug_markers_test.cpp`
    - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
    - `drone_city_nav/CMakeLists.txt`

    Materialized result:
    - Happy path:
      - valid corridor produces valid racing trajectory;
      - time-aware cost reports positive `estimated_time_s`;
      - final trajectory has speed profile and blackbox stats.
    - Negative path:
      - prohibited candidate is rejected even if estimated time is better;
      - invalid/empty candidate yields invalid estimate, not NaN/Inf in stats.
    - Edge cases:
      - straight path has near-zero curvature and high speed limit;
      - 90-degree arc has lower speed limit and higher estimated time;
      - very short trajectory remains valid and finite;
      - curvature spike is detected and regularization reduces it or logs why it could not.
   - CMake wiring:
     - add every new test with `ament_add_gtest(...)`;
     - link core-only tests to `drone_city_nav_core`;
     - for ROS marker tests, include `src/trajectory_debug_markers.cpp` in the test target and add required `ament_target_dependencies`;
     - keep new production helpers either in `drone_city_nav_core` when they are ROS-message-free, or in the specific executable/test targets when they depend on ROS visualization messages.

# Verification plan

Implementation verification should use repo-approved container commands only.

Recommended command order after code changes:

1. `./scripts/dev_shell.sh make format`
   - Required because C++ files will change and repo has approved formatter target.
2. `./scripts/build.sh`
   - Build full ROS workspace via documented container wrapper.
3. `./scripts/test.sh`
   - Run C++/script test suite through documented wrapper.
4. `./scripts/dev_shell.sh make quality`
   - Run non-mutating quality checks through approved target.

Optional targeted checks before full suite if iteration speed is needed:

- inside `./scripts/dev_shell.sh`:
  - `colcon test --packages-select drone_city_nav --ctest-args -R racing_line`
  - `colcon test --packages-select drone_city_nav --ctest-args -R offboard_velocity_follower`
  - `colcon test --packages-select drone_city_nav --ctest-args -R trajectory_planner`

Simulation verification is intentionally not part of automatic implementation verification unless the user explicitly requests it:

- `./scripts/sim_headless.sh`
- `./scripts/sim_gui.sh`

For a later user-approved headless run, expected artifacts to inspect:

- `log/offboard_blackbox.jsonl`
- `log/final_trajectory_samples/latest.csv`
- `log/final_trajectory_samples/*summary*.json` if added
- RViz marker topics in GUI only when explicitly requested.

# Testing strategy (категории 1/2/3: без рефакторинга / лёгкий / тяжёлый)

1. Без рефакторинга.

   These are pure/unit tests with minimal code movement:

   - `offboard_velocity_follower_test.cpp`
     - time estimate on straight path;
     - time estimate on curved path;
     - backward/forward pass changes estimated time;
     - no NaN/Inf for tiny or degenerate sample spacing.
   - `racing_line_test.cpp`
     - `weight_time=0` preserves geometry-only behavior;
     - `weight_time>0` prefers lower-time valid candidate;
     - prohibited candidate stays rejected.
   - Config test:
     - `racing_line_weight_time` is declared, clamped, and propagated.

2. Лёгкий.

   These test cross-module behavior without launching Gazebo/PX4:

   - `trajectory_planner_test.cpp`
     - `planRacingTrajectory(...)` returns valid trajectory, speed profile, racing time stats;
     - centerline time and final time are finite;
     - `time_gain_s` has expected sign in a controlled corridor.
   - `trajectory_diagnostics_test.cpp`:
     - curvature spike diagnostics identify an artificial spike;
     - regularization reduces spike or reports a blocked reason.
   - `trajectory_debug_markers_test.cpp`:
     - speed/curvature color markers are deterministic;
     - stale markers are deleted when trajectory is invalid.
   - `trajectory_diagnostics_io_test.cpp`:
     - CSV header and row schema are deterministic;
     - summary JSON/blackbox fragment contains all required keys and finite numeric values.

3. Тяжёлый.

   These require full simulation and should be run only on explicit user command:

   - With static map headless run:
     - mission completes;
     - `racing_final_estimated_time_s` is finite;
     - final trajectory speed map shows slow zones at high curvature;
     - actual mission time does not regress badly versus previous baseline.
   - No-static headless run:
     - mission does not crash from stale/invalid racing trajectory;
     - replans do not block setpoint publication for long periods;
     - corridor/racing recompute time remains bounded.
   - GUI/RViz run:
     - cyan path remains visible;
     - speed/curvature marker layers are visually distinguishable;
     - debug layers are on ground plane and not stale after replan.

# Risks and tradeoffs

- Performance: scoring every candidate with profiled speed estimate can make optimizer slow again. Mitigation: reuse existing sampled points, keep sample count bounded, cache candidate sample conversion if needed, log candidate eval count and build duration.
- Cost balancing: `weight_time` can dominate smoothness/safety-adjacent preferences. Mitigation: start small, keep prohibited hard-rejected, compare `time_gain_s` with trajectory shape stats.
- Curvature regularization: smoothing points can accidentally degrade apex behavior or move samples toward prohibited cells. Mitigation: clamp to corridor, validate against prohibited grid, accept only valid and not materially worse candidates.
- Diagnostics schema changes: CSV/blackbox consumers may assume old columns. Mitigation: append columns instead of reordering existing fields; prefer companion summary file for aggregate stats.
- RViz overload: per-segment colored markers can be heavy on long trajectories. Mitigation: publish sampled markers using existing `final_trajectory_debug_sample_step_m`, delete stale namespaces, keep marker count bounded.
- API churn: changing `optimizeRacingLine(...)` signature touches tests and call sites. Mitigation: update all compile-time call sites in one commit and avoid default hidden speed configs.
- Tuning ambiguity: estimated time may improve before actual mission time improves because PX4 dynamics and controller limits are approximate. Mitigation: log both estimated and actual metrics during later explicit headless runs.

# Open questions

- Initial `racing_line_weight_time`: recommended start is `0.05` or `0.1`; exact value should be confirmed after first headless comparison.
- Should the optimizer always use profiled forward/backward estimate, or keep geometric-only as a debug parameter? The plan prefers profiled estimate by default because it matches the follower better.
- RViz color palette: speed and curvature should not conflict with existing colors. Suggested:
  - speed: blue/green for fast, magenta/orange for slow;
  - curvature: white/yellow for low, purple for high.
- Should aggregate CSV stats be written as a separate JSON summary or appended to CSV rows? Preferred: separate summary JSON to keep sample CSV row schema simple.
- Heavy simulation verification requires explicit user permission because the current working rule is not to run simulations automatically.
