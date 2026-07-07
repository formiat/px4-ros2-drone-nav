# Plan: Optional Local XY Passage Insertion

## Context

Нужно спланировать этап `Optional Local XY Passage Insertion` для фичи
`known passage traversal`: если 2D-траектория уже построена, но проходит через
известный 3D-проход недостаточно точно по XY, локально заменить небольшой
фрагмент траектории на гладкий участок через opening/gate и затем снова
проверить всю траекторию.

Текущая архитектура уже разделяет задачу на несколько слоёв:

- 2D planner строит основную XY-геометрию: A*, corridor, trajectory optimizer,
  turn smoothing, isolated geometry cleanup.
- Known passage map описывает известные 3D-проходы в
  `worlds/known_passages.passages3d`.
- Known passage validation проверяет, пересекает ли траектория annotated
  structure и проходит ли она через opening.
- Vertical profile добавляет `z_m` поверх уже построенной XY-траектории.
- Speed profile строится после vertical profile и учитывает вертикальные
  ограничения.

Новый этап не должен заменять 2D planner. Он должен быть optional repair stage
после формирования гладкой 2D-траектории и до vertical profile/speed profile.
Риск высокий, потому что stage меняет XY-геометрию, поэтому он должен быть
строго валидируемым и хорошо диагностируемым.

Термины для кода и документации:

- `known passage traversal` - общий user-facing термин.
- `passage opening` / `opening` - конкретный проём в known passage map.
- `gate` - геометрическая модель opening center + normal + entry/exit frame.
- `local passage insertion` - сам этап локальной XY-вставки.

## Investigation Context

`INVESTIGATION.md` отсутствует. Перед планированием проверены текущие модули:

- `drone_city_nav/include/drone_city_nav/known_passage_map.hpp`
- `drone_city_nav/include/drone_city_nav/known_passage_matching.hpp`
- `drone_city_nav/include/drone_city_nav/known_passage_validation.hpp`
- `drone_city_nav/src/known_passage_matching.cpp`
- `drone_city_nav/src/known_passage_validation.cpp`
- `drone_city_nav/include/drone_city_nav/trajectory.hpp`
- `drone_city_nav/src/trajectory.cpp`
- `drone_city_nav/include/drone_city_nav/trajectory_vertical_profile.hpp`
- `drone_city_nav/src/trajectory_vertical_profile.cpp`
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
- `drone_city_nav/src/trajectory_planner.cpp`
- `drone_city_nav/include/drone_city_nav/trajectory_shape_cleanup.hpp`
- `drone_city_nav/src/trajectory_shape_cleanup.cpp`
- `drone_city_nav/src/trajectory_speed_planner.cpp`
- `drone_city_nav/src/planner_node_config.cpp`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/CMakeLists.txt`
- `docs/navigation_pipeline.md`
- `docs/trajectory_optimization.md`

Ключевой текущий pipeline находится в
`drone_city_nav/src/trajectory_planner.cpp::planOptimizedTrajectory()`:

1. Corridor samples.
2. `optimizeTrajectory()`.
3. `smoothTrajectoryTurns()`.
4. `smoothIsolatedCurvatureSpikes()`.
5. `lineTrajectoryFromSamples()`.
6. `applyVerticalProfileStage()`.
7. `buildTrajectorySpeedProfile()`.
8. `finalizeResult()`.

Лучшее место для нового этапа: после `smoothIsolatedCurvatureSpikes()` и
`trajectoryStageInvariantsHold()` для cleanup, но до `lineTrajectoryFromSamples()`,
`applyVerticalProfileStage()` и `buildTrajectorySpeedProfile()`.

## Detected Stack / Profiles

Прочитанные обязательные профили:

- Generic project profile:
  `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- C/C++ project profile:
  `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Прочитанные обязательные протоколы:

- Notion access protocol:
  `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- GitLab access protocol:
  `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

Локальные инструкции репозитория:

- `AGENTS.md`
- `README.md`
- `CONTRIBUTING.md`
- `CPP_BEST_PRACTICES.md`
- `Makefile`

Стек: ROS 2 workspace, C++20, CMake/ament/colcon, Docker-first workflow через
скрипты репозитория и `make` внутри container/dev shell.

Remote Notion/GitLab/HTTP/SSH не использовались: в задаче нет task id/MR и
требуется только локальный plan artifact.

## Repo-Approved Commands Found

Внешние команды из `AGENTS.md`, `README.md`, `CONTRIBUTING.md`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_headless.sh`
- `./scripts/sim_gui.sh`
- `./scripts/dev_shell.sh`
- `./scripts/stop_sim.sh`

Команды внутри container/dev shell:

- `make format`
- `make build`
- `make test`
- `make test-scripts`
- `make quality`
- `make sim-headless`
- `make sim-gui`

Для реализации C++-изменений использовать container workflow. Не запускать
ad-hoc top-level CMake-команды.

## Affected Components

- `known_passage_matching`: сейчас содержит приватную геометрию opening frame,
  которую надо переиспользовать для local insertion.
- `known_passage_validation`: источник spans/reasons, по которым понятно, где
  траектория не проходит через opening.
- `trajectory_planner`: точка подключения нового stage.
- `trajectory_vertical_profile`: должен работать уже по изменённой XY-траектории.
- `trajectory_speed_planner`: должен получить финальные samples после XY insertion
  и vertical profile.
- `trajectory_diagnostics`: должен получить counters/diagnostics по insertion.
- `planner_node_config` и `urban_mvp.yaml`: новые параметры stage.
- RViz/debug: final trajectory уже сможет отображать 3D path через `z_m`;
  для insertion нужны diagnostics в logs/JSON, а не новый marker в первом шаге.

## Implementation Steps

1. Extract reusable known-passage geometry helpers.

   Files:

   - Add `drone_city_nav/include/drone_city_nav/known_passage_geometry.hpp`
   - Add `drone_city_nav/src/known_passage_geometry.cpp`
   - Update `drone_city_nav/src/known_passage_matching.cpp`

   Code anchors:

   - Move/reuse helper logic around opening local coordinates from
     `known_passage_matching.cpp` helpers such as local opening-frame conversion
     and station clipping.
   - Expose small value types:
     `KnownPassageOpeningFrame`, `KnownPassageOpeningFootprint`,
     `KnownPassageStationSpan`.

   Materialized result:

   - Matching, validation, vertical profile and future insertion use one shared
     opening-frame implementation.
   - No behavior change expected in existing matching/validation tests.

2. Add local passage insertion module.

   Files:

   - Add `drone_city_nav/include/drone_city_nav/trajectory_passage_insertion.hpp`
   - Add `drone_city_nav/src/trajectory_passage_insertion.cpp`
   - Update `drone_city_nav/CMakeLists.txt`

   Core API:

   - `PassageInsertionConfig`
   - `PassageInsertionStatus`
   - `PassageInsertionRejectReason`
   - `PassageInsertionDiagnostic`
   - `PassageInsertionStats`
   - `PassageInsertionResult`
   - `insertLocalPassageSegments(...)`

   Recommended config fields:

   - `enabled`
   - `sample_step_m`
   - `min_anchor_margin_m`
   - `max_anchor_margin_m`
   - `opening_lateral_target_margin_m`
   - `max_lateral_shift_m`
   - `max_join_tangent_delta_rad`
   - `max_join_curvature_jump_1pm`
   - `min_inserted_radius_m`
   - `max_candidates`
   - `max_diagnostics`

   Materialized result:

   - Stage exists as isolated pure C++ logic with no ROS dependency.
   - Disabled config returns exact no-op copy/stats.

3. Detect repair candidates from known-passage diagnostics.

   Files:

   - `trajectory_passage_insertion.cpp`

   Code anchors:

   - Use `validateKnownPassageTraversal(samples, map, validation_config)`.
   - Use `findKnownPassageTraversalMatches(samples, map, validation_config,
     true)` for XY-only matching when altitude is not assigned yet.

   Candidate rules:

   - Only consider spans with reasons:
     `kStructureWithoutOpening` or `kOpeningVolumeMiss`.
   - Ignore spans that are already valid.
   - For each span, select opening by minimal lateral miss and sufficient
     station overlap.
   - If too many independent invalid spans exist, do not attempt global surgery;
     record diagnostics and let the regular invalid-trajectory handling fail or
     trigger replanning.

   Materialized result:

   - The module identifies exactly which local trajectory fragment could be
     repaired and which opening it targets.

4. Build the local smooth XY segment.

   Files:

   - `trajectory_passage_insertion.cpp`

   Algorithm:

   - Choose anchors by trajectory station:
     `anchor_s = max(0, entry_s - dynamic_pre_margin)`
     and
     `reconnect_s = min(total_s, exit_s + dynamic_post_margin)`.
   - Dynamic margins:
     `pre_margin = clamp(max(min_anchor_margin_m, opening.approach_distance_m),
     min_anchor_margin_m, max_anchor_margin_m)`.
     Same idea for post margin with `opening.exit_distance_m`.
   - Compute:
     - original anchor point/tangent from samples,
     - gate entry = opening center minus normal * depth/2,
     - gate center = opening center XY,
     - gate exit = opening center plus normal * depth/2,
     - original reconnect point/tangent from samples.
   - Generate a sampled multi-segment cubic Hermite/Bezier curve:
     - anchor -> gate entry,
     - gate entry -> gate exit through gate center/opening normal,
     - gate exit -> reconnect.
   - Preserve endpoint samples and avoid duplicate points at stitch boundaries.

   Materialized result:

   - A candidate inserted segment with continuous XY position and controlled
     tangent at anchor/reconnect.

5. Recompute geometry for the stitched trajectory.

   Files:

   - Prefer adding/reusing helper in `drone_city_nav/src/trajectory.cpp`
     or a small internal helper used by both `trajectory_shape_cleanup.cpp` and
     `trajectory_passage_insertion.cpp`.

   Code anchors:

   - Existing geometry recomputation patterns in
     `trajectory.cpp::lineTrajectoryFromSamples()` and
     `trajectory_shape_cleanup.cpp`.

   Requirements:

   - Recompute monotonic `s_m`.
   - Recompute `tangent`.
   - Recompute `curvature_1pm`.
   - Reset vertical-only fields before vertical profile:
     `z_m = cruise/default altitude`,
     `vertical_speed_cap_mps = infinity`,
     `vertical_profile_passage_id.clear()`.
   - Keep `left_bound_m/right_bound_m` and corridor metadata conservative:
     either interpolate nearby values or mark insertion-local values as
     diagnostics-only; do not use stale corridor bounds as proof of safety.

   Materialized result:

   - Stitched trajectory is internally coherent and ready for invariant checks.

6. Validate candidate before accepting it.

   Files:

   - `trajectory_passage_insertion.cpp`
   - `trajectory_planner.cpp`

   Acceptance checks:

   - `trajectorySamplesAreUsable(stitched)`.
   - Start/end anchors of the whole mission are preserved.
   - Full trajectory traversability on `prohibited_grid`.
   - Join tangent deltas are below `max_join_tangent_delta_rad`.
   - Join curvature jumps are below `max_join_curvature_jump_1pm`.
   - Inserted segment min radius is not below `min_inserted_radius_m`, unless
     no valid baseline exists and diagnostics explicitly say fallback.
   - XY known-passage validation improves:
     violations decrease or targeted span becomes a valid opening match with
     `ignore_altitude=true`.
   - Never accept candidate only because it is shorter.

   Materialized result:

   - If local insertion is risky, the original trajectory remains unchanged.
   - If accepted, the accepted path has a concrete validation trail.

7. Wire the stage into `planOptimizedTrajectory()`.

   File:

   - `drone_city_nav/src/trajectory_planner.cpp`

   Code anchor:

   - Insert after isolated geometry cleanup and its
     `trajectoryStageInvariantsHold()` call.
   - Insert before:
     `result.compact_segments = lineTrajectoryFromSamples(result.samples);`
     and before `applyVerticalProfileStage(...)`.

   Behavior:

   - If `passage_insertion.enabled == false`, exact no-op.
   - If no known passage map or no invalid passage span, no-op.
   - If a candidate passes validation, replace `result.samples`.
   - Run the normal stage invariants after replacement.
   - Then run existing vertical profile and speed profile exactly as today.

   Materialized result:

   - The main planner can produce a locally adjusted XY trajectory through known
     passages without changing the rest of the pipeline.

8. Add config loading and YAML parameters.

   Files:

   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/tests/planner_node_config_test.cpp`
   - `drone_city_nav/tests/px4_offboard_config_test.cpp` if script/default YAML
     expectations include the planner block.

   Recommended initial policy:

   - Add `passage_insertion_enabled`.
   - Default in code can be `false` for safety.
   - Enable only in scenario/config once unit/integration tests prove the stage.
   - If the project wants it immediately active, keep it `true` only after
     headless run confirms no regression on current 2D route.

   Materialized result:

   - Feature is guarded by explicit config and visible in public YAML.

9. Add diagnostics and serialization.

   Files:

   - `drone_city_nav/include/drone_city_nav/trajectory_diagnostics.hpp`
   - `drone_city_nav/src/trajectory_diagnostics_io_json_summary.cpp`
   - `drone_city_nav/src/trajectory_diagnostics_io_parser.cpp`
   - `drone_city_nav/src/trajectory_diagnostics_io_internal.hpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test_helpers.hpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_json_fields_test.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_roundtrip_test.cpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`

   Required fields:

   - `passage_insertion_enabled`
   - `passage_insertion_applied`
   - `passage_insertion_candidates`
   - `passage_insertion_inserted_count`
   - `passage_insertion_rejected_join`
   - `passage_insertion_rejected_traversability`
   - `passage_insertion_rejected_validation`
   - `passage_insertion_duration_ms`
   - first/top diagnostics:
     `structure_id`, `opening_id`, `anchor_s`, `entry_s`, `exit_s`,
     `reconnect_s`, `lateral_miss_before`, `lateral_miss_after`,
     `join_tangent_delta_before/after`, `join_curvature_jump_before/after`,
     `reason`.

   Materialized result:

   - Last-run logs show why a local passage insertion was accepted or rejected.

10. Update documentation.

    Files:

    - `docs/navigation_pipeline.md`
    - `docs/trajectory_optimization.md`
    - `docs/configuration.md`
    - `docs/diagnostics.md`
    - Later, when RViz 3D visualization is expanded:
      `docs/rviz.md`.

    Materialized result:

    - Documentation states that local passage insertion is optional, high-risk
      geometry repair after 2D smoothing and before vertical profile.

## Verification Plan

For this plan-only task:

- Run `make quality` through the repository-approved workflow if the local
  container environment is available.
- Do not run simulation for a plan-only markdown artifact.

For the implementation task:

1. Format changed C++ files:
   `./scripts/dev_shell.sh` then `make format`.
2. Build:
   `./scripts/build.sh` or `make build` inside dev shell.
3. Run scoped C++ tests:
   - `trajectory_passage_insertion_test`
   - `known_passage_validation_test`
   - `trajectory_vertical_profile_test`
   - `trajectory_planner_test`
   - `trajectory_diagnostics_io_test`
   - `planner_node_config_test`
4. Run broader checks:
   - `make test`
   - `make test-scripts`
   - `make quality`
5. Run headless simulation only after unit/integration tests pass and the
   feature is enabled in a scenario config:
   `./scripts/sim_headless.sh`.

## Testing Strategy Categories

1. New/changed tests and minimal scope.

   Add `drone_city_nav/tests/trajectory_passage_insertion_test.cpp` with:

   - disabled config returns no-op;
   - no map returns no-op;
   - already valid opening traversal returns no-op;
   - invalid XY path through known structure is repaired through opening center;
   - candidate is rejected on prohibited-grid collision;
   - candidate is rejected on excessive join tangent delta;
   - candidate is rejected on excessive curvature jump;
   - endpoints are preserved and `s_m` remains monotonic;
   - diagnostics explain accepted/rejected candidate.

2. Relevant package/component scope.

   Update and run:

   - `known_passage_validation_test`
   - `trajectory_vertical_profile_test`
   - `trajectory_planner_test`
   - `trajectory_diagnostics_io_test`
   - `planner_node_config_test`

   Add at least one planner integration test where a 2D route would otherwise
   miss the opening corridor, but insertion makes the final 3D trajectory valid
   after vertical profile.

3. Repository/workspace-level checks.

   Run `make test`, `make test-scripts`, and `make quality` because the change
   touches public headers, CMake, config, diagnostics JSON, and planner behavior.
   Simulation is a later manual/e2e check, not a substitute for unit tests.

## Risks And Tradeoffs

- XY geometry change can make a previously smooth trajectory worse. Mitigation:
  keep feature config-gated and require join tangent/curvature checks.
- Local insertion can overfit one passage and create bad reconnect geometry.
  Mitigation: dynamic margins, join validation, full trajectory validation.
- Corridor metadata may not be valid inside inserted segment. Mitigation:
  do not trust stale corridor bounds for acceptance; validate against prohibited
  grid and known-passage geometry.
- Altitude validation before vertical profile is not meaningful. Mitigation:
  insertion uses XY-only passage matching, then existing vertical profile performs
  full 3D validation.
- Multiple nearby openings/spans can make local repair ambiguous. Mitigation:
  cap candidates, log ambiguity, and fall back to existing full pipeline.
- This stage should not introduce a length/speed preference. It should only
  repair passage alignment while preserving smoothness/safety.

## Open Questions

1. Should `passage_insertion_enabled` default to true?

   Recommended answer: default `false` in code and enable only in dedicated
   scenario/config after tests and one headless run.

   Rationale: the stage changes XY geometry and is explicitly high-risk.

   What confirms: trajectory insertion unit/integration tests plus a headless run
   with known passages showing no regressions in ordinary 2D routes.

2. Should local insertion use Catmull-Rom, cubic Hermite, or Bezier?

   Recommended answer: cubic Hermite/Bezier with explicit endpoint tangents.

   Rationale: we need controlled tangent at anchor/reconnect and controlled
   direction through opening normal. Catmull-Rom is convenient but less explicit
   about join conditions.

   What confirms: tests on join tangent delta, curvature jump and minimum radius.

3. Should insertion choose the shortest valid repair?

   Recommended answer: no. It should choose the smoothest valid repair that
   satisfies opening alignment and safety constraints.

   Rationale: project direction prefers smooth/beautiful trajectory over shorter
   path after corridor planning.

   What confirms: candidate scoring has no length/time criterion, only
   smoothness/alignment/safety.

4. Should local insertion attempt to fix every invalid span in one pass?

   Recommended answer: only one or a small capped number of non-overlapping spans
   per planner build.

   Rationale: many invalid spans likely mean the global route is wrong, and a
   local patchwork could make the trajectory incoherent.

   What confirms: diagnostics show span count; if too many spans appear, full
   replanning or higher-level passage-aware route planning is the safer answer.

5. Should RViz get new marker types in this same stage?

   Recommended answer: no, not in the first insertion implementation. Existing
   3D final trajectory and known passage markers are enough for validation.

   Rationale: keep behavior change and visualization expansion separate.

   What confirms: final trajectory CSV/markers include `z_m`, known passage
   markers show opening center/approach/exit, and insertion diagnostics explain
   the local repair.
