# План: ускорить расчёт пути / траектории

## Context

Нужно запланировать одну связанную пачку ускорений для pipeline построения
траектории:

1. починить реальный параллелизм `racing_line`;
2. уйти от полного пересчёта trajectory score для каждого малого кандидата;
3. переиспользовать `corridor` / `clearance` для async refined trajectory;
4. сузить active windows, чтобы optimizer не работал по прямым участкам;
5. упростить DP: cache `segmentTraversable`, более крупный `dp_offset_step`,
   coarse-to-fine.

План должен учитывать, что новая/затронутая логика должна быть покрыта
автотестами и логами, чтобы её можно было отлаживать в headless-режиме.
Симуляцию в рамках планирования не запускать.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует. Текущий `PLAN.md` также
отсутствовал, поэтому план создаётся с нуля.

Локальный последний summary:
`log/final_trajectory_samples/latest_summary.json`.

Актуальный bottleneck из локального summary:

- `trajectory_total_duration_ms`: 2579.94 ms
- `trajectory_corridor_duration_ms`: 161.867 ms
- `trajectory_racing_line_duration_ms`: 2381.418 ms
- `trajectory_turn_smoothing_duration_ms`: 36.52 ms
- `corridor_parallel_workers_used`: 16
- `corridor_clearance_field_build_ms`: 148.849 ms
- `racing_candidate_path_evaluation_duration_ms`: 1173.017 ms
- `racing_candidate_score_duration_ms`: 997.639 ms
- `racing_candidate_point_build_duration_ms`: 40.853 ms
- `racing_candidate_sample_build_duration_ms`: 114.642 ms
- `racing_parallel_workers_used`: 2
- `racing_candidate_chunks`: 3026
- `racing_line_active_window_samples`: 89
- `racing_line_dp_states`: 3547
- `racing_line_dp_transitions`: 170743
- `racing_line_dp_duration_ms`: 1031.216 ms
- `racing_line_async_refined`: true

Вывод: основной тормоз сейчас не A*, а refined `racing_line`.
Параллелизм racing-line кандидатов фактически ограничен двумя оценками
`-step/+step`, а DP и full candidate scoring пересчитывают слишком много.
Async refined дополнительно заново строит `ClearanceField2D`, хотя caller уже
передаёт `prohibited_clearance_field`.

## Detected stack/profiles

Стек workspace:

- ROS 2 workspace;
- C++20;
- `ament_cmake` / `colcon`;
- основной пакет: `drone_city_nav`;
- manifest/build files: `drone_city_nav/CMakeLists.txt`, top-level
  `Makefile`;
- Rust-файлов (`Cargo.toml`, `Cargo.lock`, `*.rs`) в workspace не найдено.

Прочитанные профили orchestrator:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  — обязателен для любого workspace;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
  — применим, потому что проект C/C++/CMake;
- Rust profile не применялся, потому что Rust stack не обнаружен.

Прочитанные обязательные протоколы:

- `notion_access_protocol.md`;
- `gitlab_access_protocol.md`.

Notion/GitLab чтение не выполнялось: prompt не содержит Notion task id и не
запрашивает GitLab/MR-контекст, а `notion_policy=optional`.

## Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`, `CPP_BEST_PRACTICES.md`:

- `./scripts/build.sh` — approved build через container workflow;
- `./scripts/test.sh` — approved unit tests через container workflow;
- `./scripts/dev_shell.sh make format` — форматирование изменённых C++ файлов;
- `./scripts/dev_shell.sh make quality` — approved quality gate;
- внутри контейнера: `make build`, `make test`, `make test-scripts`,
  `make quality`, `make format`;
- scoped tests после build:
  `./scripts/dev_shell.sh bash -lc 'cd build/drone_city_nav && ctest -R "<regex>" --output-on-failure'`.

Симуляционные команды существуют, но для этой задачи не запускать без явной
команды пользователя:

- `./scripts/sim_headless.sh`;
- `./scripts/sim_gui.sh`;
- `./scripts/stop_sim.sh`.

## Affected components

- `drone_city_nav/src/racing_line.cpp`
  - `evaluateCandidateSnapshot()` около `racing_line.cpp:728`;
  - `buildDpSeedForWindow()` около `racing_line.cpp:802`;
  - основной цикл `optimizeRacingLine()` около `racing_line.cpp:958`;
  - текущий fake-parallel блок со `std::async` около `racing_line.cpp:1070`.
- `drone_city_nav/include/drone_city_nav/racing_line.hpp`
  - `RacingLineConfig` около `racing_line.hpp:16`;
  - `RacingLineStats` около `racing_line.hpp:40`.
- `drone_city_nav/src/planner_node_publish.cpp`
  - baseline trajectory build около `planner_node_publish.cpp:141`;
  - `startAsyncTrajectoryRefinement()` около `planner_node_publish.cpp:360`;
  - async refined lambda около `planner_node_publish.cpp:421`.
- `drone_city_nav/src/planner_node.hpp`
  - `TrajectoryRefinementRequest` около `planner_node.hpp:110`;
  - `startAsyncTrajectoryRefinement()` declaration около `planner_node.hpp:166`.
- `drone_city_nav/src/trajectory_planner.cpp`
  - `planBaselineTrajectory()` около `trajectory_planner.cpp:228`;
  - `planRacingTrajectory()` около `trajectory_planner.cpp:299`;
  - `TrajectoryPlannerInput` already supports `prohibited_clearance_field`.
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
  - `TrajectoryPlannerInput` около `trajectory_planner.hpp:68`;
  - `TrajectoryPlannerStats` около `trajectory_planner.hpp:42`.
- `drone_city_nav/src/corridor.cpp`
  - `buildCorridor()` около `corridor.cpp:305`;
  - existing clearance reuse branch около `corridor.cpp:325`;
  - existing parallel samples branch около `corridor.cpp:345`.
- Diagnostics/logs:
  - `drone_city_nav/src/planner_node_publish.cpp:230` — planner trajectory log;
  - `drone_city_nav/src/trajectory_diagnostics_io.cpp:300` и `:660` —
    JSON serialize/parse racing stats;
  - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`.
- Tests:
  - `drone_city_nav/tests/racing_line_test.cpp`;
  - `drone_city_nav/tests/trajectory_planner_test.cpp`;
  - `drone_city_nav/tests/trajectory_refinement_scheduler_test.cpp`;
  - `drone_city_nav/tests/corridor_test.cpp`;
  - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`.

## Implementation steps

1. Добавить bounded deterministic worker pool для racing candidate evaluation.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:

   - заменить текущий блок `std::async` `-step/+step` в
     `optimizeRacingLine()` (`racing_line.cpp:1070`) на ограниченный пул
     worker-ов;
   - `parallel_workers=0` должен означать auto: `std::thread::hardware_concurrency()`,
     но с cap, например `min(auto, control_indices.size(), 16)`;
   - `parallel_workers=1` остаётся полностью sequential baseline для
     deterministic tests;
   - results собирать в исходном deterministic order, выбор best candidate
     делать только в одном thread по стабильному порядку.

   Контракт:

   ```cpp
   struct CandidateTask {
     std::size_t order;
     std::size_t center_index;
     double delta_m;
   };

   // Workers только считают EvaluatedCandidate.
   // Main thread сортирует/обходит results по order и применяет best.
   for (const EvaluatedCandidate& candidate : ordered_results) {
     mergeStats(candidate);
     if (candidate.score.score + 1e-9 < best_cost) {
       acceptCandidate(candidate);
     }
   }
   ```

   Тесты:

   - обновить `RacingLine.DefaultParallelCandidateEvaluationMatchesSingleWorkerResult`:
     проверить `parallel_workers=0` и `parallel_workers=4`;
   - добавить тест, что `parallel_workers_used > 2` на corridor с достаточным
     числом control indices;
   - сохранить equality с single-worker по точкам/offset/final_cost/time;
   - negative: `parallel_workers=1` не включает parallel path.

2. Вынести candidate evaluation в reusable планировщик задач с per-worker scratch.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:

   - убрать shared `std::array<CandidateWorkBuffer, 2>` и заменить на
     `std::vector<CandidateWorkBuffer> worker_buffers`;
   - каждый worker владеет своим `CandidateWorkBuffer`;
   - не писать напрямую в общую `RacingLineStats` из worker threads;
   - `EvaluatedCandidate` должен переносить local timing/stats, main thread
     суммирует deterministic.

   Контракт:

   ```cpp
   struct CandidateBatchResult {
     std::size_t order;
     EvaluatedCandidate candidate;
     RacingLineCandidateStats local_stats;
   };
   ```

   Тесты:

   - deterministic equality между sequential и parallel;
   - проверка `worker_scratch_reuses`, `candidate_snapshot_allocations_avoided`,
     `candidate_chunks`, `parallel_workers_used`;
   - edge-case: мало candidates => `parallel_workers_used` не превышает число задач.

3. Добавить local/incremental scoring для малого offset candidate.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`;
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`.

   Материализуемый результат:

   - не пересобирать весь `candidate_points`, `candidate_samples`,
     `evaluatePath()` и `scoreForCandidate()` для каждого малого delta;
   - добавить локальный scoring window вокруг изменённого index:
     `i - score_radius_samples ... i + score_radius_samples`;
   - полный `scoreForCandidate()` оставить для:
     - initial seeds;
     - DP seed result;
     - accepted iteration winner;
     - final result;
     - fallback if local window touches endpoints or invalidates assumptions.

   Базовый контракт:

   ```cpp
   struct LocalCandidateScore {
     bool valid;
     bool requires_full_score;
     bool traversable;
     double delta_cost;
     double estimated_total_score;
   };

   LocalCandidateScore scoreLocalOffsetDelta(
       const RacingLineState& state,
       std::size_t center_index,
       double delta_m,
       std::span<const std::uint8_t> mutable_indices);
   ```

   Важно:

   - локальный score должен быть conservative: если есть сомнение, вернуть
     `requires_full_score=true`;
   - после принятия best candidate пересчитать full score и full samples один раз;
   - если full score не подтверждает улучшение, отклонить candidate и залогировать
     `local_score_false_positives`.

   Новые stats:

   - `local_candidate_evaluations`;
   - `local_candidate_full_score_fallbacks`;
   - `local_candidate_acceptance_full_scores`;
   - `local_score_false_positives`;
   - `local_candidate_score_duration_ms`;
   - `full_candidate_score_duration_ms`.

   Тесты:

   - happy path: local scoring уменьшает число full candidate evaluations;
   - negative: при candidate у endpoint или при collision uncertainty включается
     full fallback;
   - edge: accepted candidate подтверждается full score;
   - regression: итоговая trajectory совпадает или не хуже по `final_cost` в
     пределах tolerance с current full scoring на small fixture.

4. Добавить cache `segmentTraversable` для DP и candidate path checks.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:

   - в `buildDpSeedForWindow()` (`racing_line.cpp:802`) не вызывать
     `segmentTraversable()` повторно для одинаковых `(row, prev_candidate, candidate)`;
   - добавить локальный cache на окно DP;
   - для candidate evaluation добавить cache по segment index / endpoint cell pair,
     если он не меняет семантику.

   Контракт:

   ```cpp
   struct SegmentTraversabilityCacheKey {
     std::size_t a_row;
     std::size_t a_candidate;
     std::size_t b_row;
     std::size_t b_candidate;
   };
   ```

   Новые stats:

   - `dp_segment_cache_hits`;
   - `dp_segment_cache_misses`;
   - `candidate_segment_cache_hits`;
   - `candidate_segment_cache_misses`.

   Тесты:

   - DP result unchanged vs cache disabled/single-worker baseline;
   - cache hits > 0 на synthetic corridor с повторяющимися offsets;
   - collision rejection не меняется на fixture с blocked segment.

5. Сузить active windows и не оптимизировать прямые.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/config/urban_mvp.yaml`;
   - `drone_city_nav/src/planner_node_config.cpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:

   - улучшить `detectActiveWindows()` (`racing_line.cpp:406`);
   - в active window включать только участки, где есть:
     - heading change above threshold;
     - curvature/heading span above threshold;
     - sharp corridor width change;
     - low edge margin / asymmetric corridor;
   - прямые участки без curvature и без width change должны иметь
     `mutable_indices=0`, `active_window_samples=0`;
   - оставить fallback full-window только если centerline route itself is not
     traversable.

   Новые параметры:

   - `racing_line_window_min_heading_span_deg`;
   - `racing_line_window_min_curvature_1pm`;
   - `racing_line_window_min_width_asymmetry_m`;
   - возможно уменьшить default `window_pre_margin_m/post_margin_m`, но только
     после тестов.

   Псевдологика:

   ```cpp
   const bool turn_zone = heading_delta > threshold ||
                          heading_span(window) > min_heading_span ||
                          abs(curvature) > min_curvature;
   const bool width_zone = abs(width[i] - width[i - 1]) > width_change_threshold ||
                           abs(left_bound - right_bound) > asymmetry_threshold;
   if (turn_zone || width_zone) {
     addActiveWindow(...);
   }
   ```

   Тесты:

   - straight open corridor -> zero active windows (уже есть, расширить);
   - long route с одним поворотом -> active samples существенно меньше total samples;
   - blocked centerline -> full-window fallback сохраняется;
   - wide turn corridor -> active window остаётся around turn, не исчезает.

6. Упростить DP: `dp_offset_step_m` и coarse-to-fine.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/config/urban_mvp.yaml`;
   - `drone_city_nav/src/planner_node_config.cpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:

   - поднять default `racing_line_dp_offset_step_m` с `1.0` до `1.5` или `2.0`
     после локальной проверки тестами;
   - добавить coarse-to-fine mode:
     - coarse DP step: например `2.0 m`;
     - fine DP step только вокруг найденной coarse траектории: например `0.5-1.0 m`;
   - сохранить current one-pass DP как internal fallback только для тестового
     сравнения или disable path, если результат invalid.

   Контракт:

   ```cpp
   coarse_offsets = buildDpSeedForWindow(step=coarse_step);
   fine_bounds = clampAround(coarse_offsets, fine_radius_m);
   refined_offsets = buildDpSeedForWindow(step=fine_step, bounds=fine_bounds);
   ```

   Новые stats:

   - `dp_coarse_states`;
   - `dp_coarse_transitions`;
   - `dp_fine_states`;
   - `dp_fine_transitions`;
   - `dp_coarse_to_fine_used`;

   Тесты:

   - coarse-to-fine result valid and traversable;
   - state/transition count меньше one-pass fine DP на wide fixture;
   - final cost/time не регрессирует больше заданного tolerance;
   - invalid coarse/fine fallback корректно возвращает старый valid path.

7. Переиспользовать `ClearanceField2D` в async refined trajectory.

   Файлы:

   - `drone_city_nav/src/planner_node.hpp`;
   - `drone_city_nav/src/planner_node_publish.cpp`;
   - `drone_city_nav/tests/trajectory_planner_test.cpp`;
   - `drone_city_nav/tests/trajectory_refinement_scheduler_test.cpp`
     при необходимости.

   Материализуемый результат:

   - убрать `(void)prohibited_clearance_field` и
     `(void)prohibited_clearance_field_cache_hit` из
     `startAsyncTrajectoryRefinement()` (`planner_node_publish.cpp:366`);
   - сохранить или скопировать clearance field в `TrajectoryRefinementRequest`;
   - передать его в async lambda:

   ```cpp
   TrajectoryPlannerInput{
     .route_points = route,
     .prohibited_grid = &grid,
     .prohibited_clearance_field = request.clearance_field ? &*request.clearance_field
                                                           : nullptr,
     .prohibited_clearance_field_cache_hit = request.clearance_field_cache_hit,
   };
   ```

   Ограничение:

   - pointer нельзя передавать в async, если lifetime принадлежит caller stack.
     Нужно копировать `ClearanceField2D` в request или хранить shared immutable
     snapshot.

   Тесты:

   - `planRacingTrajectory()` already has
     `TrajectoryPlanner.ReusesProvidedClearanceFieldForCorridor`; расширить
     проверкой refined path;
   - добавить unit helper test вокруг request construction, если вынести
     construction в pure function;
   - проверить, что refined `stats.corridor.clearance_field_reused=true` и
     `clearance_field_build_duration_ms==0`/near-zero when field supplied.

8. Переиспользовать baseline corridor для refined trajectory там, где это
   безопасно.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`;
   - `drone_city_nav/src/trajectory_planner.cpp`;
   - `drone_city_nav/src/planner_node_publish.cpp`;
   - `drone_city_nav/tests/trajectory_planner_test.cpp`.

   Материализуемый результат:

   - добавить optional `precomputed_corridor_samples` в `TrajectoryPlannerInput`
     или новый `planRacingTrajectoryFromCorridor()`;
   - refined build должен пропустить `buildCorridor()` если:
     - route points/generation совпадают;
     - grid fingerprint/clearance source совпадает;
     - baseline corridor valid;
   - stats должны явно показать reuse.

   Возможный API:

   ```cpp
   struct TrajectoryPlannerInput {
     std::span<const Point2> route_points;
     const OccupancyGrid2D* prohibited_grid;
     const ClearanceField2D* prohibited_clearance_field;
     bool prohibited_clearance_field_cache_hit;
     std::span<const CorridorSample> precomputed_corridor_samples;
   };
   ```

   Новые stats:

   - `corridor_samples_reused`;
   - `corridor_reuse_source = baseline|none`;
   - `corridor_reuse_rejected_reason`.

   Тесты:

   - racing from precomputed corridor equals normal racing result on fixture;
   - invalid/empty precomputed corridor falls back to build;
   - mismatched endpoints/grid should not reuse silently.

9. Расширить diagnostics/blackbox-friendly logs для headless отладки.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/include/drone_city_nav/corridor.hpp`;
   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`;
   - `drone_city_nav/src/planner_node_publish.cpp`;
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp`;
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`;
   - `drone_city_nav/tests/offboard_blackbox_test.cpp` if blackbox mirrors fields.

   Материализуемый результат:

   - добавить все новые stats из шагов 1-8 в:
     - ROS log `Published trajectory result`;
     - trajectory diagnostics JSON;
     - final trajectory summary JSON;
   - в логах отделить:
     - baseline timing;
     - async refined timing;
     - candidate batch timing;
     - local-score vs full-score counts;
     - DP cache/coarse/fine counts;
     - corridor/clearance reuse.

   Тесты:

   - `trajectory_diagnostics_io_test` round-trip для новых полей;
   - `offboard_blackbox_test` если поля попадают в blackbox.

10. Обновить config defaults и документацию параметров.

    Файлы:

    - `drone_city_nav/config/urban_mvp.yaml`;
    - `drone_city_nav/src/planner_node_config.cpp`;
    - `docs/MVP_SIMULATION.md` если там перечислены planner/racing params.

    Материализуемый результат:

    - defaults в YAML и C++ должны совпадать;
    - новые acceleration parameters включены по умолчанию;
    - старые параметры не должны становиться скрытыми no-op.

    Предварительные defaults для обсуждения после тестов:

    - `racing_line_parallel_workers: 0` как auto-all-cores-with-cap;
    - `racing_line_dp_offset_step_m: 1.5` first safe bump;
    - `racing_line_coarse_dp_offset_step_m: 2.0`;
    - `racing_line_fine_dp_offset_step_m: 0.75`;
    - active-window thresholds оставить conservative, чтобы не потерять повороты.

11. Добавить performance regression unit/fixture tests без wall-clock assertions.

    Файлы:

    - `drone_city_nav/tests/racing_line_test.cpp`;
    - `drone_city_nav/tests/trajectory_planner_test.cpp`;
    - при необходимости новый fixture helper в `drone_city_nav/tests/`.

    Материализуемый результат:

    - не тестировать “быстрее в миллисекундах” как flaky condition;
    - тестировать proxy metrics:
      - fewer full candidate evaluations;
      - fewer DP transitions;
      - `parallel_workers_used` matches expected;
      - corridor/clearance reuse flags set;
      - final trajectory still traversable/valid;
      - final estimated time/cost not worse beyond tolerance.

## Verification plan

Минимальные scoped checks после реализации:

```bash
./scripts/build.sh
./scripts/dev_shell.sh bash -lc 'cd build/drone_city_nav && ctest -R "(racing_line_test|trajectory_planner_test|trajectory_refinement_scheduler_test|trajectory_diagnostics_io_test|corridor_test)" --output-on-failure'
```

Полный repo-approved quality gate перед коммитом:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
```

Если менялись только docs/config без C++:

- всё равно использовать repo-approved workflow;
- `make quality` предпочтителен перед коммитом согласно `AGENTS.md` /
  `README.md`, если стоимость приемлема.

Симуляция:

- не запускать автоматически;
- после реализации пользователь может вручную прогнать GUI/headless;
- для headless-debug после ручного прогона анализировать:
  - `log/ros_city_mvp.log`;
  - `log/final_trajectory_samples/latest_summary.json`;
  - `log/corridor_samples/latest.csv`;
  - `log/offboard_blackbox.jsonl`.

## Testing strategy

Категория 1: без рефакторинга, локальные unit tests

- `racing_line_test`:
  - deterministic sequential vs parallel;
  - straight corridor skips windows;
  - blocked centerline full-window fallback;
  - DP cache does not change result;
  - local scoring falls back when unsafe.
- `trajectory_diagnostics_io_test`:
  - serialize/parse new stats.
- `trajectory_planner_test`:
  - provided clearance field reused;
  - precomputed corridor reuse valid.

Категория 2: лёгкий рефакторинг

- Вынести candidate batch evaluation в private helpers внутри
  `racing_line.cpp`, без public API.
- Добавить small pure helpers для:
  - worker count resolution;
  - active window classification;
  - DP candidate bounds/coarse-to-fine bounds;
  - local scoring window computation.
- Тестировать helpers через поведение `optimizeRacingLine()`, а не раскрывать
  лишний public API, если возможно.

Категория 3: тяжёлый рефакторинг

- Если local/incremental scoring становится слишком сложным внутри одного файла,
  разделить `racing_line.cpp` на внутренние модули:
  - `racing_line_candidate_evaluator`;
  - `racing_line_dp`;
  - `racing_line_windows`.
- Делать это только если объём изменений в одном файле станет плохо проверяемым.
- После тяжёлого рефакторинга обязательно полный `make quality`.

Autotest gaps:

- Реальное wall-clock ускорение зависит от CPU и нагрузки, поэтому в unit tests
  нельзя жёстко assert-ить milliseconds.
- Wall-clock подтверждать только логами после ручного headless/GUI run.
- Поведение на реальной карте подтверждать ручным прогоном пользователя, но
  correctness контракт должен быть закрыт unit tests.

## Risks and tradeoffs

- Параллельный optimizer может стать nondeterministic, если best candidate
  выбирать из worker threads. Митигировать deterministic ordered merge.
- Floating-point суммы могут чуть отличаться при другом порядке. Митигировать:
  workers считают только candidate, main thread суммирует stats/выбирает best
  в фиксированном порядке.
- Local scoring может принять candidate, который выглядит лучше локально, но
  хуже глобально. Митигировать full-score confirmation before accept.
- Active-window narrowing может пропустить участок, где полезен apex/offset.
  Митигировать conservative thresholds + fallback full-window when centerline
  invalid or low margin.
- Увеличение `dp_offset_step_m` ускорит DP, но может ухудшить качество линии.
  Митигировать coarse-to-fine и tolerance tests по time/cost/traversability.
- Reusing corridor/clearance across async refined опасен, если grid/route snapshot
  не совпадают. Митигировать immutable snapshot in request + generation/path id +
  endpoint/grid bounds/fingerprint checks.
- Thread pool увеличит CPU pressure. Нужен bounded cap и stats
  `parallel_workers_used`.
- Больше diagnostics увеличит размер логов, но это приемлемо для headless
  отладки; новые поля должны быть компактными.

## Open questions

- Какой cap для `racing_line_parallel_workers=0` выбрать по умолчанию:
  `hardware_concurrency`, `min(16, cores)`, или отдельный параметр max cap?
  Предлагаемый старт: auto с cap `16`.
- Какой tolerance считать допустимым для local scoring / coarse-to-fine:
  `estimated_time_s <= baseline + 0.1s` или относительный порог?
  Предлагаемый старт: не хуже текущего full-scoring больше чем `0.25s` и не
  длиннее `max_length_ratio`.
- Нужно ли сохранять старый full-scoring path как runtime fallback или только
  как test-only helper? Предлагаемый старт: runtime fallback на first iteration
  при local scoring invalid/unstable.
- Нужно ли дополнительно к reuse `ClearanceField2D` кэшировать static grid /
  inflation сейчас? В этот план не включено, потому что текущий bottleneck из
  summary — `racing_line`, а не grid build.
