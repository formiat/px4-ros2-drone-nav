# Context

Планируем ускорение расчёта пути и финальной гоночной траектории без целевого изменения геометрии полёта.

Свежие локальные данные из последнего доступного прогона `20260630T014121Z`:

- `log/ros_city_mvp.log:201`: `grid=1561.2ms`, `path_total=743.2ms`, `astar=108.7ms`, `smoothing=62.2ms`, `clearance_field=545.5ms`, `clearance_cache_hit=false`.
- `log/final_trajectory_samples/latest_summary.json`: `trajectory_total_duration_ms=2316.698`, `trajectory_corridor_duration_ms=856.906`, `trajectory_racing_line_duration_ms=1459.452`.
- `log/ros_city_mvp.log:305`: опубликовано `waypoints=77`, `length=402.76`, `astar_runs=1`, `prohibited_replans=0`.

Вывод: A* сейчас не главный тормоз. Основные кандидаты на ускорение: `racing_line`, `corridor`, `clearance_field` diagnostics/reuse и `grid build`.

# Investigation context

`INVESTIGATION.md` в `workspace_root` отсутствует, дополнительных входных исследовательских артефактов нет.

# Detected stack/profiles

- Стек: ROS 2 workspace, пакет `drone_city_nav`, C++20, ament CMake, `colcon`.
- Прочитаны обязательные профили:
  - `generic.md`;
  - `cpp.md`, потому что в workspace есть `CMakeLists.txt`, `Makefile`, `.cpp/.hpp`.
- Прочитаны обязательные протоколы:
  - `notion_access_protocol.md`: Notion в пользовательском промпте не упомянут, policy `optional`, чтение Notion-задачи не требуется.
  - `gitlab_access_protocol.md`: GitLab/MR в пользовательском промпте не упомянуты, `glab`-чтение не требуется.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_gui.sh`
- `./scripts/sim_headless.sh`
- `./scripts/stop_sim.sh`
- внутри dev container: `make build`, `make test`, `make test-scripts`, `make quality`, `make format`, `make sim-gui`, `make sim-headless`
- scoped test fallback внутри контейнера после build: `ctest --test-dir build/drone_city_nav --output-on-failure`

Прямые ad-hoc top-level CMake-команды не использовать.

# Affected components

- `drone_city_nav/src/racing_line.cpp:618`: основной `optimizeRacingLine()`.
- `drone_city_nav/src/racing_line.cpp:695`: текущий parallel path запускает по два `std::async` на каждый sample/step.
- `drone_city_nav/src/racing_line.cpp:566`: `evaluateCandidateSnapshot()` сейчас создаёт новые `offsets`, `points`, `scratch_samples` на каждый parallel candidate.
- `drone_city_nav/include/drone_city_nav/racing_line.hpp:33`: `RacingLineStats` уже содержит timings, но не содержит worker/chunk diagnostics.
- `drone_city_nav/src/corridor.cpp:199`: `buildCorridor()`.
- `drone_city_nav/src/corridor.cpp:219`: corridor строит свой `ClearanceField2D`.
- `drone_city_nav/src/corridor.cpp:227`: corridor samples считаются последовательно.
- `drone_city_nav/src/corridor.cpp:276`: `applyLocalLateralLimit()` зависит от набора samples и должен оставаться отдельным ordered pass.
- `drone_city_nav/include/drone_city_nav/corridor.hpp:35`: `CorridorStats` нужно расширить timing/parallel diagnostics.
- `drone_city_nav/src/clearance_field.cpp:73`: `ClearanceField2D::build()` строит поле через bounded 8-neighbor propagation.
- `drone_city_nav/src/clearance_field.cpp:154`: `ClearanceFieldCache::getOrBuild()` уже есть, но кэш сейчас локален для `PlannerCore`.
- `drone_city_nav/src/planner_core.cpp:595`: `PlannerCore` строит diagnostics clearance field через `prohibited_clearance_cache_`.
- `drone_city_nav/src/trajectory_planner.cpp:117`: pipeline `corridor -> racing_line -> turn_smoothing -> speed_profile`.
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:57`: `TrajectoryPlannerInput` сейчас передаёт только route points и grid, без общего clearance field.
- `drone_city_nav/src/planning_grid_builder.cpp:69`: raw grid пересобирается с нуля.
- `drone_city_nav/src/planning_grid_builder.cpp:114`: prohibited grid и planning grid заново делают `rebuildInflation()`.
- `drone_city_nav/src/planner_node_inputs.cpp:345`: planning summary logs уже показывают grid/path timings.
- `drone_city_nav/src/planner_node_publish.cpp:198`: final racing trajectory logs уже показывают trajectory/corridor/racing timings.
- Тесты: `drone_city_nav/tests/racing_line_test.cpp`, `corridor_test.cpp`, `clearance_field_test.cpp`, `planner_core_test.cpp`, `trajectory_planner_test.cpp`, `planning_grid_builder_test.cpp`, `trajectory_diagnostics_io_test.cpp`, `planner_node_config_test.cpp`.

# Implementation steps

1. Оптимизировать `racing_line` parallel execution без изменения результата.

   Файлы:
   - `drone_city_nav/src/racing_line.cpp:566`
   - `drone_city_nav/src/racing_line.cpp:695`
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp:33`
   - `drone_city_nav/tests/racing_line_test.cpp:115`
   - `drone_city_nav/tests/racing_line_test.cpp:139`

   Материализуемый результат:
   - заменить per-candidate `std::async` на worker-local evaluation без изменения текущего порядка алгоритма;
   - создать worker-local scratch buffers, чтобы не пересоздавать `offsets`, `points`, `candidate_samples` на каждый candidate;
   - parallel evaluation разрешена только для двух независимых кандидатов текущего индекса `i`: `-step` и `+step`, построенных от текущего `offsets` snapshot;
   - accepted result текущего `i` должен быть применён сразу до перехода к `i + 1`, как сейчас делает `offsets = scratch.iteration_best_offsets` в `drone_city_nav/src/racing_line.cpp:760`;
   - не делать batching/chunking сразу по нескольким индексам `i`: это поменяет алгоритм, потому что кандидаты `i + 1` начнут считаться от устаревшего `offsets`;
   - сохранить deterministic best-candidate selection в прежнем порядке: сначала `-step`, потом `+step`, затем применить best offsets, затем следующий индекс;
   - расширить `RacingLineStats`: `parallel_workers_used`, `candidate_chunks`, `worker_scratch_reuses`, `candidate_snapshot_allocations_avoided` или эквивалентные поля;
   - не менять scoring formula, weights, regularization и `TrajectoryPointSample` output contract.

   Псевдокод:

   ```cpp
   for (std::size_t i = 1U; i + 1U < sample_count; ++i) {
     CandidateJob jobs[2] = {
         CandidateJob{.base_offsets = offsets, .center_index = i, .delta_m = -step},
         CandidateJob{.base_offsets = offsets, .center_index = i, .delta_m = step},
     };
     EvaluatedCandidate results[2];

     parallelEvaluateTwoCandidates(jobs, results, worker_scratch);

     scratch.iteration_best_offsets = offsets;
     for (const EvaluatedCandidate& candidate : results /* -step, then +step */) {
       maybeAcceptCandidate(candidate, best_cost, scratch.iteration_best_offsets,
                            best_points, changed);
     }

     offsets = scratch.iteration_best_offsets; // must happen before i + 1
   }
   ```

   Multi-index batching от одного `offsets` snapshot можно рассматривать только как отдельный behavioral experiment, не как безопасную оптимизацию этого пакета.

   Автотесты:
   - расширить `RacingLine.DefaultParallelCandidateEvaluationMatchesSingleWorkerResult`: сравнивать samples, offsets, `final_cost`, `estimated_time_s`, `candidate_evaluations`, `collision_rejections`, новые worker/chunk stats;
   - добавить `RacingLine.ChunkedParallelIsDeterministicAcrossRuns`: 3 последовательных запуска с `parallel_workers=2/4`, одинаковый output bit-for-bit там, где сейчас используется `EXPECT_DOUBLE_EQ`;
   - добавить regression fixture `RacingLine.ParallelPreservesPerIndexOffsetDependency`: synthetic corridor, где accepted candidate на раннем `i` меняет лучший кандидат на `i + 1`; parallel результат должен совпадать с single-worker текущим алгоритмом;
   - negative/edge: маленький corridor на 2 samples должен не включать parallel и должен сохранять валидность/invalid status как раньше.

2. Добавить общий `ClearanceField` context для одного planning pipeline.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/clearance_field.hpp:18`
   - `drone_city_nav/src/clearance_field.cpp:154`
   - `drone_city_nav/include/drone_city_nav/planner_core.hpp:44`
   - `drone_city_nav/src/planner_core.cpp:595`
   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:57`
   - `drone_city_nav/src/trajectory_planner.cpp:117`
   - `drone_city_nav/src/corridor.cpp:219`

   Материализуемый результат:
   - оставить существующий `ClearanceFieldCache`, но добавить возможность передать уже построенный `ClearanceField2D` из `PlannerCore`/pipeline в `TrajectoryPlannerInput`;
   - `buildCorridor()` должен принимать optional `const ClearanceField2D* prohibited_clearance_field` или новый `CorridorInput`;
   - если поле передано и его `bounds/source/maxDistanceM` подходят, corridor использует его для `sample.clearance_m`, не строит новое поле на `corridor.cpp:219`;
   - если поле не передано, текущий standalone behaviour сохраняется для unit tests и вызовов без контекста;
   - добавить diagnostics: `clearance_field_reused_by_corridor`, `corridor_clearance_field_build_ms`, `corridor_clearance_field_cache_hit`.

   Псевдокод:

   ```cpp
   struct CorridorInput {
     std::span<const Point2> route_points;
     const OccupancyGrid2D& prohibited_grid;
     const ClearanceField2D* prohibited_clearance_field{nullptr};
   };

   const ClearanceField2D* field = input.prohibited_clearance_field;
   std::optional<ClearanceField2D> owned_field;
   if (field == nullptr || !fieldMatchesGrid(*field, input.prohibited_grid, max_radius)) {
     owned_field = ClearanceField2D::build(input.prohibited_grid, max_radius,
                                           ClearanceSource::kProhibited);
     field = &*owned_field;
   }
   ```

   Автотесты:
   - `Corridor.ReusesProvidedClearanceField`: результат samples/width/clearance совпадает с текущим `buildCorridor(route, grid, config)`, но stats показывают reuse;
   - `Corridor.RebuildsClearanceWhenProvidedFieldDoesNotMatch`: mismatch bounds/radius приводит к fallback build;
   - `PlannerCore.ComputePathReusesProhibitedClearanceFieldDiagnostics` уже есть на `planner_core_test.cpp:534`; расширить проверкой, что поле можно передать в trajectory/corridor без пересчёта;
   - `TrajectoryPlanner.UsesProvidedClearanceFieldForCorridorDiagnostics`: валидная траектория и `corridor_clearance_field_reused=true`.

3. Распараллелить независимый raw pass corridor samples.

   Файлы:
   - `drone_city_nav/src/corridor.cpp:227`
   - `drone_city_nav/src/corridor.cpp:260`
   - `drone_city_nav/src/corridor.cpp:276`
   - `drone_city_nav/include/drone_city_nav/corridor.hpp:35`
   - `drone_city_nav/tests/corridor_test.cpp:30`

   Материализуемый результат:
   - считать route point/tangent/normal/recovery/raycast bounds для каждого sample в worker chunks;
   - собирать `result.samples` строго в порядке `s_m`;
   - оставить `applyLocalLateralLimit()` отдельным последовательным pass после сборки raw samples, чтобы не менять математику local median;
   - аккумулировать per-worker stats (`outside_grid_samples`, `center_recovered_samples`, etc.) и детерминированно редуцировать их после worker pass;
   - добавить `CorridorStats`: `parallel_workers_used`, `sample_build_duration_ms`, `raycast_duration_ms`, `lateral_limit_duration_ms`.

   Псевдокод:

   ```cpp
   std::vector<std::optional<CorridorSample>> raw(sample_count + 1U);
   std::vector<CorridorStats> worker_stats(workers);

   parallelForChunks(sample_indices, workers, [&](auto indices, std::size_t worker_id) {
     for (std::size_t i : indices) {
       raw[i] = buildSingleCorridorSample(route, i, grid, config, field,
                                          worker_stats[worker_id]);
     }
   });

   for (auto& item : raw) {
     if (item.has_value()) {
       result.samples.push_back(*item);
     }
   }
   reduceStatsInWorkerOrder(worker_stats, result.stats);
   applyLocalLateralLimit(result.samples, config, result.stats);
   ```

   Автотесты:
   - добавить reference helper/test-only mode или internal serial helper и тест `Corridor.ParallelSamplesMatchSerialResult`: сравнить count, `s_m`, `center`, bounds, clearance, stats;
   - edge cases: route inside prohibited, outside grid, side opening with local lateral limit из существующих тестов `RouteInsideProhibitedIsInvalid`, `OutsideGridLimitsBounds`, `LocalLateralLimitClipsSideOpening`;
   - diagnostics test: `Corridor.ReportsParallelTimingAndWorkerCount`.

4. Расширить trajectory/planner diagnostics для headless-отладки ускорения.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:32`
   - `drone_city_nav/src/trajectory_planner.cpp:117`
   - `drone_city_nav/src/planner_node_publish.cpp:198`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp:368`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
   - `drone_city_nav/tests/final_trajectory_debug_io_test.cpp`

   Материализуемый результат:
   - добавить в JSON summary и ROS logs новые поля:
     - `racing_parallel_workers_used`
     - `racing_candidate_chunks`
     - `corridor_parallel_workers_used`
     - `corridor_sample_build_duration_ms`
     - `corridor_lateral_limit_duration_ms`
     - `clearance_field_reused_by_corridor`
     - `clearance_field_shared_build_duration_ms`
   - сохранить backward-compatible parsing: отсутствующие старые JSON поля должны парситься как default values;
   - в `planner_node_publish.cpp` логировать новые поля одной строкой рядом с существующими `timing[...]`.

   Автотесты:
   - `TrajectoryDiagnosticsIo.WritesAndParsesPerformanceFields`;
   - `FinalTrajectoryDebugIo.SummaryIncludesOptimizationTimings`;
   - existing parser tests должны проходить на JSON без новых полей.

5. Подготовить static grid + static inflation cache как отдельный более рискованный пакет.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp:37`
   - `drone_city_nav/src/planning_grid_builder.cpp:69`
   - `drone_city_nav/src/planning_grid_builder.cpp:114`
   - `drone_city_nav/tests/planning_grid_builder_test.cpp:103`

   Материализуемый результат:
   - вынести состояние кэша не в свободную функцию `buildPlanningGrid()`, а в небольшой объект, например `PlanningGridBuilder`, чтобы static cache жил между вызовами;
   - сохранить текущую свободную функцию как thin wrapper для тестов/совместимости или заменить вызовы после обновления тестов;
   - кэшировать:
     - `static_raw_grid`;
     - `static_prohibited_inflated` для `inflation_radius_m`;
     - `static_planning_inflated` для `inflation_radius_m + planning_clearance_m`;
   - при runtime update строить dynamic raw из memory/current lidar и OR-ить его inflation со static inflated;
   - строго проверить эквивалентность с текущим full rebuild.

   Псевдокод:

   ```cpp
   StaticGridCacheKey key{static_grid_fingerprint, bounds, inflation_radius_m,
                          planning_clearance_m};
   if (!static_cache.matches(key)) {
     static_cache.raw = overlayStaticOnly(...);
     static_cache.prohibited = static_cache.raw;
     static_cache.prohibited.rebuildInflation(inflation_radius_m);
     static_cache.planning = static_cache.raw;
     static_cache.planning.rebuildInflation(inflation_radius_m + planning_clearance_m);
   }

   OccupancyGrid2D dynamic_raw{bounds};
   overlayMemoryAndCurrentLidar(dynamic_raw, sources);
   OccupancyGrid2D dynamic_prohibited = dynamic_raw;
   dynamic_prohibited.rebuildInflation(inflation_radius_m);
   result.grid = orProhibitedMasks(static_cache.prohibited, dynamic_prohibited);
   ```

   Автотесты:
   - `PlanningGridBuilder.CachedStaticOnlyMatchesFullBuild`;
   - `PlanningGridBuilder.CachedStaticPlusMemoryMatchesFullBuild`;
   - `PlanningGridBuilder.CachedStaticPlusCurrentLidarMatchesFullBuild`;
   - invalidation tests: static map cell changes, bounds changes, resolution changes, `inflation_radius_m`, `planning_clearance_m`;
   - negative: dynamic sources must not mutate cached static grids.

6. A* оставить вне текущего пакета оптимизации.

   Файлы:
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/include/drone_city_nav/astar_planner.hpp`
   - `drone_city_nav/src/planner_node_config.cpp:116`
   - `drone_city_nav/config/urban_mvp.yaml:96`

   Материализуемый результат:
   - не внедрять bidirectional A*, JPS или hierarchical planning в этом этапе;
   - не менять `astar_heuristic_weight`, `turn_cost_weight`, initial heading bias;
   - сохранить текущие логи `expanded`, `heuristic_weight`, `cost`, чтобы сравнение после ускорения было честным.

   Автотесты:
   - новых A* тестов в этом пакете не требуется;
   - существующие `planner_core_test` и `planner_node_config_test` остаются regression coverage.

7. Обновить конфиги только для новых diagnostics/worker controls, если они реально нужны.

   Файлы:
   - `drone_city_nav/src/planner_node_config.cpp:178`
   - `drone_city_nav/config/urban_mvp.yaml:121`
   - `drone_city_nav/tests/planner_node_config_test.cpp:73`

   Материализуемый результат:
   - если `racing_line_parallel_workers` остаётся runtime-параметром, сохранить `0 = auto`;
   - если появится `corridor_parallel_workers`, использовать такой же контракт `0 = auto`, `1 = serial/reference`, `N = fixed`;
   - defaults в C++ и YAML должны совпадать;
   - никакие legacy/off switches не добавлять без явной необходимости.

   Автотесты:
   - `PlannerNodeConfig.DefaultsKeepAutoWorkers`;
   - `PlannerNodeConfig.ClampsParallelWorkers`;
   - если параметр не вводится и используется always-auto, тестировать только новые diagnostics defaults.

# Verification plan

После реализации каждого пакета:

1. Отформатировать изменённые C++ файлы:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Запустить scoped tests внутри контейнера после build:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure -R 'racing_line_test|corridor_test|clearance_field_test|planner_core_test|trajectory_planner_test|planning_grid_builder_test|trajectory_diagnostics_io_test|final_trajectory_debug_io_test|planner_node_config_test'
   ```

3. Запустить полный package test:

   ```bash
   ./scripts/test.sh
   ```

4. Запустить quality gate:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

5. Headless verification только после явной команды на симуляционный прогон:

   ```bash
   ./scripts/sim_headless.sh
   ```

   После него проверить:
   - `log/ros_city_mvp.log`: `Planning summary`, `final racing trajectory`, `Published path`;
   - `log/final_trajectory_samples/latest_summary.json`: total/corridor/racing timings, worker/chunk diagnostics;
   - отсутствие изменения маршрута сверх ожидаемых floating-point микродельт для пакетов 1-3.

# Testing strategy

1. Категория 1: без рефакторинга.

   Подходит для `racing_line` buffer/chunk optimization и diagnostics fields.

   Тесты:
   - deterministic equivalence tests в `racing_line_test.cpp`;
   - JSON/log field tests в `trajectory_diagnostics_io_test.cpp`;
   - scoped `ctest -R 'racing_line_test|trajectory_diagnostics_io_test|final_trajectory_debug_io_test'`.

2. Категория 2: лёгкий рефакторинг.

   Подходит для shared `ClearanceField` context и parallel corridor raw sample pass.

   Тесты:
   - serial-vs-parallel corridor equivalence;
   - reuse/fallback tests для provided clearance field;
   - trajectory planner integration test, который проверяет, что `planTrajectory()` использует shared field и сохраняет валидный speed profile.

3. Категория 3: тяжёлый рефакторинг.

   Подходит для static grid + static inflation cache.

   Тесты:
   - full-build vs cached-build equivalence на static-only, static+memory, static+current-lidar;
   - cache invalidation matrix;
   - package-level `./scripts/test.sh`;
   - headless run после явной команды пользователя, потому что это затрагивает runtime planning grid.

# Risks and tradeoffs

- `racing_line` parallel rewrite может дать микроскопические отличия, если менять порядок acceptance или суммирования stats. Митигировать deterministic ordered reduction и bit-for-bit tests.
- Worker-local scratch снижает аллокации, но повышает сложность lifetime/ownership. Не хранить `std::span` на временные buffers за пределами worker call.
- Shared `ClearanceField` должен строго соответствовать grid, radius и source. Ошибка matching приведёт к неверным clearance diagnostics/corridor clearance.
- Parallel corridor raw pass безопасен только до `applyLocalLateralLimit()`. Сам local lateral limit пока оставить последовательным, иначе можно изменить поведение.
- Static grid/inflation cache даёт большой выигрыш, но самый рискованный: неправильное OR/overlay поведение может изменить prohibited/planning grid и маршрут. Делать отдельным коммитом с equivalence tests.
- Ускорение не должно отключать или пассивизировать lidar/memory/current-lidar источники.
- A* не трогаем, потому что сейчас он занимает около `108.7ms` и не является bottleneck.

# Open questions

- Нужен ли отдельный runtime-параметр `corridor_parallel_workers`, или corridor должен всегда использовать auto workers с test-only serial helper? Для production предпочтительнее auto без нового user-facing switch, но serial reference полезен в тестах.
- Нужно ли сохранять `racing_line_parallel_workers` как пользовательский параметр после перехода на chunked workers? Сейчас он уже есть в YAML и тестах; удаление параметра лучше делать отдельным cleanup, не в пакете ускорения.
- Допустимо ли считать `candidate_path_evaluation_duration_ms` и `candidate_score_duration_ms` агрегированным CPU-time по workers, а не wall-clock? Если да, надо явно добавить отдельные wall-clock fields, чтобы headless logs не вводили в заблуждение.
