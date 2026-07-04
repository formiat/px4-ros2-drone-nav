# Context

Задача: подготовить план рефакторинга и разбиения крупных C++ файлов без изменения поведения навигации, построения траектории и ROS-публикаций.

Целевые файлы и текущий размер:

| Файл | Строк | Почему разбиваем |
| --- | ---: | --- |
| `drone_city_nav/src/trajectory_optimizer.cpp` | 2929 | В одном translation unit смешаны геометрия траектории, active windows, DP, candidate evaluation, worker pool, scoring, diagnostics и публичный orchestration. |
| `drone_city_nav/src/planner_node_publish.cpp` | 1422 | В одном файле смешаны публикация пути, async refined trajectory lifecycle, safety logging, lidar overlay, RViz/grid publication и CSV/JSON dumps. |
| `drone_city_nav/src/turn_smoothing.cpp` | 1313 | В одном файле смешаны детекция углов, построение Bezier-кандидатов, traversability/collision cache, scoring, диагностика и публичный pass. |
| `drone_city_nav/src/trajectory_diagnostics_io.cpp` | 1411 | В одном файле смешаны CSV writer, JSON writer, JSON parser, speed-profile fragments и низкоуровневые parser helpers. |
| `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp` | 1320 | Один огромный fixture и разные типы тестов: CSV, JSON fragments, round-trip parser, baseline-quality fields. |
| `drone_city_nav/tests/offboard_velocity_follower_test.cpp` | 1003 | В одном файле смешаны speed-profile tests, lateral-control tests, terminal-capture tests и invalid-input tests. |

Цель плана: разбить файлы по смысловым границам, оставить публичные API стабильными, сохранить текущую математику и порядок выбора траектории, а также упростить будущую работу с optimizer/smoothing/control tests.

# Investigation context

`INVESTIGATION.md` в рабочей директории отсутствует. Дополнительный контекст из отдельного investigation-файла не применялся.

`PLAN.md` до этой задачи отсутствовал, поэтому этот файл создается с нуля.

# Detected stack/profiles

Примененные профили:

| Профиль | Почему выбран |
| --- | --- |
| `generic.md` | Это обычный git-репозиторий с локальными workflow-документами. Нужно определить стек, команды и проверки из репозитория. |
| `cpp.md` | В задаче затрагиваются `.cpp`/`.hpp`, в репозитории есть `drone_city_nav/CMakeLists.txt`, `package.xml`, `build/compile_commands.json`; пакет использует C++20 и ROS 2 `ament_cmake`. |

Не применялись:

| Профиль | Причина |
| --- | --- |
| Notion protocol | В inbox не указан Notion task id и не требуется читать Notion. |
| GitLab protocol | В inbox не требуется создавать или обновлять MR, читать issues/MR или обращаться к GitLab. |
| Rust profile | В целевом workspace нет релевантного Rust-кода для этой задачи. |

# Repo-approved commands found

Команды из `README.md`, `CONTRIBUTING.md`, `Makefile` и `AGENTS.md`:

| Цель | Команда |
| --- | --- |
| Build в контейнере | `./scripts/build.sh` |
| Tests в контейнере | `./scripts/test.sh` |
| GUI simulation | `./scripts/sim_gui.sh` |
| Headless simulation | `./scripts/sim_headless.sh` |
| Остановить simulation | `./scripts/stop_sim.sh` |
| Команды внутри контейнера | `./scripts/dev_shell.sh <command>` |
| Build внутри контейнера | `make build` |
| Tests внутри контейнера | `make test` |
| Script tests внутри контейнера | `make test-scripts` |
| Quality внутри контейнера | `make quality` |
| Format внутри контейнера | `make format` |

Запрещенный/нежелательный путь: ad-hoc top-level CMake commands. Для этой задачи нужно использовать `colcon` через Makefile/scripts.

# Affected components

| Компонент | Текущие якоря | Результат после рефакторинга |
| --- | --- | --- |
| Trajectory optimizer core | `drone_city_nav/src/trajectory_optimizer.cpp`, public API в `drone_city_nav/include/drone_city_nav/trajectory_optimizer.hpp` | Публичный `optimizeTrajectory(...)` остается в `trajectory_optimizer.cpp`; детали уходят в приватные `.cpp` и `trajectory_optimizer_internal.hpp`. |
| Planner publishing node | `drone_city_nav/src/planner_node_publish.cpp`, declarations в `drone_city_nav/src/planner_node.hpp` | Методы `PlannerNode` распределяются по publish/refinement/debug/dumps файлам без изменения класса. |
| Turn smoothing | `drone_city_nav/src/turn_smoothing.cpp`, public API в `drone_city_nav/include/drone_city_nav/turn_smoothing.hpp` | Публичный `smoothTrajectoryTurns(...)` остается orchestration-точкой; детали уходят в приватные `.cpp` и `turn_smoothing_internal.hpp`. |
| Trajectory diagnostics IO | `drone_city_nav/src/trajectory_diagnostics_io.cpp`, public API в `drone_city_nav/include/drone_city_nav/trajectory_diagnostics_io.hpp` | CSV/JSON writer/parser разделены на отдельные implementation files. |
| CMake source lists | `drone_city_nav/CMakeLists.txt:29`, `drone_city_nav/CMakeLists.txt:102`, `drone_city_nav/CMakeLists.txt:299`, `drone_city_nav/CMakeLists.txt:370` | Добавлены новые `.cpp` в `drone_city_nav_core`, `planner_node` и новые/разделенные test targets. |
| Diagnostics IO tests | `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp` | Общие fixtures вынесены в helper header; тесты разделены по CSV/fragments/parser. |
| Offboard velocity follower tests | `drone_city_nav/tests/offboard_velocity_follower_test.cpp` | Общие fixtures вынесены в helper header; тесты разделены по speed profile/lateral control/terminal capture. |

# Implementation steps

1. Зафиксировать baseline перед рефакторингом.

   Файлы: `drone_city_nav/CMakeLists.txt`, все целевые `.cpp`/tests.

   Действия:
   - Проверить `git status --short`.
   - Снять список текущих тестовых таргетов из `drone_city_nav/CMakeLists.txt`.
   - Не менять поведение и параметры в этом рефакторинге.

   Материализованный результат:
   - Чистая отправная точка.
   - Понятно, какие CMake target lists нужно обновить.

2. Разбить `trajectory_optimizer.cpp` через приватный internal API.

   Файлы:
   - Добавить `drone_city_nav/src/trajectory_optimizer_internal.hpp`.
   - Оставить `drone_city_nav/src/trajectory_optimizer.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_geometry.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_active_windows.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_scoring.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_candidates.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_dp.cpp`.
   - Добавить `drone_city_nav/src/trajectory_optimizer_workers.cpp`.

   Текущие якоря:
   - Internal structs сейчас начинаются около `trajectory_optimizer.cpp:33`: `PathEvaluation`, `CostBreakdown`, `CandidateScore`, `CandidateTask`, `CandidateWorkBuffer`, `TrajectoryOptimizerScratch`, `CandidateBatchWorkspace`, `ActiveWindow`.
   - Geometry helpers: `pointsFromOffsets`, `samplesFromPointsAndOffsets`, `applyOffsetDelta`, `optimizerCorridorSamples`, `pathLength`, `headingDeltaRad`, `discreteCurvature`, `edgeMarginM`.
   - Active windows: `addActiveWindow`, `mergeActiveWindows`, `detectActiveWindows`.
   - Scoring: `costBreakdownForPoints`, `geometrySubtotal`, `localGeometryCostForChangedSpan`, `scoreForCandidate`, `populateSampleGeometry`.
   - Candidate execution: `evaluateCandidateSnapshot`, `offsetCandidatesForSample`, `candidateTasksForIteration`, `evaluateCandidateBatch`, `mergeCandidateStats`.
   - DP/coarse-to-fine: `buildDpSeedForWindow`, `smoothedOffsets`.
   - Public orchestration: `optimizeTrajectory(...)`.

   Принцип:
   - Все типы и helpers, которые нужны нескольким `.cpp`, перенести из anonymous namespace в `namespace drone_city_nav::trajectory_optimizer_detail`.
   - Публичный header `include/drone_city_nav/trajectory_optimizer.hpp` не расширять internal-деталями.
   - `optimizeTrajectory(...)` оставить единственной публичной точкой и orchestration-файлом.

   Материализованный результат:
   - `trajectory_optimizer.cpp` содержит orchestration, input validation, final result assembly и вызовы detail-функций.
   - Геометрия, windows, scoring, DP и workers компилируются отдельно.
   - `drone_city_nav/CMakeLists.txt:29` содержит новые optimizer `.cpp` в `drone_city_nav_core`.

3. Не менять алгоритм optimizer при переносе.

   Файлы: все `trajectory_optimizer_*.cpp`.

   Действия:
   - Сохранять текущий порядок iteration/sample/candidate loops.
   - Сохранять deterministic winner selection.
   - Не менять weights, thresholds, active window logic, DP transitions, candidate diagnostics.
   - Если helper был в anonymous namespace и стал `detail`, сделать минимально необходимую сигнатуру без новых side effects.

   Материализованный результат:
   - Diff должен быть mechanical move plus namespace/include/CMake glue.
   - `trajectory_optimizer_test` и интеграционные trajectory tests не требуют изменения ожиданий.

4. Разбить `planner_node_publish.cpp` по runtime-ролям.

   Файлы:
   - Оставить тонкий `drone_city_nav/src/planner_node_publish.cpp`.
   - Добавить `drone_city_nav/src/planner_node_trajectory_publication.cpp`.
   - Добавить `drone_city_nav/src/planner_node_refinement.cpp`.
   - Добавить `drone_city_nav/src/planner_node_safety.cpp`.
   - Добавить `drone_city_nav/src/planner_node_lidar_overlay.cpp`.
   - Добавить `drone_city_nav/src/planner_node_debug_publication.cpp`.
   - Добавить `drone_city_nav/src/planner_node_dumps.cpp`.

   Текущие якоря:
   - Path publication: `publishPathFromPathCells`, `publishTrajectoryResult`, `keepCurrentPathAfterInvalidReplacement`.
   - Async refinement: `startAsyncTrajectoryRefinement`, `launchScheduledTrajectoryRefinement`, `launchQueuedTrajectoryRefinement`, `pollPendingTrajectoryRefinement`.
   - Safety: `summarizePublishedPathSafety`, `logPublishedPathSafety`, `connectRouteToCurrentPose`, `logRejectedUnsafeRoute`.
   - Lidar overlay: `currentLidarRangeMax`, `currentLidarPoseReceiveLagSeconds`, `currentLidarProjectionPose`, `currentLidarProjectionConfig`, `overlayCurrentLidarHits`.
   - Debug publication and dumps: `makePlannerHeader`, `publishStaticMapDebug`, `republishStaticMapDebug`, `publishProhibitedGrid`, `publishPath`, `publishTrajectoryDiagnostics`, `write*Dump`, `publishPlanningFailureHold`.

   Принцип:
   - Не менять `PlannerNode` declaration в `planner_node.hpp`, кроме возможной перестановки private declarations по тем же группам.
   - Не создавать новый runtime state.
   - Вынести только method definitions и локальные helpers.

   Материализованный результат:
   - `planner_node_publish.cpp` перестает быть смешанным файлом.
   - `drone_city_nav/CMakeLists.txt:102` содержит новые planner node source files в `planner_node`.

5. Разбить `turn_smoothing.cpp` через приватный internal API.

   Файлы:
   - Добавить `drone_city_nav/src/turn_smoothing_internal.hpp`.
   - Оставить `drone_city_nav/src/turn_smoothing.cpp`.
   - Добавить `drone_city_nav/src/turn_smoothing_geometry.cpp`.
   - Добавить `drone_city_nav/src/turn_smoothing_candidates.cpp`.
   - Добавить `drone_city_nav/src/turn_smoothing_metrics.cpp`.
   - Добавить `drone_city_nav/src/turn_smoothing_diagnostics.cpp`.

   Текущие якоря:
   - Geometry/math helpers: `norm`, `heading`, `headingDelta`, `discreteCurvature`, `populateSampleGeometry`, `pathLength`, `corridorWidthAt`, `corridorCenterAt`.
   - Collision/cache: `SegmentCellKey`, `SegmentTraversabilityCache`, `segmentIsTraversable`, `cachedSegmentIsTraversable`, `pathIsTraversableCached`.
   - Candidate model: `CornerCandidate`, `SmoothingRejectReason`, `LocalTrajectoryMetrics`, `SmoothingAttempt`, `BezierCacheKey`, `TurnSmoothingWorkBuffer`.
   - Candidate build: `cornerCandidateAt`, `entryIndexFor`, `exitIndexFor`, `outwardShiftFor`, `cubicBezier`, `tangentRelaxedOutward`, `buildBezierSamples`.
   - Metrics/scoring: `replaceRange`, `localTrajectoryMetrics`, `cachedBeforeMetrics`, `smoothingAttemptScore`, `shapeImprovementRejectDetail`, `candidateRegressionReason`, `trySmoothCorner`.
   - Diagnostics: `populateAttemptSpeedDiagnostics`, `updateCandidateSpeedDiagnostics`, `incrementRejectStat`.
   - Public orchestration: `smoothTrajectoryTurns(...)`.

   Принцип:
   - Internal namespace: `drone_city_nav::turn_smoothing_detail`.
   - Public `include/drone_city_nav/turn_smoothing.hpp` не должен получать implementation-only structs.
   - Сохранить текущий полный перебор candidates и текущие diagnostic fields.

   Материализованный результат:
   - `smoothTrajectoryTurns(...)` читает как pipeline.
   - Candidate generation, metrics, collision и diagnostics тестируются текущими внешними tests без изменения expected values.

6. Разбить `trajectory_diagnostics_io.cpp` на writer/parser части.

   Файлы:
   - Добавить `drone_city_nav/src/trajectory_diagnostics_io_internal.hpp`.
   - Оставить `drone_city_nav/src/trajectory_diagnostics_io.cpp` для тонких public wrappers или удалить из CMake только если все public functions перенесены.
   - Добавить `drone_city_nav/src/trajectory_diagnostics_io_csv.cpp`.
   - Добавить `drone_city_nav/src/trajectory_diagnostics_io_json_fields.cpp`.
   - Добавить `drone_city_nav/src/trajectory_diagnostics_io_json_summary.cpp`.
   - Добавить `drone_city_nav/src/trajectory_diagnostics_io_parser.cpp`.

   Текущие якоря:
   - CSV: `finalTrajectorySamplesCsvHeader`, `finalTrajectorySamplesCsvRow`.
   - JSON field writers: `trajectoryOptimizerDiagnosticsJsonFields`, `turnSmoothingDiagnosticsJsonFields`, `speedProfileConstraintDiagnosticsJsonFields`, `trajectoryTimingDiagnosticsJsonFields`.
   - Summary/envelope writer: `trajectoryPlannerDiagnosticsJson`.
   - Parser: `parseTrajectoryPlannerDiagnosticsJson`.
   - Shared helpers: `appendJsonStringField`, `appendJsonNumberField`, `parseJsonNumber`, `parseJsonBool`, enum parsers.

   Принцип:
   - Public header `include/drone_city_nav/trajectory_diagnostics_io.hpp` остается стабильным.
   - Parser helpers, JSON append helpers и enum parser helpers уходят в private header/detail namespace только если реально нужны нескольким `.cpp`.
   - Не менять имена JSON fields и CSV columns.

   Материализованный результат:
   - Writer-side tests и parser-side tests можно запускать и читать независимо.
   - `trajectory_diagnostics_io.cpp` больше не является монолитом.

7. Обновить `drone_city_nav/CMakeLists.txt`.

   Файлы: `drone_city_nav/CMakeLists.txt`.

   Действия:
   - В `drone_city_nav_core` добавить новые `trajectory_optimizer_*.cpp`, `turn_smoothing_*.cpp`, `trajectory_diagnostics_io_*.cpp`.
   - В `planner_node` добавить новые `planner_node_*.cpp`.
   - Сохранить `target_link_libraries`, `ament_target_dependencies`, `enable_project_warnings`.
   - Не добавлять приватные headers в install list.

   Материализованный результат:
   - `make build` собирает те же libraries/executables.
   - Public install surface не расширяется internal headers.

8. Разбить `trajectory_diagnostics_io_test.cpp`.

   Файлы:
   - Добавить `drone_city_nav/tests/trajectory_diagnostics_io_test_helpers.hpp`.
   - Добавить `drone_city_nav/tests/trajectory_diagnostics_io_csv_test.cpp`.
   - Добавить `drone_city_nav/tests/trajectory_diagnostics_io_json_fields_test.cpp`.
   - Добавить `drone_city_nav/tests/trajectory_diagnostics_io_roundtrip_test.cpp`.
   - Удалить или сократить `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`.

   Текущие якоря:
   - Fixture `populatedStats()` сейчас начинается около `trajectory_diagnostics_io_test.cpp:21`.
   - CSV test: `CsvHeaderAndRowContainProfiledTiming`.
   - Summary/fragment tests: `SummaryJsonContainsTraversalAndShapeMetrics`, optimizer/turn/timing fragment tests.
   - Round-trip parser tests: `PlannerDiagnosticsJsonRoundTripsRuntimeStats`, `PlannerDiagnosticsJsonExposesBaselineQuality`.

   Принцип:
   - Fixture перенести в helper header как inline helper в test-only namespace.
   - Лучше создать несколько gtest targets, а не один огромный target с одним файлом.

   Материализованный результат:
   - Тесты diagnostics IO разделены по ответственности.
   - CMake содержит новые `ament_add_gtest(...)` entries вместо одного большого target или один target с несколькими source files, если нужен минимальный CMake churn.

9. Разбить `offboard_velocity_follower_test.cpp`.

   Файлы:
   - Добавить `drone_city_nav/tests/offboard_velocity_follower_test_helpers.hpp`.
   - Добавить `drone_city_nav/tests/offboard_velocity_follower_speed_profile_test.cpp`.
   - Добавить `drone_city_nav/tests/offboard_velocity_follower_lateral_control_test.cpp`.
   - Добавить `drone_city_nav/tests/offboard_velocity_follower_terminal_capture_test.cpp`.
   - Добавить `drone_city_nav/tests/offboard_velocity_follower_invalid_input_test.cpp` или оставить invalid-input в самом маленьком target.
   - Удалить или сократить `drone_city_nav/tests/offboard_velocity_follower_test.cpp`.

   Текущие якоря:
   - Helpers: `testConfig`, `lineTrajectory`, `trajectoryWithArc`, `normalizedTestVector`.
   - Speed/profile group: tests from `StraightTrajectoryReturnsCruiseVelocityAlongTangent` through `ProfiledTraversalTimeAccountsForFinalStop`.
   - Lateral/control group: tests from `VectorDeltaLimitClampsAbruptDirectionChange` through `VelocityJerkLimitSmoothsDirectionChange`.
   - Terminal group: `FinalGoalReachedRequestsHold`, `FastGoalFlyThroughUsesTerminalCaptureUntilSlow`, terminal final-plane tests.
   - Invalid input: `EmptyTrajectoryReturnsInvalidPlan`, `NonFinitePositionReturnsInvalidPlan`.

   Принцип:
   - Helper header only for test fixtures, not production code.
   - Preserve existing assertions exactly unless names need to move.

   Материализованный результат:
   - Control tests легче запускать/читать по группе.
   - Future terminal-capture changes не затрагивают speed-profile file.

10. Проверить отсутствие циклических include и утечки internal API.

   Файлы:
   - `drone_city_nav/src/*_internal.hpp`.
   - Public headers under `drone_city_nav/include/drone_city_nav/`.

   Действия:
   - Internal headers include only what is needed.
   - Public headers do not include private internal headers.
   - Forward declarations использовать там, где это снижает compile coupling без ухудшения читаемости.

   Материализованный результат:
   - `make quality` не ловит unused includes, warnings, clang-tidy issues.

11. Сохранить behavior-equivalence через тесты и targeted review.

   Файлы:
   - Все новые/перенесенные `.cpp` и tests.

   Действия:
   - Проверить, что refactor не меняет default config values.
   - Проверить, что diagnostics names/CSV headers/JSON parser keys не изменены.
   - Проверить, что optimizer stats counters продолжают инкрементироваться в тех же местах.
   - Проверить, что planner does not publish empty path при invalid refined replacement остается как сейчас.

   Материализованный результат:
   - После переноса diff по логике должен быть механическим.
   - Любое изменение поведения выносить в отдельный commit после refactor, не смешивать.

12. Коммитить рефакторинг пачками, если реализация окажется большой.

   Рекомендуемый порядок commits:
   - `Refactor trajectory optimizer internals`
   - `Split planner publication helpers`
   - `Split turn smoothing internals`
   - `Split trajectory diagnostics IO`
   - `Split large diagnostics and velocity follower tests`

   Материализованный результат:
   - Каждая пачка компилируется или хотя бы проходит scoped build/test.
   - Откат конкретной части возможен без отката всего рефакторинга.

# Verification plan

Минимальная проверка после каждого большого move-only шага:

1. `./scripts/dev_shell.sh make build`

Полная проверка перед финальным commit:

1. `./scripts/dev_shell.sh make format`
2. `./scripts/dev_shell.sh make quality`

Дополнительные targeted тесты после разделения тестов:

1. `./scripts/dev_shell.sh make test`
2. При необходимости внутри контейнера после build: `ctest --test-dir build/drone_city_nav --output-on-failure -R "trajectory_optimizer|turn_smoothing|trajectory_diagnostics_io|offboard_velocity_follower"`

Simulation не является обязательной для чистого refactor, но после полного разбиения полезен smoke run:

1. `./scripts/sim_headless.sh`

# Testing strategy

Категория 1 - без рефакторинга.

Цель: baseline и safety net.

Проверки:
1. `./scripts/dev_shell.sh make quality`
2. Если надо сравнить runtime, короткий headless run текущей ветки и сохранить последние trajectory/planner logs.

Категория 2 - легкий рефакторинг.

Что сюда входит:
1. Разбить `trajectory_diagnostics_io.cpp`.
2. Разбить два больших test files.
3. Разбить `planner_node_publish.cpp` на member-function implementation files.

Почему легче:
1. Меньше риска изменить математику trajectory optimizer.
2. Public APIs почти не меняются.
3. Tests в основном подтверждают string/JSON/output behavior.

Проверки:
1. `./scripts/dev_shell.sh make build`
2. `./scripts/dev_shell.sh make test`
3. `./scripts/dev_shell.sh make quality`

Категория 3 - тяжелый рефакторинг.

Что сюда входит:
1. Разбить `trajectory_optimizer.cpp`.
2. Разбить `turn_smoothing.cpp`.

Почему тяжелее:
1. Сейчас много типов живет в anonymous namespace одного `.cpp`.
2. При переносе в private detail namespace легко случайно поменять lifetime/cache/stat counters.
3. Эти файлы напрямую влияют на форму траектории и сглаживание поворотов.

Проверки:
1. `./scripts/dev_shell.sh make build` после каждого крупного переноса.
2. `./scripts/dev_shell.sh make test`.
3. `./scripts/dev_shell.sh make quality`.
4. Сравнить ключевые метрики последнего/контрольного run: optimizer samples, active windows, candidate evals, final length, curvature/radius stats, turn smoothing accepted/rejected counters.
5. После завершения - один `./scripts/sim_headless.sh` или GUI smoke run, если нужна визуальная проверка траектории.

# Risks and tradeoffs

1. Риск изменения поведения из-за anonymous namespace.

   Причина: перенос internal helpers в private header/detail namespace меняет linkage и может открыть helper нескольким translation units.

   Митигация: internal headers класть в `src/`, не устанавливать, использовать `*_detail` namespace, не добавлять новые public declarations.

2. Риск нарушения deterministic candidate selection.

   Причина: optimizer содержит parallel candidate evaluation и worker buffers.

   Митигация: переносить worker pool и batch evaluation без изменения порядка записи `results[task_index]` и последующего sequential winner selection.

3. Риск потери diagnostics counters.

   Причина: stats сейчас инкрементируются в горячих helper-функциях.

   Митигация: при переносе scoring/collision/speed diagnostics сохранить владельца stats и обновлять те же поля в тех же ветках.

4. Риск CMake churn.

   Причина: `drone_city_nav/CMakeLists.txt` содержит явные списки source files и gtest targets.

   Митигация: обновлять CMake вместе с каждым split, запускать build после каждой пачки.

5. Риск ухудшения compile-time из-за private headers.

   Причина: большой `trajectory_optimizer_internal.hpp` может включать много типов.

   Митигация: держать internal headers минимальными; если header разрастается, разделить на `*_types.hpp`, `*_geometry.hpp`, `*_scoring.hpp`.

6. Tradeoff между количеством файлов и навигацией по коду.

   Решение: не дробить до уровня одной функции на файл. Делить по ролям: geometry/windows/scoring/candidates/workers/diagnostics.

# Open questions

1. Нужно ли делать refactor одним большим commit или несколькими локальными commits по компонентам? Рекомендация: несколько commits, потому что optimizer и tests можно проверять независимо.

2. Оставлять ли старые test target names для CI совместимости? Рекомендация: если external scripts запускают `trajectory_diagnostics_io_test` или `offboard_velocity_follower_test` по имени, сохранить aggregate target с несколькими source files. Если таких scripts нет, лучше разделить на несколько targets.

3. Нужен ли отдельный namespace для private implementation: `trajectory_optimizer_detail` или nested `detail` внутри `drone_city_nav`? Рекомендация: именованный component-detail namespace, чтобы избежать конфликтов между optimizer/smoothing diagnostics helpers.

4. Делать ли дополнительные unit tests для moved internal helpers? Рекомендация: не открывать internal helpers ради тестов в первом refactor. Сначала сохранить public behavior tests. Internal tests добавлять только если появится реальная необходимость.

5. Нужно ли запускать simulation для plan-only change? Для создания этого `PLAN.md` simulation не нужна; для будущего тяжелого refactor optimizer/smoothing - желательна smoke-проверка после `make quality`.
