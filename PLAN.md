# Context

Нужно запланировать оптимизации времени построения пути и финальной траектории без изменения навигационного поведения там, где это возможно. Целевые зоны из запроса:

1. убрать двойное построение `ClearanceField2D` для диагностического clearance в `PlannerCore`;
2. убрать лишние временные `std::vector` в `racing_line`;
3. кэшировать строго неизменяемые данные только при строгом cache key;
4. отдельно рассмотреть параллельную оценку racing-line candidates с сохранением детерминированного выбора.

По последнему локальному GUI-логу `log/ros_city_mvp.log:197` и `log/ros_city_mvp.log:288` актуальные timings такие:

- planning grid build: `1803.1 ms`;
- `PlannerCore::computePath`: `1375.1 ms`, из них A* `119.7 ms`, smoothing `59.6 ms`, `raw_clearance=589.4 ms`, `smoothed_clearance=582.2 ms`;
- final trajectory build: `2205.1 ms`, из них corridor `843.0 ms`, racing line `1361.7 ms`, turn smoothing `0.3 ms`, speed profile `0.0 ms`;
- внутри racing line: candidate path evaluation `767.6 ms`, candidate score `551.1 ms`, `4716` candidate evaluations.

Это подтверждает, что самый дешёвый по риску выигрыш сейчас находится не в A*, а в diagnostic clearance и в стоимости per-candidate работы racing-line optimizer.

# Investigation context

`INVESTIGATION.md` в `workspace_root` отсутствует. Существующий `PLAN.md` тоже отсутствовал, поэтому этот файл создаётся как новый план.

# Detected stack/profiles

Стек workspace:

- ROS 2 workspace;
- C++20 пакет `drone_city_nav`, ament CMake, `colcon`;
- Python helper scripts/tests есть, но затрагиваемые компоненты этого плана находятся в C++ core.

Прочитанные обязательные профили:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md` - общий профиль обязателен всегда;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md` - выбран, потому что в workspace есть `CMakeLists.txt`, `Makefile`, `.cpp/.hpp`, а затрагиваемые компоненты C++;
- `rust.md` не применялся: в целевом workspace нет признаков Rust-проекта для затрагиваемой части.

Дополнительно прочитаны обязательные протоколы:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`.

Notion/GitLab в пользовательском запросе не упоминаются, поэтому read-only CLI-запросы к ним не нужны. Политика Notion `optional`.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`, `AGENTS.md`:

- build: `./scripts/build.sh`;
- tests: `./scripts/test.sh`;
- script tests: внутри контейнера `make test-scripts`;
- quality: внутри контейнера `make quality`;
- format changed C++: внутри контейнера `make format`;
- GUI/headless simulation: `./scripts/sim_gui.sh`, `./scripts/sim_headless.sh`;
- stop leftovers: `./scripts/stop_sim.sh`.

Правило проекта: использовать только container workflow; не запускать ad-hoc top-level CMake. Для этой задачи симуляционные прогоны не нужны и не должны запускаться без прямой команды пользователя.

# Affected components

- `drone_city_nav/src/planner_core.cpp:185` - `pathMinimumProhibitedClearanceM()` сейчас строит `ClearanceField2D` внутри каждого вызова.
- `drone_city_nav/src/planner_core.cpp:472` и `drone_city_nav/src/planner_core.cpp:478` - `computePath()` вызывает clearance diagnostics отдельно для raw и smoothed path.
- `drone_city_nav/include/drone_city_nav/planner_core.hpp:43` - `PathComputationResult` хранит timing поля для path computation diagnostics.
- `drone_city_nav/src/clearance_field.cpp:67` - построение `ClearanceField2D`, потенциальная точка для cache/fingerprint reuse.
- `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp:40` - сейчас наружу доступен `cells()`, но нет доступа к inflation mask/generation/fingerprint; строгий cache key для prohibited state без доработки API невозможен.
- `drone_city_nav/src/corridor.cpp:221` - corridor строит `ClearanceField2D` для `ClearanceSource::kProhibited` и `max_radius`.
- `drone_city_nav/src/racing_line.cpp:102` - `pointsFromOffsets()` каждый раз возвращает новый vector.
- `drone_city_nav/src/racing_line.cpp:127` - `samplesFromPointsAndOffsets()` каждый раз возвращает новый vector.
- `drone_city_nav/src/racing_line.cpp:388` - `scoreForCandidate()` создаёт `samples` для каждого traversable candidate.
- `drone_city_nav/src/racing_line.cpp:525` - `updateBestCandidate()` оценивает candidate path и score, пишет timings.
- `drone_city_nav/src/racing_line.cpp:611` - основной optimizer loop создаёт `best_offsets`, `candidate_offsets`, `candidate_points`, `accepted_offsets`, `accepted_points` на каждой итерации/кандидате.
- `drone_city_nav/src/racing_line.cpp:678` - regularization loop создаёт `candidate_offsets`, `candidate_points`, `candidate_samples`.
- `drone_city_nav/include/drone_city_nav/racing_line.hpp:36` - `RacingLineStats` уже содержит timings `candidate_path_evaluation_duration_ms` и `candidate_score_duration_ms`.
- `drone_city_nav/src/trajectory_planner.cpp:101` - pipeline: route -> corridor -> racing line -> turn smoothing -> speed profile.
- `drone_city_nav/src/planner_node_publish.cpp:198` - ROS summary log уже печатает total/corridor/racing_line/turn_smoothing/speed_profile и racing candidate timings.
- `drone_city_nav/src/trajectory_diagnostics_io.cpp:350` - JSON diagnostics уже содержит trajectory stage timings.
- Tests: `drone_city_nav/tests/planner_core_test.cpp`, `drone_city_nav/tests/clearance_field_test.cpp`, `drone_city_nav/tests/racing_line_test.cpp`, `drone_city_nav/tests/trajectory_planner_test.cpp`, `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`.

# Implementation steps

1. Оптимизировать diagnostic clearance в `PlannerCore`.

   Файлы:

   - `drone_city_nav/src/planner_core.cpp:185`
   - `drone_city_nav/src/planner_core.cpp:472`
   - `drone_city_nav/include/drone_city_nav/planner_core.hpp:43`
   - `drone_city_nav/tests/planner_core_test.cpp`

   Материализуемый результат:

   - добавить internal helper, который считает minimum clearance от уже построенного поля:

     ```cpp
     double pathMinimumClearanceM(const ClearanceField2D& field,
                                  std::span<const GridIndex> path);
     ```

   - оставить существующую публичную функцию `pathMinimumProhibitedClearanceM(const OccupancyGrid2D&, ...)` как wrapper для обратной совместимости тестов/call sites;
   - в `PlannerCore::computePath()` построить `ClearanceField2D` один раз для `ClearanceSource::kProhibited` и `normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m)`, затем посчитать raw и smoothed clearance от этого поля;
   - не менять значения `raw_path_clearance_m` и `smoothed_path_clearance_m`;
   - сохранить старые timing поля `raw_path_clearance_duration_ms` и `smoothed_path_clearance_duration_ms`, но добавить отдельное поле `clearance_field_build_duration_ms` или `prohibited_clearance_field_duration_ms`, чтобы в headless логах было видно, что теперь дорогое построение поля одно;
   - обновить `planner_node_inputs.cpp:415` log format так, чтобы старые timing значения не исчезли, а новое поле было видно отдельно.

   Тесты:

   - добавить/обновить unit test в `planner_core_test.cpp`, который сравнивает результат wrapper-функции от grid и helper-логики через `computePath()`;
   - проверить, что `computePath()` по-прежнему возвращает те же raw/smoothed clearance значения на grid с occupied+inflated cells;
   - не писать тест, который зависит от абсолютного времени.

2. Убрать лишние allocations в `racing_line` без изменения порядка вычислений.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:102`
   - `drone_city_nav/src/racing_line.cpp:127`
   - `drone_city_nav/src/racing_line.cpp:388`
   - `drone_city_nav/src/racing_line.cpp:525`
   - `drone_city_nav/src/racing_line.cpp:611`
   - `drone_city_nav/tests/racing_line_test.cpp:116`

   Материализуемый результат:

   - в anonymous namespace добавить scratch/workspace struct только внутри `racing_line.cpp`:

     ```cpp
     struct RacingLineScratch {
       std::vector<double> candidate_offsets;
       std::vector<double> accepted_offsets;
       std::vector<double> smoothed_offsets;
       std::vector<Point2> candidate_points;
       std::vector<Point2> accepted_points;
       std::vector<TrajectoryPointSample> candidate_samples;
     };
     ```

   - заменить helpers, которые возвращают временные vectors, на fill-style overloads:

     ```cpp
     void pointsFromOffsets(..., std::vector<Point2>& out);
     void samplesFromPointsAndOffsets(..., std::vector<TrajectoryPointSample>& out);
     void smoothedOffsets(..., std::vector<double>& out);
     ```

   - в `optimizeRacingLine()` заранее `reserve(sample_count)` для scratch buffers;
   - в candidate loop не создавать `best_offsets`, `candidate_offsets`, `candidate_points`, `accepted_offsets`, `accepted_points` заново на каждом candidate;
   - важно: сохранить текущий порядок обхода `iteration -> i -> delta {-step, step}` и текущий tie-break `candidate_score.score + 1.0e-9 < best_cost`;
   - не менять formulas, weights, candidate generation, order of accepted candidate updates.

   Тесты:

   - усилить `RacingLine.ResultIsDeterministic`: сравнивать не только точки, но и `candidate_evaluations`, `collision_rejections`, `final_cost`, `final_length_m`, `estimated_time_s`;
   - оставить/прогнать существующие tests `WideCornerProducesTraversableSmoothLine`, `ProhibitedCenterlineCanUseLateralCorridorSeed`, `ReportsTimeFirstCostBreakdownAndEdgeMargins`;
   - добавить negative-path test: blocked centerline без lateral room остаётся invalid после buffer reuse.

3. Добавить explicit diagnostics для allocation/cache оптимизаций, не влияющие на маршрут.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp:36`
   - `drone_city_nav/src/racing_line.cpp:525`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp:250`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp:514`
   - `drone_city_nav/src/planner_node_publish.cpp:198`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`

   Материализуемый результат:

   - сохранить текущие поля `racing_candidate_path_evaluation_duration_ms` и `racing_candidate_score_duration_ms`;
   - добавить, если понадобится для headless отладки, не влияющие на поведение counters/timings:
     - `racing_candidate_point_build_duration_ms`;
     - `racing_candidate_sample_build_duration_ms`;
     - `racing_regularization_duration_ms`;
     - `racing_scratch_reused_candidates` или аналогичный счётчик, если он реально полезен;
   - добавить эти поля в JSON diagnostics и parser;
   - добавить проверки в `trajectory_diagnostics_io_test.cpp`, что новые поля пишутся и читаются.

   Ограничение: не добавлять шумные per-candidate logs в ROS stdout. Подробности должны идти в aggregate stats/JSON, иначе headless logs станут слишком тяжёлыми.

4. Спланировать и реализовать строгий cache key для `ClearanceField2D` только после того, как станет доступна полная сигнатура prohibited state.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp:40`
   - `drone_city_nav/src/occupancy_grid.cpp:124`
   - `drone_city_nav/include/drone_city_nav/clearance_field.hpp:15`
   - `drone_city_nav/src/clearance_field.cpp:67`
   - `drone_city_nav/tests/clearance_field_test.cpp`

   Материализуемый результат:

   - добавить read-only доступ к inflation mask или dedicated prohibited fingerprint API. Сейчас `cells()` не включает `inflated_`, поэтому строгий prohibited cache key невозможен без расширения API;
   - предпочтительный контракт:

     ```cpp
     struct OccupancyGridFingerprint {
       GridBounds bounds;
       std::uint64_t cells_hash;
       std::uint64_t inflated_hash;
     };
     OccupancyGridFingerprint prohibitedFingerprint() const noexcept;
     ```

   - `ClearanceFieldCacheKey` должен включать:
     - bounds/resolution/origin/size;
     - `max_distance_m` после normalization;
     - `ClearanceSource`;
     - hash/fingerprint occupied+inflated состояния;
   - `ClearanceFieldCache` должен возвращать cached field только при полном совпадении key;
   - любые сомнения в key должны приводить к rebuild, а не к reuse.

   Тесты:

   - cache hit: одинаковый grid + radius + source возвращает reuse;
   - cache miss: изменение occupied cell invalidates cache;
   - cache miss: `rebuildInflation()` invalidates prohibited cache даже если occupied cells не изменились;
   - cache miss: другой radius/source invalidates cache;
   - тесты не должны проверять время, только корректность hit/miss и clearance values.

   Практическое применение:

   - после появления строгого key можно использовать cache в `PlannerNode`/planning pipeline для повторных stable-path checks и trajectory builds;
   - не вводить global/static cache в core functions: это создаст скрытое состояние и усложнит thread-safety.

5. Рассмотреть параллельную racing-line candidate evaluation как отдельный, более рискованный этап.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:617`
   - `drone_city_nav/src/racing_line.cpp:630`
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp:15`
   - `drone_city_nav/src/planner_node_config.cpp:183`
   - `drone_city_nav/config/urban_mvp.yaml:125`
   - `drone_city_nav/tests/racing_line_test.cpp`

   Материализуемый результат:

   - добавить параметр, выключенный по умолчанию на первом этапе:

     ```yaml
     racing_line_parallel_candidate_evaluation: false
     racing_line_parallel_workers: 0
     ```

   - безопасный scope параллелизма:
     - initial seed candidates независимы;
     - для одного `i` в текущем алгоритме candidates `{-step, +step}` независимы друг от друга, если оба считаются от одного snapshot `offsets`;
   - не параллелить разные `i` в текущем loop, потому что `offsets` обновляется после каждого index, и это изменит алгоритм;
   - каждый worker пишет результат в отдельный slot:

     ```cpp
     struct EvaluatedCandidate {
       std::size_t order_index;
       bool noop;
       CandidateScore score;
       PathEvaluation path;
       std::vector<double> offsets;
       std::vector<Point2> points;
     };
     ```

   - после join выбор лучшего делать строго последовательным проходом по `order_index`, тем же tie-break, что сейчас: `score + 1.0e-9 < best_cost`;
   - не суммировать floating-point cost из разных потоков; каждый candidate считает свой total целиком.

   Тесты:

   - parallel off и parallel on на одной fixture дают одинаковые samples, `final_cost`, `candidate_evaluations`, `collision_rejections`;
   - tie-break fixture: два одинаковых по score candidates выбираются в старом order;
   - repeated runs deterministic.

   Риск: даже при аккуратной реализации возможны микроотличия из-за неявного изменения порядка обновления stats/timings или из-за случайного shared state. Поэтому этот пункт должен идти после allocation cleanup и только при достаточной тестовой страховке.

6. Не менять алгоритмические параметры в рамках этой пачки оптимизаций.

   Файлы:

   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/src/corridor.cpp`
   - `drone_city_nav/src/trajectory_speed_planner.cpp`

   Материализуемый результат:

   - не менять `astar_heuristic_weight`, `corridor_sample_step_m`, `racing_line_optimizer_sample_step_m`, weights racing-line optimizer, speed profile, turn smoothing windows;
   - если в ходе реализации появится соблазн ускорить через уменьшение sample count или weighted A*, вынести это в отдельный behavioral tuning commit/plan.

# Verification plan

Минимальные targeted checks после реализации:

1. Форматирование изменённых C++ файлов:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Targeted unit tests после PlannerCore clearance changes:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R "planner_core_test|clearance_field_test"
   ```

3. Targeted unit tests после racing-line allocation changes:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R "racing_line_test|trajectory_planner_test|trajectory_diagnostics_io_test"
   ```

4. Repository quality gate перед commit:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

5. Если меняется только `PLAN.md`, достаточно:

   ```bash
   git diff --check
   ```

   и можно не запускать simulation. Но после реальной C++ реализации нужны targeted tests и `make quality`.

Симуляции `./scripts/sim_gui.sh` / `./scripts/sim_headless.sh` в рамках этого плана не запускать без отдельной прямой команды пользователя.

# Testing strategy

Категория 1: без рефакторинга / низкий риск.

- PlannerCore clearance reuse.
- Автотесты:
  - `planner_core_test`: raw/smoothed clearance values не меняются;
  - `clearance_field_test`: field values прежние;
  - diagnostics: новые timing поля пишутся без удаления старых.

Категория 2: лёгкий рефакторинг / средний риск.

- Racing-line scratch buffers и fill-style helpers.
- Автотесты:
  - deterministic repeated output;
  - existing positive/negative racing-line cases;
  - JSON diagnostics round-trip, если добавлены новые stats.

Категория 3: тяжёлый / повышенный риск.

- Strict `ClearanceField2D` cache с grid fingerprint.
- Parallel candidate evaluation.
- Автотесты:
  - cache hit/miss correctness;
  - parallel on/off equivalence;
  - deterministic tie-break;
  - repeated runs.

# Risks and tradeoffs

- Поведение траектории: allocation cleanup не должен менять порядок candidate evaluation. Любое изменение порядка acceptance может изменить racing line.
- Diagnostics timings: после PlannerCore clearance reuse старые поля могут стать несопоставимыми с историческими логами, если не добавить отдельный build timing. Нужно явно сохранить старые поля и добавить новое.
- Cache correctness: неполный key для `ClearanceField2D` может дать stale clearance field и сломать безопасность маршрута. Поэтому cache только со строгим fingerprint, включая inflated/prohibited state.
- Performance/resources: fingerprint over full grid тоже стоит CPU. Его надо мерить; cache имеет смысл только если hash дешевле rebuild или если grid identity/generation можно использовать строго.
- Threading: parallel candidate evaluation не должен писать shared stats из worker threads напрямую. Иначе будут races и недетерминизм.
- API/контракты: добавление read-only fingerprint/inflated access к `OccupancyGrid2D` расширяет public API core library; нужно держать его const-only.
- Интеграции: diagnostics JSON/blackbox consumers должны пережить добавление новых полей. Удалять старые keys нельзя.

Что могло сломаться после реализации:

- route/trajectory geometry может измениться, если случайно изменён tie-break или порядок candidate update;
- `trajectory_diagnostics` parser может перестать читать старые логи, если новые поля сделаны обязательными;
- cache может вернуть field от старой карты, если key неполный;
- parallel mode может давать nondeterministic output при shared mutable stats;
- `make quality` может выявить include/order issues после добавления новых headers.

Проверка рисков:

- сравнить targeted unit tests до/после;
- для реальной C++ реализации проверить `racing_line_test` deterministic output;
- по headless/GUI логам после отдельного пользовательского разрешения сравнить `trajectory_points`, `final_length_m`, `final_cost`, `candidate_evaluations`, `time_final`, `candidate_path_evaluation_duration_ms`, `candidate_score_duration_ms`.

# Open questions

1. Нужен ли cache `ClearanceField2D` уже в первой реализации или сначала достаточно убрать двойное построение в `PlannerCore` и allocations в `racing_line`?
2. Допустимо ли расширить `OccupancyGrid2D` read-only API для strict fingerprint/inflated mask, или лучше держать cache только на уровне одного planning call без межвызовного reuse?
3. Нужно ли включать parallel racing-line evaluation по умолчанию после тестов, или оставить параметр выключенным и включать только для измерений?
4. Какая целевая latency для full path+trajectory rebuild: `<500 ms`, `<1000 ms`, или достаточно убрать самые очевидные `~1.1 s` diagnostic clearance и часть `~1.3 s` racing-line optimizer?
