# План: Passage Traversal Mode / Sensor Policy

## Context

Нужно спланировать отдельный этап фичи 3D passage traversal: политика сенсоров и реплана во время пролета через заранее известный passage/opening. Распознавание passage сенсорами в этот этап не входит: карта passage уже известна через `worlds/known_passages.passages3d` и загружается в planner.

Целевое поведение:

1. Если дрон находится внутри активного известного passage segment, ожидаемые lidar/memory возвраты от стен вокруг opening не должны запускать обычный full replan.
2. Если obstacle попал внутрь opening corridor, это остается emergency blocker и не подавляется.
3. В логах должны быть поля:
   - `passage_traversal_active`
   - `lidar_policy`
   - `ignored_expected_obstacle_count`
   - `emergency_blocker_count`
4. Изменение высокорисковое, потому что касается safety/replan logic, поэтому его нужно делать отдельным этапом с прямыми unit-тестами и headless-диагностикой.

Текущий код уже имеет базу для этого этапа:

- known passage map: `drone_city_nav/include/drone_city_nav/known_passage_map.hpp`;
- matching/validation: `drone_city_nav/include/drone_city_nav/known_passage_matching.hpp`, `drone_city_nav/include/drone_city_nav/known_passage_validation.hpp`;
- vertical profile через passage: `drone_city_nav/include/drone_city_nav/trajectory_vertical_profile.hpp`;
- trajectory samples уже содержат `z_m` и `vertical_profile_passage_id`: `drone_city_nav/include/drone_city_nav/trajectory.hpp`;
- runtime path safety сейчас проверяется на prohibited grid и при пересечении запускает A*: `drone_city_nav/src/planner_node_runtime.cpp:151`;
- current lidar overlay сейчас просто добавляет hits в raw dynamic grid: `drone_city_nav/src/current_lidar_overlay.cpp`.

Главный пробел: planner пока хранит только `last_valid_path_points_` как 2D points (`drone_city_nav/src/planner_node.hpp:384`), но не хранит последний executable trajectory artifact с `TrajectoryPointSample`, `z_m`, `s_m` и `vertical_profile_passage_id`. Для passage-aware sensor policy нужен именно последний executable trajectory, а не только плоская линия.

## Investigation context

`INVESTIGATION.md` в репозитории отсутствует. Предыдущего investigation artifact для этой задачи нет.

Проверенные документы и ограничения:

- `.agent-io/inbox.txt`: workflow `plan`, требуется только план и запись результата в outbox.
- `AGENTS.md`: соблюдать `CPP_BEST_PRACTICES.md`, работать через repo-approved workflow, перед commit после изменений запустить quality, не коммитить `.agent-io`.
- `README.md`, `CONTRIBUTING.md`, `Makefile`: проект является ROS 2 workspace, сборка/тесты через container scripts/make targets.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`: Notion не требуется, потому что задача не ссылается на Notion task.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`: GitLab/MR не требуется, потому что задача не просит MR/remote.

Remote HTTP/SSH не использовался.

## Detected stack/profiles

- Stack: C++20, ROS 2, `ament_cmake`, `rclcpp`, PX4 SITL/Gazebo/RViz.
- Workspace: ROS 2 colcon workspace.
- Build/test workflow: container-only через scripts/Makefile.
- Applied profiles:
  - `generic`
  - `cpp`

## Repo-approved commands found

Основные команды из README/CONTRIBUTING/Makefile:

1. `./scripts/build.sh`
2. `./scripts/test.sh`
3. `./scripts/sim_headless.sh`
4. `./scripts/sim_gui.sh`
5. `./scripts/dev_shell.sh`
6. Внутри container shell:
   - `make build`
   - `make test`
   - `make test-scripts`
   - `make quality`
   - `make format`
   - `make sim-headless`
   - `make sim-gui`

Для этого plan-only шага C++-файлы не меняются. После записи `PLAN.md` нужно запустить `./scripts/dev_shell.sh make quality` перед commit.

## Affected components

1. Planner runtime safety/reuse
   - `drone_city_nav/src/planner_node_runtime.cpp`
   - `PlannerNode::keepCurrentPathIfStillClear()`
   - Сейчас `kProhibitedConfirmed` всегда ведет к `running A* from current pose`.

2. Planner grid input build
   - `drone_city_nav/src/planner_node_inputs.cpp`
   - `PlannerNode::buildPlanningGrid()`
   - Сейчас current lidar и memory попадают в dynamic raw sources до prohibited inflation без passage-aware policy.

3. Current lidar / memory source grids
   - `drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp`
   - `drone_city_nav/src/current_lidar_overlay.cpp`
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
   - `drone_city_nav/src/planning_grid_builder.cpp`
   - Важно: policy должна фильтровать рабочие копии dynamic sources, а не разрушать permanent memory.

4. Known passage geometry/matching
   - `drone_city_nav/include/drone_city_nav/known_passage_map.hpp`
   - `drone_city_nav/include/drone_city_nav/known_passage_matching.hpp`
   - `drone_city_nav/src/known_passage_matching.cpp`
   - Нужно переиспользовать локальную систему координат opening/structure и не дублировать математику в planner.

5. Executable trajectory state inside planner
   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`
   - Сейчас после успешной публикации сохраняется только `last_valid_path_points_`; нужно хранить `last_valid_trajectory_samples_` или компактный artifact для active passage policy.

6. Planner config
   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - Нужно добавить явные параметры passage traversal sensor policy.

7. Diagnostics/logging
   - `drone_city_nav/src/planner_node_inputs.cpp`
   - `drone_city_nav/src/planner_node_runtime.cpp`
   - опционально `TrajectoryPlannerStats` / trajectory diagnostics JSON, если нужно видеть policy в merged blackbox.

8. Tests/contracts
   - `drone_city_nav/tests/*`
   - `drone_city_nav/CMakeLists.txt`
   - `scripts/tests/test_topic_contract.py`
   - возможно `scripts/tests/test_offboard_telemetry_contract.py`, если поля попадут в offboard/blackbox contract.

## Implementation steps

1. Добавить чистый модуль passage traversal sensor policy.

   Новые файлы:

   - `drone_city_nav/include/drone_city_nav/passage_traversal_sensor_policy.hpp`
   - `drone_city_nav/src/passage_traversal_sensor_policy.cpp`

   Основные типы:

   ```cpp
   enum class PassageLidarPolicy {
     kNormal,
     kIgnoreExpectedWalls,
     kEmergencyBlocker,
   };

   struct PassageTraversalSensorPolicyConfig {
     bool enabled{true};
     double activation_margin_m{3.0};
     double opening_corridor_lateral_margin_m{0.75};
     double opening_corridor_depth_margin_m{1.0};
     double expected_wall_margin_m{0.5};
     std::size_t max_active_passages{2U};
     std::size_t max_diagnostics{8U};
   };

   struct ActivePassageTraversal {
     bool active{false};
     std::string structure_id;
     std::string opening_id;
     double entry_s_m{0.0};
     double exit_s_m{0.0};
     double activation_start_s_m{0.0};
     double activation_end_s_m{0.0};
     PassageOpening opening{};
   };

   struct PassageTraversalSensorPolicyStats {
     bool passage_traversal_active{false};
     PassageLidarPolicy lidar_policy{PassageLidarPolicy::kNormal};
     std::size_t ignored_expected_obstacle_count{0U};
     std::size_t emergency_blocker_count{0U};
     std::size_t current_lidar_cells_checked{0U};
     std::size_t memory_cells_checked{0U};
     std::size_t current_lidar_expected_wall_cells{0U};
     std::size_t memory_expected_wall_cells{0U};
     std::string active_structure_id;
     std::string active_opening_id;
     double active_s_m{std::numeric_limits<double>::quiet_NaN()};
   };
   ```

   Материализованный результат:

   - чистая функция определяет active passage по current projection `s_m` и известным passage matches;
   - чистая функция классифицирует occupied raw cell как:
     - `normal_obstacle`;
     - `expected_passage_wall`;
     - `opening_corridor_blocker`;
   - чистая функция применяет policy к копиям current lidar/memory grids и возвращает stats.

2. Сохранить последний executable trajectory artifact в planner.

   Файлы:

   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`
   - `drone_city_nav/src/planner_node_debug_publication.cpp`

   Изменение:

   - добавить `std::vector<TrajectoryPointSample> last_valid_trajectory_samples_;`
   - при успешном `publishTrajectoryResult()` сохранять `trajectory_result.samples` рядом с `last_valid_path_points_`;
   - при публикации empty/hold path чистить оба состояния;
   - при refined update делать то же самое, потому что refined path тоже становится executable trajectory.

   Почему это нужно:

   - active passage нужно считать по `s_m`, `z_m`, `vertical_profile_passage_id`, а не по плоским XY points;
   - это уменьшает риск случайно фильтровать lidar вне фактического 3D passage traversal.

3. Добавить вычисление active passage state.

   Файлы:

   - `passage_traversal_sensor_policy.*`
   - `drone_city_nav/src/planner_node_inputs.cpp`

   Логика:

   - проектировать текущую позицию на `last_valid_trajectory_samples_`;
   - получить current `s_m`;
   - использовать `findKnownPassageTraversalMatches()` по последней trajectory;
   - active, если current `s_m` попал в `[entry_s_m - activation_margin_m, exit_s_m + activation_margin_m]`;
   - active только для валидного `KnownPassageTraversalMatch` с matched opening;
   - если known passage map отсутствует, trajectory samples пустые или match невалидный, policy остается `normal`.

   Материализованный результат:

   - `passage_traversal_active=false` в обычном полете;
   - `passage_traversal_active=true` только в коротком участке фактического passage traversal.

4. Применить policy к dynamic source grids до inflation.

   Файлы:

   - `drone_city_nav/src/planner_node_inputs.cpp:265`
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
   - `drone_city_nav/src/planning_grid_builder.cpp`

   Точка интеграции:

   - после `overlayCurrentLidarHits(*current_lidar_grid, now_ns)`;
   - до `planning_grid_builder_.build(config, sources)`.

   Предлагаемая схема:

   ```text
   memory_grid_                     -> immutable source
   current_lidar_grid               -> raw overlay for this tick
   applyPassageTraversalSensorPolicy
     -> filtered_memory_grid_copy
     -> filtered_current_lidar_grid_copy
     -> PassageTraversalSensorPolicyStats
   PlanningGridSources uses filtered copies
   ```

   Правила безопасности:

   - static grid никогда не фильтровать;
   - permanent memory grid не мутировать;
   - фильтровать только expected walls внутри active known passage structure footprint и вне opening corridor;
   - cells inside opening corridor не фильтровать, считать `emergency_blocker_count` и оставлять occupied;
   - outside-grid и unknown geometry не подавлять.

   Материализованный результат:

   - known wall returns вокруг passage не раздуваются в prohibited obstacle;
   - реальный blocker в opening corridor остается prohibited и вызывает существующую safety/replan реакцию.

5. Уточнить runtime path reuse при `kProhibitedConfirmed`.

   Файл:

   - `drone_city_nav/src/planner_node_runtime.cpp:151`

   Сейчас:

   - `StablePathDecisionReason::kProhibitedConfirmed` всегда increment `prohibited_replans_` и full A*.

   Нужно:

   - добавить чистую decision-функцию, например:

     ```cpp
     enum class PassageAwareReuseAction {
       kKeepCurrentPath,
       kRunAStar,
       kEmergencyBlocker,
     };
     ```

   - если intersection source классифицирован как expected passage wall и `passage_traversal_active=true`, вернуть `kKeepCurrentPath`;
   - если `emergency_blocker_count > 0`, оставить `kRunAStar`/существующее safety поведение и логировать emergency;
   - если policy inactive, поведение должно остаться byte-for-byte логически прежним.

   Материализованный результат:

   - expected wall больше не запускает normal replan;
   - emergency blocker не подавляется.

6. Добавить config params.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/tests/planner_node_config_test.cpp`

   Предлагаемые YAML keys:

   ```yaml
   passage_traversal_sensor_policy_enabled: true
   passage_traversal_activation_margin_m: 3.0
   passage_traversal_opening_corridor_lateral_margin_m: 0.75
   passage_traversal_opening_corridor_depth_margin_m: 1.0
   passage_traversal_expected_wall_margin_m: 0.5
   passage_traversal_max_active_passages: 2
   passage_traversal_max_diagnostics: 8
   ```

   Все значения clamp-ить в `loadPlannerNodeConfig()`.

7. Добавить обязательные planner logs.

   Файлы:

   - `drone_city_nav/src/planner_node_inputs.cpp`
   - `drone_city_nav/src/planner_node_runtime.cpp`

   В `Planning summary` добавить блок:

   ```text
   passage_sensor_policy[
     passage_traversal_active=%s
     lidar_policy=%s
     ignored_expected_obstacle_count=%zu
     emergency_blocker_count=%zu
     structure=%s
     opening=%s
     active_s=%.2f
     current_lidar_checked=%zu
     memory_checked=%zu
   ]
   ```

   В `Current path intersects newly available prohibited obstacle data` добавить те же 4 обязательных поля и классификацию blocker-а.

   Материализованный результат:

   - headless log сразу показывает, почему obstacle был проигнорирован или почему стал emergency;
   - обязательные поля из inbox присутствуют в plain logs.

8. Добавить diagnostics artifact для offline/headless анализа.

   Ближайший безопасный вариант:

   - добавить `PassageTraversalSensorPolicyStats` в `PlanningGridBuildResult`;
   - логировать stats в planner summary и в prohibited-intersection warning.

   Расширенный вариант, если нужен blackbox merge:

   - добавить planning-only stats в `TrajectoryPlannerStats`;
   - сериализовать в `trajectory_diagnostics_io_json_summary.cpp`;
   - парсить в `trajectory_diagnostics_io_parser.cpp`;
   - мержить в `offboard_trajectory_state.cpp`.

   Для первого implementation pass достаточно `PlanningGridBuildResult + planner logs`, потому что sensor policy живет в planner grid build/replan layer, а не в offboard follower.

9. Добавить unit tests для policy module.

   Новый тест:

   - `drone_city_nav/tests/passage_traversal_sensor_policy_test.cpp`

   Кейсы:

   1. inactive policy does not modify current lidar/memory grids;
   2. no known passage map -> normal policy;
   3. active passage, occupied cell in expected wall area -> cleared in filtered grid, `ignored_expected_obstacle_count=1`;
   4. active passage, occupied cell inside opening corridor -> remains occupied, `emergency_blocker_count=1`, `lidar_policy=emergency_blocker`;
   5. static obstacle is never filtered;
   6. permanent memory source is not mutated, only filtered copy changes;
   7. overlapping/adjacent passage spans choose the active span closest to current `s_m`;
   8. disabled config leaves all source grids unchanged.

10. Добавить tests для runtime decision.

    Подход:

    - вынести маленькую pure-функцию из `keepCurrentPathIfStillClear()` для passage-aware decision;
    - покрыть ее в `planner_runtime_state_test.cpp` или новом `passage_traversal_runtime_decision_test.cpp`.

    Кейсы:

    1. `kProhibitedConfirmed + normal policy -> kRunAStar`;
    2. `kProhibitedConfirmed + active expected wall -> kKeepCurrentPath`;
    3. `kProhibitedConfirmed + emergency blocker -> kRunAStar`;
    4. non-prohibited stable-path rejection reasons не меняются;
    5. stats counters do not increment `prohibited_replans_` for expected wall suppression.

11. Обновить CMake/test registration.

    Файл:

    - `drone_city_nav/CMakeLists.txt`

    Добавить:

    - core source `src/passage_traversal_sensor_policy.cpp`;
    - gtest target `passage_traversal_sensor_policy_test`;
    - при необходимости target для runtime decision.

12. Обновить script-level diagnostics contracts.

    Файлы:

    - `scripts/tests/test_topic_contract.py`
    - возможно `scripts/tests/test_offboard_telemetry_contract.py`

    Что проверить:

    - если добавлен новый topic, внести его в topic contract;
    - если не добавлен topic, script contract не трогать;
    - если fields попадут в blackbox/trajectory diagnostics, добавить contract на:
      - `passage_traversal_active`
      - `lidar_policy`
      - `ignored_expected_obstacle_count`
      - `emergency_blocker_count`

13. Обновить документацию/config comments.

    Файлы:

    - `docs/navigation_pipeline.md`
    - `docs/replanning.md`
    - `docs/obstacle_mapping.md`
    - `docs/configuration.md`
    - `docs/diagnostics.md`

    Материализованный результат:

    - явно описано, что static map remains authoritative;
    - expected passage walls фильтруются только во время active traversal;
    - opening corridor blocker остается emergency;
    - 2D lidar не становится passage detector-ом, он только проходит через policy layer.

## Verification plan

1. Статическая проверка после реализации:

   ```bash
   ./scripts/dev_shell.sh make format
   ./scripts/dev_shell.sh make quality
   ```

2. Unit tests:

   ```bash
   ./scripts/dev_shell.sh make test
   ```

   Обязательно должны проходить новые тесты:

   - `passage_traversal_sensor_policy_test`
   - runtime decision tests
   - `planner_node_config_test`
   - existing known passage tests
   - existing trajectory vertical profile tests

3. Script contracts:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

4. Headless scenario:

   ```bash
   ./scripts/sim_headless.sh
   ```

   Проверить в logs:

   - до active passage: `passage_traversal_active=false`, `lidar_policy=normal`;
   - внутри active passage со стенами вокруг opening:
     - `passage_traversal_active=true`;
     - `lidar_policy=ignore_expected_walls`;
     - `ignored_expected_obstacle_count>0`;
     - normal `prohibited_replans_` не растет из-за expected wall;
   - если в opening corridor поставить blocker:
     - `emergency_blocker_count>0`;
     - obstacle не фильтруется;
     - path safety/replan behavior остается active.

5. RViz/debug verification:

   - visible trajectory still goes through known passage;
   - prohibited grid no longer paints expected passage wall returns as reason for immediate replan during active traversal;
   - blocker inside opening corridor remains visible as obstacle.

## Testing strategy

### Category 1: no refactor / pure additions

Подходит для:

- новый `passage_traversal_sensor_policy` как pure module;
- config loading tests;
- JSON/log field contract tests, если поля добавляются без изменения control behavior.

Тесты:

- deterministic gtest на маленьких synthetic grids;
- no ROS node spin required;
- проверять exact counters and filtered cells.

### Category 2: light refactor

Подходит для:

- сохранение `last_valid_trajectory_samples_`;
- вынос passage-aware reuse decision из `keepCurrentPathIfStillClear()` в pure helper;
- добавление stats в `PlanningGridBuildResult`.

Тесты:

- unit tests на helper decision;
- existing planner runtime tests;
- regression: invalid/no trajectory samples -> policy normal;
- regression: empty hold path clears executable artifact.

### Category 3: heavy/high-risk behavior

Подходит для:

- фактическое подавление expected wall replan;
- emergency blocker preservation;
- взаимодействие с memory/current_lidar/static grids;
- future full blackbox/trajectory diagnostics merge.

Тесты:

- headless simulation with known passage;
- forced synthetic blocker inside opening corridor;
- repeated run comparison:
  - replan count;
  - path publications;
  - obstacle source diagnostics;
  - passage logs;
  - final trajectory validity.

Важно: category 3 включать только после зеленых category 1/2 tests.

## Risks and tradeoffs

1. Safety false negative.

   Главный риск: принять реальный obstacle за expected wall и проигнорировать его. Поэтому policy должна:

   - работать только при `passage_traversal_active=true`;
   - фильтровать только dynamic sources;
   - не фильтровать opening corridor blockers;
   - не фильтровать static map;
   - иметь явные counters/logs.

2. 2D lidar не знает высоту obstacle.

   Current lidar overlay является 2D projection. Он не может доказать, что hit находится выше/ниже opening. Поэтому первая версия policy должна быть консервативной:

   - outside opening corridor wall area можно игнорировать как expected wall;
   - inside opening corridor нельзя игнорировать, потому что это может быть obstacle в проходе.

3. Memory может содержать stale wall hits.

   Если фильтровать memory, нельзя стирать permanent memory. Нужно фильтровать только copy, которая идет в `PlanningGridSources`.

4. Active span mismatch.

   Если current projection `s_m` вычислен по старой trajectory или после path update, policy может активироваться не там. Поэтому нужен `last_valid_trajectory_samples_`, path id/generation и fallback to normal policy при любой неуверенности.

5. Planner performance.

   Сканирование occupied cells current_lidar/memory может быть дорогим. Начать с occupied-cell iteration по raw source grids и ограничить classification active passage bounding box; логировать checked cells.

6. Static map contract.

   Статическая 2D карта специально не содержит здания с passage как blocking rectangles. Если в будущем static map начнет содержать 3D passage structures как 2D obstacles, этот policy layer не должен тайно удалять static blocking. Это отдельный архитектурный шаг.

7. Emergency behavior.

   В первом шаге emergency blocker должен использовать существующее prohibited/replan поведение. Отдельный hard-stop/hold mode можно делать позже, если headless logs покажут, что обычный replan недостаточно быстрый.

## Open questions

1. Фильтровать ли memory вместе с current lidar в первой реализации?

   Моя рекомендация: да, но только filtered copy и только во время active passage. Иначе expected wall hits, уже попавшие в memory, продолжат ломать traversal даже после фильтрации current lidar.

2. Нужен ли отдельный emergency stop при `emergency_blocker_count>0`?

   Моя рекомендация для первого шага: не вводить новый stop mode, оставить existing prohibited/replan behavior и добавить явные logs. Если headless покажет, что этого мало, сделать отдельный emergency gate-blocker response следующим этапом.

3. Нужно ли публиковать отдельный ROS topic с policy stats?

   Моя рекомендация: сначала нет. Достаточно planner logs и stats в `PlanningGridBuildResult`. Topic добавить позже, если RViz/blackbox debugging будет неудобен.

4. Нужно ли использовать `vertical_profile_passage_id` как единственный active criterion?

   Моя рекомендация: нет. Использовать его как сильный сигнал, но финально проверять `KnownPassageTraversalMatch` и current `s_m`, потому что trajectory может пройти через passage geometry даже если id не заполнен из-за будущих изменений vertical profile.

5. Как назвать пользовательский термин?

   В коде и документации использовать `known passage`, `opening`, `passage traversal`, `opening corridor`. Не использовать `hole` как основной термин.
