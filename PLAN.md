# План: единая penalty-модель от raw occupancy без planner/prohibited grid

## Context

Задача выбирает вариант 2: только `raw occupied` остается абсолютным 2D-запретом,
а прежние зоны inflation становятся мягкими уровнями риска:

- `raw occupied` и выход за известные bounds/локальный ROI: hard reject;
- расстояние меньше прежнего `inflation_radius_m` (`1.0 м`): critical risk;
- расстояние меньше `inflation_radius_m + planning_clearance_m`: high risk;
- остальное пространство: preferred.

Для static режима граница preferred остается `1.0 + 3.0 = 4.0 м`, для no-static
режима `1.0 + 5.0 = 6.0 м`. Эти значения больше не материализуются как
`prohibited_grid` и `planning_grid`; они вычисляются по единственному distance
field от объединенного raw occupancy.

Требование распространяется на все режимы и все grid-зависимые стадии:
полный/частичный A*, no-static rollout, route smoothing, corridor, optimizer,
turn smoothing, passage insertion, stitching и финальную validation. Выбор
кандидата должен быть лексикографическим, а не через большие scalar penalties.

Под “полным отказом от grid” далее понимается отказ от двух производных
inflated grid. Raw occupancy остается необходимым дискретным представлением
статической карты, obstacle memory и lidar overlay. Удалить и его нельзя без
замены всех источников препятствий на другой spatial index, чего задача не
требует и что противоречило бы условию `raw occupied -> hard reject`.

`safe truncation` сохраняется. Значения `15.0 м` longitudinal margin и `5.0 м`
terminal clearance не меняются. При этом runtime blocker обязан означать новое
пересечение принятой траектории с `raw occupied`: если оставить trigger по
бывшей critical band, он будет немедленно обрезать маршрут, который новая
penalty-модель намеренно разрешила.

Объем изменения: **XL**. По текущему поиску старые grid/escape-контракты
затрагивают около 90 исходников, тестов, конфигов и RViz/docs-артефактов. Это
обычная, но широкая архитектурная миграция; объективного blocker в workspace
нет. Реализацию следует вести последовательными локальными коммитами, но
production cutover делать только после перевода всех shape-changing стадий на
единый контракт, чтобы не получить смешение hard-grid и soft-risk семантики.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на прямом чтении
текущего `main` (`HEAD=1eb5863`) и актуальных call sites.

Ключевые подтвержденные факты:

- `PlanningGridBuilder` сейчас объединяет raw sources, дважды строит/использует
  occupied distance field и материализует `prohibited_grid` и `planning_grid`
  (`drone_city_nav/src/planning_grid_builder.cpp:183-254`).
- В static и no-static конфигурациях используются соответственно
  `planning_clearance_m=3.0` и `no_static_planning_clearance_m=5.0` при
  `inflation_radius_m=1.0`
  (`drone_city_nav/config/urban_mvp.yaml:155-157`,
  `drone_city_nav/src/planner_node_config.cpp:55-60,828-831`).
- A* отвергает prohibited start/goal и соседей бинарно
  (`drone_city_nav/src/astar_planner.cpp:197-212,256-285`).
- Rollout также бинарно отвергает каждый segment, пересекающий текущий planning
  grid, и только затем ранжирует оставшиеся scalar score
  (`drone_city_nav/src/receding_horizon_trajectory_planner.cpp:24-63,199-225`).
- Orchestration уже использует vector/loop abstraction, но фактически передает
  один `planning_clearance` candidate
  (`drone_city_nav/src/planner_node_inputs.cpp:877-878`). Старые per-grid loops
  остаются внутри `trajectory_planner.cpp` и route publication.
- Snapshot содержит обе grid-копии, две clearance fields, unrelaxed grid и
  directed escape state
  (`drone_city_nav/include/drone_city_nav/planning_grid_snapshot.hpp:12-42`,
  `drone_city_nav/src/planning_grid_snapshot.cpp:23-105`).
- Directed escape и radial relaxation являются чистым следствием hard
  inflation: отдельные planner/debug modules, episode state, parameters,
  unrelaxed-tail gate и RViz markers. После soft-risk cutover они не нужны.
- Runtime rollout/global-path checks ищут первый `isProhibited()` span
  (`drone_city_nav/src/trajectory_repair.cpp:181-200`,
  `drone_city_nav/src/planner_node_runtime.cpp:242-372`).
- Offboard получает `/drone_city_nav/prohibited_grid`, преобразует все значения
  от `kInflatedOccupancyValue` в occupied и использует результат для terminal
  station safe truncation
  (`drone_city_nav/src/px4_offboard_node_control_state.cpp:58-117`,
  `drone_city_nav/src/px4_offboard_node_replan.cpp:157-217`).
- Safe truncation выбирает nominal stop на `blocker_s - 15 м`, затем двигает
  station назад до `5 м` от текущих prohibited cells
  (`drone_city_nav/src/safe_trajectory_truncation.cpp:85-114,119-190`).
- Known lower/upper solid validation, vertical/speed profiles, terminal braking,
  truncation ACK/pending protocol и local rollout ROI не зависят концептуально
  от hard inflation и должны сохраниться.
- Horizontal handover является отдельной post-finalization shape-changing
  стадией: planner preflight и offboard consumer независимо вызывают
  `buildHorizontalTrajectoryHandover()`, который строит Hermite bridge и сейчас
  бинарно проверяет его по prohibited grid
  (`drone_city_nav/src/planner_node_trajectory_publication.cpp:333-390`,
  `drone_city_nav/src/px4_offboard_node_trajectory.cpp:806-828`,
  `drone_city_nav/src/trajectory_horizontal_handover.cpp:149-342`). Поэтому
  одного risk-aware planner finalization недостаточно.

Notion и GitLab не читались: prompt не содержит Notion task, GitLab MR или
review context; при `notion_policy=optional` удаленный контекст не нужен.

## Detected stack/profiles

- ROS 2 Jazzy workspace, один основной package `drone_city_nav`.
- C++20, CMake/ament/colcon, GoogleTest, warnings-as-errors и локальные
  clang-format/clang-tidy/cppcheck checks.
- Gazebo Harmonic/PX4 SITL для интеграционных headless прогонов.
- Python используется в launch/scripts/tests, но планируемая production-логика
  находится в C++; отдельный Python project profile не требуется.
- Применены обязательные profiles:
  - `project_profiles/generic.md`;
  - `project_profiles/cpp.md`.
- Прочитаны repo rules: `AGENTS.md`, `CPP_BEST_PRACTICES.md`, `README.md`,
  `CONTRIBUTING.md`.

## Repo-approved commands found

Все build/test/quality/simulation команды выполняются только через container
workflow:

- `./scripts/build.sh` — `colcon build --packages-select drone_city_nav`;
- `./scripts/test.sh` — package build и полный package `ctest`;
- `./scripts/dev_shell.sh make format` — форматирование только измененных C++;
- `./scripts/dev_shell.sh make quality` — format check, clang-tidy, cppcheck,
  package build и package tests;
- `./scripts/dev_shell.sh make test-scripts` — Python script tests, нужен только
  если меняются scripts;
- `./scripts/sim_headless.sh` — документированный headless Gazebo/PX4 run;
- внутри container допустим targeted
  `ctest --test-dir build/drone_city_nav --output-on-failure -R '<regex>'`
  после `make build`.

Нельзя использовать ad-hoc top-level CMake или host build/test commands.

## Affected components

1. **Raw obstacle model и distance field**
   - `occupancy_grid.hpp/.cpp`;
   - `distance_field.hpp/.cpp`, `clearance_field.hpp/.cpp`;
   - `planning_grid_builder.hpp/.cpp`;
   - `planning_grid_snapshot.hpp/.cpp`,
     `planner_node_grid_snapshot.cpp`.

2. **Path generators**
   - `astar_planner.hpp/.cpp`, `planner_core.hpp/.cpp`,
     `path_smoothing.cpp`;
   - `receding_horizon_trajectory_planner.hpp/.cpp`;
   - `no_static_planner_orchestrator.hpp/.cpp`.

3. **Executable trajectory pipeline**
   - `trajectory_planner.hpp/.cpp`;
   - `corridor.hpp/.cpp`;
   - `trajectory_optimizer*.cpp` и optimizer internal headers;
   - `turn_smoothing*.cpp`, `trajectory_shape_cleanup.cpp`;
   - `trajectory_passage_insertion.cpp`;
   - `trajectory_horizontal_handover.hpp/.cpp`;
   - `trajectory_repair.hpp/.cpp`, `repair_race.hpp/.cpp`.

4. **Planner orchestration/publication/runtime**
   - `planner_node.hpp`;
   - `planner_node_inputs.cpp`, `planner_node_runtime.cpp`,
     `planner_node_route_publication.cpp`,
     `planner_node_trajectory_publication.cpp`,
     `planner_node_repair.cpp`, `planner_node_truncation.cpp`;
   - planner config/lifecycle/debug publication.

5. **Offboard и safe truncation**
   - `safe_trajectory_truncation.hpp/.cpp`;
   - `px4_offboard_node*.hpp/.cpp`;
   - ROS topic/config contract для raw obstacle snapshot.

6. **Удаляемая escape/relaxation подсистема**
   - `directed_inflation_escape.hpp/.cpp`;
   - `directed_inflation_escape_debug_markers.hpp/.cpp`;
   - radial relaxation API/state;
   - escape parameters, logs, RViz displays и tests.

7. **Diagnostics/config/docs/tests/build graph**
   - `trajectory_diagnostics*`, blackbox/debug fields;
   - `urban_mvp.yaml`, launch/config tests;
   - `README.md`, `docs/obstacle_mapping.md`,
     `docs/configuration.md`, RViz configs;
   - `drone_city_nav/CMakeLists.txt` и перечисленные unit/integration tests.

## Implementation steps

### 1. Ввести единый тип риска и единственный comparator

Добавить `drone_city_nav/include/drone_city_nav/obstacle_risk_field.hpp` и
`drone_city_nav/src/obstacle_risk_field.cpp`. Тип должен владеть/ссылаться на
immutable raw occupancy и `DistanceField2D` от `DistanceFieldSource::kOccupied`.
Политика получает critical distance `inflation_radius_m` и preferred distance
`inflation_radius_m + mode planning_clearance_m`.

Материализуемый контракт:

```cpp
enum class ObstacleRiskTier : std::uint8_t {
  kPreferred,
  kPlanningBand,
  kCriticalBand,
};

struct PathRiskScore {
  bool outside_bounds{false};
  bool intersects_raw_occupied{false};
  ObstacleRiskTier worst_tier{ObstacleRiskTier::kPreferred};
  double critical_exposure_m{0.0};
  double planning_exposure_m{0.0};
  double minimum_raw_clearance_m{infinity};
};

struct RankedPathCost {
  PathRiskScore risk;
  double algorithm_cost{0.0};
  std::uint64_t deterministic_tiebreak{0};
};
```

`compareRankedPathCost()` обязан сравнивать hard flags, worst tier, critical
exposure, planning exposure, затем algorithm-specific quality и deterministic
tie-break. `minimum_raw_clearance_m` сохраняется как диагностика полного
кандидата, но не должен становиться динамическим distance penalty: это не
возвращает ранее отвергнутую модель, которая вела вдоль границы карты.

Risk evaluator должен интегрировать длину segment внутри tiers по supercover
cells/отрезкам, а не считать только endpoints. Raw collision и выход за
evaluation ROI возвращают hard reject. Unknown-cell policy остается текущей.

Обновить `drone_city_nav/CMakeLists.txt`; добавить
`tests/obstacle_risk_field_test.cpp` с boundary, diagonal, partial-cell,
raw-collision и lexicographic-order cases.

### 2. Заменить builder/snapshot двумя raw-derived продуктами

В `planning_grid_builder.hpp/.cpp` (anchors
`buildPlanningGridUncached():183-254` и static cache path `:287-412`) оставить:

1. объединение static/memory/current-lidar raw occupied;
2. existing global/static cache;
3. local rollout ROI extraction с достаточным halo;
4. один occupied distance field;
5. mode-aware `ObstacleRiskPolicy`.

Удалить materialization через `applyInflationFromDistanceField()` и поля
`result.grid/result.planning_grid`. Переименовать result/status/cache types в
raw/risk terminology, например `ObstacleFieldBuildResult`.

ROI должен разделять:

- evaluation bounds, за которые candidate выходить не может;
- source halo, из которого строится distance field.

Halo должен быть не меньше preferred distance плюс половина клетки, иначе
obstacle сразу за краем ROI получит ложный infinite clearance.

В `planning_grid_snapshot.hpp/.cpp` заменить
`PreparedPlanningGridSnapshot` на immutable `PreparedObstacleRiskSnapshot`:

```cpp
struct PreparedObstacleRiskSnapshot {
  RawObstacleVersion version;
  OccupancyGrid2D raw_occupancy;
  DistanceField2D occupied_distance;
  ObstacleRiskField risk_field;
  GridBounds evaluation_bounds;
};
```

Snapshot reuse для rollout successor и generation/fingerprint guards сохранить;
fingerprint должен включать raw cells, bounds, source revisions и risk-policy
thresholds, но не бывшие inflation hashes.

### 3. Перевести A* на лексикографический path cost

В `astar_planner.hpp/.cpp`:

- заменить `kProhibitedStartOrGoal` на raw-occupied status;
- разрешить любой raw-free start/goal, включая бывшие inflation bands;
- в neighbor expansion (`astar_planner.cpp:256-285`) hard-reject делать только
  для raw occupied/outside;
- diagonal corner-cut check применять к raw occupied;
- заменить `double g_score/f_score` на монотонный tuple:

```cpp
struct AStarPathCost {
  ObstacleRiskTier worst_tier;
  double critical_exposure_m;
  double planning_exposure_m;
  double geometry_cost;
};
```

Каждый step обновляет `worst_tier=max(...)`, exposure неотрицательной длиной
step и текущие direction/heading costs. Heuristic добавляется только к
`geometry_cost`; risk-компоненты heuristic равны нулю. Это гарантирует
доминирование risk tiers без огромных coefficients.

В `planner_core.cpp:366-412,559-605` удалить
`nearestAllowedEscapeStartCell()` и `start_prohibited_escape_search_radius_m`.
Path result должен возвращать risk score и occupied distance diagnostics вместо
“prohibited escape/clearance”.

`path_smoothing.cpp` должен принимать shortcut только если он raw-clear и его
`PathRiskScore` лексикографически не хуже заменяемого subpath. Иначе smoothing
может уничтожить safe preference, найденную A*.

### 4. Перевести rollout на тот же evaluator

В `receding_horizon_trajectory_planner.hpp/.cpp` заменить бинарный
`traversable()` (`:24-63`) на shared continuous risk evaluation.

Каждый динамически валидный rollout:

1. hard-reject при raw collision/outside ROI;
2. получает `PathRiskScore`;
3. сравнивается с остальными сначала по risk comparator;
4. при равном риске использует текущий progress/lateral/heading/curvature score;
5. при полном равенстве использует `deterministic_index`.

Удалить reject reason/diagnostics `kProhibited`, заменить на
`kRawOccupied` и per-tier exposure diagnostics. `stationary_restart`,
minimum rollout length, terminal braking и local ROI contracts сохранить.

No-static и static rollout API должны использовать тот же
`PreparedObstacleRiskSnapshot`; если rollout в static режиме пока не вызывается
orchestration, его контракт все равно не должен иметь отдельной risk модели.

### 5. Перевести весь trajectory refinement pipeline без бинарного fallback

В `trajectory_planner.hpp/.cpp` удалить `TrajectoryGridCandidate`,
`grid_candidates`, per-stage grid attempt loops и fallback candidate. Заменить
их одним `TrajectoryRiskContext {raw_occupancy, risk_field, policy}`.

Для каждой shape-changing стадии зафиксировать один контракт:

- **Corridor** (`corridor.cpp:108,158,367-476`): границы строятся до raw
  occupied, occupied distance field остается источником доступного радиуса;
  fingerprint переименовать в raw occupancy fingerprint.
- **Optimizer** (`trajectory_optimizer*.cpp`): hard collision остается
  обязательным; `CandidateScore`/DP/window candidate ordering сравнивают
  `PathRiskScore` перед существующим geometry scalar. Кэш включает raw/risk
  fingerprint.
- **Turn smoothing**, **shape cleanup**, **passage insertion**:
  generated segment hard-reject только по raw occupied/outside/known solids;
  изменение принимается, если выполняет функциональную цель стадии и не
  ухудшает risk tuple относительно входного segment, либо возвращает явно
  ранжированный degraded candidate в общий comparator.
- **Known-passage 3D solid validation**, structural samples, Z/speed profiles,
  curvature-derived speed cap и terminal braking остаются hard gates.

Финальная validation должна повторно оценивать полный executable candidate по
тому же immutable snapshot и hard-reject только raw collision/outside/known
solid/structural invalidity. Soft-tier exposure записывается в diagnostics, но
не превращается обратно в reject.

### 6. Перевести global, partial repair и rollout orchestration атомарно

В `planner_node_inputs.cpp`, `planner_node_route_publication.cpp`,
`planner_node_trajectory_publication.cpp`, `planner_node_repair.cpp`,
`repair_race.*`:

- удалить создание/перебор `planning_clearance`/`runtime_prohibited` vectors;
- передавать один immutable risk snapshot в A*, rollout, partial/full repair и
  finalization;
- сохранить race arbiter/generation cancellation, но сравнивать результаты
  единым `RankedPathCost`;
- partial repair anchors должны быть hard-valid по raw occupancy, а не
  “allowed by any candidate grid” (`trajectory_repair.cpp:203-238`);
- сохранить no-static rollout successor snapshot reuse и `grid_build_ms=0`
  семантику, переименовав diagnostic в `risk_snapshot_reused`;
- не включать no-static A* recovery при default `false`; при явном включении он
  использует ту же risk policy.

Cutover должен быть общим для static/no-static. Нельзя оставить “временно A*
hard-grid, rollout soft-risk” в публикуемой конфигурации.

### 7. Перевести post-finalization horizontal handover на тот же risk contract

В `trajectory_horizontal_handover.hpp/.cpp`
(`buildHorizontalTrajectoryHandover():149-342`) заменить
`const OccupancyGrid2D* validation_grid` на обязательный
`TrajectoryRiskContext`. Параметр `require_validation_grid` переименовать в
`require_risk_context`.

Builder должен:

1. сохранить текущие projection, hard-window, join-distance и geometry limits;
2. hard-reject любой bridge, пересекающий raw occupied или выходящий за
   evaluation bounds;
3. оценивать всю фактически сшитую trajectory тем же `PathRiskScore`;
4. перебирать ограниченный детерминированный набор допустимых
   `old_join/candidate_join/bridge_scale` layouts и выбирать
   лексикографически лучший, а не первый geometric bridge;
5. возвращать исходный и итоговый risk score и явное качество
   `strict|degraded_handover`.

```cpp
struct HorizontalTrajectoryHandoverResult {
  // existing geometry diagnostics
  PathRiskScore candidate_risk;
  PathRiskScore stitched_risk;
  HandoverRiskQuality risk_quality;
};
```

Strict bridge не ухудшает risk tuple относительно candidate на заменяемом
интервале. Если из-за фактического offset дрона такого bridge нет, но есть
raw-clear вариант, допускается только лексикографически лучший
`degraded_handover`; ухудшение не скрывается, попадает в diagnostics и
скоростной профиль. Это сохраняет идеологию “движение лучше стояния”, не
возвращая prohibited band как hard gate.

Оба call path обязаны использовать этот API:

- planner preflight в
  `planner_node_trajectory_publication.cpp:333-390` передает тот же immutable
  `PreparedObstacleRiskSnapshot`, по которому был финализирован candidate, и
  проверяет именно `preflight.samples`;
- offboard в `px4_offboard_node_trajectory.cpp:806-828` строит актуальный
  raw/risk context и после bridge повторно применяет hard/risk validation до
  `buildOffboardTrajectoryState()`.

Чтобы offboard не угадывал static/no-static policy, расширить
`msg/ExecutableTrajectory.msg` полями policy identity:

```text
uint64 obstacle_snapshot_revision
float64 risk_critical_distance_m
float64 risk_preferred_distance_m
```

Offboard сверяет конечность/порядок thresholds, применяет их к текущему raw
obstacle snapshot и логирует несовпадение revision как использование более
свежих runtime данных, а не как повод принять bridge без проверки.

Обновить `tests/trajectory_horizontal_handover_test.cpp`:

- preferred bridge выбирается вместо более короткого critical bridge;
- unavoidable critical bridge принимается как `degraded_handover`;
- raw-occupied и outside-ROI bridge отклоняются;
- returned `stitched_risk` совпадает с повторной независимой оценкой samples;
- planner preflight и offboard consumer используют одинаковые thresholds;
- hard-window metadata и существующие geometry limits сохраняются.

### 8. Удалить escape и inflation relaxation полностью

Удалить production/test targets и файлы:

- `directed_inflation_escape.hpp/.cpp`;
- `directed_inflation_escape_debug_markers.hpp/.cpp`;
- `directed_inflation_escape_test.cpp`;
- `directed_inflation_escape_debug_markers_test.cpp`.

Удалить из `OccupancyGrid2D`:

- `inflated_`, `isInflated()`, `isProhibited()`;
- inflation apply/clear API;
- `LocalInflationRelaxationStats`;
- `inflated_hash`.

Удалить из planner:

- escape episode/generation/target/mission-continuation state;
- Dijkstra tunnel и radial 5 м relaxation;
- `unrelaxed_planning_clearance_grid`;
- `no_static_rollout_min_unrelaxed_tail_m` и tail validation;
- start-in-inflated detection и “escape target” target switching;
- escape logs, marker publisher/topic, RViz displays;
- все directed/radial parameters из config/lifecycle/YAML/tests.

`no_static_planner_orchestrator.*` после очистки должен отвечать только за
обычный rollout target/continuation/ACK behavior; если отдельный класс после
удаления escape не несет самостоятельной логики, объединить его с rollout
orchestration и удалить пустую abstraction.

### 9. Сохранить safe truncation, но устранить конфликт с soft critical band

В `planner_node_runtime.cpp:242-405` и `trajectory_repair.cpp:181-200`
заменить `findFirstProhibited...` на поиск первого raw-occupied span оставшейся
принятой trajectory. Только такое новое hard collision создает
`ReplanBlockerEvent` и запускает существующий
blocker -> truncation -> confirmation -> suffix ACK protocol.

Изменение risk exposure без raw intersection может планировать более хороший
successor в обычном periodic cycle, но не должно запускать аварийную truncation.

Заменить `/drone_city_nav/prohibited_grid` на
`/drone_city_nav/raw_obstacle_grid` для offboard runtime snapshot.
`currentProhibitedGrid()` и связанные freshness/diagnostic names переименовать
в raw-obstacle terms. Offboard строит occupied distance field локально.

Числа оставить неизменными:

- `safe_trajectory_truncation_margin_m = 15.0`;
- `safe_trajectory_terminal_prohibited_clearance_m = 5.0` (параметр можно
  переименовать, default не менять).

Чтобы сохранить текущую геометрию terminal hold после удаления materialized
prohibited grid, рекомендуемая формула:

```text
required_raw_clearance =
    former_critical_band_width(1.0 м)
    + terminal_clearance_beyond_critical_band(5.0 м)
```

То есть существующие `15.0` и `5.0` не меняются, а stop station остается
эквивалентной прежним `5 м от prohibited grid`. Raw blocker intersection
остается на расстоянии `0`.

Обновить `safe_trajectory_truncation.hpp/.cpp` так, чтобы функция принимала raw
grid/occupied distance field и явно логировала both
`terminal_clearance_beyond_critical_m=5.0` и
`required_raw_clearance_m=6.0`.

### 10. Обновить diagnostics, config и документацию

Удалить/переименовать поля:

- `planning_grid`, `prohibited_grid`, `grid_attempts`, inflation hashes;
- `planning_start_inflated`, prohibited intersection source;
- escape/relaxation diagnostics.

Добавить bounded diagnostics:

- raw snapshot revision/fingerprint и ROI/halo dimensions;
- policy thresholds;
- `worst_risk_tier`, critical/planning exposure, minimum raw clearance;
- hard reject source: raw occupied/outside/known solid;
- algorithm (`rollout|partial_astar|full_astar`);
- risk score before/after каждого shape-changing stage;
- safe trunc raw blocker и derived terminal clearance.
- horizontal handover `candidate_risk`, `stitched_risk`,
  `strict|degraded_handover` и raw/runtime snapshot revisions.

В `trajectory_diagnostics_io_*` writer перейти на новые names. Parser
рекомендуется оставить способным читать старые JSON/CSV fields как legacy
aliases, чтобы исторические run artifacts не перестали анализироваться.

Обновить `urban_mvp.yaml`, config tests, `README.md`,
`docs/obstacle_mapping.md`, `docs/configuration.md`, launch logs и RViz:
показывать raw occupied и при необходимости отдельную debug-визуализацию
distance/risk tiers, которая не участвует в planning input.

### 11. Обновить и расширить category-1 tests

Кроме нового `obstacle_risk_field_test`, изменить существующие tests без
добавления test-only production hooks:

- `planner_core_test`: A* предпочитает любой preferred detour критическому
  shortcut; при отсутствии detour проходит planning band, затем critical band;
  raw occupied остается unreachable; raw-free start внутри critical band
  получает выход без escape state.
- `receding_horizon_trajectory_planner_test`: те же tier priorities для полного
  segment, deterministic ties и raw/outside rejection.
- `path/corridor/trajectory_optimizer/turn_smoothing/trajectory_planner tests`:
  postprocessing не ухудшает risk tuple, narrow critical route остается
  executable, raw collision никогда не проходит.
- `trajectory_horizontal_handover_test`: planner preflight/offboard bridge
  hard-reject raw/outside, использует те же thresholds, возвращает наблюдаемый
  risk score и выбирает лучший strict/degraded bridge без скрытого ухудшения.
- `trajectory_repair_test` и `repair_race_test`: partial/global candidates
  сравниваются тем же score, anchors могут лежать в soft bands, raw anchors
  отклоняются.
- `planning_grid_builder_test` и `planning_grid_snapshot_test`: заменить на
  raw merge, one distance field, static cache, ROI halo, mode thresholds,
  immutable snapshot reuse.
- удалить escape tests и заменить scenario на естественный выход из critical
  band через A*/rollout risk ranking.
- `safe_trajectory_truncation_test`: raw blocker запускает truncation,
  intentional critical-band-only path не запускает blocker, `15/5` defaults и
  derived raw terminal distance сохранены.
- config/diagnostics/ROS conversion/offboard tests: новый topic/schema,
  отсутствие inflation values и legacy diagnostics parse.

### 12. Очистить build graph и доказать отсутствие старой семантики

Удалить stale sources/tests из `CMakeLists.txt`, includes, topic wiring и config.
В конце выполнить source-level guard:

```bash
rg -n \
  'isProhibited|isInflated|TrajectoryGridCandidate|grid_candidates|\
directed_inflation_escape|clearInflationWithinRadius|\
runtime_prohibited_grid|planning_clearance_grid'
```

Ожидаемый результат — пусто, кроме явно документированных legacy parser keys
или migration comments. Отдельным guard проверить, что `15.0` и `5.0` safe
trunc defaults не изменены.

## Verification plan

### Mandatory local verification

1. `./scripts/dev_shell.sh make format`
   - форматирует только измененные C++ файлы перед commit.
2. Targeted build + tests:

   ```bash
   ./scripts/dev_shell.sh bash -lc \
     "make build && ctest --test-dir build/drone_city_nav --output-on-failure \
      -R 'obstacle_risk_field|planner_core|receding_horizon_trajectory_planner|\
planning_grid_builder|planning_grid_snapshot|corridor|trajectory_optimizer|\
turn_smoothing|trajectory_planner|trajectory_horizontal_handover|\
trajectory_repair|repair_race|\
safe_trajectory_truncation|planner_node_config|px4_offboard_node_config|\
trajectory_diagnostics'"
   ```

   Scope широк, потому что один public planning contract проходит через
   generator, refinement, repair, serialization и offboard; меньший unit target
   не проверит согласованность этих компонентов.
3. `./scripts/dev_shell.sh make quality`
   - обязательный repo pre-commit check;
   - полный package build/test обоснован удалением public headers/sources,
     CMake targets и изменением центрального контракта для обоих режимов.
4. `ENABLE_STATIC_MAP=false ./scripts/sim_headless.sh`
   - no-static end-to-end: rollout из preferred и critical start, runtime raw
     blocker, truncation/suffix ACK, отсутствие escape/inflation logs.
5. `ENABLE_STATIC_MAP=true ./scripts/sim_headless.sh`
   - static end-to-end: A*, corridor/optimizer и partial/full repair используют
     тот же risk comparator; нет regression в passage/vertical execution.
6. После каждого run проверить logs:
   - raw collision никогда не опубликован;
   - при существовании preferred candidate не выбран soft-band candidate;
   - soft critical exposure не создает truncation само по себе;
   - новый raw occupied span создает ровно один truncation generation;
   - thresholds static `1/4 м`, no-static `1/6 м`;
   - `15/5` safe trunc values сохранены.

### Optional/CI verification

- Три static и три no-static headless run как soak/regression matrix с разными
  seeds. Это ловит sensor/timing variability, но не требуется для каждого
  локального промежуточного commit.
- `./scripts/dev_shell.sh make test-scripts` только если implementation затронет
  shell/Python launcher или log-analyzer scripts; без таких изменений check
  пропускается как нерелевантный.
- GUI/RViz smoke для raw/risk debug layers. Это не заменяет automated tests и
  нужен только для визуального контракта markers.

## Testing strategy

### Category 1 — обязательные tests без test-only refactoring

1. **Risk primitive**: exact tier boundaries, exposure integration,
   raw/outside hard reject, deterministic comparator.
2. **A***: preferred-over-critical regardless of geometric detour; fallback
   planning -> critical только при отсутствии более безопасного path; start
   inside critical; diagonal raw corner.
3. **Rollout**: full-trajectory risk beats scalar progress, not endpoint-only;
   deterministic shortlist; stationary restart.
4. **Refinement invariants**: smoothing/optimizer/insertion не ухудшают risk
   tier/exposure незаметно и не создают raw collision.
5. **Horizontal handover**: Hermite bridge в planner preflight и offboard
   hard-rejectится по raw/outside, оценивается общей policy, возвращает
   observable before/after score и детерминированно выбирает лучший
   strict/degraded layout.
6. **All-mode orchestration**: static A*, no-static rollout, partial repair и
   explicit no-static A* recovery получают одинаковую policy и snapshot.
7. **Snapshot/ROI**: obstacle in halo affects edge clearance; obstacle outside
   halo does not leak; successor reuses exact immutable revision.
8. **Runtime truncation**: raw intersection triggers; soft-band-only proximity
   does not; terminal station сохраняет `15/5` policy; stale generation/ACK
   behavior unchanged.
9. **Removal contract**: config no longer declares escape/relaxation params,
   CMake no longer builds removed targets, ROS raw topic conversion contains no
   inflated cells.

### Category 2/3 — optional follow-up

Property-based generation of random obstacle fields could проверить
лексикографическую монотонность A*/optimizer и отсутствие risk-regression после
arbitrary smoothing. Для этого потребуется отдельный fixture/generator layer,
поэтому это не блокирует основную реализацию.

## Risks and tradeoffs

1. **Физический риск повышается намеренно.** Planner сможет выбрать путь ближе
   `1 м` к raw obstacle, если более безопасного выхода нет. Hard raw/known-solid
   validation и controller limits остаются последней границей.
2. **Safe truncation может конфликтовать с penalty-моделью.** Старый
   prohibited trigger нельзя сохранить; иначе допустимый critical candidate
   будет обрезан сразу после публикации. Raw-intersection trigger обязателен.
3. **A* станет дороже.** Tuple comparison и risk accumulation увеличат стоимость
   expansion. Одновременно исчезают две materialized inflation grid и duplicate
   distance fields; измерить нужно отдельно build time и search time.
4. **Optimizer требует настоящего comparator, не post-filter.** Если применить
   risk только в final validation, промежуточные DP/window choices уже потеряют
   безопасный candidate. Это главный объем алгоритмического изменения.
5. **ROI edge artifacts.** Без source halo candidate у края локального window
   увидит завышенный clearance. Halo и отдельные evaluation bounds обязательны.
6. **Diagnostics schema изменится.** Writer names необходимо обновить, а legacy
   parser сохранить на переходный период, иначе старые runs перестанут
   анализироваться.
7. **Удаление `inflated_` затрагивает raw-source validation.** Проверки,
   запрещающие inflation в memory input, нужно заменить прямым raw-only type/API,
   а не просто удалить; иначе producer снова сможет передать derived state.
8. **Большой atomic cutover.** Реализацию можно разбить на commits, но нельзя
   включать production path до перевода всех стадий. Рекомендуемый порядок:
   primitive/builder -> A*/rollout -> refinement/repair -> runtime/offboard ->
   removal/docs/tests.
9. **Post-finalization geometry может обойти planner validation.** Horizontal
   handover строится заново в offboard по более свежему pose/raw snapshot.
   Поэтому policy identity должна передаваться в executable message, а
   фактические stitched samples обязаны проходить shared evaluator перед
   активацией; planner preflight сам по себе этого риска не закрывает.

## Open questions

### 1. Что именно означает сохранение `5.0 м` terminal clearance после удаления prohibited grid?

**Recommended decision:** сохранить текущую геометрию: `5.0 м` за внешней
границей former critical band, то есть требовать `1.0 + 5.0 = 6.0 м` raw
clearance. `15.0` и `5.0` defaults не меняются.

**Rationale:** это единственный вариант, который действительно сохраняет
текущий safe truncation contract, а не только цифру в YAML.

**Confirmation:** unit test на одинаковую stop station до/после миграции для
одной и той же raw wall geometry.

### 2. Должен ли minimum raw clearance участвовать в основном comparator?

**Recommended decision:** хранить его в diagnostics и использовать только как
последний tie-break для завершенных кандидатов; основные приоритеты —
worst tier и exposure lengths.

**Rationale:** пользователь явно отверг непрерывный distance penalty из-за
движения по краю карты. Exposure tiers дают статический и доказуемый порядок.

**Confirmation:** golden tests, где два preferred пути отличаются clearance,
но progress/geometry должны остаться решающими.

### 3. Как мигрировать diagnostics schema?

**Recommended decision:** writer пишет только новые raw/risk fields, parser один
переходный период читает и новые, и старые prohibited/inflated aliases.

**Rationale:** production contract очищается, но локальные исторические JSON/CSV
артефакты остаются анализируемыми.

**Confirmation:** roundtrip нового schema плюс parse существующего legacy
fixture.

### 4. Нужен ли runtime replan при ухудшении soft risk без raw intersection?

**Recommended decision:** да, обычный coalesced rollout/replan может искать
лучший path, но без safe truncation и без invalidation текущей raw-clear
траектории.

**Rationale:** это улучшает маршрут при новых данных и не возвращает бесконечную
truncation-петлю.

**Confirmation:** integration test: новая obstacle memory переводит suffix из
preferred в critical, текущий path остается executable, successor строится;
raw overlap отдельно запускает truncation.

### 5. Нужно ли сохранять неизвестные клетки как сейчас?

**Recommended decision:** да, не менять unknown policy в этой задаче.

**Rationale:** задача меняет inflation semantics, а одновременное изменение
unknown-space policy сделает причины поведения неразличимыми.

**Confirmation:** existing raw overlay tests и новый risk-field test для unknown
cell.

### 6. Делить ли реализацию на отдельные задачи?

**Recommended decision:** вести как один feature с несколькими reviewable
commits и единым финальным cutover; при организационном ограничении выделить
optimizer/refinement migration в отдельную зависимую подзадачу, но не включать
soft-risk в production до ее завершения.

**Rationale:** scope XL и затрагивает около 90 файлов, однако функционально
неделим: partial conversion нарушит единый механизм, явно потребованный
пользователем.

**Confirmation:** source-level guard из шага 12 и all-mode integration tests не
должны находить ни одного hard inflation consumer.
