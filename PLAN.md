# План: постоянная 3D-фильтрация известных passage-building масс

## Context

Нужно заменить временную, зависящую от положения дрона
`passage_traversal_sensor_policy` на постоянную классификацию каждого lidar hit
до сведения измерения к 2D obstacle layer. Попадания, совпадающие с известными
физическими `left_mass` / `right_mass` / `lower_mass` / `upper_mass`, не должны
становиться dynamic obstacles ни в `obstacle_memory`, ни в current lidar overlay.
Более близкий неизвестный объект, объект в свободном opening volume и любой
неоднозначный hit должны сохраняться.

Границы задачи:

- не менять A*, его веса и выбор маршрута;
- не добавлять предпочтение или обязательность пролёта через opening;
- не менять geometry/vertical profile, passage insertion, passage validation,
  speed profile и `known_passage_traversal_speed_limit_mps`;
- использовать `knownPassageSolidVolumes()` как единственный источник 3D-геометрии
  известных статических масс;
- сохранить fail-open поведение: если known-passage map или 3D pose недоступны,
  lidar hit не подавляется.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на чтении текущих call
sites и тестов.

Обнаружено:

- `projectLidarBeam()` в
  `drone_city_nav/src/lidar_projection.cpp:155` уже вычисляет `endpoint`,
  `endpoint_altitude_m` и нормализованное направление луча, но наружу не даёт
  явно названные map-frame origin/direction с Z вверх.
- `ObstacleMemoryGrid::integrateScan()` в
  `drone_city_nav/src/obstacle_memory.cpp:109` применяет free-ray misses, затем
  без 3D-классификации вызывает `applyHit()`; Z после projection теряется.
- `overlayCurrentLidarHits()` в
  `drone_city_nav/src/current_lidar_overlay.cpp:20` независимо повторяет projection
  и также записывает только XY endpoint.
- `knownPassageSolidVolumes()` в
  `drone_city_nav/src/known_passage_solid_volumes.cpp:77` уже материализует
  ориентированные объёмы физических частей passage-building и используется
  mission monitor и RViz.
- Obstacle memory node уже получает altitude, roll, pitch, yaw и mount RPY, но
  пока не загружает known-passage map. Planner node map загружает.
- Старая policy в `drone_city_nav/src/planner_node_inputs.cpp:270` фильтрует
  временную копию memory/current lidar только в station-window текущей trajectory.
  `evaluatePassageAwareProhibitedIntersectionAction()` не отменяет реплан и
  используется только для диагностической метки.
- 2D memory не хранит provenance/Z отдельного hit. Уже накопленные expected-wall
  scores нельзя безопасно удалить выборочно: нужен полный reset при смене
  классификатора и повторное накопление через новый фильтр.

## Detected stack/profiles

- Прочитан обязательный профиль `generic.md`: выбран repo-approved container
  workflow и scoped-to-broad verification.
- Прочитан обязательный профиль `cpp.md`: затрагиваются C++20 `.hpp/.cpp`,
  `CMakeLists.txt`, GTest и ROS 2 nodes.
- Stack: ROS 2 Jazzy, `ament_cmake`, C++20, `colcon`, GTest, Python unittest для
  script contracts, PX4 SITL/Gazebo для end-to-end проверки.
- Rust profile не применяется: Rust-код не затрагивается.
- Notion policy `optional`; prompt не содержит Notion task, поэтому Notion read
  не выполняется.
- Prompt не содержит GitLab/MR, поэтому GitLab protocol прочитан, но удалённые
  GitLab reads не выполняются.

## Repo-approved commands found

Из `AGENTS.md`, `README.md`, `CONTRIBUTING.md` и `Makefile`:

```bash
./scripts/build.sh
./scripts/test.sh
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make test-scripts
./scripts/dev_shell.sh make quality
MISSION_CHECK=1 SMOKE_DURATION_S=120 ./scripts/sim_headless.sh
```

Для focused GTest после documented build допустим documented container `ctest`:

```bash
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
  -R 'known_static_lidar_hit_classifier|lidar_projection|obstacle_memory|current_lidar_overlay|planner_node_config|planning_grid_builder' \
  --output-on-failure
```

## Affected components

| Компонент | Файлы | Изменение |
|---|---|---|
| 3D ray contract | `include/drone_city_nav/lidar_projection.hpp`, `src/lidar_projection.cpp` | Явные map-frame origin/direction с Z вверх, согласованные с существующими endpoint полями |
| Known static classifier | новые `include/drone_city_nav/known_static_lidar_hit_classifier.hpp`, `src/known_static_lidar_hit_classifier.cpp` | Ray/OBB intersection, nearest expected range и консервативная классификация |
| Shared passage solids | `include/drone_city_nav/known_passage_solid_volumes.hpp`, `src/known_passage_solid_volumes.cpp` | Машиночитаемый part kind для bounded diagnostics без строкового разбора в hot loop |
| Accumulated memory | `include/drone_city_nav/obstacle_memory.hpp`, `src/obstacle_memory.cpp`, `src/obstacle_memory_node.cpp` | Подавление expected hit до `applyHit()`, reset, загрузка map, counters/logs |
| Current lidar | `include/drone_city_nav/current_lidar_overlay.hpp`, `src/current_lidar_overlay.cpp`, `src/planner_node_lidar_overlay.cpp`, `src/planner_node_inputs.cpp` | Тот же classifier до маркировки XY cell |
| Planner/config | `include/drone_city_nav/planner_node_config.hpp`, `src/planner_node_config.cpp`, `src/planner_node.hpp`, `src/planner_node_lifecycle.cpp`, `config/urban_mvp.yaml` | Новый tolerance contract, classifier lifecycle, удаление old policy params |
| Planning diagnostics | `include/drone_city_nav/planning_grid_builder.hpp`, `src/planner_node_inputs.cpp`, `src/planner_node_runtime.cpp` | Удаление filtered-memory/passage-policy diagnostics, добавление classifier counters |
| Build/tests/docs | `drone_city_nav/CMakeLists.txt`, релевантные `tests/*.cpp`, `scripts/validate_drone_nav_headless.py`, `scripts/tests/test_validate_drone_nav_headless.py`, `docs/configuration.md`, `docs/diagnostics.md`, `docs/rviz.md` | Новые regression contracts и удаление stale policy surface |

## Implementation steps

1. **Зафиксировать единый map-frame 3D ray contract и pure classifier.**
   - В `LidarBeamProjection` (`lidar_projection.hpp`, `projectLidarBeam()` в
     `lidar_projection.cpp`) добавить origin и unit direction в map frame, где
     `z` направлен вверх. Новые поля должны удовлетворять инварианту:

     ```cpp
     endpoint_3d == ray_origin_map_m + used_range_m * ray_direction_map;
     endpoint_3d.x/y == endpoint.x/y;
     endpoint_3d.z == endpoint_altitude_m;
     ```

   - В `known_passage_solid_volumes.hpp/.cpp` добавить enum части здания
     (`left`, `right`, `lower`, `upper`), сохранив `part_id` для существующих
     RViz/mission diagnostics.
   - Добавить `KnownStaticLidarHitClassifier`, который при construction один раз
     принимает результат `knownPassageSolidVolumes(map)` и не аллоцирует в
     `classify()`.
   - Реализовать slab ray intersection в локальных OBB-координатах
     `(normal, lateral, z-up)` для всех volumes и выбирать ближайший
     неотрицательный expected range.
   - Материализуемый результат: pure API с результатом
     `expected_static / unexpected / ambiguous`, expected range, range delta,
     part kind и bounded structure/opening/part identity.

     ```cpp
     if (!pose_3d_valid || !ray_valid || volumes.empty()) return ambiguous;
     expected = nearestSolidIntersection(ray, volumes);
     if (!expected) return unexpected;
     if (measured < expected.range - tolerance) return unexpected;
     if (abs(measured - expected.range) <= tolerance) return expected_static;
     return ambiguous;
     ```

2. **Интегрировать classifier в accumulated obstacle memory до `applyHit()`.**
   - Расширить `ObstacleMemoryGrid::integrateScan()` в
     `obstacle_memory.hpp/.cpp` optional classifier input без ownership transfer.
   - Сохранить free-ray updates для expected static hit и подавлять только
     `applyHit(endpoint_cell)`: измерение всё ещё доказывает свободное пространство
     до известной стены.
   - Для `unexpected` и `ambiguous` сохранить текущее hit scoring без изменений.
   - Добавить `ObstacleMemoryGrid::reset()` для атомарного сброса raw states и
     scores. Вызывать reset при установке/смене known-passage geometry/tolerance;
     на обычном startup map загружается до создания scan subscription, поэтому
     legacy expected hits не успевают накопиться.
   - В `obstacle_memory_node.cpp` загрузить `known_passages_enabled/path` через
     существующий `loadKnownPassageMapSource()`, добавить dependency
     `ament_index_cpp` в `CMakeLists.txt`, построить immutable classifier до
     подписок. При load/frame failure работать fail-open и логировать ошибку.
   - Добавить одинаковый параметр
     `known_static_lidar_hit_range_tolerance_m` в obstacle-memory namespace YAML.
   - Материализуемый результат: `upper/lower/side` hits никогда не увеличивают
     memory occupancy score, а более близкие/неоднозначные hits продолжают это
     делать.

3. **Применить тот же classifier к current lidar overlay.**
   - Расширить `overlayCurrentLidarHits()` в
     `current_lidar_overlay.hpp/.cpp` optional classifier input и вызывать его
     после accepted hit projection, до `markCurrentLidarObstacle()`.
   - В `PlannerNode::loadConfiguredKnownPassages()` построить/сбросить classifier
     вместе с `known_passages_`; хранить его immutable в `planner_node.hpp`.
   - Передать classifier из `PlannerNode::overlayCurrentLidarHits()` в общий core
     overlay. Добавить тот же tolerance parameter в planner namespace YAML и
     config parser; script/config test должен подтверждать равенство двух
     node-specific значений.
   - Материализуемый результат: current lidar и accumulated memory принимают
     решение одной функцией, на одной геометрии и с одним tolerance.

4. **Добавить bounded diagnostics без логирования каждого beam.**
   - В общую stats-структуру добавить:
     `expected_static_hits_ignored`, `unexpected_hits_kept`,
     `ambiguous_hits_kept`, fixed counters для `left/right/lower/upper`, а также
     identity первого ignored/ambiguous volume и соответствующий `range_delta_m`.
   - Включить stats в `ObstacleMemoryStats` и `CurrentLidarOverlayStats`;
     obstacle memory update и planner planning summary логируют агрегаты один раз
     за scan/build, без heap growth и per-beam logs.
   - В `planner_node_runtime.cpp` сохранить полный unthrottled prohibited-replan
     event и raw source probe; убрать только старый `passage_sensor_policy[...]`
     блок. Новый classifier не отменяет реплан: retained unexpected obstacle
     остаётся обычной причиной `prohibited_confirmed`.
   - Материализуемый результат: по логам видно, какая известная масса подавлена,
     а какой hit сохранён; A*/stable-path contracts не меняются.

5. **Удалить proximity-based expected-wall filtering после подключения обеих lidar paths.**
   - Удалить:
     `include/drone_city_nav/passage_traversal_sensor_policy.hpp`,
     `src/passage_traversal_sensor_policy.cpp`,
     `tests/passage_traversal_sensor_policy_test.cpp` и их CMake entries.
   - Из `planning_grid_builder.hpp` удалить `filtered_memory_grid` и
     `PassageTraversalSensorPolicyStats`; из `PlannerNode::buildPlanningGrid()`
     удалить временную memory copy и station-dependent policy invocation.
   - Из planner config/lifecycle/YAML удалить:
     `passage_traversal_sensor_policy_enabled`, activation/lookahead margins,
     opening-corridor margins и expected-wall margin.
   - Удалить `ignore_expected_walls`, `emergency_blocker`, active passage policy
     logs и их config/unit tests. Не удалять другие `known_passage_*` параметры,
     особенно validation, vertical profile и traversal speed limit.
   - Обновить `docs/configuration.md`, `docs/diagnostics.md`, `docs/rviz.md` на
     always-on 3D classifier и его fail-open semantics.
   - Материализуемый результат: в runtime нет trajectory/proximity-dependent
     sensor filtering и нет мёртвого policy API.

6. **Добавить автотесты и regression scenario.**
   - Новый `known_static_lidar_hit_classifier_test.cpp`:
     axis-aligned и rotated OBB, upper/lower/left/right, no intersection,
     exact-within-tolerance, closer unexpected hit, farther ambiguous hit,
     invalid altitude/direction и origin-inside-volume fail-open edge case.
   - `lidar_projection_test.cpp`: map-frame origin/direction/end-point invariants
     при разных roll/pitch/yaw и mount RPY.
   - `obstacle_memory_test.cpp`: expected upper/lower не занимают cell, free ray
     обновляется, closer object занимает cell, opening hit сохраняется, reset
     удаляет старые scores.
   - `current_lidar_overlay_test.cpp`: те же classifier decisions дают те же
     counters и grid result, включая rotated passage.
   - Regression fixture для `connector_22_23`: origin около высоты `13.8 m`, луч
     к `upper_mass` не появляется ни в current lidar grid, ни в memory/prohibited
     input; объект перед upper и объект внутри opening остаются.
   - Обновить `planner_node_config_test.cpp`, `px4_offboard_config_test.cpp` и
     script contracts: новые параметры присутствуют и согласованы, удалённые
     параметры отсутствуют.
   - Расширить headless validator optional expected-passage contract так, чтобы
     test mission могла автоматически сравнить
     `actual_passage_openings_seen == known_passage_openings`, не влияя на
     runtime mission control или A*.
   - Материализуемый результат: happy path, negative path и coordinate/tolerance
     edges защищены unit/component/e2e тестами.

7. **Выполнить end-to-end passage regression без специальных route preferences.**
   - Запустить default known-passage mission в headless режиме и проверить:
     все openings реально увидены mission diagnostics; expected solid counters
     ненулевые; в prohibited-replan events нет raw source от известных
     upper/lower/side hits; настоящий synthetic blocker test остаётся retained.
   - Не принимать успешный A-to-B flight как достаточное доказательство без
     opening count и classifier counters.
   - Материализуемый результат: тест доказывает достоверность obstacle layer, а
     не искусственное предпочтение A*.

## Verification plan

Проверки выполнять последовательно, чтобы не повредить общие colcon/CTest
артефакты:

1. `./scripts/dev_shell.sh make format` — только изменённые C++ files.
2. `./scripts/build.sh` — ROS package build через documented colcon wrapper.
3. Focused `ctest` command из раздела repo-approved commands — новые и напрямую
   затронутые unit/component tests.
4. `./scripts/dev_shell.sh make test-scripts` — YAML/topic/headless-validator
   contracts.
5. `./scripts/dev_shell.sh make quality` — format dry-run, build, все C++ tests,
   scoped clang-tidy и cppcheck.
6. `MISSION_CHECK=1 SMOKE_DURATION_S=120 ./scripts/sim_headless.sh` — тяжёлый
   passage end-to-end; проверить три opening events, mission success и новые
   classifier/replan diagnostics.

Никакие проверки реализации в plan-round не выполняются; они относятся к
будущему implementation-round. Для самого `PLAN.md` перед commit выполняется
обязательный repo `make quality` согласно `AGENTS.md`.

## Testing strategy

### Категория 1 — без рефакторинга

- Pure unit tests ray/OBB intersection и classification truth table.
- `LidarBeamProjection` frame/sign invariants.
- Geometry part-kind tests для `knownPassageSolidVolumes()`.
- Цель: локализовать математические ошибки без ROS и grid side effects.

### Категория 2 — лёгкий рефакторинг

- Component tests `ObstacleMemoryGrid` и current lidar overlay с одним shared
  classifier fixture.
- Config/load failure/reset tests и удаление old policy contracts.
- Planning-grid regression: ignored known static hit не становится raw occupied
  source; closer/ambiguous hit становится.
- Цель: доказать одинаковое поведение двух ingestion paths и fail-open policy.

### Категория 3 — тяжёлый рефакторинг/integration

- Script validator tests для passage-count/counter contract.
- Полный headless PX4/Gazebo прогон с физическими passage buildings.
- Отдельный synthetic blocker scenario внутри opening, если его можно задать
  существующим world fixture без изменения production route logic; иначе этот
  negative path остаётся обязательным component test, а e2e gap явно фиксируется.
- Цель: поймать frame drift, SDF/annotation mismatch, timing и cross-node config
  divergence, которые unit tests не видят.

## Risks and tradeoffs

- **Слишком большой tolerance может скрыть объект непосредственно перед известной
  массой.** Митигация: nearest expected range, closer-hit branch и conservative
  ambiguous=keep; подобрать tolerance по logged range delta distribution.
- **Слишком маленький tolerance сохранит шумные expected hits.** Митигация:
  параметр, counters по part и headless regression на реальные scan/pose latency.
- **Ошибка знака Z или frame conversion даст систематическую misclassification.**
  Митигация: явный map-frame ray contract и invariant tests при roll/pitch/yaw.
- **Known passage annotation может разойтись с SDF.** Митигация: существующий
  single geometry file contract/script tests, range mismatch => ambiguous/keep,
  fail-open при load/frame error.
- **Полный reset memory временно удалит реальные dynamic obstacles.** Митигация:
  reset только до subscriptions на startup или при явной map/classifier смене;
  current lidar остаётся активным, memory быстро переобучается.
- **Per-beam O(volumes) raycast увеличит CPU.** Для текущих трёх structures это
  bounded число slab checks; volumes precomputed, `classify()` без allocations.
  При масштабировании до сотен passages понадобится spatial index, но добавлять
  его без измерения сейчас не следует.
- **Удаление старых diagnostics меняет log/config contracts.** Митигация:
  одновременное обновление docs, config tests и script contracts, без compatibility
  aliases и мёртвых параметров.

## Open questions

1. **Какой initial range tolerance выбрать?**
   - Recommended decision: `0.5 m` для первого включения.
   - Rationale: текущая grid resolution равна `0.5 m`, pose latency уже motion
     compensated, а classifier сравнивает continuous ranges до grid projection.
   - Подтверждение: собрать histogram/summary `abs(measured-expected)` в headless
     runs и убедиться, что expected surfaces попадают в tolerance, а injected
     closer obstacles — нет.

2. **Что делать при ошибке загрузки/несовпадении frame known-passage map?**
   - Recommended decision: fail-open — сохранить все lidar hits, залогировать
     ошибку и не создавать classifier.
   - Rationale: false obstacle/replan безопаснее, чем скрытый неизвестный объект.
   - Подтверждение: unit/config test с missing path и mismatched frame.

3. **Как удалить уже накопленные ambiguous 2D scores?**
   - Recommended decision: полный `ObstacleMemoryGrid::reset()` при установке или
     изменении classifier geometry/tolerance; не применять глобальную XY-mask.
   - Rationale: у старой cell потерян Z, поэтому selective erase может скрыть
     настоящий blocker внутри opening.
   - Подтверждение: reset test и startup ordering test/log, доказывающий загрузку
     classifier до scan subscription.

4. **Нужно ли фильтровать только upper/lower?**
   - Recommended decision: фильтровать все volumes, возвращаемые
     `knownPassageSolidVolumes()` (`left/right/lower/upper`).
   - Rationale: все они являются известной статической 3D-геометрией; отдельные
     правила снова создадут drift между physical model и sensor policy.
   - Подтверждение: part-specific unit tests и per-part counters.

5. **Нужно ли сохранять старый proximity filter как fallback?**
   - Recommended decision: нет; удалить после подключения classifier к обеим
     ingestion paths.
   - Rationale: два слоя будут конфликтовать, а старый 2D filter не способен
     отличить upper/lower от blocker внутри opening.
   - Подтверждение: `rg` не находит old API/config/log fields, а full quality и
     headless regression проходят.

