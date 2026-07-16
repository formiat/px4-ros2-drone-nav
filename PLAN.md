# План: отбрасывание ожидаемых отражений от земли до 2D lidar ingestion

## Context

Задача должна исключить ложные динамические препятствия от плоской поверхности Gazebo,
когда roll/pitch дрона направляет номинально горизонтальный 2D lidar вниз. Решение должно
работать для каждого луча по ожидаемой дальности до известной плоскости земли, а не через
глобальное отключение lidar по углу наклона. Обычный быстрый полёт уже достигает примерно
40-42 градусов tilt, поэтому пороговая блокировка всего scan неприемлема.

Изменение затрагивает оба независимых пути lidar evidence:

1. накопительную `ObstacleMemoryGrid` в `obstacle_memory_node`;
2. моментальный planner overlay `current_lidar`.

Для одного и того же спроецированного луча оба пути должны получать одинаковое решение до
любой мутации 2D grid. Изменение поведения должно остаться отдельным, независимо ревьюимым
и откатываемым коммитом относительно уже завершённой task `001` с persistent XYZ
provenance.

В scope не входят 3D/voxel memory, рельеф, изменение A*/inflation/clearance, управление,
траектория, геометрия мира, global tilt cutoff и выборочное удаление старой memory по Z.

## Investigation context

`INVESTIGATION.md` в корне workspace отсутствует, поэтому отдельного входного артефакта
расследования нет. `PLAN.md` перед этим раундом также отсутствовал; план создан заново на
основании текущего `main` и локального кода.

Проверенные факты текущей реализации:

- `projectLidarBeam()` уже формирует map-frame `ray_origin_map_m`, нормализованный
  `ray_direction_map`, `used_range_m`, endpoint XYZ и applied roll/pitch/tilt в
  `drone_city_nav/src/lidar_projection.cpp:153-221` и
  `drone_city_nav/include/drone_city_nav/lidar_projection.hpp:18-80`.
- Контракт task `001` (`LaserScanTiming`, `LidarBeamObservation`,
  `AcceptedObstacleMemoryHit`, sparse `MemoryCellProvenance`) находится в
  `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:18-109`.
  Фабрики observation/snapshot сейчас спрятаны в anonymous namespace
  `drone_city_nav/src/obstacle_memory.cpp:104-149`, поэтому второй ingestion path не может
  переиспользовать их без небольшого extract-refactor.
- В `ObstacleMemoryGrid::integrateScan()` projected free cells получают `applyMiss()` в
  `drone_city_nav/src/obstacle_memory.cpp:270-302`, а known-static классификация выполняется
  только после этого в `:314-337`. Endpoint suppression сейчас не может отменить уже
  выполненное clearing.
- `currentLidarOverlay()` независимо проецирует scan и помечает endpoint в
  `drone_city_nav/src/current_lidar_overlay.cpp:24-96`. Free-ray clearing там нет, но ложный
  endpoint сразу попадает в raw planner evidence и затем может быть inflated.
- `KnownStaticLidarHitClassifier::classify()` сам ищет ближайший solid и применяет
  асимметричные tolerances в
  `drone_city_nav/src/known_static_lidar_hit_classifier.cpp:181-228`, но API не отдаёт
  независимый expected-surface candidate для сравнения с ground plane.
- Оба ROS node независимо загружают projection и known-static параметры из двух секций
  `drone_city_nav/config/urban_mvp.yaml:1-140`; ground config также должен присутствовать в
  обеих секциях и иметь одинаковую семантику.
- `min_projected_lidar_altitude_m: 1.0` уже является coarse sanity filter в обоих путях.
  Его нужно сохранить как отдельный ранний non-mutating guard, а не переименовывать и не
  использовать вместо range-based ground decision.
- Физическая земля в `drone_city_nav/worlds/generated_city.sdf:48-69` — box высотой
  `0.10 m` с pose `z=0`, поэтому её верхняя collision-плоскость находится на map Z `0.05 m`.
  Это обоснованный default для `ground_altitude_m` текущего мира.
- Existing task-001 provenance сериализуется через
  `drone_city_nav/msg/ObstacleMemoryProvenance.msg` и связанные cell/hit messages; planner
  сопоставляет snapshot по stamp/grid contract. Rejected ground beams не являются accepted
  memory evidence и не должны появляться в этом sparse state.
- Headless validator в `scripts/validate_drone_nav_headless.py:99-125` всё ещё парсит старое
  поле `tolerance=`, хотя runtime known-static log уже разделён на closer/farther. Этот stale
  parser нужно поправить в том же script-contract изменении, иначе новая effective-config
  проверка будет опираться на уже некорректный baseline.
- Существующие unit/integration tests покрывают projection, memory score/free clearing,
  current overlay, known-static ingestion и planning-grid integration. Новая логика должна
  расширить эти targets, а не заменять их отдельным тестовым workflow.

Notion не читался: policy `optional`, а prompt не содержит Notion task ID или ссылки. GitLab
не читался: prompt не упоминает MR/GitLab. Это соответствует прочитанным обязательным
протоколам и не ограничивает локальное планирование.

## Detected stack/profiles

- Основной стек: C++20, ROS 2, `ament_cmake`/CMake и `colcon`; production code находится в
  `drone_city_nav/include` и `drone_city_nav/src`, gtest — в `drone_city_nav/tests`.
- Дополнительный затронутый стек: Python `unittest` для script/config/headless contracts.
- Симуляция: PX4 SITL + Gazebo, запуск только через container scripts.
- Прочитан обязательный профиль `project_profiles/generic.md`: repo-approved команды имеют
  приоритет, проверки идут от targeted к package/repository scope.
- Прочитан профиль `project_profiles/cpp.md`, поскольку workspace содержит C++ headers,
  `.cpp`, `CMakeLists.txt` и ROS 2 CMake targets.
- Отдельный Rust profile не применяется: Rust-файлы и Cargo build не затрагиваются;
  orchestrator docs были только инструкциями, не workspace задачи.
- Прочитаны `AGENTS.md`, `CPP_BEST_PRACTICES.md`, `README.md`, `CONTRIBUTING.md` и
  `Makefile`. Они требуют container-only workflow, scoped C++ formatting и `make quality`
  перед коммитом.

## Repo-approved commands found

Команды запускаются с корня workspace:

```bash
# Build всего ROS package через документированный container/colcon workflow.
./scripts/build.sh

# Targeted CTest после build; используется документированный build directory.
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure \
  -R 'lidar_projection_test|ground_lidar_ingestion_decision_test|obstacle_memory_test|current_lidar_overlay_test|known_static_lidar_hit_classifier_test|known_static_lidar_ingestion_test|known_static_lidar_planning_grid_integration_test'

# Полный package test scope при cross-component изменении shared ingestion contract.
./scripts/test.sh

# Script/config/headless-validator contracts.
./scripts/dev_shell.sh make test-scripts

# Только изменённые C++ файлы, затем обязательный quality gate.
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality

# Headless simulation после implementation commit.
./scripts/sim_headless.sh
```

Прямой top-level `cmake`, host-side `colcon`, ad-hoc compiler invocation и запуск simulation
вне container workflow не разрешены локальными инструкциями.

## Affected components

1. **Projected beam observation contract**
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`
   - новый reusable header/source для task-001 observation factory, например
     `drone_city_nav/include/drone_city_nav/lidar_beam_observation.hpp` и
     `drone_city_nav/src/lidar_beam_observation.cpp`

2. **Expected surfaces и shared ingestion decision**
   - `drone_city_nav/include/drone_city_nav/known_static_lidar_hit_classifier.hpp`
   - `drone_city_nav/src/known_static_lidar_hit_classifier.cpp`
   - новые
     `drone_city_nav/include/drone_city_nav/lidar_ingestion_decision.hpp` и
     `drone_city_nav/src/lidar_ingestion_decision.cpp`
   - `drone_city_nav/CMakeLists.txt`

3. **Accumulated obstacle memory**
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`
   - `drone_city_nav/src/obstacle_memory.cpp`
   - `drone_city_nav/src/obstacle_memory_node.cpp`
   - task-001 provenance messages/ROS conversion, только если accepted-decision snapshot
     добавляется в persistent accepted evidence

4. **Planner current lidar**
   - `drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp`
   - `drone_city_nav/src/current_lidar_overlay.cpp`
   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/src/planner_node_inputs.cpp`
   - `drone_city_nav/src/planner_node_lidar_overlay.cpp`
   - `drone_city_nav/src/planner_node_lifecycle.cpp`
   - `drone_city_nav/src/planner_node_runtime.cpp`

5. **Config, tests, contracts и docs**
   - `drone_city_nav/config/urban_mvp.yaml`
   - C++ tests в `drone_city_nav/tests`
   - `scripts/validate_drone_nav_headless.py`
   - `scripts/tests/test_validate_drone_nav_headless.py`
   - `scripts/tests/test_topic_contract.py`
   - `docs/obstacle_mapping.md`
   - `docs/configuration.md`
   - `docs/diagnostics.md`
   - `docs/troubleshooting.md`
   - `docs/known_passages.md`

## Implementation steps

1. **Вынести reusable task-001 observation construction без изменения mapping behavior.**
   Из `drone_city_nav/src/obstacle_memory.cpp:104-149` вынести
   `makeClassificationSnapshot()` и `makeBeamObservation()` в focused module
   `lidar_beam_observation.hpp/.cpp`; оставить `LidarBeamObservation` единственным beam DTO.
   Дополнить вход фабрики `effective_max_range_m`, потому что для measured hit
   `projection.used_range_m` равен measured range и сам по себе не показывает, находится ли
   ожидаемая земля дальше hit, но ещё в пределах сенсора. При необходимости перенести
   `LaserScanTiming`/timestamp helper из `obstacle_memory.hpp`, сохранив source compatibility
   через include. Materialized result: memory и current-overlay могут построить идентичное
   observation без дублирования XYZ/attitude/timestamp формул.

2. **Открыть focused nearest-solid query в existing known-static classifier.**
   В `KnownStaticLidarHitClassifier` добавить read-only метод, например
   `nearestExpectedSurface(origin, direction, max_range_m)`, возвращающий expected range и
   metadata ближайшего physical solid без применения measured-range policy. Реиспользовать
   текущий ray/AABB код `known_static_lidar_hit_classifier.cpp:73-150`; существующий
   `classify()` переписать поверх этого метода, чтобы не появилось двух реализаций
   intersection. Materialized result: known solids становятся одним из двух реальных
   expected-surface providers, а старые boundary semantics остаются покрыты прежними tests.

3. **Добавить pure ground-plane geometry и единый per-beam decision API.**
   В `lidar_ingestion_decision.hpp/.cpp` определить:

   ```cpp
   struct GroundLidarRejectionConfig {
     bool enabled{true};
     double ground_altitude_m{0.05};
     double closer_tolerance_m{0.5};
     double farther_tolerance_m{1.5};
   };

   enum class LidarIngestionAction {
     kApplyFreeAndHit,
     kApplyFreeOnly,
     kSuppressAll,
   };

   enum class ExpectedSurfaceKind { kNone, kKnownStatic, kGround };
   ```

   Pure ground query должен принимать map-frame origin/direction и effective sensor range:

   ```text
   if direction.z < -epsilon:
       t_ground = (ground_z - origin.z) / direction.z
       candidate exists iff finite(t_ground) && 0 < t_ground <= effective_max_range
   ```

   Decision query получает task-001 observation, hit/no-return, ground config и optional
   known-static classifier, выбирает минимальный positive expected range и только затем
   применяет tolerances выбранного provider. Selection не должен зависеть от порядка вызова
   providers. При практически равных ranges результат маркируется deterministic ambiguity и
   получает conservative `kSuppressAll`, а tie epsilon фиксируется и тестируется.
   Materialized result: одна чистая функция полностью задаёт surface selection,
   classification, action, expected range и delta для обоих ingestion paths.

4. **Зафиксировать update policy, не расширяя scope known-static поведения.**
   Реализовать следующие решения в shared decision:

   ```text
   no reachable expected surface:
       hit -> ApplyFreeAndHit
       no-return -> ApplyFreeOnly

   selected ground:
       measured < expected - closer_tolerance -> ApplyFreeAndHit
       measured within asymmetric tolerances  -> SuppressAll (expected ground)
       measured > expected + farther_tolerance -> SuppressAll (ambiguous)
       no-return past reachable ground         -> SuppressAll (ambiguous)

   selected known solid:
       measured hit -> existing expected/unexpected/ambiguous known-static policy
       no-return    -> preserve current free-only behavior
   ```

   Invalid/missing altitude, non-finite/non-normalized 3D direction, invalid source attitude
   при включённой attitude compensation, absent decision object или invalid ground config
   дают explicit `classification_unavailable`, но action остаётся legacy fail-open
   (`hit -> free+hit`, `no-return -> free-only`). Horizontal/upward beam и ground за пределом
   effective range считаются ordinary no-ground, а не unavailable. `min_projected_*` остаётся
   независимым coarse non-mutating rejection. Materialized result: реальный объект перед
   землёй сохраняется, expected/ambiguous ground ничего не очищает и не занимает, invalid
   input не отключает lidar.

5. **Переставить decision перед `applyMiss()`/`applyAcceptedHit()` в obstacle memory.**
   Изменить `ObstacleMemoryGrid::integrateScan()` в
   `drone_city_nav/src/obstacle_memory.cpp:224-350`: после projection/observation и до
   `clipSegmentToGrid()`/`cellsOnLine()` вычислять decision. Для `kSuppressAll` немедленно
   переходить к следующему beam; для `kApplyFreeOnly` выполнять существующий miss path без
   endpoint hit; для `kApplyFreeAndHit` сохранить текущие miss + scoring semantics.
   Altitude-rejected projection по-прежнему не мутирует memory. Rejected ground не вызывает
   `applyAcceptedHit()` и не входит в `activeProvenance()`. Materialized result: regression
   `expected/ambiguous ground cannot clear pre-populated cell` становится истинным именно по
   production call order.

6. **Подключить тот же decision к current-lidar overlay.**
   Расширить `LidarScanView` в `current_lidar_overlay.hpp` timing/effective-range данными,
   заполнить их из `PlannerNode::onScan()` (`planner_node_inputs.cpp:100-140`) и передать
   shared decision context в `currentLidarOverlay()` до `markOccupied()`.
   `kSuppressAll` и `kApplyFreeOnly` не создают overlay endpoint; closer unknown obstacle
   остаётся occupied. Planner wrapper в `planner_node_lidar_overlay.cpp:47-98` должен
   переносить все bounded decision diagnostics, а не только scalar counters. Materialized
   result: одинаковый projected observation/config даёт одинаковую ground/known-surface
   classification в memory и overlay.

7. **Добавить bounded diagnostics, не смешивая rejected evidence с task-001 state.**
   В общую stats structure добавить counters минимум для:

   - expected ground suppressed;
   - closer unknown obstacles retained;
   - ambiguous ground-facing beams suppressed;
   - classification unavailable/fail-open.

   Для каждого класса хранить максимум фиксированное число (рекомендуется 8-16) first
   samples: task-001 observation, selected surface, measured/expected/delta, endpoint XYZ,
   roll/pitch/tilt, beam index и reason. Эти samples живут только в result stats/logs и не
   сериализуются как active memory provenance. Для accepted closer blockers добавить
   компактный decision snapshot к `AcceptedObstacleMemoryHit`; если он передаётся через
   task-001 ROS provenance, обновить `ObstacleMemoryHitObservation.msg`, schema version,
   conversion/parser и round-trip/invalid-schema tests атомарно. Materialized result:
   rejected decisions аудитируются bounded summary, а accepted blocker после реплана можно
   связать с ground decision и конечным XYZ без ложного provenance для отброшенного hit.

8. **Провести ground config через оба node с единым validation contract.**
   Добавить в обе ROS parameter sections `urban_mvp.yaml`:

   ```yaml
   ground_lidar_rejection_enabled: true
   ground_altitude_m: 0.05
   ground_lidar_hit_closer_range_tolerance_m: 0.5
   ground_lidar_hit_farther_range_tolerance_m: 1.5
   ```

   В `obstacle_memory_node.cpp` и planner config/state/lifecycle объявить, validate и
   залогировать requested/effective values. Enabled config валиден только при finite ground
   altitude, finite non-negative tolerances и finite positive max range. Явно invalid config
   переводит classifier в `unavailable/fail-open` с WARN, а не молча подменяется permissive
   tolerance. Обе node создают decision context до scan subscriptions; текущий startup reset
   memory достаточен, так как dynamic parameter update в scope не добавляется. Materialized
   result: config drift виден в логах, а default соответствует верхней поверхности текущего
   ground collision.

9. **Интегрировать summary и blocker diagnostics в ROS logs.**
   В `obstacle_memory_node.cpp` добавить throttled aggregate summary и bounded first-sample
   lines; в planner lifecycle — effective config; в `planner_node_lidar_overlay.cpp` —
   накопление current-scan stats; в `planner_node_runtime.cpp:126-183` — вывод принятого
   blocker decision рядом с task-001 exact-stamp provenance. Не логировать каждый beam.
   Отдельно устранить существующую потерю `retained_known_static_hits` при копировании overlay
   stats, чтобы shared nearest-surface diagnostics действительно доходили до replan log.
   Materialized result: для каждого headless run можно посчитать suppression/ambiguity и
   доказать происхождение любого accepted passage blocker.

10. **Добавить portable automated tests и CMake target.**
    Создать `drone_city_nav/tests/lidar_ingestion_decision_test.cpp` и подключить его к
    `drone_city_nav/CMakeLists.txt`. Покрыть pure geometry, tolerance boundaries, no-return,
    invalid input и перестановки ground/solid providers. Расширить:

    - `lidar_projection_test.cpp` — observation inputs для tilted/downward rays;
    - `obstacle_memory_test.cpp` — pre-populated cells не очищаются expected/ambiguous ground;
      closer obstacle сохраняет miss/hit score и finite-Z accepted provenance;
    - `current_lidar_overlay_test.cpp` — expected/ambiguous ground отсутствует, closer hit есть;
    - `known_static_lidar_hit_classifier_test.cpp` — nearest expected-surface query;
    - `known_static_lidar_ingestion_test.cpp` — solid-before-ground и ground-before-solid,
      одинаковый результат обоих ingestion paths;
    - `known_static_lidar_planning_grid_integration_test.cpp` — false ground не попадает в raw
      evidence/inflated prohibited grid, closer real obstacle попадает.

    Existing test, который ожидает free clearing для expected known-static hit, оставить
    зелёным: запрет clearing относится к selected ground, а не молча меняет legacy solid
    contract. Materialized result: все 10 обязательных test categories из task prompt имеют
    deterministic automated coverage.

11. **Обновить script/config contracts и headless validator.**
    В `scripts/tests/test_topic_contract.py` потребовать ровно две согласованные копии ground
    params/defaults, отсутствие global tilt cutoff и отсутствие duplicate ground-intersection
    formula в production/config search (с исключением `tasks/`). В
    `scripts/validate_drone_nav_headless.py` сначала исправить stale closer/farther
    known-static regex, затем валидировать effective ground config обеих node и наличие
    aggregate decision counters. Обновить fixtures/negative cases в
    `scripts/tests/test_validate_drone_nav_headless.py`: missing config, mismatch, invalid
    status и valid matching config. Materialized result: headless success не может пройти,
    если один ingestion path работает без ground classifier или с другим ground/tolerance.

12. **Документировать фактическую архитектуру и operational diagnostics на английском.**
    Обновить перечисленные docs: ground как known flat expected surface, nearest-surface
    composition, asymmetric tolerance, no-hit/no-clear policy, fail-open, отличие coarse
    altitude bound, отсутствие tilt cutoff, связь с known upper/lower filtering, task-001
    accepted provenance и bounded rejected diagnostics. В troubleshooting добавить разбор
    `expected_ground`, `ambiguous_ground`, `classification_unavailable`, range delta и
    проверку ground Z против SDF. Materialized result: operator может объяснить suppression
    и accepted blocker без чтения C++.

13. **Закрыть implementation одним коммитом до simulation evidence.**
    После C++ format, targeted tests, full package test, quality и script tests создать один
    локальный commit с ground-rejection implementation; не push. Только после него выполнить
    три независимых headless run. Если анализ потребует только дополнительных diagnostics,
    внести их отдельным coherent commit после повторных automated checks и лишь затем
    перезапустить runs. Materialized result: behavioral change независимо откатывается от
    task `001`, а run evidence соответствует точному commit hash.

## Verification plan

1. До форматирования выполнить scoped production/config searches:

   ```bash
   rg -n --glob '!tasks/**' 'tilt.*(suspend|disable|ignore)|ground.*intersection|ground_altitude_m' \
     drone_city_nav scripts docs
   ```

   Результат должен показывать отсутствие global tilt-based cutoff и единственную production
   ground formula в shared classifier module (повтор формулы допустим только в tests).

2. Отформатировать только изменённые C++ файлы:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

3. Собрать package и запустить targeted tests:

   ```bash
   ./scripts/build.sh
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure \
     -R 'lidar_projection_test|ground_lidar_ingestion_decision_test|obstacle_memory_test|current_lidar_overlay_test|known_static_lidar_hit_classifier_test|known_static_lidar_ingestion_test|known_static_lidar_planning_grid_integration_test|obstacle_memory_provenance'
   ```

4. Из-за изменения public headers, CMake, ROS message/config и двух runtime paths выполнить
   полный package scope и scripts:

   ```bash
   ./scripts/test.sh
   ./scripts/dev_shell.sh make quality
   ./scripts/dev_shell.sh make test-scripts
   ```

5. Проверить `git diff`, отсутствие unrelated/world/control/trajectory edits и clean status,
   затем создать implementation commit. Не запускать simulation на незакоммиченных source
   changes.

6. После commit последовательно выполнить три независимых run с отдельными директориями:

   ```bash
   for run in 01 02 03; do
     DRONE_GAZEBO_LOG_DIR="log/ground_rejection/run_${run}" \
       MISSION_CHECK=1 SMOKE_DURATION_S=180 ./scripts/sim_headless.sh
   done
   ```

7. Для каждого run сохранить в отчёте:

   - exact commit hash и terminal exit status;
   - `MISSION_RESULT`, `actual_passage_openings_seen/known_passage_openings`;
   - число replans и каждую reason/source/blocker cell;
   - expected-ground suppressed, closer-hit retained, ambiguous-ground и unavailable counts
     отдельно для obstacle memory и current overlay;
   - representative high-tilt ground decisions и одновременно non-ground accepted evidence,
     доказывающее отсутствие global lidar suspension;
   - каждый accepted memory/current-lidar blocker около passage с task-001 XYZ, attitude,
     measured/expected/delta и selected surface;
   - подтверждение, что ни один accepted blocker/replan не совпадает с ground range внутри
     configured tolerance.

8. Если run падает по unrelated pre-existing причине, не менять control/trajectory/world:
   сохранить четыре runtime logs, lidar debug snapshots и validator output, явно отделить
   mapping verdict от mission verdict. Tolerance не расширять только ради зелёной миссии;
   сначала сравнить observed delta distribution с ожидаемой геометрией.

## Testing strategy

### 1. Без рефакторинга

Добавить ground check непосредственно в `obstacle_memory.cpp` и
`current_lidar_overlay.cpp`, оставив known-static API как есть. Это минимальный diff, но требует
дублировать ground formula и не позволяет корректно выбрать nearest ground/solid до update.
Также task-001 observation останется доступен только memory path. Стратегия **не рекомендуется**,
потому что нарушает explicit shared decision и order-independence из task contract.

### 2. Лёгкий рефакторинг

Вынести observation factory, открыть nearest-solid query и добавить focused pure
`lidar_ingestion_decision` module. Оба callers остаются владельцами только своих update
operations, а pure module владеет geometry/classification/action. Existing known-static
`classify()` и task-001 provenance переиспользуются. Это **рекомендуемая стратегия**: два
реальных provider оправдывают abstraction, API остаётся узким, а unit tests не требуют ROS или
Gazebo.

### 3. Тяжёлый рефакторинг

Ввести polymorphic expected-surface framework, 3D static scene index, sparse voxel memory или
унифицированный lidar ingestion service для всех node. Это могло бы поддержать terrain и future
3D mapping, но существенно расширяет API/lifetime/performance scope, усложняет ROS ownership и
противоречит явным out-of-scope ограничениям. Стратегия **отклоняется для этой задачи**; future
terrain/voxel work должно быть отдельной задачей после измерения текущего pure API.

## Risks and tradeoffs

1. **Скрытие низкого неизвестного объекта перед землёй.** Слишком широкий closer tolerance
   превратит реальный blocker в ground. Митигация: strict boundary tests, default `0.5 m`,
   accepted closer-hit integration tests и анализ delta distribution без тюнинга ради mission.
2. **Ложное clearing после endpoint suppression.** Главный текущий defect останется, если
   decision будет вызван после ray mutation. Митигация: pre-populated-cell regression для
   expected и ambiguous ground и code review call order до `applyMiss()`.
3. **Расхождение двух ingestion paths.** Memory может стать корректной, а overlay продолжит
   запускать replans. Митигация: один pure API, equality integration tests, одинаковый YAML и
   effective-config validator.
4. **Order-dependent ground/solid selection.** Разные evaluation order скроют объект либо
   выберут неверную tolerance policy. Митигация: nearest expected range + permutation/tie tests.
5. **Изменение legacy known-static semantics.** Общий decision может случайно запретить free
   clearing для upper/lower, хотя scope требует только ground. Митигация: сохранить existing
   expected-solid tests, явно разделить `ApplyFreeOnly` и `SuppressAll`.
6. **Invalid attitude/config отключает mapping.** Митигация: explicit unavailable counter/WARN и
   fail-open action; отдельные tests для invalid altitude, attitude, vector, config и absent
   classifier.
7. **Слишком шумная диагностика и рост памяти.** Per-beam logs недопустимы. Митигация: counters,
   фиксированный first-sample cap и отсутствие rejected ground в persistent provenance.
8. **ROS message/API compatibility.** Добавление decision snapshot в task-001 message требует
   coordinated schema bump и rebuild всех consumers. Митигация: добавить поля только если без
   них accepted blocker нельзя аудитировать после exact-stamp correlation; обновить conversion,
   parser, round-trip tests и docs в одном commit.
9. **Ground Z drift относительно world.** Default `0.05` доказан текущим SDF, но custom world
   может отличаться. Митигация: явный параметр в обеих node, startup log и validator mismatch;
   не извлекать SDF автоматически в этой задаче.
10. **Performance на 720 beams.** Повторный обход всех known solids отдельно от текущего
    classifier увеличит callback cost. Митигация: один nearest-solid query на beam, reuse его
    результата в policy/diagnostics, без virtual provider hierarchy; измерить callback/summary
    timing в headless logs при наличии текущей метрики.
11. **Старые occupied cells.** Новый classifier не может доказательно удалить legacy 2D memory
    по Z. Митигация: не делать selective deletion; startup memory создаётся/reset до scans.
    Runtime dynamic config не вводится. Если он появится позже, reset entire memory должен быть
    отдельным явным policy.

## Open questions

1. **Какой ground altitude использовать по умолчанию?**
   - Recommended decision: `0.05 m`, то есть верхняя поверхность collision box текущего
     `generated_city.sdf`, а не geometric center `0.0 m`.
   - Rationale: ray должен совпадать с первой физически достижимой поверхностью Gazebo.
   - Confirmation needed: pure geometry test и headless measured-minus-expected distribution;
     менять default только при доказанном расхождении collision response.

2. **Какие asymmetric tolerances взять как initial defaults?**
   - Recommended decision: отдельные ground defaults `closer=0.5 m`, `farther=1.5 m`, совпадающие
     с уже обкатанной формой known-static policy, но не связанные с ней одним config field.
   - Rationale: closer остаётся строгим для obstacle-before-ground, farther допускает pose/range
     noise. Независимые поля позволяют позже тюнить ground без изменения upper/lower.
   - Confirmation needed: boundary tests и три headless delta distributions. Не расширять
     tolerances только для достижения mission success.

3. **Что делать при почти равной дальности до ground и known solid?**
   - Recommended decision: deterministic tie epsilon и `ambiguous -> SuppressAll` с обеими
     surface metadata в bounded diagnostic.
   - Rationale: в геометрическом ребре нельзя надёжно доказать unknown obstacle; suppress-all не
     создаёт ни ложный obstacle, ни ложное free clearing.
   - Confirmation needed: permutation test и реальные samples, если tie встречается в headless.

4. **Нужно ли расширять task-001 wire provenance ground decision snapshot?**
   - Recommended decision: да, только для **accepted** hits добавить compact selected-surface,
     action/reason, expected range и delta; rejected ground оставить исключительно в bounded
     scan stats/logs.
   - Rationale: planner replan log должен объяснять, почему blocker был принят как closer unknown,
     даже после exact-stamp передачи между node. Одного endpoint Z недостаточно для этого аудита.
   - Confirmation needed: reviewer должен подтвердить, что существующий ROS message consumer set
     ограничен этим package; затем обязательны schema/round-trip/size tests. Если внешний
     consumer обнаружится, вынести wire extension в отдельный follow-up, а текущий task завершить
     bounded node logs с documented observability gap.

5. **Должен ли invalid configured ground параметр заменяться default?**
   - Recommended decision: нет; объявить ground classifier unavailable и сохранить legacy
     fail-open mapping, одновременно WARN-логировать requested values/status.
   - Rationale: silent fallback маскирует configuration error, а permissive clamp может скрыть
     реальные obstacles. Valid YAML defaults обеспечивают нормальный production path.
   - Confirmation needed: config unit/script tests на NaN/negative tolerance и headless validator
     на `status=ready` в обеих node.

6. **Нужно ли классифицировать beam, уже отклонённый `min_projected_lidar_altitude_m`?**
   - Recommended decision: coarse altitude guard остаётся отдельным более ранним non-mutating
     исходом; ground decision управляет только beams с usable 3D geometry, которые дошли до
     ingestion. Pure classifier отдельно тестирует ground/no-return semantics без зависимости от
     coarse bound.
   - Rationale: это сохраняет существующий safety filter и не смешивает два разных счётчика.
   - Confirmation needed: integration test доказывает, что altitude-rejected beam и ground-
     suppressed beam оба не мутируют grid, но учитываются раздельно.

7. **Нужен ли runtime parameter update/reset memory в этой задаче?**
   - Recommended decision: нет; ground config читается на startup до lidar subscription, а
     existing startup/reset semantics дают чистую memory. Dynamic reconfiguration остаётся вне
     scope.
   - Rationale: selective legacy cleanup запрещён, а runtime parameter callback существенно
     расширит lifecycle и concurrency surface.
   - Confirmation needed: проверить, что ни одна из четырёх ground params не объявлена dynamic в
     текущем node lifecycle; если позже появится callback, full reset policy оформить отдельной
     задачей.

Блокеров для реализации в текущем workspace не обнаружено. Необходимые ray geometry, task-001
observation/provenance, оба ingestion call site, тестовые targets и container workflow уже есть.
