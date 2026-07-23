# План реализации partial replan/repair

## Context

Нужно добавить production-механизм частичного перепланирования для уже
исполняемой траектории. После подтверждения safe truncation все кандидаты
используют одну стабильную точку `A`, вычисленную offboard-узлом, но возвращаются
на старую траекторию в разных точках:

```text
remaining safe prefix -> repaired A-Bn -> old suffix Bn-goal
```

Параллельно запускаются десять partial jobs с reconnect margin от 10 до 100 м и
один существующий full global replan. Побеждает не лучший по качеству, а первый
результат, который прошёл обязательные физические и freshness-проверки.

Принятая проектная граница: «опасный полёт лучше безопасного стояния на месте».
Поэтому planner clearance, passage quality и высокая curvature являются
характеристиками качества, но не должны блокировать физически допустимый
кандидат. Обязательными остаются структурная корректность, отсутствие NaN и
позиционных разрывов, проходимость хотя бы по `runtime_prohibited`, отсутствие
пересечений известных `lower_mass`/`upper_mass` и соблюдение актуального
truncation-контракта.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на прямом исследовании
текущего кода, конфигурации, ROS-сообщений и тестов.

Уже существующая инфраструктура:

- `PlannerNode::keepCurrentPathIfStillClear()` обнаруживает конфликт текущего
  пути и публикует `ReplanBlockerEvent`
  (`drone_city_nav/src/planner_node_runtime.cpp:235`).
- Offboard вычисляет фактически исполнимый truncated prefix и подтверждает
  стабильную точку `A` через `ReplanTruncation`
  (`drone_city_nav/src/px4_offboard_node_replan.cpp:102`).
- Planner хранит generation, fingerprint, `A`, tangent и altitude в
  `TruncationReplanState`
  (`drone_city_nav/src/planner_node.hpp:156`).
- `ExecutableTrajectory` и `TruncationSuffixAck` уже поддерживают
  `MOVING_JOIN`, `AFTER_HOLD`, pending suffix и retry после отказа offboard
  (`drone_city_nav/msg/ExecutableTrajectory.msg`,
  `drone_city_nav/src/planner_node_truncation.cpp:87`).
- Принятая полная траектория доступна как
  `last_valid_trajectory_samples_`
  (`drone_city_nav/src/planner_node.hpp:532`).
- Полный planning pipeline уже имеет ordered fallback
  `planning_clearance -> runtime_prohibited`
  (`drone_city_nav/src/planner_node_inputs.cpp:588`,
  `drone_city_nav/src/trajectory_planner.cpp:437`).
- Перед публикацией уже строятся свежие grids и выполняется повторная
  traversability/handover-проверка
  (`drone_city_nav/src/planner_node_trajectory_publication.cpp:204`).

Недостающие части:

- для prohibited-trigger сейчас сохраняется только первое пересечение, а не
  полный непрерывный `[first_blocked_s, last_blocked_s]`;
- planning worker запускает один полный pipeline и читает mutable state
  `PlannerNode`, то есть готового immutable `RepairSnapshot` нет;
- geometry-only pipeline `A -> B` и повторная глобальная финализация stitched
  trajectory не выделены в самостоятельные API;
- optimizer не принимает cancellation token;
- нет race coordinator/arbiter и единого контракта результата repair job.

## Detected stack/profiles

- Основной стек: C++20, ROS 2, `ament_cmake`, `colcon`, Gazebo/PX4.
- Сборка и тесты выполняются только в dev-container через репозиторные scripts и
  `Makefile`.
- Прочитаны и применены:
  - общий profile
    `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`;
  - C/C++ profile
    `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`;
  - `AGENTS.md`, `CPP_BEST_PRACTICES.md`, `README.md`, `CONTRIBUTING.md`.
- Rust profile не применяется: затрагиваемый workspace и планируемые изменения
  относятся к C++/ROS 2 package.
- Действует source-size contract: tracked C/C++ source не должен превышать 1000
  строк. `planner_node_inputs.cpp` уже содержит 998 строк, поэтому race и repair
  orchestration должны размещаться в новых `.cpp/.hpp`, а не добавляться в этот
  файл.

## Repo-approved commands found

- `./scripts/build.sh` или `./scripts/dev_shell.sh make build` — container build.
- `./scripts/test.sh` или `./scripts/dev_shell.sh make test` — полный CTest package.
- `./scripts/dev_shell.sh make test-scripts` — Python contract tests, включая
  source-size и entrypoint checks.
- `./scripts/dev_shell.sh make format` — форматирование изменённых C++ файлов.
- `./scripts/dev_shell.sh make quality` — обязательная перед commit комплексная
  C++ quality-проверка.
- `ENABLE_STATIC_MAP=false MISSION_CHECK=... ./scripts/sim_headless.sh` —
  headless no-static integration run; конкретное значение `MISSION_CHECK`
  выбирается по сценарию acceptance, а не выдумывается в unit-test плане.

## Affected components

- `planner_core`: поиск полного blocked span и выбор endpoint `Bn`.
- Safe truncation state в `PlannerNode`: фиксация old trajectory artifact и
  immutable repair context до ожидания подтверждения offboard.
- `trajectory_planner`, `corridor`, `trajectory_optimizer`, `turn_smoothing`:
  переиспользуемый geometry-only pipeline и cooperative cancellation.
- Новый модуль trajectory repair: sampling `A/B`, stitching, повторная
  глобальная metadata/profile finalization и hard validation.
- Новый race coordinator: 11 внешних однопоточных jobs, completion mailbox,
  first-hard-valid arbiter и cancellation.
- Planner publication path: короткая свежая проверка winner и ровно одна
  публикация.
- Planner config/YAML: включение фичи, reconnect margins и контролируемый worker
  budget.
- Диагностика и документация replanning pipeline.
- Unit/integration tests и CMake registration.

## Implementation steps

1. **Добавить явные core-типы blocked span и immutable repair artifact.**

   Файлы:

   - `drone_city_nav/include/drone_city_nav/planner_core.hpp`
   - `drone_city_nav/src/planner_core.cpp`
   - новый `drone_city_nav/include/drone_city_nav/trajectory_repair.hpp`

   Code anchors:

   - `PathProhibitedIntersection`
   - `firstPathProhibitedIntersection()`
   - `PlannerCore::evaluateStablePath()`

   Добавить `BlockedSpan` с абсолютными stations старой executable trajectory,
   индексами/точками входа и выхода и причиной (`prohibited` или
   `raw_clearance`). Для prohibited grid пройти первый непрерывный конфликтующий
   участок до первого устойчивого выхода, вместо остановки на первой клетке. Для
   raw-clearance trigger преобразовать существующие `entry_distance_m` и
   `length_m` в тот же контракт.

   Добавить `ExecutableTrajectoryArtifact`, который хранит immutable копию
   принятого `path_id`, полных `TrajectoryPointSample`, mission goal и
   fingerprint геометрии. Результат шага: оба trigger-а дают единый
   `[first_blocked_s, last_blocked_s]`, а repair не зависит от мутирующего
   `last_valid_path_points_`.

2. **Зафиксировать repair context в момент trigger-а и связать его с
   подтверждённой точкой `A`.**

   Файлы:

   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_runtime.cpp`
   - `drone_city_nav/src/planner_node_truncation.cpp`
   - новый `drone_city_nav/src/planner_node_repair.cpp`

   Code anchors:

   - `PlannerNode::beginTruncationReplan()`
   - `PlannerNode::onReplanTruncation()`
   - `TruncationReplanState`

   Расширить внутреннее состояние, не ROS message: при создании generation
   сохранить blocked span и `ExecutableTrajectoryArtifact`, соответствующий
   `blocked_path_id`. После `ReplanTruncation` спроецировать подтверждённую
   offboard-точку на этот artifact и проверить, что `A` не находится после
   `first_blocked_s`, а fingerprint/generation совпадают.

   ```cpp
   struct PendingRepairContext {
     TruncationReplanState truncation;
     BlockedSpan blocked_span;
     ExecutableTrajectoryArtifact old_trajectory;
     double truncation_s_m;
   };
   ```

   Если old artifact уже заменён или `A` нельзя однозначно сопоставить с ним,
   partial race не запускается; существующий full replan остаётся fallback.
   Результат шага: `A`, blocked span и old suffix относятся к одной версии
   принятого пути.

3. **Добавить конфигурацию repair race и построение одного
   `RepairSnapshot`.**

   Файлы:

   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - новый `drone_city_nav/include/drone_city_nav/repair_race.hpp`

   Code anchors:

   - `loadPlannerNodeConfig()`
   - `PlannerNodeConfig`
   - `PlanningGridBuildResult`

   Добавить:

   ```yaml
   partial_replan_enabled: true
   partial_replan_reconnect_margins_m: [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
   partial_replan_internal_parallel_workers: 1
   ```

   Валидировать возрастающие конечные положительные margins и принудительно
   ограничить внутренний parallelism значением `1` для corridor и optimizer в
   каждом race job. Внешняя гонка запускает все 11 jobs сразу, как явно
   потребовано.

   `RepairSnapshot` создавать один раз после подтверждения truncation и
   передавать jobs как `std::shared_ptr<const RepairSnapshot>`. Вместо одного
   неоднозначного `clearance` хранить два согласованных grid candidates, у
   каждого собственный grid fingerprint и `ClearanceField2D`:

   ```cpp
   struct RepairGridSnapshot {
     std::string name;
     OccupancyGrid2D grid;
     ClearanceField2D clearance;
   };
   ```

   Snapshot также содержит copied `KnownPassageMap`, config, generation,
   blocked path/fingerprint, old artifact, `A`, `current_s`, `truncation_s` и
   blocked span. Ни один job не обращается к mutable полям `PlannerNode`.

4. **Выделить отменяемый geometry-only trajectory pipeline.**

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
   - `drone_city_nav/src/trajectory_planner.cpp`
   - при необходимости новый
     `drone_city_nav/src/trajectory_planner_geometry.cpp`
   - `drone_city_nav/include/drone_city_nav/trajectory_optimizer.hpp`
   - `drone_city_nav/src/trajectory_optimizer.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:

   - `planOptimizedTrajectory()`
   - `optimizeTrajectory()`
   - optimizer iteration loop
     (`drone_city_nav/src/trajectory_optimizer.cpp:162`)
   - `applyVerticalProfileStage()`
   - `finalizeResult()`

   Разделить существующую orchestration на:

   - `planTrajectoryGeometry(A, B, grids, config, stop_token)`:
     A*, corridor, optimizer, turn smoothing и shape cleanup только для нового
     участка;
   - `finalizeStitchedTrajectory(samples, passages, grids, config)`:
     глобальный пересчёт geometry metadata, vertical profile, passage metadata,
     speed profile и hard validation.

   Математику A*, corridor, optimizer и smoothing не менять. В
   `optimizeTrajectory()` добавить `std::stop_token`/cancellation callback и
   проверять его между iterations; также проверять cancellation после A*, после
   corridor и до финальной validation. В race-конфиге задать
   `corridor.parallel_workers=1` и
   `trajectory_optimizer.parallel_workers=1`.

   Результат шага: один job не порождает до 16 внутренних workers и может
   кооперативно завершиться после выбора winner.

5. **Реализовать выбор `Bn`, partial A* и stitching без перестройки old
   suffix corridor.**

   Файлы:

   - новый `drone_city_nav/src/trajectory_repair.cpp`
   - `drone_city_nav/include/drone_city_nav/trajectory_repair.hpp`
   - существующие geometry helpers из
     `drone_city_nav/include/drone_city_nav/trajectory.hpp`

   Для margin `N` вычислять:

   ```text
   reconnect_s = blocked_span.last_blocked_s + N
   Bn = sample(old_trajectory, reconnect_s)
   ```

   Candidate не запускать, если `Bn` вышел за конец old trajectory либо
   запрещён во всех допустимых grids. Если `Bn` prohibited только в
   `planning_clearance`, но разрешён в `runtime_prohibited`, запускать job на
   более слабом grid, потому что `blocked` здесь означает именно запрет
   планирования, а не обязательно физическую стену.

   Для geometry:

   1. A* от подтверждённого `A` до точного `Bn`;
   2. corridor/optimizer/smoothing только участка `A-Bn`;
   3. удалить дублирующий sample `Bn`;
   4. присоединить неизменённую XY-геометрию old suffix `Bn-goal`;
   5. заново заполнить stations, tangent/curvature и очистить устаревшие
      passage/vertical/speed metadata;
   6. выполнить глобальную finalization из шага 4.

   Позиционный стык `Bn` обязателен. Плохие tangent/curvature являются
   degraded quality и передаются speed profile, а не автоматическим reject.
   Пересечение `runtime_prohibited`, NaN/разрыв или known solid остаются hard
   reject.

   `Bn` внутри known passage hard-window не переносить безусловно. Такой стык
   допустим только если полная stitched trajectory сохраняет непрерывную
   геометрию прохода, успешно получает новый vertical profile и проходит
   mandatory known-solid validation. Если короткий вариант этого не делает,
   его отклоняет finalization, а более длинный `Bn` естественно оказывается
   после окна.

6. **Поддержать `MOVING_JOIN` и `AFTER_HOLD` для каждого partial
   candidate.**

   Файлы:

   - `drone_city_nav/src/trajectory_repair.cpp`
   - `drone_city_nav/src/planner_node_route_publication.cpp`
   - `drone_city_nav/src/planner_node_truncation.cpp`

   Code anchors:

   - `publishPathFromPathCells()`
   - существующая логика `PassageInsertionStartMode`
   - `deliverTruncationSuffix()` и ACK handling

   Сначала строить moving-вариант с tangent из truncation confirmation. Если
   начальный стык/vertical transition у `A` требует остановки, повторить job с
   существующей семантикой `AFTER_HOLD`, без требования продолжать летящий
   tangent и с уже реализованным vertical pre-alignment. Ограничения на стык
   `Bn` не снимаются семантикой `AFTER_HOLD`, потому что остановка происходит
   только в `A`.

   Если winner готов до достижения `A`, offboard использует существующее
   сшивание remaining truncated prefix с suffix. Если дрон уже достиг hold,
   winner помечается `AFTER_HOLD` и активируется через существующий pending/ACK
   protocol. Новые ROS message fields не требуются.

7. **Реализовать внешнюю гонку 10 partial jobs + 1 full job и first-valid
   arbiter.**

   Файлы:

   - `drone_city_nav/include/drone_city_nav/repair_race.hpp`
   - `drone_city_nav/src/repair_race.cpp`
   - `drone_city_nav/src/planner_node_repair.cpp`
   - `drone_city_nav/src/planner_node_refinement.cpp`
   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:

   - `PlannerNode::planningWorkerLoop()`
   - `PlannerNode::runPlanningCycle()`

   Создать один `std::stop_source`, 11 `std::jthread` и thread-safe completion
   mailbox. Каждый job имеет собственный `PlannerCore`/clearance cache и читает
   только shared immutable snapshot. Full job использует тот же `A` и
   существующий полный pipeline `A -> mission goal`, но также с внутренними
   workers `1`.

   Race thread не вызывает ROS publisher. Он кладёт `RepairResult` в mailbox.
   Planner worker извлекает результаты в порядке завершения:

   ```cpp
   while (auto result = mailbox.next()) {
     if (!snapshotHardValid(*result)) {
       continue;
     }
     if (!freshHardValid(*result)) {
       continue;
     }
     if (!winner_selected.exchange(true)) {
       stop_source.request_stop();
       publish(*result);
       break;
     }
   }
   ```

   Первый завершившийся invalid/stale candidate не закрывает гонку. Winner
   выбирается только после snapshot и fresh validation. После выбора остальные
   результаты игнорируются по generation даже если job завершает
   неотменяемый участок. В initial planning и replan без подтверждённого safe
   truncation сохранить текущий последовательный full pipeline.

8. **Вынести свежую winner-validation из publication path и запретить двойную
   публикацию.**

   Файлы:

   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`
   - новый `drone_city_nav/src/planner_node_repair_publication.cpp`
   - `drone_city_nav/src/planner_node_truncation.cpp`

   Code anchors:

   - `PlannerNode::publishTrajectoryResult()`
   - `PlannerNode::onTruncationSuffixAck()`

   Переиспользовать существующую свежую сборку planning/prohibited grids и
   final traversability check, но перед публикацией дополнительно проверить:

   - активны те же truncation generation и blocked path;
   - fingerprint temporary prefix совпадает;
   - old path ещё не заменён;
   - prefix до `A` не получил новый hard blocker;
   - candidate проходит свежий `runtime_prohibited` и mandatory known-solid
     validation.

   Не отвергать candidate только из-за численного изменения grid revision:
   критической новизной считать изменение, которое фактически инвалидирует
   prefix или candidate на свежем grid. Это не замораживает гонку при каждом
   новом lidar snapshot.

   Публиковать ровно один suffix path на generation и затем ждать существующий
   ACK. При `REJECTED` не восстанавливать поздние результаты старой гонки:
   запросить новый planning cycle с новым свежим snapshot для той же актуальной
   truncation task. При `PENDING` сохранить текущую ACK-семантику.

9. **Добавить полную runtime-диагностику race без изменения алгоритмического
   решения.**

   Файлы:

   - `drone_city_nav/src/planner_node_repair.cpp`
   - `drone_city_nav/src/repair_race.cpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`
   - `docs/diagnostics.md`

   Логировать:

   - race generation, blocked path/fingerprint, span, `A`, grid fingerprints;
   - для каждого job: kind, reconnect margin, `B`, выбранный grid, durations
     A*/corridor/optimizer/stitch/finalization, activation mode;
   - причины skip/reject: `beyond_goal`, `endpoint_prohibited`,
     `astar_failed`, `non_traversable`, `known_solid_intersection`,
     `stale_generation`, `prefix_invalidated`, `canceled`;
   - порядок completion, winner kind/margin, blocker-to-winner и
     winner-to-publication latency;
   - aggregate summary, если все partial и full job не дали winner.

   Cancellation логировать отдельно от invalid result, чтобы по логам можно было
   отличить проигравший job от алгоритмического failure.

10. **Добавить category-1 автотесты и зарегистрировать новые source/test
    targets.**

    Файлы:

    - новый `drone_city_nav/tests/trajectory_repair_test.cpp`
    - новый `drone_city_nav/tests/repair_race_test.cpp`
    - `drone_city_nav/tests/planner_core_test.cpp`
    - `drone_city_nav/tests/truncation_suffix_protocol_test.cpp`
    - `drone_city_nav/tests/planner_node_config_test.cpp`
    - `drone_city_nav/CMakeLists.txt`

    Материализуемый результат: детерминированные unit/component tests для
    span detection, B generation, stitching/finalization, arbiter,
    cancellation и config; существующий ACK protocol остаётся совместимым.
    Конкретная матрица приведена в разделе `Testing strategy`.

11. **Обновить архитектурную документацию и эксплуатационный контракт.**

    Файлы:

    - `docs/replanning.md`
    - `docs/navigation_pipeline.md`
    - `docs/configuration.md`
    - `docs/diagnostics.md`

    Описать отличие partial repair от safe truncation и full replan, first-valid
    semantics, 11-thread external budget, обязательные hard gates, advisory
    quality, `MOVING_JOIN`/`AFTER_HOLD`, cancellation и поведение при отсутствии
    winner.

## Verification plan

### Mandatory local verification

1. `./scripts/dev_shell.sh make format`

   Форматирует изменённые C++ файлы до проверок; нужен из-за добавления новых
   headers/sources и изменения публичных signatures.

2. `./scripts/dev_shell.sh make build`

   Проверяет C++/ROS message/CMake link contract всего package. Меньшего target
   недостаточно: новые core API используются planner node, trajectory planner и
   tests одновременно.

3. Точный CTest-набор после build:

   ```bash
   ./scripts/dev_shell.sh \
     ctest --test-dir build/drone_city_nav --output-on-failure \
     -R '^(trajectory_repair_test|repair_race_test|planner_core_test|trajectory_planner_test|trajectory_optimizer_test|safe_trajectory_truncation_test|truncation_suffix_protocol_test|planner_node_config_test)$'
   ```

   Он проверяет новый repair/race контракт и существующие границы, которые
   меняются: span detection, optimizer cancellation, truncation ACK, config и
   trajectory finalization.

4. `./scripts/dev_shell.sh make test-scripts`

   Обязателен из-за новых CMake sources и риска превысить 1000 строк в уже
   крупных planner files; также проверяет repository entrypoint contracts.

5. `./scripts/dev_shell.sh make quality`

   Финальный обязательный repo gate перед commit. Здесь широкая проверка
   оправдана изменением shared C++ API, concurrency и trajectory behavior:
   точечный CTest не обнаружит format/static-analysis/source-size и случайные
   regressions в остальных consumers.

### Optional/CI verification

- Выполнить три no-static headless run через repo entrypoint с
  `ENABLE_STATIC_MAP=false` и подходящим mission validator после прохождения
  unit/component tests. Проверить, что при каждом blocker race стартует один
  раз, публикуется не более одного winner, suffix получает ACK и миссия не
  зависает в temporary hold.
- Выполнить один static-map headless regression run, чтобы убедиться, что
  initial planning и обычный full replan не изменились.
- Снять CPU/thread telemetry: одновременно должно быть около 11 внешних jobs,
  а `parallel_workers_used=1` внутри каждого corridor/optimizer. Это проверяет
  главный performance contract, который unit test доказывает только
  конфигурационно.

Точные `MISSION_CHECK` и duration для runtime acceptance следует выбрать по
актуальному headless сценарию команды запуска. План не фиксирует выдуманное
значение validator mode.

## Testing strategy

### Категория 1: обязательные тесты без дополнительного рефакторинга

1. `planner_core_test`:
   - одно непрерывное prohibited-пересечение даёт точные first/last stations;
   - два раздельных span выбирают первый;
   - конфликт, начинающийся в escape-префиксе, не теряет фактический конец;
   - raw-clearance entry/length преобразуются в тот же `BlockedSpan`.

2. `trajectory_repair_test` — B generation:
   - создаются `B10...B100` от `last_blocked_s`, а `A` у всех одинаков;
   - `Bn` после mission goal пропускается;
   - `Bn`, запрещённый в planning grid, но свободный в runtime grid, остаётся
     кандидатом;
   - `Bn`, запрещённый в обоих grids, пропускается;
   - `Bn` внутри hard-window не переносится автоматически и принимается только
     после успешной full stitched validation;
   - short candidate отклоняется из-за второго конфликта в old suffix, более
     длинный candidate может обойти оба.

3. `trajectory_repair_test` — stitching/finalization:
   - `A` и `B` совпадают по позиции, duplicate samples удалены, stations строго
     возрастают, NaN/разрывов нет;
   - XY-геометрия old suffix после `B` не меняется;
   - curvature, passage metadata, vertical profile и speed profile
     пересчитаны для всей stitched trajectory;
   - known `lower_mass`/`upper_mass` intersection отклоняет candidate;
   - degraded planner clearance/passage quality/curvature не отклоняют
     runtime-traversable и solid-clear candidate;
   - moving join и `AFTER_HOLD` получают корректный activation mode.

4. `repair_race_test` с fake jobs/mailbox:
   - первый завершившийся invalid result игнорируется, первый hard-valid
     выигрывает;
   - при одновременном completion winner выбирается атомарно один раз;
   - stale generation/path/fingerprint отклоняются;
   - candidate, ставший invalid на свежем grid, не закрывает гонку, следующий
     valid result может выиграть;
   - winner вызывает `request_stop()`, optimizer замечает stop между iterations;
   - все partial failures не мешают full job победить;
   - failure всех 11 jobs оставляет temporary hold и публикует aggregate reason.

5. `planner_node_config_test`:
   - default margins равны 10...100;
   - отрицательные, NaN, дубликаты и неупорядоченные margins отклоняются или
     нормализуются по документированному контракту;
   - internal workers в race config равны `1`.

6. `truncation_suffix_protocol_test`:
   - winner до `A` проходит `MOVING_JOIN`;
   - winner после достижения hold проходит `AFTER_HOLD`;
   - `PENDING`, `ACCEPTED`, `REJECTED` не допускают второй публикации той же
     generation;
   - ACK reject приводит к новой planning task, а не к публикации stale late
     result.

### Неавтоматизированные gaps

- Реальное распределение wall-clock latency и CPU contention 11 jobs зависит от
  ROS/Gazebo host load; это проверяется headless telemetry, а не
  детерминированным unit test.
- Физическое качество degraded curvature проверяется существующим speed profile
  unit coverage и headless полётом. Отдельный hardware/PX4-in-the-loop тест в
  текущем workspace отсутствует.

## Risks and tradeoffs

- **CPU и memory pressure.** Один snapshot содержит крупные grids, но он должен
  быть shared immutable, а не копироваться 11 раз. Локальные caches и
  промежуточные trajectories всё равно умножаются на число jobs. Ограничение
  внутренних workers значением `1` обязательно; telemetry должна подтвердить,
  что внешняя гонка быстрее последовательного full replan.
- **Сложность refactor trajectory planner.** Выделение geometry-only и
  finalization API затрагивает shared pipeline. Нельзя копировать существующую
  orchestration в repair module: это создаст расходящиеся правила vertical,
  passage и speed validation.
- **Blocked-span неоднозначность.** Grid conflict может состоять из нескольких
  соседних/раздельных кластеров. Контракт намеренно берёт первый непрерывный
  span; последующие пересечения ловит full final validation, после чего более
  длинный partial или full job остаётся в гонке.
- **Grid revision churn.** Строгое равенство revision сделает winner почти
  невозможным при lidar updates. Поэтому freshness определяется повторной
  фактической проверкой prefix/candidate на свежем grid, а revision используется
  для диагностики и обнаружения смены контекста.
- **Join внутри passage.** Автоматический перенос `B` за hard-window может
  необоснованно удлинить repair. Разрешение внутреннего `B` безопасно только при
  full vertical/solid validation stitched trajectory.
- **Degraded curvature.** Разрешение неидеальной curvature соответствует
  принятой идеологии, но требует корректного глобального speed profile.
  Позиционный разрыв, structural invalidity и solid intersection ослаблять
  нельзя.
- **Cancellation latency.** A* и corridor прерываются только после завершения
  стадии; optimizer — между iterations. После выбора winner некоторые threads
  кратковременно продолжат работу, но publication guard обязан отбросить их
  результаты.
- **Source-size contract.** `planner_node_inputs.cpp` уже почти достиг лимита.
  Новую orchestration нужно вынести в отдельные compilation units.
- **Offboard rejection после planner winner.** Planner hard-valid не гарантирует
  принятие suffix offboard из-за более свежего execution state. Существующий
  ACK остаётся источником истины; rejection запускает новую task.

## Open questions

1. **Нужно ли расширять ROS message `ReplanBlockerEvent` полем
   `last_blocked_s`?**

   Recommended decision: нет. Blocked span нужен planner-у для repair jobs и
   должен храниться во внутреннем generation context. Offboard для safe
   truncation по-прежнему достаточно первой blocker station. Это сохраняет
   существующий ROS-контракт. Подтверждение: component test должен доказать, что
   span не теряется между `beginTruncationReplan()` и
   `onReplanTruncation()`.

2. **Следует ли переносить `Bn`, попавший внутрь known passage hard-window, за
   выход из окна?**

   Recommended decision: нет, не безусловно. Точный positional join внутри
   окна допустим, потому что old suffix содержит продолжение того же passage.
   Candidate проходит только после полного пересчёта vertical metadata и
   mandatory known-solid validation. Если это не удаётся, job отклоняется, а
   следующие margins дают более дальний `B`. Подтверждение: dedicated tests со
   стыком внутри и с нарушенным Z-профилем.

3. **Что означает «grid revision не стала критически новее»?**

   Recommended decision: не вводить произвольный числовой lag threshold.
   Критической считать revision, на которой fresh `runtime_prohibited` или
   known-solid validation инвалидирует prefix/candidate, либо изменился
   blocked path/generation/fingerprint. Иначе непрерывные lidar revisions будут
   отвергать полезные результаты без геометрической причины. Подтверждение:
   race test с новой revision, но неизменной проходимостью, и test с новым
   blocker на candidate.

4. **Нужно ли повторно запускать passage insertion после stitching?**

   Recommended decision: нет в первой реализации. Новый corridor строится
   только для `A-B`, old suffix сохраняется, а для полной stitched trajectory
   пересчитываются passage matching/vertical profile и mandatory solid
   validation. Повторная insertion может изменить old suffix и нарушить
   заявленный scope partial repair. Если headless logs покажут систематические
   `opening_volume_miss` именно на repaired span при solid-clear траектории,
   локальную insertion можно добавить отдельным этапом. Подтверждение:
   category-1 tests и runtime diagnostics passage quality.

5. **Нужно ли принимать позже пришедший более качественный full result после
   публикации partial winner?**

   Recommended decision: нет. На generation публикуется только первый
   hard-valid winner. Поздняя замена снова создаст handover и дёрганье, а также
   нарушит явно заданный приоритет latency над качеством. Новый full replan
   возможен только как отдельная generation при новом blocker или ACK failure.

6. **Что делать, если 11 одновременных single-thread jobs всё равно перегружают
   host?**

   Recommended decision: сначала реализовать требуемый внешний параллелизм и
   измерить stage latency/CPU. Не сокращать число одновременно запущенных jobs
   без данных. Если telemetry докажет регрессию wall-clock, вынести bounded pool
   в отдельное follow-up изменение, сохранив тот же arbiter и порядок margins.
   Для подтверждения нужны headless CPU/thread metrics на целевом host.

7. **Нужно ли запускать partial race без подтверждённой offboard truncation
   point?**

   Recommended decision: нет. Общий стабильный `A` — ключевой контракт фичи.
   При disabled safe truncation, invalid confirmation или потерянном artifact
   planner использует существующий full replan от доступного planning start.
   Это не сокращение scope, а защита от partial candidates с разными или уже
   недостижимыми anchors.
