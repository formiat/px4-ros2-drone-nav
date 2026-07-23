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
   - `drone_city_nav/include/drone_city_nav/path_raw_clearance_monitor.hpp`
   - `drone_city_nav/src/path_raw_clearance_monitor.cpp`
   - `drone_city_nav/include/drone_city_nav/trajectory.hpp`
   - `drone_city_nav/src/trajectory.cpp`
   - новый `drone_city_nav/include/drone_city_nav/trajectory_repair.hpp`

   Code anchors:

   - `PathProhibitedIntersection`
   - `firstPathProhibitedIntersection()`
   - `PlannerCore::evaluateStablePath()`
   - `evaluatePathRawClearance()`
     (`drone_city_nav/src/path_raw_clearance_monitor.cpp:57`)
   - `projectOnTrajectorySamples()`
     (`drone_city_nav/src/trajectory.cpp:562`)

   Добавить `BlockedSpan` с абсолютными stations старой executable trajectory,
   индексами/точками входа и выхода и причиной (`prohibited` или
   `raw_clearance`). Station frame всегда начинается с `samples.front().s_m=0`
   принятого `ExecutableTrajectoryArtifact`; относительные distances
   `decision.remaining_path` в этот тип не попадают.

   До использования projection в repair progress исправить общий контракт
   `minimum_s_m` у обоих публичных helper-ов:
   `projectOnTrajectory()` и `projectOnTrajectorySamples()`. Текущая реализация
   (`drone_city_nav/src/trajectory.cpp:510` и
   `drone_city_nav/src/trajectory.cpp:583`) зажимает `t=1` у сегмента,
   полностью лежащего до lower bound, но всё равно оставляет этот сегмент
   кандидатом; поэтому возвращаемый `s_m` может быть меньше `minimum_s_m`.

   Новый обязательный invariant при валидной trajectory:

   ```text
   project(..., minimum_s_m).s_m >=
       clamp(minimum_s_m, 0, trajectory_length)
   ```

   Сегменты с `end_s < minimum_s_m` полностью исключать из выбора. Сегмент,
   содержащий lower bound, участвует только с
   `t >= (minimum_s_m - start_s) / (end_s - start_s)`; сегмент, заканчивающийся
   ровно в lower bound, может дать только endpoint с `s_m=minimum_s_m`.
   Следующие сегменты рассматриваются целиком. При lower bound, равном концу
   trajectory, возвращать конечную точку. После вычисления candidate добавить
   defensive invariant/check, но не маскировать ошибку одним
   `std::max(minimum_s_m, projection.s_m)`, потому что point/tangent должны
   соответствовать той же station.

   При принятии нового path сбрасывать progress в `0`, а в каждом runtime check
   обновлять абсолютный `current_s` через уже исправленный
   `projectOnTrajectorySamples(artifact.samples, current_pose,
   previous_current_s)`. Жёсткая нижняя граница `previous_current_s` делает
   progress монотонным и не позволяет повторно спроецироваться на пройденную
   ветвь самопересекающегося пути.

   Реализовать единый scanner состояния первого span над old artifact после
   `current_s`, но с двумя trigger-specific источниками probes:

   - prohibited adapter проходит все grid cells каждого сегмента тем же
     `cellsOnLine`/`isProhibited` контрактом, что runtime traversability;
   - raw-clearance adapter строит `ClearanceField2D` и сэмплирует stations с
     шагом не больше
     `min(path_raw_clearance_sample_step_m, 0.5 * grid.resolution())`.

   Первый blocked probe задаёт `first_blocked_s`. После подтверждения raw run
   минимальной длиной scanner **не возвращается**, а продолжает до первого safe
   probe. Консервативный `last_blocked_s` равен station первого safe probe после
   run; если выход до конца artifact не найден, он равен
   `artifact.samples.back().s_m`. Неподтверждённый короткий raw run
   сбрасывается, после чего scanner ищет следующий непрерывный run. Для
   prohibited predicate минимальная длина подтверждения не применяется.

   На границе старого ROS-контракта
   `ReplanBlockerEvent.blocker_path_distance_m` остаётся относительным и
   вычисляется явно как `first_blocked_s - current_s`; внутренний
   `PendingRepairContext` хранит только абсолютные stations. Результат шага:
   оба trigger-а дают фактический конец первого непрерывного span, а не
   `min_violation_length_m`.

   Добавить `ExecutableTrajectoryArtifact`, который хранит immutable копию
   принятого `path_id`, полных `TrajectoryPointSample`, mission goal,
   fingerprint геометрии и последний абсолютный `current_s`. Результат шага:
   оба trigger-а дают единый `[first_blocked_s, last_blocked_s]` в station frame
   artifact, а repair не зависит от мутирующего `last_valid_path_points_`.

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
     double current_s_m;
     double truncation_s_m;
   };
   ```

   Проецировать подтверждённую `A` с нижней границей `current_s_m`, хранить её
   как абсолютный `truncation_s_m` и проверять:

   ```text
   current_s <= truncation_s < first_blocked_s
   last_blocked_s >= first_blocked_s
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
   - новый `drone_city_nav/include/drone_city_nav/planning_grid_snapshot.hpp`
   - новый `drone_city_nav/src/planning_grid_snapshot.cpp`
   - новый `drone_city_nav/src/planner_node_grid_snapshot.cpp`
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

   Выбрать детерминированную fail-fast policy. Если параметр отсутствует,
   ROS declaration использует default `10...100`. Явно заданный список должен
   быть non-empty, состоять только из конечных положительных значений и быть
   строго возрастающим; NaN/inf, `<=0`, дубликат или нарушение порядка вызывают
   `std::invalid_argument` при загрузке planner config. Не сортировать, не
   удалять элементы и не подменять invalid explicit value default-списком,
   иначе изменятся cardinality/priority race без ведома пользователя.

   `partial_replan_internal_parallel_workers` обязан быть равен `1`; другое
   явно заданное значение также является config error. Внешняя гонка с
   default-конфигурацией запускает все 11 jobs сразу, как явно потребовано.

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

   Материализовать version identity всего composed grid, а не использовать
   только memory sequence:

   ```cpp
   struct PlanningGridVersion {
     std::uint64_t build_revision;
     std::uint64_t memory_producer_instance_id;
     std::uint64_t memory_sequence;
     std::int64_t lidar_update_ns;
     std::uint64_t config_fingerprint;
     OccupancyGridFingerprint raw;
     OccupancyGridFingerprint runtime_prohibited;
     OccupancyGridFingerprint planning_clearance;
   };

   struct RepairSnapshot {
     std::uint64_t generation;
     std::uint64_t blocked_path_id;
     PlanningGridVersion grid_version;
     std::array<RepairGridSnapshot, 2> grids;
     ExecutableTrajectoryArtifact old_trajectory;
     double current_s_m;
     double truncation_s_m;
     BlockedSpan blocked_span;
     KnownPassageMap passages;
     PlannerConfigSnapshot config;
   };

   struct RepairResult {
     std::uint64_t generation;
     std::uint64_t blocked_path_id;
     PlanningGridVersion source_grid_version;
     // Candidate trajectory, status and diagnostics.
   };
   ```

   Вынести production-подготовку в чистый
   `PlanningGridSnapshotBuilder` (`planning_grid_snapshot.hpp/.cpp`), а
   `planner_node_grid_snapshot.cpp` оставить тонким adapter-ом, который
   собирает source identities из node state. Helper принимает успешный
   `PlanningGridBuildResult`, центр/radius local inflation relaxation,
   применённые memory/lidar identities и config fingerprint; возвращает
   immutable prepared snapshot с raw/runtime/planning grids, двумя clearance
   fields и `PlanningGridVersion`.

   ```cpp
   std::optional<PreparedPlanningGridSnapshot>
   PlanningGridSnapshotBuilder::prepare(
       const PlanningGridPreparationInput& input);
   ```

   Порядок внутри helper-а фиксирован:

   ```text
   validate completed grid build
   -> copy raw/runtime/planning grids
   -> clear only inflation in runtime/planning around actual pose
   -> build clearance fields from final relaxed grids
   -> fingerprint final raw/runtime/planning grids
   -> copy applied memory/lidar/config identities
   -> assign and increment build_revision
   ```

   Failed/incomplete build, invalid bounds или ошибка подготовки возвращают
   `nullopt` и **не расходуют revision**. `build_revision` выдаётся
   node-owned counter-ом helper-а ровно один раз после полностью успешной
   подготовки. `memory_*` берутся из последнего применённого atomic snapshot,
   `lidar_update_ns` — из применённого `LidarInputSnapshot::update_ns`,
   fingerprints runtime/planning вычисляются только после local relaxation, а
   `config_fingerprint` покрывает static-map identity, bounds, inflation и
   planning-clearance параметры. Это одновременно даёт production API,
   который можно напрямую проверить без ROS/Gazebo, и выносит подготовку из
   почти предельного `planner_node_inputs.cpp`.

   Snapshot также содержит copied `KnownPassageMap`, config, generation,
   blocked path/fingerprint, old artifact, `A`, `current_s`, `truncation_s` и
   blocked span, а также точный `PlanningGridVersion`. Каждый `RepairResult`
   копирует эту version identity. Ни один job не обращается к mutable полям
   `PlannerNode`.

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

   Перед запуском соблюдать абсолютный station invariant:

   ```text
   reconnect_s > max(current_s, truncation_s) + endpoint_tolerance
   reconnect_s > last_blocked_s
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

   `RepairResult.grid_version` должен точно совпадать с version исходного
   immutable snapshot; это доказывает identity job input. Перед публикацией
   подготовить новый `PlanningGridVersion fresh_version` тем же API. Требовать
   `fresh_version.build_revision >= result.grid_version.build_revision` и
   логировать, какие source revisions/fingerprints изменились.

   Не отвергать candidate только из-за того, что fresh build revision больше:
   критической новизной считать изменение, которое фактически инвалидирует
   prefix или candidate на свежем `runtime_prohibited`, меняет known solids
   либо path/generation/fingerprint. Таким образом version даёт ordering и
   доказуемую принадлежность результата snapshot, а физическая fresh
   validation остаётся hard gate и не замораживает гонку при каждом lidar
   update.

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

   - race generation, blocked path/fingerprint, absolute
     `current_s/truncation_s/first_blocked_s/last_blocked_s`, span trigger,
     `A`, полный `PlanningGridVersion`;
   - для каждого job: kind, reconnect margin, `B`, выбранный grid, durations
     A*/corridor/optimizer/stitch/finalization, activation mode;
   - причины skip/reject: `beyond_goal`, `endpoint_prohibited`,
     `astar_failed`, `non_traversable`, `known_solid_intersection`,
     `stale_generation`, `prefix_invalidated`, `canceled`;
   - порядок completion, snapshot/fresh build revisions и изменившиеся source
     revisions/fingerprints, winner kind/margin, blocker-to-winner и
     winner-to-publication latency;
   - aggregate summary, если все partial и full job не дали winner.

   Cancellation логировать отдельно от invalid result, чтобы по логам можно было
   отличить проигравший job от алгоритмического failure.

10. **Добавить category-1 автотесты и зарегистрировать новые source/test
    targets.**

    Файлы:

    - новый `drone_city_nav/tests/trajectory_repair_test.cpp`
    - новый `drone_city_nav/tests/repair_race_test.cpp`
    - новый `drone_city_nav/tests/planning_grid_snapshot_test.cpp`
    - `drone_city_nav/tests/trajectory_test.cpp`
    - `drone_city_nav/tests/planner_core_test.cpp`
    - `drone_city_nav/tests/truncation_suffix_protocol_test.cpp`
    - `drone_city_nav/tests/planner_node_config_test.cpp`
    - `drone_city_nav/CMakeLists.txt`

    Материализуемый результат: детерминированные unit/component tests для
    hard-lower-bound projection, prepared grid/version issuance, span detection,
    B generation, stitching/finalization, arbiter, cancellation и config;
    существующий ACK protocol остаётся совместимым.
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
    winner. В `docs/configuration.md` явно зафиксировать fail-fast semantics
    reconnect margins: default применяется только при отсутствии параметра, а
    invalid explicit list останавливает запуск planner node.

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
     -R '^(trajectory_test|planning_grid_snapshot_test|trajectory_repair_test|repair_race_test|planner_core_test|path_raw_clearance_monitor_test|trajectory_planner_test|trajectory_optimizer_test|safe_trajectory_truncation_test|truncation_suffix_protocol_test|planner_node_config_test)$'
   ```

   Он проверяет новый repair/race контракт и существующие границы, которые
   меняются: projection lower bound, prepared grid/version issuance, span
   detection, optimizer cancellation, truncation ACK, config и trajectory
   finalization.

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

1. `trajectory_test` — жёсткий `minimum_s_m` contract для обоих
   `projectOnTrajectory*` API:
   - segment/polyline, ближайший к query, полностью лежит до lower bound:
     он не участвует, а результат имеет `s_m >= minimum_s_m`;
   - lower bound лежит внутри segment: `t` зажат снизу и point/tangent
     соответствуют ровно допустимой station, а не только численно исправленному
     `s_m`;
   - segment заканчивается ровно на lower bound и lower bound совпадает с
     концом trajectory;
   - self-crossing trajectory: геометрически ближайшая уже пройденная ветвь
     игнорируется и выбирается допустимая будущая ветвь;
   - одинаковый invariant проверяется отдельно для
     `projectOnTrajectory()` и `projectOnTrajectorySamples()`.

2. `planning_grid_snapshot_test` — прямой contract production builder:
   - incomplete/failed `PlanningGridBuildResult` не создаёт snapshot и не
     расходует revision; следующий успешный build получает revision `1`;
   - каждый полностью подготовленный successful build увеличивает revision
     ровно один раз (`1`, затем `2`), без increment на промежуточной ошибке;
   - fixture с inflation внутри relaxation radius доказывает, что runtime и
     planning fingerprints вычислены после снятия inflation, совпадают с
     возвращёнными final grids и отличаются от pre-relaxation fingerprints;
   - raw occupied cells сохраняются, а clearance fields соответствуют
     окончательным relaxed grids;
   - `memory_producer_instance_id`, `memory_sequence`, `lidar_update_ns` и
     `config_fingerprint` копируются из фактически применённых source
     identities без подмены или смешивания версий.

3. `planner_core_test`:
   - одно непрерывное prohibited-пересечение при `current_s > 0` даёт точные
     absolute first/last stations old artifact;
   - два раздельных span выбирают первый;
   - конфликт, начинающийся в escape-префиксе, не теряет фактический конец;
   - progress projection не уменьшается на самопересекающемся пути.

4. `path_raw_clearance_monitor_test`:
   - raw violation длиной существенно больше `min_violation_length_m`
     продолжается до фактического первого safe probe, а не заканчивается на
     минимальной длине подтверждения;
   - короткий неподтверждённый run пропускается и scanner находит следующий
     длинный run;
   - span, продолжающийся до mission goal, завершается последней station;
   - два раздельных длинных run возвращают первый.

5. `trajectory_repair_test` — B generation:
   - создаются `B10...B100` от `last_blocked_s`, а `A` у всех одинаков;
   - при non-zero `current_s` все запущенные candidates соблюдают
     `Bn > max(current_s, truncation_s)` и не возвращают дрон назад;
   - `Bn` после mission goal пропускается;
   - `Bn`, запрещённый в planning grid, но свободный в runtime grid, остаётся
     кандидатом;
   - `Bn`, запрещённый в обоих grids, пропускается;
   - `Bn` внутри hard-window не переносится автоматически и принимается только
     после успешной full stitched validation;
   - short candidate отклоняется из-за второго конфликта в old suffix, более
     длинный candidate может обойти оба.

6. `trajectory_repair_test` — stitching/finalization:
   - `A` и `B` совпадают по позиции, duplicate samples удалены, stations строго
     возрастают, NaN/разрывов нет;
   - XY-геометрия old suffix после `B` не меняется;
   - curvature, passage metadata, vertical profile и speed profile
     пересчитаны для всей stitched trajectory;
   - known `lower_mass`/`upper_mass` intersection отклоняет candidate;
   - degraded planner clearance/passage quality/curvature не отклоняют
     runtime-traversable и solid-clear candidate;
   - moving join и `AFTER_HOLD` получают корректный activation mode.

7. `repair_race_test` с fake jobs/mailbox:
   - первый завершившийся invalid result игнорируется, первый hard-valid
     выигрывает;
   - при одновременном completion winner выбирается атомарно один раз;
   - stale generation/path/fingerprint отклоняются;
   - каждый result несёт точный snapshot `PlanningGridVersion`;
   - новая build revision с теми же fingerprints проходит fresh validation;
   - новая build revision с изменённым lidar source и новым blocker отклоняет
     только затронутый candidate;
   - candidate, ставший invalid на свежем grid, не закрывает гонку, следующий
     valid result может выиграть;
   - winner вызывает `request_stop()`, optimizer замечает stop между iterations;
   - все partial failures не мешают full job победить;
   - failure всех 11 jobs оставляет temporary hold и публикует aggregate reason.

8. `planner_node_config_test`:
   - default margins равны 10...100;
   - отрицательные, NaN/inf, дубликаты, пустой и неупорядоченный explicit list
     приводят к `std::invalid_argument`, не сортируются и не заменяются
     default-списком;
   - валидный пользовательский строго возрастающий список сохраняет исходный
     порядок и задаёт ровно соответствующее число partial jobs;
   - internal workers в race config равны `1`.

9. `truncation_suffix_protocol_test`:
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
- **Blocked-span discretization.** Grid conflict может состоять из нескольких
  соседних/раздельных кластеров. Контракт намеренно берёт первый непрерывный
  span в абсолютной station frame: prohibited использует полный ordered
  cell traversal, raw-clearance — ограниченный grid-relative sample step и
  консервативную safe exit station. Последующие пересечения ловит full final
  validation, после чего более длинный partial или full job остаётся в гонке.
- **Projection API behavior.** Исправление `minimum_s_m` меняет поведение обоих
  общих projection helper-ов для всех consumers: прошедшие сегменты больше не
  смогут победить только из-за меньшей XY-distance. Это требуемый контракт для
  монотонного repair progress, но существующие callers должны пройти
  `trajectory_test` и package regression, особенно на endpoint/self-crossing
  случаях.
- **Grid revision churn.** Строгое равенство revision сделает winner почти
  невозможным при lidar updates. `PlanningGridVersion` поэтому разделяет
  монотонный build ordering, source identity и content fingerprints, а
  freshness определяется повторной фактической проверкой prefix/candidate на
  свежем grid.
- **Достоверность grid identity.** Revision нельзя выдавать до local relaxation
  и clearance construction, иначе failed build создаст gap, а fingerprints
  опишут не те grids, на которых работали jobs. Чистый prepared-grid builder и
  прямой component test закрепляют порядок и applied memory/lidar identities.
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

   Recommended decision: использовать конкретный `PlanningGridVersion` из шага
   3. Его `build_revision` — монотонный номер каждой полностью подготовленной
   композиции, а memory producer/sequence, lidar update timestamp, config hash и
   три grid fingerprints объясняют изменение содержимого. `RepairSnapshot` и
   `RepairResult` несут одну и ту же version identity.

   При fresh publication меньшая build revision является internal error,
   равная означает тот же build, большая требует повторной физической проверки,
   но не является самостоятельным reject. Критической новая revision становится
   только если fresh `runtime_prohibited`/known-solid validation инвалидирует
   prefix или candidate либо изменился blocked path/generation/fingerprint.
   Подтверждение: race tests с новой revision при одинаковых fingerprints и с
   новым lidar blocker.

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
