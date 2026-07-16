# Context

Нужно расширить существующую двумерную obstacle memory постоянной диагностической
3D provenance для каждой активной occupied-клетки. Планирование, hit/miss scoring,
inflation, A*, trajectory и flight control должны остаться строго без изменений.

Сейчас `projectLidarBeam()` уже вычисляет полноценный map-frame луч и endpoint XYZ,
но `ObstacleMemoryGrid` сохраняет только XY state/score. Коммит `f9d7aa7` добавил
одноразовый `ObstacleMemoryOccupiedTransition` и лог `PASSAGE_MEMORY_HIT`; после
завершения scan callback эти данные исчезают. Поэтому поздний prohibited replan можно
связать с исходным 3D hit только вручную по старому логу, а последнее подтверждение,
диапазон высот и полная жизнь occupied-клетки вообще не сохраняются.

Целевое состояние:

```text
ObstacleMemoryGrid
  raw_grid_ + scores_             -> единственный planning contract
  sparse occupied provenance     -> только diagnostics

obstacle_memory_node
  one publication stamp
    -> OccupancyGrid
    -> typed provenance PointCloud2

planner_node
  exact stamp/frame/geometry match
    -> enrich memory-blocker log
  no match / malformed / late data
    -> explicit diagnostic absence, planning continues unchanged
```

Per-beam ground rejection, 3D voxel memory и tilt-based lidar suspension в этот план
не входят. Они не должны быть реализованы попутно.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на непосредственном чтении
текущего `main`, HEAD `f9d7aa7`, call sites, тестов, CMake/package wiring, конфигурации
и документации.

Notion не запрашивался: в prompt нет Notion task/ID, а policy запуска `optional`.
GitLab не запрашивался: в prompt нет MR, GitLab review или discussions. Поэтому
удалённые read-команды не выполнялись; использованы только локальный код и история Git.

Проверенные факты:

- `LidarBeamProjection` содержит `ray_origin_map_m`, `ray_direction_map`,
  `endpoint_map_m` и `endpoint_altitude_m`; их заполняет
  `projectLidarBeam()` после attitude/mount compensation
  (`drone_city_nav/include/drone_city_nav/lidar_projection.hpp:42`,
  `drone_city_nav/src/lidar_projection.cpp:155`).
- `LaserScan2DView` передаёт roll/pitch и параметры projection, но не scan stamp или
  `time_increment` (`drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:15`).
- Постоянное состояние `ObstacleMemoryGrid` состоит только из `raw_grid_` и `scores_`
  (`drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:91`).
- One-shot transition создаётся только при `!occupied_before && isOccupied(...)`
  (`drone_city_nav/src/obstacle_memory.cpp:226`); repeated-hit тест подтверждает, что
  второй transition не публикуется (`drone_city_nav/tests/obstacle_memory_test.cpp:281`).
- `KnownStaticLidarHitResult` содержит `string_view`, ссылающиеся на classifier-owned
  volumes; для persistent state нужны owning copies
  (`drone_city_nav/include/drone_city_nav/known_static_lidar_hit_classifier.hpp:30`).
- `syncCellState()` централизованно переводит score в occupied/free/unknown, а
  `reset()` сейчас очищает только grid и scores
  (`drone_city_nav/src/obstacle_memory.cpp:259`, `:294`).
- `obstacle_memory_node` публикует только transient-local OccupancyGrid и генерирует
  новый `now()` внутри `publishMemoryGrid()`
  (`drone_city_nav/src/obstacle_memory_node.cpp:221`, `:552`).
- Planner после `rawOccupancyGridFromRos()` сохраняет grid, но не header stamp/frame
  (`drone_city_nav/src/planner_node_inputs.cpp:62`).
- Источник replan blocker определяется в
  `PlannerNode::describeProhibitedIntersectionSource()`; current-lidar provenance уже
  добавляется там же, memory provenance отсутствует
  (`drone_city_nav/src/planner_node_runtime.cpp:109`).
- ROS adapter library уже зависит от `sensor_msgs` и содержит PointCloud2 builders;
  новый interface package не требуется (`drone_city_nav/CMakeLists.txt:106`).
- `PointCloud2` не имеет message-level grid metadata, поэтому snapshot будет
  коррелироваться по exact header stamp/frame и проверяться относительно принятого
  OccupancyGrid: cell bounds, duplicate cells, occupied count и endpoint-to-cell
  consistency.
- Размеры затрагиваемых `.cpp` ниже repository limit 1000 строк; serialization/cache
  следует вынести в отдельные модули, чтобы не раздувать node files
  (`scripts/tests/test_cpp_source_size_contract.py:11`).

# Detected stack/profiles

- ROS 2 workspace, один `ament_cmake` C++20 package `drone_city_nav`.
- Build orchestration: top-level Makefile вызывает `colcon`.
- Runtime contracts: ROS 2 `nav_msgs`, `sensor_msgs`, `px4_msgs`, `rclcpp`.
- Tests: `ament_cmake_gtest`/CTest и Python `unittest` script contracts.
- Static analysis/format: repository scripts wrapping clang-format, clang-tidy,
  cppcheck и build/test checks.

Прочитаны и применяются профили:

- обязательный `project_profiles/generic.md`, потому что он применяется ко всем
  workspace;
- `project_profiles/cpp.md`, потому что изменяются C++ headers/sources и CMake target
  wiring.

Rust profile не применяется: workspace задачи является C++/ROS 2 проектом; sibling
repository использовался только для чтения orchestrator protocols.

# Repo-approved commands found

Источник команд: `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, `Makefile`.

- `./scripts/build.sh` — containerized `make build`/colcon build.
- `./scripts/test.sh` — containerized build + CTest.
- `./scripts/dev_shell.sh make format` — форматирование только изменённых C++ файлов.
- `./scripts/dev_shell.sh make quality` — обязательная broad quality-проверка перед
  commit; включает non-mutating format check, scoped clang-tidy при наличии compile DB,
  build и CTest.
- `./scripts/dev_shell.sh make test-scripts` — Python script contracts.
- После build допустим документированный scoped CTest:
  `./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R '<regex>'`.

Прямой host CMake, случайный новый build dir, root workspace writes и remote commands
не используются.

# Affected components

1. **Projected observation contract**
   - `drone_city_nav/include/drone_city_nav/lidar_projection.hpp`
   - новый focused core header/source для beam observation при необходимости.

2. **Obstacle-memory ownership/lifecycle**
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`
   - `drone_city_nav/src/obstacle_memory.cpp`
   - существующий `ObstacleMemoryOccupiedTransition` из `f9d7aa7`.

3. **Typed ROS diagnostic transport**
   - новые `obstacle_memory_provenance_ros.hpp/.cpp` в core-owned include и
     `drone_city_nav_ros_adapters` source scope;
   - `sensor_msgs::msg::PointCloud2` с фиксированной проверяемой схемой, без JSON.

4. **Producer wiring**
   - `drone_city_nav/src/obstacle_memory_node.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`
   - transient-local provenance publisher с тем же stamp, что и memory grid.

5. **Planner diagnostic consumer**
   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_lifecycle.cpp`
   - `drone_city_nav/src/planner_node_inputs.cpp` либо новый focused callback file;
   - `drone_city_nav/src/planner_node_runtime.cpp`.

6. **Build/tests/contracts/docs**
   - `drone_city_nav/CMakeLists.txt`; `package.xml` менять не требуется, если остаётся
     существующий `sensor_msgs` transport;
   - новые unit tests для core provenance, PointCloud2 adapter и snapshot matcher/cache;
   - существующие memory/known-static/planning-grid tests;
   - `scripts/tests/test_topic_contract.py` и при необходимости bag/topic contracts;
   - `docs/obstacle_mapping.md`, `docs/diagnostics.md`, `docs/configuration.md`.

# Implementation steps

1. **Ввести reusable observation и owning accepted-hit snapshot без изменения
   projection math.**

   Файлы: `drone_city_nav/include/drone_city_nav/lidar_projection.hpp`,
   `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` или новый
   `lidar_beam_observation.hpp`; node заполнение в
   `drone_city_nav/src/obstacle_memory_node.cpp:onScan()`.

   Материализуемый результат:

   - `LidarBeamObservation` композиционно использует существующий
     `LidarBeamProjection`, а не копирует его геометрию;
   - хранит beam index, beam acquisition stamp, stamp-valid flag, callback receive stamp,
     фактически использованные roll/pitch/tilt и attitude-valid flag;
   - beam stamp вычисляется как valid `LaserScan.header.stamp + i*time_increment`; если
     stamp отсутствует/некорректен, validity=false, а receive time хранится отдельно;
   - `AcceptedObstacleMemoryHit` добавляет owning snapshot existing known-static context:
     `classifier_applied`, classification, expected range/delta, volume/face flags,
     part kind validity и owning `structure/opening/part` strings. Нельзя сохранять
     `KnownStaticLidarHitResult::string_view`.

   Ориентир контракта:

   ```cpp
   struct LidarBeamObservation {
     std::size_t beam_index;
     std::int64_t acquisition_stamp_ns;
     bool acquisition_stamp_valid;
     std::int64_t receive_stamp_ns;
     LidarBeamProjection projection;
     AttitudeEuler projection_attitude;
     bool attitude_valid;
     double measured_range_m;
   };

   struct AcceptedObstacleMemoryHit {
     LidarBeamObservation beam;
     KnownStaticClassificationSnapshot known_static; // owns strings
   };
   ```

   Full pose/attitude interpolation к beam timestamp не добавлять: observation должна
   честно фиксировать projection state, который runtime фактически использовал.

2. **Сделать `ObstacleMemoryGrid` authoritative owner sparse provenance и связать её
   lifecycle с существующим state machine.**

   Файлы/anchors:
   `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:91`,
   `drone_city_nav/src/obstacle_memory.cpp:111`, `:272`, `:294`.

   Добавить `std::unordered_map<std::size_t, MemoryCellProvenance>` (или эквивалентный
   sparse associative container), keyed by `raw_grid_.linearIndex(cell)`:

   ```cpp
   struct MemoryCellProvenance {
     GridIndex cell;
     AcceptedObstacleMemoryHit occupancy_trigger;
     AcceptedObstacleMemoryHit last_hit;
     std::optional<double> min_endpoint_z_m;
     std::optional<double> max_endpoint_z_m;
     std::uint64_t accepted_hit_count;
   };
   ```

   Семантика:

   - только hit, реально прошедший existing known-static filter и переведший state в
     occupied, создаёт trigger/last/count=1;
   - repeated accepted hit при уже occupied сохраняет trigger, обновляет last,
     count и finite min/max Z;
   - pre-occupied score accumulation provenance не создаёт: trigger означает именно
     observation перехода;
   - переход occupied -> unknown/free в `applyMiss()/syncCellState()` удаляет запись в
     том же update;
   - re-entry создаёт новый trigger;
   - `reset()` очищает grid, scores и map;
   - invariant после каждого scan/reset: `provenance.size() == occupied_cell_count`,
     и каждая запись соответствует occupied cell;
   - hit без valid 3D projection сохраняет explicit invalid flag/empty Z; finite Z не
     выдумывается, а существующее 2D поведение сохраняется.

   Refactor `ObstacleMemoryOccupiedTransition`: transition event должен ссылаться на
   authoritative newly-created record (cell + score delta, либо копия через один helper),
   а не повторно вручную собирать все поля. `PASSAGE_MEMORY_HIT` должен читать trigger из
   authoritative provenance accessor/snapshot.

3. **Добавить компактный typed PointCloud2 adapter с полной schema validation.**

   Новые файлы:
   `drone_city_nav/include/drone_city_nav/obstacle_memory_provenance_ros.hpp`,
   `drone_city_nav/src/obstacle_memory_provenance_ros.cpp`,
   `drone_city_nav/tests/obstacle_memory_provenance_ros_test.cpp`;
   wiring в `drone_city_nav/CMakeLists.txt`.

   Выбранный transport: `sensor_msgs::msg::PointCloud2`, потому что package уже зависит
   от `sensor_msgs`, repo уже использует отдельные PointCloud2 adapters, а diagnostic
   snapshot состоит из homogeneous records. Не вводить opaque JSON или новый rosidl
   interface package.

   Фиксированная wire schema должна как минимум содержать:

   - standard map-frame `x/y/z` = trigger endpoint (без RViz-only Z inversion);
   - `cell_x`, `cell_y`;
   - trigger/last endpoint XYZ и validity flags;
   - trigger/last acquisition `sec/nanosec`, stamp-valid flags, receive stamp и beam;
   - trigger/last roll/pitch/tilt и attitude-valid flags;
   - trigger/last measured/expected range и delta;
   - trigger/last classifier-applied, classification, part-kind-valid, part kind;
   - min/max endpoint Z + validity flags;
   - accepted hit count (если wire field UINT32, saturating conversion должен иметь
     explicit saturation flag; core counter остаётся UINT64).

   Serializer устанавливает exact field names/types/offsets, `is_bigendian=false` и
   `is_dense=false` при invalid XYZ. Parser не делает unchecked offset access:

   - exact schema/version/point_step/row_step/data-size validation;
   - reject unsupported big-endian, duplicate cells, unknown enum values и malformed
     booleans;
   - checked `memcpy` в aligned local values;
   - owning `MemoryProvenanceSnapshot {stamp_ns, frame_id, records}`;
   - пустой snapshot остаётся валидным и сохраняет fields/header.

4. **Публиковать grid и provenance как один логический snapshot с общим stamp.**

   Файлы/anchors:
   `drone_city_nav/src/obstacle_memory_node.cpp:221`, `:328`, `:444`, `:552`;
   `drone_city_nav/config/urban_mvp.yaml:1`.

   - Добавить параметр/topic
     `obstacle_memory_provenance_topic: /drone_city_nav/obstacle_memory_provenance`.
   - Publisher: reliable transient-local depth 1, аналогично memory grid.
   - После `integrateScan()` один раз получить `publication_stamp = now()` и построить
     оба сообщения с одинаковыми `header.stamp`/`frame_id`; не вызывать `now()` внутри
     отдельных builders.
   - Публиковать полный sparse snapshot active occupied provenance при каждом memory-grid
     publish, чтобы exact-stamp matching был возможен. Добавить throttled counters:
     occupied/provenance count, serialized bytes, invalid-Z count.
   - Existing `PASSAGE_MEMORY_HIT` оставить unthrottled, но заполнять из trigger
     authoritative record и дополнить acquisition/receive timestamp source.
   - При known-passage classifier installation/reset provenance очищается вместе с memory.

5. **Добавить planner-side parsed snapshot cache и строгий matcher, не влияющий на
   planning readiness.**

   Файлы:
   `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:17`,
   `drone_city_nav/src/planner_node_config.cpp:493`,
   `drone_city_nav/src/planner_node.hpp:303`,
   `drone_city_nav/src/planner_node_lifecycle.cpp:17`,
   `drone_city_nav/src/planner_node_inputs.cpp:62`; при росте callbacks вынести в новый
   `planner_node_memory_provenance.cpp`.

   - Добавить topic в `PlannerTopics` и reliable transient-local subscription.
   - `onMemoryGrid()` после успешной conversion сохраняет header stamp/frame рядом с
     `memory_grid_`; invalid grid не меняет accepted stamp.
   - Provenance callback использует adapter parser и кладёт successful snapshot в
     bounded cache последних 4 stamps. Cache должен быть pure/testable helper, не
     ROS callback-only logic.
   - Lookup разрешён только при exact stamp, exact frame и полной geometry consistency:
     все cells уникальны/in-bounds, record count равен occupied-cell count, valid
     trigger/last XY map back to record cell. Любое нарушение отклоняет snapshot целиком.
   - Late/out-of-order provenance может сопоставиться благодаря cache; arrival order не
     является identity.
   - Отсутствие publisher/topic, malformed snapshot и mismatch не меняют
     `memory_grid_seen_`, planning readiness, source merge или A*; сохраняется status/reason
     для diagnostics (`not_received`, `stamp_mismatch`, `frame_mismatch`,
     `schema_invalid`, `geometry_mismatch`, `cell_missing`).
   - Cache ограничен четырьмя snapshots и очищается/пересматривается при смене frame/grid
     geometry; никакого unbounded накопления.

6. **Обогатить только memory-source replan diagnostics.**

   Файл/anchor: `drone_city_nav/src/planner_node_runtime.cpp:109`, функция
   `describeProhibitedIntersectionSource()`.

   Выбирать provenance cell детерминированно:

   - если blocker cell непосредственно occupied в memory source, использовать её;
   - иначе использовать `nearest_source->cell` только когда nearest source = `memory`;
   - для static/current-lidar-only blocker memory provenance имеет status
     `not_applicable`.

   При matched snapshot/record добавить:

   ```text
   memory_provenance[status=matched cell=(...)
     trigger_endpoint=(x,y,z) trigger_stamp=... trigger_attitude=(...)
     trigger_range=... trigger_classification=... trigger_part=...
     last_endpoint=(x,y,z) last_stamp=... last_attitude=(...)
     last_range=... last_classification=... last_part=...
     z_range=[min,max] accepted_hits=N]
   ```

   При отсутствии добавить `memory_provenance[status=unavailable reason=...]`.
   Formatter лучше вынести в pure helper и unit-test, чтобы не тестировать private ROS node
   через log scraping. Existing current-lidar `known_static_hit` contract не менять.

7. **Закрепить публичный diagnostic contract тестами и документацией.**

   Файлы:
   `drone_city_nav/tests/obstacle_memory_test.cpp`, новые adapter/cache/formatter tests,
   `drone_city_nav/tests/known_static_lidar_ingestion_test.cpp`,
   `drone_city_nav/tests/known_static_lidar_planning_grid_integration_test.cpp`,
   `scripts/tests/test_topic_contract.py`, `scripts/record_debug_bag.sh`,
   `docs/obstacle_mapping.md`, `docs/diagnostics.md`, `docs/configuration.md`,
   при необходимости `README.md`.

   - Добавить provenance topic в default YAML, topic contract и debug bag, но не в
     planner raw obstacle source list.
   - Документировать: grid остаётся authoritative 2D evidence; provenance diagnostic-only;
     exact stamp matching; trigger vs last; invalid Z; bounded cache; bandwidth/size counters;
     no guarantee of per-beam pose interpolation.
   - Обновить старую формулировку `docs/diagnostics.md:131`, чтобы ручной
     `PASSAGE_MEMORY_HIT` correlation больше не описывался как единственный способ.
   - Scoped search excluding `tasks/`, `build/`, `install/`, `log/` должен подтвердить,
     что old duplicate transition field population удалён и provenance topic нигде не
     подключён как raw/prohibited planner input.

# Verification plan

Последовательность после реализации:

1. Container build:

   ```bash
   ./scripts/build.sh
   ```

2. Scoped CTest после build (точные новые target names уточнить по реализации, но не
   заменять их ad-hoc binary запуском):

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
     --output-on-failure \
     -R '^(obstacle_memory_test|obstacle_memory_provenance_ros_test|memory_provenance_cache_test|planner_memory_provenance_format_test|known_static_lidar_ingestion_test|known_static_lidar_planning_grid_integration_test)$'
   ```

3. Форматировать только changed C++:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

4. Обязательная broad quality-проверка:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

5. Script contracts:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

6. Проверить diff/status, затем создать один coherent local commit; не push.

Live/headless flight для этой diagnostic-only задачи не обязателен. Если executor всё же
выполнит smoke run, он является дополнительной проверкой topic publication/log matching,
но не заменяет unit/adapter/cache tests.

# Testing strategy

## 1. Без дополнительного рефакторинга — обязательно

- `ObstacleMemoryGrid`: первый occupied transition создаёт trigger/last/count=1 с finite
  XYZ при valid altitude/attitude.
- Repeated accepted hit с другим origin altitude, но в той же XY cell: trigger неизменен,
  last обновлён, min/max расширены, count увеличен, второго transition нет.
- Достаточные misses переводят occupied в unknown/free и удаляют provenance; новый hit
  после re-entry создаёт новый trigger.
- `reset()` очищает grid/scores/provenance.
- Existing expected-known-static suppression не создаёт provenance.
- Invalid 3D projection явно отмечается и не меняет существующий 2D occupancy result.
- Existing raw-memory/no-inflation/scoring tests остаются без semantic expectation changes.

## 2. С лёгким рефакторингом — включено в задачу

- Pure PointCloud2 round trip для всех полей, empty snapshot и invalid-Z record.
- Negative adapter tests: missing/wrong field, wrong datatype/offset/count, bad
  point/row step, truncated/oversized data, duplicate cell, invalid enum/flag, unsupported
  endianness.
- Pure bounded-cache/matcher tests: provenance-before-grid, grid-before-provenance,
  exact match, stale stamp, frame mismatch, geometry mismatch, occupied-count mismatch,
  out-of-bounds cell, duplicate cells, cache eviction.
- Pure formatter tests для `matched`, каждого unavailable reason и `not_applicable`.
- Config default/override tests и Python topic/bag contract.

Production-only test hooks не добавлять. Небольшие pure adapter/cache/formatter modules
являются частью реального runtime contract, а не helpers только ради тестов.

## 3. С тяжёлым рефакторингом — не входит

- ROS executor integration test с двумя реальными nodes и искусственной перестановкой
  DDS delivery потребовал бы отдельной launch-test infrastructure, которой в package сейчас
  нет. Риск закрывается pure parser/cache ordering tests и existing node wiring contracts.
- Performance/load test на сотнях тысяч occupied cells требует отдельного benchmark harness.
  В этой задаче обязательны sparse storage, serialized-byte counters и bounded cache;
  benchmark остаётся future gap, если реальные counters покажут высокий bandwidth.
- Full scan-time pose/attitude interpolation и per-beam synchronized state — отдельная
  sensor-fusion задача и явно out of scope.

# Risks and tradeoffs

- **Bandwidth/CPU:** fixed PointCloud2 record заметно больше OccupancyGrid cell. Полный
  snapshot на каждый scan даёт точную correlation, но стоимость растёт с occupied count.
  Смягчение: compact fixed schema, sparse records, serialized-byte counters, QoS depth 1,
  bounded planner cache. Нельзя молча truncation records: это нарушит «каждая occupied cell».
- **Cross-topic non-atomicity:** одинаковый stamp не гарантирует order доставки. Решение:
  exact stamp/frame identity и cache четырёх snapshots; при отсутствии — unavailable, не
  guessed association.
- **Diagnostic-only regression:** случайное подключение provenance к planning может изменить
  маршрут. Решение: subscriber используется только formatter/cache; planning builder получает
  прежний `memory_grid_`; script/search contract это закрепляет.
- **Lifecycle drift:** provenance может пережить occupied state или не появиться для occupied
  cell. Решение: state transition и provenance mutation находятся в одном owner; invariant и
  count-match tests.
- **Dangling classification IDs:** current classifier возвращает `string_view`. Решение:
  owning snapshot до выхода из scan integration.
- **Timestamp semantics:** beam acquisition time и projection pose не полностью синхронны.
  Решение: хранить acquisition validity и receive time отдельно, документировать отсутствие
  interpolation, не выдавать callback time за sensor time.
- **Malformed wire data:** ручной PointCloud2 layout опасен unchecked offsets/alignment.
  Решение: один adapter, exact schema validation, checked `memcpy`, reject-whole-snapshot,
  exhaustive negative tests.
- **Counter overflow:** core hit count UINT64, PointCloud2 не имеет UINT64 field. Решение:
  либо two-UINT32 wire representation, либо saturating UINT32 + saturation flag; recommended
  второй вариант как компактный и достаточный для runtime diagnostics.
- **Source-size growth:** node files уже 628/719 строк. Serialization/cache/formatter вынести
  в отдельные modules; contract limit 1000 строк должен остаться зелёным.

# Open questions

1. **Использовать `PointCloud2` или новый custom ROS message package?**

   Recommended decision: использовать `sensor_msgs::msg::PointCloud2` с жёсткой fixed schema
   и отдельным adapter. Rationale: `sensor_msgs` уже зависимость, данные homogeneous, это не
   требует rosidl/package split и соответствует существующим debug pointcloud patterns.
   Custom message следует пересмотреть только если reviewer считает обязательной передачу
   variable-length `structure/opening/part` strings planner-у. Текущий requested planner log
   требует classification/known part, которые кодируются numeric fields; owning strings
   сохраняются внутри memory node и `PASSAGE_MEMORY_HIT`.

2. **Какой stamp является identity snapshot-а?**

   Recommended decision: один `publication_stamp = obstacle_memory_node.now()` для grid и
   provenance headers; per-beam acquisition stamp хранится внутри records и не используется
   как snapshot identity. Rationale: grid является накопленным state после целого callback,
   поэтому scan stamp отдельного beam не идентифицирует весь published snapshot.

3. **Публиковать provenance реже для экономии bandwidth?**

   Recommended decision: в этой задаче публиковать вместе с каждым memory grid update.
   Rationale: более редкая публикация делает exact-stamp provenance систематически недоступной
   для большинства grid snapshots и нарушает основной forensic contract. Сначала добавить
   byte/count diagnostics; throttling/delta protocol допустим отдельной задачей только по
   измеренным performance данным.

4. **Что делать с snapshot, где валидна часть records?**

   Recommended decision: reject whole snapshot с конкретной причиной; planning продолжает
   работать по grid. Rationale: partial acceptance может создать правдоподобную, но неполную
   provenance и скрыть именно blocker cell. Diagnostic truth важнее частичного обогащения.

5. **Нужен ли headless run для completion?**

   Recommended decision: нет, не как mandatory gate. Rationale: задача не меняет flight
   behavior, а все lifecycle/serialization/ordering contracts портируемо покрываются unit tests.
   Один optional smoke полезен только для подтверждения publication/log wiring и не заменяет
   автоматические tests.

Объективных blockers для реализации в текущем workspace не обнаружено.
