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
    -> typed ObstacleMemoryProvenance message

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
текущего `main`, исходного implementation HEAD `f9d7aa7`, call sites, тестов,
CMake/package wiring, конфигурации и документации. Первый plan-round был закоммичен как
`445df2a`; после peer review отдельно перепроверены transport geometry identity и место
создания projected observation.

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
- `PointCloud2` не имеет message-level grid metadata. Cell bounds/count и endpoint-to-cell
  consistency не доказывают equality geometry, а при zero-record snapshot вообще не дают
  geometry evidence. Поэтому первоначальный PointCloud2 вариант отвергнут после review.
- В workspace один package, а approved Makefile собирает его через
  `colcon build --packages-select drone_city_nav`. Custom `.msg` целесообразно генерировать
  внутри существующего package через `rosidl_generate_interfaces()`, не создавая второй
  package и не меняя approved package selection (`drone_city_nav/CMakeLists.txt:1`,
  `Makefile:8`).
- `projectLidarBeam()`, known-static classification и accepted-hit decision выполняются
  только внутри `ObstacleMemoryGrid::integrateScan()`
  (`drone_city_nav/src/obstacle_memory.cpp:111-253`). `onScan()` должен передать timing
  metadata через `LaserScan2DView`, но не выполнять второй projection/classification pass.
- Фактически применённые roll/pitch сейчас определяются внутри projection условием
  `config.compensate_attitude && pose.attitude_valid`; при false используются нули
  (`drone_city_nav/src/lidar_projection.cpp:69-79`). Этот effective state должен явно
  возвращаться в projection result, иначе provenance может ошибочно назвать stale raw
  attitude применённой.
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
   - новые `drone_city_nav/msg/ObstacleMemoryHitObservation.msg`,
     `ObstacleMemoryCellProvenance.msg`, `ObstacleMemoryProvenance.msg`;
   - новые `obstacle_memory_provenance_ros.hpp/.cpp` в include/ROS-adapter scope;
   - parent message переносит `Header`, полную `MapMetaData`, version и raw-grid hash,
     поэтому geometry проверяется и при zero occupied cells; JSON/PointCloud2 не используются.

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
   - `drone_city_nav/CMakeLists.txt`, `drone_city_nav/package.xml` для rosidl generation и
     runtime typesupport внутри существующего package;
   - новые unit tests для core provenance, generated-message adapter и snapshot matcher/cache;
   - существующие memory/known-static/planning-grid tests;
   - `scripts/tests/test_topic_contract.py` и при необходимости bag/topic contracts;
   - `docs/obstacle_mapping.md`, `docs/diagnostics.md`, `docs/configuration.md`.

# Implementation steps

1. **Ввести reusable observation и owning accepted-hit snapshot без изменения
   projection math.**

   Файлы/anchors: `drone_city_nav/include/drone_city_nav/lidar_projection.hpp:18-60`,
   `drone_city_nav/src/lidar_projection.cpp:69-79,154-213`,
   `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:15-34`,
   `drone_city_nav/src/obstacle_memory.cpp:127-252`,
   `drone_city_nav/src/obstacle_memory_node.cpp:328-391`; при необходимости новый
   focused `lidar_beam_observation.hpp/.cpp`.

   Материализуемый результат:

   - расширить `LaserScan2DView` transport-only timing полями
     `scan_stamp_ns`, `scan_stamp_valid`, `time_increment_s`,
     `receive_stamp_ns`, `receive_stamp_valid`; `onScan()` только проверенно конвертирует
     `LaserScan.header.stamp`, `scan.time_increment` и уже вычисленный `now_ns` в эти поля;
   - zero ROS stamp (`sec=0,nanosec=0`), `nanosec>=1e9`, negative sec и overflow
     помечаются invalid; callback receive stamp не подменяет acquisition stamp;
   - pure checked helper вычисляет beam acquisition time как
     `scan_stamp_ns + round(i * time_increment_s * 1e9)`. Для beam 0 valid scan stamp
     достаточен; для последующих beams non-finite/negative increment или arithmetic overflow
     дают `acquisition_stamp_valid=false`. Нулевой finite increment допустим и означает тот
     же acquisition stamp для всех beams;
   - refactor `projectLidarBeam()` вычисляет effective attitude ровно один раз и возвращает
     в `LidarBeamProjection`: `attitude_compensation_applied`,
     `applied_roll_rad`, `applied_pitch_rad`, `applied_tilt_rad`. Тот же effective object
     используется в `projectDirectionToNed()`, поэтому reported и применённые значения не
     могут разойтись;
   - когда compensation disabled либо `pose.attitude_valid=false`, applied flag=false и
     applied roll/pitch/tilt равны нулю. Отдельные source-attitude поля хранят raw
     roll/pitch/tilt только при `attitude_valid=true`, иначе validity=false и NaN; stale raw
     values никогда не маркируются как использованные projection attitude;
   - `LidarBeamObservation` композиционно использует единственный уже вычисленный
     `LidarBeamProjection`, а не копирует его геометрию и не вызывает projection повторно;
   - observation создаётся внутри `ObstacleMemoryGrid::integrateScan()` после единственного
     `projectLidarBeam()` и known-static classification, но до `applyAcceptedHit()`.
     Expected-static hit завершает branch до observation/provenance mutation;
   - `AcceptedObstacleMemoryHit` добавляет owning snapshot existing known-static context:
     `classifier_applied`, classification, expected range/delta, volume/face flags,
     part kind validity и owning `structure/opening/part` strings. Нельзя сохранять
     `KnownStaticLidarHitResult::string_view`.

   Ориентир контракта:

   ```cpp
   struct LaserScanTiming {
     std::int64_t first_beam_stamp_ns;
     bool first_beam_stamp_valid;
     double time_increment_s;
     std::int64_t receive_stamp_ns;
     bool receive_stamp_valid;
   };

   struct LidarBeamObservation {
     std::size_t beam_index;
     std::int64_t acquisition_stamp_ns;
     bool acquisition_stamp_valid;
     std::int64_t receive_stamp_ns;
     bool receive_stamp_valid;
     LidarBeamProjection projection;
     AttitudeEuler source_attitude;       // NaN when source_attitude_valid=false
     bool source_attitude_valid;
     AttitudeEuler applied_attitude;      // roll/pitch are exactly those used
     bool attitude_compensation_applied;
     double measured_range_m;
   };

   struct AcceptedObstacleMemoryHit {
     LidarBeamObservation beam;
     KnownStaticClassificationSnapshot known_static; // owns strings
   };
   ```

   Exact call flow:

   ```text
   onScan: ROS stamp/time_increment/receive stamp -> LaserScan2DView
   integrateScan loop:
     projectLidarBeam once
     -> classify known static once
     -> expected static: continue
     -> makeLidarBeamObservation(scan, i, projection)
     -> applyAcceptedHit(cell, observation + owning classification)
   ```

   Full pose/attitude interpolation к beam timestamp не добавлять: observation честно
   фиксирует projection state, который runtime фактически использовал.

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

   Заменить private `applyHit(cell, config)` на
   `applyAcceptedHit(cell, AcceptedObstacleMemoryHit, config)`, который в одной функции
   обновляет score/state и создаёт либо обновляет provenance. Он возвращает optional
   transition descriptor только при фактическом входе в occupied. Refactor
   `ObstacleMemoryOccupiedTransition`: transition event материализуется одним helper из
   authoritative newly-created record + score before/after, а не повторно вручную
   заполняет projection/classification поля. `PASSAGE_MEMORY_HIT` читает trigger из того же
   authoritative record/accessor.

3. **Добавить custom typed ROS contract, который переносит geometry даже при пустом
   snapshot.**

   Новые файлы:
   `drone_city_nav/msg/ObstacleMemoryHitObservation.msg`,
   `drone_city_nav/msg/ObstacleMemoryCellProvenance.msg`,
   `drone_city_nav/msg/ObstacleMemoryProvenance.msg`,
   `drone_city_nav/include/drone_city_nav/obstacle_memory_provenance_ros.hpp`,
   `drone_city_nav/src/obstacle_memory_provenance_ros.cpp`,
   `drone_city_nav/tests/obstacle_memory_provenance_ros_test.cpp`;
   wiring в `drone_city_nav/CMakeLists.txt` и `drone_city_nav/package.xml`.

   Выбранный transport: generated custom messages **внутри существующего
   `drone_city_nav` package**. Это устраняет ручной byte layout PointCloud2, сохраняет
   `uint64` без saturation и переносит parent-level grid identity при нуле records. Не
   создавать второй package: текущий approved `--packages-select drone_city_nav` остаётся
   рабочим.

   Exact message contract:

   ```text
   # ObstacleMemoryHitObservation.msg
   uint8 CLASSIFICATION_EXPECTED_STATIC=0
   uint8 CLASSIFICATION_UNEXPECTED=1
   uint8 CLASSIFICATION_AMBIGUOUS=2
   uint8 KNOWN_PART_LEFT=0
   uint8 KNOWN_PART_RIGHT=1
   uint8 KNOWN_PART_LOWER=2
   uint8 KNOWN_PART_UPPER=3
   uint64 beam_index
   builtin_interfaces/Time acquisition_stamp
   bool acquisition_stamp_valid
   builtin_interfaces/Time receive_stamp
   bool receive_stamp_valid
   geometry_msgs/Point ray_origin_map_m
   geometry_msgs/Vector3 ray_direction_map
   geometry_msgs/Point endpoint_map_m
   bool endpoint_xyz_valid
   float64 measured_range_m
   bool source_attitude_valid
   float64 source_roll_rad
   float64 source_pitch_rad
   float64 source_tilt_rad
   bool attitude_compensation_applied
   float64 applied_roll_rad
   float64 applied_pitch_rad
   float64 applied_tilt_rad
   bool classifier_applied
   uint8 classification
   bool volume_matched
   bool confident_face_interior
   bool known_part_valid
   uint8 known_part
   string structure_id
   string opening_id
   string part_id
   float64 expected_range_m
   float64 range_delta_m

   # ObstacleMemoryCellProvenance.msg
   int32 cell_x
   int32 cell_y
   drone_city_nav/ObstacleMemoryHitObservation occupancy_trigger
   drone_city_nav/ObstacleMemoryHitObservation last_hit
   bool endpoint_z_range_valid
   float64 min_endpoint_z_m
   float64 max_endpoint_z_m
   uint64 accepted_hit_count

   # ObstacleMemoryProvenance.msg
   uint32 CURRENT_SCHEMA_VERSION=1
   std_msgs/Header header
   uint32 schema_version
   nav_msgs/MapMetaData grid_info
   uint64 raw_grid_data_hash
   uint64 occupied_cell_count
   drone_city_nav/ObstacleMemoryCellProvenance[] cells
   ```

   Classification/known-part constants со значениями current C++ enums объявить в
   соответствующем `.msg`; adapter делает exhaustive C++ enum conversion и reject unknown
   values. `accepted_hit_count` остаётся exact ROS `uint64`, без saturation policy.
   Parser принимает только `schema_version==CURRENT_SCHEMA_VERSION==1`; любое несовместимое
   изменение fields/hash semantics требует bump version и до явной migration отклоняется
   как `schema_invalid`.

   `raw_grid_data_hash` использует versioned-by-schema FNV-1a по exact row-major bytes
   готового `OccupancyGrid.data`; producer и consumer вызывают один pure helper над ROS
   message, поэтому float conversion geometry не может разнести hash. `grid_info`
   копируется целиком из опубликованного `OccupancyGrid.info` и отдельно переносит width,
   height, resolution, full origin pose и map-load stamp. Поэтому same-stamp
   geometry/content mismatch и zero-record snapshot проверяемы. Existing core
   `prohibitedFingerprint()` для этого wire identity не переиспользовать: он хеширует
   pre-serialization `double` bounds и может отличаться после `MapMetaData.resolution`
   conversion в `float32`.

   ROS build wiring:

   - `find_package(rosidl_default_generators REQUIRED)` и
     `rosidl_generate_interfaces(${PROJECT_NAME} ... DEPENDENCIES builtin_interfaces
     geometry_msgs nav_msgs std_msgs)` до consuming targets;
   - получить `rosidl_typesupport_cpp` target и связать generated types с
     `drone_city_nav_ros_adapters`, `obstacle_memory_node`, `planner_node` и adapter tests;
   - `package.xml`: `rosidl_default_generators`, `rosidl_default_runtime` и
     `member_of_group rosidl_interface_packages`; existing message dependencies оставить;
   - `ament_export_dependencies(rosidl_default_runtime)`.

   Adapter строит owning core `MemoryProvenanceSnapshot` и валидирует schema version,
   finite geometry/origin quaternion, hash/count, duplicate/out-of-bounds cells, unknown
   enums, invalid validity/value combinations, запрет accepted provenance с
   `classification=EXPECTED_STATIC` и endpoint-to-cell consistency. Malformed message
   отклоняется целиком; empty `cells` валиден только при
   `occupied_cell_count==0` и matching parent geometry/hash.

4. **Публиковать grid и provenance как один логический snapshot с общим stamp.**

   Файлы/anchors:
   `drone_city_nav/src/obstacle_memory_node.cpp:221`, `:328`, `:444`, `:552`;
   `drone_city_nav/config/urban_mvp.yaml:1`.

   - Добавить параметр/topic
     `obstacle_memory_provenance_topic: /drone_city_nav/obstacle_memory_provenance`.
   - Publisher: reliable transient-local depth 1, аналогично memory grid.
   - После `integrateScan()` один раз получить `publication_stamp = now()` и сначала
     построить `nav_msgs::msg::OccupancyGrid`. Provenance builder принимает готовые
     `grid_msg.header`, `grid_msg.info` и `rawGridDataHash(grid_msg.data)`; он не
     пересоздаёт geometry независимо. Оба сообщения получают byte-for-byte одинаковые
     stamp/frame/MapMetaData;
     `now()` внутри отдельных builders больше не вызывается.
   - Публиковать полный sparse snapshot active occupied provenance при каждом memory-grid
     publish, чтобы exact-stamp matching был возможен. Добавить throttled counters:
     occupied/provenance count, serialized bytes, invalid-Z count.
   - Existing `PASSAGE_MEMORY_HIT` оставить unthrottled, но заполнять из trigger
     authoritative record и дополнить acquisition/receive timestamp source, source
     attitude и explicitly applied projection attitude.
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
   - `onMemoryGrid()` после успешной conversion сохраняет immutable
     `MemoryGridSnapshotIdentity`: exact header stamp/frame, complete `msg.info`, computed
     raw-grid data hash и occupied count рядом с `memory_grid_`; invalid grid не меняет
     accepted identity.
   - Provenance callback использует adapter parser и кладёт successful snapshot в
     bounded cache последних 4 full identities. Cache должен быть pure/testable helper,
     не ROS callback-only logic; одинаковый stamp с разной geometry/hash не должен
     перезаписывать корректный candidate до matcher lookup.
   - Lookup разрешён только при exact header stamp/frame, supported schema version,
     exact MapMetaData geometry (`width`, `height`, `resolution`, full origin pose;
     map-load stamp также должен совпасть), exact `raw_grid_data_hash`, exact
     `occupied_cell_count` и полной record consistency: cells уникальны/in-bounds, каждый
     record cell действительно occupied, record count равен occupied count, valid
     trigger/last XY map back to record cell. Любое нарушение отклоняет snapshot целиком.
     Эти parent fields обеспечивают проверку geometry/content и при empty `cells`.
   - Late/out-of-order provenance может сопоставиться благодаря cache; arrival order не
     является identity.
   - Отсутствие publisher/topic, malformed snapshot и mismatch не меняют
     `memory_grid_seen_`, planning readiness, source merge или A*; сохраняется status/reason
     для diagnostics (`not_received`, `stamp_mismatch`, `frame_mismatch`,
     `schema_invalid`, `geometry_mismatch`, `cell_missing`).
   - Cache ограничен четырьмя snapshots и очищается/пересматривается при смене frame/grid
     geometry; key — полный `MemoryGridSnapshotIdentity` (stamp, frame, MapMetaData,
     raw-grid data hash), duplicate exact identity заменяется, same-stamp mismatch хранится
     отдельно до bounded eviction. Никакого unbounded накопления.

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
     -R '^(lidar_projection_test|obstacle_memory_test|obstacle_memory_provenance_ros_test|memory_provenance_cache_test|planner_memory_provenance_format_test|known_static_lidar_ingestion_test|known_static_lidar_planning_grid_integration_test)$'
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
- Timing/effective-attitude unit cases: valid stamp + increment; zero/invalid stamp;
  zero increment; negative/NaN/overflowing increment; attitude compensation enabled,
  disabled и `attitude_valid=false`. Проверить, что applied roll/pitch/tilt совпадают с
  единственным projection pass, а stale source attitude не помечается applied.
- Repeated accepted hit с другим origin altitude, но в той же XY cell: trigger неизменен,
  last обновлён, min/max расширены, count увеличен, второго transition нет.
- Достаточные misses переводят occupied в unknown/free и удаляют provenance; новый hit
  после re-entry создаёт новый trigger.
- `reset()` очищает grid/scores/provenance.
- Existing expected-known-static suppression не создаёт provenance.
- Invalid 3D projection явно отмечается и не меняет существующий 2D occupancy result.
- Existing raw-memory/no-inflation/scoring tests остаются без semantic expectation changes.

## 2. С лёгким рефакторингом — включено в задачу

- Generated-message/core round trip для всех полей, exact UINT64 max hit count, empty
  snapshot и invalid-Z record.
- Negative adapter tests: unsupported schema version, duplicate/out-of-bounds/non-occupied
  cell, invalid enum/flag combination, non-finite geometry, bad quaternion,
  occupied-count mismatch, raw-grid-data-hash mismatch и endpoint/cell mismatch.
- Pure bounded-cache/matcher tests: provenance-before-grid, grid-before-provenance,
  exact match, stale stamp, frame mismatch, same-stamp width/height/resolution/origin
  mismatch, same-stamp raw-grid-data-hash mismatch, occupied-count mismatch, zero-occupied
  snapshot with matching geometry, zero-occupied snapshot with mismatched geometry,
  cache eviction.
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

- **Bandwidth/CPU:** generated provenance record заметно больше OccupancyGrid cell. Полный
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
- **Generated interface wiring:** package впервые одновременно генерирует ROS interfaces и
  собирает runtime targets. Решение: explicit rosidl generator/runtime dependencies,
  typesupport linkage всех consumers и clean container build до scoped tests.
- **Malformed wire data:** generated messages убирают byte-offset UB, но semantic mismatch
  всё ещё возможен. Решение: один adapter, exact schema/geometry/hash validation,
  reject-whole-snapshot и exhaustive negative tests.
- **Counter overflow:** core и wire используют UINT64 без narrowing. Serializer/deserializer
  round-trip тестируется на `std::numeric_limits<std::uint64_t>::max()`.
- **Source-size growth:** node files уже 628/719 строк. Serialization/cache/formatter вынести
  в отдельные modules; contract limit 1000 строк должен остаться зелёным.

# Open questions

1. **Использовать PointCloud2, отдельный interfaces package или custom messages в текущем
   package?**

   Recommended decision: custom messages внутри `drone_city_nav` package. Rationale:
   PointCloud2 не переносит parent grid geometry для zero-record snapshot; отдельный package
   потребовал бы менять approved `--packages-select drone_city_nav`; same-package rosidl
   сохраняет typed contract, exact UINT64 и текущий build workflow. Подтверждение решения:
   clean container build должен сгенерировать headers/typesupport, а round-trip и node-link
   tests должны пройти до принятия runtime wiring.

2. **Какой stamp является identity snapshot-а?**

   Recommended decision: один `publication_stamp = obstacle_memory_node.now()` для grid и
   provenance headers; per-beam acquisition stamp хранится внутри records и не используется
   как snapshot identity. Rationale: grid является накопленным state после целого callback,
   поэтому scan stamp отдельного beam не идентифицирует весь published snapshot. Для
   подтверждения matcher tests должны покрыть одинаковый stamp с geometry/hash mismatch.

3. **Публиковать provenance реже для экономии bandwidth?**

   Recommended decision: в этой задаче публиковать вместе с каждым memory grid update.
   Rationale: более редкая публикация делает exact-stamp provenance систематически недоступной
   для большинства grid snapshots и нарушает основной forensic contract. Сначала добавить
   byte/count diagnostics; throttling/delta protocol допустим отдельной задачей только по
   измеренным performance данным. Подтверждение: runtime counters record count/serialized
   size должны показать приемлемый объём в optional smoke; это не блокирует unit completion.

4. **Что делать с snapshot, где валидна часть records?**

   Recommended decision: reject whole snapshot с конкретной причиной; planning продолжает
   работать по grid. Rationale: partial acceptance может создать правдоподобную, но неполную
   provenance и скрыть именно blocker cell. Diagnostic truth важнее частичного обогащения.
   Подтверждение: negative adapter tests на каждый mismatch reason.

5. **Нужен ли headless run для completion?**

   Recommended decision: нет, не как mandatory gate. Rationale: задача не меняет flight
   behavior, а все lifecycle/serialization/ordering contracts портируемо покрываются unit tests.
   Один optional smoke полезен только для подтверждения publication/log wiring и не заменяет
   автоматические tests. Изменить решение следует только если automated node-link/topic
   contract не способен подтвердить фактическую publication wiring.

Объективных blockers для реализации в текущем workspace не обнаружено.
