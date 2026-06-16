# Context

Нужно подготовить рефакторинг `drone_city_nav/src/planner_node.cpp`, не меняя
поведение планировщика. Текущий `PlannerNode` совмещает ROS-обвязку,
объявление параметров, преобразование сообщений, загрузку статической карты,
debug-публикацию и часть orchestration-логики. В рамках этого плана
рефакторятся три зоны:

1. Чтение параметров и сбор `PlannerNodeConfig`.
2. ROS message adapters для `OccupancyGrid2D` и `nav_msgs::Path`.
3. `StaticMapSource` и debug publisher/builder для статической карты.

Явный не-гол этого шага: не выносить `replanAndPublish()` /
planning-cycle orchestration в отдельный runner. Это отдельный, более рискованный
рефакторинг, потому что он затрагивает принятие решений `hold/reuse/replan/fallback`.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует, поэтому входных investigation-заметок
для этого плана нет.

Перед планированием были прочитаны:

- `.agent-io/inbox.txt` с контрактом workflow `plan`.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`.
- `README.md`, `CONTRIBUTING.md`, `CPP_BEST_PRACTICES.md`, `Makefile`.
- `drone_city_nav/CMakeLists.txt`.
- Релевантные anchors в `drone_city_nav/src/planner_node.cpp`.

Notion/GitLab не использовались: в пользовательском запросе нет Notion task,
GitLab MR или review/discussion context, а политика Notion указана как optional.

# Detected stack/profiles

Проект является ROS 2 workspace на C++:

- build system: `colcon` + `ament_cmake`, package `drone_city_nav`.
- C++ standard: `cxx_std_20`.
- Test framework: `ament_cmake_gtest` + `ctest`.
- Симуляционный runtime: Gazebo + PX4 SITL + ROS 2.
- Форматирование/качество: `.clang-format`, `scripts/format_cpp_changed.sh`,
  `scripts/check_cpp_quality.sh`.

Применимые project profiles:

- `generic.md`: обязателен для любого workspace.
- `cpp.md`: выбран из-за `CMakeLists.txt`, `Makefile`, `.cpp/.hpp` и ROS 2 C++
  package.

Rust profile не применялся: целевые изменения находятся в C++/ROS 2 части, а
`Cargo.toml` для этого workspace не является основным build manifest.

# Repo-approved commands found

Основные repo-approved команды из `README.md`, `CONTRIBUTING.md` и `Makefile`:

```bash
./scripts/dev_shell.sh
make build
make test
make test-scripts
make quality
make format
make format-check
make sim-gui
make sim-headless
```

Эквивалентные documented команды:

```bash
colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ctest --test-dir build/drone_city_nav --output-on-failure
./scripts/format_cpp_changed.sh
./scripts/check_cpp_quality.sh
HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
```

Для долгих проверок использовать `runlim`:

```bash
/home/formi/.local/bin/runlim -- make build
/home/formi/.local/bin/runlim -- ctest --test-dir build/drone_city_nav --output-on-failure
/home/formi/.local/bin/runlim -- ./scripts/check_cpp_quality.sh
```

# Affected components

Основные места в `drone_city_nav/src/planner_node.cpp`:

- `PlannerNode::PlannerNode()` около `drone_city_nav/src/planner_node.cpp:50`:
  длинный constructor с `declare_parameter`, subscriptions, publishers, timers и
  initial logging.
- `occupancyGridFromMessage()` около `drone_city_nav/src/planner_node.cpp:398`:
  `nav_msgs::msg::OccupancyGrid` -> `OccupancyGrid2D`.
- `resolveStaticMapPath()` около `drone_city_nav/src/planner_node.cpp:444`:
  path resolution через current directory и `ament_index_cpp`.
- `loadConfiguredStaticMap()` около `drone_city_nav/src/planner_node.cpp:474`:
  load/rasterize/log static map.
- `makeOccupancyGridMessage()` около `drone_city_nav/src/planner_node.cpp:869`:
  `OccupancyGrid2D` -> `nav_msgs::msg::OccupancyGrid`.
- `publishStaticMapDebug()` около `drone_city_nav/src/planner_node.cpp:899` и
  `publishStaticMapPoints()` около `drone_city_nav/src/planner_node.cpp:920`:
  static map debug outputs.
- `publishPath()` около `drone_city_nav/src/planner_node.cpp:979`:
  `std::vector<Point2>` -> `nav_msgs::msg::Path` и first waypoint.

Планируемые новые/изменённые файлы:

- `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
- `drone_city_nav/src/planner_node_config.cpp`
- `drone_city_nav/tests/planner_node_config_test.cpp`
- `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`
- `drone_city_nav/src/ros_conversions.cpp`
- `drone_city_nav/tests/ros_conversions_test.cpp`
- `drone_city_nav/include/drone_city_nav/static_map_source.hpp`
- `drone_city_nav/src/static_map_source.cpp`
- `drone_city_nav/tests/static_map_source_test.cpp`
- `drone_city_nav/include/drone_city_nav/static_map_debug.hpp`
- `drone_city_nav/src/static_map_debug.cpp`
- `drone_city_nav/tests/static_map_debug_test.cpp`
- `drone_city_nav/CMakeLists.txt`
- `drone_city_nav/src/planner_node.cpp`

# Implementation steps

1. Добавить `PlannerNodeConfig` и loader параметров.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`
   - `drone_city_nav/tests/planner_node_config_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Anchor: constructor `PlannerNode::PlannerNode()` около
   `drone_city_nav/src/planner_node.cpp:50`.

   Ожидаемый результат: объявление всех ROS parameters и clamp/default-логика
   переезжают в одну функцию `loadPlannerNodeConfig(rclcpp::Node&)`.
   Constructor перестаёт быть простынёй `declare_parameter` и получает готовую
   структуру.

   Минимальный контракт:

   ```cpp
   struct PlannerNodeConfig {
     std::string frame_id{"map"};
     Point2 start{};
     Point2 goal{85.0, 0.0};
     double cruise_altitude_m{12.0};
     double inflation_radius_m{2.5};

     PlannerCoreConfig planner_core;
     PathSmoothingConfig path_smoothing;
     PlanningGridBuilderConfig planning_grid_builder;
     LidarProjectionConfig lidar_projection;

     StaticMapSourceConfig static_map;
     PlannerTopics topics;
     PlannerTiming timing;
     PlannerFallbackConfig fallback;
     InitialPoseConfig initial_pose;
   };

   [[nodiscard]] PlannerNodeConfig loadPlannerNodeConfig(rclcpp::Node& node);
   ```

   Важно: `PlannerNodeConfig` может жить в ROS-facing части, потому что
   `loadPlannerNodeConfig()` зависит от `rclcpp`. Его не нужно включать в
   `drone_city_nav_core`, если из-за этого core начнёт зависеть от ROS.

   Тесты:

   - `PlannerNodeConfigTest.UsesDocumentedDefaults`: проверить defaults
     `frame_id`, `goal`, `cruise_altitude_m`, `use_static_map`,
     `use_obstacle_memory`, `use_current_lidar_obstacles`.
   - `PlannerNodeConfigTest.ClampsUnsafeValues`: отрицательные/слишком большие
     параметры clamp-ятся так же, как сейчас в constructor.
   - `PlannerNodeConfigTest.BuildsNestedCoreConfigs`: `astar_config`,
     `path_smoothing_config`, `PlanningGridBuilderConfig`,
     `LidarProjectionConfig` получают ожидаемые значения.

2. Подключить `PlannerNodeConfig` в `PlannerNode` без изменения поведения.

   Файлы:

   - `drone_city_nav/src/planner_node.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Anchor: `PlannerNode::PlannerNode()` около
   `drone_city_nav/src/planner_node.cpp:50`, `updatePlannerCoreConfig()` около
   `drone_city_nav/src/planner_node.cpp:295`.

   Ожидаемый результат: constructor выглядит как orchestration:

   ```cpp
   PlannerNode() : Node{"planner_node"} {
     config_ = loadPlannerNodeConfig(*this);
     applyConfig(config_);
     createSubscriptions(config_.topics);
     createPublishers(config_.topics);
     loadConfiguredStaticMap();
     createTimers(config_.timing);
     logPlannerReady(config_);
   }
   ```

   На первом проходе допустимо сохранить существующие private members
   (`frame_id_`, `goal_`, `astar_config_` и т.п.) и заполнять их из `config_`.
   Это снижает риск большого diff. Важно не менять имена ROS parameters и их
   defaults.

   Тесты:

   - Использовать тесты из шага 1 как regression guard.
   - После сборки убедиться, что `planner_node` линкуется с новым source-файлом.

   Логи:

   - Сохранить существующие ключевые log markers `Planner ready`,
     `Planner subscriptions`, `Planner obstacle sources`, `Planning summary`.
   - Если добавляется новый config-log, он должен быть compact и useful для
     headless: `Planner config loaded: frame='map' static_map=true memory=true current_lidar=true`.

3. Вынести ROS message adapters в `ros_conversions`.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`
   - `drone_city_nav/src/ros_conversions.cpp`
   - `drone_city_nav/tests/ros_conversions_test.cpp`
   - `drone_city_nav/src/planner_node.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Anchors:

   - `occupancyGridFromMessage()` около
     `drone_city_nav/src/planner_node.cpp:398`.
   - `makeOccupancyGridMessage()` около
     `drone_city_nav/src/planner_node.cpp:869`.
   - `publishPath()` около `drone_city_nav/src/planner_node.cpp:979`.

   Ожидаемый результат: node вызывает адаптеры, а conversion semantics покрыты
   unit-тестами.

   Минимальный контракт:

   ```cpp
   struct OccupancyGridFromRosConfig {
     int occupied_threshold{65};
     int free_threshold{0};
   };

   struct OccupancyGridToRosConfig {
     std::string frame_id{"map"};
     rclcpp::Time stamp{};
     bool include_inflation{true};
   };

   [[nodiscard]] std::optional<OccupancyGrid2D>
   occupancyGridFromRos(const nav_msgs::msg::OccupancyGrid& msg,
                        const OccupancyGridFromRosConfig& config);

   [[nodiscard]] nav_msgs::msg::OccupancyGrid
   occupancyGridToRos(const OccupancyGrid2D& grid,
                      const OccupancyGridToRosConfig& config);

   [[nodiscard]] nav_msgs::msg::Path
   pathToRos(std::span<const Point2> points,
             const std_msgs::msg::Header& header,
             double altitude_m);
   ```

   Если `rclcpp::Time` в header создаёт лишнюю dependency для тестов, заменить
   config на `std_msgs::msg::Header header`.

   Тесты:

   - `RosConversionsTest.RejectsInvalidOccupancyGridMetadata`: resolution <= 0,
     zero width/height, слишком большой размер.
   - `RosConversionsTest.RejectsMismatchedOccupancyGridDataSize`.
   - `RosConversionsTest.ConvertsUnknownFreeOccupiedByThresholds`: `-1`, free,
     occupied.
   - `RosConversionsTest.SerializesInflatedCellsOnlyWhenRequested`: occupied=100,
     inflated=80 только при `include_inflation=true`, free=0, unknown=-1.
   - `RosConversionsTest.BuildsPathWithHeaderAltitudeAndIdentityOrientation`.
   - `RosConversionsTest.BuildsEmptyPathWithoutWaypointSideEffects`: adapter
     возвращает empty path, а side effects остаются в node.

   Логи:

   - Adapter не должен логировать. Логи об invalid incoming memory grid остаются
     в `PlannerNode`, чтобы throttle и logger были в одном месте.
   - Adapter возвращает `std::nullopt`/status, node пишет существующие warnings.

4. Вынести path resolution и загрузку статической карты в `StaticMapSource`.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/static_map_source.hpp`
   - `drone_city_nav/src/static_map_source.cpp`
   - `drone_city_nav/tests/static_map_source_test.cpp`
   - `drone_city_nav/src/planner_node.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Anchors:

   - `resolveStaticMapPath()` около
     `drone_city_nav/src/planner_node.cpp:444`.
   - `loadConfiguredStaticMap()` около
     `drone_city_nav/src/planner_node.cpp:474`.

   Ожидаемый результат: node остаётся владельцем ROS logger/publishers, но
   resolution/load/rasterize result приходит из отдельного provider.

   Минимальный контракт:

   ```cpp
   enum class StaticMapSourceStatus {
     kDisabled,
     kLoaded,
     kLoadFailed,
   };

   struct StaticMapSourceConfig {
     bool enabled{true};
     std::filesystem::path configured_path{"worlds/generated_city.map2d"};
     std::filesystem::path package_share_directory;
     std::string expected_frame_id{"map"};
     double min_blocking_height_m{0.0};
   };

   struct StaticMapSourceResult {
     StaticMapSourceStatus status{StaticMapSourceStatus::kDisabled};
     std::filesystem::path resolved_path;
     std::optional<OccupancyGrid2D> grid;
     std::string map_frame_id;
     std::size_t rectangles{0};
     std::size_t occupied_cells{0};
     bool frame_matches{true};
     std::string error_message;
   };

   [[nodiscard]] std::filesystem::path
   resolveStaticMapPath(const std::filesystem::path& configured_path,
                        const std::filesystem::path& package_share_directory);

   [[nodiscard]] StaticMapSourceResult
   loadStaticMapSource(const StaticMapSourceConfig& config);
   ```

   `ament_index_cpp::get_package_share_directory("drone_city_nav")` лучше
   оставить в node или в маленьком ROS wrapper, а в `StaticMapSourceConfig`
   передавать уже вычисленный `package_share_directory`. Так тесты не зависят от
   ament index и проще покрывают path resolution.

   Тесты:

   - `StaticMapSourceTest.DisabledDoesNotTouchFilesystem`: `enabled=false`
     возвращает `kDisabled`.
   - `StaticMapSourceTest.KeepsAbsoluteConfiguredPath`.
   - `StaticMapSourceTest.ResolvesExistingRelativePathFromCwd`.
   - `StaticMapSourceTest.ResolvesPackageShareCandidate`.
   - `StaticMapSourceTest.ResolvesPackageWorldsFallbackByFilename`.
   - `StaticMapSourceTest.LoadsAndRasterizesValidMap`: создать временный
     `.map2d`, проверить `rectangles`, `occupied_cells`, `grid`.
   - `StaticMapSourceTest.ReportsFrameMismatchWithoutFailingLoad`.
   - `StaticMapSourceTest.ReportsLoadFailureWithErrorMessage`.

   Логи:

   - Node должен сохранить существующий log:
     `Static city map loaded: path=... frame=... rectangles=... occupied_cells=...`.
   - Для headless диагностики добавить/сохранить лог disabled/failure:
     `Static city map source is disabled` и
     `Failed to load static city map: path=... error=...`.

5. Вынести static map debug message building в `static_map_debug`.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/static_map_debug.hpp`
   - `drone_city_nav/src/static_map_debug.cpp`
   - `drone_city_nav/tests/static_map_debug_test.cpp`
   - `drone_city_nav/src/planner_node.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Anchors:

   - `publishStaticMapDebug()` около
     `drone_city_nav/src/planner_node.cpp:899`.
   - `publishStaticMapPoints()` около
     `drone_city_nav/src/planner_node.cpp:920`.

   Ожидаемый результат: формирование `nav_msgs::msg::OccupancyGrid` и
   `sensor_msgs::msg::PointCloud2` для статической карты тестируется отдельно.
   Node только вызывает builder и публикует сообщения.

   Минимальный контракт:

   ```cpp
   struct StaticMapDebugConfig {
     std_msgs::msg::Header header;
     float point_z_m{0.05F};
   };

   [[nodiscard]] nav_msgs::msg::OccupancyGrid
   staticMapGridMessage(const OccupancyGrid2D& grid,
                        const StaticMapDebugConfig& config);

   [[nodiscard]] sensor_msgs::msg::PointCloud2
   staticMapPointCloud(const OccupancyGrid2D& grid,
                       const StaticMapDebugConfig& config);
   ```

   `staticMapGridMessage()` может переиспользовать
   `occupancyGridToRos(... include_inflation=false)`, чтобы не дублировать
   encoding occupied/free/unknown.

   Тесты:

   - `StaticMapDebugTest.GridMessageUsesStaticMapFrameAndNoInflation`.
   - `StaticMapDebugTest.PointCloudContainsOnlyOccupiedCells`.
   - `StaticMapDebugTest.PointCloudFieldsAreXyzFloat32`.
   - `StaticMapDebugTest.PointCloudRowStepAndDataSizeMatchPointCount`.
   - `StaticMapDebugTest.EmptyMapPublishesValidEmptyPointCloud`.

   Логи:

   - Сохранить log marker `Published static map grid: cells=... occupied=...`.
   - Не логировать внутри pure builder: throttle/logger остаются в node.

6. Обновить `planner_node.cpp`, чтобы он был ROS orchestration layer.

   Файлы:

   - `drone_city_nav/src/planner_node.cpp`

   Ожидаемый результат:

   - Удалены private helpers `occupancyGridFromMessage`,
     `makeOccupancyGridMessage`, `resolveStaticMapPath`, большая часть
     `publishStaticMapPoints`.
   - `onMemoryGrid()` вызывает `occupancyGridFromRos(...)`.
   - `publishOccupancyGrid()` вызывает `occupancyGridToRos(...)`.
   - `publishPath()` вызывает `pathToRos(...)`, а side effects
     `last_valid_path_points_`, `stable_path_blocked_confirmations_`,
     `waypoint_pub_`, `logPathUpdate()` остаются в node.
   - `loadConfiguredStaticMap()` вызывает `loadStaticMapSource(...)`, переносит
     result в `static_grid_`, `static_map_rectangles_`,
     `static_map_occupied_cells_` и пишет прежние логи.
   - `publishStaticMapDebug()` строит messages через `static_map_debug` и
     публикует их.

   Псевдокод:

   ```cpp
   void onMemoryGrid(const nav_msgs::msg::OccupancyGrid& msg) {
     auto converted = occupancyGridFromRos(
         msg, OccupancyGridFromRosConfig{occupied_threshold_, free_threshold_});
     if (!converted.has_value()) {
       RCLCPP_WARN_THROTTLE(...);
       return;
     }
     memory_grid_ = std::move(converted);
   }
   ```

   Этот шаг не должен менять `replanAndPublish()` decision logic.

7. Обновить CMake targets без случайного расширения dependency graph.

   Файл:

   - `drone_city_nav/CMakeLists.txt`

   Ожидаемый результат:

   - Если helpers зависят от ROS message types, подключить их к `planner_node`
     и соответствующим gtests как source files или через отдельную библиотеку
     `drone_city_nav_ros_adapters`.
   - Не добавлять `rclcpp`, `nav_msgs`, `sensor_msgs` в
     `drone_city_nav_core`, если core сейчас может оставаться ROS-free.

   Предпочтительный вариант:

   ```cmake
   add_library(
     drone_city_nav_ros_adapters
     src/planner_node_config.cpp
     src/ros_conversions.cpp
     src/static_map_debug.cpp
     src/static_map_source.cpp)
   target_link_libraries(drone_city_nav_ros_adapters drone_city_nav_core)
   ament_target_dependencies(
     drone_city_nav_ros_adapters
     ament_index_cpp
     geometry_msgs
     nav_msgs
     rclcpp
     sensor_msgs
     std_msgs)
   ```

   Если `static_map_source.cpp` получится ROS-free, он может быть добавлен в
   `drone_city_nav_core` позже отдельным маленьким diff, но не обязательно в
   рамках этого рефакторинга.

8. Сохранить и при необходимости дополнить headless-диагностику.

   Файлы:

   - `drone_city_nav/src/planner_node.cpp`
   - при необходимости `docs/MVP_SIMULATION.md`

   Ожидаемый результат: после рефакторинга по логам всё ещё можно понять:

   - какие obstacle sources включены;
   - загружена ли static map, сколько rectangles/cells;
   - какие topics подписаны/публикуются;
   - почему memory grid rejected;
   - публикуется ли static map debug grid/pointcloud;
   - какой источник использован в planning summary.

   Новые логи не должны быть шумными в каждом timer tick. Для повторяющихся
   warnings использовать throttle.

# Verification plan

После каждого куска C++-рефакторинга:

```bash
./scripts/format_cpp_changed.sh
```

Scoped build/test для изменённого package:

```bash
/home/formi/.local/bin/runlim -- make build
/home/formi/.local/bin/runlim -- ctest --test-dir build/drone_city_nav --output-on-failure -R 'planner_node_config|ros_conversions|static_map_source|static_map_debug'
```

Перед финальным commit реализации:

```bash
/home/formi/.local/bin/runlim -- ./scripts/check_cpp_quality.sh
```

Если изменения затронут scripts:

```bash
/home/formi/.local/bin/runlim -- make test-scripts
```

Headless smoke после интеграции static map/refactor:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=180 ./scripts/run_city_mvp.sh'
```

Если локальная среда не поддерживает active Gazebo/PX4/GPU lidar run, выполнить
static-map-only fallback и явно записать skipped active-lidar run:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=180 ENABLE_OBSTACLE_MEMORY=false ENABLE_CURRENT_LIDAR=false ./scripts/run_city_mvp.sh'
```

# Testing strategy

Категория 1: без рефакторинга.

- Перед изменениями можно запустить текущие tests как baseline:
  `/home/formi/.local/bin/runlim -- make test`.
- Для планового раунда без кода достаточно non-mutating checks по artifact:
  `test -s PLAN.md` и `git diff --check`.

Категория 2: лёгкий рефакторинг.

- `planner_node_config_test`: defaults, clamps, nested config mapping.
- `ros_conversions_test`: happy-path, invalid metadata, data size mismatch,
  threshold edge cases, inflated cell serialization, empty/non-empty path.
- `static_map_source_test`: disabled, absolute/relative/package path resolution,
  valid map load, frame mismatch, load failure.
- `static_map_debug_test`: static grid encoding, pointcloud fields, row/data size,
  empty pointcloud.
- Запускать scoped `ctest -R` после каждого нового test target.

Категория 3: тяжёлый рефакторинг.

- Полный `./scripts/check_cpp_quality.sh`, потому что меняются headers,
  `CMakeLists.txt`, ROS-facing adapters и executable link graph.
- Headless mission smoke с `MISSION_CHECK=1`, потому что static map loading,
  obstacle sources и debug topics участвуют в runtime orchestration.
- При возможности дополнительно проверить GUI/RViz вручную, но это не замена
  автотестам и должно считаться только визуальным sanity check.

# Risks and tradeoffs

- Можно случайно поменять ROS parameter names/defaults. Это ломает launch/config
  без compile-time ошибки. Митигировать через `PlannerNodeConfig` defaults/clamp
  tests и review списка параметров.
- Можно протащить ROS dependencies в `drone_city_nav_core`. Это ухудшит
  тестируемость core и сборочную архитектуру. Митигировать отдельной
  `drone_city_nav_ros_adapters` library или source-level linking только в
  executable/tests.
- Conversion helpers могут потерять текущую семантику `unknown=-1`,
  `free=0`, `inflated=80`, `occupied=100`. Митигировать unit-тестами
  `ros_conversions_test`.
- Static map path resolution может начать искать файл в другом порядке. Это
  риск для dev shell/install-space сценариев. Митигировать тестами absolute,
  relative cwd, package share и package `worlds` fallback.
- `PointCloud2` легко испортить через неверные `point_step`, `row_step`,
  `fields` или packing float. Митигировать `static_map_debug_test`.
- Изменение log text может сломать headless-анализ, если scripts ждут markers.
  Митигировать сохранением ключевых фраз и outbox-отчётом о любых изменённых
  markers.
- Большой diff в constructor может смешать mechanical refactor и behavioral
  changes. Митигировать маленькими коммитами: config, conversions, static map,
  integration.

# Что могло сломаться

- Поведение планировщика: из-за неверно перенесённых config values A* может
  получить другие `inflation_radius_m`, `nearest_free_radius_cells`,
  `turn_cost_weight`, `use_static_map`, `use_current_lidar_obstacles`.
  Проверка: `planner_node_config_test`, `planner_core_test`,
  `planning_grid_builder_test`, headless `MISSION_CHECK=1`.
- ROS API/контракты: topic names, frame id, occupancy values, path altitude или
  first waypoint могут измениться. Проверка: `ros_conversions_test`,
  existing launch smoke, RViz topic visibility.
- Static map integration: карта может не найтись в source tree/install space,
  может игнорироваться frame mismatch или неверно считаться occupied cells.
  Проверка: `static_map_source_test`, `static_city_map_test`, static-only
  headless smoke.
- Debug visualizations: green static map в RViz и pointcloud могут исчезнуть
  или стать пустыми из-за ошибки message builder. Проверка:
  `static_map_debug_test` и headless log `Published static map grid`.
- Производительность/ресурсы: лишнее копирование grid/messages в adapters может
  добавить нагрузку на timer. Проверка: headless logs на стабильность timer и
  отсутствие роста warning/error rate; при необходимости передавать grid по
  `const&` и возвращать messages move-enabled.
- Build/link: новый adapter target может не получить нужные ROS dependencies или
  headers. Проверка: `/home/formi/.local/bin/runlim -- make build`.

# Open questions

- Делать `drone_city_nav_ros_adapters` отдельной library или подключать новые
  `.cpp` напрямую к `planner_node` и gtests? Рекомендация: отдельная library,
  потому что несколько tests будут использовать одни и те же helpers.
- Должен ли `StaticMapSource` полностью избегать ROS dependencies? Рекомендация:
  да, передавать `package_share_directory` извне и тестировать provider как pure
  C++ code. `ament_index_cpp` оставить в node/config layer.
- Обновлять ли `docs/MVP_SIMULATION.md` в том же рефакторинге? Рекомендация:
  только если изменятся topic names, log markers или documented run flags.
- Нужно ли в этот же цикл выносить `replanAndPublish()` в
  `PlannerCycleRunner`? Рекомендация: нет, это отдельный тяжёлый шаг после
  adapter/static-map refactor, иначе будет сложно отделить mechanical changes от
  behavioral changes.
