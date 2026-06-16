# Context

Нужно детально спланировать рефакторинг трёх крупных C++ ROS 2 node-файлов:

- `drone_city_nav/src/planner_node.cpp` — 1632 строки.
- `drone_city_nav/src/lidar_debug_node.cpp` — 1196 строк.
- `drone_city_nav/src/px4_offboard_node.cpp` — 1173 строки.

Цель — уменьшить связность, вынести проверяемую доменную логику из ROS node-классов в `drone_city_nav_core`, по возможности приблизить каждый node-файл к размеру менее 1000 строк, не ломая Gazebo + ROS 2 + PX4 SITL MVP. Рефакторинг не должен быть «разбиением ради красоты»: каждый вынесенный модуль должен иметь понятный контракт, автотесты и полезные логи для headless-отладки.

# Investigation context

`INVESTIGATION.md` отсутствует. Существующего `PLAN.md` перед началом не было, поэтому этот файл создан как новый план.

Проведённое локальное исследование:

- Обязательные протоколы оркестратора прочитаны:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- Notion task и GitLab MR в пользовательском промпте не указаны; `notion_policy=optional`, поэтому чтение Notion/GitLab не требуется и не выполнялось.
- Прочитаны repo docs и build files:
  - `README.md`
  - `CONTRIBUTING.md`
  - `CPP_BEST_PRACTICES.md`
  - `Makefile`
  - `drone_city_nav/CMakeLists.txt`
  - `drone_city_nav/package.xml`
- Прочитаны релевантные code anchors:
  - `drone_city_nav/src/planner_node.cpp:564` `buildPlanningGrid`
  - `drone_city_nav/src/planner_node.cpp:642` `replanAndPublish`
  - `drone_city_nav/src/planner_node.cpp:739` `computePathOnGrid`
  - `drone_city_nav/src/planner_node.cpp:1290` `pathSegmentIsUnblocked`
  - `drone_city_nav/src/planner_node.cpp:1302` `pathSegmentOccupiedLengthM`
  - `drone_city_nav/src/planner_node.cpp:1320` `closestPathProjection`
  - `drone_city_nav/src/planner_node.cpp:1356` `remainingPathFromCurrentPose`
  - `drone_city_nav/src/planner_node.cpp:1406` `keepCurrentPathIfStillClear`
  - `drone_city_nav/src/px4_offboard_node.cpp:245` `onPath`
  - `drone_city_nav/src/px4_offboard_node.cpp:322` `lookaheadWaypointIndex`
  - `drone_city_nav/src/px4_offboard_node.cpp:355` `closestPathProjection`
  - `drone_city_nav/src/px4_offboard_node.cpp:390` `lookaheadTargetOnPath`
  - `drone_city_nav/src/px4_offboard_node.cpp:418` `continuityWaypointIndex`
  - `drone_city_nav/src/px4_offboard_node.cpp:686` `advanceWaypointIfNeeded`
  - `drone_city_nav/src/px4_offboard_node.cpp:717` `currentTarget`
  - `drone_city_nav/src/px4_offboard_node.cpp:742` `limitedTarget`
  - `drone_city_nav/src/px4_offboard_node.cpp:758` `smoothedCommandTarget`
  - `drone_city_nav/src/lidar_debug_node.cpp:46` `Image`
  - `drone_city_nav/src/lidar_debug_node.cpp:155` `drawLine`
  - `drone_city_nav/src/lidar_debug_node.cpp:181` `drawDisc`
  - `drone_city_nav/src/lidar_debug_node.cpp:194` `writePpm`
  - `drone_city_nav/src/lidar_debug_node.cpp:411` `writeSnapshot`
  - `drone_city_nav/src/lidar_debug_node.cpp:553` `writeScanCsv`
  - `drone_city_nav/src/lidar_debug_node.cpp:623` `gridWorldToPixel`
  - `drone_city_nav/src/lidar_debug_node.cpp:730` `rememberHitPoints`
  - `drone_city_nav/src/lidar_debug_node.cpp:774` `drawRememberedHits`
  - `drone_city_nav/src/lidar_debug_node.cpp:783` `drawPath`
  - `drone_city_nav/src/lidar_debug_node.cpp:810` `drawScan`
  - `drone_city_nav/src/lidar_debug_node.cpp:849` `writeSummary`
  - `drone_city_nav/src/lidar_debug_node.cpp:957` `publishPointCloud`
  - `drone_city_nav/src/lidar_debug_node.cpp:1006` `baseMarker`
  - `drone_city_nav/src/lidar_debug_node.cpp:1029` `addRangeRing`
  - `drone_city_nav/src/lidar_debug_node.cpp:1048` `addScanRayMarkers`
  - `drone_city_nav/src/lidar_debug_node.cpp:1099` `publishRadarMarkers`
- Прочитаны существующие тестовые паттерны:
  - `drone_city_nav/tests/planner_core_test.cpp:37`
  - `drone_city_nav/tests/offboard_speed_controller_test.cpp:38`
  - `drone_city_nav/tests/lidar_projection_test.cpp:25`

# Detected stack/profiles

Основной стек workspace:

- ROS 2 Jazzy workspace.
- Один основной package: `drone_city_nav`.
- C++20.
- Build system: `ament_cmake` через `colcon`.
- Tests: `ament_cmake_gtest` + `ctest`.
- Simulation stack: Gazebo + PX4 SITL + ROS 2 nodes.
- Python helper scripts есть, но задача затрагивает C++/docs planning, не Python runtime.

Прочитанные project profiles:

- `generic.md` — обязателен для любого workspace.
- `cpp.md` — выбран, потому что есть `Makefile`, `CMakeLists.txt`, `.cpp`, `.hpp`, `compile_commands.json` workflow.

Rust profile не применялся: в workspace нет признаков Rust package (`Cargo.toml`, `Cargo.lock`, `.rs`) в target repo.

# Repo-approved commands found

Repo-approved commands из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- Dev shell:
  - `./scripts/dev_shell.sh`
- Build:
  - `make build`
  - equivalent: `colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Unit tests:
  - `make test`
  - equivalent: `ctest --test-dir build/drone_city_nav --output-on-failure`
- Script tests:
  - `make test-scripts`
- Quality:
  - `make quality`
  - equivalent: `./scripts/check_cpp_quality.sh`
- Format changed C++ files:
  - `./scripts/format_cpp_changed.sh`
- GUI simulation:
  - `make sim-gui`
  - equivalent: `./scripts/run_city_mvp.sh`
- Headless smoke:
  - `make sim-headless`
  - equivalent: `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`
- Full mission validation, documented in repo workflow and existing scripts:
  - `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=<seconds> ./scripts/run_city_mvp.sh`

Для долгих команд в рамках оркестратора использовать `/home/formi/.local/bin/runlim`.
Локальный `runlim` имеет интерфейс `systemd-run`; не использовать `-t` как timeout,
потому что здесь `-t` означает `--pty`. Проверенный базовый синтаксис:

```bash
/home/formi/.local/bin/runlim -- make test
```

Если проверка запускается внутри Docker dev image без интерактивного shell, использовать тот же контейнерный шаблон, что `scripts/dev_shell.sh`, и оборачивать внешний `docker run ... ./scripts/check_cpp_quality.sh` через `runlim`.

# Affected components

1. `planner_node.cpp`

   Сейчас в одном ROS node-классе смешаны:

   - ROS parameters, subscriptions, publishers, timers.
   - Loading/rasterizing static map: `loadConfiguredStaticMap` (`planner_node.cpp:515`).
   - Planning grid source union: `buildPlanningGrid` (`planner_node.cpp:564`).
   - Replan orchestration and fallback: `replanAndPublish` (`planner_node.cpp:642`).
   - A* + smoothing + clearance diagnostics: `computePathOnGrid` (`planner_node.cpp:739`).
   - Current lidar overlay/projection helpers around `currentLidarProjectionPose`, `markCurrentLidarObstacle`, `overlayCurrentLidarHits`.
   - Stable path reuse: `remainingPathFromCurrentPose` (`planner_node.cpp:1356`) and `keepCurrentPathIfStillClear` (`planner_node.cpp:1406`).
   - ROS message creation/publication for occupancy grids, static map points, path and current waypoint.

2. `px4_offboard_node.cpp`

   Сейчас в одном node-классе смешаны:

   - ROS/PX4 subscriptions and publishers.
   - Path ingestion and target continuity: `onPath` (`px4_offboard_node.cpp:245`), `continuityWaypointIndex` (`px4_offboard_node.cpp:418`).
   - Path projection/lookahead: `closestPathProjection` (`px4_offboard_node.cpp:355`), `lookaheadTargetOnPath` (`px4_offboard_node.cpp:390`).
   - Waypoint advancement: `advanceWaypointIfNeeded` (`px4_offboard_node.cpp:686`).
   - Target selection and hold behavior: `currentTarget` (`px4_offboard_node.cpp:717`).
   - Command target smoothing/clamping: `limitedTarget` (`px4_offboard_node.cpp:742`), `smoothedCommandTarget` (`px4_offboard_node.cpp:758`), `clampCommandedTargetToCurrent` (`px4_offboard_node.cpp:977`).
   - Speed limiting already partly extracted into `OffboardSpeedController`.

3. `lidar_debug_node.cpp`

   Сейчас в одном node-классе смешаны:

   - Low-level image primitives: `Pixel`, `Image`, `drawLine`, `drawDisc`, `writePpm` (`lidar_debug_node.cpp:40`, `:46`, `:155`, `:181`, `:194`).
   - Snapshot orchestration: `writeSnapshot` (`lidar_debug_node.cpp:411`).
   - CSV writer: `writeScanCsv` (`lidar_debug_node.cpp:553`).
   - Coordinate mapping/rendering: `gridWorldToPixel`, `worldToPixel`, `drawRememberedHits`, `drawPath`, `drawScan`, `drawDrone` (`lidar_debug_node.cpp:623`, `:657`, `:774`, `:783`, `:810`, `:833`).
   - JSON summary writer: `writeSummary` (`lidar_debug_node.cpp:849`).
   - PointCloud2 and RViz marker generation: `publishPointCloud`, `baseMarker`, `addRangeRing`, `addScanRayMarkers`, `addDroneMarker`, `publishRadarMarkers` (`lidar_debug_node.cpp:957`, `:1006`, `:1029`, `:1048`, `:1082`, `:1099`).

# Implementation steps

1. Вынести pure planning path/stable-path логику из `planner_node.cpp` в новый core-модуль.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/planner_core.hpp`.
   - Добавить `drone_city_nav/src/planner_core.cpp`.
   - Обновить `drone_city_nav/CMakeLists.txt`: добавить `src/planner_core.cpp` в `drone_city_nav_core`.
   - Обновить `drone_city_nav/tests/planner_core_test.cpp`.
   - Уменьшить `drone_city_nav/src/planner_node.cpp`.

   Переносимые anchors:

   - `planner_node.cpp:739` `computePathOnGrid`.
   - `planner_node.cpp:1290` `pathSegmentIsUnblocked`.
   - `planner_node.cpp:1302` `pathSegmentOccupiedLengthM`.
   - `planner_node.cpp:1320` `closestPathProjection`.
   - `planner_node.cpp:1356` `remainingPathFromCurrentPose`.
   - `planner_node.cpp:1406` `keepCurrentPathIfStillClear`.
   - `planner_node.cpp:1454` `pathHasOccupiedCells`.
   - `planner_node.cpp:1481` `pathIsUnblocked`.

   Ожидаемый результат:

   - Новый `PlannerCore` или набор свободных функций без `rclcpp`, `nav_msgs`, `sensor_msgs`.
   - `planner_node.cpp` оставляет ROS I/O, параметры, throttled logs и публикацию сообщений.
   - Stable path reuse становится unit-testable без запуска ROS.

   Контрактный sketch:

   ```cpp
   struct PlannerCoreConfig {
     AStarConfig astar;
     PathSmoothingConfig smoothing;
     int nearest_free_radius_cells{10};
     double stable_path_goal_tolerance_m{3.0};
     double stable_path_reuse_max_deviation_m{12.0};
     double stable_path_blocking_occupied_length_m{4.0};
     int stable_path_blocked_confirmations_required{2};
   };

   struct PathComputationResult {
     AStarResult astar;
     std::vector<GridIndex> smoothed_cells;
     double raw_path_clearance_m;
     double smoothed_path_clearance_m;
   };

   class PlannerCore {
   public:
     std::optional<PathComputationResult>
     computePath(const OccupancyGrid2D& grid, Point2 current, Point2 goal) const;

     StablePathDecision evaluateStablePath(const OccupancyGrid2D& grid,
                                           std::span<const Point2> previous_path,
                                           Point2 current, Point2 goal,
                                           int current_confirmations) const;
   };
   ```

   Новые/обновлённые автотесты:

   - `PlannerCore.ComputePathAdjustsBlockedEndpoints` — start/goal внутри inflation корректно двигаются через `nearestUnblocked`.
   - `PlannerCore.ComputePathRejectsOutOfGridGoal` — negative-path для цели вне grid.
   - `PlannerCore.StablePathKeepsClearRemainingPath` — текущий путь не пересчитывается, если remaining path чистый.
   - `PlannerCore.StablePathRequiresConfirmedOccupiedIntersection` — первое обнаружение препятствия только увеличивает confirmation, второе разрешает replan.
   - `PlannerCore.StablePathRejectsLargeDeviationFromPath` — edge-case для `stable_path_reuse_max_deviation_m`.

2. Вынести построение planning grid из трёх источников в отдельный pure-модуль или тонкий helper с явными status-кодами.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`.
   - Добавить `drone_city_nav/src/planning_grid_builder.cpp`.
   - Обновить `drone_city_nav/CMakeLists.txt`.
   - Обновить/расширить `drone_city_nav/tests/planner_core_test.cpp` или добавить `drone_city_nav/tests/planning_grid_builder_test.cpp`.
   - Упростить `planner_node.cpp:554` `makeBasePlanningGrid` и `planner_node.cpp:564` `buildPlanningGrid`.

   Ожидаемый результат:

   - Логика source enable/ready/geometry mismatch переносится из ROS node.
   - Node отвечает за сбор входов (`static_grid_`, `memory_grid_`, текущий lidar grid) и за преобразование status в logs.
   - `PlanningGridBuildResult` сохраняет diagnostic stats для headless logs.

   Контрактный sketch:

   ```cpp
   enum class PlanningGridStatus {
     kReady,
     kNoEnabledSources,
     kStaticMapEnabledButMissing,
     kNoReadySourceData,
     kMemoryGeometryMismatch,
   };

   struct PlanningGridSources {
     const OccupancyGrid2D* static_grid{nullptr};
     const OccupancyGrid2D* memory_grid{nullptr};
     const OccupancyGrid2D* current_lidar_grid{nullptr};
   };
   ```

   Новые автотесты:

   - `PlanningGridBuilder.StaticOnlyBuildsInflatedGrid`.
   - `PlanningGridBuilder.MemoryGeometryMismatchIsReportedAndSkipped`.
   - `PlanningGridBuilder.NoEnabledSourcesReturnsHoldStatus`.
   - `PlanningGridBuilder.CurrentLidarOverlayWinsAsFreshSource`.

3. Оставить current lidar projection в planner ROS node, но вынести grid-marking часть в core helper.

   Файлы:

   - Добавить функции в `planning_grid_builder.hpp` или отдельный `current_lidar_overlay.hpp`.
   - Использовать существующий `drone_city_nav/include/drone_city_nav/lidar_projection.hpp`.
   - Обновить `planner_node.cpp` вокруг `planner_node.cpp:894` `currentLidarRangeMax`, `planner_node.cpp:898` `currentLidarProjectionPose`, `planner_node.cpp:923` `markCurrentLidarObstacle`.

   Ожидаемый результат:

   - `planner_node.cpp` всё ещё получает `sensor_msgs::LaserScan`, pose и attitude.
   - Core helper принимает уже подготовленный `LaserScan2DView`/projection config и пишет в `OccupancyGrid2D`.
   - Статистика `CurrentLidarStats` остаётся доступной для `Planning summary`.

   Автотесты:

   - Accepted lidar hit marks depth cells behind endpoint.
   - Max-range/non-hit beam does not mark occupied.
   - Altitude-rejected beam increments rejected stats and does not mark grid.

4. Разбить `px4_offboard_node.cpp`: вынести ROS-free path follower/target selector.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`.
   - Добавить `drone_city_nav/src/offboard_path_follower.cpp`.
   - Добавить `drone_city_nav/tests/offboard_path_follower_test.cpp`.
   - Обновить `drone_city_nav/CMakeLists.txt`: добавить source в `drone_city_nav_core` и новый GTest target.
   - Упростить `drone_city_nav/src/px4_offboard_node.cpp`.

   Переносимые anchors:

   - `px4_offboard_node.cpp:303` `closestWaypointIndex`.
   - `px4_offboard_node.cpp:322` `lookaheadWaypointIndex`.
   - `px4_offboard_node.cpp:355` `closestPathProjection`.
   - `px4_offboard_node.cpp:390` `lookaheadTargetOnPath`.
   - `px4_offboard_node.cpp:418` `continuityWaypointIndex`.
   - `px4_offboard_node.cpp:686` `advanceWaypointIfNeeded`.
   - `px4_offboard_node.cpp:717` `currentTarget`.
   - `px4_offboard_node.cpp:742` `limitedTarget`.
   - `px4_offboard_node.cpp:758` `smoothedCommandTarget`.
   - `px4_offboard_node.cpp:977` `clampCommandedTargetToCurrent`.

   Ожидаемый результат:

   - `Px4OffboardNode` остаётся владельцем ROS subscriptions/publishers, arming/offboard commands и `TrajectorySetpoint`.
   - `OffboardPathFollower` работает с `std::vector<Point2>`, текущей map-frame pose и конфигом.
   - `OffboardSpeedController` остаётся отдельным speed limiter, но получает input от follower.

   Контрактный sketch:

   ```cpp
   struct OffboardPathFollowerConfig {
     double acceptance_radius_m{1.0};
     double lookahead_distance_m{8.0};
     double min_lookahead_distance_m{8.0};
     double max_lookahead_distance_m{15.0};
     double path_switch_hysteresis_m{6.0};
     double path_continuity_reuse_radius_m{12.0};
     double max_setpoint_distance_m{12.0};
   };

   struct FollowerState {
     std::size_t waypoint_index{0};
     bool no_path_hold_target_valid{false};
     Point2 commanded_target{};
   };

   class OffboardPathFollower {
   public:
     void setPath(std::vector<Point2> path, Point2 current_position,
                  std::optional<Point2> previous_target);
     FollowerOutput update(const FollowerInput& input);
   };
   ```

   Новые автотесты:

   - `OffboardPathFollower.EmptyPathHoldsCurrentPosition`.
   - `OffboardPathFollower.LookaheadSelectsForwardWaypointThatProgressesToGoal`.
   - `OffboardPathFollower.ContinuityKeepsNearPreviousTarget`.
   - `OffboardPathFollower.AdvancesWaypointAfterAcceptanceRadius`.
   - `OffboardPathFollower.ClampsTargetToMaxSetpointDistance`.
   - `OffboardPathFollower.ZeroTargetStepFallsBackToHoldAtCurrentPosition`.
   - Edge-case: no local position returns configured hold target and does not use stale current position.

5. Разбить `lidar_debug_node.cpp`: вынести image/rendering primitives.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/debug_image.hpp`.
   - Добавить `drone_city_nav/src/debug_image.cpp`.
   - Добавить `drone_city_nav/tests/debug_image_test.cpp`.
   - Обновить `drone_city_nav/CMakeLists.txt`.
   - Упростить `lidar_debug_node.cpp` за счёт удаления `Pixel`, `Image`, `drawLine`, `drawDisc`, `writePpm` из anonymous namespace.

   Переносимые anchors:

   - `lidar_debug_node.cpp:40` `Pixel`.
   - `lidar_debug_node.cpp:46` `Image`.
   - `lidar_debug_node.cpp:155` `drawLine`.
   - `lidar_debug_node.cpp:181` `drawDisc`.
   - `lidar_debug_node.cpp:194` `writePpm`.

   Ожидаемый результат:

   - Low-level rendering testable without ROS, PX4, filesystem-heavy node setup.
   - `lidar_debug_node.cpp` использует `DebugImage` API.

   Автотесты:

   - `DebugImage.SetIgnoresOutOfBounds`.
   - `DebugImage.DrawLineDrawsEndpointsAndMiddle`.
   - `DebugImage.DrawDiscDrawsExpectedRadius`.
   - `DebugImage.WritePpmWritesValidHeaderAndPayload`.

6. Вынести snapshot rendering из `lidar_debug_node.cpp` в `LidarDebugRenderer`.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/lidar_debug_renderer.hpp`.
   - Добавить `drone_city_nav/src/lidar_debug_renderer.cpp`.
   - Добавить `drone_city_nav/tests/lidar_debug_renderer_test.cpp`.
   - Обновить `drone_city_nav/CMakeLists.txt`.
   - Упростить `lidar_debug_node.cpp:623` `gridWorldToPixel`, `:657` `worldToPixel`, `:774` `drawRememberedHits`, `:783` `drawPath`, `:810` `drawScan`, `:833` `drawDrone`.

   Ожидаемый результат:

   - Renderer получает входной `LidarDebugFrame` с pose, grid, path, scan projections, remembered hits.
   - Node отвечает за subscriptions, timers и публикацию pointcloud/markers.
   - Логика координат image/grid покрыта тестами.

   Контрактный sketch:

   ```cpp
   struct LidarDebugFrame {
     Pose2 current_pose;
     std::optional<nav_msgs::msg::OccupancyGrid> grid;
     std::vector<Point2> path;
     std::vector<Point2> current_hits;
     std::vector<Point2> remembered_hits;
   };

   class LidarDebugRenderer {
   public:
     DebugImage render(const LidarDebugFrame& frame) const;
     std::optional<ImagePixel> worldToPixel(Point2 point,
                                            const LidarDebugFrame& frame) const;
   };
   ```

   Примечание: если `nav_msgs::msg::OccupancyGrid` в core header создаёт лишнюю ROS-зависимость, заменить его на небольшой `GridImageView` с width/height/resolution/origin/data-span.

   Автотесты:

   - `LidarDebugRenderer.GridWorldToPixelUsesFullMapCoordinates`.
   - `LidarDebugRenderer.FallsBackToDroneCenteredViewWithoutGrid`.
   - `LidarDebugRenderer.DrawsRememberedHitsAmberAndCurrentHitsRed`.
   - `LidarDebugRenderer.DrawsPathCyanWithoutThrowingOnOutOfBoundsPoints`.

7. Вынести lidar snapshot writer и JSON/CSV summary форматирование.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/lidar_snapshot_writer.hpp`.
   - Добавить `drone_city_nav/src/lidar_snapshot_writer.cpp`.
   - Добавить `drone_city_nav/tests/lidar_snapshot_writer_test.cpp`.
   - Обновить `lidar_debug_node.cpp:553` `writeScanCsv` и `lidar_debug_node.cpp:849` `writeSummary`.

   Ожидаемый результат:

   - JSON/CSV schema становится явным контрактом.
   - Node передаёт `SnapshotRecord`, writer пишет `.jsonl`, `.csv`, `.ppm`.
   - Headless artifacts остаются теми же: `snapshots.jsonl`, `snapshot_*_scan.csv`, `snapshot_*.ppm`.

   Автотесты:

   - `LidarSnapshotWriter.JsonEscapesStrings`.
   - `LidarSnapshotWriter.WritesFiniteNumbersAndNullForNonFinite`.
   - `LidarSnapshotWriter.CsvContainsProjectionStatusAndHitCoordinates`.
   - Edge-case: empty hits still writes valid JSON array and CSV header.

8. Вынести RViz radar marker generation из `lidar_debug_node.cpp` в отдельный builder.

   Файлы:

   - Добавить `drone_city_nav/include/drone_city_nav/lidar_radar_markers.hpp`.
   - Добавить `drone_city_nav/src/lidar_radar_markers.cpp`.
   - Добавить `drone_city_nav/tests/lidar_radar_markers_test.cpp`.
   - Упростить `lidar_debug_node.cpp:997` `markerPoint`, `:1006` `baseMarker`, `:1029` `addRangeRing`, `:1048` `addScanRayMarkers`, `:1082` `addDroneMarker`, `:1099` `publishRadarMarkers`.

   Ожидаемый результат:

   - Marker construction testable without running RViz.
   - Node остаётся владельцем publisher and `now()` timestamp.

   Автотесты:

   - `LidarRadarMarkers.BuildsExpectedRangeRingCount`.
   - `LidarRadarMarkers.SeparatesHitAndFreeRays`.
   - `LidarRadarMarkers.UsesMapFrameAndConfiguredZ`.

9. Обновить CMake targets и include graph без циклов.

   Файлы:

   - `drone_city_nav/CMakeLists.txt`.
   - Новые headers в `drone_city_nav/include/drone_city_nav/*.hpp`.
   - Новые sources в `drone_city_nav/src/*.cpp`.
   - Новые tests в `drone_city_nav/tests/*_test.cpp`.

   Ожидаемый результат:

   - Все новые pure modules входят в `drone_city_nav_core`.
   - ROS message types не протекают в core headers, кроме случаев, где это сознательно оправдано. Если нужен ROS type, предпочесть adapter в node-файле.
   - Каждый новый test target добавлен через `ament_add_gtest`.

10. Сохранить и усилить headless observability.

    Файлы:

    - `drone_city_nav/src/planner_node.cpp`.
    - `drone_city_nav/src/px4_offboard_node.cpp`.
    - `drone_city_nav/src/lidar_debug_node.cpp`.
    - `docs/MVP_SIMULATION.md`.

    Ожидаемый результат:

    - Существующие log markers не исчезают:
      - `Planning summary:`
      - `Keeping current path`
      - `Published path`
      - `Received path`
      - `Control summary:`
      - `LIDAR_DEBUG snapshot=`
    - Если формат строки меняется, обновить `scripts/run_city_mvp.sh` patterns только осознанно.
    - Добавить новые диагностические поля только там, где они помогают headless отладке:
      - planner core decision/status;
      - stable path decision reason;
      - follower selected target reason;
      - snapshot writer result status.

11. Обновить документацию по архитектуре после рефакторинга.

    Файлы:

    - `docs/MVP_SIMULATION.md`.
    - При необходимости `README.md`.

    Ожидаемый результат:

    - В `docs/MVP_SIMULATION.md` добавить короткий раздел про разделение:
      - ROS node layer;
      - core planning layer;
      - offboard path follower;
      - lidar debug renderer/writer/markers.
    - Не описывать внутренности чрезмерно, но зафиксировать где искать логи и тесты.

12. Выполнить рефакторинг по маленьким коммитам.

    Ожидаемый порядок коммитов:

    1. `Extract planner core path reuse logic`
    2. `Extract planning grid builder`
    3. `Extract offboard path follower`
    4. `Extract lidar debug image renderer`
    5. `Extract lidar snapshot writer and markers`
    6. `Update architecture docs`

    После каждого коммита:

    - Запустить `./scripts/format_cpp_changed.sh`.
    - Запустить scoped test target, который затронут текущим шагом.
    - Запустить `./scripts/check_cpp_quality.sh` перед финальным/крупным коммитом или после каждого рискованного шага.

# Verification plan

Минимальные команды после каждого C++ шага:

```bash
./scripts/format_cpp_changed.sh
/home/formi/.local/bin/runlim -- make build
/home/formi/.local/bin/runlim -- ctest --test-dir build/drone_city_nav --output-on-failure -R '<relevant_test_regex>'
```

Перед финальным commit/перед сдачей:

```bash
/home/formi/.local/bin/runlim -- ./scripts/check_cpp_quality.sh
```

Если из-за существующего build dir с `/workspace` host/container path mismatch `check_cpp_quality.sh` падает на host, запускать ту же команду внутри dev container по шаблону `scripts/dev_shell.sh`, но внешний `docker run ... ./scripts/check_cpp_quality.sh` обернуть в:

```bash
/home/formi/.local/bin/runlim -- docker run ...
```

После тяжёлого изменения planner/offboard поведения:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh'
```

После финального объединения всех refactor steps, если окружение симуляции доступно:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=240 ./scripts/run_city_mvp.sh'
```

Если GUI/RViz не нужен, не запускать GUI. Ручная GUI проверка — только дополнительный fallback, не замена автотестам.

# Testing strategy

## Категория 1: без рефакторинга

Цель: убедиться, что текущая база зелёная до изменений.

Команды:

```bash
/home/formi/.local/bin/runlim -- make build
/home/formi/.local/bin/runlim -- ctest --test-dir build/drone_city_nav --output-on-failure
```

Если build уже валиден и изменяется только план/документация, допустимо ограничиться:

```bash
test -s PLAN.md
git diff --check
```

## Категория 2: лёгкий рефакторинг

Применять для чистого переноса pure functions/classes без изменения поведения:

- `PlannerCore` extraction.
- `OffboardPathFollower` extraction.
- `DebugImage` extraction.

Обязательные проверки:

```bash
./scripts/format_cpp_changed.sh
/home/formi/.local/bin/runlim -- make build
/home/formi/.local/bin/runlim -- ctest --test-dir build/drone_city_nav --output-on-failure -R 'planner_core|offboard_path_follower|debug_image|lidar_debug_renderer|lidar_snapshot_writer|lidar_radar_markers'
```

Покрытие:

- Happy-path для каждого нового класса.
- Negative-path для invalid/empty inputs.
- Edge-case для out-of-bounds, stale/empty path, geometry mismatch, non-finite values.

## Категория 3: тяжёлый рефакторинг

Применять, когда меняется orchestration между node и core, logging contract, path reuse, follower target selection или lidar snapshot artifacts.

Обязательные проверки:

```bash
/home/formi/.local/bin/runlim -- ./scripts/check_cpp_quality.sh
```

Если изменение затронуло runtime поведение planner/offboard/lidar debug:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh'
```

Если изменение затронуло mission completion, path reuse, target follower или emergency/hold behavior:

```bash
/home/formi/.local/bin/runlim -- bash -lc 'HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=240 ./scripts/run_city_mvp.sh'
```

Если менялись lidar snapshots:

```bash
/home/formi/.local/bin/runlim -- python3 scripts/analyze_lidar_projection_snapshots.py log/lidar_debug/snapshots.jsonl --static-map drone_city_nav/worlds/generated_city.map2d
```

# Risks and tradeoffs

- Риск избыточной абстракции: если вынести слишком мелкие функции в отдельные классы, код станет длиннее и сложнее. Митигировать: выносить только устойчивые границы с тестами.
- Риск изменения runtime behavior при переносе path reuse/follower logic. Митигировать: сначала написать characterization tests по текущему поведению, затем переносить.
- Риск разрыва ROS logging patterns, от которых зависят `scripts/run_city_mvp.sh` и headless validation. Митигировать: сохранять ключевые log substrings или обновлять script patterns в том же коммите.
- Риск протекания ROS message types в core headers. Митигировать: core APIs должны принимать `Point2`, `OccupancyGrid2D`, spans/vectors и plain structs; ROS adapters оставить в node-файлах.
- Риск роста compile time из-за новых headers. Митигировать: минимальные includes, forward declarations где возможно, реализации в `.cpp`.
- Риск смены ownership/lifetime в callbacks. Митигировать: ROS subscriptions/publishers/timers остаются в node classes; core classes не хранят ссылок на ROS messages дольше вызова.
- Риск snapshot JSON/CSV schema drift. Митигировать: тесты writer-а и явное сохранение существующих field names.
- Риск, что цель «каждый файл <1000 строк» не будет достигнута за один шаг. Это приемлемо: функциональная безопасность важнее механического лимита. Приоритет — `planner_node.cpp`, потом `px4_offboard_node.cpp`, потом `lidar_debug_node.cpp`.

# Что могло сломаться

- Поведение планирования:
  - A* endpoints adjustment.
  - Inflation/clearance rejection.
  - Static-only fallback.
  - Stable path reuse and confirmed replanning.
  - Проверять unit-тестами `PlannerCore.*`, `PlanningGridBuilder.*`, `planner_core_test`, затем `MISSION_CHECK=1`.
- Контракт follower/offboard:
  - Empty path должен удерживать текущую позицию.
  - Target continuity не должен возвращать дрон назад без причины.
  - `map -> PX4 local` offset должен сохраниться.
  - Проверять `OffboardPathFollower.*`, `offboard_speed_controller_test`, headless smoke.
- Lidar debug artifacts:
  - Цвета remembered/current hits.
  - `snapshots.jsonl` schema.
  - CSV columns.
  - PPM output.
  - RViz marker namespace/frame/id.
  - Проверять `debug_image_test`, `lidar_debug_renderer_test`, `lidar_snapshot_writer_test`, `lidar_radar_markers_test`, analyzer script.
- Build/API:
  - Public headers в `include/drone_city_nav` должны быть self-contained.
  - Новые `.cpp` должны быть добавлены в `drone_city_nav_core`.
  - Новые GTest targets должны быть добавлены в CMake.
  - Проверять `make build`, `ctest`, `check_cpp_quality`.
- Производительность/ресурсы:
  - Дополнительные копии `std::vector<Point2>` и `OccupancyGrid2D` могут увеличить CPU/memory.
  - Митигировать move semantics, `std::span<const T>`, передачу grid by const ref.
  - Проверять headless logs на частоту replanning/snapshot и отсутствие задержек.
- Интеграции:
  - `scripts/run_city_mvp.sh` regex checks могут не найти изменённые log lines.
  - RViz topics/names не должны измениться без явного обновления config.
  - Проверять smoke run и `drone_city_nav/rviz/city_nav_debug.rviz` только если topic names менялись.

# Open questions

- Нужно ли строго добиваться `<1000` строк для каждого из трёх node-файлов или достаточно существенного снижения связности и тестируемости? Рекомендация: не делать механические дробления ради лимита.
- Нужна ли отдельная публичная архитектурная схема в `docs/MVP_SIMULATION.md`, или достаточно списка модулей и тестов? Рекомендация: короткий текстовый раздел без диаграммы.
- Допустимо ли в renderer/writer tests использовать ROS message types (`nav_msgs::msg::OccupancyGrid`, `visualization_msgs::msg::MarkerArray`) внутри core tests? Рекомендация: избегать ROS типов в core headers; ROS message tests допустимы только для adapter/builder, где они являются выходным контрактом.
- Нужно ли после каждого маленького extraction запускать полный `MISSION_CHECK=1`, или достаточно unit/build/quality до финального шага? Рекомендация: полный mission run после planner/offboard behavioral changes и финально перед сдачей, но не после каждого чистого переноса.
