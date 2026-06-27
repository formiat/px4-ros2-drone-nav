# Context

Планируем рефакторинг крупных C++ translation units без изменения runtime-поведения навигации, PX4 offboard управления и lidar debug. Текущий замер tracked-файлов показал:

```text
2942  drone_city_nav/src/px4_offboard_node.cpp
1805  drone_city_nav/src/planner_node.cpp
1030  drone_city_nav/src/lidar_debug_node.cpp
```

Цель реализации: разбить все `.cpp/.hpp` исходники длиной более 1000 строк, особенно `px4_offboard_node.cpp`, сохранив ROS topics, YAML parameters, log/blackbox contracts, RViz debug outputs и headless diagnostics. После завершения рефакторинга в tracked C++ source/header files не должно остаться файлов длиннее 1000 строк, кроме явно обоснованных generated/third-party файлов, если такие появятся.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. Контекст собран напрямую из локальных файлов:

- `README.md`: container workflow является единственным поддерживаемым workflow; approved commands: `./scripts/build.sh`, `./scripts/test.sh`, `./scripts/sim_headless.sh`, `./scripts/sim_gui.sh`, `./scripts/stop_sim.sh`, внутри container shell: `make build`, `make test`, `make test-scripts`, `make quality`, `make format`.
- `CONTRIBUTING.md`: C++ код должен следовать `CPP_BEST_PRACTICES.md`; использовать ROS 2/colcon через Makefile/scripts; не создавать ad-hoc CMake workflow; форматировать только измененные C++ файлы.
- `CPP_BEST_PRACTICES.md`: разделять public headers/private implementation, не полагаться на transitive includes, держать API малым, выносить тестируемую доменную логику из runtime glue, использовать RAII и явные lifetime/ownership contracts.
- `drone_city_nav/CMakeLists.txt`: основной reusable код живет в `drone_city_nav_core`; ROS adapters живут в `drone_city_nav_ros_adapters`; `planner_node`, `px4_offboard_node`, `lidar_debug_node` сейчас являются executables с крупными `.cpp`.
- `scripts/tests/test_offboard_telemetry_contract.py` и `scripts/tests/test_topic_contract.py`: есть Python contract tests, которые сейчас проверяют строки прямо в node `.cpp`; при выносе кода их нужно обновить на новые файлы, иначе рефакторинг сломает static checks.

Обязательные orchestrator protocols прочитаны:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

Notion/GitLab не упомянуты в пользовательском prompt, `notion_policy=optional`, поэтому удаленный доступ и CLI-чтение Notion/GitLab не выполняются.

# Detected stack/profiles

Основной стек workspace:

- ROS 2 Jazzy workspace.
- C++20, ament CMake package `drone_city_nav`.
- Build/test entrypoint: `colcon` через repository Makefile и container wrapper scripts.
- Runtime: Gazebo/PX4 SITL, ROS 2 nodes, RViz/debug topics, JSONL/CSV diagnostics.
- Дополнительные Python script tests в `scripts/tests`.

Прочитанные project profiles:

- `generic.md`: обязателен для всех workspace; применен для command discovery, scope strategy и reporting.
- `cpp.md`: выбран потому что workspace содержит `Makefile`, `CMakeLists.txt`, `.cpp/.hpp`; применен к C++ refactor/build/test plan.
- `rust.md` не читался и не применяется: в целевом workspace нет Rust manifest/source для затрагиваемой задачи.

# Repo-approved commands found

Команды из `README.md`, `CONTRIBUTING.md`, `Makefile`:

```bash
./scripts/build.sh
./scripts/test.sh
./scripts/sim_headless.sh
./scripts/sim_gui.sh
./scripts/stop_sim.sh
./scripts/dev_shell.sh
```

Внутри container shell:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-headless
make sim-gui
ctest --test-dir build/drone_city_nav --output-on-failure
```

Для реализации рефакторинга использовать container workflow. GUI/headless simulation не запускать без отдельной явной команды пользователя; для автоматической верификации рефакторинга достаточно build/unit/script tests.

# Affected components

1. `drone_city_nav/src/px4_offboard_node.cpp`
   - `Px4OffboardNode::Px4OffboardNode()` line 168: ROS parameter loading, subscriptions, publishers, startup logs.
   - Final trajectory ingestion/debug: `publishFinalTrajectoryDebug()` line 451, `addDroneDebugMarkers()` line 479, `applyReceivedFinalTrajectoryPath()` line 642, `onPath()` line 687, `onTrajectoryDiagnostics()` line 767.
   - Blackbox/dumps: `openFlightBlackbox()` line 799, `writeFinalTrajectorySamplesCsvFile()` line 878, `writeFinalTrajectorySummaryJsonFile()` line 919, `writeFinalTrajectorySamplesCsv()` line 931, `logTelemetry()` line 1998 and blackbox JSON fields line 2221 onward.
   - Control loop: `onTimer()` line 1106, `publishTrajectorySetpoint()` line 1191, `publishVelocityTrajectorySetpoint()` line 1349, `advanceWaypointIfNeeded()` line 1436, `updateCommandDiagnostics()` line 1776, `logControlSummary()` line 1864.
   - State block line 2812 onward.

2. `drone_city_nav/src/planner_node.cpp`
   - `PlannerNode::PlannerNode()` line 126: ROS wiring/startup logs.
   - Input callbacks/config application: `applyConfig()` line 302, `onLocalPosition()` line 342, `onMemoryGrid()` line 397, `onScan()` line 435, `loadConfiguredStaticMap()` line 498.
   - Pipeline trigger: `buildPlanningGrid()` line 552, `checkCurrentPathAndPublish()` line 625.
   - A*/trajectory publication: `publishPathFromPathCells()` line 886, `logPublishedPathSafety()` line 1175, `logRejectedUnsafeRoute()` line 1246, `publishPath()` line 1386, `publishTrajectoryDiagnostics()` line 1417.
   - Static/corridor debug: `publishStaticMapDebug()` line 1358, `republishStaticMapDebug()` line 1373, `publishProhibitedGrid()` line 1381, `writeCorridorSamplesCsvFile()` line 1438, `writeCorridorSamplesDump()` line 1489.
   - Stable path runtime decision: `keepCurrentPathIfStillClear()` line 1555.

3. `drone_city_nav/src/lidar_debug_node.cpp`
   - `LidarDebugNode::LidarDebugNode()` line 83: parameter loading, subscriptions, publishers, startup log.
   - Pose/scan callbacks: `onLocalPosition()` line 258, `onAttitude()` line 290, `onScan()` line 303.
   - Snapshot pipeline: `writeSnapshot()` line 353, `lidarProjectionPoseForBeam()` line 505, `collectScanRows()` line 549, `countGrid()` line 621, `writeSummary()` line 772.
   - Pointcloud/markers: `collectOccupiedGridPoints()` line 706, `publishProhibitedPointCloud()` line 710, `rememberHitPoints()` line 728, `publishPointCloud()` line 838, `publishRadarMarkers()` line 896.

4. Existing tests/contracts impacted by file movement:
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
   - `drone_city_nav/tests/trajectory_debug_markers_test.cpp`
   - `drone_city_nav/tests/lidar_debug_renderer_test.cpp`
   - `drone_city_nav/tests/lidar_snapshot_writer_test.cpp`
   - `drone_city_nav/tests/planner_node_config_test.cpp`
   - `scripts/tests/test_offboard_telemetry_contract.py`
   - `scripts/tests/test_topic_contract.py`
   - `scripts/tests/test_validate_city_mvp_headless.py`

# Implementation steps

1. Добавить baseline size guard для oversized source files.
   - Файлы:
     - добавить `scripts/tests/test_cpp_source_size_contract.py`;
     - при необходимости обновить `scripts/tests/test_container_entrypoints.py` только если discovery test должен знать новый script test.
   - Контракт:
     - считать `git ls-files` для `*.cpp`, `*.hpp`, `*.cc`, `*.h`;
     - исключить generated/build/log/external paths;
     - на время многошагового рефакторинга разрешить allowlist только для трех текущих файлов с TODO-комментарием в test data;
     - финальный шаг обязан удалить allowlist для этих трех файлов.
   - Материализуемый результат:
     - `make test-scripts` показывает, что новый size-contract test выполняется;
     - после финального шага test падает при любом source/header file > 1000 строк.

   Пример намерения:

   ```python
   MAX_SOURCE_LINES = 1000
   TEMPORARY_ALLOWLIST = {
       "drone_city_nav/src/px4_offboard_node.cpp",
       "drone_city_nav/src/planner_node.cpp",
       "drone_city_nav/src/lidar_debug_node.cpp",
   }
   ```

2. Вынести PX4 offboard parameter/config loading из `px4_offboard_node.cpp`.
   - Исходный anchor: `drone_city_nav/src/px4_offboard_node.cpp:168`.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/px4_offboard_node_config.hpp`;
     - добавить `drone_city_nav/src/px4_offboard_node_config.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`, включив новый `.cpp` в `drone_city_nav_ros_adapters` или отдельную small library, если нужен `rclcpp`;
     - заменить/расширить `drone_city_nav/tests/px4_offboard_config_test.cpp`.
   - Контракт:
     - `Px4OffboardNode::Px4OffboardNode()` вызывает `loadPx4OffboardNodeConfig(*this)`;
     - defaults/clamps остаются теми же, что сейчас в constructor;
     - startup log значения не меняются.
   - Материализуемый результат:
     - constructor уменьшается и становится ROS wiring + `applyConfig(config)`;
     - unit test покрывает happy-path/defaults, clamp edge cases, topic names, blackbox path, velocity follower config.

   Пример API:

   ```cpp
   struct Px4OffboardNodeConfig {
     double cruise_altitude_m{12.0};
     VelocityFollowerConfig velocity_follower{};
     std::string path_topic{"/drone_city_nav/path"};
     std::string flight_blackbox_path{"log/offboard_blackbox.jsonl"};
   };

   [[nodiscard]] Px4OffboardNodeConfig loadPx4OffboardNodeConfig(rclcpp::Node& node);
   ```

3. Вынести PX4 final trajectory debug dumps из `px4_offboard_node.cpp`.
   - Исходные anchors: `diagnosticDumpDirectory()` line 831, `writeFinalTrajectorySamplesCsvFile()` line 878, `writeFinalTrajectorySummaryJsonFile()` line 919, `writeFinalTrajectorySamplesCsv()` line 931.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/final_trajectory_debug_io.hpp`;
     - добавить `drone_city_nav/src/final_trajectory_debug_io.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/final_trajectory_debug_io_test.cpp`.
   - Контракт:
     - `latest.csv`, timestamped CSV, `latest_summary.json` и timestamped summary остаются в `log/final_trajectory_samples/`;
     - CSV header остается совместимым с `finalTrajectorySamplesCsvHeader()`/`trajectory_diagnostics_io`;
     - все non-finite numeric values пишутся тем же форматом, что сейчас.
   - Материализуемый результат:
     - node только собирает `FinalTrajectoryDebugDumpInput` и вызывает writer;
     - tests парсят CSV/JSON summary и проверяют path id, sample count, status, speed profile fields.

4. Вынести PX4 offboard debug markers в отдельный ROS adapter.
   - Исходные anchors: `publishFinalTrajectoryDebug()` line 451, `addDroneDebugMarkers()` line 479, `publishOffboardDebugMarkers()` line 515.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/offboard_debug_markers.hpp`;
     - добавить `drone_city_nav/src/offboard_debug_markers.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/offboard_debug_markers_test.cpp`.
   - Контракт:
     - marker namespaces, colors, ids, z-level and delete behavior остаются совместимыми с RViz config;
     - `trajectory_debug_markers.cpp` остается источником speed/curvature colormap для final trajectory.
   - Материализуемый результат:
     - marker construction тестируется без запуска PX4/Gazebo;
     - node остается только publisher owner.

5. Вынести PX4 telemetry summary и blackbox JSONL writer.
   - Исходные anchors: `openFlightBlackbox()` line 799, `logTelemetry()` line 1998, blackbox JSON write block line 2221 onward.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/offboard_blackbox.hpp`;
     - добавить `drone_city_nav/src/offboard_blackbox.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/offboard_blackbox_test.cpp`;
     - обновить `scripts/tests/test_offboard_telemetry_contract.py`, чтобы он искал telemetry contract в `offboard_blackbox.cpp/.hpp` и node glue, а не только в старом `.cpp`.
   - Контракт:
     - JSONL field names из `scripts/tests/test_offboard_telemetry_contract.py` сохраняются;
     - ROS text logs `Drone telemetry:`, `Drone path diagnostics:`, `Drone velocity command diagnostics:`, `Drone obstacle diagnostics:` сохраняют machine-readable fields для headless debug.
   - Материализуемый результат:
     - writer тестируется через `std::ostringstream` и JSON parse/substring checks;
     - node собирает `OffboardBlackboxRecord` и вызывает writer;
     - Python telemetry contract продолжает защищать отсутствие legacy fields.

   Пример API:

   ```cpp
   struct OffboardBlackboxRecord {
     std::int64_t time_ns{0};
     std::uint64_t planner_path_id{0};
     Pose2 pose{};
     VelocitySetpointPlan velocity_plan{};
     TrajectoryPlannerStats trajectory_stats{};
   };

   void writeOffboardBlackboxRecord(std::ostream& stream,
                                    const OffboardBlackboxRecord& record);
   ```

6. Вынести PX4 setpoint publishing glue, оставив `px4_offboard_node.cpp` владельцем ROS subscriptions/publishers.
   - Исходные anchors: `onTimer()` line 1106, `publishOffboardControlMode()` line 1177, `publishTrajectorySetpoint()` line 1191, `publishVelocityTrajectorySetpoint()` line 1349, `publishVehicleCommand()` line 1418.
   - Файлы:
     - добавить private adapter header `drone_city_nav/src/px4_offboard_setpoint_io.hpp` или public header `include/drone_city_nav/px4_offboard_setpoint_io.hpp` только если нужен тестовый include;
     - добавить `drone_city_nav/src/px4_offboard_setpoint_io.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/px4_offboard_setpoint_io_test.cpp`.
   - Контракт:
     - `OffboardControlMode` остается velocity mode for cruise, position mode for takeoff/hold/final hold, acceleration disabled;
     - NED altitude conversion (`z = -altitude`) сохраняется;
     - `VehicleCommand` target/source systems/components сохраняются.
   - Материализуемый результат:
     - pure builder functions for PX4 messages тестируются без spinning node;
     - node вызывает helpers and publishes returned messages.

7. Вынести final trajectory receive/apply state из `px4_offboard_node.cpp`.
   - Исходные anchors: `pathPointsFromMessage()` line 431, `clearFinalTrajectory()` line 530, `mergePlannerDiagnosticsIntoCurrentTrajectoryStats()` line 553, `updatePlannerStatsForReceivedTrajectory()` line 573, `resetVelocitySmootherState()` line 633, `applyReceivedFinalTrajectoryPath()` line 642, `onPath()` line 687, `onPathId()` line 762, `onTrajectoryDiagnostics()` line 767.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/offboard_trajectory_state.hpp`;
     - добавить `drone_city_nav/src/offboard_trajectory_state.cpp`;
     - обновить `drone_city_nav/src/px4_offboard_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/offboard_trajectory_state_test.cpp`.
   - Контракт:
     - offboard node consumes only planner final trajectory path; rough A* route не должен использоваться;
     - `buildTrajectorySpeedProfile()`, `lineTrajectoryFromSamples()`, `trajectoryPointSamplesFromPoints()` вызываются в том же порядке;
     - smoother reset reason/count сохраняется для path updates.
   - Материализуемый результат:
     - tests покрывают valid path, empty path, stale path id ordering, diagnostics merge, invalid/non-finite path rejection;
     - `scripts/tests/test_offboard_telemetry_contract.py::test_velocity_mode_consumes_planner_final_trajectory` обновлен на новые файлы.

8. Перепроверить `px4_offboard_node.cpp` size и удалить allowlist entry.
   - Файлы:
     - `drone_city_nav/src/px4_offboard_node.cpp`;
     - `scripts/tests/test_cpp_source_size_contract.py`.
   - Контракт:
     - `px4_offboard_node.cpp` остается тонким ROS node: constructor/apply config, subscriptions, publisher ownership, callbacks forwarding to extracted helpers, main.
   - Материализуемый результат:
     - `wc -l drone_city_nav/src/px4_offboard_node.cpp` < 1000;
     - size contract больше не allowlist'ит `px4_offboard_node.cpp`.

9. Вынести planner corridor samples dump из `planner_node.cpp`.
   - Исходные anchors: `writeCsvNumberOrEmpty()` line 1432, `writeCorridorSamplesCsvFile()` line 1438, `writeCorridorSamplesDump()` line 1489.
   - Файлы:
     - добавить `drone_city_nav/include/drone_city_nav/corridor_samples_io.hpp`;
     - добавить `drone_city_nav/src/corridor_samples_io.cpp`;
     - обновить `drone_city_nav/src/planner_node.cpp`;
     - обновить `drone_city_nav/CMakeLists.txt`;
     - добавить `drone_city_nav/tests/corridor_samples_io_test.cpp`.
   - Контракт:
     - `log/corridor_samples/latest.csv` and history `path_<id>_<stamp>.csv` формат сохраняется;
     - `route_x/route_y/center_x/center_y/width_m/clearance_m/center_recovery_m` columns сохраняются.
   - Материализуемый результат:
     - CSV writer тестируется отдельно;
     - planner node только выбирает directory/stamp и вызывает writer.

10. Вынести planner path publication and diagnostics formatting.
    - Исходные anchors: `publishPathFromPathCells()` line 886, `logPublishedPathSafety()` line 1175, `logRejectedUnsafeRoute()` line 1246, `publishPath()` line 1386, `publishTrajectoryDiagnostics()` line 1417, `logPathUpdate()` line 1611, `recordPathPublication()` line 1666.
    - Файлы:
      - добавить `drone_city_nav/include/drone_city_nav/planner_path_publication.hpp`;
      - добавить `drone_city_nav/src/planner_path_publication.cpp`;
      - добавить `drone_city_nav/include/drone_city_nav/planner_diagnostics_format.hpp`;
      - добавить `drone_city_nav/src/planner_diagnostics_format.cpp`;
      - обновить `drone_city_nav/src/planner_node.cpp`;
      - обновить `drone_city_nav/CMakeLists.txt`;
      - добавить `drone_city_nav/tests/planner_path_publication_test.cpp` и `drone_city_nav/tests/planner_diagnostics_format_test.cpp`.
    - Контракт:
      - final published path must always be final racing trajectory, not rough A*;
      - path id publication before path remains unchanged if current behavior requires it;
      - `Planning summary`, `Path smoothing diagnostics`, `Published path`, `Planner counters` logs keep fields used by `scripts/tests/test_validate_city_mvp_headless.py`.
    - Материализуемый результат:
      - route candidate selection, collinear collapse fallback, traversability rejection and empty hold decisions тестируются без ROS publisher;
      - diagnostics format tests assert critical field names: `heuristic_weight`, `core_breakdown`, `racing_line`, `turn_smoothing`, `speed_profile`.

11. Вынести planner grid/source orchestration into a small coordinator.
    - Исходные anchors: `onLocalPosition()` line 342, `onMemoryGrid()` line 397, `onScan()` line 435, `loadConfiguredStaticMap()` line 498, `buildPlanningGrid()` line 552, `checkCurrentPathAndPublish()` line 625, `keepCurrentPathIfStillClear()` line 1555.
    - Файлы:
      - добавить `drone_city_nav/include/drone_city_nav/planner_runtime_state.hpp`;
      - добавить `drone_city_nav/src/planner_runtime_state.cpp`;
      - обновить `drone_city_nav/src/planner_node.cpp`;
      - обновить `drone_city_nav/CMakeLists.txt`;
      - добавить `drone_city_nav/tests/planner_runtime_state_test.cpp`.
    - Контракт:
      - raw sources remain raw; only planner builder inflates merged grid once;
      - replan decision remains based on current final trajectory intersection, not rough A* or corridor;
      - stale pose/no grid/no source cases keep existing hold/skip behavior and logs.
    - Материализуемый результат:
      - tests cover no pose, stale pose, no ready sources, memory geometry mismatch, current path clear, trajectory intersects prohibited.

12. Перепроверить `planner_node.cpp` size и удалить allowlist entry.
    - Файлы:
      - `drone_city_nav/src/planner_node.cpp`;
      - `scripts/tests/test_cpp_source_size_contract.py`.
    - Материализуемый результат:
      - `wc -l drone_city_nav/src/planner_node.cpp` < 1000;
      - size contract больше не allowlist'ит `planner_node.cpp`.

13. Вынести lidar debug config loading.
    - Исходный anchor: `LidarDebugNode::LidarDebugNode()` line 83.
    - Файлы:
      - добавить `drone_city_nav/include/drone_city_nav/lidar_debug_node_config.hpp`;
      - добавить `drone_city_nav/src/lidar_debug_node_config.cpp`;
      - обновить `drone_city_nav/src/lidar_debug_node.cpp`;
      - обновить `drone_city_nav/CMakeLists.txt`;
      - добавить `drone_city_nav/tests/lidar_debug_node_config_test.cpp`.
    - Контракт:
      - defaults/clamps for `output_dir`, `snapshot_period_s`, pointcloud topics, lidar projection params, memory params сохраняются;
      - startup log fields остаются достаточными для headless debug.
    - Материализуемый результат:
      - constructor reduces to config load, subscriptions/publishers, timer setup;
      - config tests cover defaults, custom topics, clamp boundaries.

14. Вынести lidar debug pointcloud/grid collection.
    - Исходные anchors: `countGrid()` line 621, `collectOccupiedGridPoints()` line 706, `publishProhibitedPointCloud()` line 710, `publishPointCloud()` line 838.
    - Файлы:
      - добавить `drone_city_nav/include/drone_city_nav/lidar_debug_pointclouds.hpp`;
      - добавить `drone_city_nav/src/lidar_debug_pointclouds.cpp`;
      - обновить `drone_city_nav/src/lidar_debug_node.cpp`;
      - обновить `drone_city_nav/CMakeLists.txt`;
      - добавить `drone_city_nav/tests/lidar_debug_pointclouds_test.cpp`.
    - Контракт:
      - prohibited points and raw memory points remain ground-level debug outputs;
      - point cloud layout remains `x/y/z` FLOAT32 with same `point_step`, `row_step`, frame id.
    - Материализуемый результат:
      - tests cover occupied/prohibited cell extraction, empty grid, non-100 occupancy values, generated `PointCloud2` binary layout.

15. Вынести lidar debug snapshot orchestration helpers that do not need `rclcpp::Node`.
    - Исходные anchors: `writeSnapshot()` line 353, `lidarProjectionPoseForBeam()` line 505, `collectScanRows()` line 549, `writeSummary()` line 772.
    - Файлы:
      - добавить `drone_city_nav/include/drone_city_nav/lidar_debug_snapshot_pipeline.hpp`;
      - добавить `drone_city_nav/src/lidar_debug_snapshot_pipeline.cpp`;
      - обновить `drone_city_nav/src/lidar_debug_node.cpp`;
      - обновить `drone_city_nav/CMakeLists.txt`;
      - расширить `drone_city_nav/tests/lidar_snapshot_writer_test.cpp` или добавить `drone_city_nav/tests/lidar_debug_snapshot_pipeline_test.cpp`.
    - Контракт:
      - `snapshots.jsonl`, PPM and scan CSV filenames remain `snapshot_<n>`;
      - motion compensation / scan deskew math remains delegated to `lidar_projection.cpp`;
      - `LIDAR_DEBUG snapshot=...` log keeps fields used by headless analysis.
    - Материализуемый результат:
      - node owns ROS timing and files, helper returns `LidarDebugSnapshotOutput`;
      - tests cover no scan/pose, accepted hit, altitude rejected hit, remembered hits count.

16. Перепроверить `lidar_debug_node.cpp` size и удалить allowlist entry.
    - Файлы:
      - `drone_city_nav/src/lidar_debug_node.cpp`;
      - `scripts/tests/test_cpp_source_size_contract.py`.
    - Материализуемый результат:
      - `wc -l drone_city_nav/src/lidar_debug_node.cpp` < 1000;
      - size contract has empty allowlist for these three files.

17. Обновить CMake ownership после всех выносов.
    - Файл: `drone_city_nav/CMakeLists.txt`.
    - Правило:
      - pure C++ helpers without ROS message dependencies go into `drone_city_nav_core`;
      - helpers requiring `rclcpp`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `px4_msgs` go into `drone_city_nav_ros_adapters` or are attached only to the executable/test that needs them;
      - avoid circular dependencies: executables link libraries, libraries never link executables.
    - Материализуемый результат:
      - all new tests are registered with `ament_add_gtest`;
      - executable source lists are smaller and explicit.

18. Обновить Python/static contract tests after file movement.
    - Файлы:
      - `scripts/tests/test_offboard_telemetry_contract.py`;
      - `scripts/tests/test_topic_contract.py`;
      - `scripts/tests/test_validate_city_mvp_headless.py`, если log field ownership moved but text contracts remain.
    - Контракт:
      - static tests should check semantic ownership across new files, not assume all strings live in `px4_offboard_node.cpp`;
      - forbidden legacy strings remain forbidden repository-wide where appropriate.
    - Материализуемый результат:
      - `make test-scripts` passes;
      - tests fail if final trajectory consumption accidentally moves back to rough route or if debug topics disappear.

19. Финальная интеграционная верификация рефакторинга.
    - Команды:
      - `./scripts/dev_shell.sh make format`
      - `./scripts/dev_shell.sh make quality`
      - `./scripts/dev_shell.sh make test-scripts`
      - optional scoped CTest during development:
        `./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R "(offboard|planner|lidar|trajectory|ros_conversions)"`
    - Материализуемый результат:
      - build, unit tests, C++ quality and script contracts pass;
      - no C++ source/header files >1000 lines according to `scripts/tests/test_cpp_source_size_contract.py`.

# Verification plan

Минимальные проверки на каждую пачку C++ changes:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
```

Если изменены Python contract tests или scripts:

```bash
./scripts/dev_shell.sh make test-scripts
```

Если добавлены/изменены конкретные C++ tests, до `make quality` можно запускать scoped CTest:

```bash
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R "(offboard_blackbox|offboard_debug_markers|final_trajectory_debug_io|planner_path_publication|corridor_samples_io|lidar_debug_pointclouds|lidar_debug_node_config)"
```

Для headless-отладки после отдельного явного разрешения пользователя:

```bash
./scripts/sim_headless.sh
python3 scripts/analyze_lidar_projection_snapshots.py log/lidar_debug/snapshots.jsonl --static-map drone_city_nav/worlds/generated_city.map2d
```

Симуляцию не включать в обязательную автоматическую проверку рефакторинга без отдельной команды пользователя.

# Testing strategy

1. Категория 1: без рефакторинга / guard rails.
   - `scripts/tests/test_cpp_source_size_contract.py`: фиксирует текущую проблему и финальный порог.
   - `scripts/tests/test_offboard_telemetry_contract.py`: переносится с проверки одного файла на проверку набора файлов, чтобы сохранять semantic contract.
   - `scripts/tests/test_topic_contract.py`: проверяет, что topic names/ROS contracts остались доступны после выноса.

2. Категория 2: лёгкий рефакторинг / pure helper tests.
   - `offboard_blackbox_test`: JSONL fields, non-finite handling, no legacy fields.
   - `final_trajectory_debug_io_test`: CSV/summary dump structure.
   - `offboard_debug_markers_test`: marker namespaces/actions/colors.
   - `corridor_samples_io_test`: CSV columns and non-finite formatting.
   - `planner_diagnostics_format_test`: log strings contain required fields.
   - `lidar_debug_pointclouds_test`: OccupancyGrid -> PointCloud2 conversion.
   - `lidar_debug_node_config_test` and `px4_offboard_config_test`: defaults/clamps/topic overrides.

3. Категория 3: тяжёлый рефакторинг / behavior-preserving integration tests.
   - `offboard_trajectory_state_test`: valid/empty/invalid final trajectory path, path id ordering, diagnostics merge, speed profile rebuild.
   - `planner_runtime_state_test`: source readiness, current path reuse, prohibited intersection replan trigger.
   - Existing tests: `trajectory_planner_test`, `offboard_velocity_follower_test`, `trajectory_speed_planner_test`, `planner_core_test`, `planning_grid_builder_test`, `lidar_projection_test`, `lidar_snapshot_writer_test`.
   - Script-level contracts: `make test-scripts` protects launch/log/topic assumptions.

# Risks and tradeoffs

- Поведение полёта может измениться, если helper extraction accidentally меняет order of operations в `onTimer()`, `publishTrajectorySetpoint()`, `applyReceivedFinalTrajectoryPath()`. Проверять unit tests плюс blackbox/log contracts; в ручном/headless прогоне смотреть `path_id`, `trajectory_valid`, `velocity_command`, `final_goal_hold_active`.
- ROS topic QoS может измениться при выносе publisher/subscriber setup. Проверять `scripts/tests/test_topic_contract.py` и review constructor wiring.
- JSONL/CSV debug contracts могут сломаться при переносе writer кода. Проверять parseable JSON tests, `test_offboard_telemetry_contract.py`, `final_trajectory_debug_io_test`, `corridor_samples_io_test`.
- CMake dependency graph может стать цикличным, если ROS adapters начнут зависеть от executable-private code. Держать pure helpers in `drone_city_nav_core`, ROS message builders in `drone_city_nav_ros_adapters` or executable-private sources.
- Static Python tests сейчас привязаны к конкретным файлам. Их надо обновлять одновременно с переносом, иначе появятся ложные failures или, хуже, contract gaps.
- Большой mechanical move может затруднить review. Делать небольшие коммиты по одному extraction boundary; после каждого запускать scoped tests и `make quality`.
- Возможна потеря headless observability, если часть логов сочтена duplicate и удалена. В этом рефакторинге логи не удалять; только переносить форматирование/запись в helper modules.

# Open questions

- Жесткий финальный порог: план предполагает `<1000` строк для всех tracked `.cpp/.hpp`. Если нужен более строгий порог, например `<800`, size-contract test надо задать сразу.
- Допустим ли public header для executable-private helpers с `px4_msgs`/`visualization_msgs`, или предпочтительнее private headers under `src/` плюс tests compiling those `.cpp` directly? План допускает оба варианта; предпочтение: public headers только для reusable/tested contracts, private headers для узкой executable glue.
- Нужно ли после каждого крупного extraction запускать headless smoke? По текущим пользовательским правилам симуляцию нельзя запускать без явной команды; план считает unit/script tests достаточными до отдельного разрешения на simulation verification.
