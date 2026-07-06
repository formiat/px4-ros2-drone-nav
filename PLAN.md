# План: 3D Representation + RViz Foundation для `3D passage traversal`

## Context

Планируем только первый этап фичи `3D passage traversal`: проект должен летать как раньше, но executable trajectory должна стать технически 3D. На этом этапе не добавляем распознавание passage/opening, не добавляем passage annotation map, не меняем 2D planner, не вводим вертикальные манёвры через проёмы и не меняем lidar policy.

Текущая картина по коду:

- `Point3` уже есть в `drone_city_nav/include/drone_city_nav/types.hpp:12`, но `TrajectoryPointSample` в `drone_city_nav/include/drone_city_nav/trajectory.hpp:47` хранит только `Point2 point`.
- Геометрия, projection, stationing и curvature в `drone_city_nav/src/trajectory.cpp:255`, `drone_city_nav/src/trajectory.cpp:390`, `drone_city_nav/src/trajectory.cpp:474` намеренно 2D.
- `pathToRos()` в `drone_city_nav/src/ros_conversions.cpp:243` умеет назначить всем `Point2` одну высоту, но planner сейчас публикует runtime path на `kGroundDebugZ` в `drone_city_nav/src/planner_node_debug_publication.cpp:61`.
- `PlannerNodeConfig` читает `cruise_altitude_m` в `drone_city_nav/src/planner_node_config.cpp:25`, но `PlannerNode` сейчас не хранит этот параметр и не использует его при публикации path.
- Offboard принимает `nav_msgs::Path`, сразу теряет `z` через `pathPointsFromMessage()` в `drone_city_nav/src/offboard_trajectory_state.cpp:17`, а затем пересобирает samples через `trajectoryPointSamplesFromPoints()` в `drone_city_nav/src/offboard_trajectory_state.cpp:131`.
- Финальный RViz path публикуется на `z=0.0` в `drone_city_nav/src/px4_offboard_node_trajectory.cpp:40`.
- Debug markers рисуются от `ground_z_m`: `buildTrajectoryDebugMarkers()` в `drone_city_nav/src/trajectory_debug_markers.cpp:85` и drone marker в `drone_city_nav/src/offboard_debug_markers.cpp:39`.
- Drone marker уже имеет тип `SPHERE`, но фактически сплющен и стоит на земле: `scale.z = 0.25` и `pose.position.z = ground_z_m` в `drone_city_nav/src/offboard_debug_markers.cpp:43`.
- Runtime vertical control сейчас отдельный и держит `cruise_altitude_m`: `verticalVelocitySetpointNed()` в `drone_city_nav/src/px4_offboard_node_control.cpp:272`. На этом этапе его не меняем.

Целевой контракт этапа: `z_m` должен проходить через trajectory samples, ROS path, final trajectory RViz path, marker/debug/dump слой и headless diagnostics. При `z_m == cruise_altitude_m` XY path, curvature, speed profile, lateral control, replanning и terminal position capture должны остаться поведенчески прежними.

## Investigation context

`INVESTIGATION.md` отсутствует. Дополнительного investigation artifact для этого запуска нет.

Notion не использовался: в prompt нет Notion task id, а `notion_policy=optional`. GitLab не использовался: prompt не содержит MR/review target. Оба протокола прочитаны и учтены как read-only ограничения.

## Detected stack/profiles

- Стек: ROS 2 workspace, C++20, ament CMake package `drone_city_nav`, сборка через `colcon`.
- Прочитанные профили:
  - `generic.md`: обязателен для любого workspace.
  - `cpp.md`: применим, потому что есть `CMakeLists.txt`, `.cpp/.hpp`, C++ targets и gtest.
- Локальные инструкции:
  - `AGENTS.md`: требует container workflow, `make quality` перед commit, C++ comments/docs in English.
  - `CONTRIBUTING.md`: подтверждает container-only workflow, `colcon`, `make test`, `make test-scripts`, `make quality`, `make format`.
  - `README.md`: подтверждает approved commands и расположение logs/artifacts.

## Repo-approved commands found

Основные wrapper-команды из repo root:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_gui.sh`
- `./scripts/sim_headless.sh`
- `./scripts/stop_sim.sh`
- `./scripts/dev_shell.sh`

Команды внутри container/dev shell:

- `make build`
- `make test`
- `make test-scripts`
- `make quality`
- `make format`
- `make sim-headless`
- `make sim-gui`
- scoped test command from `CONTRIBUTING.md`: `ctest --test-dir build/drone_city_nav --output-on-failure`

Для будущей реализации C++ изменений перед commit: форматировать изменённые C++ файлы через `make format`, затем запускать `make quality`.

## Affected components

1. Core trajectory model:
   - `drone_city_nav/include/drone_city_nav/trajectory.hpp`
   - `drone_city_nav/src/trajectory.cpp`
   - `drone_city_nav/tests/trajectory_test.cpp`

2. Planner trajectory build/publication:
   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
   - `drone_city_nav/src/trajectory_planner.cpp`
   - `drone_city_nav/src/planner_node.hpp`
   - `drone_city_nav/src/planner_node_lifecycle.cpp`
   - `drone_city_nav/src/planner_node_debug_publication.cpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp`
   - `drone_city_nav/src/ros_conversions.cpp`
   - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`

3. Offboard accepted trajectory state:
   - `drone_city_nav/include/drone_city_nav/offboard_trajectory_state.hpp`
   - `drone_city_nav/src/offboard_trajectory_state.cpp`
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp`
   - `drone_city_nav/src/px4_offboard_node_control.cpp`
   - `drone_city_nav/src/px4_offboard_node_telemetry.cpp`
   - `drone_city_nav/src/px4_offboard_node.hpp`

4. RViz marker/debug helpers:
   - `drone_city_nav/include/drone_city_nav/trajectory_debug_markers.hpp`
   - `drone_city_nav/src/trajectory_debug_markers.cpp`
   - `drone_city_nav/include/drone_city_nav/offboard_debug_markers.hpp`
   - `drone_city_nav/src/offboard_debug_markers.cpp`
   - новый helper-модуль, например `drone_city_nav/include/drone_city_nav/visualization_marker_helpers.hpp` и `drone_city_nav/src/visualization_marker_helpers.cpp`
   - `drone_city_nav/CMakeLists.txt`

5. Dumps/headless diagnostics:
   - `drone_city_nav/src/trajectory_diagnostics_io_csv.cpp`
   - `drone_city_nav/include/drone_city_nav/final_trajectory_debug_io.hpp`
   - `drone_city_nav/src/final_trajectory_debug_io.cpp`
   - `drone_city_nav/include/drone_city_nav/offboard_blackbox.hpp`
   - `drone_city_nav/src/offboard_blackbox.cpp`
   - `scripts/tests/test_offboard_telemetry_contract.py`

6. Docs/RViz config:
   - `docs/overview.md`
   - `docs/navigation_pipeline.md`
   - `docs/drone_control.md`
   - `docs/rviz.md`
   - `docs/diagnostics.md`
   - `docs/configuration.md`
   - `drone_city_nav/rviz/city_nav_debug.rviz` only if marker namespaces/display names change.

## Implementation steps

1. Добавить `z_m` в core trajectory samples без изменения 2D-геометрии.

   Файлы и anchors:
   - `drone_city_nav/include/drone_city_nav/trajectory.hpp:47` (`TrajectoryPointSample`)
   - `drone_city_nav/src/trajectory.cpp:125` (`sampleIsFinite`)
   - `drone_city_nav/src/trajectory.cpp:255` (`trajectorySamplesAreUsable`)
   - `drone_city_nav/src/trajectory.cpp:474` (`sampleTrajectoryDetailed`)
   - `drone_city_nav/src/trajectory.cpp:511` (`trajectoryPointSamplesFromPoints`)

   Материализуемый результат:
   - `TrajectoryPointSample` получает поле `double z_m{0.0}`.
   - `trajectorySamplesAreUsable()` отвергает non-finite `z_m`, но продолжает проверять stationing/tangent по XY.
   - Добавить helper:

     ```cpp
     void assignTrajectorySampleAltitude(std::span<TrajectoryPointSample> samples,
                                         double altitude_m);

     [[nodiscard]] double trajectorySampleAltitudeAtS(
         std::span<const TrajectoryPointSample> samples, double s_m);
     ```

   - `sampleTrajectoryDetailed()` и `trajectoryPointSamplesFromPoints()` оставляют прежнее поведение по умолчанию (`z_m = 0.0`), чтобы unit tests и внутренние 2D call sites не получили скрытую смену semantics.
   - Вся длина, curvature, projection и traversability остаются XY-only. Не использовать `Point3` distance для speed profile или optimizer на этом этапе.

   Тесты:
   - `drone_city_nav/tests/trajectory_test.cpp`: добавить кейс, что non-finite `z_m` делает samples unusable.
   - `drone_city_nav/tests/trajectory_test.cpp`: добавить кейс, что `trajectorySampleAltitudeAtS()` линейно интерполирует `z_m` по `s_m`.
   - `drone_city_nav/tests/trajectory_test.cpp`: добавить regression, что `lineTrajectoryFromSamples()` и `projectOnTrajectorySamples()` дают те же XY distance/projection при разных `z_m`.

2. Сделать `cruise_altitude_m` источником default trajectory altitude в planner.

   Файлы и anchors:
   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:34` (`TrajectoryPlannerConfig`)
   - `drone_city_nav/src/planner_node_config.cpp:25` (`loadPlannerNodeConfig`)
   - `drone_city_nav/src/planner_node_lifecycle.cpp:190` (`PlannerNode::configure`)
   - `drone_city_nav/src/planner_node.hpp:312` и `drone_city_nav/src/planner_node.hpp:360`
   - `drone_city_nav/src/trajectory_planner.cpp:339` (`planBaselineTrajectory`)
   - `drone_city_nav/src/trajectory_planner.cpp:533` (`planTrajectory`/optimized flow)

   Материализуемый результат:
   - Добавить `double default_altitude_m{0.0}` в `TrajectoryPlannerConfig`.
   - В `loadPlannerNodeConfig()` после чтения `cruise_altitude_m` прокинуть:

     ```cpp
     config.trajectory_planner.default_altitude_m = config.cruise_altitude_m;
     ```

   - Добавить `double cruise_altitude_m_{12.0};` в `PlannerNode` и присваивать его в `PlannerNode::configure()`.
   - После построения final/baseline/optimized `TrajectoryPlannerResult::samples` централизованно вызывать `assignTrajectorySampleAltitude(result.samples, config.default_altitude_m)`.
   - Для async refined trajectory это должно происходить внутри `planOptimizedTrajectory*()`, чтобы refined result тоже уже нёс `z_m`.

   Тесты:
   - `drone_city_nav/tests/planner_node_config_test.cpp`: проверить, что `trajectory_planner.default_altitude_m` равен `cruise_altitude_m`.
   - `drone_city_nav/tests/trajectory_planner_test.cpp`: проверить, что `planOptimizedTrajectory()` возвращает samples с `z_m == config.default_altitude_m`.

3. Протащить `z_m` через ROS `nav_msgs::Path`.

   Файлы и anchors:
   - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp:58`
   - `drone_city_nav/src/ros_conversions.cpp:243`
   - `drone_city_nav/src/planner_node_debug_publication.cpp:47`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp:262` и `:672`
   - `drone_city_nav/src/planner_node.hpp:83` (`trajectorySamplePoints`)

   Материализуемый результат:
   - Сохранить существующий overload `pathToRos(std::span<const Point2>, header, altitude_m)`.
   - Добавить overload:

     ```cpp
     [[nodiscard]] nav_msgs::msg::Path
     pathToRos(std::span<const TrajectoryPointSample> samples,
               const std_msgs::msg::Header& header);
     ```

   - В новом overload каждая pose получает `x/y` из `sample.point` и `z` из `sample.z_m`.
   - В `publishTrajectoryResult()` оставить `trajectorySamplePoints()` для 2D validation (`pathIsTraversable()`), но публиковать final executable path из `trajectory_result.samples`, а не из временного `std::vector<Point2>`.
   - `last_valid_path_points_`, `logPublishedPathSafety()` и rough route diagnostics остаются `Point2`, потому что prohibited grid и replanning на этом этапе 2D.
   - Empty hold path остаётся empty path без poses.

   Тесты:
   - `drone_city_nav/tests/ros_conversions_test.cpp`: добавить `BuildsPathFromTrajectorySamplesWithPerSampleAltitude`.
   - `drone_city_nav/tests/ros_conversions_test.cpp`: сохранить существующие тесты constant-altitude overload.

4. Сохранять `z_m` при приёме path в offboard.

   Файлы и anchors:
   - `drone_city_nav/include/drone_city_nav/offboard_trajectory_state.hpp:33`
   - `drone_city_nav/src/offboard_trajectory_state.cpp:17`
   - `drone_city_nav/src/offboard_trajectory_state.cpp:131`
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp:270`

   Материализуемый результат:
   - Добавить:

     ```cpp
     [[nodiscard]] std::vector<TrajectoryPointSample>
     pathSamplesFromMessage(const nav_msgs::msg::Path& path);
     ```

   - `pathSamplesFromMessage()` должен:
     - копировать `pose.pose.position.x/y` в `sample.point`;
     - копировать `pose.pose.position.z` в `sample.z_m`;
     - затем заполнить `s_m`, `tangent`, `curvature_1pm` через существующую XY geometry population.
   - `pathPointsFromMessage()` оставить для route/progress diagnostics, но сделать его thin wrapper over samples или оставить явно diagnostics-only.
   - Добавить overload:

     ```cpp
     [[nodiscard]] OffboardTrajectoryState
     buildOffboardTrajectoryState(std::span<const TrajectoryPointSample> path_samples,
                                  const VelocityFollowerConfig& velocity_config);
     ```

   - `Px4OffboardNode::onPath()` строит `candidate_path_samples = pathSamplesFromMessage(path)`, а `candidate_path_points` продолжает использовать только для rough route/goal/progress.
   - `OffboardTrajectoryState::samples` сохраняет `z_m`, а `state.trajectory`, speed profile и shape diagnostics остаются XY-only.

   Тесты:
   - `drone_city_nav/tests/offboard_trajectory_state_test.cpp`: обновить `makePath()` с разными z.
   - Добавить проверку, что `pathSamplesFromMessage()` и `buildOffboardTrajectoryState(samples, ...)` сохраняют z.
   - Добавить negative test: path с non-finite z даёт invalid state и continuity reject до stale-pose reset.

5. Обновить final trajectory RViz path на реальную высоту.

   Файлы и anchors:
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp:30` (`publishFinalTrajectoryDebug`)
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp:179` (`applyReceivedFinalTrajectoryPath`)

   Материализуемый результат:
   - `publishFinalTrajectoryDebug()` больше не собирает временный `std::vector<Point2>` и не вызывает `pathToRos(..., 0.0)`.
   - Он вызывает `pathToRos(final_trajectory_samples_, makeDebugHeader())`.
   - `last_final_trajectory_debug_samples_` остаётся как счётчик.
   - При очистке trajectory publish остаётся пустым.

   Тесты:
   - Если node-level тестов для `Px4OffboardNode` нет, покрыть через `ros_conversions_test` и `offboard_trajectory_state_test`.
   - Добавить script contract в `scripts/tests/test_offboard_telemetry_contract.py`, который проверяет, что `publishFinalTrajectoryDebug()` использует `final_trajectory_samples_` напрямую и не хардкодит `0.0`.

6. Добавить базовые 3D marker helpers и перевести trajectory/drone markers на sample/current altitude.

   Файлы и anchors:
   - `drone_city_nav/src/trajectory_debug_markers.cpp:17` (`markerPoint`)
   - `drone_city_nav/src/trajectory_debug_markers.cpp:85` (`buildTrajectoryDebugMarkers`)
   - `drone_city_nav/src/offboard_debug_markers.cpp:15` (`markerPoint`)
   - `drone_city_nav/src/offboard_debug_markers.cpp:39` (`buildDroneDebugMarkers`)
   - `drone_city_nav/include/drone_city_nav/offboard_debug_markers.hpp:14` (`DroneDebugMarkerState`)
   - `drone_city_nav/CMakeLists.txt:96` (`drone_city_nav_ros_adapters`)

   Материализуемый результат:
   - Добавить helper-модуль `visualization_marker_helpers` в `drone_city_nav_ros_adapters`:

     ```cpp
     [[nodiscard]] geometry_msgs::msg::Point markerPoint(Point3 point);
     [[nodiscard]] geometry_msgs::msg::Point markerPoint(Point2 point, double z_m);
     [[nodiscard]] std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a);
     [[nodiscard]] visualization_msgs::msg::Marker makeMarker(
         const std_msgs::msg::Header& header, std::string_view ns,
         int id, int type);
     ```

   - Убрать дублирующиеся локальные `markerPoint()` из `trajectory_debug_markers.cpp` и `offboard_debug_markers.cpp`.
   - Изменить `buildTrajectoryDebugMarkers()` API так, чтобы он использовал `sample.z_m`:

     ```cpp
     buildTrajectoryDebugMarkers(header, trajectory_samples, speed_profile);
     ```

     Линии speed/curvature можно рисовать на `sample.z_m + 0.04/0.08`, как сейчас рисуется от `marker_z_m`.

   - Изменить `DroneDebugMarkerState`:

     ```cpp
     struct DroneDebugMarkerState {
       bool pose_fresh{false};
       Point2 position{};
       double altitude_m{std::numeric_limits<double>::quiet_NaN()};
       bool altitude_valid{false};
       double heading_rad{0.0};
     };
     ```

   - `Px4OffboardNode::publishOffboardDebugMarkers()` заполняет `altitude_m` из `current_altitude_m_`; если altitude stale/invalid, marker удаляется или использует fallback только явно. Для этапа лучше удалять sphere при invalid altitude, чтобы RViz не показывал ложную высоту.
   - Drone position marker остаётся `SPHERE`, но становится настоящей 3D sphere: `scale.x == scale.y == scale.z`, например `2.5`.
   - Heading arrow start/end получают `z = altitude_m`, а не `ground_z_m + 0.06`.

   Тесты:
   - `drone_city_nav/tests/trajectory_debug_markers_test.cpp`: проверить, что line-list points используют разные `z_m` из samples.
   - `drone_city_nav/tests/offboard_debug_markers_test.cpp`: проверить, что drone sphere `pose.position.z == altitude_m` и `scale.z == scale.x`.
   - `drone_city_nav/tests/offboard_debug_markers_test.cpp`: проверить DELETE markers при stale pose или invalid altitude.
   - Добавить `visualization_marker_helpers_test` или покрыть helpers через existing marker tests. Если helpers не trivial header-only, добавить gtest target в `drone_city_nav/CMakeLists.txt`.

7. Добавить `z` в final trajectory dumps и headless diagnostics.

   Файлы и anchors:
   - `drone_city_nav/src/trajectory_diagnostics_io_csv.cpp:7` (`finalTrajectorySamplesCsvHeader`)
   - `drone_city_nav/src/trajectory_diagnostics_io_csv.cpp:15` (`finalTrajectorySamplesCsvRow`)
   - `drone_city_nav/src/final_trajectory_debug_io.cpp:45`
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp:223` (`Received executable final trajectory` log)
   - `drone_city_nav/src/px4_offboard_node_telemetry.cpp:68` (`Drone path diagnostics`)
   - `drone_city_nav/src/px4_offboard_node_telemetry.cpp:86` (`Drone velocity command diagnostics`)
   - `drone_city_nav/include/drone_city_nav/offboard_blackbox.hpp:26` (`OffboardBlackboxPathTracking`)
   - `drone_city_nav/src/offboard_blackbox.cpp:537`

   Материализуемый результат:
   - CSV header получает `z_m` после `y`, row пишет `sample.z_m`.
   - `FinalTrajectorySamplesCsvInput` не требует нового поля: z уже в samples.
   - Добавить helper для altitude range:

     ```cpp
     struct TrajectoryAltitudeStats {
       bool valid{false};
       double min_z_m;
       double max_z_m;
       double mean_z_m;
     };
     ```

     Его можно держать в `trajectory.hpp/cpp` или локально в offboard diagnostics, но лучше в core, чтобы tests были проще.

   - В `Received executable final trajectory` log добавить `altitude[min=... mean=... max=...]`.
   - В `Drone path diagnostics` добавить `projection_z=...`.
   - В blackbox `tracking` object добавить `projection_z_m`.
   - При желании добавить в blackbox `trajectory_altitude_min_m/max_m/mean_m` рядом с trajectory summary; это полезно для headless сравнения 3D profile.

   Тесты:
   - `drone_city_nav/tests/trajectory_diagnostics_io_csv_test.cpp`: header содержит `z_m`, row содержит z и не содержит `nan`.
   - `drone_city_nav/tests/final_trajectory_debug_io_test.cpp`: CSV строки содержат `x,y,z`.
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`: JSON содержит `projection_z_m` и trajectory altitude range, если добавлена.
   - `scripts/tests/test_offboard_telemetry_contract.py`: contract обновлён под новые z diagnostics.

8. Не менять runtime vertical control в этом этапе, но подготовить seam для будущего vertical profile follower.

   Файлы и anchors:
   - `drone_city_nav/src/px4_offboard_node_control.cpp:272` (`verticalVelocitySetpointNed`)
   - `drone_city_nav/src/px4_offboard_node_control.cpp:299` (`publishVelocityTrajectorySetpoint`)
   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:167` (`planVelocitySetpoint`)

   Материализуемый результат:
   - `verticalVelocitySetpointNed()` продолжает использовать `cruise_altitude_m_`, поэтому flight behavior не меняется.
   - В коде/логах явно зафиксировать, что trajectory `z_m` пока representation/debug altitude, а runtime vertical target на этапе 1 остаётся `cruise_altitude_m_`.
   - Не добавлять `vz` из trajectory profile, `dz/ds`, vertical speed caps, passage speed caps или `max_climb_angle_deg` в этом этапе.

   Тесты:
   - existing `px4_offboard_setpoint_io_test` и velocity/offboard tests должны остаться зелёными.
   - Добавить script contract, что `verticalVelocitySetpointNed()` всё ещё использует `cruise_altitude_m_` до следующего этапа, если это важно для предотвращения случайной behavior change.

9. Обновить документацию и RViz описание.

   Файлы и anchors:
   - `docs/overview.md:71`
   - `docs/drone_control.md:22`
   - `docs/navigation_pipeline.md:106`
   - `docs/rviz.md:37`
   - `docs/diagnostics.md:32`
   - `docs/configuration.md:30` и `docs/configuration.md:171`
   - `drone_city_nav/rviz/city_nav_debug.rviz:47`

   Материализуемый результат:
   - Обновить docs на английском:
     - planner still uses 2D XY planning;
     - executable trajectory samples now carry `z_m`;
     - default `z_m` is `cruise_altitude_m`;
     - offboard vertical control still holds cruise altitude in Stage 1;
     - RViz final trajectory and drone marker are displayed at real altitude;
     - dumps include `z_m`.
   - Если marker namespace/API не меняются, `city_nav_debug.rviz` можно не трогать. Если добавляется отдельный namespace для 3D helpers/altitude markers, включить его там.

   Тесты:
   - `make test-scripts` должен пройти после обновления doc/script contracts.
   - Если docs содержат user-facing английский текст, не добавлять русские комментарии/доки в репозиторий.

10. Обновить build targets при добавлении новых файлов.

    Файлы и anchors:
    - `drone_city_nav/CMakeLists.txt:96` (`drone_city_nav_ros_adapters`)
    - `drone_city_nav/CMakeLists.txt:334` (`trajectory_debug_markers_test`)
    - `drone_city_nav/CMakeLists.txt:471` (`offboard_debug_markers_test`)
    - при новом helper test добавить target рядом с marker tests.

    Материализуемый результат:
    - Новый `src/visualization_marker_helpers.cpp` добавлен в `drone_city_nav_ros_adapters`.
    - Новые/обновлённые tests линкуются с `drone_city_nav_core` или `drone_city_nav_ros_adapters` по существующему паттерну.
    - Нет ad-hoc CMake workflow; только текущий ament target-based style.

## Verification plan

Для реализации этого плана запускать проверки в контейнерном workflow.

Минимальный scoped набор после C++ изменений:

```bash
./scripts/dev_shell.sh
make format
ctest --test-dir build/drone_city_nav --output-on-failure -R 'trajectory_test|trajectory_planner_test|ros_conversions_test|offboard_trajectory_state_test|trajectory_debug_markers_test|offboard_debug_markers_test|final_trajectory_debug_io_test|trajectory_diagnostics_io_test|offboard_blackbox_test'
make test-scripts
make quality
```

Если build dir отсутствует, сначала:

```bash
./scripts/build.sh
```

Широкая проверка перед merge/publication:

```bash
./scripts/test.sh
make test-scripts
make quality
```

Simulation smoke не обязателен для чистого representation/RViz этапа, но полезен после успешных unit/script checks:

```bash
SMOKE_DURATION_S=90 ./scripts/sim_headless.sh
```

Headless post-run checks:

- `log/final_trajectory_samples/latest.csv` содержит `z_m` и все rows имеют `z_m == 18.0` при текущем `urban_mvp.yaml`.
- `log/offboard_blackbox.jsonl` содержит `projection_z_m` и/или trajectory altitude stats.
- `/drone_city_nav/final_trajectory_path` в rosbag/RViz имеет non-zero `pose.position.z`.

## Testing strategy

1. Категория 1: без рефакторинга.

   Использовать, если реализация ограничится добавлением `z_m` в существующие structs/functions.

   Покрытие:
   - `trajectory_test`: z finite validation, altitude interpolation, XY projection unchanged.
   - `ros_conversions_test`: per-sample z -> `nav_msgs::Path`.
   - `offboard_trajectory_state_test`: Path z сохраняется в accepted samples.
   - `trajectory_diagnostics_io_csv_test` и `final_trajectory_debug_io_test`: CSV schema includes `z_m`.
   - `offboard_blackbox_test`: JSON includes z diagnostics.

2. Категория 2: лёгкий рефакторинг.

   Использовать для выноса marker helpers и overloads без изменения flight logic.

   Покрытие:
   - `trajectory_debug_markers_test`: speed/curvature line-list uses sample z.
   - `offboard_debug_markers_test`: drone sphere at altitude, delete behavior when pose/altitude invalid.
   - Новый helper test, если helper содержит logic beyond simple assignment.
   - `scripts/tests/test_offboard_telemetry_contract.py`: статический контракт на отсутствие hardcoded `z=0.0` в final trajectory debug path.

3. Категория 3: тяжёлый/интеграционный.

   Нужна, если реализация случайно затронет planner publication, offboard path acceptance или node callbacks шире, чем описано.

   Покрытие:
   - `./scripts/test.sh` для всего package.
   - `make test-scripts` для script-level contracts.
   - `make quality` как обязательный pre-commit gate.
   - `./scripts/sim_headless.sh` как smoke test, если есть риск, что z propagation повлиял на offboard startup, RViz/debug publishers или path acceptance.

## Risks and tradeoffs

- Риск: случайно начать считать длину/curvature/speed profile в 3D и изменить скорость/плавность текущего 2D полёта.
  - Митигировать тестами на неизменность XY projection, length, curvature и speed profile при constant z.

- Риск: `PlannerNode` и `Px4OffboardNode` имеют отдельные `cruise_altitude_m` параметры. В текущем `urban_mvp.yaml` оба равны `18.0`, но drift между ними может дать path.z, отличающийся от altitude hold target.
  - На этапе 1 логировать accepted trajectory altitude range и altitude target. В будущем решать через executable trajectory artifact/config fingerprint.

- Риск: CSV schema change может сломать внешние scripts, которые ожидают старый порядок колонок.
  - Добавлять `z_m` как явное новое поле после `y`, обновить tests/docs, не переименовывать существующие поля без необходимости.

- Риск: RViz markers на реальной высоте станут визуально менее заметными при текущем camera/view.
  - Проверить `city_nav_debug.rviz`; при необходимости добавить docs note, но не менять runtime поведение.

- Риск: удаление ground-based marker assumptions может сломать tests/contract, которые искали `ground_z_m`.
  - Обновить tests под новый explicit altitude contract.

- Риск: non-finite z из path теперь должен делать candidate invalid. Это правильно для 3D representation, но может выявить старые publisher bugs.
  - Negative test на non-finite z и reject before stale-pose reset.

- Tradeoff: offboard vertical control пока не следует `sample.z_m`.
  - Это сознательное ограничение этапа 1. Оно сохраняет flight behavior и создаёт представление/диагностику для следующего этапа vertical profile follower.

## Open questions

- Нужно ли считать `/drone_city_nav/path` user-facing final executable trajectory или оставить отдельный rough route topic для 2D route? Сейчас `/path` одновременно является входом offboard и RViz rough-route display. В этом этапе публикуем executable path с z, а rough route diagnostics остаются внутри logs/progress.
- При invalid altitude in drone marker лучше удалять marker или fallback на `cruise_altitude_m`? Для отладки 3D я предлагаю удалять, чтобы не показывать ложную высоту. Если UX важнее, можно fallback-ить на `cruise_altitude_m` и логировать `altitude_valid=false`.
- Нужно ли уже сейчас добавлять `trajectory_altitude_min/max/mean` в `TrajectoryPlannerStats`, или держать altitude stats только в offboard/debug layer? Для этапа 1 достаточно offboard/debug stats; в planner stats есть смысл добавить позже, когда появятся non-constant z profiles.
