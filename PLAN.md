# Context

Нужно перейти от текущей схемы `lidar -> поиск пути` к схеме
`lidar -> память препятствий -> поиск пути`.

Цель: дрон должен строить настоящую 2D-память препятствий по данным лидара,
собственной позиции и курса, а планировщик должен использовать эту накопленную
карту, а не мгновенный скан. Логика должна оставаться переносимой на реальный
дрон: без чтения Gazebo world state, без знания заранее заданных зданий и без
симуляторных shortcut-ов. Для симуляции допустимо использовать PX4 estimator
topics, потому что такой же контракт доступен на реальном PX4, но код должен
быть оформлен как вход навигационной позы, а не как зависимость от Gazebo.

Текущий код уже содержит несколько важных базовых свойств:

- `drone_city_nav/src/planner_node.cpp:100` подписывается на `/scan`, а
  `drone_city_nav/src/planner_node.cpp:182` напрямую интегрирует лидар в
  локальный `OccupancyGrid2D`.
- `drone_city_nav/src/planner_node.cpp:284` в том же узле делает inflation,
  A*, smoothing и публикует path.
- `drone_city_nav/src/px4_offboard_node.cpp:173` уже умеет зависать при пустом
  пути, а `drone_city_nav/src/px4_offboard_node.cpp:504` выбирает hold target,
  если пути нет.
- `drone_city_nav/src/lidar_debug_node.cpp:266` пишет снимки лидара/grid/path,
  а `drone_city_nav/src/lidar_debug_node.cpp:717` публикует live radar markers.
- `drone_city_nav/worlds/generated_city.sdf:12` и
  `drone_city_nav/worlds/generated_city.sdf:13` уже включают NavSat и
  Magnetometer systems; PX4 SITL получает GPS/compass-подобные данные через
  стандартную симуляционную цепочку.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. Этот `PLAN.md` был создан в первом
раунде планирования и в текущем раунде доработан по peer review: исправлены
команды `runlim`, уточнён реальный `gps_compass` adapter и добавлено требование
clipping для lidar rays на границах grid.

Прочитаны локальные источники:

- `README.md`
- `CONTRIBUTING.md`
- `Makefile`
- `drone_city_nav/CMakeLists.txt`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/config/real_drone_template.yaml`
- `drone_city_nav/launch/city_nav.launch.py`
- `drone_city_nav/src/planner_node.cpp`
- `drone_city_nav/src/lidar_debug_node.cpp`
- `drone_city_nav/src/px4_offboard_node.cpp`
- `drone_city_nav/src/mission_monitor_node.cpp`
- `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`
- `drone_city_nav/src/occupancy_grid.cpp`
- `drone_city_nav/include/drone_city_nav/astar_planner.hpp`
- `drone_city_nav/src/astar_planner.cpp`
- `drone_city_nav/include/drone_city_nav/path_smoothing.hpp`
- `drone_city_nav/src/path_smoothing.cpp`
- `drone_city_nav/tests/planner_core_test.cpp`
- `drone_city_nav/rviz/city_nav_debug.rviz`
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
- `drone_city_nav/worlds/generated_city.sdf`
- `scripts/run_city_mvp.sh`
- `docs/MVP_SIMULATION.md`

# Detected stack/profiles

Основной стек: C++20 ROS 2 workspace, ament CMake package `drone_city_nav`,
сборка через `colcon`, симуляция через Gazebo + PX4 SITL, визуализация через
RViz.

Прочитанные verification profiles:

- `generic.md`: обязателен для любого workspace, задаёт repo-approved command
  discovery, scoped checks и reporting.
- `cpp.md`: обязателен, потому что в проекте есть `CMakeLists.txt`, `Makefile`
  и C++ файлы в `drone_city_nav/src` и `drone_city_nav/include`.

Rust profile не читался: в workspace не найден `Cargo.toml` и задача не
затрагивает Rust-код.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md` и `Makefile` найдены утверждённые команды:

- Вход в контейнер: `./scripts/dev_shell.sh`.
- Сборка: `make build`.
- Эквивалент сборки:
  `colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
- Тесты: `make test`.
- Эквивалент тестов:
  `ctest --test-dir build/drone_city_nav --output-on-failure`.
- Качество: `make quality` или `./scripts/check_cpp_quality.sh`.
- Форматирование изменённых C++ файлов:
  `./scripts/format_cpp_changed.sh`.
- GUI simulation: `make sim-gui` или `./scripts/run_city_mvp.sh`.
- Headless smoke:
  `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`.
- Full mission validation:
  `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh`.

Для долгих запусков использовать `/home/formi/.local/bin/runlim`, например:

```bash
/home/formi/.local/bin/runlim timeout 420 \
  env HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh
```

# Affected components

- Core grid/map logic:
  `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`,
  `drone_city_nav/src/occupancy_grid.cpp`, новые
  `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`,
  `drone_city_nav/src/obstacle_memory.cpp`.
- Navigation pose / GPS / compass adaptation:
  новый `drone_city_nav/include/drone_city_nav/navigation_pose.hpp` и
  ROS-адаптер в новом `drone_city_nav/src/obstacle_memory_node.cpp`.
- Planner:
  `drone_city_nav/src/planner_node.cpp`.
- Debug/GUI:
  `drone_city_nav/src/lidar_debug_node.cpp`,
  `drone_city_nav/rviz/city_nav_debug.rviz`.
- Launch/config:
  `drone_city_nav/launch/city_nav.launch.py`,
  `drone_city_nav/config/urban_mvp.yaml`,
  `drone_city_nav/config/real_drone_template.yaml`.
- Build:
  `drone_city_nav/CMakeLists.txt`.
- Tests:
  `drone_city_nav/tests/planner_core_test.cpp`, при росте тестов лучше
  выделить новый `drone_city_nav/tests/obstacle_memory_test.cpp`.
- Documentation:
  `docs/MVP_SIMULATION.md`, возможно `README.md`.

# Implementation steps

1. Добавить переносимую модель навигационной позы.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/navigation_pose.hpp`
   - при необходимости `drone_city_nav/src/navigation_pose.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат: общий тип, который описывает только то, что
   нужно алгоритму карты: локальная позиция в метрах, yaw от компаса/оценщика,
   altitude, timestamp и валидность.

   Минимальный контракт:

   ```cpp
   struct NavigationPose2D {
     Pose2 pose;
     double altitude_m;
     rclcpp::Time stamp; // in ROS adapter only, or int64 nanoseconds in core
     bool position_valid;
     bool yaw_valid;
     bool altitude_valid;
   };
   ```

   Для core-части лучше не тянуть `rclcpp`; если timestamp нужен в core,
   хранить `std::int64_t stamp_ns`.

2. Добавить `gps_compass` adapter и GPS-to-local frame helper для реального
   режима.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/navigation_pose.hpp`
   - `drone_city_nav/src/navigation_pose.cpp`
   - `drone_city_nav/tests/obstacle_memory_test.cpp`

   Материализуемый результат: отдельный adapter, который принимает
   `sensor_msgs/NavSatFix` и compass/yaw input, валидирует данные, и выдаёт
   `NavigationPose2D` в той же локальной системе координат, что и planner. Для
   совместимости с текущим проектом использовать `Point2{x = north_m,
   y = east_m}`, потому что `docs/MVP_SIMULATION.md:31` фиксирует PX4 local `x`
   как north и `y` как east.

   Обязательные правила adapter:

   - `NavSatFix.status.status` должен быть не хуже `STATUS_FIX`; сообщения
     `STATUS_NO_FIX`, NaN latitude/longitude/altitude и устаревший timestamp
     отклоняются.
   - Если `position_covariance_type` известен, adapter сравнивает горизонтальную
     covariance с параметром `max_gps_horizontal_variance_m2`; слишком шумные
     GPS samples не обновляют pose.
   - `home/origin` задаётся параметрами `origin_latitude_deg`,
     `origin_longitude_deg`, `origin_altitude_m` или автоматически фиксируется
     первым валидным GPS fix при `auto_initialize_origin=true`. Автоинициализация
     должна логироваться один раз.
   - Compass/yaw принимается из `sensor_msgs/Imu` orientation quaternion или из
     отдельного yaw topic. Quaternion нормализуется и конвертируется в yaw вокруг
     вертикальной оси; невалидная orientation covariance или NaN yaw отклоняются.
   - Frame convention задаётся явно: GPS projector выдаёт north/east в local
     navigation frame, lidar beam rotation применяет yaw в этом же frame, а
     параметры `yaw_offset_rad`, `magnetic_declination_rad` и
     `compass_to_body_yaw_offset_rad` документируют калибровку компаса и
     ориентацию лидара относительно корпуса.
   - Adapter не читает Gazebo state и не использует заранее известные building
     coordinates. В симуляции default может оставаться `pose_source:
     px4_local_position`, но `gps_compass` путь должен быть полноценным и
     покрытым unit tests.

   Pseudocode:

   ```cpp
   if (!validNavSatFix(fix) || !fresh(fix.header.stamp)) {
     return std::nullopt;
   }
   if (!origin.initialized) {
     origin = makeOriginFromFix(fix);
   }
   north_m = deg_to_rad(latitude_deg - origin_latitude_deg) * earth_radius_m;
   east_m = deg_to_rad(longitude_deg - origin_longitude_deg) *
            earth_radius_m * cos(deg_to_rad(origin_latitude_deg));
   yaw_rad = normalizeYaw(imuYaw + magnetic_declination_rad +
                          compass_to_body_yaw_offset_rad + yaw_offset_rad);
   return NavigationPose2D{Pose2{Point2{north_m, east_m}, yaw_rad}, ...};
   ```

   Это не Gazebo-зависимость. В симуляции можно продолжать использовать PX4
   local position как уже готовую оценку, а на железе подключить GPS/compass
   topics через тот же адаптер.

   Тесты этого шага обязательны, а не optional:

   - WGS84 -> local projection: изменение latitude даёт `+north`, изменение
     longitude даёт `+east`, знаки соответствуют текущему `Point2{x=north,
     y=east}`.
   - Origin handling: manual origin и auto-initialized origin дают ожидаемый
     local zero для первого fix.
   - `NavSatFix` validation: no-fix, NaN, stale timestamp и превышенная
     covariance не обновляют pose.
   - Compass conversion: quaternion yaw, отдельный yaw topic, yaw offset и
     magnetic declination дают ожидаемый normalized yaw.
   - Frame convention: один и тот же yaw поворачивает lidar endpoint в ту же
     сторону, которую ожидает memory integration.

3. Добавить core-класс памяти препятствий.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`
   - `drone_city_nav/src/obstacle_memory.cpp`
   - `drone_city_nav/tests/obstacle_memory_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат: `ObstacleMemoryGrid`, который хранит persistent
   occupancy отдельно от planner node. Он должен принимать beams + pose и
   обновлять карту через ray tracing.

   Рекомендуемый контракт:

   ```cpp
   struct LaserScan2DView {
     std::span<const float> ranges;
     double angle_min_rad;
     double angle_increment_rad;
     double range_min_m;
     double range_max_m;
   };

   struct ObstacleMemoryConfig {
     double max_lidar_range_m;
     double range_hit_epsilon_m;
     double hit_obstacle_depth_m;
     int scan_stride;
     int occupied_hit_threshold;
     int free_miss_threshold;
   };

   class ObstacleMemoryGrid {
   public:
     void integrateScan(const Pose2& pose, const LaserScan2DView& scan,
                        const ObstacleMemoryConfig& config);
     void rebuildInflation(double radius_m);
     const OccupancyGrid2D& rawGrid() const;
     const OccupancyGrid2D& inflatedGrid() const;
     OccupancyGrid2D localWindow(Point2 center, double radius_m) const;
   };
   ```

   Логика интеграции:

   ```cpp
   for beam in scan with scan_stride:
     hit = finite(range) && range >= min && range < max - epsilon
     used_range = hit ? range : max
     angle = pose.yaw_rad + scan_yaw_offset + angle_min + i * angle_increment
     end = pose.position + used_range * unit(angle)
     clipped_end = clipSegmentToGridBounds(pose.position, end)
     if no clipped in-bounds segment:
       continue
     mark in-bounds ray cells through clipped_end as free evidence
     if hit and original endpoint is inside grid:
       mark endpoint and optional obstacle_depth cells as occupied evidence
   ```

   Обязательное отличие от текущего `OccupancyGrid2D::markRay` на
   `drone_city_nav/src/occupancy_grid.cpp:133`: ray integration не должен
   полностью пропускать beam только потому, что endpoint вне grid. Если старт
   внутри grid, а endpoint снаружи, нужно обновить free evidence до границы
   карты. Для hit за пределами grid endpoint не помечается occupied, но
   in-bounds часть луча всё равно очищается. Для hit около границы
   `hit_obstacle_depth_m` должен clip-аться и не писать вне bounds.

   Важная деталь: free ray не должен мгновенно стирать устойчивое препятствие
   от одного случайного промаха. Нужны hit/miss counters или log-odds. Для MVP
   достаточно bounded score:

   ```cpp
   score[cell] = clamp(score[cell] + hit_weight, min_score, max_score)
   score[cell] = clamp(score[cell] - miss_weight, min_score, max_score)
   occupied if score >= occupied_threshold
   free if score <= free_threshold
   unknown otherwise
   ```

4. Добавить `obstacle_memory_node`.

   Файлы:

   - `drone_city_nav/src/obstacle_memory_node.cpp`
   - `drone_city_nav/CMakeLists.txt`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/config/real_drone_template.yaml`
   - `drone_city_nav/launch/city_nav.launch.py`

   Материализуемый результат: отдельный ROS 2 узел, который подписывается на
   lidar и navigation pose, строит память препятствий и публикует:

   - `/drone_city_nav/obstacle_memory_grid`: полная raw/persistent карта.
   - `/drone_city_nav/obstacle_memory_local_grid`: локальное окно вокруг дрона
     для GUI.
   - `/drone_city_nav/obstacle_memory_inflated_grid`: карта после inflation,
     если нужно отлаживать именно planning clearance.
   - `tf` transform `map -> obstacle_memory_view`, если добавляем `tf2_ros`,
     чтобы RViz мог держать локальную карту с дроном в центре.

   Источники позы через параметры:

   - `pose_source: px4_local_position` для текущей симуляции и реального PX4
     estimator output: `/fmu/out/vehicle_local_position_v1`.
   - `pose_source: gps_compass` для прямого режима:
     `sensor_msgs/NavSatFix` + compass yaw из `sensor_msgs/Imu` или отдельного
     yaw topic. Этот режим использует local projector из шага 2.

   Узел должен логировать:

   - выбранный `pose_source`;
   - первый валидный GPS/local pose и yaw;
   - количество processed beams/hits;
   - raw/free/occupied/unknown/inflated cell counts;
   - размер локального GUI окна;
   - причину пропуска scan, если нет pose/yaw/altitude.

5. Перевести `planner_node` на карту памяти.

   Файл: `drone_city_nav/src/planner_node.cpp`.

   Материализуемый результат: planner больше не подписывается на `/scan` и не
   содержит алгоритм построения карты из лидара. Он подписывается на
   `/drone_city_nav/obstacle_memory_grid`, конвертирует сообщение в
   `OccupancyGrid2D`, выполняет inflation, A*, smoothing и публикует path.

   Конкретные anchors:

   - удалить или перенести lidar subscription из
     `drone_city_nav/src/planner_node.cpp:100`;
   - удалить/перенести `onScan` из `drone_city_nav/src/planner_node.cpp:182`;
   - удалить/перенести `markObstacleDepth` из
     `drone_city_nav/src/planner_node.cpp:255`;
   - оставить планирование в `replanAndPublish` на
     `drone_city_nav/src/planner_node.cpp:284`, но входной grid брать из
     последнего memory topic;
   - сохранить поведение `publishFallbackPath` на
     `drone_city_nav/src/planner_node.cpp:476`: при отсутствии карты или пути
     публиковать empty path/hold, а не direct-to-goal fallback.

   Pseudocode:

   ```cpp
   void onMemoryGrid(const nav_msgs::msg::OccupancyGrid& msg) {
     memory_grid_ = occupancyGridFromMessage(msg, occupied_threshold);
     memory_grid_seen_ = true;
   }

   void replanAndPublish() {
     if (!pose_valid_ || !memory_grid_seen_) {
       publishPath({});
       return;
     }
     auto planning_grid = *memory_grid_;
     planning_grid.rebuildInflation(inflation_radius_m_);
     plan with AStarPlanner on planning_grid;
   }
   ```

6. Обновить GUI: вместо live radar показывать локальную карту памяти.

   Файлы:

   - `drone_city_nav/src/lidar_debug_node.cpp`
   - `drone_city_nav/rviz/city_nav_debug.rviz`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `docs/MVP_SIMULATION.md`

   Материализуемый результат:

   - RViz display `Lidar Radar Overlay` заменить на `Local Obstacle Memory`.
   - Основной Map display подписать на
     `/drone_city_nav/obstacle_memory_local_grid`.
   - Если используется `tf2_ros`, RViz view target frame сделать
     `obstacle_memory_view`, чтобы дрон визуально оставался в центре окна.
   - `lidar_debug_node` оставить как headless/debug recorder, но его PPM и JSONL
     должны отражать memory-grid stats, а не только мгновенный scan.

   Важно: полный global memory grid может быть большим. Для GUI публиковать
   cropped grid радиуса `memory_view_radius_m`, например 45 m, с origin
   `(-radius, -radius)` в frame `obstacle_memory_view` или с origin
   `(pose.x - radius, pose.y - radius)` в frame `map`, если откажемся от TF.

7. Обновить launch и параметры.

   Файлы:

   - `drone_city_nav/launch/city_nav.launch.py`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/config/real_drone_template.yaml`

   Материализуемый результат:

   - launch запускает `obstacle_memory_node` между bridge и planner;
   - lidar/map параметры (`lidar_topic`, `scan_yaw_offset_rad`,
     `use_px4_heading_for_scan`, `swap_lidar_xy_to_local_frame`,
     `max_lidar_range_m`, `range_hit_epsilon_m`, `hit_obstacle_depth_m`,
     `scan_stride`, `min_mapping_altitude_m`) переезжают из `planner_node` в
     `obstacle_memory_node`;
   - planner получает `obstacle_memory_grid_topic`;
   - real template содержит явные параметры GPS/compass topics и `pose_source`.

8. Обновить build config.

   Файл: `drone_city_nav/CMakeLists.txt`.

   Материализуемый результат:

   - добавить `src/obstacle_memory.cpp` в `drone_city_nav_core`;
   - добавить executable `obstacle_memory_node`;
   - добавить зависимости `sensor_msgs`, `nav_msgs`, `geometry_msgs`;
   - если выбран centered RViz через TF, добавить `tf2_ros`.

9. Добавить unit tests для памяти препятствий.

   Файл: `drone_city_nav/tests/obstacle_memory_test.cpp` или расширение
   `drone_city_nav/tests/planner_core_test.cpp`.

   Материализуемый результат: автотесты без ROS runtime, проверяющие core
   алгоритм.

   Обязательные happy-path tests:

   - scan hit при `yaw=0` ставит occupied endpoint в ожидаемую cell;
   - scan hit при `yaw=pi/2` поворачивает endpoint корректно;
   - occupied endpoint сохраняется в памяти между replanning cycles;
   - local window вокруг `Point2{cx, cy}` содержит препятствие в правильном
     относительном месте.

   Negative-path tests:

   - invalid/too-short range игнорируется;
   - scan без валидного pose/yaw не меняет карту;
   - one free miss не стирает устойчивый obstacle, если включены thresholds.

   Edge-case tests:

   - beams за границей global grid не падают;
   - ray clipping: луч изнутри grid к endpoint вне grid обновляет free cells до
     границы, а не делает полный no-op;
   - hit endpoint вне grid не создаёт occupied вне bounds, но очищает
     in-bounds часть луча;
   - obstacle_depth не выходит за bounds;
   - inflation вокруг remembered obstacle блокирует safety radius.

10. Добавить интеграционный тест planner-on-memory contract.

    Файл: `drone_city_nav/tests/obstacle_memory_test.cpp` или
    `drone_city_nav/tests/planner_core_test.cpp`.

    Материализуемый результат: тест строит memory grid с препятствием,
    передаёт её A* pipeline и проверяет, что путь огибает blocked/inflated
    cells. Это закрывает новый контракт `memory -> search`.

11. Обновить документацию.

    Файлы:

    - `docs/MVP_SIMULATION.md`
    - `README.md`

    Материализуемый результат:

    - описать новую схему `lidar -> obstacle_memory_node -> planner_node`;
    - зафиксировать, что planner не читает live lidar;
    - описать topic list для global/local memory;
    - описать GUI: локальная карта памяти вокруг дрона вместо radar overlay;
    - описать real-drone mode: PX4 local position как estimator output или
      `gps_compass` adapter, без Gazebo world state.

12. Обновить headless smoke checks.

    Файл: `scripts/run_city_mvp.sh`.

    Материализуемый результат:

    - проверять в `check_headless_run`, что memory node получил scan и pose;
    - проверять, что опубликована карта памяти;
    - сохранить проверки planner path/offboard/mission;
    - лог failure должен печатать tail ROS/Gazebo/PX4 logs как сейчас.

    Новые patterns:

    ```bash
    require_log_pattern "obstacle memory receives lidar" "${ros_log_file}" \
      "Obstacle memory update:"
    require_log_pattern "obstacle memory publishes local grid" "${ros_log_file}" \
      "Published obstacle memory local grid"
    ```

# Verification plan

После реализации запускать только repo-approved commands.

Быстрый локальный цикл после C++ изменений:

```bash
./scripts/format_cpp_changed.sh
./scripts/check_cpp_quality.sh
```

Минимальный build/test scope:

```bash
make build
ctest --test-dir build/drone_city_nav --output-on-failure
```

Headless smoke после успешных unit tests:

```bash
/home/formi/.local/bin/runlim timeout 180 \
  env HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
```

Полный mission validation перед финальным коммитом реализации:

```bash
/home/formi/.local/bin/runlim timeout 420 \
  env HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh
```

GUI smoke вручную, если есть X11/GPU:

```bash
./scripts/run_city_mvp.sh
```

Ожидаемые признаки успеха в логах:

- `Obstacle memory ready: pose_source=...`
- `First valid navigation pose`
- `First lidar scan`
- `Obstacle memory update: processed=... hits=... occupied=...`
- `Planning summary: ... occupied=... inflated=...`
- `Published path: waypoints=...`
- `Offboard summary: ... distance_to_mission_goal=...`
- при `MISSION_CHECK=1`: `MISSION_RESULT success=true`

# Testing strategy

Категория 1, без рефакторинга:

- Использовать существующие tests в `planner_core_test.cpp`.
- Добавить небольшой тест на `OccupancyGrid2D` только если понадобится новый
  helper для message conversion или local window.

Категория 2, лёгкий рефакторинг:

- Вынести mapping из `planner_node` в `ObstacleMemoryGrid`.
- Покрыть core memory unit tests без ROS runtime.
- Покрыть `gps_compass` adapter unit tests: WGS84 projection, origin handling,
  `NavSatFix` validity/covariance/staleness, compass quaternion/yaw conversion,
  yaw offsets и magnetic declination.
- Покрыть boundary ray clipping unit tests: free ray к endpoint вне grid, hit
  outside grid и obstacle depth near boundary.
- Проверить planner-on-memory contract через C++ unit test.
- Это основной рекомендуемый путь: изменение архитектурное, но ограничено
  текущим package.

Категория 3, тяжёлый рефакторинг:

- Добавить полноценные ROS launch tests с test nodes, publishers/subscribers и
  assertions по topics.
- Добавить TF-based RViz follow frame и тестировать наличие transform.
- Добавить отдельный hardware abstraction package.
- Это полезно позже, но для текущей задачи можно ограничиться category 2 +
  headless mission validation, чтобы не раздуть MVP.

# Risks and tradeoffs

- Frame convention риск: текущий проект использует PX4 local `x=north`,
  `y=east`, а Gazebo визуально ENU-like. Ошибка знака или swap снова приведёт к
  смещению препятствий. Проверять yaw/endpoint unit tests и PPM snapshots.
- Compass/yaw риск: в симуляции сейчас `use_px4_heading_for_scan=false`,
  потому что scan уже aligned with map frame. Для реального режима yaw нужен.
  Нужно оставить параметр frame alignment и явно логировать режим.
- Persistent memory риск: слишком агрессивная память может оставить ложные
  препятствия и заблокировать маршрут. Слишком агрессивное clearing может
  стереть реальные здания. Нужны thresholds/log-odds и тесты.
- Planner contract риск: если planner получает карту с уже inflated cells и сам
  ещё раз делает inflation, clearance станет завышенным. Нужно разделить raw
  memory topic и planning/inflated debug topic.
- GUI риск: RViz Map display показывает grid в fixed frame; чтобы дрон был
  реально в центре экрана, может потребоваться dynamic TF target frame. Без TF
  можно показать cropped map around drone, но камера RViz не будет
  автоматически следовать за ним.
- Performance риск: полная grid 115x175 при 0.5 m сейчас небольшая, но
  persistent memory + local crop + inflation на каждом scan может стать
  дорогим при росте карты. Нужны throttled publish rates и отдельные periods.
- Реальный GPS риск: GPS шум и compass drift будут двигать/вращать карту.
  Нужны параметры origin, yaw offset, min altitude и возможно фильтрация позы.
- Boundary mapping риск: если clipping лучей у границ grid не реализовать, lidar
  beams с endpoint вне карты будут терять free evidence. Это особенно заметно
  рядом с краями global grid и на реальном маршруте с rolling/global map.
- Safety риск: это всё ещё не certified collision avoidance. Даже с памятью
  препятствий нужен geofence, failsafe, RC override и staged tests.

# Что могло сломаться

- Поведение планировщика: planner может начать ждать memory grid и публиковать
  empty path, если `obstacle_memory_node` не запущен или topic name не совпал.
  Проверка: `./scripts/check_cpp_quality.sh` и headless smoke должны видеть
  `Obstacle memory update` и `Published path`.
- Контракт topics: старый `/drone_city_nav/occupancy_grid` использовался RViz и
  lidar debug. После разделения raw/local/inflated topics можно сломать
  визуализацию. Проверка: RViz config и ROS log topics, плюс PPM snapshots.
- Совместимость real template: перенос lidar params из planner в memory node
  может оставить старые параметры без эффекта. Проверка: `real_drone_template`
  должен содержать все новые topics/pose_source параметры.
- Headless validation: `scripts/run_city_mvp.sh` patterns могут устареть после
  переименования логов. Проверка: запустить headless smoke и full mission.
- Build/CMake: добавление новых executable/source/dependency может сломать
  colcon build. Проверка: `make build`.
- Tests: перенос mapping logic из planner может потребовать обновить unit
  tests. Проверка: `ctest --test-dir build/drone_city_nav --output-on-failure`.
- Производительность: карта памяти и local crop могут увеличить CPU при 15 Hz
  lidar. Проверка: throttle logs, publish period и headless run без missed
  deadlines/lag.
- Алгоритм у границы карты: если clipping лучей реализован неправильно, память
  может оставлять ложные occupied/unknown зоны или писать за bounds. Проверка:
  unit tests на free ray clipping, hit outside grid и obstacle depth near
  boundary.
- GPS/compass adapter: неверная обработка covariance, stale timestamps,
  magnetic declination или ENU/NED convention может сместить всю карту
  препятствий. Проверка: unit tests на projection/yaw conversion и debug log
  первого валидного origin/yaw.

# Open questions

- Какой real hardware topic считать основным для GPS: `sensor_msgs/NavSatFix`,
  PX4 `VehicleGlobalPosition` или PX4 `VehicleGpsPosition`? План поддерживает
  `sensor_msgs/NavSatFix` как portable ROS API и PX4 local estimator как
  текущий default.
- Какой compass/yaw topic будет на реальном дроне: `sensor_msgs/Imu`
  orientation, PX4 attitude/heading или отдельный heading topic? Нужен
  параметризуемый adapter; default для текущей симуляции остаётся PX4 local
  heading/alignment mode.
- Нужна ли local GUI map north-up или body-forward? Для отладки маршрута проще
  north-up с дроном в центре; body-forward можно добавить параметром позже.
- Нужна ли долговременная память за пределами текущего global grid? Для MVP
  достаточно существующих bounds из `urban_mvp.yaml`; для реального города
  понадобится rolling/global tiling.
