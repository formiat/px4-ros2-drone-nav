# План: движение по дуге в повороте

## Context

Цель: заменить временную модель "ломаная A* + торможение по сырому углу" на
траекторию из типизированных участков `line/arc` и профиль скорости по длине
этой траектории.

Финальная архитектура:

```text
A*
  -> line-of-sight / collapse
  -> optional tunnel / centerline
  -> corner rounding
  -> trajectory: line + arc + line + arc ...
  -> velocity profile over trajectory
  -> velocity setpoint follower
  -> PX4
```

Ключевое требование: скорость перед поворотом и в повороте должна считаться от
реального радиуса дуги, а не от сырого угла ломаной. Для дуги:

```text
curvature = 1 / R
v_arc = sqrt(a_lat_max * R)
```

Два одинаковых 90-градусных угла должны давать разную допустимую скорость, если
радиус дуги разный: узкая улица даёт маленький радиус и низкую скорость, широкая
улица даёт большой радиус и более высокую скорость.

Важно не превращать дугу в основной набор waypoint-ов для управления. Внутри
offboard follower должна появиться typed trajectory, а PX4 должен получать
velocity setpoint по касательной к текущему участку траектории плюс bounded
cross-track correction.

## Investigation Context

`INVESTIGATION.md` в workspace отсутствует, поэтому входного исследовательского
артефакта нет. План основан на прямом чтении текущего локального кода.

Текущее состояние:

- `drone_city_nav/src/planner_node.cpp:708` строит published path из A* cells,
  применяет `smoothPathWithStats()` и `collapseCollinearPath()`, проверяет
  `pathIsTraversable()`, затем публикует `nav_msgs::msg::Path`.
- `drone_city_nav/src/planner_node.cpp:1045` публикует итоговую
  `/drone_city_nav/prohibited_grid`. Это правильный источник для проверки дуг:
  он уже содержит единожды раздутую planning map.
- `drone_city_nav/src/planner_node.cpp:1061` публикует path через
  `pathToRos(..., kGroundDebugZ)`. Planner пока лучше оставить источником
  грубой ломаной, а не владельцем динамики полёта.
- `drone_city_nav/src/path_smoothing.cpp:96` делает line-of-sight smoothing по
  `OccupancyGrid2D::isProhibited()`.
- `drone_city_nav/src/path_smoothing.cpp:154` схлопывает почти коллинеарные
  точки. Corner rounding должен идти после этого слоя, чтобы схлопывание не
  выпрямило дуги обратно.
- `drone_city_nav/src/px4_offboard_node.cpp:384` принимает `nav_msgs::Path` и
  сохраняет `path_points_`.
- `drone_city_nav/src/px4_offboard_node.cpp:890` публикует PX4
  `TrajectorySetpoint` в velocity mode: `position={NaN,NaN,NaN}`,
  `velocity={plan.velocity_xy.x, plan.velocity_xy.y, vz_ned}`.
- `drone_city_nav/src/px4_offboard_node.cpp:1183` уже умеет восстановить
  `OccupancyGrid2D` из свежей `/drone_city_nav/prohibited_grid` для локальных
  проверок.
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:26` и
  `drone_city_nav/src/offboard_velocity_follower.cpp:394` описывают текущий
  velocity follower API.
- `drone_city_nav/src/offboard_velocity_follower.cpp:188` и
  `drone_city_nav/src/offboard_velocity_follower.cpp:273` считают радиус и
  торможение от сырого угла ломаной через `turn_radius_base_m / sin(angle/2)`.
  Это временный слой, который должен уйти из активного управления после
  появления line/arc trajectory.
- `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp:32` даёт
  `worldToCell()`, `cellsOnLine()` и `isProhibited()`. Для проверки дуг нужно
  использовать эти API, а не новый параллельный формат карты.

## Detected Stack/Profiles

Применённые профили:

- `generic.md`: обязателен для любого workspace; по нему найдены
  repo-approved команды и выбрана scoped verification strategy.
- `cpp.md`: применён, потому что workspace содержит `CMakeLists.txt`,
  `Makefile`, `compile_commands.json`, `.cpp` и `.hpp`.

Rust profile не применялся: `Cargo.toml`/Rust workspace в проекте не найден.

Стек проекта:

- ROS 2 Jazzy workspace.
- Основной пакет: `drone_city_nav`, ament CMake package.
- Сборка через `colcon` из Makefile/wrapper scripts.
- C++20, gtest, clang-format, clang-tidy/cppcheck через project scripts.
- Симуляция: Gazebo + PX4 SITL + ROS 2 nodes.

Протоколы Notion/GitLab прочитаны. Prompt не содержит Notion task или GitLab MR,
поэтому удалённые/CLI чтения Notion/GitLab не запускались.

## Repo-Approved Commands Found

Команды из `README.md`, `CONTRIBUTING.md` и `Makefile`:

```bash
./scripts/build.sh
./scripts/test.sh
./scripts/sim_gui.sh
./scripts/sim_headless.sh
./scripts/stop_sim.sh
```

Для интерактивного контейнера:

```bash
./scripts/dev_shell.sh
make build
make test
make test-scripts
make quality
make format
make sim-gui
make sim-headless
ctest --test-dir build/drone_city_nav --output-on-failure
```

Правила выбора:

- Использовать только контейнерный workflow.
- Не запускать прямой top-level CMake.
- Не пересоздавать build dir без необходимости.
- Для C++ изменений перед коммитом: `make format`, затем `make quality`.
- Симуляции запускать только по явной команде пользователя.

## Affected Components

- `drone_city_nav/include/drone_city_nav/trajectory.hpp` и
  `drone_city_nav/src/trajectory.cpp`: новый core-модуль typed trajectory,
  projection, sampling, curvature.
- `drone_city_nav/include/drone_city_nav/corner_rounding.hpp` и
  `drone_city_nav/src/corner_rounding.cpp`: новый core-модуль скругления углов
  после collapse.
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp` и
  `drone_city_nav/src/offboard_velocity_follower.cpp`: переход от
  `std::span<const Point2>` к trajectory/speed profile в active velocity
  follower.
- `drone_city_nav/src/px4_offboard_node.cpp`: построение trajectory из
  принятого path, rebuild при новом path/prohibited grid, публикация velocity
  setpoint по trajectory, логи/blackbox/debug path.
- `drone_city_nav/src/ros_conversions.cpp` и
  `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`: опционально,
  если удобнее вынести sampled trajectory debug path в общий conversion helper.
- `drone_city_nav/config/urban_mvp.yaml`: новые параметры corner rounding и
  trajectory speed profile.
- `drone_city_nav/CMakeLists.txt`: подключить новые core sources и новые gtest.
- Тесты:
  - `drone_city_nav/tests/trajectory_test.cpp`
  - `drone_city_nav/tests/corner_rounding_test.cpp`
  - обновить `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
  - при изменении JSONL/логов обновить script-level contract tests в
    `scripts/tests/`.

## Implementation Steps

1. Добавить core-модель typed trajectory.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory.hpp`
   - `drone_city_nav/src/trajectory.cpp`
   - `drone_city_nav/tests/trajectory_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат: в core-библиотеке появляется типизированная
   траектория, которую можно проецировать, семплировать и использовать для
   velocity follower без ROS.

   Предлагаемый API:

   ```cpp
   enum class TrajectorySegmentKind { kLine, kArc };

   struct TrajectorySegment {
     TrajectorySegmentKind kind;
     Point2 start;
     Point2 end;
     Point2 center;
     double radius_m;
     double start_angle_rad;
     double sweep_rad;
     double s_start_m;
     double length_m;
   };

   struct TrajectoryProjection {
     bool valid;
     std::size_t segment_index;
     double segment_t;
     double s_m;
     Point2 point;
     Point2 tangent;
     double curvature_1pm;
     double distance_sq;
   };
   ```

   Обязательные функции:

   - `trajectoryLengthM(...)`
   - `projectOnTrajectory(...)`
   - `sampleTrajectory(...)`
   - `trajectoryPointAtS(...)`
   - `trajectoryTangentAtS(...)`

   Автотесты:

   - projection на line segment;
   - projection на arc segment;
   - корректная tangent direction на дуге clockwise/counterclockwise;
   - curvature `0` для line и `±1/R` для arc;
   - total length равен сумме line/arc lengths;
   - degenerate/non-finite inputs дают invalid result без exception.

2. Добавить corner rounding после collapse.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/corner_rounding.hpp`
   - `drone_city_nav/src/corner_rounding.cpp`
   - `drone_city_nav/tests/corner_rounding_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат: `std::span<const Point2>` после
   `collapseCollinearPath()` превращается в `RoundedTrajectory`, где углы по
   возможности заменены дугами, а небезопасные или слишком короткие углы
   оставлены как sharp joins.

   Предлагаемый API:

   ```cpp
   struct CornerRoundingConfig {
     bool enabled{true};
     double min_radius_m{3.0};
     double max_radius_m{30.0};
     double min_segment_remainder_m{1.0};
     double collision_sample_step_m{0.25};
   };

   struct CornerRoundingStats {
     std::size_t input_points;
     std::size_t output_segments;
     std::size_t corners_seen;
     std::size_t corners_rounded;
     std::size_t skipped_short_segments;
     std::size_t skipped_collision;
     std::size_t skipped_degenerate;
     double min_radius_m;
     double max_radius_m;
     double mean_radius_m;
   };

   struct CornerRoundingResult {
     std::vector<TrajectorySegment> segments;
     CornerRoundingStats stats;
   };
   ```

   Алгоритм для каждой тройки `prev -> corner -> next`:

   ```cpp
   theta = deflectionAngle(prev, corner, next);
   max_r_by_lengths =
       min(len_in, len_out) / tan(theta / 2) - min_segment_remainder_m;
   r = min(config.max_radius_m, max_r_by_lengths);

   while (r >= config.min_radius_m) {
     candidate_arc = buildTangentArc(prev, corner, next, r);
     if (arcFitsSegments(candidate_arc) &&
         arcDoesNotOverlapPrevious(candidate_arc) &&
         arcIsTraversable(candidate_arc, prohibited_grid)) {
       accept;
       break;
     }
     shrink r;
   }

   if (!accepted) {
     keep sharp line join;
   }
   ```

   Проверка prohibited-зон:

   - семплировать дугу шагом `<= min(config.collision_sample_step_m,
     0.5 * grid.resolution())`;
   - каждую пару соседних samples дополнительно проводить через
     `OccupancyGrid2D::cellsOnLine()` и `isProhibited()`;
   - outside-grid считать reject reason, а не silent success.

   Автотесты:

   - 90-градусный угол в пустой grid создаёт `line -> arc -> line`;
   - радиус уменьшается, если максимальный радиус не помещается на коротких
     сегментах;
   - дуга отклоняется или shrink-ится, если sample/cellsOnLine пересекает
     prohibited cell;
   - соседние дуги не перекрываются;
   - почти прямая линия остаётся line-only;
   - если rounding disabled, результат остаётся line-only trajectory по
     исходной ломаной.

3. Интегрировать построение rounded trajectory в `px4_offboard_node`.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp:384`
   - `drone_city_nav/src/px4_offboard_node.cpp:667`
   - `drone_city_nav/src/px4_offboard_node.cpp:1183`
   - `drone_city_nav/config/urban_mvp.yaml`

   Материализуемый результат: offboard node хранит не только `path_points_`, но
   и актуальную typed trajectory для active path.

   Предлагаемая схема:

   - в `onPath()` после сохранения `path_points_` вызвать
     `rebuildRoundedTrajectory("path_update")`;
   - при обновлении свежей `prohibited_grid_` перестроить trajectory для
     текущего path или пометить её stale и перестроить перед следующим
     setpoint;
   - если fresh prohibited grid недоступна, строить line-only trajectory без
     дуг или строить дуги без collision check только при явном config flag
     `corner_rounding_allow_without_grid=false` по умолчанию;
   - если rounding не смог построить ни одного segment, fallback должен быть
     line-only trajectory из path, а не empty path/hold.

   Новые параметры:

   ```yaml
   corner_rounding_enabled: true
   corner_rounding_min_radius_m: 3.0
   corner_rounding_max_radius_m: 30.0
   corner_rounding_min_segment_remainder_m: 1.0
   corner_rounding_collision_sample_step_m: 0.25
   rounded_trajectory_debug_topic: /drone_city_nav/rounded_trajectory_path
   rounded_trajectory_debug_sample_step_m: 1.0
   ```

   Логи при каждом path update:

   ```text
   Rounded trajectory rebuilt: path_id=... source=path_update
   path_points=... line_segments=... arc_segments=...
   total_length=... rounded_corners=... skipped_collision=...
   radius[min=... mean=... max=...]
   ```

4. Публиковать debug view округлённой trajectory без превращения её в control
   waypoint-ы.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp` и
     `drone_city_nav/src/ros_conversions.cpp`, если helper будет общим
   - `drone_city_nav/config/urban_mvp.yaml`

   Материализуемый результат: в RViz/headless logs можно видеть sampled rounded
   trajectory, но control layer продолжает работать с typed line/arc segments.

   Реализация:

   - publish `nav_msgs::msg::Path` на ground z в отдельный topic
     `/drone_city_nav/rounded_trajectory_path`;
   - points в этом topic являются только visualization samples;
   - логировать `debug_samples=N`, чтобы не спутать их с control waypoint count.

   Автотесты:

   - pure helper `sampleTrajectory()` покрыт в `trajectory_test.cpp`;
   - если добавляется conversion helper, покрыть его в
     `drone_city_nav/tests/ros_conversions_test.cpp`.

5. Заменить active velocity planning с raw-angle braking на speed profile по
   trajectory.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:26`
   - `drone_city_nav/src/offboard_velocity_follower.cpp:394`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Материализуемый результат: `planVelocitySetpoint()` использует projection на
   typed trajectory, tangent/curvature текущего segment и заранее посчитанный
   speed profile. Сырой угол A* больше не участвует в active speed limit.

   Предлагаемый контракт:

   ```cpp
   struct TrajectorySpeedSample {
     double s_m;
     double geometric_limit_mps;
     double profiled_limit_mps;
     SpeedConstraintType reason;
     std::size_t segment_index;
   };

   struct TrajectorySpeedProfile {
     std::vector<TrajectorySpeedSample> samples;
   };

   VelocitySetpointPlan planVelocitySetpoint(
       const Trajectory& trajectory,
       const TrajectorySpeedProfile& speed_profile,
       Point2 current_position,
       Point2 current_velocity,
       bool current_velocity_valid,
       double dt_s,
       const VelocityFollowerState& previous_state,
       const VelocityFollowerConfig& config);
   ```

   Profile build:

   ```cpp
   for sample in trajectory_samples:
     if sample.curvature == 0:
       v_limit = cruise_speed_mps;
     else:
       v_limit = sqrt(max_lateral_accel_mps2 / abs(sample.curvature));

   final sample speed = 0;

   backward pass:
     v[i] = min(v[i], sqrt(v[i+1]^2 + 2 * max_decel * ds));

   forward pass:
     v[i] = min(v[i], sqrt(v[i-1]^2 + 2 * max_accel * ds));
   ```

   Per tick:

   - project current position to trajectory;
   - read `profiled_limit_mps` at projection `s`;
   - apply existing acceleration/vector delta limiting;
   - command velocity along trajectory tangent;
   - add bounded cross-track correction as сейчас.

   Старые функции `turnKinematics()` и `speedLimitForUpcomingTurn()` после
   перехода нужно удалить из active path или переименовать в raw diagnostic,
   если diagnostics still needed. Их тесты должны быть заменены тестами
   curvature/profile behavior.

   Автотесты:

   - широкая 90-градусная дуга даёт speed limit выше, чем узкая дуга;
   - line-only trajectory держит cruise speed до final stop profile;
   - profile начинает тормозить до начала arc, а не в самой corner point;
   - final stop profile работает одинаково, если goal расположен после прямой
     или после дуги;
   - vector delta limiting продолжает ограничивать резкую смену velocity;
   - invalid/empty trajectory возвращает invalid/hold без публикации мусора.

6. Обновить offboard diagnostics и blackbox под trajectory.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp`
   - возможные script-level tests в `scripts/tests/`

   Материализуемый результат: headless-run logs позволяют понять, где находится
   дрон на trajectory, какой участок ограничивает скорость и почему.

   Добавить в 2 Hz summary/log и JSONL:

   - `trajectory_valid`
   - `trajectory_total_length_m`
   - `trajectory_line_segments`
   - `trajectory_arc_segments`
   - `trajectory_s_m`
   - `trajectory_segment_index`
   - `trajectory_segment_type`
   - `trajectory_curvature_1pm`
   - `trajectory_arc_radius_m`
   - `speed_profile_limit_mps`
   - `speed_profile_reason`
   - `speed_profile_distance_to_constraint_m`
   - `rounded_corners`
   - `rounding_skipped_collision`
   - `rounding_skipped_short_segments`

   Старые поля `limiting_turn_angle_rad` и `limiting_turn_radius_m` не должны
   означать "сырой угол A*". Если они остаются, их нужно переименовать или
   явно писать как `raw_upcoming_turn_*` только для диагностики.

7. Обновить waypoint/progress логику так, чтобы она не конфликтовала с
   trajectory follower.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp:884`
   - `drone_city_nav/src/px4_offboard_node.cpp:890`
   - `drone_city_nav/src/offboard_path_follower.cpp`

   Материализуемый результат: legacy waypoint index может оставаться для
   path continuity/логов, но velocity follower не должен выбирать tangent по
   `path_points_[waypoint_index_]`. Источник движения - projection на trajectory.

   Правило:

   - `path_points_` остаётся входным route contract от planner;
   - `trajectory_` становится control contract для velocity setpoint;
   - `waypoint_index_` не должен ограничивать projection на trajectory так, что
     дрон не может восстановиться после небольшого overshoot;
   - final goal hold должен проверять прогресс по trajectory end и физическое
     расстояние до final goal.

   Автотесты:

   - overshoot около конца дуги не сбрасывает follower в начало path;
   - final goal reached не вызывает бесконечные качели вокруг цели;
   - при новом path trajectory rebuild не создаёт резкий backward projection.

8. Обновить CMake и тестовую матрицу.

   Файлы:

   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат:

   - `src/trajectory.cpp` и `src/corner_rounding.cpp` включены в
     `drone_city_nav_core`;
   - добавлены `trajectory_test` и `corner_rounding_test`;
   - обновлён `offboard_velocity_follower_test`;
   - при необходимости обновлены script tests для новых JSONL fields.

9. Удалить или переименовать устаревшие параметры raw-angle braking.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:26`
   - `drone_city_nav/src/offboard_velocity_follower.cpp:188`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Материализуемый результат: config и code больше не создают впечатление, что
   скорость поворота считается от сырого угла.

   Кандидаты на удаление или переименование:

   - `turn_radius_base_m`
   - raw-angle `turn_preview_distance_m`, если он больше не нужен speed profile;
   - `TurnSpeedPlan` fields, если их полностью заменяет
     `TrajectorySpeedProfile`.

   Если `turn_preview_distance_m` нужен для diagnostics, назвать его так, чтобы
   было видно, что он не управляет скоростью.

## Verification Plan

Для реализации, затрагивающей C++ код:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
```

Scoped проверки после добавления новых тестов:

```bash
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R 'trajectory|corner_rounding|offboard_velocity_follower|offboard_path_follower'
```

Полная локальная проверка без GUI:

```bash
./scripts/test.sh
```

Headless simulation validation запускать только по явной команде пользователя:

```bash
./scripts/sim_headless.sh
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

Ожидаемые сигналы в headless logs после реализации:

- `trajectory_arc_segments > 0` на маршруте с поворотами;
- `rounded_corners > 0`, если есть достаточно места и дуга не пересекает
  prohibited grid;
- `speed_profile_reason=arc` до и внутри дуги;
- на прямой скорость стремится к `cruise_speed_mps`;
- перед дугой скорость начинает снижаться заранее по backward profile;
- final goal не вызывает перелёты туда-сюда;
- path update/replan не создаёт пустую trajectory при валидном path.

## Testing Strategy

### Категория 1: без рефакторинга

Pure unit tests без ROS и без симуляции:

- `trajectory_test.cpp`: line/arc projection, tangent, curvature, length,
  sampling, invalid inputs.
- `corner_rounding_test.cpp`: геометрия tangent points, radius shrink,
  prohibited collision reject, adjacent arc overlap prevention.
- `offboard_velocity_follower_test.cpp`: speed profile по line/arc trajectory,
  final stop profile, acceleration/vector delta limiting.

Эти тесты должны запускаться быстро и быть основным доказательством
математической корректности.

### Категория 2: лёгкий integration scope

ROS-node-adjacent проверки без полноценной симуляции:

- тесты helpers для conversion/debug sampled trajectory path;
- contract tests для JSONL/blackbox fields, если меняется telemetry schema;
- `ctest --test-dir build/drone_city_nav --output-on-failure -R ...`;
- `make quality`, потому что изменение затрагивает public headers, CMake и
  shared core library.

### Категория 3: тяжёлый runtime scope

Только по явной команде пользователя:

- `./scripts/sim_headless.sh` with-static;
- `./scripts/sim_headless.sh` no-static через documented env/config;
- GUI sanity check через `./scripts/sim_gui.sh`, если нужно проверить RViz
  debug trajectory.

В runtime оценивать:

- миссия достигнута;
- столкновений нет;
- число replans не выросло без причины;
- velocity profile не заставляет дрон ползти заранее;
- дуги видны в debug topic и не используются как control waypoint flood.

## Risks And Tradeoffs

- Слишком крупный sampling дуги может пропустить prohibited cell. Нужно
  семплировать шагом не больше половины resolution и проверять `cellsOnLine()`
  между соседними samples.
- Слишком большой радиус дуги может срезать угол через prohibited zone. Поэтому
  radius должен shrink-иться или corner должен оставаться sharp.
- Соседние дуги могут перекрыться на коротком участке. Нужно резервировать
  tangent distances на adjacent segments.
- Если trajectory перестраивать на каждое обновление prohibited grid, возможна
  лишняя нагрузка и jitter diagnostics. Лучше перестраивать при path update и
  при meaningful grid update/stale trajectory flag.
- `nav_msgs::Path` не умеет семантически хранить дуги. Поэтому sampled debug
  path нельзя использовать как источник управления.
- Удаление raw-angle braking изменит существующие логи и тесты. Нужно явно
  переименовать поля, чтобы не было ложной интерпретации старых метрик.
- В no-static режиме карта может меняться во время полёта, и ранее построенная
  дуга может стать запрещённой. Нужен rebuild/fallback to line-only/sharp turn,
  а не silent reuse небезопасной дуги.
- Большой `max_lateral_accel_mps2` может улучшить скорость, но увеличить tilt и
  риск раскачки. Значение должно быть видно в логах и проверяться runtime
  телеметрией.
- Debug sampled trajectory path может визуально выглядеть как "много точек".
  Нужно логировать, что это visualization samples, а не control waypoints.

## Open Questions

- Где именно должен жить owner rounded trajectory в финальной архитектуре:
  в `px4_offboard_node` как control-layer transform или в planner как часть
  published route? Для первого этапа лучше `px4_offboard_node`, потому что это
  слой управления, а planner должен оставаться A*/grid route planner.
- Какие дефолтные радиусы выбрать для текущего масштаба города: стартово
  предлагается `min_radius_m=3`, `max_radius_m=30`, но после реализации нужно
  подтвердить их headless логами.
- Нужен ли acceleration feedforward в PX4 setpoint после появления дуг?
  Текущий план ограничивается velocity setpoint по касательной и bounded
  cross-track correction. Feedforward acceleration можно добавить отдельным
  слоем после стабилизации line/arc velocity profile.
- Какой цвет/стиль использовать для rounded trajectory в RViz, чтобы не
  спутать её с planner path и prohibited grid? Технически достаточно отдельного
  topic, но визуальный стиль лучше согласовать перед GUI polishing.
