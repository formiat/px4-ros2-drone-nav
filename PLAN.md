# План: trajectory planner поверх A*

## Context

Цель: спланировать финальный слой trajectory planner поверх существующего A*,
чтобы дрон летел не по сырой ломаной, а по единой финальной траектории,
рассчитанной внутри свободного пространства и пригодной для быстрого плавного
полёта.

Финальная цепочка:

```text
occupancy/prohibited grid
  -> A* rough route
  -> route simplification
  -> corridor / tunnel
  -> racing line / apex trajectory
  -> curvature-based speed profile
  -> velocity setpoint follower
  -> RViz final trajectory
```

Разделение ответственности:

- A* остаётся топологическим планировщиком: найти проходимый маршрут по
  `prohibited` grid.
- Trajectory layer отвечает за геометрию полёта: corridor, apex/racing line,
  curvature, speed profile и debug metrics.
- Offboard follower исполняет именно final trajectory через velocity setpoint.
- RViz по умолчанию показывает ту же final trajectory, которую исполняет
  follower. Сырой A* route допустим только как отдельный debug topic.

Текущее состояние уже содержит baseline:

- typed trajectory `line/arc`;
- corner rounding;
- curvature-based speed profile;
- velocity setpoint follower;
- debug topic `/drone_city_nav/rounded_trajectory_path`.

Новая работа должна не добавлять параллельный режим, а превратить текущий
baseline в один финальный режим: racing trajectory. Отдельный режим
`corner rounding без apex` оставлять не нужно.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует, поэтому входного исследовательского
артефакта нет. План основан на чтении текущего локального кода.

Ключевые code anchors:

- `drone_city_nav/src/planner_node.cpp:733` публикует маршрут из A* cells после
  `smoothPathWithStats()` и `collapseCollinearPath()`.
- `drone_city_nav/src/planner_node.cpp:1068` публикует финальную planning map
  `/drone_city_nav/prohibited_grid`; это источник для corridor/collision checks.
- `drone_city_nav/include/drone_city_nav/trajectory.hpp:13` уже содержит
  `TrajectorySegmentKind::kLine/kArc`.
- `drone_city_nav/include/drone_city_nav/trajectory.hpp:18` уже содержит
  `TrajectorySegment`.
- `drone_city_nav/include/drone_city_nav/trajectory.hpp:30` уже содержит
  `TrajectoryProjection`.
- `drone_city_nav/include/drone_city_nav/corner_rounding.hpp:13` уже содержит
  `CornerRoundingConfig`.
- `drone_city_nav/include/drone_city_nav/corner_rounding.hpp:40` уже содержит
  `roundCorners(...)`.
- `drone_city_nav/include/drone_city_nav/clearance_field.hpp:15` уже содержит
  `ClearanceField2D`.
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:64`
  уже содержит `TrajectorySpeedSample`.
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:76`
  уже содержит `TrajectorySpeedProfile`.
- `drone_city_nav/src/offboard_velocity_follower.cpp:258` уже строит speed
  profile по trajectory samples.
- `drone_city_nav/src/offboard_velocity_follower.cpp:409` уже планирует
  velocity setpoint по typed trajectory.
- `drone_city_nav/src/px4_offboard_node.cpp:420` сейчас строит rounded
  trajectory из принятого path.
- `drone_city_nav/src/px4_offboard_node.cpp:946` публикует PX4
  `TrajectorySetpoint` в velocity mode.
- `drone_city_nav/src/px4_offboard_node.cpp:398` публикует debug path
  округлённой trajectory.
- `drone_city_nav/config/urban_mvp.yaml:122` включает velocity cruise mode.
- `drone_city_nav/config/urban_mvp.yaml:133` включает corner rounding.

Главный разрыв между текущим кодом и целевым состоянием:

- текущая final trajectory строится как `A* simplified path -> local corner
  rounding`;
- corridor/tunnel и racing-line/apex optimizer ещё не выделены как core-слои;
- RViz основная синяя линия всё ещё может быть planner path, а не final
  executable trajectory;
- speed profile уже curvature-based, но должен стать профилем поверх финальной
  racing trajectory, а не поверх временной rounded trajectory;
- diagnostics ещё содержит legacy поля `turn_*`, которые могут путать анализ.

## Detected stack/profiles

Применённые профили:

- `generic.md`: обязателен для любого workspace; по нему найдены
  repo-approved команды и выбран порядок проверок.
- `cpp.md`: применён, потому что workspace содержит `CMakeLists.txt`,
  `Makefile`, `compile_commands.json`, `.cpp` и `.hpp`.

Rust profile не применялся: `Cargo.toml`, `Cargo.lock` и `.rs` файлы в
workspace не найдены.

Стек проекта:

- ROS 2 Jazzy workspace.
- Основной пакет: `drone_city_nav`, ament CMake package.
- Сборка через `colcon` из Makefile/wrapper scripts.
- C++20, gtest, clang-format, clang-tidy/cppcheck через project scripts.
- Симуляция: Gazebo + PX4 SITL + ROS 2 nodes.

Протоколы Notion/GitLab прочитаны. Prompt не содержит Notion task или GitLab MR,
поэтому удалённые чтения Notion/GitLab не выполнялись.

## Repo-approved commands found

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

Правила для этой задачи:

- Использовать только container workflow.
- Не запускать прямой top-level CMake.
- Не пересоздавать build dir без необходимости.
- Перед коммитом после C++ изменений: `make format`, затем `make quality`.
- Симуляционные прогоны запускать только по явной команде пользователя.

## Affected components

- `drone_city_nav/include/drone_city_nav/trajectory.hpp`
- `drone_city_nav/src/trajectory.cpp`
- `drone_city_nav/include/drone_city_nav/clearance_field.hpp`
- `drone_city_nav/src/clearance_field.cpp`
- `drone_city_nav/include/drone_city_nav/corridor.hpp` (новый)
- `drone_city_nav/src/corridor.cpp` (новый)
- `drone_city_nav/include/drone_city_nav/racing_line.hpp` (новый)
- `drone_city_nav/src/racing_line.cpp` (новый)
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp` (новый)
- `drone_city_nav/src/trajectory_planner.cpp` (новый)
- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
- `drone_city_nav/src/offboard_velocity_follower.cpp`
- `drone_city_nav/src/px4_offboard_node.cpp`
- `drone_city_nav/src/planner_node.cpp`
- `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`
- `drone_city_nav/src/ros_conversions.cpp`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/rviz/`
- `drone_city_nav/CMakeLists.txt`
- Тесты в `drone_city_nav/tests/`
- Скриптовые проверки JSONL/debug output в `scripts/tests/`, если меняется
  контракт blackbox/logs.

## Implementation steps

1. Зафиксировать текущий route/trajectory контракт.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory.hpp:13`
   - `drone_city_nav/src/trajectory.cpp`
   - `drone_city_nav/tests/trajectory_test.cpp`

   Результат: `TrajectorySegment` остаётся базовым carrier для final
   trajectory, но API явно поддерживает не только `line/arc`, а будущие
   sampled racing curves.

   Изменение:

   - добавить `TrajectoryPointSample` с `s_m`, `point`, `tangent`,
     `curvature_1pm`, `left_bound_m`, `right_bound_m`;
   - добавить helper для построения sampled trajectory из произвольных samples;
   - оставить `line/arc` как текущую компактную форму, но публичные follower
     API должны уметь принимать final sampled trajectory.

   Псевдокод:

   ```cpp
   struct TrajectoryPointSample {
     double s_m;
     Point2 point;
     Point2 tangent;
     double curvature_1pm;
     double left_bound_m;
     double right_bound_m;
   };
   ```

   Автотесты:

   - stationing samples монотонен;
   - non-finite sample отклоняется;
   - projection на sampled trajectory выбирает ближайший segment;
   - curvature сохраняется при resampling.

2. Вынести построение executable trajectory из `px4_offboard_node` в core.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp` (новый)
   - `drone_city_nav/src/trajectory_planner.cpp` (новый)
   - `drone_city_nav/src/px4_offboard_node.cpp:420`
   - `drone_city_nav/CMakeLists.txt`

   Результат: ROS node больше не владеет алгоритмом построения trajectory.
   `px4_offboard_node` только получает path/prohibited grid, вызывает core
   planner и публикует результат.

   Предлагаемый контракт:

   ```cpp
   struct TrajectoryPlannerInput {
     std::span<const Point2> route_points;
     const OccupancyGrid2D* prohibited_grid;
   };

   struct TrajectoryPlannerResult {
     std::vector<TrajectorySegment> compact_segments;
     std::vector<TrajectoryPointSample> samples;
     TrajectorySpeedProfile speed_profile;
     TrajectoryPlannerStats stats;
   };
   ```

   Автотесты:

   - empty/one-point route даёт invalid result;
   - при выключенном corridor/racing optimizer возвращается текущий rounded
     baseline;
   - отсутствие prohibited grid не приводит к silent unsafe apex: result либо
     line-only baseline, либо invalid согласно config.

3. Добавить corridor/tunnel builder на базе `ClearanceField2D`.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/corridor.hpp` (новый)
   - `drone_city_nav/src/corridor.cpp` (новый)
   - `drone_city_nav/include/drone_city_nav/clearance_field.hpp:15`
   - `drone_city_nav/tests/corridor_test.cpp` (новый)
   - `drone_city_nav/CMakeLists.txt`

   Результат: вдоль simplified route строится допустимый corridor, где для
   каждой station/sample известна левая и правая граница смещения до
   `prohibited`-зоны.

   Алгоритм:

   - построить `ClearanceField2D::build(grid, max_corridor_radius_m,
     ClearanceSource::kProhibited)`;
   - resample route с шагом `corridor_sample_step_m`;
   - для каждой station вычислить tangent и normal;
   - трассировать влево/вправо по normal до `prohibited`, outside-grid или
     `max_corridor_radius_m`;
   - сохранить границы с safety margin, но не менять A* grid.

   Псевдокод:

   ```cpp
   for (RouteSample sample : route_samples) {
     Point2 normal = leftNormal(sample.tangent);
     left = raycastUntilProhibited(grid, sample.point, normal);
     right = raycastUntilProhibited(grid, sample.point, -normal);
     corridor.push_back({sample.s_m, sample.point, sample.tangent, left, right});
   }
   ```

   Автотесты:

   - прямой проход между двумя стенами даёт симметричные bounds;
   - возле стены одна сторона уже другой;
   - outside-grid ограничивает corridor;
   - prohibited cell на route даёт invalid/narrow corridor;
   - corridor не зависит от источника препятствия: static/lidar/memory уже
     объединены в `prohibited_grid`.

4. Добавить racing-line/apex optimizer по lateral offsets внутри corridor.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp` (новый)
   - `drone_city_nav/src/racing_line.cpp` (новый)
   - `drone_city_nav/tests/racing_line_test.cpp` (новый)
   - `drone_city_nav/CMakeLists.txt`

   Результат: вместо ручного "апекса" строится новая кривая внутри corridor.
   Она минимизирует длину, кривизну и изменение кривизны, не выходя за bounds.

   MVP-реализация без тяжёлой внешней зависимости:

   - переменная оптимизации: lateral offset `d_i` для каждого corridor sample;
   - начальное значение: `d_i = 0`;
   - ограничение: `-right_bound_i <= d_i <= left_bound_i`;
   - cost:

   ```text
   cost =
     w_length * path_length
     + w_curvature * sum(curvature_i^2)
     + w_curvature_change * sum((curvature_i - curvature_{i-1})^2)
     + w_center_bias * optional_center_bias
   ```

   - решатель: deterministic iterative coordinate descent / gradient-free
     local search, чтобы не вводить внешнюю зависимость;
   - каждый кандидат проверять через segment collision sampling против
     `prohibited_grid`.

   Псевдокод:

   ```cpp
   for (iteration = 0; iteration < max_iterations; ++iteration) {
     for (i = 1; i + 1 < samples.size(); ++i) {
       for (delta : {-step, 0, +step}) {
         candidate[i].offset = clamp(offset[i] + delta, -right[i], left[i]);
         if (candidateIsInsideCorridor(candidate) &&
             candidateSegmentsAreTraversable(candidate, prohibited_grid)) {
           keep lowest cost;
         }
       }
     }
     step *= cooling;
   }
   ```

   Автотесты:

   - в широком 90-градусном повороте optimizer увеличивает effective radius по
     сравнению с centerline;
   - в узком проходе stays inside corridor и не выходит в prohibited;
   - при заблокированном apex fallback остаётся безопасным baseline;
   - результат детерминирован для одинакового входа;
   - optimizer не делает route длиннее сверх заданного guard без снижения
     curvature.

5. Заменить corner rounding как публичный режим на internal fallback.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/corner_rounding.hpp:13`
   - `drone_city_nav/src/corner_rounding.cpp`
   - `drone_city_nav/src/trajectory_planner.cpp`
   - `drone_city_nav/config/urban_mvp.yaml:133`
   - `drone_city_nav/tests/corner_rounding_test.cpp`

   Результат: в конфиге и логах нет отдельного режима "corner rounding без
   apex". `roundCorners()` может остаться как private/internal fallback внутри
   `TrajectoryPlanner`, но основная фича называется `racing_trajectory`.

   Изменение:

   - параметры `corner_rounding_*` либо переименовать в
     `trajectory_baseline_rounding_*`, либо скрыть внутри defaults;
   - добавить параметры `racing_trajectory_enabled`,
     `corridor_max_radius_m`, `racing_line_*`;
   - old rounded debug topic заменить на final trajectory topic.

   Автотесты:

   - при `racing_trajectory_enabled=true` используется optimizer;
   - при optimizer failure result безопасно падает на baseline trajectory с
     явным `fallback_reason`;
   - старые тесты corner rounding остаются core-level тестами fallback, но не
     тестируют отдельный runtime mode.

6. Переключить RViz/offboard на final trajectory как единственный runtime
   contract.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp:398`
   - `drone_city_nav/src/px4_offboard_node.cpp:946`
   - `drone_city_nav/src/lidar_debug_node.cpp`
   - `drone_city_nav/rviz/*.rviz`
   - `drone_city_nav/config/urban_mvp.yaml`

   Результат: основная синяя линия в RViz соответствует trajectory, которую
   исполняет offboard follower.

   Изменение:

   - publish `/drone_city_nav/final_trajectory_path` как основной topic;
   - оставить `/drone_city_nav/a_star_route_debug_path` или существующий planner
     path как debug topic с другим цветом/именем;
   - `lidar_debug_node` должен подписываться на final trajectory для основной
     визуализации;
   - `px4_offboard_node` должен исполнять именно `TrajectoryPlannerResult`, а не
     напрямую `path_points_`.

   Автотесты:

   - conversion helper строит `nav_msgs::Path` с `z=0`;
   - empty final trajectory публикует empty debug path;
   - path id/final trajectory id не расходятся в blackbox.

7. Обобщить speed profile на final racing trajectory.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:64`
   - `drone_city_nav/src/offboard_velocity_follower.cpp:258`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
   - `drone_city_nav/src/trajectory_planner.cpp`

   Результат: скорость считается по curvature всей final trajectory, а не по
   сырому A* углу и не только "перед поворотом".

   Формула:

   ```text
   v_limit(s) = sqrt(max_lateral_accel_mps2 / abs(curvature(s)))
   ```

   Для прямых:

   ```text
   abs(curvature) < epsilon -> v_limit = cruise_speed_mps
   ```

   Профиль:

   ```cpp
   geometric_limit[i] = curvatureLimit(sample[i]);
   speed.back() = 0.0; // final stop

   for (i = n - 2; i >= 0; --i) {
     speed[i] = min(geometric_limit[i],
                    sqrt(speed[i+1]^2 + 2 * max_decel * ds));
   }

   for (i = 1; i < n; ++i) {
     speed[i] = min(speed[i],
                    sqrt(speed[i-1]^2 + 2 * max_accel * ds));
   }
   ```

   Автотесты:

   - narrow curvature даёт меньший speed limit, чем wide curvature;
   - профиль тормозит до high-curvature region;
   - final stop работает одинаково после line и после curve;
   - zero/near-zero curvature не создаёт NaN/Inf;
   - velocity vector limiter не ломает speed profile reason.

8. Удалить legacy `turn_*` diagnostics из active semantics.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:46`
   - `drone_city_nav/src/offboard_velocity_follower.cpp:517`
   - `drone_city_nav/src/px4_offboard_node.cpp:1469`
   - `drone_city_nav/src/px4_offboard_node.cpp:1729`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Результат: diagnostics говорят на языке trajectory/profile, а не сырого
   turn angle.

   Изменение:

   - удалить или переименовать `TurnSpeedPlan`;
   - заменить `limiting_turn_*` на `limiting_curve_*`;
   - `VelocitySetpointReason::kBrakingForTurn` переименовать в
     `kTrajectorySpeedProfile`;
   - JSONL поля `turn_target_speed_mps`, `distance_to_turn_m`,
     `turn_angle_rad` либо удалить, либо оставить только как
     `raw_route_debug_*`.

   Автотесты:

   - JSON writer/script tests парсят новый контракт;
   - старые fields не используются в runtime assertions;
   - no NaN raw-turn fields для normal trajectory profile.

9. Интегрировать trajectory planner lifecycle в `px4_offboard_node`.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp:480`
   - `drone_city_nav/src/px4_offboard_node.cpp:946`
   - `drone_city_nav/src/px4_offboard_node.cpp:1167`

   Результат: при новом path или новой relevant prohibited grid строится final
   trajectory, speed profile и debug output атомарно.

   Правила:

   - новый planner path всегда инвалидирует старую trajectory;
   - prohibited grid update перестраивает trajectory только если текущая
     trajectory пересекает prohibited или corridor bounds изменились materially;
   - при rebuild failure follower не получает мусор: либо last valid trajectory
     с явным `reuse_reason`, либо hold/no-path согласно текущей policy;
   - final trajectory id логируется вместе с planner path id.

   Автотесты:

   - path update rebuilds final trajectory once;
   - unchanged grid does not churn trajectory id;
   - prohibited intersection triggers rebuild;
   - rebuild failure produces explicit invalid result and log reason.

10. Обновить planner-side debug publication.

    Файлы:

    - `drone_city_nav/src/planner_node.cpp:1073`
    - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp:58`
    - `drone_city_nav/src/ros_conversions.cpp`

    Результат: planner path явно называется rough/simplified route, чтобы не
    путать его с final trajectory.

    Изменение:

    - сохранить raw planner path publication для debug;
    - переименовать topic/config в `rough_route_debug_topic`, если совместимо;
    - в логах planner писать `rough_route_points`, `smoothed_points`,
      `collapsed_points`, `published_as_debug_route=true`;
    - основной RViz config переключить на offboard final trajectory.

    Автотесты:

    - `ros_conversions_test` проверяет `pathToRos` на ground z;
    - planner unit tests не требуют final trajectory.

11. Добавить headless diagnostics для corridor/racing-line.

    Файлы:

    - `drone_city_nav/src/px4_offboard_node.cpp:1460`
    - `drone_city_nav/src/px4_offboard_node.cpp:1667`
    - `scripts/tests/`

    Результат: по логам без GUI видно, почему trajectory получилась такой и
    где ограничена скорость.

    Логировать:

    - `corridor_samples`
    - `corridor_width_min_m`, `corridor_width_mean_m`
    - `racing_line_iterations`
    - `racing_line_cost_initial`, `racing_line_cost_final`
    - `racing_line_max_offset_m`
    - `curvature_min/max/mean`
    - `speed_profile_min/max/mean`
    - `speed_profile_limited_by_curvature_count`
    - `trajectory_fallback_reason`
    - `final_trajectory_samples`

    Автотесты:

    - blackbox JSONL remains parseable;
    - missing/invalid numeric values are written as `null`;
    - expected fields exist in one synthetic record.

12. Обновить configuration.

    Файлы:

    - `drone_city_nav/config/urban_mvp.yaml:122`
    - `drone_city_nav/config/urban_mvp.yaml:133`
    - `docs/MVP_SIMULATION.md`
    - `README.md`, если меняются user-facing topics/commands.

    Результат: дефолтный режим один: racing trajectory.

    Предлагаемые параметры:

    ```yaml
    racing_trajectory_enabled: true
    final_trajectory_debug_topic: /drone_city_nav/final_trajectory_path
    rough_route_debug_topic: /drone_city_nav/rough_route_debug_path
    corridor_max_radius_m: 40.0
    corridor_sample_step_m: 1.0
    corridor_safety_margin_m: 0.5
    racing_line_max_iterations: 80
    racing_line_initial_offset_step_m: 2.0
    racing_line_min_offset_step_m: 0.1
    racing_line_weight_length: 1.0
    racing_line_weight_curvature: 25.0
    racing_line_weight_curvature_change: 10.0
    speed_profile_sample_step_m: 0.5
    ```

    Автотесты:

    - config loader clamps invalid values;
    - defaults produce enabled racing trajectory;
    - disabling only used for tests/debug, not as second production mode.

13. Обновить CMake/test matrix.

    Файлы:

    - `drone_city_nav/CMakeLists.txt`

    Результат:

    - новые core sources добавлены в `drone_city_nav_core`;
    - добавлены `corridor_test`, `racing_line_test`,
      `trajectory_planner_test`;
    - обновлены existing tests для changed contracts.

14. Удалить устаревшие runtime параметры и dead wrappers.

    Файлы:

    - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
    - `drone_city_nav/src/offboard_velocity_follower.cpp`
    - `drone_city_nav/src/px4_offboard_node.cpp`
    - `drone_city_nav/config/urban_mvp.yaml`

    Результат: код не содержит двух конкурирующих моделей движения.

    Кандидаты:

    - `TurnSpeedPlan`, если он больше не нужен;
    - compatibility wrapper `planVelocitySetpoint(std::span<const Point2>...)`,
      если все call sites перешли на final trajectory;
    - `velocityCruisePathIsUsable(...)`, если production больше не использует
      path-only follower;
    - `corner_rounding_*` как user-facing параметры, если они поглощены
      `racing_trajectory_*`.

## Verification plan

Обязательные проверки после реализации:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
```

Scoped checks при разработке отдельных этапов:

```bash
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
  -R 'trajectory|corner_rounding|corridor|racing_line|trajectory_planner|offboard_velocity_follower' \
  --output-on-failure
```

Script-level checks при изменении JSONL/debug tooling:

```bash
./scripts/dev_shell.sh make test-scripts
```

Headless simulation не является обязательной проверкой для каждого маленького
коммита и запускается только по явной команде пользователя. Когда пользователь
разрешит прогон, проверять:

```bash
./scripts/sim_headless.sh
python3 scripts/analyze_lidar_projection_snapshots.py \
  log/lidar_debug/snapshots.jsonl \
  --static-map drone_city_nav/worlds/generated_city.map2d
```

Критерии runtime-успеха для будущего прогона:

- mission success без collision;
- final trajectory visible in RViz/debug topic;
- rough route и final trajectory не перепутаны;
- velocity profile ограничивает скорость до high-curvature regions;
- нет бесконечных oscillations около goal;
- blackbox JSONL parseable и содержит trajectory/corridor/racing metrics.

## Testing strategy

Категория 1: без рефакторинга

- Добавлять unit tests рядом с новыми pure helpers.
- Проверять geometry math: projection, curvature, bounds, finite inputs.
- Запускать scoped `ctest -R ...` и `make quality`.

Категория 2: лёгкий рефакторинг

- Вынести reusable helpers из `px4_offboard_node.cpp` в core без изменения ROS
  contracts.
- Покрыть old-vs-new equivalence tests: baseline route даёт такой же line/arc
  result, пока racing optimizer выключен в тестовом config.
- Проверить blackbox JSON contract через script tests.

Категория 3: тяжёлый рефакторинг

- Переключить runtime contract с planner path на final trajectory.
- Добавить corridor/racing optimizer и удалить legacy turn semantics.
- Нужны unit tests, integration-like tests на synthetic grids, `make quality`,
  и отдельный headless simulation run после явного разрешения пользователя.

Happy-path coverage:

- широкий поворот: racing line увеличивает радиус и speed limit;
- узкий проход: trajectory остаётся внутри corridor;
- straight route: cruise speed без лишнего торможения;
- final goal: корректный stop profile.

Negative-path coverage:

- prohibited route/corridor invalid;
- optimizer candidate выходит в prohibited;
- non-finite points/speeds;
- missing prohibited grid.

Edge cases:

- короткие сегменты;
- degenerate duplicate points;
- very small/large curvature;
- trajectory overshoot near end;
- path update while vehicle is moving fast.

## Risks and tradeoffs

- Оптимизатор lateral offsets может быть дорогим. Нужно ограничить sample count,
  iteration count и логировать runtime/cost metrics.
- Слишком агрессивная racing line может подвести близко к prohibited зоне.
  Corridor safety margin и segment collision checks обязательны.
- Если RViz переключить на final trajectory без сохранения rough route debug,
  станет сложнее отлаживать A*. Поэтому rough route нужен отдельным debug
  topic.
- Если оставить legacy `turn_*` поля, можно снова неправильно интерпретировать
  логи. Их нужно переименовать или удалить.
- Временный fallback на corner rounding полезен для robustness, но не должен
  выглядеть как отдельный production mode.
- Чем плотнее samples final trajectory, тем выше CPU cost и размер debug topics.
  Нужно подобрать sample step и добавить лимиты.
- Headless validation зависит от симуляции и по правилу пользователя не
  запускается без явной команды.

## Open questions

- Какой первый целевой уровень сложности optimizer: deterministic coordinate
  descent без зависимостей или подключение внешнего QP/optimization solver?
  Для текущего проекта предпочтителен вариант без новой зависимости.
- Нужно ли полностью удалять старый `/drone_city_nav/plan_path` topic или
  оставить его как backward-compatible rough route debug topic?
- Какие initial weights выбрать для racing-line cost:
  `length/curvature/curvature_change/center_bias`? План предлагает стартовые
  значения, но их надо будет калибровать по headless/GUI прогонам.
- Какой минимальный corridor safety margin принять для racing trajectory при
  текущем `inflation=5m`: 0.5m, 1.0m или завязать на скорость?
- Нужно ли добавлять acceleration feedforward после racing-line trajectory, или
  пока оставить только velocity setpoint + bounded cross-track correction?
