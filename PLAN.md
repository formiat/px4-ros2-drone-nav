# PLAN

## Context

Нужно спланировать явное управление скоростью в `px4_offboard_node`, потому что
сейчас скорость дрона получается косвенно: node публикует position setpoints с
частотой 10 Hz (`drone_city_nav/src/px4_offboard_node.cpp:136`), ограничивает
дальность target через `max_setpoint_distance_m`
(`drone_city_nav/src/px4_offboard_node.cpp:65`) и ограничивает движение
commanded target за тик через `max_commanded_target_step_m`
(`drone_city_nav/src/px4_offboard_node.cpp:67`, `:546`). В текущем
`urban_mvp.yaml` это `max_setpoint_distance_m: 3.0`,
`max_commanded_target_step_m: 0.5`, `lookahead_distance_m: 5.0`
(`drone_city_nav/config/urban_mvp.yaml:85`-`87`), то есть верхний темп движения
commanded target примерно `5 m/s`, но это не является явным контрактом
`desired_speed_mps`.

Текущий offboard режим position-based: `OffboardControlMode.position=true`,
`velocity=false` (`drone_city_nav/src/px4_offboard_node.cpp:426`-`436`), а
`TrajectorySetpoint.velocity` выставляется в нули
(`drone_city_nav/src/px4_offboard_node.cpp:439`-`458`). Это безопаснее для MVP,
но плохо подходит для понятной настройки "лети с такой скоростью, замедляйся
перед поворотом/целью/узким местом".

Цель изменения: сделать скорость first-class параметром follower, сохранив
существующий безопасный hold-on-empty-path контракт
(`drone_city_nav/src/px4_offboard_node.cpp:173`-`195`,
`drone_city_nav/src/px4_offboard_node.cpp:504`-`527`) и не привязать core logic
к Gazebo.

## Investigation context

`INVESTIGATION.md` прочитан и использован. Ключевые выводы:

- `INVESTIGATION.md:267`-`273`: предпочтительный следующий вариант - explicit
  velocity-aware offboard follower с `desired_speed_mps`, acceleration limit и
  braking distance.
- `INVESTIGATION.md:318`-`323`: нужно добавить явный `desired_speed_mps`, держать
  `max_commanded_target_step_m` как safety limiter и логировать actual/requested
  speed, target lead и commanded target step.
- `INVESTIGATION.md:324`-`330`: speed sweep следует делать только после
  headless mission validation; PX4 `MPC_*` tuning - отдельный следующий этап.

Важно: часть чисел в `INVESTIGATION.md` относится к предыдущей конфигурации.
Актуальные значения MVP speed tuning сейчас находятся в
`drone_city_nav/config/urban_mvp.yaml:85`-`87`.

## Detected stack/profiles

Прочитаны обязательные профили оркестратора:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`.

Основной стек workspace: C++20 ROS 2 Jazzy package `drone_city_nav`, build system
`ament_cmake` через `colcon`. В репозитории есть `Makefile`, `CMakeLists.txt`,
`.clang-format`, `.clang-tidy`, `build/compile_commands.json` и
`build/drone_city_nav/compile_commands.json`. `CMakePresets.json` не найден, так
что top-level ad-hoc CMake не использовать; предпочтительный entry point -
`colcon` через `Makefile`/scripts.

Notion не читался: в prompt нет Notion task ID/URL, а policy `optional`.
GitLab не читался: prompt не просит MR/GitLab context. Удалённые SSH/HTTP
источники не использовались.

## Repo-approved commands found

Найдены в `README.md`, `CONTRIBUTING.md`, `Makefile`,
`docs/MVP_SIMULATION.md` и scripts:

- `./scripts/dev_shell.sh`;
- `make build`;
- `colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`;
- `make test`;
- `ctest --test-dir build/drone_city_nav --output-on-failure`;
- `make quality`;
- `./scripts/check_cpp_quality.sh`;
- `./scripts/format_cpp_changed.sh`;
- `make sim-gui`;
- `./scripts/run_city_mvp.sh`;
- `make sim-headless`;
- `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`;
- `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh`.

Для долгих build/test/SITL команд использовать wrapper:
`/home/formi/.local/bin/runlim`.

## Affected components

- `drone_city_nav/src/px4_offboard_node.cpp`: ROS parameters, local position
  velocity intake, waypoint following, target smoothing, trajectory setpoint
  publication, speed diagnostics.
- Новый portable core module:
  `drone_city_nav/include/drone_city_nav/offboard_speed_controller.hpp` и
  `drone_city_nav/src/offboard_speed_controller.cpp`.
- `drone_city_nav/CMakeLists.txt`: добавить новый source в
  `drone_city_nav_core` и новый gtest target.
- `drone_city_nav/tests/offboard_speed_controller_test.cpp`: deterministic unit
  tests для speed profile.
- `drone_city_nav/config/urban_mvp.yaml`: simulation speed parameters.
- `drone_city_nav/config/real_drone_template.yaml`: conservative real-drone
  defaults, без aggressive SITL tuning.
- `drone_city_nav/src/mission_monitor_node.cpp`: желательно расширить итоговые
  logs observed max/average speed для headless evidence.
- `drone_city_nav/launch/city_nav.launch.py` и `scripts/run_city_mvp.sh`: только
  если нужен speed sweep без ручного редактирования YAML.
- `docs/MVP_SIMULATION.md`: документация новых speed parameters и log evidence
  на английском.

## Implementation steps

1. Добавить testable speed policy core.

   Файлы:
   `drone_city_nav/include/drone_city_nav/offboard_speed_controller.hpp`,
   `drone_city_nav/src/offboard_speed_controller.cpp`.

   Материализуемый результат: новый модуль без ROS/PX4/Gazebo зависимостей,
   подключённый к `drone_city_nav_core` в `drone_city_nav/CMakeLists.txt:28`-`34`.

   Типы и функции:

   - `struct SpeedControllerConfig`;
   - `struct SpeedControllerState`;
   - `struct SpeedLimitBreakdown`;
   - `struct SpeedControllerInput`;
   - `struct SpeedControllerOutput`;
   - `class OffboardSpeedController`;
   - `OffboardSpeedController::update(const SpeedControllerInput&)`;
   - helper functions `brakingLimitedSpeedMps`, `turnLimitedSpeedMps`,
     `clearanceLimitedSpeedMps`, `advanceToward`.

   Минимальный контракт:

   ```cpp
   struct SpeedControllerConfig {
     double desired_speed_mps{3.0};
     double max_accel_mps2{2.0};
     double min_command_speed_mps{0.0};
     double goal_slowdown_radius_m{10.0};
     double braking_safety_margin_m{1.0};
     double turn_slowdown_angle_rad{0.7};
     double turn_slowdown_min_speed_mps{1.5};
     double narrow_clearance_slowdown_radius_m{7.0};
     double narrow_clearance_min_speed_mps{1.0};
     double max_commanded_target_step_m{0.5};
   };
   ```

   Нетривиальная логика:

   ```cpp
   allowed = desired_speed_mps;
   remaining = max(distance_to_goal_m - braking_safety_margin_m, 0.0);
   allowed = min(allowed, sqrt(2.0 * max_accel_mps2 * remaining));
   allowed = min(allowed, turnLimitedSpeedMps(path_turn_angle_rad, config));
   allowed = min(allowed, clearanceLimitedSpeedMps(local_clearance_m, config));
   requested = advanceToward(previous_requested_speed, allowed,
                             max_accel_mps2 * controller_dt_s);
   target_step = min(requested * controller_dt_s, max_commanded_target_step_m);
   ```

   При hold/no-path/emergency/takeoff-hold output должен быть
   `requested_speed_mps=0`, `target_step_m=0`, `limit_reason="hold"`.

2. Подключить speed policy к `Px4OffboardNode` без изменения базового PX4 mode.

   Файл: `drone_city_nav/src/px4_offboard_node.cpp`.

   Code anchors:

   - объявление параметров в constructor:
     `drone_city_nav/src/px4_offboard_node.cpp:59`-`87`;
   - timer period: `drone_city_nav/src/px4_offboard_node.cpp:136`;
   - local position callback: `drone_city_nav/src/px4_offboard_node.cpp:310`-`348`;
   - `publishTrajectorySetpoint()`:
     `drone_city_nav/src/px4_offboard_node.cpp:439`-`458`;
   - `smoothedCommandTarget()`:
     `drone_city_nav/src/px4_offboard_node.cpp:546`-`569`;
   - members: `drone_city_nav/src/px4_offboard_node.cpp:663`-`721`.

   Материализуемый результат:

   - добавить параметры `desired_speed_mps`, `max_accel_mps2`,
     `goal_slowdown_radius_m`, `braking_safety_margin_m`,
     `turn_slowdown_angle_rad`, `turn_slowdown_min_speed_mps`,
     `narrow_clearance_slowdown_radius_m`,
     `narrow_clearance_min_speed_mps`, `velocity_feedforward_enabled`;
   - сохранить `max_commanded_target_step_m` как hard cap;
   - добавить `current_velocity_` и `current_speed_mps_`, заполнять из
     `VehicleLocalPosition.vx/vy` в `onLocalPosition()`;
   - заменить прямое использование `max_commanded_target_step_m_` в
     `smoothedCommandTarget()` на step из `OffboardSpeedController::update()`;
   - when no valid path/no navigation: держать текущую позицию и явно
     запрашивать speed `0 m/s`.

   Backward compatibility: если `desired_speed_mps` отсутствует в старом config,
   default должен соответствовать прежнему поведению примерно как
   `max_commanded_target_step_m / 0.1s`, но быть ограничен sane clamp range.

3. Добавить path-geometry slowdown перед поворотами.

   Файл: `drone_city_nav/src/px4_offboard_node.cpp`.

   Code anchors:

   - `advanceWaypointIfNeeded()`:
     `drone_city_nav/src/px4_offboard_node.cpp:478`-`502`;
   - `lookaheadWaypointIndex()`:
     `drone_city_nav/src/px4_offboard_node.cpp:239`-`259`;
   - `targetYaw()`:
     `drone_city_nav/src/px4_offboard_node.cpp:594`-`606`.

   Материализуемый результат: добавить helper
   `pathTurnAngleAtWaypoint(std::size_t waypoint_index) const`, который считает
   угол между segment `(previous/current)` и `(current/next)` для текущего
   waypoint. Передавать angle в speed controller. Для последней точки angle = 0,
   slowdown задаёт только braking-to-goal.

   Pseudocode:

   ```cpp
   v1 = normalize(current_wp - previous_wp);
   v2 = normalize(next_wp - current_wp);
   angle = acos(clamp(dot(v1, v2), -1.0, 1.0));
   ```

4. Добавить slowdown по узким местам через planner occupancy grid.

   Файл: `drone_city_nav/src/px4_offboard_node.cpp`.

   Материализуемый результат:

   - include `nav_msgs/msg/occupancy_grid.hpp`;
   - добавить parameter `occupancy_grid_topic`, default
     `/drone_city_nav/occupancy_grid`;
   - подписаться на planner grid, который публикуется в
     `planner_node::publishOccupancyGrid()`
     (`drone_city_nav/src/planner_node.cpp:543`-`570`);
   - добавить `estimateLocalClearanceM(Point2 point) const`, который по metadata
     grid ищет ближайшую occupied/inflated cell (`data >= 80`) в радиусе
     `narrow_clearance_slowdown_radius_m`;
   - если grid отсутствует/stale/outside, возвращать `NaN` и не замедляться по
     clearance, но логировать `clearance=nan`.

   Это не меняет A* contract: planner уже строит grid из persistent memory +
   current lidar overlay (`drone_city_nav/src/planner_node.cpp:313`-`317`), а
   offboard follower только читает этот опубликованный debug/planning artifact
   для speed limit.

5. Оставить velocity feedforward выключенным по умолчанию.

   Файл: `drone_city_nav/src/px4_offboard_node.cpp`.

   Code anchors:

   - `publishOffboardControlMode()`:
     `drone_city_nav/src/px4_offboard_node.cpp:426`-`436`;
   - `publishTrajectorySetpoint()`:
     `drone_city_nav/src/px4_offboard_node.cpp:439`-`458`.

   Материализуемый результат:

   - default: `position=true`, `velocity=false`, `TrajectorySetpoint.velocity`
     остаётся `{0,0,0}` для сохранения текущей PX4 semantics;
   - если `velocity_feedforward_enabled=true`, вычислять bounded velocity vector
     в направлении commanded target и публиковать его как feedforward только
     после unit test coverage и headless SITL проверки;
   - не переводить MVP сразу в velocity-only setpoints. PX4 velocity setpoints
     оставить отдельным follow-up после успешного position+feedforward sweep.

6. Обновить simulation и real-drone configs.

   Файлы:
   `drone_city_nav/config/urban_mvp.yaml`,
   `drone_city_nav/config/real_drone_template.yaml`.

   Материализуемый результат для `urban_mvp.yaml` рядом с
   `px4_offboard_node.ros__parameters` (`drone_city_nav/config/urban_mvp.yaml:72`-`102`):

   - `desired_speed_mps: 5.0`;
   - `max_accel_mps2: 2.5`;
   - `goal_slowdown_radius_m: 14.0`;
   - `braking_safety_margin_m: 1.0`;
   - `turn_slowdown_angle_rad: 0.65`;
   - `turn_slowdown_min_speed_mps: 1.5`;
   - `narrow_clearance_slowdown_radius_m: 7.0`;
   - `narrow_clearance_min_speed_mps: 1.2`;
   - `velocity_feedforward_enabled: false`;
   - оставить `max_commanded_target_step_m` не ниже `desired_speed_mps * 0.1`
     либо явно задокументировать, что он ограничивает requested speed сверху.

   Материализуемый результат для `real_drone_template.yaml`
   (`drone_city_nav/config/real_drone_template.yaml:80`-`107`):

   - добавить те же параметры, но conservative defaults:
     `desired_speed_mps: 1.0`, `max_accel_mps2: 0.75`,
     `velocity_feedforward_enabled: false`;
   - не копировать aggressive SITL values в real-drone template.

7. Расширить logs так, чтобы headless run доказывал управление скоростью.

   Файлы:
   `drone_city_nav/src/px4_offboard_node.cpp`,
   `drone_city_nav/src/mission_monitor_node.cpp`.

   Материализуемый результат:

   - startup log `PX4 offboard node ready` (`px4_offboard_node.cpp:140`-`153`)
     должен печатать `desired_speed_mps`, `max_accel_mps2`, slowdown radii,
     `velocity_feedforward_enabled`;
   - `logControlSummary()` (`px4_offboard_node.cpp:631`-`660`) должен печатать:
     `requested_speed`, `actual_speed`, `speed_limit_reason`,
     `braking_distance`, `target_step`, `turn_angle`, `local_clearance`;
   - `mission_monitor_node` уже считает actual speed
     (`drone_city_nav/src/mission_monitor_node.cpp:188`-`193`) и пишет final speed
     (`drone_city_nav/src/mission_monitor_node.cpp:302`-`311`); добавить
     `max_observed_speed_mps` и, при необходимости, `mean_observed_speed_mps` в
     `MISSION_RESULT` и `Mission summary`
     (`drone_city_nav/src/mission_monitor_node.cpp:335`-`351`).

8. Добавить unit tests для speed policy.

   Файл: `drone_city_nav/tests/offboard_speed_controller_test.cpp`.

   CMake anchor: `drone_city_nav/CMakeLists.txt:115`-`127`.

   Материализуемый результат:

   - добавить `ament_add_gtest(offboard_speed_controller_test ...)`;
   - link с `drone_city_nav_core`;
   - покрыть happy-path, negative-path и edge cases.

   Test names:

   - `OffboardSpeedController.CruiseSpeedAdvancesBySpeedTimesDt`;
   - `OffboardSpeedController.AccelerationRampLimitsSpeedIncrease`;
   - `OffboardSpeedController.BrakingDistanceLimitsGoalApproach`;
   - `OffboardSpeedController.SharpTurnUsesTurnSlowdownLimit`;
   - `OffboardSpeedController.NarrowClearanceUsesClearanceLimit`;
   - `OffboardSpeedController.HoldModeRequestsZeroSpeed`;
   - `OffboardSpeedController.MaxCommandedTargetStepHardCapWins`;
   - `OffboardSpeedController.NonFiniteInputsFailClosedToHold`.

9. Добавить возможность speed sweep без ручного редактирования tracked YAML.

   Файлы:
   `scripts/run_city_mvp.sh`, возможно новый `scripts/run_speed_sweep.sh`.

   Code anchors:

   - launch params file сейчас захардкожен:
     `scripts/run_city_mvp.sh:281`-`312`;
   - `city_nav.launch.py` принимает `params_file`:
     `drone_city_nav/launch/city_nav.launch.py:20`, `:101`-`105`.

   Материализуемый результат:

   - добавить в `run_city_mvp.sh` env `CITY_NAV_PARAMS_FILE`, default
     `${repo_root}/drone_city_nav/config/urban_mvp.yaml`;
   - `scripts/run_speed_sweep.sh` создаёт временные YAML copies под
     `build/speed_sweep/`, меняет только
     `px4_offboard_node.ros__parameters.desired_speed_mps`, запускает
     `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh`
     для набора скоростей `3 5 7`;
   - generated YAML и logs остаются в ignored `build/`/`log/`, tracked config не
     пачкается.

10. Обновить документацию на английском.

    Файл: `docs/MVP_SIMULATION.md`.

    Материализуемый результат:

    - обновить раздел `Current Limitations` (`docs/MVP_SIMULATION.md:283`-`320`)
      и добавить краткий subsection про speed-aware offboard follower;
    - задокументировать, что `desired_speed_mps` - requested cruise speed, а
      фактическая скорость ограничивается `max_accel_mps2`, braking-to-goal,
      turn/narrow slowdown, `max_commanded_target_step_m` и PX4 internals;
    - описать log evidence: `requested_speed`, `actual_speed`,
      `speed_limit_reason`, `MISSION_RESULT max_observed_speed_mps`.

## Verification plan

Минимальная проверка после реализации:

1. Форматирование изменённых C++ файлов:

   ```bash
   ./scripts/format_cpp_changed.sh
   ```

2. Scoped unit test нового target после build:

   ```bash
   /home/formi/.local/bin/runlim make build
   /home/formi/.local/bin/runlim ctest --test-dir build/drone_city_nav -R offboard_speed_controller_test --output-on-failure
   ```

3. Полный repo-approved quality gate:

   ```bash
   /home/formi/.local/bin/runlim ./scripts/check_cpp_quality.sh
   ```

4. Headless smoke:

   ```bash
   /home/formi/.local/bin/runlim env HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh
   ```

5. Full mission validation:

   ```bash
   /home/formi/.local/bin/runlim env HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh
   ```

6. Log evidence после full mission:

   ```bash
   rg "requested_speed|actual_speed|speed_limit_reason|MISSION_RESULT|collision_risk" log/ros_city_mvp.log
   ```

   Ожидаемый результат: есть `MISSION_RESULT success=true`, дрон стартует возле
   A, движется, сохраняет clearance, достигает B, останавливается; в offboard logs
   видно requested speed, actual speed и причины slowdown.

7. Если реализован `scripts/run_speed_sweep.sh`:

   ```bash
   /home/formi/.local/bin/runlim ./scripts/run_speed_sweep.sh 3 5 7
   ```

   Ожидаемый результат: каждая скорость либо проходит mission check, либо sweep
   явно останавливается на первой failing speed и оставляет logs.

Примечание для текущего хоста: предыдущие headless SITL прогоны могли требовать
запуска container с AMD DRI devices (`/dev/dri/renderD128`, `/dev/dri/card1`) из-за
EGL выбора NVIDIA dGPU. Если headless падает до ROS/PX4 evidence, сначала
проверить именно Gazebo/EGL logs, а не speed logic.

## Testing strategy

1. Категория 1: без рефакторинга.

   Можно добавить параметры прямо в `px4_offboard_node.cpp` и покрыть только
   mission-level headless checks. Это быстрее, но хуже: braking/turn/narrow logic
   останется завязанной на ROS node internals и будет почти недетерминированной в
   unit tests. Этот путь не рекомендован, кроме аварийного hotfix.

2. Категория 2: лёгкий рефакторинг, рекомендованный путь.

   Вынести speed math в `OffboardSpeedController` под `drone_city_nav_core`,
   оставить ROS/PX4 publication в `px4_offboard_node.cpp`. Unit tests покрывают
   speed limits, acceleration ramp, braking, turn slowdown, clearance slowdown и
   hold/no-path. SITL mission check покрывает интеграцию с PX4, planner и Gazebo.

3. Категория 3: тяжёлый рефакторинг.

   Ввести speed profile как часть planner output: отдельный message/waypoint DTO
   со speed limit per waypoint, clearance/cost metadata и возможно velocity-mode
   offboard controller. Это правильнее для будущего, но меняет ROS contracts и
   увеличивает blast radius. Для текущей задачи не делать, пока category 2 не
   упрётся в реальные ограничения.

Автотесты должны покрыть:

- happy-path: cruise speed на прямом участке и успешная mission check;
- negative-path: empty path/no local pose/emergency/NaN inputs дают hold speed 0;
- edge cases: почти нулевая дистанция до цели, резкий 90-degree turn, stale/absent
  clearance grid, hard cap `max_commanded_target_step_m`.

Gap: точная PX4 реакция на `velocity_feedforward_enabled=true` не покрывается unit
tests; её можно закрыть только SITL sweep. Поэтому default должен оставаться
`false`, пока sweep не подтверждён.

## Risks and tradeoffs

- Position-setpoint mode не гарантирует фактическую скорость ровно
  `desired_speed_mps`; он задаёт requested target progression, а PX4 всё ещё
  применяет свои velocity/acceleration/jerk limits.
- Слишком высокая скорость уменьшает время реакции на `35 m` lidar range и
  `replan_period_s: 0.75` (`drone_city_nav/config/urban_mvp.yaml:63`, `:70`).
- Turn slowdown по геометрии smoothed path может быть слишком поздним, если
  replanning часто меняет ближайший waypoint; mitigations: path continuity уже
  есть (`px4_offboard_node.cpp:261`-`308`), плюс logs `turn_angle`.
- Narrow-place slowdown по опубликованному occupancy grid может быть stale или
  coarse; не использовать его как collision proof, только как speed limiter.
- Velocity feedforward может изменить PX4 interpretation of setpoints; default
  должен быть disabled.
- Conservative `real_drone_template.yaml` обязателен, чтобы simulation tuning не
  утёк в real-hardware profile.

## Что могло сломаться

- Поведение полёта: дрон может начать тормозить слишком рано, слишком поздно или
  осциллировать около waypoint. Проверка: unit tests на speed profile плюс
  `MISSION_CHECK=1` full mission и logs `requested_speed/actual_speed`.
- API/контракты ROS parameters: старые YAML без новых параметров должны запускаться
  с defaults. Проверка: build/tests и запуск с `real_drone_template.yaml` в
  launch-only/smoke профиле без Gazebo-specific assumptions.
- PX4 integration: включение velocity feedforward может конфликтовать с текущим
  position-mode. Проверка: default `velocity_feedforward_enabled=false`; отдельный
  SITL sweep перед включением.
- Planner/offboard integration: чтение `/drone_city_nav/occupancy_grid` в offboard
  node может использовать stale grid и давать неверный narrow slowdown. Проверка:
  stale timeout test/helper и logs `local_clearance=nan|value`.
- Данные/БД: БД нет. Возможный риск только generated YAML/log artifacts для speed
  sweep. Проверка: artifacts создаются под ignored `build/`/`log/`, `git status`
  не содержит generated files.
- Производительность/ресурсы: поиск nearest occupied cell в grid может быть дорогим
  при каждом 10 Hz tick. Проверка: ограничить радиус поиска, не сканировать весь
  grid, добавить unit test на bounded search window и смотреть CPU/log timing в
  headless run.
- Документация/операторский UX: если logs переименованы неаккуратно,
  `scripts/run_city_mvp.sh` checks могут не находить ожидаемые patterns
  (`scripts/run_city_mvp.sh:228`-`279`). Проверка: smoke и full mission scripts.

## Open questions

1. Какая целевая MVP скорость нужна после explicit speed layer: оставить `5 m/s`
   как текущую примерно эквивалентную настройку или сразу валидировать `7 m/s`?
2. Нужно ли после category 2 делать отдельный этап PX4 `MPC_XY_VEL_MAX`,
   `MPC_ACC_HOR`, `MPC_JERK_MAX` tuning для технологического максимума?
3. Должен ли mission monitor в будущем fail-ить run при превышении
   `desired_speed_mps + tolerance`, или speed logs достаточно как diagnostics?
4. Включать ли `velocity_feedforward_enabled` в simulation default только после
   отдельного green speed sweep, или оставить это experimental flag навсегда?
