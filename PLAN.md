# Context

Нужно запланировать две связанные фичи для текущей ломаной траектории после A*
и `line-of-sight` схлопывания:

1. `velocity setpoint follower` для cruise-фазы движения по пути.
2. Плавное ограничение скорости/ускорения перед углами с учетом текущей
   скорости, расстояния до поворота и угла поворота.

Сейчас `px4_offboard_node` управляет PX4 через position setpoint:
`publishOffboardControlMode()` выставляет `position=true`, `velocity=false`
в `drone_city_nav/src/px4_offboard_node.cpp:705`, а
`publishTrajectorySetpoint()` отправляет `position=(target_x,target_y,-alt)` и
`velocity=NaN` в `drone_city_nav/src/px4_offboard_node.cpp:718`. В результате
PX4 сам разгоняется к очередной точке и гасит скорость около нее; это плохо
подходит для быстрого плавного пролета между препятствиями.

Планируемый результат после реализации: в takeoff/hold/no-path/final-stop
режимах узел может продолжать использовать position hold, но в cruise-фазе по
валидному пути он должен отправлять velocity setpoint: `position=NaN`,
`velocity=(vx,vy,vz)`, где горизонтальная скорость ведет дрон вдоль текущего
сегмента пути, а вертикальная скорость удерживает целевую высоту.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. `PLAN.md` до этого раунда также
отсутствовал, поэтому план создан с нуля.

Прочитанные локальные входные данные:

- `README.md`
- `CONTRIBUTING.md`
- `CPP_BEST_PRACTICES.md`
- `Makefile`
- `drone_city_nav/CMakeLists.txt`
- `docs/MVP_SIMULATION.md`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
- `drone_city_nav/src/offboard_path_follower.cpp`
- `drone_city_nav/src/px4_offboard_node.cpp`
- `drone_city_nav/include/drone_city_nav/offboard_target_continuity.hpp`
- `drone_city_nav/src/offboard_target_continuity.cpp`
- `drone_city_nav/tests/offboard_path_follower_test.cpp`
- `scripts/tests/test_offboard_telemetry_contract.py`
- `scripts/tests/test_topic_contract.py`

Обязательные orchestrator-протоколы прочитаны:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Notion/GitLab не запрашивались пользовательским промптом, `notion_policy` был
`optional`, поэтому внешние Notion/GitLab чтения не выполнялись.

# Detected stack/profiles

Основной стек workspace:

- ROS 2 workspace.
- `drone_city_nav` - `ament_cmake` C++20 package.
- PX4/Gazebo integration через ROS 2 nodes and launch/config files.
- Python helper/script tests under `scripts/tests`.

Примененные verification profiles:

- `generic.md` - обязателен для любого workspace.
- `cpp.md` - применим, потому что есть `Makefile`, `CMakeLists.txt`, C++
  headers/sources/tests.

Rust profile не применялся: в target workspace не найден Rust stack для
затрагиваемой задачи.

# Repo-approved commands found

Документированные команды из `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_gui.sh`
- `./scripts/sim_headless.sh`
- `./scripts/stop_sim.sh`
- `./scripts/container_run.sh make build`
- `./scripts/container_run.sh make test`
- `./scripts/container_run.sh make test-scripts`
- `./scripts/container_run.sh make quality`
- `./scripts/container_run.sh make format`

Workflow проекта: только container workflow. Не использовать host build и
ad-hoc top-level CMake. Симуляционные прогоны запускать только по явной команде
пользователя.

# Affected components

- `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp:12` -
  текущая ROS-free конфигурация follower helper.
- `drone_city_nav/src/offboard_path_follower.cpp:39` - projection/waypoint/turn
  helpers для ломаного пути.
- `drone_city_nav/tests/offboard_path_follower_test.cpp:23` - существующие unit
  tests для follower helpers.
- `drone_city_nav/src/px4_offboard_node.cpp:705` - публикация
  `OffboardControlMode`.
- `drone_city_nav/src/px4_offboard_node.cpp:718` - публикация
  `TrajectorySetpoint`.
- `drone_city_nav/src/px4_offboard_node.cpp:770` - advance/hold логика для
  текущего waypoint follower.
- `drone_city_nav/src/px4_offboard_node.cpp:1453` и
  `drone_city_nav/src/px4_offboard_node.cpp:1626` - runtime logs and
  `offboard_blackbox.jsonl`.
- `drone_city_nav/config/urban_mvp.yaml:103` - параметры `px4_offboard_node`.
- `docs/MVP_SIMULATION.md:456` - текущая документация position-only offboard.
- `scripts/tests/test_offboard_telemetry_contract.py:14` - статический
  контракт telemetry/blackbox полей.
- `drone_city_nav/CMakeLists.txt:29` and `drone_city_nav/CMakeLists.txt:149` -
  регистрация core sources and tests.

# Implementation steps

1. Добавить ROS-free модель velocity follower.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
   - `drone_city_nav/src/offboard_velocity_follower.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Результат:

   - Новый core helper без ROS/PX4 типов, пригодный для unit tests.
   - Типы:
     - `VelocityFollowerConfig`
     - `VelocityFollowerState`
     - `VelocitySetpointPlan`
     - `TurnSpeedPlan`
   - Функции:
     - `planVelocitySetpoint(...)`
     - `distanceFromProjectionToWaypoint(...)`
     - `speedLimitForUpcomingTurn(...)`
     - `limitVelocityVectorDelta(...)`

   Псевдокод контракта:

   ```cpp
   struct VelocityFollowerConfig {
     double cruise_speed_mps{12.0};
     double min_turn_speed_mps{2.0};
     double max_accel_mps2{3.0};
     double max_decel_mps2{4.0};
     double max_lateral_accel_mps2{3.0};
     double turn_slowdown_min_angle_rad{0.25};
     double sharp_turn_angle_rad{std::numbers::pi / 2.0};
     double braking_margin_m{2.0};
     double cross_track_gain{0.25};
     double max_cross_track_correction_angle_rad{0.35};
   };

   VelocitySetpointPlan planVelocitySetpoint(
       std::span<const Point2> path,
       Point2 current_position,
       Point2 current_velocity,
       bool current_velocity_valid,
       std::size_t waypoint_index,
       double dt_s,
       const VelocityFollowerState& previous_state,
       const VelocityFollowerConfig& config);
   ```

   Логика:

   - Проецировать текущую позицию на текущий/ближайший сегмент через уже
     существующий `closestOffboardPathProjection()`.
   - Базовое направление брать вдоль текущего path segment, а не прямо в точку.
   - Добавлять ограниченную cross-track correction компоненту от текущей
     позиции к projection point, чтобы дрон возвращался к линии без резкого
     рывка.
   - Не синтезировать дополнительные waypoint'ы и не менять опубликованный path.

2. Посчитать плавный speed profile перед углами текущей ломаной.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
   - `drone_city_nav/src/offboard_velocity_follower.cpp`
   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`

   Результат:

   - Скорость не переключается ступенькой на повороте, а снижается заранее по
     физической дистанции торможения.
   - Для каждого upcoming turn вычисляются:
     - `turn_angle_rad`
     - `distance_to_turn_m`
     - `target_turn_speed_mps`
     - `braking_distance_m`
     - `raw_speed_limit_mps`
     - `accel_limited_speed_mps`

   Псевдокод:

   ```cpp
   const double turn_speed = interpolateByAngle(
       angle_rad,
       config.turn_slowdown_min_angle_rad,
       config.sharp_turn_angle_rad,
       config.cruise_speed_mps,
       config.min_turn_speed_mps);

   const double braking_distance =
       std::max(0.0, (current_speed * current_speed - turn_speed * turn_speed) /
                          (2.0 * config.max_decel_mps2)) +
       config.braking_margin_m;

   if (distance_to_turn_m <= braking_distance) {
     speed_limit = std::sqrt(std::max(
         0.0,
         turn_speed * turn_speed +
             2.0 * config.max_decel_mps2 *
                 std::max(0.0, distance_to_turn_m - config.braking_margin_m)));
   } else {
     speed_limit = config.cruise_speed_mps;
   }
   ```

   Edge behavior:

   - Straight path: speed limit remains `cruise_speed_mps`.
   - Gentle turn below threshold: no slowdown.
   - Sharp turn near current position: speed limited near `min_turn_speed_mps`.
   - Invalid/degenerate path: return invalid plan; ROS node should fall back to
     position hold.

3. Заменить cruise publication in `px4_offboard_node` на velocity setpoint.

   Файл:

   - `drone_city_nav/src/px4_offboard_node.cpp`

   Code anchors:

   - `publishOffboardControlMode()` at `drone_city_nav/src/px4_offboard_node.cpp:705`
   - `publishTrajectorySetpoint()` at `drone_city_nav/src/px4_offboard_node.cpp:718`
   - `shouldHoldPosition()` at `drone_city_nav/src/px4_offboard_node.cpp:1099`
   - `updateNavigationStartState()` at `drone_city_nav/src/px4_offboard_node.cpp:1395`

   Результат:

   - В takeoff, no-path, stale-pose, emergency and final hold режимах оставить
     position setpoint behavior.
   - В cruise-фазе при `navigationAllowed() && path_valid_ &&
     localPositionFresh()` публиковать:

     ```cpp
     msg.position = {nan, nan, nan};
     msg.velocity = {vx, vy, vz_ned};
     msg.acceleration = {nan, nan, nan};
     msg.jerk = {nan, nan, nan};
     ```

   - `publishOffboardControlMode()` должен выставлять `velocity=true`,
     `position=false` только для cruise velocity mode; для hold modes оставлять
     `position=true`, `velocity=false`.
   - Добавить vertical velocity hold:

     ```cpp
     altitude_error_m = cruise_altitude_m_ - current_altitude_m_;
     vz_ned = -clamp(altitude_error_m * altitude_hold_kp,
                     -max_vertical_speed_mps,
                     max_vertical_speed_mps);
     ```

   - При достижении последней waypoint/goal переключаться в position hold у
     текущей/goal позиции, чтобы mission monitor видел остановку.

4. Убрать fixed sharp-turn hold из нормального cruise decision path.

   Файл:

   - `drone_city_nav/src/px4_offboard_node.cpp`

   Code anchors:

   - `advanceWaypointIfNeeded()` at `drone_city_nav/src/px4_offboard_node.cpp:770`
   - `shouldStartSharpTurnHold()` at `drone_city_nav/src/px4_offboard_node.cpp:847`
   - `startSharpTurnHold()` at `drone_city_nav/src/px4_offboard_node.cpp:863`
   - `motionPhaseName()` at `drone_city_nav/src/px4_offboard_node.cpp:1126`

   Результат:

   - Fixed hold перед резким поворотом больше не является основным способом
     "торможения".
   - Если решено оставить compatibility-параметры на один переходный релиз, они
     должны быть явно documented as legacy/disabled in velocity mode, а logs
     должны показывать, что cruise speed profile управляет замедлением.
   - `target_switch_hold_*` тоже не должен использоваться как тормоз в velocity
     cruise; резкие изменения направления сглаживаются через
     `limitVelocityVectorDelta()`.

5. Добавить параметры velocity follower в runtime config.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/config/urban_mvp.yaml`

   Результат:

   - Новые параметры `px4_offboard_node`:
     - `cruise_velocity_control_enabled: true`
     - `cruise_speed_mps`
     - `min_turn_speed_mps`
     - `max_accel_mps2`
     - `max_decel_mps2`
     - `max_lateral_accel_mps2`
     - `turn_slowdown_min_angle_deg`
     - `sharp_turn_angle_deg`
     - `braking_margin_m`
     - `cross_track_gain`
     - `max_cross_track_correction_angle_deg`
     - `altitude_hold_kp`
     - `max_vertical_speed_mps`
   - Значения по умолчанию должны быть консервативными для первого запуска,
     например `cruise_speed_mps=10.0-12.0`, `min_turn_speed_mps=2.0-3.0`,
     `max_accel_mps2=2.0-3.0`, `max_decel_mps2=3.0-4.0`.
   - Startup log должен печатать все новые параметры одной строкой.

6. Обновить waypoint advancement for velocity cruise.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
   - `drone_city_nav/src/offboard_path_follower.cpp`
   - `drone_city_nav/src/px4_offboard_node.cpp`
   - `drone_city_nav/tests/offboard_path_follower_test.cpp`

   Результат:

   - Advance должен учитывать projection progress along segment, чтобы waypoint
     переключался, когда дрон фактически прошел waypoint along path, а не только
     когда попал в маленький радиус around point.
   - Сохранить защиту от перепрыгивания назад.
   - Для финальной точки сохранить acceptance radius / position hold behavior.

   Псевдокод:

   ```cpp
   if (projection.segment_start_index + 1U > waypoint_index) {
     waypoint_index = projection.segment_start_index + 1U;
   }
   if (distance(current_position, path[waypoint_index]) <= acceptance_radius_m) {
     ++waypoint_index;
   }
   ```

   При velocity mode нужно добавить тест, что дрон, идущий параллельно сегменту
   с небольшой lateral error, не застревает на уже пройденной waypoint.

7. Расширить runtime logs and blackbox для headless debugging.

   Файл:

   - `drone_city_nav/src/px4_offboard_node.cpp`

   Code anchors:

   - `logControlSummary()` at `drone_city_nav/src/px4_offboard_node.cpp:1453`
   - `logTelemetry()` at `drone_city_nav/src/px4_offboard_node.cpp:1511`
   - `writeFlightBlackbox()` at `drone_city_nav/src/px4_offboard_node.cpp:1626`

   Результат:

   Runtime log and JSONL должны содержать:

   - `control_mode=position_hold|velocity_cruise`
   - `velocity_setpoint=(vx,vy,vz)`
   - `velocity_setpoint_speed_mps`
   - `speed_limit_reason=straight|gentle_turn|braking_for_turn|invalid_path|hold`
   - `raw_speed_limit_mps`
   - `accel_limited_speed_mps`
   - `turn_target_speed_mps`
   - `braking_distance_m`
   - `distance_to_turn_m`
   - `turn_angle_rad`
   - `velocity_delta_mps`
   - `cross_track_correction_mps`
   - `altitude_error_m`

   Это позволит в headless run видеть, действительно ли дрон заранее снижает
   velocity command перед поворотом и не получает резких векторных команд.

8. Обновить документацию.

   Файлы:

   - `docs/MVP_SIMULATION.md`
   - `README.md` при необходимости

   Результат:

   - Раздел `Offboard Position Control` должен быть заменен или расширен на
     `Offboard Velocity Cruise Control`.
   - Документировать, что:
     - takeoff/hold uses position setpoints;
     - cruise uses velocity setpoints;
     - no intermediate path waypoints are synthesized;
     - corner rounding пока не входит в этот этап;
     - speed profile работает на текущей схлопнутой ломаной.

9. Добавить unit tests for velocity follower.

   Файлы:

   - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Категории тестов:

   - Happy path:
     - straight path returns cruise velocity along segment;
     - far sharp turn keeps cruise speed;
     - near sharp turn reduces speed according to braking formula.
   - Negative path:
     - empty path returns invalid/hold plan;
     - non-finite current position/current velocity returns invalid/hold plan;
     - degenerate zero-length segment does not divide by zero.
   - Edge cases:
     - already near final waypoint returns final/hold intent;
     - acceleration limit clamps speed increase between ticks;
     - vector delta limit clamps abrupt 90-degree command changes;
     - cross-track correction is bounded by
       `max_cross_track_correction_angle_rad`.

10. Обновить existing tests/contracts.

    Файлы:

    - `drone_city_nav/tests/offboard_path_follower_test.cpp`
    - `scripts/tests/test_offboard_telemetry_contract.py`
    - возможно `scripts/tests/test_validate_city_mvp_headless.py`

    Результат:

    - Existing path follower tests остаются зелеными.
    - Добавить проверки новых telemetry/blackbox field names.
    - Если headless validator ожидает position-only logs, обновить markers на
      velocity cruise logs без ослабления проверки.

# Verification plan

Команды выполнять только через container workflow.

Минимальные проверки после реализации:

1. Format changed C++ files:

   ```bash
   ./scripts/container_run.sh make format
   ```

2. Unit/integration tests for package:

   ```bash
   ./scripts/container_run.sh make test
   ```

3. Script-level static/contract tests:

   ```bash
   ./scripts/container_run.sh make test-scripts
   ```

4. Full quality gate:

   ```bash
   ./scripts/container_run.sh make quality
   ```

Симуляционные прогоны не запускать автоматически: пользователь отдельно
запретил run без явной команды. Если пользователь отдельно разрешит, после
реализации полезные проверки:

```bash
MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/sim_headless.sh
```

и затем анализ `log/offboard_blackbox.jsonl` по новым velocity fields.

# Testing strategy

## Категория 1: без рефакторинга

- Добавить `offboard_velocity_follower_test` для ROS-free алгоритма скорости.
- Обновить `offboard_path_follower_test` для progress/advance поведения, если
  меняется helper.
- Обновить `test_offboard_telemetry_contract.py` для новых log/blackbox полей.

## Категория 2: легкий рефакторинг

- Вынести вычисление `VelocitySetpointPlan` из `px4_offboard_node.cpp` в core
  helper, чтобы не тестировать ROS timers/PX4 messages напрямую.
- В `px4_offboard_node.cpp` оставить только:
  - чтение параметров;
  - сбор state snapshot;
  - выбор `position_hold` vs `velocity_cruise`;
  - публикацию PX4 messages;
  - logs/blackbox.

## Категория 3: тяжелый рефакторинг

- Полностью разделить node orchestration and flight-control backend на отдельный
  класс, например `OffboardFlightController`.
- Для текущих двух фич это не обязательно. Делать только если реализация в
  `px4_offboard_node.cpp` начнет неконтролируемо расти или станет плохо
  тестируемой.

Автотестовые gaps:

- Реальное поведение PX4 velocity mode нельзя доказать unit tests без SITL.
  Это закрывается только явным headless/GUI run по команде пользователя и
  анализом `offboard_blackbox.jsonl`.

# Risks and tradeoffs

- PX4 velocity mode требует корректного вертикального управления. Если
  `vz_ned` знак или clamp ошибочны, дрон может терять/набирать высоту. Это
  проверяется unit tests на sign convention и headless telemetry после явного
  разрешения run.
- Velocity cruise может хуже удерживать path without cross-track correction.
  Поэтому correction нужна, но она должна быть ограничена, иначе вернется
  раскачка.
- Слишком высокий `cruise_speed_mps` при резких углах будет требовать раннего
  торможения. Если `max_decel_mps2` завысить, plan будет оптимистичным и дрон
  может не успеть сбросить скорость.
- Слишком низкий `min_turn_speed_mps` сделает движение безопаснее, но может
  выглядеть как "патруль" вместо быстрого пролета.
- Удаление/деактивация fixed sharp-turn hold меняет текущий safety behavior.
  Компенсация: speed profile плюс acceleration/vector-delta limits.
- Replan continuity сейчас завязана на target points. После velocity cruise
  нужно убедиться, что path update не сбрасывает velocity state резко; это
  проверяется тестом на `limitVelocityVectorDelta()` and blackbox fields.
- Mission monitor ожидает достижение goal and low stop speed. Нужно явно
  переключаться в final position hold после достижения последней waypoint.

# Open questions

- Стартовые численные значения `cruise_speed_mps`, `max_accel_mps2`,
  `max_decel_mps2` и `min_turn_speed_mps` нужно будет уточнять по SITL telemetry.
  Для первого implementation pass план предлагает консервативные defaults.
- Нужно ли полностью удалить старые `sharp_turn_hold_*` параметры сразу или
  оставить как legacy-disabled compatibility на один переходный этап? С точки
  зрения новой схемы они не должны управлять cruise-полетом.
- Нужно ли в этом же этапе добавить отдельный analyzer script для
  `offboard_blackbox.jsonl`, который автоматически считает пики speed/tilt,
  speed command delta и braking-before-turn evidence? Это полезно, но может быть
  отдельным небольшим follow-up, если не требуется для минимальной приемки.
