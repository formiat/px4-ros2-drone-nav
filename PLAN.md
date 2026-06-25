# Context

Нужно детально спланировать архитектурное улучшение определения оптимальной скорости
для текущего velocity-based Offboard управления дроном. Цель: разнести смешанную
логику из `planVelocitySetpoint()` на явные слои:

```text
final racing trajectory + drone_state
  -> project drone to trajectory
  -> sample/look ahead speed profile
  -> target scalar speed
  -> target velocity vector
  -> acceleration/jerk smoothing
  -> PX4 velocity setpoint
```

Важно спланировать все перечисленные пункты в текущей итерации, без переноса
части discovery на этап реализации. Новый и затронутый функционал должен быть
достаточно покрыт автотестами и логами, чтобы поведение можно было отлаживать в
headless-режиме по логам и JSONL.

# Investigation context

`INVESTIGATION.md` в корне workspace отсутствует. План составлен по текущему
локальному коду и проектным документам.

Ключевые наблюдения по текущей реализации:

1. Публичный API velocity follower сейчас находится в
   `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:27`.
   В одном `VelocityFollowerConfig` смешаны параметры speed profile, cross-track
   correction, feedforward acceleration и smoothing.
2. `TrajectorySpeedProfile` и `TrajectorySpeedSample` уже существуют в
   `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:68`.
   Они описывают геометрический и профилированный лимит скорости.
3. Speed profile строится в `buildTrajectorySpeedProfile()`:
   - по `TrajectorySegment`:
     `drone_city_nav/src/offboard_velocity_follower.cpp:451`;
   - по `TrajectoryPointSample`:
     `drone_city_nav/src/offboard_velocity_follower.cpp:495`.
4. `finalizeSpeedProfile()` уже делает backward pass для торможения и forward
   pass для разгона:
   `drone_city_nav/src/offboard_velocity_follower.cpp:374`.
5. Текущий runtime-control смешан в одной функции
   `planVelocitySetpoint()`:
   `drone_city_nav/src/offboard_velocity_follower.cpp:651`.
   Внутри одновременно выполняются projection, выбор scalar speed,
   cross-track guard, correction vector, vector accel/decel limiting,
   velocity jerk limiting, curvature feedforward acceleration и заполнение
   диагностики.
6. Offboard node строит final samples/profile при получении новой траектории в
   `applyReceivedFinalTrajectoryPath()`:
   `drone_city_nav/src/px4_offboard_node.cpp:598`.
7. Offboard node вызывает `planVelocitySetpoint()` каждые 50 ms через
   controller timer:
   `drone_city_nav/src/px4_offboard_node.cpp:1282`.
8. PX4 сейчас получает именно velocity + acceleration setpoint, не position
   setpoint, в
   `drone_city_nav/src/px4_offboard_node.cpp:1302`.
9. Runtime-логи уже содержат частично разделённые speed stages:
   `profile_speed_limit`, `cross_track_speed_factor`,
   `cross_track_limited_speed`, `final_command_speed`:
   `drone_city_nav/src/px4_offboard_node.cpp:1830` и
   `drone_city_nav/src/px4_offboard_node.cpp:1998`.
10. JSONL blackbox уже пишет velocity command diagnostics:
    `drone_city_nav/src/px4_offboard_node.cpp:2178`.
11. Конфигурация текущих speed/control параметров находится в
    `drone_city_nav/config/urban_mvp.yaml:153`.
12. Существующие unit tests для текущего смешанного follower находятся в
    `drone_city_nav/tests/offboard_velocity_follower_test.cpp:47`.
13. Script-level telemetry contract проверяется в
    `scripts/tests/test_offboard_telemetry_contract.py:37`.

# Detected stack/profiles

Стек workspace:

- ROS 2 workspace.
- Основной пакет: `drone_city_nav`, `ament_cmake` / CMake / C++20.
- Build/test entrypoint: `colcon` через repo-approved `Makefile` и wrapper
  scripts.
- Есть Python script-level tests в `scripts/tests`.
- Rust manifest в workspace не обнаружен, Rust profile не применялся.

Прочитанные обязательные профили и протоколы:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Notion/GitLab в пользовательском prompt не указаны; `notion_policy=optional`,
поэтому чтение Notion/GitLab задач не выполнялось.

# Repo-approved commands found

Документированные команды найдены в `AGENTS.md`, `README.md`, `CONTRIBUTING.md`
и `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/test.sh` / внутри контейнера `make test`
- `./scripts/dev_shell.sh make format`
- `./scripts/dev_shell.sh make quality`
- `./scripts/dev_shell.sh make test-scripts`
- `./scripts/sim_headless.sh`
- `./scripts/sim_gui.sh`
- `./scripts/stop_sim.sh`

Правило проекта: использовать только container workflow. Не использовать
ad-hoc top-level CMake commands.

# Affected components

1. `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
   - сейчас содержит смешанные типы speed planning, command planning,
     smoothing state и финальный `VelocitySetpointPlan`.
2. `drone_city_nav/src/offboard_velocity_follower.cpp`
   - сейчас содержит все алгоритмы speed profile, scalar speed selection,
     cross-track correction, velocity vector limiting, jerk limiting и final
     plan composition.
3. `drone_city_nav/src/px4_offboard_node.cpp`
   - call site для построения trajectory speed profile и публикации
     PX4 `TrajectorySetpoint`.
   - runtime logs и blackbox JSONL.
4. `drone_city_nav/config/urban_mvp.yaml`
   - runtime параметры speed profile, accel/decel, cross-track correction,
     smoothing.
5. `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
   - текущие unit tests смешанной функции.
6. `drone_city_nav/tests/px4_offboard_config_test.cpp`
   - YAML contract для параметров.
7. `scripts/tests/test_offboard_telemetry_contract.py`
   - contract для логов, JSONL и runtime config.
8. `drone_city_nav/CMakeLists.txt`
   - регистрация новых test targets, если новые модули будут отдельными
     translation units.

# Implementation steps

1. Вынести scalar speed profile в отдельный модуль
   `TrajectorySpeedPlanner`.

   Файлы:
   - добавить `drone_city_nav/include/drone_city_nav/trajectory_speed_planner.hpp`;
   - добавить `drone_city_nav/src/trajectory_speed_planner.cpp`;
   - обновить `drone_city_nav/CMakeLists.txt`;
   - убрать speed-profile declarations из
     `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:68`,
     оставив обратимо совместимые `#include`/using при необходимости.

   Переносимый функционал:
   - `SpeedConstraintType`;
   - `TrajectorySpeedSample`;
   - `TrajectorySpeedProfile`;
   - `TraversalTimeEstimate`;
   - `buildTrajectorySpeedProfile(...)`;
   - `speedProfileSampleAtS(...)`;
   - `estimateTraversalTime(...)`;
   - `distanceFromTrajectorySToEnd(...)`;
   - private helpers из
     `drone_city_nav/src/offboard_velocity_follower.cpp:233-590`,
     относящиеся только к profile generation.

   Ожидаемый результат:
   - speed profile можно тестировать без velocity command и PX4/offboard
     зависимостей;
   - `offboard_velocity_follower.cpp` перестаёт владеть логикой curvature
     speed profile.

2. Добавить runtime scalar speed selection API с lookahead поверх готового
   profile.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/trajectory_speed_planner.hpp`;
   - `drone_city_nav/src/trajectory_speed_planner.cpp`;
   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
     только для интеграции результата в `VelocitySetpointPlan`.

   Новый контракт:

   ```cpp
   struct ScalarSpeedQuery {
     double trajectory_s_m;
     double cross_track_error_m;
     double previous_command_speed_mps;
     double current_speed_mps;
     double dt_s;
   };

   struct ScalarSpeedPlan {
     bool valid;
     SpeedConstraintType constraint_type;
     std::size_t constraint_index;
     double profile_speed_limit_mps;
     double lookahead_distance_m;
     double lookahead_speed_limit_mps;
     double cross_track_speed_factor;
     double cross_track_limited_speed_mps;
     double accel_limited_speed_mps;
     double final_scalar_speed_mps;
   };
   ```

   Алгоритм:

   ```text
   current profile limit = profile(s)
   lookahead distance = clamp(current_speed * lookahead_time,
                              lookahead_min_m,
                              lookahead_max_m)
   lookahead speed limit = min(profile(s ... s + lookahead_distance))
   profile/lookahead limit = min(current profile limit, lookahead speed limit)
   cross-track limited speed = profile/lookahead limit * crossTrackSpeedFactor(error)
   accel/decel limited speed = previous_speed +/- max_accel/max_decel * dt
   ```

   Новые параметры `VelocityFollowerConfig` или отдельного
   `TrajectorySpeedPlannerConfig`:
   - `speed_profile_lookahead_time_s`;
   - `speed_profile_lookahead_min_m`;
   - `speed_profile_lookahead_max_m`.

   Начальные значения для `urban_mvp.yaml`:
   - `speed_profile_lookahead_time_s: 1.0`;
   - `speed_profile_lookahead_min_m: 5.0`;
   - `speed_profile_lookahead_max_m: 35.0`.

   Ожидаемый результат:
   - оптимальная scalar speed явно считается отдельной функцией;
   - runtime смотрит немного вперёд по траектории, но не заменяет уже
     существующий backward pass;
   - в логах можно отличить `profile_speed_limit` от
     `lookahead_speed_limit`.

3. Вынести построение desired velocity vector в отдельный модуль
   `VelocityCommandPlanner`.

   Файлы:
   - добавить `drone_city_nav/include/drone_city_nav/velocity_command_planner.hpp`;
   - добавить `drone_city_nav/src/velocity_command_planner.cpp`;
   - обновить `drone_city_nav/CMakeLists.txt`;
   - удалить соответствующие private helpers из
     `drone_city_nav/src/offboard_velocity_follower.cpp:111-166`;
   - заменить блоки в `planVelocitySetpoint()`:
     `drone_city_nav/src/offboard_velocity_follower.cpp:709-738`.

   Новый контракт:

   ```cpp
   struct VelocityCommandQuery {
     TrajectoryProjection projection;
     Point2 current_position;
     Point2 current_velocity;
     bool current_velocity_valid;
     double scalar_speed_mps;
     double dt_s;
     Point2 previous_cross_track_correction_velocity;
     bool previous_cross_track_correction_velocity_valid;
   };

   struct VelocityCommandPlan {
     bool valid;
     Point2 desired_velocity_xy;
     Point2 raw_cross_track_correction_velocity;
     Point2 cross_track_correction_velocity;
     double raw_cross_track_correction_mps;
     double cross_track_correction_mps;
     double cross_track_correction_delta_mps;
     double cross_track_lateral_velocity_mps;
     double desired_velocity_tangent_mps;
     double desired_velocity_normal_mps;
   };
   ```

   Алгоритм остаётся текущим:

   ```text
   cross_track = projection.point - current_position
   correction_speed = max(0, gain * cross_track_error
                             - derivative_gain * lateral_velocity)
   bounded correction = angle-limited correction relative to scalar speed
   rate-limited correction = limitVectorRate(...)
   desired direction = normalize(tangent * max(speed, 1.0) + correction)
   desired velocity = desired direction * scalar speed
   ```

   Ожидаемый результат:
   - cross-track correction можно тестировать изолированно;
   - `planVelocitySetpoint()` больше не смешивает correction vector и speed
     profile.

4. Вынести smoothing/accel/jerk limiting в отдельный модуль
   `VelocitySmoother`.

   Файлы:
   - добавить `drone_city_nav/include/drone_city_nav/velocity_smoother.hpp`;
   - добавить `drone_city_nav/src/velocity_smoother.cpp`;
   - обновить `drone_city_nav/CMakeLists.txt`;
   - перенести `VelocityFollowerState` или выделить из него
     `VelocitySmootherState` из
     `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:50`;
   - перенести `VelocityVectorLimitResult`, `limitVelocityVectorDelta()`,
     `limitVelocitySetpointJerk()` из
     `drone_city_nav/src/offboard_velocity_follower.cpp:168` и
     `drone_city_nav/src/offboard_velocity_follower.cpp:592`.

   Новый контракт:

   ```cpp
   struct VelocitySmootherInput {
     Point2 desired_velocity_xy;
     Point2 previous_velocity_setpoint;
     Point2 previous_velocity_acceleration_setpoint;
     bool previous_velocity_setpoint_valid;
     bool previous_velocity_acceleration_setpoint_valid;
     double dt_s;
   };

   struct VelocitySmootherPlan {
     Point2 velocity_xy;
     Point2 velocity_setpoint_acceleration_xy;
     double velocity_delta_mps;
     double desired_velocity_delta_mps;
     double velocity_setpoint_acceleration_mps2;
     double velocity_setpoint_jerk_mps3;
   };
   ```

   Обязательное поведение сохранить:
   - продольное торможение не блокируется `max_velocity_jerk_mps3`;
   - боковое изменение velocity vector сглаживается;
   - invalid/non-finite input возвращает invalid plan без undefined behavior.

   Ожидаемый результат:
   - smoothing state можно сбрасывать и тестировать независимо от
     trajectory/speed profile;
   - риск “старый вектор тянет новый маршрут” становится локализованным.

5. Оставить `planVelocitySetpoint()` как фасад-оркестратор, но убрать из него
   алгоритмические детали.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`;
   - `drone_city_nav/src/offboard_velocity_follower.cpp`.

   Новый порядок внутри `planVelocitySetpoint()`:

   ```text
   validate inputs
   final goal hold check
   projection = projectOnTrajectory(...)
   scalar_speed = planScalarSpeed(...)
   command = planVelocityCommand(...)
   smoothed = smoothVelocityCommand(...)
   feedforward_accel = planCurvatureFeedforward(...)
   fill VelocitySetpointPlan diagnostics
   ```

   Ожидаемый результат:
   - call site в `px4_offboard_node.cpp:1287` не меняется;
   - публичное поведение сохраняется, но код становится проверяемым слоями;
   - `VelocitySetpointPlan` остаётся единым диагностическим объектом для
     offboard logs/blackbox.

6. Добавить явную диагностику lookahead speed stage.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`;
   - `drone_city_nav/src/offboard_velocity_follower.cpp`;
   - `drone_city_nav/src/px4_offboard_node.cpp`;
   - `scripts/tests/test_offboard_telemetry_contract.py`.

   Добавить в `VelocitySetpointPlan`:
   - `speed_lookahead_distance_m`;
   - `lookahead_speed_limit_mps`;
   - `lookahead_limiting_constraint_type`;
   - `lookahead_limiting_constraint_distance_m`;
   - `speed_after_lookahead_mps`.

   Добавить в runtime logs:

   ```text
   profile_speed_limit=...
   lookahead_distance=...
   lookahead_speed_limit=...
   speed_after_lookahead=...
   cross_track_limited_speed=...
   accel_limited_speed=...
   final_command_speed=...
   ```

   Добавить в blackbox JSONL внутри `velocity_command`:
   - `lookahead_distance_m`;
   - `lookahead_speed_limit_mps`;
   - `speed_after_lookahead_mps`;
   - `lookahead_limiting_constraint_type`;
   - `lookahead_limiting_constraint_distance_m`.

   Ожидаемый результат:
   - headless debug сможет ответить, почему скорость была такой: из-за
     curvature profile, lookahead, cross-track guard, accel/decel или smoother.

7. Обновить runtime config.

   Файлы:
   - `drone_city_nav/config/urban_mvp.yaml`;
   - `drone_city_nav/tests/px4_offboard_config_test.cpp`;
   - `scripts/tests/test_offboard_telemetry_contract.py`.

   Добавить параметры:
   - `speed_profile_lookahead_time_s: 1.0`;
   - `speed_profile_lookahead_min_m: 5.0`;
   - `speed_profile_lookahead_max_m: 35.0`.

   В `px4_offboard_node.cpp` добавить чтение параметров рядом с текущими
   speed profile параметрами:
   `drone_city_nav/src/px4_offboard_node.cpp:188`.

   Ожидаемый результат:
   - lookahead управляется конфигом;
   - дефолтные параметры зафиксированы тестами.

8. Обновить CMake targets.

   Файл:
   - `drone_city_nav/CMakeLists.txt`.

   Добавить новые source files в `drone_city_nav_core` или соответствующий
   target:
   - `src/trajectory_speed_planner.cpp`;
   - `src/velocity_command_planner.cpp`;
   - `src/velocity_smoother.cpp`.

   Добавить новые gtest targets:
   - `trajectory_speed_planner_test`;
   - `velocity_command_planner_test`;
   - `velocity_smoother_test`.

   Ожидаемый результат:
   - новые слои собираются и тестируются отдельно;
   - `offboard_velocity_follower_test` остаётся integration-level test для
     фасада.

9. Добавить unit tests для `TrajectorySpeedPlanner`.

   Файл:
   - добавить `drone_city_nav/tests/trajectory_speed_planner_test.cpp`.

   Покрыть:
   - happy-path: широкий радиус даёт больший `geometric_limit_mps`, чем узкий;
   - happy-path: backward pass снижает скорость до поворота/goal;
   - lookahead: при upcoming low-speed sample `lookahead_speed_limit_mps`
     меньше текущего `profile_speed_limit_mps`;
   - edge-case: `lookahead_distance` clamp по min/max;
   - edge-case: пустой/invalid profile возвращает invalid scalar plan;
   - negative-path: non-finite `trajectory_s_m` или `dt_s` sanitization не
     создаёт NaN в публичном result.

   Ожидаемый результат:
   - scalar speed можно валидировать без PX4/offboard node.

10. Добавить unit tests для `VelocityCommandPlanner`.

    Файл:
    - добавить `drone_city_nav/tests/velocity_command_planner_test.cpp`.

    Покрыть:
    - straight trajectory: desired velocity совпадает с tangent при нулевом
      cross-track;
    - cross-track correction bounded by max correction angle;
    - derivative damping уменьшает correction при движении к траектории;
    - rate limit сглаживает correction velocity;
    - invalid projection/tangent возвращает invalid command plan.

    Ожидаемый результат:
    - lateral correction логика отделена от scalar speed и smoothing.

11. Добавить unit tests для `VelocitySmoother`.

    Файл:
    - добавить `drone_city_nav/tests/velocity_smoother_test.cpp`.

    Покрыть:
    - direction change: боковая компонента ограничена jerk/accel;
    - aggressive braking: продольное торможение не блокируется jerk-limit;
    - acceleration limiting: `max_accel_mps2` ограничивает разгон;
    - deceleration limiting: `max_decel_mps2` ограничивает снижение скорости;
    - reset state: после reset новый desired velocity не тянется старым
      acceleration state;
    - non-finite input: invalid/hold без NaN в публичной диагностике.

    Ожидаемый результат:
    - поведение smoother проверяется отдельно от траектории.

12. Обновить integration tests для фасада `planVelocitySetpoint()`.

    Файл:
    - `drone_city_nav/tests/offboard_velocity_follower_test.cpp`.

    Сохранить и расширить существующие проверки:
    - `StraightTrajectoryReturnsCruiseVelocityAlongTangent`;
    - `FinalStopProfileLimitsSpeedNearGoal`;
    - `CrossTrackSpeedGuardReducesSpeedWhenFarFromPath`;
    - `VelocityJerkLimitDoesNotBlockLongitudinalBraking`;
    - добавить тест “lookahead stage affects final scalar speed before
      command planner”.

    Ожидаемый результат:
    - после refactor фасад выдаёт те же ключевые поля `VelocitySetpointPlan`;
    - новая lookahead-диагностика проходит через весь pipeline.

13. Обновить telemetry/script contract tests.

    Файл:
    - `scripts/tests/test_offboard_telemetry_contract.py`.

    Добавить проверки наличия:
    - `lookahead_distance=%.2f`;
    - `lookahead_speed_limit=%.2f`;
    - `speed_after_lookahead=%.2f`;
    - `lookahead_distance_m`;
    - `lookahead_speed_limit_mps`;
    - `speed_after_lookahead_mps`;
    - новых YAML параметров.

    Ожидаемый результат:
    - диагностический контракт не потеряется при будущих правках.

14. Обновить blackbox/CSV-debug ожидания без добавления ручных проверок.

    Файлы:
    - `drone_city_nav/src/px4_offboard_node.cpp`;
    - `scripts/tests/test_offboard_telemetry_contract.py`;
    - при необходимости `drone_city_nav/src/trajectory_diagnostics_io.cpp`.

    Ожидаемый результат:
    - `log/offboard_blackbox.jsonl` в headless run содержит полный speed-stage
      breakdown;
    - существующие `final_trajectory_samples` остаются совместимыми.

15. Провести механическую миграцию без изменения PX4 call site.

    Файлы:
    - `drone_city_nav/src/px4_offboard_node.cpp:598`;
    - `drone_city_nav/src/px4_offboard_node.cpp:1282`.

    Ожидаемый результат:
    - `px4_offboard_node` продолжает:
      - строить `trajectory_speed_profile_` при новой trajectory;
      - вызывать один фасадный `planVelocitySetpoint()`;
      - публиковать PX4 `TrajectorySetpoint` с velocity/acceleration.
    - архитектурный refactor не меняет ROS topics и PX4 message contract.

16. Зафиксировать отсутствие ручной симуляционной зависимости в unit-level
    проверках.

    Файлы:
    - `README.md` менять не требуется;
    - `PLAN.md` фиксирует verification strategy.

    Ожидаемый результат:
    - корректность новых модулей проверяется через C++/script tests;
    - headless simulation остаётся дополнительной e2e-проверкой, а не
      единственным способом увидеть проблему.

# Verification plan

После реализации:

1. Форматирование изменённых C++ файлов:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Минимальная C++ проверка новых/изменённых tests после build:

   ```bash
   ./scripts/dev_shell.sh make test
   ```

   Внутри `ctest` ожидаемые test targets:
   - `trajectory_speed_planner_test`;
   - `velocity_command_planner_test`;
   - `velocity_smoother_test`;
   - `offboard_velocity_follower_test`;
   - `px4_offboard_config_test`.

3. Script contract tests:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

4. Полная quality gate перед commit:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

5. Whitespace check:

   ```bash
   git diff --check
   ```

6. E2E/headless smoke, если в конкретной implementation-итерации пользователь
   явно разрешит/попросит симуляционный прогон:

   ```bash
   ./scripts/sim_headless.sh
   ```

   Headless run не должен заменять unit tests. Он нужен для проверки, что в
   реальном PX4/Gazebo loop новые logs/blackbox достаточно объясняют скорость,
   cross-track, acceleration и jerk.

# Testing strategy

Категория 1: без рефакторинга внешнего поведения.

- `trajectory_speed_planner_test`: pure scalar speed profile, no ROS/PX4.
- `velocity_command_planner_test`: pure desired velocity/cross-track vector.
- `velocity_smoother_test`: pure acceleration/jerk/vector limiting.
- Цель: проверить математику и edge cases без simulator.

Категория 2: лёгкий integration/refactor.

- `offboard_velocity_follower_test`: фасад `planVelocitySetpoint()` связывает
  scalar speed, command planner и smoother в прежний `VelocitySetpointPlan`.
- `px4_offboard_config_test`: YAML параметры доступны и зафиксированы.
- `test_offboard_telemetry_contract.py`: runtime logs/blackbox контракт
  содержит все speed stages.

Категория 3: тяжёлый/system scope.

- `make quality`: build + C++ tests + clang-tidy/cppcheck для изменённых C++
  файлов.
- `make test-scripts`: Python/script contracts.
- `./scripts/sim_headless.sh`: только по явному разрешению на simulation run.
  Проверять:
  - mission completion;
  - max/mean speed;
  - max cross-track error;
  - tilt peaks;
  - `lookahead_speed_limit_mps`, `cross_track_limited_speed_mps`,
    `final_command_speed_mps` в `log/offboard_blackbox.jsonl`.

# Risks and tradeoffs

1. Поведенческая регрессия скорости.
   - Lookahead может сделать дрон осторожнее, если `lookahead_max_m` слишком
     большой.
   - Проверка: unit tests на lookahead clamp + headless blackbox speed stages.

2. Дублирование или рассинхронизация config.
   - Если разделить `VelocityFollowerConfig` на несколько config structs,
     можно забыть прокинуть параметр из `px4_offboard_node`.
   - Проверка: `px4_offboard_config_test` и script telemetry contract.

3. Потеря диагностических полей.
   - При refactor легко потерять старые поля `VelocitySetpointPlan`, которыми
     уже пользуются logs/blackbox.
   - Проверка: `test_offboard_telemetry_contract.py`.

4. Ошибка в stationing/lookahead.
   - Неверный `s_m` или `lower_bound` может брать speed limit не из будущего
     участка.
   - Проверка: `trajectory_speed_planner_test` с несколькими constraints и
     точными expected distances.

5. Производительность.
   - Runtime lookahead не должен сканировать весь profile на каждом 20 Hz tick
     при длинных траекториях.
   - Решение в реализации: использовать bounded scan от текущего sample index
     до `s + lookahead`, либо binary search + локальный проход.
   - Проверка: unit test на корректность; в headless blackbox смотреть absence
     of controller stalls по косвенным признакам.

6. API churn.
   - Новые headers/modules увеличат количество типов.
   - Tradeoff принят: явные доменные границы важнее короткого одного файла,
     потому что текущий `planVelocitySetpoint()` уже смешивает несколько
     причин изменения скорости.

# Open questions

1. Блокирующих вопросов нет.
2. Значения `speed_profile_lookahead_time_s/min/max` могут потребовать
   дальнейшей настройки по headless логам, но начальные значения достаточно
   конкретны для реализации и автотестов.
3. Частоту controller loop (`50 ms`, 20 Hz) на этом этапе не менять. Сначала
   нужно отделить и проверить математику speed/vector/smoothing. Повышение до
   50-100 Hz имеет смысл планировать только после логового подтверждения, что
   текущая частота стала главным ограничителем.
