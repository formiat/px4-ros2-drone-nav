# Context

Планируем только этап 5 фичи `3D passage traversal`: offboard должен реально следовать вертикальному профилю `z(s)` уже построенной 3D trajectory. Распознавание passages, локальная 3D-вставка и изменение 2D XY-логики в этот этап не входят.

Текущий pipeline уже подготовил часть входных данных для этого этапа:

- `TrajectoryPointSample` хранит `z_m`, `vertical_slope_dz_ds`, vertical speed/accel/jerk caps, `vertical_constraint_active` и `vertical_profile_passage_id`: `drone_city_nav/include/drone_city_nav/trajectory.hpp:55`.
- Planner применяет vertical profile и known-passage validation перед speed profile: `drone_city_nav/src/trajectory_planner.cpp:101`.
- Speed profile уже учитывает vertical profile caps как `SpeedConstraintType::kVerticalProfile`: `drone_city_nav/src/trajectory_speed_planner.cpp:130`, `drone_city_nav/src/trajectory_speed_planner.cpp:160`.
- Offboard получает `z` из `nav_msgs/Path` и сохраняет его в samples: `drone_city_nav/src/offboard_trajectory_state.cpp:54`.
- Runtime vertical control сейчас слишком простой: берёт только `target z` по `trajectory_s_m`, делает P-feedback и отдаёт `vz` в PX4 NED: `drone_city_nav/src/px4_offboard_node_control.cpp:272`, `drone_city_nav/src/px4_offboard_node_control.cpp:285`.

Цель этапа: добавить отдельный vertical follower, который считает `target_z`, `target_vz`, altitude feedback, vertical smoothing/limits и пишет детальные blackbox/telemetry поля, не трогая XY lateral controller.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на локальном коде, repo docs, `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, `Makefile`, orchestrator protocols и профилях `generic`/`cpp`.

# Detected stack/profiles

- Стек: ROS 2 C++ workspace, пакет `drone_city_nav`, сборка через `colcon`, тесты через `ctest` и Python script tests.
- Прочитанные project profiles:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md` - обязательный общий профиль.
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md` - применим, потому что workspace содержит C++ source/header files, `CMakeLists.txt`, `package.xml`, `colcon` workflow.
- Notion policy `optional`; Notion task в prompt не указан, поэтому Notion чтение не требуется.
- GitLab/MR в prompt не указан, поэтому GitLab CLI не используется.

# Repo-approved commands found

Из `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, `Makefile`:

- Снаружи контейнера:
  - `./scripts/build.sh`
  - `./scripts/test.sh`
  - `./scripts/sim_gui.sh`
  - `./scripts/sim_headless.sh`
  - `./scripts/stop_sim.sh`
  - `./scripts/dev_shell.sh`
- Внутри контейнера:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
  - `make sim-gui`
  - `make sim-headless`
- Build system entrypoint: `colcon`, не ad-hoc top-level CMake.

# Affected components

- Vertical runtime control:
  - `drone_city_nav/src/px4_offboard_node_control.cpp:272`
  - `drone_city_nav/src/px4_offboard_node_control.cpp:285`
  - `drone_city_nav/src/px4_offboard_node_control.cpp:299`
  - `drone_city_nav/src/px4_offboard_node.hpp:211`
- Trajectory sample helpers:
  - `drone_city_nav/include/drone_city_nav/trajectory.hpp:55`
  - `drone_city_nav/src/trajectory.cpp:278`
- PX4 setpoint IO and NED sign convention:
  - `drone_city_nav/src/px4_offboard_setpoint_io.cpp:49`
  - `drone_city_nav/src/px4_offboard_setpoint_io.cpp:67`
- Offboard config:
  - `drone_city_nav/include/drone_city_nav/px4_offboard_node_config.hpp:33`
  - `drone_city_nav/src/px4_offboard_node_config.cpp:300`
  - `drone_city_nav/config/urban_mvp.yaml:248`
- Runtime diagnostics and blackbox:
  - `drone_city_nav/src/px4_offboard_node_telemetry.cpp:130`
  - `drone_city_nav/include/drone_city_nav/offboard_blackbox.hpp:48`
  - `drone_city_nav/src/offboard_blackbox.cpp:92`
  - `drone_city_nav/tests/offboard_blackbox_test.cpp:358`
- Build/test registration:
  - `drone_city_nav/CMakeLists.txt:39`
  - `drone_city_nav/CMakeLists.txt:492`

# Implementation steps

1. Добавить отдельный модуль vertical follower.

   Файлы:
   - добавить `drone_city_nav/include/drone_city_nav/offboard_vertical_follower.hpp`;
   - добавить `drone_city_nav/src/offboard_vertical_follower.cpp`;
   - подключить `src/offboard_vertical_follower.cpp` в `drone_city_nav/CMakeLists.txt` рядом с `src/offboard_velocity_follower.cpp` (`drone_city_nav/CMakeLists.txt:52`).

   Материализуемый результат: появляется чистый unit-testable модуль без ROS/PX4 зависимостей. Он принимает trajectory samples, `trajectory_s_m`, текущую высоту, `dt_s`, scalar speed из XY planner-а и возвращает vertical plan.

   Минимальный контракт:

   ```cpp
   struct VerticalFollowerConfig {
     double altitude_feedback_kp_1ps{0.5};
     double max_vertical_speed_mps{2.0};
     double max_vertical_accel_mps2{2.0};
     double max_vertical_jerk_mps3{6.0};
     double target_vz_feedforward_scale{1.0};
   };

   struct VerticalFollowerState {
     bool previous_command_valid{false};
     double previous_commanded_vz_mps{0.0};      // ENU/up-positive
     double previous_vertical_accel_mps2{0.0};
   };

   struct VerticalSetpointPlan {
     bool valid{false};
     bool trajectory_target_valid{false};
     bool passage_mode{false};
     double target_z_m{NaN};
     double actual_z_m{NaN};
     double z_error_m{NaN};
     double vertical_slope_dz_ds{0.0};
     double target_vz_mps{0.0};                  // ENU/up-positive feedforward
     double feedback_vz_mps{0.0};                // ENU/up-positive feedback
     double desired_vz_mps{0.0};                 // before limits, ENU/up-positive
     double commanded_vz_mps{0.0};               // after limits, ENU/up-positive
     double commanded_vz_ned_mps{0.0};           // PX4 setpoint convention
     std::string passage_id;
   };
   ```

   Нетривиальная логика:

   ```cpp
   target_vz = vertical_slope_dz_ds * std::max(0.0, scalar_speed_mps);
   feedback_vz = clamp((target_z - actual_z) * kp, -max_vz, max_vz);
   desired_vz = clamp(target_vz * feedforward_scale + feedback_vz, -max_vz, max_vz);
   commanded_vz = applyVerticalAccelAndJerkLimits(desired_vz, state, dt_s, config);
   commanded_vz_ned = -commanded_vz;
   ```

   Важно: этот модуль не должен менять `vx/vy`, tangent, cross-track feedback, curvature feedforward или velocity smoother для XY.

2. Добавить runtime helper для vertical target at `s`.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/trajectory.hpp`;
   - `drone_city_nav/src/trajectory.cpp`.

   Code anchors:
   - существующий `TrajectoryPointSample`: `drone_city_nav/include/drone_city_nav/trajectory.hpp:55`;
   - существующий `trajectorySampleAltitudeAtS()`: `drone_city_nav/src/trajectory.cpp:278`.

   Материализуемый результат: helper возвращает не только `z`, но и slope/limits/passage metadata, интерполированные между соседними samples.

   Предлагаемый тип:

   ```cpp
   struct TrajectoryVerticalTarget {
     bool valid{false};
     double s_m{0.0};
     double z_m{NaN};
     double vertical_slope_dz_ds{0.0};
     bool vertical_constraint_active{false};
     std::string vertical_profile_passage_id;
   };

   [[nodiscard]] TrajectoryVerticalTarget
   trajectoryVerticalTargetAtS(std::span<const TrajectoryPointSample> samples, double s_m);
   ```

   Правила:
   - `z_m` и `vertical_slope_dz_ds` интерполировать по `s`;
   - `vertical_constraint_active` считать true, если активен любой из соседних samples в интервале;
   - `vertical_profile_passage_id` брать у ближайшего active sample с непустым id;
   - при невалидных samples вернуть `valid=false`.

3. Интегрировать vertical follower в velocity setpoint publication.

   Файлы:
   - `drone_city_nav/src/px4_offboard_node_control.cpp`;
   - `drone_city_nav/src/px4_offboard_node.hpp`.

   Code anchors:
   - заменить текущий `targetAltitudeForCurrentTrajectory()`: `drone_city_nav/src/px4_offboard_node_control.cpp:272`;
   - заменить текущий `verticalVelocitySetpointNed()`: `drone_city_nav/src/px4_offboard_node_control.cpp:285`;
   - вызвать новый vertical follower из `publishVelocityTrajectorySetpoint()`: `drone_city_nav/src/px4_offboard_node_control.cpp:299`.

   Материализуемый результат:
   - `publishVelocityTrajectorySetpoint()` продолжает получать `plan.velocity_xy` из существующего XY follower;
   - `vz_ned` берётся из `VerticalSetpointPlan::commanded_vz_ned_mps`;
   - `buildVelocityTrajectorySetpoint()` остаётся единой точкой сборки `vx/vy/vz`: `drone_city_nav/src/px4_offboard_setpoint_io.cpp:67`;
   - все старые diagnostic поля `last_target_altitude_m_`, `last_altitude_error_m_`, `last_vertical_velocity_setpoint_mps_` обновляются из vertical plan, чтобы не ломать существующие логи сразу.

   В `Px4OffboardNode` добавить:
   - `VerticalFollowerConfig vertical_follower_config_`;
   - `VerticalFollowerState vertical_follower_state_`;
   - `VerticalSetpointPlan last_vertical_plan_`;
   - `bool last_vertical_plan_valid_`.

4. Сброс vertical follower state при реальном discontinuity.

   Файлы:
   - `drone_city_nav/src/px4_offboard_node_control.cpp`;
   - `drone_city_nav/src/px4_offboard_node_trajectory.cpp`;
   - `drone_city_nav/src/trajectory_update_continuity.cpp` при необходимости только для wiring reason.

   Материализуемый результат:
   - если trajectory continuity decision сохраняет smoother state, сохраняем и vertical follower state;
   - если trajectory update требует reset, invalid path, pose stale reset, terminal state reset или no-path hold, сбрасываем `VerticalFollowerState`;
   - telemetry/blackbox получает `vertical_reset_reason` или хотя бы reuse существующего smoother reset reason.

   Это важно, чтобы вертикальный limiter не создавал скачок `vz` на валидном compatible path update, но не тянул history через несовместимую траекторию.

5. Сохранить terminal position capture и перевести его на trajectory goal altitude.

   Файлы:
   - `drone_city_nav/src/px4_offboard_node_control.cpp`;
   - `drone_city_nav/src/px4_offboard_setpoint_io.cpp` при необходимости расширения имени аргумента;
   - `drone_city_nav/src/px4_offboard_node.hpp`.

   Code anchors:
   - terminal/position setpoint сейчас задаёт `cruise_altitude_m_`: `drone_city_nav/src/px4_offboard_node_control.cpp:82`;
   - `buildPositionTrajectorySetpoint()` пишет PX4 NED altitude как `-abs(cruise_altitude_m)`: `drone_city_nav/src/px4_offboard_setpoint_io.cpp:49`.

   Материализуемый результат:
   - position setpoint mode остаётся обязательным финальным режимом;
   - target altitude для terminal position capture становится `finalTrajectoryGoalAltitudeM()`:
     - если `final_trajectory_samples_` valid и back().`z_m` finite - использовать его;
     - иначе fallback на `cruise_altitude_m_`;
   - не менять state machine terminal semantics, только высоту target point.

6. Развести config для vertical profile construction и runtime vertical following.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/px4_offboard_node_config.hpp:33`;
   - `drone_city_nav/src/px4_offboard_node_config.cpp:300`;
   - `drone_city_nav/config/urban_mvp.yaml:248`;
   - `drone_city_nav/tests/px4_offboard_node_config_test.cpp:131`;
   - `drone_city_nav/src/velocity_control_config.cpp` и `drone_city_nav/tests/velocity_control_config_test.cpp`, если runtime fingerprint должен учитывать новые параметры.

   Материализуемый результат:
   - оставить `vertical_profile_*` как construction/profile-speed параметры;
   - добавить runtime параметры:
     - `altitude_feedback_kp_1ps`;
     - `vertical_setpoint_max_speed_mps`;
     - `vertical_setpoint_max_accel_mps2`;
     - `vertical_setpoint_max_jerk_mps3`;
     - `vertical_target_vz_feedforward_scale`;
   - временно сохранить `altitude_hold_kp` и `max_vertical_speed_mps` как backward-compatible aliases, если YAML/launch/tests ещё используют их;
   - новые имена должны быть в YAML и tests, чтобы публичная config-семантика была понятной.

7. Расширить telemetry и blackbox для headless debugging.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/offboard_blackbox.hpp:48`;
   - `drone_city_nav/src/offboard_blackbox.cpp:92`;
   - `drone_city_nav/src/px4_offboard_node_telemetry.cpp:130`;
   - `drone_city_nav/tests/offboard_blackbox_test.cpp:358`;
   - `scripts/tests/test_offboard_telemetry_contract.py`, если контракт перечисляет telemetry/blackbox fields.

   Материализуемый результат: в blackbox и log line появляются обязательные поля:

   - `target_z_m`;
   - `actual_z_m`;
   - `z_error_m`;
   - `target_vz_mps`;
   - `feedback_vz_mps`;
   - `desired_vz_mps`;
   - `commanded_vz_mps`;
   - `commanded_vz_ned_mps`;
   - `passage_mode`;
   - `passage_id`;
   - `vertical_slope_dz_ds`;
   - `vertical_constraint_active`.

   Старые поля `target_altitude_m`, `altitude_error_m`, `vertical_velocity_setpoint_mps` можно оставить на один переходный шаг как aliases, но новые tests должны проверять новые имена.

8. Определить `passage_mode` без отдельного perception слоя.

   Файлы:
   - `drone_city_nav/src/offboard_vertical_follower.cpp`;
   - `drone_city_nav/include/drone_city_nav/trajectory.hpp`;
   - `drone_city_nav/src/trajectory.cpp`;
   - `drone_city_nav/tests/offboard_vertical_follower_test.cpp`.

   Материализуемый результат:
   - `passage_mode=true`, если vertical target в текущем `s` попал в интервал, где `vertical_constraint_active=true` или есть `vertical_profile_passage_id`;
   - `passage_id` берётся из ближайшего sample;
   - для избежания flicker допускается маленький hysteresis/window в module state, но только если это покрыто unit tests.

   В этот этап не входит изменение lidar/memory policy для known passage walls; это отдельный будущий этап `Passage traversal mode / sensor policy`.

9. Обновить CMake и добавить unit tests.

   Файлы:
   - `drone_city_nav/CMakeLists.txt`;
   - добавить `drone_city_nav/tests/offboard_vertical_follower_test.cpp`;
   - обновить `drone_city_nav/tests/offboard_blackbox_test.cpp`;
   - обновить `drone_city_nav/tests/px4_offboard_node_config_test.cpp`;
   - при добавлении `trajectoryVerticalTargetAtS()` обновить `drone_city_nav/tests/trajectory_test.cpp`;
   - при изменении telemetry contract обновить `scripts/tests/test_offboard_telemetry_contract.py`.

   Материализуемый результат: новый test target `offboard_vertical_follower_test` зарегистрирован рядом с `offboard_velocity_follower_test` (`drone_city_nav/CMakeLists.txt:446`) или как отдельный `ament_add_gtest`.

10. Проверить, что final trajectory samples и offboard debug остаются совместимыми.

   Файлы:
   - `drone_city_nav/src/offboard_trajectory_state.cpp:54`;
   - `drone_city_nav/tests/offboard_trajectory_state_test.cpp:89`;
   - `drone_city_nav/src/final_trajectory_debug_io.cpp`, если CSV schema проверяет vertical fields.

   Материализуемый результат:
   - path samples из ROS message сохраняют altitude;
   - после получения path offboard имеет usable samples с `z_m`;
   - если slope/vertical metadata в Path не передаются как отдельные поля, offboard пересчитывает runtime vertical target slope из `z_m` через helper или уже существующий `populateTrajectoryVerticalSpeedConstraints()`.

# Verification plan

Для реализации этапа:

1. Форматирование C++ изменений:
   - `./scripts/dev_shell.sh`
   - внутри контейнера: `make format`

2. Scoped C++ tests:
   - `./scripts/dev_shell.sh`
   - внутри контейнера:
     ```bash
     make build
     ctest --test-dir build/drone_city_nav --output-on-failure -R 'offboard_vertical_follower|trajectory_test|offboard_trajectory_state|offboard_blackbox|px4_offboard_node_config|velocity_control_config|trajectory_speed_planner'
     ```

3. Script contracts:
   - внутри контейнера: `make test-scripts`

4. Broad quality gate before commit:
   - внутри контейнера: `make quality`

5. Headless integration signal after unit/script tests pass:
   - `./scripts/sim_headless.sh`
   - проверить `log/offboard_blackbox.jsonl` на наличие `target_z_m`, `actual_z_m`, `target_vz_mps`, `commanded_vz_mps`, `passage_mode`;
   - для обычной constant-altitude карты ожидать `target_vz_mps ~= 0`, `passage_mode=false`, отсутствие ухудшения XY tracking.

# Testing strategy

1. Без рефакторинга / pure behavior tests.

   - `offboard_vertical_follower_test`:
     - constant altitude: `target_vz=0`, при нулевой ошибке `commanded_vz=0`;
     - ramp profile: `vertical_slope_dz_ds=0.1`, scalar speed `10 m/s` даёт `target_vz=1 m/s`;
     - altitude error feedback складывается с feedforward;
     - speed clamp ограничивает `desired_vz`;
     - invalid altitude или invalid samples дают safe `commanded_vz=0` и диагностический invalid reason;
     - NED sign: climb up-positive `commanded_vz_mps > 0` превращается в `commanded_vz_ned_mps < 0`.

2. Лёгкий integration/module scope.

   - `trajectory_test`: интерполяция `trajectoryVerticalTargetAtS()` по `z`, slope, passage id, active flag.
   - `offboard_trajectory_state_test`: Path с разной `pose.position.z` превращается в usable trajectory samples.
   - `offboard_blackbox_test`: JSON содержит новые altitude control fields и экранирует `passage_id`.
   - `px4_offboard_node_config_test`: новые runtime vertical params читаются, sanitize clamps работают, legacy aliases не ломают существующий YAML.
   - `velocity_control_config_test`: runtime fingerprint меняется при изменении runtime vertical following params, если они включены в fingerprint.

3. Тяжёлый / системный scope.

   - `make test` и `make test-scripts` после scoped tests.
   - `make quality` перед commit.
   - `sim_headless` как интеграционный smoke для headless-отладки. Это не заменяет unit tests, но проверяет реальный PX4 setpoint path и blackbox output.

# Risks and tradeoffs

- Control-риск: добавление target-vz feedforward и vertical smoothing может изменить высотную динамику даже на constant-altitude runs, если state reset или sign convention ошибочны. Проверка: unit tests на NED sign, constant altitude, invalid samples; headless blackbox `z_error_m` и `commanded_vz_mps`.
- API/контракт-риск: blackbox/telemetry schema изменится. Проверка: `offboard_blackbox_test`, script telemetry contract, backward-compatible aliases на переходный шаг.
- Config-риск: старые `altitude_hold_kp` / `max_vertical_speed_mps` могут конфликтовать с новыми `vertical_setpoint_*`. Проверка: config tests и явный priority rule: новые имена override legacy aliases.
- Интеграционный риск: terminal position capture должен остаться position setpoint и не потерять стабилизацию финиша. Проверка: unit-level target altitude helper и headless run на финал.
- Производительность: vertical follower должен быть O(локальный поиск/interpolation) и не должен добавлять заметный runtime overhead. Если helper делает linear scan по samples каждый tick, это приемлемо для текущих десятков/сотен samples, но при росте samples стоит кэшировать индекс по `trajectory_s_m`.
- Разделение ответственности: этот этап не решает lidar policy внутри known passages. Если lidar/memory будет считать ожидаемые стены obstacle-ами, полёт через passage может быть прерван replanning-ом. Это отдельный следующий этап.

# Open questions

- `passage_mode` должен быть true только внутри opening/vertical constraint или на всём approach-entry-exit-reconnect окне? Для этого этапа предлагается включать по `vertical_constraint_active || passage_id`, а при необходимости добавить hysteresis.
- Для `target_vz = dz/ds * speed` брать `plan.speed_mps`, `plan.accel_limited_speed_mps` или actual tangential speed? Для первого implementation шага предлагается `plan.accel_limited_speed_mps`, потому что это уже ограниченная scalar speed command, до финального XY smoother. В blackbox дополнительно логировать выбранный scalar speed.
- Нужно ли сохранять старые blackbox field names как aliases навсегда или удалить после обновления scripts/docs? Для безопасной миграции предлагается оставить aliases на один этап и отметить новые поля canonical.
- Нужно ли terminal position capture целиться в final sample altitude всегда или только если trajectory altitude target valid в последнем velocity plan? Предлагается использовать final sample altitude, потому что position capture должен дожимать конечную 3D-точку даже после выхода из velocity follower.
