# Context

Нужно заменить текущую пороговую логику ограничения скорости перед поворотами на
непрерывный speed profile перед всеми будущими ограничениями пути: каждым углом и
финальной точкой. Цель этого плана - не скругление поворотов и не добавление новых
waypoint'ов, а расчёт целевой скорости на текущем control tick так, чтобы дрон
успевал прийти к любому upcoming constraint с допустимой скоростью.

Текущее управление уже работает в Offboard velocity mode: `px4_offboard_node`
публикует `TrajectorySetpoint.velocity`, а `offboard_velocity_follower` считает
модуль и направление velocity setpoint вдоль текущего сегмента пути. Это нужно
сохранить: направление движения остаётся отдельным от расчёта модуля скорости.

# Investigation context

`INVESTIGATION.md` отсутствует в workspace, поэтому входных данных от отдельного
исследовательского артефакта нет.

По локальному коду обнаружено:

- `VelocityFollowerConfig` сейчас хранит `turn_slowdown_min_angle_rad`,
  `sharp_turn_angle_rad` и `braking_margin_m` в
  `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:21`.
- `targetTurnSpeedMps()` в
  `drone_city_nav/src/offboard_velocity_follower.cpp:162` использует линейную
  интерполяцию между `turn_slowdown_min_angle_rad` и `sharp_turn_angle_rad`.
  Углы меньше `turn_slowdown_min_angle_rad` вообще не ограничивают скорость.
- `speedLimitForUpcomingTurn()` в
  `drone_city_nav/src/offboard_velocity_follower.cpp:253` возвращает первый
  найденный поворот в preview horizon, а не самый строгий speed constraint.
- `speedLimitForFinalStop()` в
  `drone_city_nav/src/offboard_velocity_follower.cpp:310` отдельно обрабатывает
  финальную точку и ограничивает скорость только на последнем сегменте.
- `planVelocitySetpoint()` в
  `drone_city_nav/src/offboard_velocity_follower.cpp:387` берёт минимум между
  turn-limit и final-stop-limit, затем применяет rate limit через
  `max_accel_mps2` / `max_decel_mps2`.
- `px4_offboard_node` загружает параметры velocity follower в
  `drone_city_nav/src/px4_offboard_node.cpp:198` и логирует их в
  `drone_city_nav/src/px4_offboard_node.cpp:338`.
- Runtime-диагностика скорости уже есть в
  `drone_city_nav/src/px4_offboard_node.cpp:1765`, а JSONL blackbox пишет
  `velocity_command` в `drone_city_nav/src/px4_offboard_node.cpp:1887`.
- Unit-тесты follower'а находятся в
  `drone_city_nav/tests/offboard_velocity_follower_test.cpp:13`.

# Detected stack/profiles

- Основной stack: ROS 2 workspace, C++ package `drone_city_nav`, ament CMake,
  сборка через `colcon`.
- Обнаружены `drone_city_nav/CMakeLists.txt`, top-level `Makefile`,
  `.clang-format`, существующие `build/compile_commands.json` и
  `build/drone_city_nav/compile_commands.json`.
- Rust stack не обнаружен в target workspace: `Cargo.toml` / `Cargo.lock` нет.
- Прочитанные обязательные профили:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- Прочитанные обязательные протоколы:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- Notion/GitLab не читались: prompt не содержит Notion task id, GitLab MR или
  GitLab review context, а `notion_policy=optional`.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md` и `Makefile`:

- Build: `./scripts/build.sh` или внутри контейнера `make build`.
- Tests: `./scripts/test.sh` или внутри контейнера `make test`.
- Script tests: внутри контейнера `make test-scripts`.
- Quality: внутри контейнера `make quality`.
- C++ formatting for changed C++ files: внутри контейнера `make format`.
- GUI simulation: `./scripts/sim_gui.sh`.
- Headless simulation: `./scripts/sim_headless.sh`.
- Stop simulation leftovers: `./scripts/stop_sim.sh`.

Для этой задачи при реализации нужно использовать только container workflow.
Симуляционные прогоны запускать только по отдельной явной команде пользователя.

# Affected components

- `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp`
  - конфиг velocity follower;
  - структуры diagnostic output для turn/goal constraints.
- `drone_city_nav/src/offboard_velocity_follower.cpp`
  - геометрия поворотов;
  - расчёт speed constraints;
  - выбор самого строгого ограничения;
  - rate-limited target speed.
- `drone_city_nav/src/px4_offboard_node.cpp`
  - загрузка новых параметров;
  - startup log;
  - 2 Hz telemetry log;
  - `log/offboard_blackbox.jsonl`.
- `drone_city_nav/config/urban_mvp.yaml`
  - параметры новой модели торможения.
- `drone_city_nav/tests/offboard_velocity_follower_test.cpp`
  - unit coverage speed profile / constraints / edge cases.
- `scripts/tests/test_offboard_telemetry_contract.py`
  - contract coverage для telemetry/blackbox полей в
    `drone_city_nav/src/px4_offboard_node.cpp`.

# Implementation steps

1. Обновить публичный контракт velocity follower в
   `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:21`.
   Результат: конфиг перестаёт описывать "sharp turn slowdown" как основную
   модель и получает параметры физической модели pre-turn speed profile.

   Конкретно:
   - оставить `cruise_speed_mps`, `min_turn_speed_mps`,
     `max_accel_mps2`, `max_decel_mps2`, `max_lateral_accel_mps2`,
     `turn_preview_distance_m`, `braking_margin_m`;
   - заменить или депрецировать внутри кода `turn_slowdown_min_angle_rad` и
     `sharp_turn_angle_rad` так, чтобы они не управляли основной формулой;
   - добавить параметр виртуального радиуса/скругления для расчёта скорости
     поворота, например `turn_radius_base_m` или
     `turn_corner_cut_distance_m`;
   - расширить `TurnSpeedPlan` полями:
     `turn_radius_m`, `corner_cut_distance_m`, `constraint_speed_mps`,
     `allowed_speed_now_mps`, `constraint_type`.
   - синхронизировать public declarations и definitions для
     `speedLimitForUpcomingTurn()` и `speedLimitForFinalStop()`:
     сейчас header объявляет параметр `current_speed_mps` в
     `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:89`,
     а definitions в
     `drone_city_nav/src/offboard_velocity_follower.cpp:253` и
     `drone_city_nav/src/offboard_velocity_follower.cpp:310` его не принимают.
     Реализация должна либо удалить stale parameter из header, либо добавить
     его в definitions и реально использовать; unit tests должны вызывать
     согласованную сигнатуру.

   Псевдоконтракт:

   ```cpp
   struct TurnSpeedPlan {
     bool valid{false};
     std::size_t waypoint_index{0U};
     double angle_rad{0.0};
     double distance_to_turn_m{infinity};
     double turn_radius_m{quiet_NaN};
     double constraint_speed_mps{quiet_NaN};
     double raw_speed_limit_mps{quiet_NaN};
   };
   ```

2. Переписать расчёт скорости поворота в
   `drone_city_nav/src/offboard_velocity_follower.cpp:162`.
   Результат: вместо линейной интерполяции по порогам угла появится физическая
   формула допустимой скорости у поворота.

   Базовая формула:

   ```cpp
   severity = std::sin(angle_rad / 2.0);
   radius = turn_radius_base_m / std::max(severity, epsilon);
   v_turn = std::sqrt(max_lateral_accel_mps2 * radius);
   v_turn = std::clamp(v_turn, min_turn_speed_mps, cruise_speed_mps);
   ```

   Требования:
   - очень малые углы не должны давать NaN/Inf;
   - мягкий угол может фактически оставить `v_turn == cruise_speed_mps`;
   - большой угол должен снижать `v_turn`;
   - расчёт не должен менять направление движения или добавлять точки пути.

3. Переписать `speedLimitForUpcomingTurn()` в
   `drone_city_nav/src/offboard_velocity_follower.cpp:253`.
   Результат: функция просматривает все повороты в пределах
   `turn_preview_distance_m`, считает для каждого текущую допустимую скорость
   перед constraint и возвращает самый строгий constraint, а не первый поворот.

   Псевдологика:

   ```cpp
   best.raw_speed_limit_mps = cruise_speed;
   for each turn inside preview:
     angle = turnAngleRad(...)
     distance_to_turn = distanceFromProjectionToWaypoint(...)
     v_constraint = targetTurnSpeedMps(angle, config)
     v_allowed_now = speedLimitBeforeConstraint(
         distance_to_turn, v_constraint, cruise_speed, max_decel, braking_margin)
     if v_allowed_now < best.raw_speed_limit_mps:
       best = this turn
   return best if any turn was considered
   ```

   Важно: constraint может быть валидным и для углов меньше старого
   `turn_slowdown_min_angle_rad`; порог можно оставить только как числовой
   epsilon против шума, не как поведенческий cutoff.

4. Унифицировать финальную точку с той же моделью в
   `speedLimitForFinalStop()` в
   `drone_city_nav/src/offboard_velocity_follower.cpp:310`.
   Результат: финиш рассматривается как speed constraint с
   `constraint_speed_mps = 0.0`, но без изменения геометрии пути.

   Псевдологика:

   ```cpp
   distance_to_goal = distanceFromProjectionToWaypoint(path, projection, last_index);
   effective_distance = max(0.0, distance_to_goal - final_acceptance_radius_m);
   raw_speed_limit = speedLimitBeforeConstraint(
       effective_distance, 0.0, cruise_speed, max_decel, braking_margin)
   ```

   Поведение перед goal и перед обычным поворотом должно отличаться только
   `constraint_speed_mps`, а не отдельной ручной веткой торможения.

5. Сохранить разделение "направление отдельно, скорость отдельно" в
   `planVelocitySetpoint()` в
   `drone_city_nav/src/offboard_velocity_follower.cpp:420`.
   Результат: `raw_speed_limit` считается как минимум по всем turn/goal
   constraints, затем применяется существующий rate limit через
   `max_accel_mps2` / `max_decel_mps2`, после чего velocity vector направляется
   вдоль текущего сегмента плюс bounded cross-track correction.

   Нельзя в этой задаче:
   - добавлять waypoint'ы;
   - возвращать lookahead;
   - строить дуги;
   - менять A* path smoothing.

6. Обновить причины и диагностику velocity follower в
   `drone_city_nav/src/offboard_velocity_follower.cpp:199` и
   `drone_city_nav/include/drone_city_nav/offboard_velocity_follower.hpp:12`.
   Результат: `VelocitySetpointReason` отражает новую модель, например
   `kCruise`, `kTurnConstraint`, `kGoalConstraint` или сохраняет старые имена,
   но `reasonFromTurnPlan()` больше не должен классифицировать поведение через
   old sharp/gentle thresholds.

7. Обновить загрузку параметров и startup log в
   `drone_city_nav/src/px4_offboard_node.cpp:198` и
   `drone_city_nav/src/px4_offboard_node.cpp:338`.
   Результат:
   - новый параметр радиуса/геометрии читается из ROS params;
   - старые `turn_slowdown_min_angle_deg` / `sharp_turn_angle_deg` либо
     удалены из активного пути, либо оставлены только если принято решение
     сохранить совместимость с YAML;
   - startup log печатает параметры физической модели:
     `turn_preview_distance`, `turn_radius_base`, `max_lateral_accel`,
     `max_decel`, `braking_margin`, `min_turn_speed`, `cruise_speed`.

8. Обновить runtime telemetry log в
   `drone_city_nav/src/px4_offboard_node.cpp:1765`.
   Результат: 2 Hz лог показывает, какой constraint ограничил скорость и какие
   числа дали итоговый speed limit.

   Добавить поля:
   - `limiting_constraint_type` (`none`, `turn`, `goal`);
   - `limiting_constraint_index`;
   - `limiting_constraint_distance_m`;
   - `limiting_turn_angle_rad`;
   - `limiting_turn_radius_m`;
   - `limiting_constraint_speed_mps`;
   - `limiting_allowed_speed_now_mps`;
   - `raw_speed_limit_mps`;
   - `accel_limited_speed_mps`;
   - `velocity_setpoint_speed_mps`.

9. Обновить JSONL blackbox в
   `drone_city_nav/src/px4_offboard_node.cpp:1887`.
   Результат: headless-отладка может восстановить, почему дрон начал или не
   начал тормозить перед конкретным углом/goal.

   Все новые числовые поля писать через `writeJsonNumberOrNull()`.
   Добавить JSON поля в `velocity_command`, например:

   ```json
   {
     "limiting_constraint_type": "turn",
     "limiting_constraint_index": 2,
     "limiting_constraint_distance_m": 31.5,
     "limiting_turn_angle_rad": 0.52,
     "limiting_turn_radius_m": 38.0,
     "limiting_constraint_speed_mps": 15.1,
     "limiting_allowed_speed_now_mps": 19.4
   }
   ```

10. Обновить `drone_city_nav/config/urban_mvp.yaml:120`.
    Результат: YAML содержит параметры новой физической модели и не вводит в
    заблуждение названиями `sharp_turn`.

    Предлагаемые параметры для первого implementation pass:

    ```yaml
    turn_preview_distance_m: 90.0
    turn_radius_base_m: 10.0
    cruise_speed_mps: 22.0
    min_turn_speed_mps: 1.5
    max_accel_mps2: 7.0
    max_decel_mps2: 8.0
    max_lateral_accel_mps2: 6.0
    braking_margin_m: 45.0
    ```

    Значения можно оставить близкими к текущим, чтобы первый diff менял модель,
    а не одновременно агрессивность полёта.

11. Обновить unit tests в
    `drone_city_nav/tests/offboard_velocity_follower_test.cpp:13`.
    Результат: новая формула и выбор constraints покрыты автотестами.

    Обязательные тесты:
    - `SmallTurnCanStillLimitSpeedWhenCloseAndFast`: малый угол вблизи даёт
      speed limit ниже cruise, если физика требует торможения.
    - `AllowedSpeedNowDecreasesAsTurnConstraintApproaches`: один и тот же
      path/angle/config/current speed, две позиции или projection на разных
      расстояниях до одного и того же turn constraint; assert, что ближняя
      позиция даёт меньший `allowed_speed_now_mps` / `raw_speed_limit_mps`,
      чем дальняя, и оба значения finite. Это напрямую покрывает требование
      "чем ближе к повороту, тем ниже `v_allowed_now`".
    - `LargeTurnLimitsMoreThanSmallTurnAtSameDistance`: большой угол даёт
      меньший `constraint_speed_mps` и меньший `raw_speed_limit_mps`.
    - `MostRestrictiveUpcomingConstraintWins`: среди нескольких будущих углов
      выбирается не первый, а самый строгий.
    - `GoalConstraintUsesSameSpeedLimitFormula`: goal даёт `constraint_speed=0`
      и ограничивает скорость по той же формуле.
    - `FarTurnKeepsCruiseWhenBrakingDistanceAllows`: дальний мягкий/средний
      поворот не заставляет ползти заранее.
    - `AccelerationAndDecelerationRateLimitStillApply`: итоговая скорость не
      прыгает мгновенно, а ограничена `max_accel_mps2` / `max_decel_mps2`.
    - `DegenerateOrTinyAnglesRemainFinite`: near-zero angles не дают NaN/Inf.

12. Обновить/добавить telemetry contract tests, если существующие script/unit
    tests проверяют строки логов или blackbox fields.
    Результат: новые diagnostic поля не остаются непроверенным контрактом.

    Основной существующий файл:
    - `scripts/tests/test_offboard_telemetry_contract.py`.

    Если реализации понадобится C++ contract coverage вместо script-level
    проверки, явно добавить новый test file и подключить его в CMake, но не
    ссылаться на несуществующий файл как на текущий артефакт.

13. Удалить или нейтрализовать старую терминологию active path.
    Результат: в runtime path больше нет смысла "тормозим только перед
    `sharp_turn`".

    Конкретно проверить:
    - `turn_slowdown_min_angle_deg`;
    - `sharp_turn_angle_deg`;
    - `targetTurnSpeedMps()`;
    - `VelocitySetpointReason::kGentleTurn`;
    - log labels `turn_target_speed` / `braking_distance`, чтобы они не
      скрывали новый limiting-constraint смысл.

# Verification plan

Перед коммитом реализации:

1. Форматирование изменённых C++ файлов:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Минимально релевантные C++ тесты после сборки:

   ```bash
   ./scripts/dev_shell.sh make build
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
     --output-on-failure -R OffboardVelocityFollower
   ```

3. Полная repo-approved проверка:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

4. Script-level contract checks, если реализация меняет
   `scripts/tests/test_offboard_telemetry_contract.py`, telemetry/blackbox
   строки в `drone_city_nav/src/px4_offboard_node.cpp` или config values,
   которые этот script проверяет:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

   Текущий baseline risk: на момент планирования эта команда падает в
   `scripts/tests/test_offboard_telemetry_contract.py:86`, потому что test
   ожидает `cruise_speed_mps: 12.0`, а
   `drone_city_nav/config/urban_mvp.yaml:125` уже содержит
   `cruise_speed_mps: 22.0`. Будущая реализация должна либо обновить contract
   test вместе с актуальными config values, либо отдельно закрыть эту
   рассинхронизацию до финальной проверки.

5. Симуляционные проверки не запускать автоматически. После отдельной явной
   команды пользователя использовать:

   ```bash
   ./scripts/sim_headless.sh
   ```

   Затем анализировать:
   - `log/offboard_blackbox.jsonl`;
   - `Drone velocity command diagnostics`;
   - mission result;
   - max speed / min clearance / tilt / limiting constraint history.

Skipped checks на этапе планирования:

- Notion read: не требуется, потому что prompt не содержит Notion task id, а
  policy `optional`.
- GitLab read: не требуется, потому что prompt не содержит GitLab MR/review.
- Simulation run: запрещено запускать без явной команды пользователя.
- `make test-scripts` на этапе планирования был проверен как baseline и сейчас
  падает на существующей рассинхронизации telemetry contract/config; это не
  исправлялось в plan-раунде, но должно быть закрыто при реализации, если
  затрагиваются telemetry/blackbox/config contract fields.

# Testing strategy

## Категория 1: без рефакторинга

- Расширить `offboard_velocity_follower_test.cpp` вокруг текущих public
  функций `speedLimitForUpcomingTurn()`, `speedLimitForFinalStop()` и
  `planVelocitySetpoint()`.
- Перед добавлением тестов согласовать signatures declarations/definitions для
  `speedLimitForUpcomingTurn()` и `speedLimitForFinalStop()`, чтобы public API
  не содержал stale параметров и тесты не закрепляли рассинхронизацию.
- Проверить happy path: прямой путь держит cruise, близкий средний/резкий угол
  снижает target speed, goal снижает target speed до остановки.
- Проверить monotonic property на одном и том же constraint: меньшая дистанция
  до поворота даёт меньший `allowed_speed_now_mps` / `raw_speed_limit_mps`.
- Проверить negative path: невалидный/degenerate path не даёт valid plan и не
  создаёт NaN.
- Проверить edge cases: tiny angle, несколько поворотов в preview horizon,
  constraint за пределами preview.

## Категория 2: лёгкий рефакторинг

- Вынести helper расчёта `TurnConstraint` / `speedLimitBeforeConstraint()` в
  `offboard_velocity_follower.cpp` с тестами через существующий public API.
- Расширить diagnostic structs без отдельного нового production component.
- Обновить `scripts/tests/test_offboard_telemetry_contract.py` на наличие новых
  полей blackbox/telemetry.

## Категория 3: тяжёлый рефакторинг

- Делать только если implementation станет слишком сложной:
  выделить отдельный модуль `offboard_speed_profile.hpp/cpp` с чистой
  функцией построения constraints и расчёта speed limit.
- Тогда добавить отдельный test file
  `drone_city_nav/tests/offboard_speed_profile_test.cpp`.
- Этот вариант полезен, если `offboard_velocity_follower.cpp` начнёт смешивать
  projection, geometry, speed profile и PX4-facing diagnostics слишком плотно.

# Risks and tradeoffs

- Поведение: более раннее торможение перед малыми углами может снизить среднюю
  скорость, если `turn_radius_base_m`, `braking_margin_m` или `max_decel_mps2`
  выбраны слишком консервативно.
- Поведение: слишком агрессивная формула может оставить проблему overshoot на
  быстрых участках, особенно на серии мягких поворотов.
- API/контракты: переименование/удаление `turn_slowdown_min_angle_deg` и
  `sharp_turn_angle_deg` может сломать YAML или тесты, которые ожидают эти
  параметры. Нужно решить: сохранить alias на один релиз или удалить сразу.
- Диагностика: новые JSON поля должны писаться через `writeJsonNumberOrNull()`,
  иначе можно вернуть проблему non-finite JSON.
- Script-level contract: текущий `scripts/tests/test_offboard_telemetry_contract.py`
  уже рассинхронизирован с `urban_mvp.yaml` по speed config values
  (`cruise_speed_mps: 12.0` в test против `22.0` в YAML). Если будущая
  реализация меняет telemetry/blackbox/config fields, нужно обновить этот
  contract test и запускать `./scripts/dev_shell.sh make test-scripts`, иначе
  можно оставить headless/debug contract красным несмотря на зелёный
  `make quality`.
- Интеграция PX4: меняется только velocity setpoint magnitude, но если новый
  speed profile часто ограничивает скорость, PX4 может выглядеть более
  медленным на прямых до настройки параметров.
- Производительность: просмотр всех поворотов в preview horizon на каждом
  control tick линейный по числу waypoint'ов в horizon. Для текущих коротких
  сглаженных путей это дешево; при очень длинных путях можно оптимизировать
  кэшированием cumulative distances.

# Open questions

1. Сохранять ли старые YAML параметры `turn_slowdown_min_angle_deg` и
   `sharp_turn_angle_deg` как deprecated aliases на один переходный период, или
   удалить сразу из активного конфига?
2. Какое стартовое имя параметра выбрать для геометрии поворота:
   `turn_radius_base_m`, `turn_corner_cut_distance_m` или другое? По смыслу
   предпочтительно `turn_radius_base_m`, потому что он напрямую участвует в
   расчёте виртуального радиуса.
3. Нужно ли после реализации сразу менять агрессивность текущих значений, или
   сначала оставить скорости/ускорения как есть и сравнить поведение только от
   новой формулы?
