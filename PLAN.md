# Context

Нужно детально запланировать улучшение плавности полета в поворотах и при
резкой смене target после replan. Цель не вернуть lookahead, а сделать текущую
схему "летим по waypoint'ам пути" менее дерганой:

- торможение перед поворотом должно зависеть от расстояния до поворота и
  физически допустимого замедления, а не только от факта "угол уже рядом";
- `velocity feedforward` не должен мгновенно менять направление вектора;
- после replan target должен сохраняться, если он все еще совместим с новым
  путем и не ведет в prohibited-зону;
- новые решения должны быть покрыты автотестами и логами, достаточными для
  headless-отладки.

Текущая важная предпосылка: после недавнего изменения offboard follower больше
не использует lookahead/lead target и возвращает текущий waypoint как target
пути.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует, отдельного investigation-артефакта
для этого планирования нет.

Из локального кода:

- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:126):
  параметры скорости, target continuity и hysteresis объявляются в конструкторе
  `Px4OffboardNode`.
- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:363):
  `onPath()` принимает новый `nav_msgs::Path`, выбирает `waypoint_index_` через
  `advanceWaypointIndex()` и включает `path_update_target_hysteresis_pending_`.
- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:801):
  `currentTarget()` сейчас возвращает `path_points_[waypoint_index_]`, то есть
  target равен waypoint, а не lookahead-точке.
- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:848):
  `applyPathUpdateTargetHysteresis()` сейчас сравнивает старый и новый target по
  расстоянию и близости старого target к новому пути, но не логирует полноценную
  причину решения и не проверяет segment `current_position -> previous_target`
  на prohibited-пересечение.
- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:914):
  `makeSpeedControllerInput()` передает в speed controller только `turn_angle_rad`,
  но не расстояние до поворота.
- [drone_city_nav/src/px4_offboard_node.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node.cpp:746):
  `publishTrajectorySetpoint()` публикует target position и velocity feedforward
  в PX4 `TrajectorySetpoint`.
- [drone_city_nav/src/offboard_speed_controller.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/offboard_speed_controller.cpp:96):
  `turnLimitedSpeedMps()` ограничивает скорость только по величине угла поворота.
- [drone_city_nav/src/offboard_speed_controller.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/offboard_speed_controller.cpp:149):
  `OffboardSpeedController::update()` применяет goal limit, turn limit,
  acceleration ramp, optional tracking overspeed и hard step cap.
- [drone_city_nav/src/offboard_path_follower.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/offboard_path_follower.cpp:147):
  `pathTurnAngleAtWaypoint()` умеет вернуть угол ближайшего waypoint-поворота,
  но не возвращает расстояние до него и diagnostic state.
- [drone_city_nav/config/urban_mvp.yaml](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/config/urban_mvp.yaml:128):
  текущие параметры offboard: `desired_speed_mps: 5.0`,
  `max_accel_mps2: 2.5`, `turn_slowdown_min_speed_mps: 0.8`,
  `turn_slowdown_preview_distance_m: 32.0`,
  `commanded_target_hysteresis_m: 0.5`.
- [drone_city_nav/tests/offboard_speed_controller_test.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/tests/offboard_speed_controller_test.cpp:74):
  уже есть тесты для дискретного turn slowdown, goal slowdown, acceleration ramp,
  hard step cap и tracking overspeed.
- [drone_city_nav/tests/offboard_path_follower_test.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/tests/offboard_path_follower_test.cpp:49):
  уже есть тесты для turn-angle preview distance, но нет теста расстояния до
  upcoming turn.

# Detected stack/profiles

Основной стек workspace: ROS 2 workspace с пакетом `drone_city_nav` на C++,
ament CMake, `colcon`, Gazebo/PX4 SITL. В репозитории есть `.clang-format`,
`Makefile`, `drone_city_nav/CMakeLists.txt`, C++ headers/sources/tests и
`compile_commands.json` в build-директориях.

Прочитанные обязательные профили:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Также прочитаны обязательные протоколы:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

Notion/GitLab CLI не вызывались, потому что prompt не содержит Notion task,
GitLab MR или GitLab discussion; при `notion_policy: optional` чтение Notion
не требуется.

# Repo-approved commands found

Найденные repo-approved команды из `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_gui.sh`
- `./scripts/sim_headless.sh`
- `./scripts/dev_shell.sh` для интерактивной container shell
- внутри контейнера: `make build`
- внутри контейнера: `make test`
- внутри контейнера: `make test-scripts`
- внутри контейнера: `make quality`
- внутри контейнера: `make format`
- внутри контейнера: `make sim-gui`
- внутри контейнера: `make sim-headless`

Контейнерный workflow является единственным поддерживаемым build/test/quality
и simulation workflow. Прямой top-level CMake workflow использовать не нужно.
`CMakePresets.json` в workspace не найден.

# Affected components

- `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
- `drone_city_nav/src/offboard_path_follower.cpp`
- `drone_city_nav/include/drone_city_nav/offboard_speed_controller.hpp`
- `drone_city_nav/src/offboard_speed_controller.cpp`
- новый ROS-free helper для ограничения velocity-vector, например
  `drone_city_nav/include/drone_city_nav/offboard_velocity_limiter.hpp` и
  `drone_city_nav/src/offboard_velocity_limiter.cpp`
- новый ROS-free helper для решения по target continuity, если логика станет
  крупной, например
  `drone_city_nav/include/drone_city_nav/offboard_target_continuity.hpp` и
  `drone_city_nav/src/offboard_target_continuity.cpp`
- `drone_city_nav/src/px4_offboard_node.cpp`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/config/real_drone_template.yaml`
- `drone_city_nav/CMakeLists.txt`
- `drone_city_nav/tests/offboard_path_follower_test.cpp`
- `drone_city_nav/tests/offboard_speed_controller_test.cpp`
- новые тесты для velocity limiter / target continuity, если helper'ы будут
  вынесены
- `docs/MVP_SIMULATION.md`

# Implementation steps

1. Добавить diagnostic-модель upcoming turn в path follower.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_path_follower.hpp`
   - `drone_city_nav/src/offboard_path_follower.cpp`
   - `drone_city_nav/tests/offboard_path_follower_test.cpp`

   Code anchors:

   - `OffboardPathFollowerConfig`
   - `pathTurnAngleAtWaypoint()`
   - `advanceWaypointIndex()`

   Материализуемый результат: вместо одиночного `double turn_angle_rad` будет
   доступна структура, которая описывает ближайший релевантный поворот:

   ```cpp
   struct UpcomingTurn {
     bool valid{false};
     std::size_t waypoint_index{0U};
     double distance_to_turn_m{std::numeric_limits<double>::infinity()};
     double angle_rad{0.0};
     Point2 turn_point{};
   };
   ```

   Новый helper должен считать расстояние от текущей позиции до waypoint-поворота
   по текущему пути, а не просто евклидово расстояние до точки. Для текущей
   direct-waypoint схемы достаточно начинать с текущего `waypoint_index_` и
   суммировать длины сегментов до первого waypoint, где угол больше нуля и точка
   находится в `turn_slowdown_preview_distance_m`.

   Псевдокод:

   ```text
   upcomingTurn(path, current_position, waypoint_index, config):
     if path.size < 3 or waypoint_index >= path.size:
       return invalid

     distance_accumulator = distance(current_position, path[waypoint_index])
     for i in waypoint_index .. path.size - 2:
       angle = turnAngle(path[i-1 or current], path[i], path[i+1])
       if angle > tiny_angle:
         if distance_accumulator <= config.turn_slowdown_preview_distance_m:
           return valid(i, distance_accumulator, angle, path[i])
         return invalid
       distance_accumulator += distance(path[i], path[i + 1])
   ```

   Обновить существующий `pathTurnAngleAtWaypoint()` так, чтобы он либо
   использовал новый helper, либо остался тонким compatibility wrapper'ом.

   Автотесты:

   - straight path возвращает `valid=false`;
   - 90-degree turn в пределах preview возвращает корректный `angle_rad` и
     `distance_to_turn_m`;
   - turn за пределами `turn_slowdown_preview_distance_m` игнорируется;
   - текущая позиция после начала сегмента дает меньшее `distance_to_turn_m`;
   - path из одной/двух точек не падает и возвращает invalid.

2. Сделать turn slowdown distance-aware в speed controller.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_speed_controller.hpp`
   - `drone_city_nav/src/offboard_speed_controller.cpp`
   - `drone_city_nav/tests/offboard_speed_controller_test.cpp`

   Code anchors:

   - `SpeedControllerConfig`
   - `SpeedControllerInput`
   - `SpeedLimitBreakdown`
   - `turnLimitedSpeedMps()`
   - `OffboardSpeedController::update()`

   Материализуемый результат: speed controller получает не только угол, но и
   расстояние до upcoming turn. Существующая функция `turnLimitedSpeedMps()` может
   остаться как расчет entry speed по углу, но итоговый turn limit должен
   учитывать дистанцию торможения:

   ```cpp
   struct SpeedControllerInput {
     bool hold_position{true};
     double controller_dt_s{0.1};
     double distance_to_goal_m{std::numeric_limits<double>::infinity()};
     double distance_to_turn_m{std::numeric_limits<double>::infinity()};
     double turn_angle_rad{0.0};
     double actual_speed_mps{0.0};
   };
   ```

   Новая логика:

   ```text
   turn_entry_speed = turnLimitedSpeedMps(config, turn_angle_rad)
   if no valid turn or turn_entry_speed >= desired:
     turn_limit = desired
   else:
     usable_distance = max(distance_to_turn_m - turn_braking_safety_margin_m, 0)
     turn_limit = sqrt(turn_entry_speed^2 + 2 * max_accel_mps2 * usable_distance)
     turn_limit = clamp(turn_limit, turn_entry_speed, desired)
   ```

   Добавить в `SpeedControllerConfig` отдельный параметр
   `turn_braking_safety_margin_m`, чтобы не смешивать goal braking margin и turn
   braking margin. Добавить в `SpeedLimitBreakdown` diagnostic fields:

   - `turn_entry_speed_mps`
   - `turn_braking_distance_m`
   - `distance_to_turn_m`

   Автотесты:

   - далекий sharp turn не режет cruise speed;
   - близкий sharp turn ограничивает `allowed_speed_mps`;
   - на самом повороте limit равен `turn_slowdown_min_speed_mps`;
   - invalid/infinite `distance_to_turn_m` не ломает cruise;
   - acceleration ramp не позволяет мгновенно подпрыгнуть к новому limit;
   - goal limit остается сильнее turn limit, если goal ближе.

3. Подключить upcoming turn diagnostic в `Px4OffboardNode`.

   Файл:

   - `drone_city_nav/src/px4_offboard_node.cpp`

   Code anchors:

   - `pathFollowerConfig()`
   - `makeSpeedControllerInput()`
   - `pathTurnAngleAtWaypoint()`
   - `logControlSummary()`
   - `writeFlightBlackboxRecord()`

   Материализуемый результат:

   - `makeSpeedControllerInput()` заполняет `distance_to_turn_m` и `turn_angle_rad`
     из нового `upcomingTurn(...)`;
   - telemetry log каждые 0.5 секунды выводит `turn_distance_m`,
     `turn_angle_deg`, `turn_entry_speed_mps`, `turn_limit_mps` и причину
     speed-limit;
   - `log/offboard_blackbox.jsonl` получает те же поля, чтобы headless-прогон
     позволял найти место, где началось торможение перед поворотом.

   Пример ожидаемой строки лога:

   ```text
   Offboard telemetry: ... speed[requested=3.20 allowed=3.70 reason=turn]
   turn[valid=true index=4 distance=18.4 angle_deg=91.0 entry_speed=0.8 limit=3.7]
   ```

4. Добавить ROS-free velocity feedforward vector limiter.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/offboard_velocity_limiter.hpp`
   - `drone_city_nav/src/offboard_velocity_limiter.cpp`
   - `drone_city_nav/tests/offboard_velocity_limiter_test.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Code anchors:

   - `Px4OffboardNode::velocityFeedforward(...)`
   - `Px4OffboardNode::publishTrajectorySetpoint(...)`

   Материализуемый результат: raw feedforward vector "скорость в направлении
   target" пропускается через limiter, который ограничивает изменение вектора
   за `dt`. Это не меняет waypoint target и не добавляет lookahead, а только
   сглаживает команду скорости.

   Предлагаемый контракт:

   ```cpp
   struct VelocityLimiterConfig {
     double max_vector_accel_mps2{3.0};
     double max_heading_rate_radps{1.5};
   };

   struct VelocityLimiterOutput {
     Point2 velocity_mps{};
     double raw_delta_mps{0.0};
     double applied_delta_mps{0.0};
     bool vector_delta_limited{false};
     bool heading_rate_limited{false};
   };
   ```

   Псевдокод:

   ```text
   raw = unit(target - current_position) * requested_speed
   max_delta = max_vector_accel_mps2 * dt
   limited = previous + clamp_norm(raw - previous, max_delta)
   if heading_rate_limit_enabled:
     ограничить угол между previous и limited за dt
   previous = limited
   ```

   Важно: limiter должен reset'иться при hold, stale pose, выключенном
   `velocity_feedforward_enabled` и при disarm/mission complete, чтобы не
   протащить старую команду в новый режим.

   Автотесты:

   - при прямолинейном ускорении вектор растет не быстрее
     `max_vector_accel_mps2 * dt`;
   - при резкой смене направления на 90 градусов вектор не разворачивается за
     один тик;
   - reset очищает предыдущий вектор;
   - disabled/zero speed возвращает нулевой vector без NaN;
   - heading-rate limit не увеличивает magnitude выше requested speed.

5. Смягчить смену target при replan без возврата к lookahead.

   Файлы:

   - `drone_city_nav/src/px4_offboard_node.cpp`
   - при необходимости:
     `drone_city_nav/include/drone_city_nav/offboard_target_continuity.hpp`
     и `drone_city_nav/src/offboard_target_continuity.cpp`
   - при необходимости:
     `drone_city_nav/tests/offboard_target_continuity_test.cpp`

   Code anchors:

   - `Px4OffboardNode::onPath()`
   - `Px4OffboardNode::applyPathUpdateTargetHysteresis()`
   - `Px4OffboardNode::selectCommandTarget()`
   - subscriber на `prohibited_grid_topic`

   Материализуемый результат: при новом path старый target сохраняется, если:

   - старый target близок к новой path geometry в пределах
     `commanded_target_hysteresis_m`;
   - segment `current_position -> previous_target` не пересекает prohibited grid;
   - старый target не находится явно позади текущего progress по пути.

   Если target нужно заменить, замена происходит сразу на waypoint нового пути,
   но velocity-vector limiter из шага 4 делает смену направления управляемой.
   Дополнительные synthetic/intermediate target'ы не добавлять.

   Причины решения должны быть явными:

   - `kept_previous_target`
   - `switched_to_new_waypoint`
   - `forced_switch_unsafe_previous`
   - `forced_switch_previous_behind_path`
   - `no_previous_target`

   Псевдокод:

   ```text
   decideTargetAfterReplan(previous_target, proposed_target, current_position, path, prohibited):
     if no previous:
       return switched_to_new_waypoint
     projection = closestOffboardPathProjection(path, previous_target, min_segment)
     if projection missing or projection.distance > hysteresis:
       return switched_to_new_waypoint
     if segment(current_position, previous_target) crosses prohibited:
       return forced_switch_unsafe_previous
     if projection is behind current path progress:
       return forced_switch_previous_behind_path
     return kept_previous_target
   ```

   Значение `commanded_target_hysteresis_m` в
   `drone_city_nav/config/urban_mvp.yaml` нужно поднять с `0.5` до стартового
   значения `2.0`, потому что текущие логи показывали, что 0.5 м почти не
   срабатывает. Для `real_drone_template.yaml` указать консервативное значение
   `1.0` или оставить комментарий в документации, что параметр требует полетной
   настройки.

   Автотесты для вынесенного helper:

   - previous target на новом path сохраняется;
   - previous target далеко от нового path заменяется;
   - previous target, segment к которому пересекает prohibited, заменяется;
   - previous target позади progress заменяется;
   - пустой path / stale pose не вызывает UB и возвращает безопасное решение.

6. Обновить параметры и документацию.

   Файлы:

   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/config/real_drone_template.yaml`
   - `docs/MVP_SIMULATION.md`

   Материализуемый результат:

   - добавить параметры:
     - `turn_braking_safety_margin_m`
     - `max_feedforward_vector_accel_mps2`
     - `max_feedforward_heading_rate_radps`
   - пересмотреть `commanded_target_hysteresis_m`;
   - описать, что target остается waypoint'ом пути, а сглаживание происходит
     через speed profile и velocity-vector limiter;
   - описать новые blackbox/log fields для headless-анализа.

7. Подключить новые source/test files в CMake.

   Файл:

   - `drone_city_nav/CMakeLists.txt`

   Материализуемый результат:

   - новые `.cpp` helper'ы добавлены в соответствующую library/executable target
     по существующему target-based паттерну;
   - новые gtest binaries зарегистрированы через существующий test pattern;
   - include graph не тянет ROS/PX4 headers в ROS-free helper'ы.

8. Обновить headless diagnostics и, если нужно, script-level parsing.

   Файлы:

   - `scripts/validate_city_mvp_headless.py`
   - `scripts/tests/test_validate_city_mvp_headless.py`
   - `scripts/analyze_lidar_projection_snapshots.py` только если туда уже
     логически попадают offboard metrics

   Материализуемый результат:

   - validator не обязан падать на новых полях, но должен уметь находить новые
     telemetry markers, если это уже проверяется;
   - при необходимости добавить lightweight проверку наличия `turn[...]`,
     `feedforward[...]` и target-switch reasons в логах.

9. Выполнить scoped и full verification через repo-approved container workflow.

   Материализуемый результат:

   - форматированы только измененные C++ файлы;
   - unit tests покрывают новую алгоритмику;
   - quality gate проходит;
   - headless-прогоны дают численные логи по speed, turn distance, target-switch
     reason и velocity-vector limiting.

# Verification plan

Команды выполнять из корня репозитория через container workflow:

1. Форматирование измененных C++ файлов:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Полная C++ quality gate:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

3. Если менялись Python validation scripts:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

4. Headless-прогон со статической картой:

   ```bash
   SMOKE_DURATION_S=300 ./scripts/sim_headless.sh
   ```

5. Headless-прогон без статической карты:

   ```bash
   ENABLE_STATIC_MAP=false SMOKE_DURATION_S=300 ./scripts/sim_headless.sh
   ```

6. После headless-прогона проверить offboard blackbox и lidar snapshots:

   ```bash
   python3 scripts/analyze_lidar_projection_snapshots.py \
     log/lidar_debug/snapshots.jsonl \
     --static-map drone_city_nav/worlds/generated_city.map2d
   ```

   Для no-static прогона `--static-map` использовать только как alignment/debug
   reference, не как требование включенной static map.

Ожидаемые проверяемые признаки:

- mission reaches goal;
- нет emergency stop/collision;
- turn slowdown начинается до поворота, а не в момент входа в waypoint;
- `requested_speed_mps` меняется плавно;
- commanded velocity vector не меняет направление скачком за один tick;
- target-switch reasons логируются;
- после replan нет резкого lateral command spike без limiter reason в логах;
- `path waypoints` остаются waypoint'ами, без возврата lookahead.

# Testing strategy

Категория 1: без рефакторинга.

- Расширить `offboard_speed_controller_test.cpp`:
  - happy path: далекий поворот не ограничивает cruise;
  - negative path: близкий sharp turn снижает allowed speed;
  - edge cases: invalid distance, zero/negative accel, near-goal сильнее turn.
- Расширить `offboard_path_follower_test.cpp`:
  - upcoming turn distance;
  - preview threshold;
  - degenerate path.

Категория 2: легкий рефакторинг.

- Вынести velocity-vector limiter в ROS-free helper и покрыть unit tests.
- Вынести target-continuity decision helper, если логика в
  `applyPathUpdateTargetHysteresis()` станет больше простого if/else.
- Тестировать helper'ы без ROS executor/PX4 messages, чтобы тесты были быстрыми
  и детерминированными.

Категория 3: тяжелый/integration.

- `make quality` как workspace-level build + ctest + static analysis.
- `sim_headless.sh` со static map и no-static map по 5 минут.
- Лог-анализ:
  - количество target-switch reasons;
  - max/mean speed;
  - max roll/pitch/tilt;
  - max commanded velocity delta per tick;
  - моменты turn braking start относительно `distance_to_turn_m`.

Gap: полностью доказать "плавность" только unit tests нельзя, потому что PX4
динамика и Gazebo physics проявляются только в integration run. Поэтому unit
tests закрывают алгоритмические контракты, а headless-прогоны закрывают
динамическую регрессию по логам.

# Risks and tradeoffs

- Flight can become too conservative: раннее turn braking может снизить среднюю
  скорость и увеличить время миссии. Проверять по blackbox speed/time metrics.
- Если `turn_slowdown_preview_distance_m` слишком мал, braking все еще начнется
  поздно; если слишком велик, дрон будет тормозить за несколько кварталов.
- Vector limiter может временно направлять velocity не точно на target. Это
  уменьшает дерганье, но может увеличить cross-track error. Проверять
  `cross_track_error_m` и segment progress.
- Сохранение previous target после replan может быть опасно, если prohibited-grid
  freshness плохая. Нужен explicit log, какой grid stamp использовался для
  проверки segment safety.
- Увеличение `commanded_target_hysteresis_m` может задержать переход на новый
  target, если path реально сильно изменился. Проверять forced switch reasons и
  mission reach.
- Новые helper'ы меняют публичные project headers внутри `drone_city_nav`; нужно
  обновить CMake/tests синхронно, иначе build сломается.
- Дополнительные логи/blackbox fields увеличат размер логов, но период 2 Hz
  должен остаться приемлемым.

Что могло сломаться после реализации:

- offboard setpoint publishing в PX4 из-за неверного преобразования limited
  velocity в NED/map frame;
- target continuity при пустом path или stale pose;
- speed controller при NaN/infinity inputs;
- headless validator, если он ожидает старый формат строк;
- existing tests around sharp turn slowdown, потому что turn limit станет
  distance-aware;
- no-static режим, если velocity smoothing замедлит реакцию на свежий obstacle.

# Open questions

1. Какие численные thresholds считать целевыми для "плавности": максимальный
   `tilt_rad`, `roll/pitch`, `commanded_velocity_delta_mps` и допустимый
   `cross_track_error_m`? Для первого implementation pass можно взять текущие
   safety thresholds из логов и улучшать относительно baseline.
2. Нужно ли ограничивать heading-rate отдельно от vector-delta limit? План
   предлагает оба параметра, но если unit/headless покажут, что vector-delta
   достаточно, heading-rate можно оставить выключаемым параметром.
3. Нужно ли менять `desired_speed_mps` вместе с этим изменением? План не меняет
   cruise speed по умолчанию; сначала нужно стабилизировать плавность на текущих
   5 м/с.
