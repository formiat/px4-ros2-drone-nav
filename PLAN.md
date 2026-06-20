# Context

Планируем добавить переключаемый режим полёта `Mission`, сохранив текущий
режим `Offboard`. Переключение должно идти через env var, по умолчанию остаётся
`Offboard`. Недавно добавленный `sharp_turn_hold` относится только к текущему
Offboard waypoint follower и не должен переноситься в Mission: в Mission backend
PX4 получает mission waypoints и сам управляет прохождением waypoint'ов.

Текущая архитектура:

- `planner_node` публикует путь в `nav_msgs/Path` на `/drone_city_nav/path` и
  `path_id` на `/drone_city_nav/path_id`; см. `drone_city_nav/config/urban_mvp.yaml:54`
  и `drone_city_nav/config/urban_mvp.yaml:55`.
- `city_nav.launch.py` всегда стартует `px4_offboard_node`; см.
  `drone_city_nav/launch/city_nav.launch.py:149`.
- `px4_offboard_node` подписывается на path/path_id, PX4 status/local position и
  публикует `OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`; см.
  `drone_city_nav/src/px4_offboard_node.cpp:222`,
  `drone_city_nav/src/px4_offboard_node.cpp:270`,
  `drone_city_nav/src/px4_offboard_node.cpp:725`.
- Offboard уже не отправляет horizontal velocity feed-forward: `TrajectorySetpoint`
  содержит position setpoint, а `velocity`, `acceleration`, `jerk` заполняются
  `NaN`; см. `drone_city_nav/src/px4_offboard_node.cpp:741`.
- Offboard sharp-turn hold реализован внутри `px4_offboard_node`:
  `sharp_turn_hold_angle_deg` и `sharp_turn_hold_s` читаются в
  `drone_city_nav/src/px4_offboard_node.cpp:155`, старт hold - в
  `drone_city_nav/src/px4_offboard_node.cpp:870`.
- PX4 SITL MAVLink API/Offboard link слушает локальный UDP порт `14580` и шлёт на
  remote `14540` для `px4_instance=0`; см.
  `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink:4`.
- В dev container доступен `pymavlink`
  (`/usr/local/lib/python3.12/dist-packages/pymavlink/__init__.py`), значит Mission
  upload можно реализовать без добавления MAVSDK/MAVROS.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. Существующий `PLAN.md` также
отсутствовал, поэтому этот файл создаётся как новый план.

Notion policy в inbox указан как `optional`, а промпт не содержит Notion task ID
или ссылки. По протоколу Notion чтение не требуется. GitLab/MR в промпте также не
упомянуты, поэтому GitLab read-only workflow не запускался.

# Detected stack/profiles

- Основной stack: ROS 2 Jazzy workspace, `ament_cmake`, C++20, PX4 SITL, Gazebo
  Harmonic, Python/bash helper scripts.
- Прочитаны обязательные профили:
  - `generic.md`, потому что он обязателен для любого workspace.
  - `cpp.md`, потому что в workspace есть `Makefile`, `CMakeLists.txt`, `.cpp`,
    `.hpp`.
- Rust profile не применялся: в workspace не найдено `Cargo.toml`, `Cargo.lock`
  или `.rs` файлов.
- Build system: top-level `Makefile` вызывает `colcon`; прямой top-level CMake
  workflow в repo docs запрещён.
- `CMakePresets.json` отсутствует, поэтому CMake presets не используются.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_gui.sh`
- `./scripts/sim_headless.sh`
- `./scripts/dev_shell.sh` только для интерактивной оболочки или запуска
  конкретной команды внутри контейнера.
- Внутри контейнера:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
  - `make sim-gui`
  - `make sim-headless`
- Для scoped C++ tests после build:
  `ctest --test-dir build/drone_city_nav --output-on-failure`.

Перед commit после изменений по repo policy нужно выполнить:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
```

Если изменяются Python helper scripts/tests, дополнительно выполнить:

```bash
./scripts/dev_shell.sh make test-scripts
```

# Affected components

- `scripts/container_run.sh:30` - пропуск новой env var внутрь контейнера.
- `scripts/run_city_mvp.sh:48` и `scripts/run_city_mvp.sh:499` - чтение env var,
  валидация backend, передача launch argument, headless validation context.
- `drone_city_nav/launch/city_nav.launch.py:40` и
  `drone_city_nav/launch/city_nav.launch.py:149` - launch argument
  `navigation_backend` и выбор ровно одного flight-control node.
- `drone_city_nav/config/urban_mvp.yaml:112` и
  `drone_city_nav/config/real_drone_template.yaml:113` - добавить параметры
  нового `px4_mission_node`, не трогая Offboard sharp-turn hold.
- `drone_city_nav/package.xml:23` - добавить runtime dependency для `rclpy`, если
  Mission backend будет Python ROS node.
- `drone_city_nav/CMakeLists.txt:132` - install нового executable/script.
- Новый Mission backend, рекомендуемый файл:
  `drone_city_nav/scripts/px4_mission_node.py`.
- Новые pure helper tests для Mission item generation/upload state machine,
  рекомендуемый файл: `scripts/tests/test_px4_mission_node.py`.
- `scripts/validate_city_mvp_headless.py:26` и
  `scripts/validate_city_mvp_headless.py:232` - backend-aware validation.
- `scripts/tests/test_run_city_mvp_launch_contract.py:13` - contract tests для env
  и launch args.
- `scripts/tests/test_validate_city_mvp_headless.py:27` - log validator tests для
  Offboard и Mission markers.
- `docs/MVP_SIMULATION.md:138` и `docs/MVP_SIMULATION.md:473` - документация по
  двум backend'ам и headless diagnostics.

# Implementation steps

1. Добавить единый runtime selector `NAVIGATION_BACKEND`.

   Файлы:

   - `scripts/container_run.sh:30`
   - `scripts/run_city_mvp.sh:48`
   - `scripts/run_city_mvp.sh:499`
   - `drone_city_nav/launch/city_nav.launch.py:40`

   Материализуемый результат:

   - `NAVIGATION_BACKEND` прокидывается в контейнер.
   - `scripts/run_city_mvp.sh` нормализует значение в lower-case, принимает только
     `offboard` и `mission`, дефолт `offboard`.
   - В ROS launch добавляется argument `navigation_backend`, дефолт `offboard`.
   - В логах запуска есть стабильная строка вида
     `Navigation backend: offboard` или `Navigation backend: mission`.

   Pseudocode для shell:

   ```bash
   navigation_backend="${NAVIGATION_BACKEND:-offboard}"
   navigation_backend="${navigation_backend,,}"
   case "${navigation_backend}" in
     offboard|mission) ;;
     *) echo "NAVIGATION_BACKEND must be offboard or mission" >&2; exit 1 ;;
   esac
   ros_launch_args+=(navigation_backend:="${navigation_backend}")
   ```

2. В launch запускать ровно один flight-control backend.

   Файл:

   - `drone_city_nav/launch/city_nav.launch.py:149`

   Материализуемый результат:

   - `Offboard` mode стартует только `px4_offboard_node`.
   - `Mission` mode стартует только `px4_mission_node`.
   - `planner_node`, `obstacle_memory_node`, `mission_monitor_node`,
     `lidar_debug_node` остаются общими.
   - Неверный backend приводит к понятной ошибке launch.

   Pseudocode:

   ```python
   def flight_control_nodes(context, *args, **kwargs):
       backend = navigation_backend.perform(context).strip().lower() or "offboard"
       if backend == "offboard":
           return [Node(package="drone_city_nav",
                        executable="px4_offboard_node",
                        name="px4_offboard_node",
                        output="screen",
                        parameters=[params_file.perform(context)])]
       if backend == "mission":
           return [Node(package="drone_city_nav",
                        executable="px4_mission_node.py",
                        name="px4_mission_node",
                        output="screen",
                        parameters=[params_file.perform(context)])]
       raise RuntimeError(
           f"Launch argument 'navigation_backend' must be offboard or mission, got '{backend}'"
       )
   ```

   Важно: не переносить параметры `sharp_turn_hold_*` в Mission node.

3. Добавить Mission backend через MAVLink mission upload.

   Рекомендуемый файл:

   - `drone_city_nav/scripts/px4_mission_node.py`

   Материализуемый результат:

   - Node подписывается на `/drone_city_nav/path` и `/drone_city_nav/path_id`.
   - Node подключается к PX4 MAVLink endpoint через `pymavlink`, дефолт
     `udpin:0.0.0.0:14540`.
   - При новом непустом `path_id` конвертирует `nav_msgs/Path` в MAVLink mission
     items и загружает mission в PX4.
   - После успешной загрузки при `auto_mission=true` переводит PX4 в
     `AUTO.MISSION`, при `auto_arm=true` армит.
   - При пустом path не отправляет mission upload; логирует skip marker.
   - Не публикует `TrajectorySetpoint` и `OffboardControlMode`.
   - Не содержит sharp-turn hold, target-switch hold или другую Offboard-specific
     waypoint timing логику.

   Нужные параметры:

   - `path_topic`
   - `path_id_topic`
   - `mission_connection_url`
   - `mission_upload_timeout_s`
   - `mission_acceptance_radius_m`
   - `mission_cruise_altitude_m`
   - `mission_home_source`, например `mavlink_home` или `params`
   - `mission_home_latitude_deg`
   - `mission_home_longitude_deg`
   - `mission_home_altitude_m`
   - `px4_local_origin_x_m`
   - `px4_local_origin_y_m`
   - `auto_arm`
   - `auto_mission`
   - `mission_blackbox_enabled`
   - `mission_blackbox_path`

   Pseudocode:

   ```python
   def on_path(path_msg: Path) -> None:
       path_id = latest_path_id
       if not path_msg.poses:
           log("MISSION_BACKEND empty_path_skip path_id=%s", path_id)
           return
       if path_id == last_uploaded_path_id:
           log("MISSION_BACKEND duplicate_path_skip path_id=%s", path_id)
           return

       home = resolve_home_position()
       items = build_mission_items(path_msg, home, config)
       log("MISSION_BACKEND upload_started path_id=%s waypoints=%d",
           path_id, len(items))
       ack = mavlink_client.upload_mission(items, timeout_s)
       log("MISSION_BACKEND upload_result path_id=%s success=%s ack=%s",
           path_id, ack.ok, ack.type_name)
       if ack.ok:
           last_uploaded_path_id = path_id
           if auto_mission:
               mavlink_client.set_auto_mission_mode()
           if auto_arm:
               mavlink_client.arm()
   ```

4. Конвертировать local map path в PX4 global-relative mission items.

   Файл:

   - `drone_city_nav/scripts/px4_mission_node.py`

   Материализуемый результат:

   - Mission backend не пытается отправлять local frame mission waypoints, потому
     что PX4 `mavlink_mission.cpp` принимает обычные waypoint coordinates как
     `MAV_FRAME_GLOBAL*` / `MAV_FRAME_GLOBAL_RELATIVE_ALT*`; см.
     `external/PX4-Autopilot/src/modules/mavlink/mavlink_mission.cpp:1399`.
   - Для MVP используется `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` и
     `MAV_CMD_NAV_WAYPOINT`; PX4 читает acceptance radius из `param2`; см.
     `external/PX4-Autopilot/src/modules/mavlink/mavlink_mission.cpp:1439`.
   - `map` point переводится в PX4 local NED тем же смыслом, что Offboard:
     `map_to_px4_local = map - px4_local_origin`.
   - PX4 local north/east переводится в lat/lon относительно home.

   Pseudocode:

   ```python
   def map_to_px4_local(point, origin):
       return point.x - origin.x, point.y - origin.y

   def local_ne_to_global(home_lat_deg, home_lon_deg, north_m, east_m):
       earth_radius_m = 6378137.0
       lat_rad = math.radians(home_lat_deg)
       lat = home_lat_deg + math.degrees(north_m / earth_radius_m)
       lon = home_lon_deg + math.degrees(east_m / (earth_radius_m * math.cos(lat_rad)))
       return lat, lon

   def build_waypoint_item(seq, point):
       north_m, east_m = map_to_px4_local(point, origin)
       lat, lon = local_ne_to_global(home.lat, home.lon, north_m, east_m)
       return MissionItemInt(
           seq=seq,
           frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
           command=MAV_CMD_NAV_WAYPOINT,
           param1=0.0,
           param2=mission_acceptance_radius_m,
           param3=0.0,
           param4=float("nan"),
           x=int(round(lat * 1e7)),
           y=int(round(lon * 1e7)),
           z=mission_cruise_altitude_m,
           autocontinue=1,
       )
   ```

5. Реализовать MAVLink upload adapter с тестируемым интерфейсом.

   Файл:

   - `drone_city_nav/scripts/px4_mission_node.py`

   Материализуемый результат:

   - Логика upload отделена от ROS callbacks классом `MavlinkMissionClient`.
   - Для unit tests можно подставить fake client без реального UDP/PX4.
   - Upload протокол покрывает минимум:
     - `MISSION_CLEAR_ALL` или explicit clear перед загрузкой.
     - `MISSION_COUNT`.
     - Ответы `MISSION_REQUEST_INT` / `MISSION_REQUEST`.
     - Отправку `MISSION_ITEM_INT`.
     - Финальный `MISSION_ACK`.
   - Ошибки timeout/unsupported ack логируются и пишутся в mission blackbox.

   Pseudocode:

   ```python
   class MavlinkMissionClient:
       def upload_mission(self, items: list[MissionItemInt], timeout_s: float) -> UploadResult:
           self.clear_all()
           self.send_mission_count(len(items))
           while pending:
               msg = self.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                     timeout=timeout_s)
               if msg is None:
                   return UploadResult.timeout()
               if msg.get_type() == "MISSION_ACK":
                   return UploadResult.from_ack(msg)
               self.send_mission_item(items[msg.seq])
           return self.wait_ack(timeout_s)
   ```

6. Зарегистрировать Mission backend в package/build.

   Файлы:

   - `drone_city_nav/CMakeLists.txt:132`
   - `drone_city_nav/package.xml:23`
   - возможно `docker/Dockerfile`, только если import `pymavlink` перестанет быть
     доступен в image.

   Материализуемый результат:

   - `px4_mission_node.py` устанавливается в `lib/drone_city_nav`.
   - `package.xml` содержит `exec_depend` на `rclpy`.
   - Не добавлять MAVSDK/MAVROS, если `pymavlink` в dev image остаётся доступен.

   CMake sketch:

   ```cmake
   install(
     PROGRAMS scripts/px4_mission_node.py
     DESTINATION lib/${PROJECT_NAME})
   ```

7. Добавить параметры Mission backend в runtime configs.

   Файлы:

   - `drone_city_nav/config/urban_mvp.yaml:112`
   - `drone_city_nav/config/real_drone_template.yaml:113`

   Материализуемый результат:

   - Появляется секция `px4_mission_node`.
   - Секция содержит path/path_id topics, home/origin/altitude, MAVLink URL,
     auto flags и blackbox params.
   - В секции Mission нет `sharp_turn_hold_angle_deg`, `sharp_turn_hold_s`,
     `target_switch_hold_*`.

   YAML sketch:

   ```yaml
   px4_mission_node:
     ros__parameters:
       path_topic: /drone_city_nav/path
       path_id_topic: /drone_city_nav/path_id
       mission_connection_url: udpin:0.0.0.0:14540
       mission_acceptance_radius_m: 1.0
       mission_cruise_altitude_m: 18.0
       mission_home_source: mavlink_home
       mission_home_latitude_deg: 47.397742
       mission_home_longitude_deg: 8.545594
       mission_home_altitude_m: 0.0
       px4_local_origin_x_m: 27.0
       px4_local_origin_y_m: 27.0
       auto_arm: true
       auto_mission: true
       mission_blackbox_enabled: true
       mission_blackbox_path: log/mission_blackbox.jsonl
   ```

8. Добавить headless-friendly logs и blackbox для Mission.

   Файл:

   - `drone_city_nav/scripts/px4_mission_node.py`

   Материализуемый результат:

   - Runtime logs имеют стабильные markers:
     - `Mission backend ready:`
     - `MISSION_BACKEND path_received`
     - `MISSION_BACKEND upload_started`
     - `MISSION_BACKEND upload_result success=true|false`
     - `MISSION_BACKEND mode_command`
     - `MISSION_BACKEND arm_command`
     - `MISSION_BACKEND progress`
   - JSONL blackbox `log/mission_blackbox.jsonl` содержит:
     - `time_ns`
     - `path_id`
     - `waypoints`
     - `upload_attempt`
     - `upload_success`
     - `ack_type`
     - `current_seq`
     - `finished`
     - `home`
     - `connection_url`

9. Обновить headless validator под два backend'а.

   Файлы:

   - `scripts/validate_city_mvp_headless.py:26`
   - `scripts/run_city_mvp.sh:480`

   Материализуемый результат:

   - Добавлен CLI аргумент `--navigation-backend offboard|mission`.
   - Для Offboard остаются существующие требования:
     - `Sent PX4 command: VEHICLE_CMD_DO_SET_MODE`
     - `Sent PX4 command: VEHICLE_CMD_COMPONENT_ARM_DISARM`
     - `Offboard summary: ... armed=true ... offboard=true`
   - Для Mission проверяются Mission markers:
     - `Mission backend ready:`
     - `MISSION_BACKEND upload_result ... success=true`
     - `MISSION_BACKEND mode_command ... AUTO.MISSION`
     - `MISSION_BACKEND arm_command`
   - Общие planner/lidar/static/memory checks остаются общими для обоих backend'ов.

10. Покрыть selector и scripts contract тестами.

    Файлы:

    - `scripts/tests/test_run_city_mvp_launch_contract.py:13`
    - новый или существующий launch static contract test для
      `drone_city_nav/launch/city_nav.launch.py`
    - `scripts/tests/test_validate_city_mvp_headless.py:27`

    Материализуемый результат:

    - Тест доказывает, что `NAVIGATION_BACKEND` передаётся через
      `container_run.sh` и `run_city_mvp.sh`.
    - Тест доказывает, что default backend - `offboard`.
    - Тест доказывает, что launch содержит branch для `px4_offboard_node` и
      `px4_mission_node.py`, но не запускает оба безусловно.
    - Тест доказывает, что validator принимает Offboard logs и Mission logs по
      разным контрактам.

11. Покрыть Mission pure logic unit tests без PX4/Gazebo.

    Файл:

    - `scripts/tests/test_px4_mission_node.py`

    Материализуемый результат:

    - Happy path: path из 3 точек превращается в 3
      `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` waypoint item'а с правильным seq,
      altitude, acceptance radius и lat/lon direction.
    - Negative path: пустой path не запускает upload.
    - Edge case: повторный тот же `path_id` не загружает mission второй раз.
    - Edge case: timeout/failed `MISSION_ACK` логируется как failure и не
      переводит PX4 в mission mode.
    - Edge case: home из params используется, если `mavlink_home` не пришёл до
      timeout.
    - Fake MAVLink client проверяет порядок вызовов:
      `clear -> count -> item(seq...) -> ack -> set_mode -> arm`.

12. Обновить документацию запуска и ограничений.

    Файлы:

    - `README.md:7`
    - `docs/MVP_SIMULATION.md:138`
    - `docs/MVP_SIMULATION.md:473`

    Материализуемый результат:

    - Документирован default:

      ```bash
      ./scripts/sim_headless.sh
      ./scripts/sim_gui.sh
      ```

      это `NAVIGATION_BACKEND=offboard`.

    - Документирован Mission mode:

      ```bash
      NAVIGATION_BACKEND=mission ./scripts/sim_headless.sh
      NAVIGATION_BACKEND=mission ./scripts/sim_gui.sh
      MISSION_CHECK=1 SMOKE_DURATION_S=300 NAVIGATION_BACKEND=mission ./scripts/sim_headless.sh
      ```

    - Документировано, что Offboard sharp-turn hold не применяется в Mission mode.
    - Документировано, что Mission mode требует working MAVLink endpoint и global
      home/origin conversion.

# Verification plan

Минимальные проверки после реализации:

```bash
./scripts/dev_shell.sh make format
./scripts/dev_shell.sh make quality
./scripts/dev_shell.sh make test-scripts
```

Scoped tests, если build уже существует:

```bash
./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure
```

Headless smoke для дефолтного Offboard:

```bash
SMOKE_DURATION_S=90 ./scripts/sim_headless.sh
```

Headless smoke для Mission backend:

```bash
NAVIGATION_BACKEND=mission SMOKE_DURATION_S=90 ./scripts/sim_headless.sh
```

Полные A-to-B проверки:

```bash
MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/sim_headless.sh
MISSION_CHECK=1 SMOKE_DURATION_S=300 NAVIGATION_BACKEND=mission ./scripts/sim_headless.sh
```

Если Mission full run нестабилен из-за PX4 mission tuning, нельзя скрывать это
`ALLOW_MISSION_FAILURE=true` для финальной проверки. Допустимо использовать
`ALLOW_MISSION_FAILURE=true` только как диагностический smoke, а в outbox явно
указать skipped/failing full mission check.

# Testing strategy

## 1. Без рефакторинга

- Static/script tests:
  - `test_run_city_mvp_launch_contract.py` проверяет env var, launch arg и
    отсутствие безусловного запуска обоих backend'ов.
  - `test_validate_city_mvp_headless.py` проверяет backend-specific log
    contracts.
- Pure Python unit tests:
  - `test_px4_mission_node.py` проверяет path-to-mission conversion,
    upload state machine через fake MAVLink client и duplicate path handling.
- Headless smoke:
  - Offboard default smoke.
  - Mission smoke.

## 2. Лёгкий рефакторинг

- Вынести Mission pure logic из executable script в импортируемый модуль, например
  `drone_city_nav/scripts/px4_mission_backend.py`, а `px4_mission_node.py` оставить
  thin ROS entrypoint.
- Тестировать pure module без запуска ROS/PX4.
- CMake устанавливает оба файла или только executable, но tests импортируют pure
  module по `Path`.

Результат: меньше static string tests, больше нормальных unit tests.

## 3. Тяжёлый рефакторинг

- Выделить общий `flight_backend` слой:
  - `OffboardBackend` остаётся C++ node.
  - `MissionBackend` получает path и публикует/upload'ит mission.
  - Общие conversion/path metrics helpers выносятся в ROS-free C++ или Python
    модуль.
- Добавить integration test harness с fake MAVLink UDP server.
- Это оправдано только если Mission mode станет основным runtime режимом или
  нужно регулярно проверять MAVLink protocol без PX4 SITL.

Рекомендуемый старт: категория 1 + небольшой элемент категории 2 для pure Mission
logic, если executable script становится слишком большим.

# Risks and tradeoffs

- `Mission` через MAVLink global waypoints требует корректного home/origin
  conversion. Ошибка в `px4_local_origin_*` или home lat/lon сдвинет весь маршрут.
  Проверять unit tests на направление north/east и headless mission monitor.
- PX4 Mission mode может проходить waypoint'ы иначе, чем наш Offboard follower:
  acceptance radius, yaw, speed, corner behavior будут определяться PX4 Navigator.
  Проверять headless full run и логи `MISSION_BACKEND progress`.
- Replan в Mission mode фактически будет restart/reupload mission. Это нормально
  для MVP, но может вызвать задержку или резкий переход, если path часто
  перестраивается. Нужно логировать `path_id`, upload duration и duplicate skips.
- Если `pymavlink` исчезнет из будущего dev image, Mission node не стартует.
  Проверять `make test-scripts` import test; при необходимости явно добавить
  dependency в Dockerfile.
- Если launch selector ошибочно запустит оба backend'а, Offboard setpoints и
  Mission mode будут конфликтовать. Это обязательно покрыть contract test'ом.
- Если headless validator останется Offboard-only, Mission smoke будет ложно
  падать или пропускать важные Mission failures.
- В режиме Mission не будет Offboard `sharp_turn_hold`; это ожидаемое поведение,
  но траектория может стать менее предсказуемой на резких углах. Проверять
  mission monitor и PX4 mission progress вместо Offboard target logs.

# Open questions

- Нужно ли для Mission mode использовать только `MAV_CMD_NAV_WAYPOINT`, или сразу
  добавлять `MAV_CMD_NAV_TAKEOFF` первым item'ом? Для текущего MVP проще
  оставить arming/mode отдельно и waypoint mission на постоянной altitude.
- Источник home position: достаточно ли params fallback для MVP, или Mission mode
  должен требовать MAVLink `HOME_POSITION` перед upload? Рекомендация: сначала
  ждать `HOME_POSITION`, затем fallback на params с warning.
- Нужно ли при каждом replan очищать mission и загружать заново с seq=0, или
  пытаться продолжить с ближайшего waypoint? Для MVP безопаснее полный reupload с
  явным логом `replan_upload`.
- Нужно ли Mission backend учитывать emergency stop topic? Текущий Offboard node
  дисармит при emergency stop. Для parity стоит подписать Mission backend на
  `/drone_city_nav/emergency_stop` и отправлять disarm, но это нужно согласовать
  с ожидаемой безопасностью Mission mode.
