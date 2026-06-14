# Context

Нужно исправить баг в recently added lidar projection feature: при наклонах
дрона красный current lidar layer почти пропадает, а желтый remembered lidar
layer становится косым и не совпадает с зеленой static map. Задача следующего
implementation раунда: исправить projection math, покрыть затронутый функционал
автотестами и добавить достаточно логов/артефактов для headless-отладки.

Этот план не меняет production code. Он фиксирует конкретный путь реализации,
файлы, тесты, проверки и риски.

# Investigation context

Входной артефакт: `INVESTIGATION.md`.

Ключевые выводы расследования:

- Зеленая static map корректна, потому что не проходит через lidar projection.
- Баг локализован в shared `projectLidarBeam`:
  `drone_city_nav/src/lidar_projection.cpp:119`.
- Все потребители используют одну и ту же projection-функцию:
  `drone_city_nav/src/obstacle_memory.cpp:141`,
  `drone_city_nav/src/planner_node.cpp:884`,
  `drone_city_nav/src/lidar_debug_node.cpp:424`.
- Текущий код сначала применяет `swap_lidar_xy_to_local_frame` к scan vector:
  `drone_city_nav/src/lidar_projection.cpp:147`, затем инвертирует yaw:
  `drone_city_nav/src/lidar_projection.cpp:151`, затем применяет roll/pitch:
  `drone_city_nav/src/lidar_projection.cpp:154`.
- PX4 локально документирует attitude quaternion как FRD body -> NED earth:
  `external/PX4-Autopilot/msg/versioned/VehicleAttitude.msg:10`.
- PX4 local position использует NED: x = north, y = east, z = down:
  `external/PX4-Autopilot/msg/versioned/VehicleLocalPosition.msg:14`.
- Lidar SDF sensor имеет zero rotation относительно своего link:
  `drone_city_nav/models/lidar_2d_v2/model.sdf:77`.
- Lidar model mounted to X500 fixed joint без rotation:
  `drone_city_nav/models/x500_lidar_2d/model.sdf:11`.
- Тесты не покрывают включенный production-профиль:
  `swap_lidar_xy_to_local_frame=true` + nonzero roll/pitch:
  `drone_city_nav/tests/lidar_projection_test.cpp:36`.

Главная причина: текущий pipeline смешивает старый 2D workaround
`swap_lidar_xy_to_local_frame`, yaw sign hack и PX4 FRD/NED roll/pitch как будто
это одна физическая 3D transform chain. При ненулевом roll/pitch оси наклона
применяются к уже переставленному вектору, поэтому remembered/current lidar
points уезжают относительно static map.

# Detected stack/profiles

- Workspace: ROS 2 Jazzy workspace.
- Основной пакет: `drone_city_nav`.
- Язык: C++20, `ament_cmake`, `colcon`.
- Build entry point: top-level `Makefile`.
- C++ profile применим: есть `Makefile`, `drone_city_nav/CMakeLists.txt`,
  `.cpp/.hpp`, existing `build/` artifacts.
- Rust profile не применим: в target workspace не обнаружены `Cargo.toml`,
  `Cargo.lock` или `.rs` как часть проекта.

Прочитанные профили/протоколы:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Notion/GitLab не читаются как remote systems: в prompt нет Notion task или
GitLab MR, а политика remote access запрещает SSH/HTTP к удаленным целям.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- `./scripts/dev_shell.sh`
- `make build`
- `make test`
- `make quality`
- `./scripts/check_cpp_quality.sh`
- `./scripts/format_cpp_changed.sh`
- `make sim-gui` / `./scripts/run_city_mvp.sh`
- `make sim-headless` /
  `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`

Для долгих build/test/run verification commands использовать
`/home/formi/.local/bin/runlim`. В этой среде это shell-wrapper, который сам
добавляет `systemd-run --user --scope --collect --same-dir` и memory limits.
Корректная форма вызова: передать команду напрямую, без дополнительных
`--user`, `--scope`, `--wait`:

```bash
/home/formi/.local/bin/runlim <command> [args...]
```

# Affected components

- `drone_city_nav/include/drone_city_nav/lidar_projection.hpp:28` -
  public projection config/result contract.
- `drone_city_nav/src/lidar_projection.cpp:23` - scan direction construction.
- `drone_city_nav/src/lidar_projection.cpp:33` - current Euler rotation helper.
- `drone_city_nav/src/lidar_projection.cpp:119` - `projectLidarBeam`.
- `drone_city_nav/tests/lidar_projection_test.cpp:21` - current unit tests.
- `drone_city_nav/src/obstacle_memory.cpp:124` - scan integration projection.
- `drone_city_nav/src/obstacle_memory_node.cpp:96` - projection-related ROS
  params and startup logs.
- `drone_city_nav/src/planner_node.cpp:131` - current lidar overlay params.
- `drone_city_nav/src/planner_node.cpp:848` - current lidar obstacle overlay.
- `drone_city_nav/src/lidar_debug_node.cpp:403` - debug projection pose/config.
- `drone_city_nav/src/lidar_debug_node.cpp:439` - per-beam CSV output.
- `drone_city_nav/src/lidar_debug_node.cpp:710` - JSONL summary output.
- `drone_city_nav/config/urban_mvp.yaml:25`,
  `drone_city_nav/config/urban_mvp.yaml:87`,
  `drone_city_nav/config/urban_mvp.yaml:167` - enabled projection flags.
- `docs/MVP_SIMULATION.md:183` - lidar debug docs.

# Implementation steps

1. Обновить public projection contract в
   `drone_city_nav/include/drone_city_nav/lidar_projection.hpp:28`.

   Материализуемый результат:
   - `LidarProjectionConfig` получает явные параметры sensor/body transform:
     `lidar_mount_roll_rad`, `lidar_mount_pitch_rad`, `lidar_mount_yaw_rad`.
   - `swap_lidar_xy_to_local_frame` остается как deprecated legacy 2D
     compatibility flag, но больше не используется как физический body-frame
     swap перед roll/pitch.
   - `LidarBeamProjection` получает debug поля, чтобы headless logs могли
     объяснить результат projection без видео:
     `Point3 lidar_direction`, `Point3 body_frd_direction`,
     `Point3 ned_direction`.

   Ориентировочный contract snippet:

   ```cpp
   struct LidarProjectionConfig {
     double max_lidar_range_m{35.0};
     double range_hit_epsilon_m{0.05};
     double scan_yaw_offset_rad{0.0};
     double lidar_z_offset_m{0.0};
     double min_projected_altitude_m{0.0};
     double max_projected_altitude_m{100000.0};
     bool swap_lidar_xy_to_local_frame{false}; // legacy 2D mode only
     bool compensate_attitude{false};
     double lidar_mount_roll_rad{0.0};
     double lidar_mount_pitch_rad{0.0};
     double lidar_mount_yaw_rad{0.0};
   };
   ```

2. Переписать frame math в `drone_city_nav/src/lidar_projection.cpp:23`.

   Материализуемый результат:
   - добавить маленький internal `Rotation3`/`Matrix3` helper без внешних
     зависимостей;
   - заменить текущий `scanDirectionInBodyFrame` и `rotateBodyToWorld` на явную
     цепочку:
     `LaserScan/ROS FLU -> configured lidar mount -> PX4 body FRD -> PX4 local NED`;
   - endpoint считать из NED x/y, altitude filter считать через NED z/down;
   - `swap_lidar_xy_to_local_frame` применять только в legacy path при
     `compensate_attitude=false`, либо как final 2D visualization compatibility
     step до удаления флага из конфигов. Не применять его перед roll/pitch.

   Псевдокод ожидаемой логики:

   ```cpp
   const Point3 lidar_flu{std::cos(angle), std::sin(angle), 0.0};
   const Point3 mounted_flu = rotateMount(lidar_flu, config.lidar_mount_*);
   const Point3 body_frd{mounted_flu.x, -mounted_flu.y, -mounted_flu.z};
   const Point3 ned = config.compensate_attitude && pose.attitude_valid
                          ? rotateFrdToNed(body_frd, pose.roll_rad, pose.pitch_rad,
                                           pose.yaw_rad + config.scan_yaw_offset_rad)
                          : rotatePlanar(body_frd, pose.yaw_rad + config.scan_yaw_offset_rad);
   projection.endpoint = {pose.position.x + range * ned.x,
                          pose.position.y + range * ned.y};
   projection.endpoint_altitude_m = origin_altitude_m - range * ned.z;
   ```

   Важно: текущие sign conventions зафиксировать тестами до включения в default
   config. Если конкретный Gazebo bridge angle convention окажется отличным от
   ROS FLU, компенсировать это через `lidar_mount_yaw_rad`, а не через
   non-physical XY swap.

3. Обновить construction of `LidarProjectionConfig` во всех call sites.

   Материализуемый результат:
   - `drone_city_nav/src/obstacle_memory.cpp:127` передает новые mount поля из
     `LaserScan2DView`/config;
   - `drone_city_nav/src/planner_node.cpp:811` передает новые node params;
   - `drone_city_nav/src/lidar_debug_node.cpp:410` передает новые node params;
   - все aggregate initializers остаются compile-safe и читаемыми.

4. Добавить ROS params для mount/extrinsics в узлы.

   Материализуемый результат:
   - в `drone_city_nav/src/obstacle_memory_node.cpp:96`,
     `drone_city_nav/src/planner_node.cpp:142`,
     `drone_city_nav/src/lidar_debug_node.cpp:174` объявить:
     `lidar_mount_roll_rad`, `lidar_mount_pitch_rad`, `lidar_mount_yaw_rad`;
   - добавить эти поля в startup logs:
     `obstacle_memory_node.cpp:206`, `planner_node.cpp:255`,
     `lidar_debug_node.cpp:275`;
   - в `drone_city_nav/config/urban_mvp.yaml` задать значения по умолчанию для
     simulation profile и убрать зависимость default path от
     `swap_lidar_xy_to_local_frame` для attitude-compensated режима;
   - если после unit tests для zero-yaw simulation нужно сохранить старую
     ориентацию лучей, сделать это через `lidar_mount_yaw_rad`, а не через swap.

5. Обновить `LaserScan2DView` и obstacle memory core contract.

   Материализуемый результат:
   - в `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` добавить
     mount fields в `LaserScan2DView`;
   - в `drone_city_nav/src/obstacle_memory_node.cpp:402` заполнить эти fields;
   - в `drone_city_nav/src/obstacle_memory.cpp:124` передать их в
     `LidarProjectionConfig`;
   - добавить unit coverage в `drone_city_nav/tests/obstacle_memory_test.cpp`,
     проверяющее, что tilted scan не создает diagonal phantom obstacle cells при
     known vertical-wall geometry.

6. Расширить `drone_city_nav/tests/lidar_projection_test.cpp:21`.

   Материализуемый результат:
   - добавить happy-path tests для zero tilt:
     `ExplicitFluToFrdProjectionKeepsLevelForwardBeam`,
     `ConfiguredMountYawReorientsLevelBeam`;
   - добавить negative-path/edge tests:
     `TiltedProjectionUsesBodyFrdAxesInsteadOfLegacySwap`,
     `AltitudeFilterRejectsGroundOnlyAfterFrameTransform`,
     `InvalidAttitudeFallsBackToLevelProjectionWhenCompensationUnavailable`;
   - сохранить тест на legacy no-swap behavior, но переименовать/уточнить его,
     чтобы было понятно, что он не является физическим 3D contract;
   - добавить тест, который был отсутствующим regression guard:
     `SwapCompatibilityDoesNotChangeTiltAxis`.

   Пример ключевого expectation:

   ```cpp
   TEST(LidarProjection, TiltedProjectionUsesBodyFrdAxesInsteadOfLegacySwap) {
     LidarProjectionPose pose{Point2{0.0, 0.0}, 18.0, 0.0,
                              0.25, -0.35, true, true};
     LidarProjectionConfig config{};
     config.compensate_attitude = true;
     config.swap_lidar_xy_to_local_frame = true;
     const auto projection = project(pose, config, 10.0F);
     EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
     EXPECT_NEAR(norm(projection.ned_direction), 1.0, 1.0e-9);
     EXPECT_GT(projection.endpoint_altitude_m, 1.0);
   }
   ```

   Финальные numeric expected values должны быть не loose smoke checks, а
   вычислены из выбранной matrix convention и зафиксированы через `EXPECT_NEAR`.

7. Расширить per-beam debug CSV в `drone_city_nav/src/lidar_debug_node.cpp:439`.

   Материализуемый результат:
   - CSV header добавить:
     `status,lidar_dir_x,lidar_dir_y,lidar_dir_z,body_frd_x,body_frd_y,body_frd_z,ned_x,ned_y,ned_z,depth_end_x_m,depth_end_y_m,depth_end_altitude_m`;
   - писать строку не только для accepted beams, но и для
     `AltitudeRejected`/`InvalidRange`/`InvalidScan`, чтобы в headless-режиме
     было видно, почему красного нет;
   - ограничить объем через существующий `beam_csv_stride`.

8. Расширить JSONL summary в `drone_city_nav/src/lidar_debug_node.cpp:710`.

   Материализуемый результат:
   - добавить `projection_config` object:
     `compensate_attitude`, `swap_lidar_xy_to_local_frame`,
     `scan_yaw_offset_rad`, `lidar_mount_roll_rad`,
     `lidar_mount_pitch_rad`, `lidar_mount_yaw_rad`,
     `projected_altitude_range`;
   - добавить `projection_stats` object:
     `accepted`, `hit`, `altitude_rejected`, `invalid_range`,
     `invalid_scan`, `endpoint_altitude_min`, `endpoint_altitude_max`;
   - оставить текущие поля `attitude`, `scan`, `remembered_hits` для обратной
     совместимости с текущими jq-командами.

9. Добавить headless analyzer script.

   Материализуемый результат:
   - создать `scripts/analyze_lidar_projection_snapshots.py`;
   - вход: `log/lidar_debug/snapshots.jsonl` и optional path к static map
     `drone_city_nav/worlds/generated_city.map2d`;
   - выход: human-readable summary + nonzero exit при явных ошибках:
     no snapshots, no accepted cruise hits, altitude rejection dominates at
     cruise altitude, remembered hits absent after cruise, projection config
     inconsistent across snapshots;
   - script не должен требовать GUI/Gazebo, только локальные files.

   Минимальные критерии для default headless run:

   ```text
   cruise snapshots: altitude >= 15.0
   accepted current hits: max(hits) > 0
   altitude rejection ratio at cruise: max(altitude_rejected / processed) < 0.75
   remembered hits after cruise: final remembered_hits > 0
   image_ok: true for every snapshot with grid_seen=true
   ```

10. Добавить tests для analyzer script.

    Материализуемый результат:
    - создать `scripts/tests/test_analyze_lidar_projection_snapshots.py` или
      аналогичный локальный pytest/unittest file;
    - покрыть:
      happy-path JSONL;
      empty JSONL;
      cruise snapshots with all beams altitude-rejected;
      missing remembered hits;
      malformed JSON line;
    - если в репозитории нет Python test runner, явно запускать через
      `python3 -m unittest discover scripts/tests`.

11. Обновить documentation.

    Материализуемый результат:
    - `docs/MVP_SIMULATION.md:183` описывает новые CSV/JSONL поля и команду
      analyzer script;
    - `README.md` при необходимости добавляет короткую ссылку на analyzer рядом
      с lidar debugging section;
    - `CPP_BEST_PRACTICES.md` не менять, если новые C++ правила не нужны.

12. Зафиксировать compatibility/cleanup policy.

    Материализуемый результат:
    - если `swap_lidar_xy_to_local_frame` остается в YAML, явно залогировать
      warning при `compensate_lidar_attitude=true && swap_lidar_xy_to_local_frame=true`,
      что это legacy mode и физически корректный путь должен использовать mount
      params;
    - preferred outcome для default `urban_mvp.yaml`: `swap_lidar_xy_to_local_frame:
      false`, mount params задают нужную ориентацию scan frame;
    - не удалять параметр сразу, чтобы не ломать старые configs без миграции.

13. После кода обновить test/build wiring.

    Материализуемый результат:
    - если добавлен Python analyzer test, добавить documented command в
      `README.md`/`CONTRIBUTING.md` или Makefile target, например
      `make test-scripts`;
    - если CMake-only достаточно, оставить C++ tests через существующий
      `drone_city_nav/CMakeLists.txt:150`;
    - не добавлять new dependency без явной необходимости.

# Verification plan

Минимальный implementation verification:

1. Format changed C++ files:

   ```bash
   ./scripts/format_cpp_changed.sh
   ```

2. Targeted C++ projection tests после build:

   ```bash
   /home/formi/.local/bin/runlim \
     docker run --rm --privileged --network host \
       --user "$(id -u):$(id -g)" \
       --group-add "$(getent group render | cut -d: -f3)" \
       --group-add "$(getent group video | cut -d: -f3)" \
       --env HOME="/tmp/drone-gazebo-home-$(id -u)" \
       --env XDG_RUNTIME_DIR="/tmp/drone-gazebo-runtime-$(id -u)" \
       --volume "$PWD:/workspace:rw" \
       --workdir /workspace \
       drone-gazebo-dev:latest \
       bash -lc 'set -euo pipefail; mkdir -p "$HOME" "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"; make build && ctest --test-dir build/drone_city_nav --output-on-failure -R "lidar_projection|obstacle_memory"'
   ```

3. Full repo-approved C++ quality:

   ```bash
   /home/formi/.local/bin/runlim \
     docker run --rm --privileged --network host \
       --user "$(id -u):$(id -g)" \
       --group-add "$(getent group render | cut -d: -f3)" \
       --group-add "$(getent group video | cut -d: -f3)" \
       --env HOME="/tmp/drone-gazebo-home-$(id -u)" \
       --env XDG_RUNTIME_DIR="/tmp/drone-gazebo-runtime-$(id -u)" \
       --volume "$PWD:/workspace:rw" \
       --workdir /workspace \
       drone-gazebo-dev:latest \
       bash -lc 'set -euo pipefail; mkdir -p "$HOME" "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"; ./scripts/check_cpp_quality.sh'
   ```

4. Python analyzer tests, если analyzer script добавлен:

   ```bash
   python3 -m unittest discover scripts/tests
   ```

5. Headless simulation validation:

   ```bash
   /home/formi/.local/bin/runlim \
     docker run --rm --privileged --network host \
       --user "$(id -u):$(id -g)" \
       --group-add "$(getent group render | cut -d: -f3)" \
       --group-add "$(getent group video | cut -d: -f3)" \
       --env HOME="/tmp/drone-gazebo-home-$(id -u)" \
       --env XDG_RUNTIME_DIR="/tmp/drone-gazebo-runtime-$(id -u)" \
       --volume "$PWD:/workspace:rw" \
       --workdir /workspace \
       drone-gazebo-dev:latest \
       bash -lc 'set -euo pipefail; mkdir -p "$HOME" "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"; HEADLESS=1 SMOKE_DURATION_S=90 LIDAR_DEBUG_DIR=/workspace/log/lidar_projection_fix ./scripts/run_city_mvp.sh'
   ```

6. Headless log analysis:

   ```bash
   python3 scripts/analyze_lidar_projection_snapshots.py \
     log/lidar_projection_fix/snapshots.jsonl \
     --static-map drone_city_nav/worlds/generated_city.map2d
   rg "MISSION_RESULT|LIDAR_DEBUG|Obstacle memory update|Planner grid sources" log/ros_city_mvp.log
   ```

Acceptance criteria:

- `lidar_projection_test` passes with new tilted-frame regression tests.
- `obstacle_memory_test` passes for tilted scan geometry.
- `./scripts/check_cpp_quality.sh` passes.
- Headless run produces `MISSION_RESULT success=true`.
- At cruise altitude red/current hits are present and not mostly
  `altitude_rejected`.
- Yellow remembered hits visually/statistically align with static map edges in
  generated PPM/CSV evidence.

# Testing strategy

## Category 1: без рефакторинга

Подходит только как emergency/minimal fix. Изменить формулу в
`lidar_projection.cpp` и добавить несколько tests в
`lidar_projection_test.cpp`. Риск высокий: можно снова получить частный sign fix
без ясного frame contract. Использовать только если нужно быстро доказать
конкретный знак/ось.

Автотесты:

- C++ unit tests для `projectLidarBeam`.
- Negative path для altitude rejection.
- Edge case для invalid attitude fallback.

## Category 2: лёгкий refactor, recommended

Основной план. Ввести маленькую internal frame-math прослойку в
`lidar_projection.cpp`, расширить config/result debug fields, обновить три
потребителя и YAML. Это ограничивает blast radius, но делает frame chain явной.

Автотесты:

- C++ unit tests на математику projection.
- C++ obstacle memory regression test на tilted scan.
- Python analyzer unit tests для JSONL/CSV headless diagnostics.
- Existing package tests через `ctest`.

## Category 3: тяжёлый refactor

Перевести projection на TF2/geometry transforms и полноценные ROS frame IDs
(`map`, `base_link`, `lidar_link`). Это наиболее правильно для реального дрона,
но сейчас слишком широкий change: новые зависимости, runtime TF contracts,
launch/config changes, больше точек отказа. Не делать в ближайшем bug-fix
раунде без отдельного решения.

Автотесты:

- Unit tests для transform adapters.
- Integration tests с mocked TF buffer.
- Headless simulation.
- Отдельные docs для real drone calibration.

# Что могло сломаться

- Поведение: lidar hits могут повернуться на 90/180 градусов, если mount yaw
  выбран неверно. Проверка: new projection tests + headless PPM/CSV alignment
  against static map.
- API/контракты: расширение `LidarProjectionConfig`/`LidarBeamProjection` может
  сломать aggregate initializers. Проверка: `make build` и compile warnings.
- Данные/log artifacts: CSV/JSONL header/schema изменятся. Проверка: analyzer
  tests и сохранение старых summary fields для обратной совместимости.
- Интеграции: planner и obstacle memory используют общий projection; неверная
  математика может создавать phantom obstacles или терять current lidar
  obstacles. Проверка: `obstacle_memory_test`, headless mission monitor,
  `MISSION_RESULT success=true`.
- Производительность/ресурсы: новые CSV поля и analyzer могут увеличить размер
  logs. Проверка: оставить `beam_csv_stride`, `max_snapshots`,
  `max_logged_hit_points`; headless run должен завершиться без заметного роста
  runtime.
- Реальный дрон: explicit mount params могут быть неверны для другого sensor
  mounting. Проверка: docs должны требовать calibration; simulation defaults не
  считать real-drone defaults.
- Legacy configs: `swap_lidar_xy_to_local_frame` меняет семантику или становится
  deprecated. Проверка: warning log и сохранение compatibility path без
  немедленного удаления параметра.

# Risks and tradeoffs

- Самый большой риск - перепутать Gazebo LaserScan FLU convention и PX4 FRD/NED
  convention. Поэтому fix должен сначала добавить deterministic unit tests с
  явными expected vectors, а не только смотреть GUI.
- Сохранение `swap_lidar_xy_to_local_frame` снижает риск поломать старые params,
  но оставляет технический долг. Компромисс: оставить флаг deprecated,
  залогировать warning, а default simulation перевести на explicit mount params.
- Headless analyzer не заменяет физическую валидацию, но закрывает текущую
  проблему: можно увидеть наличие current hits, altitude rejection и remembered
  hits без GUI.
- Полный TF2-refactor архитектурно чище, но слишком широк для текущего bug-fix.
  Его лучше делать отдельной задачей после стабилизации MVP.

# Open questions

- Нужно ли сразу менять `real_drone_template.yaml`, если он существует и тоже
  использует projection params, или оставить только documentation warning до
  отдельной hardware calibration задачи?
- Какой exact `lidar_mount_yaw_rad` нужен для default simulation после отказа
  от `swap_lidar_xy_to_local_frame`: это должен подтвердить первый unit-test +
  headless run. Блокером это не является, потому что mount yaw становится
  явным config parameter.
- Нужно ли считать отсутствие current red hits на земле ошибкой, если altitude
  filter намеренно отбрасывает ground projections? В плане acceptance criteria
  проверяют cruise altitude, а не takeoff/ground state.
