# Context/Task

Нужно исследовать баг в недавно добавленной фиче проекции данных лидара на
горизонтальную плоскость при наклонах дрона. Симптом из пользовательского
скриншота `/home/formi/Downloads/gazebo/Screenshot.png`: зеленая статическая
карта выглядит правильно, красный слой текущих lidar hits почти отсутствует,
желтый слой accumulated/remembered lidar hits кривой и не совпадает с зеленой
статической картой.

Задача этого раунда - investigation only: локально изучить код, историю и
артефакты, сформулировать причины/риски/следующие шаги. Production code в этом
раунде не менялся.

# Research questions

- Где реализована attitude-aware lidar projection?
- Какие узлы используют одну и ту же projection-логику?
- Почему красный current lidar слой может исчезать?
- Почему желтый remembered lidar слой может становиться косым и не совпадать со
  static map?
- Есть ли тесты, которые покрывают комбинацию `swap_lidar_xy_to_local_frame=true`
  и ненулевых roll/pitch?
- Какие изменения/коммиты ввели это поведение?

# Scope and constraints

- Исследовались только локальный workspace, локальные build/log artifacts,
  локальный checkout PX4 и локальный скриншот.
- SSH и HTTP-запросы не выполнялись.
- Notion policy в inbox: `optional`; конкретная Notion task в промпте не
  указана, поэтому Notion не читался.
- GitLab/MR в промпте не указаны, поэтому GitLab не читался.
- `.agent-io/inbox.txt` и `.agent-io/outbox.txt` являются transport files и не
  должны коммититься.

# Detected stack/profiles

- Основной workspace: ROS 2 Jazzy workspace.
- Основной пакет: `drone_city_nav`, C++/ament CMake package.
- Build runner: top-level `Makefile` над `colcon`.
- C++ profile применим, потому что есть `drone_city_nav/CMakeLists.txt`,
  top-level `Makefile`, `.cpp/.hpp`, `build/compile_commands.json`.
- Rust profile не применим: в target workspace не обнаружены `Cargo.toml`,
  `Cargo.lock` или `.rs` как часть проекта.
- Прочитанные обязательные профили:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

# Repo-approved commands found

Из локальных `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/dev_shell.sh` - вход в dev container.
- `make build` - approved build.
- `make test` - approved unit tests.
- `make quality` / `./scripts/check_cpp_quality.sh` - approved quality checks.
- `./scripts/format_cpp_changed.sh` - format changed C++ files only.
- `make sim-gui` / `./scripts/run_city_mvp.sh` - GUI simulation.
- `make sim-headless` / `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`
  - headless smoke validation.

Для этого investigation production C++ не менялся, поэтому C++ formatter был
не нужен для code files. Перед коммитом артефакта был запущен approved
`./scripts/check_cpp_quality.sh` в dev container.

# Sources checked

- `README.md`
- `CONTRIBUTING.md`
- `Makefile`
- `drone_city_nav/config/urban_mvp.yaml`
- `drone_city_nav/include/drone_city_nav/lidar_projection.hpp`
- `drone_city_nav/src/lidar_projection.cpp`
- `drone_city_nav/tests/lidar_projection_test.cpp`
- `drone_city_nav/src/obstacle_memory.cpp`
- `drone_city_nav/src/obstacle_memory_node.cpp`
- `drone_city_nav/src/planner_node.cpp`
- `drone_city_nav/src/lidar_debug_node.cpp`
- `external/PX4-Autopilot/msg/versioned/VehicleAttitude.msg`
- `build/px4/platforms/ros2/rosidl_adapter/px4/msg/VehicleAttitude.idl`
- Локальные runtime logs:
  - `log/ros_city_mvp.log`
  - `log/lidar_debug/snapshots.jsonl`
  - `log/lidar_debug_default_speed_stable/snapshots.jsonl`
  - `log/lidar_debug_current_only/snapshots.jsonl`
  - `log/lidar_debug_tilt_fix/snapshots.jsonl`
- Локальный screenshot:
  - `/home/formi/Downloads/gazebo/Screenshot.png`
- Git history:
  - `536469b Compensate lidar projection for vehicle attitude`
  - `7665837 Stabilize higher-speed city mission`
  - `75fc4ba Enable all obstacle sources by default`

# Evidence

Скриншот `/home/formi/Downloads/gazebo/Screenshot.png` визуально показывает:

- Зеленый static map слой ровный и совпадает с Manhattan grid.
- Желтые remembered hits рядом с первыми зданиями наклонены/сдвинуты
  относительно зеленых квадратов.
- Красный current lidar слой практически отсутствует.

Текущая конфигурация включает attitude compensation во всех трех местах:

- `drone_city_nav/config/urban_mvp.yaml:25-29` для `obstacle_memory_node`
- `drone_city_nav/config/urban_mvp.yaml:87-91` для `planner_node`
- `drone_city_nav/config/urban_mvp.yaml:167-171` для `lidar_debug_node`

Одна и та же shared projection-функция используется всеми потребителями:

- `ObstacleMemoryGrid::integrateScan` вызывает `projectLidarBeam`:
  `drone_city_nav/src/obstacle_memory.cpp:141-144`.
- Planner current-lidar overlay вызывает `projectLidarBeam`:
  `drone_city_nav/src/planner_node.cpp:884-888`.
- Lidar debug CSV/red/yellow points вызывают `projectScanBeam`, который
  делегирует в `projectLidarBeam`:
  `drone_city_nav/src/lidar_debug_node.cpp:421-429`,
  `drone_city_nav/src/lidar_debug_node.cpp:457-468`.

В `projectLidarBeam` beam angle сначала превращается в `scan_direction`, причем
`swap_lidar_xy_to_local_frame` меняет компоненты до применения roll/pitch:

- `drone_city_nav/src/lidar_projection.cpp:147-150`

Затем при `swap_lidar_xy_to_local_frame=true` yaw инвертируется отдельным
2D-хаком:

- `drone_city_nav/src/lidar_projection.cpp:151-153`

После этого roll/pitch из PX4 применяются к уже переставленному вектору:

- `drone_city_nav/src/lidar_projection.cpp:154-159`

Endpoint и depth endpoint считаются через `used_range * world_direction.x/y`:

- `drone_city_nav/src/lidar_projection.cpp:161-168`

Altitude filter применяет `world_direction.z` и может отвергнуть весь луч:

- `drone_city_nav/src/lidar_projection.cpp:170-179`

PX4 локально документирует `VehicleAttitude.q` как FRD body -> NED earth:

- `external/PX4-Autopilot/msg/versioned/VehicleAttitude.msg:10`
- `build/px4/platforms/ros2/rosidl_adapter/px4/msg/VehicleAttitude.idl:24-26`

Это не тот же самый контракт, что ручной `swap_lidar_xy_to_local_frame` в
2D map visualization. Сейчас код смешивает FRD/NED attitude, ручной XY swap,
инверсию yaw и z-up altitude filter без явной sensor extrinsic transform.

Runtime evidence из `log/ros_city_mvp.log`:

```text
LIDAR_DEBUG snapshot=snapshot_000001 ... altitude=-0.05 ... roll=0.004 pitch=-0.006
tilt=0.007 beams=1080 hits=0 altitude_rejected=1080 ...
```

`log/ros_city_mvp.log:58`, `:64`, `:67`, `:72`, `:74`, `:75` показывают,
что на ранних snapshots при отрицательной/почти нулевой altitude все 1080 beams
отбрасываются altitude filter. Это объясняет отсутствие красного слоя на земле
или после падения.

После взлета current hits появляются, но remembered layer уже строится по
attitude-aware projection. Пример:

```text
snapshot_000017 ... altitude=18.03 speed=2.45 ... roll=0.247 pitch=-0.632
tilt=0.679 beams=1080 hits=450 altitude_rejected=0 remembered_hits=245 ...
```

`log/ros_city_mvp.log:119` показывает, что remembered hits накапливались при
существенном pitch/tilt, то есть именно в режиме, где неверный frame transform
будет визуально искажать стены.

Позже mission monitor фиксирует clearance violation:

```text
MISSION_RESULT success=false reason='building clearance violation' ...
min_building_clearance=2.99 ...
```

Это `log/ros_city_mvp.log:144-146`.

После emergency stop/падения altitude становится отрицательной, roll около -pi,
и красный слой почти полностью пропадает:

```text
snapshot_000025 ... altitude=-1.88 ... roll=-3.130 pitch=0.147 tilt=3.133
beams=1080 hits=2 altitude_rejected=955 remembered_hits=465 ...
```

Это `log/ros_city_mvp.log:166` и последующие строки `:167`, `:170`, `:171`,
`:174`, `:178`.

Агрегированная статистика из `log/lidar_debug/snapshots.jsonl`:

```json
{
  "snapshots": 59,
  "max_tilt": 3.1331,
  "max_roll_abs": 3.1297,
  "max_pitch_abs": 0.7796,
  "max_altitude_rejected": 1080,
  "min_hits": 0,
  "max_hits": 450,
  "final_remembered": 465
}
```

Тестовое покрытие недостаточно:

- `ZeroTiltKeepsLegacySwapEndpoint` покрывает `swap_lidar_xy_to_local_frame`,
  но только при `roll=0`, `pitch=0`:
  `drone_city_nav/tests/lidar_projection_test.cpp:36-48`.
- `PitchChangesProjectedAltitude` покрывает pitch, но без XY swap:
  `drone_city_nav/tests/lidar_projection_test.cpp:50-61`.
- Нет теста для комбинации `swap_lidar_xy_to_local_frame=true` +
  nonzero roll/pitch + expected map endpoint.

# Evidence references

- `drone_city_nav/src/lidar_projection.cpp:23-31` - scan direction construction
  and XY swap.
- `drone_city_nav/src/lidar_projection.cpp:33-48` - roll/pitch/yaw rotation
  matrix.
- `drone_city_nav/src/lidar_projection.cpp:147-159` - beam angle, swap, yaw
  inversion, roll/pitch application.
- `drone_city_nav/src/lidar_projection.cpp:161-179` - endpoint/depth endpoint
  and altitude rejection.
- `drone_city_nav/src/obstacle_memory.cpp:124-144` - obstacle memory uses shared
  projection.
- `drone_city_nav/src/planner_node.cpp:800-819` - planner projection pose/config.
- `drone_city_nav/src/planner_node.cpp:884-900` - planner current lidar overlay
  uses shared projection endpoint.
- `drone_city_nav/src/lidar_debug_node.cpp:403-429` - debug projection pose/config
  and beam projection.
- `drone_city_nav/src/lidar_debug_node.cpp:439-480` - debug CSV and current red
  hit points.
- `drone_city_nav/src/lidar_debug_node.cpp:591-632` - remembered yellow hit
  memory.
- `drone_city_nav/config/urban_mvp.yaml:25-29`,
  `drone_city_nav/config/urban_mvp.yaml:87-91`,
  `drone_city_nav/config/urban_mvp.yaml:167-171` - attitude compensation enabled.
- `external/PX4-Autopilot/msg/versioned/VehicleAttitude.msg:10` - q is FRD body
  to NED earth.
- `drone_city_nav/tests/lidar_projection_test.cpp:36-72` - tests do not cover
  swapped frame with nonzero tilt.
- Commit `536469b4040cfa9b8d739d6089ce74d38ced06b5` - introduced shared
  attitude-compensated lidar projection.
- Command excerpt:
  `jq -s -r '{snapshots: length, max_tilt: ..., max_altitude_rejected: ...}' log/lidar_debug/snapshots.jsonl`
  returned `max_tilt=3.1331`, `max_altitude_rejected=1080`, `min_hits=0`.

# Findings

1. **Главная вероятная ошибка: frame transform смешан из 2D workaround и 3D
   attitude compensation.**

   До attitude compensation `swap_lidar_xy_to_local_frame` был 2D способом
   совместить Gazebo lidar scan с PX4/map axes. После коммита `536469b` этот
   swap начал применяться к 3D body-frame vector до roll/pitch. Это меняет то,
   к какой оси применяется roll и pitch. При нулевом tilt баг не виден, поэтому
   старые zero-tilt ожидания проходят, но при ускорении/наклонах стены становятся
   косыми.

2. **`VehicleAttitude.q` используется без явной проверки frame convention и
   sensor extrinsic.**

   PX4 сообщает quaternion как rotation from FRD body frame to NED earth.
   Текущий код дальше вручную инвертирует yaw при `swap=true`, но roll/pitch
   оставляет как есть. Это не полноценная 3D frame transform. Нужна единая
   матрица/кватернион `lidar_frame -> vehicle_frd -> ned/map` вместо смеси
   `swap_xy + negative yaw + raw roll/pitch`.

3. **Красный current lidar слой пропадает из-за altitude rejection.**

   В свежем логе первые snapshots имеют `hits=0` и `altitude_rejected=1080`.
   После падения/emergency stop снова почти все beams отбрасываются:
   `hits=2..16`, `altitude_rejected=841..955`. На скриншоте красного почти нет,
   что соответствует состоянию после collision/emergency или при некорректной
   altitude/attitude.

4. **Желтый remembered слой сохраняет уже накопленные ошибочные hit points.**

   `lidar_debug_node` накапливает `remembered_hit_points_` и затем рисует их
   желтым. Если projection ошиблась во время полета с roll/pitch, желтый слой
   продолжит показывать эти точки даже когда current red hits позже пропали.

5. **A* и obstacle memory получают тот же искаженный источник.**

   Это не только проблема визуализации. Planner current-lidar overlay и
   obstacle-memory mapper используют тот же `projectLidarBeam`, поэтому кривой
   yellow/debug слой является индикатором риска для planning grid. В проблемном
   run после накопления memory planner начал публиковать paths с clearance
   `0.50`, mission monitor зафиксировал `building clearance violation`.

6. **Тесты не ловят основной сценарий.**

   Нет unit test, который проверяет nonzero roll/pitch при
   `swap_lidar_xy_to_local_frame=true`, хотя именно такой режим включен в
   `urban_mvp.yaml`.

# Relevant code paths

- `lidar_projection.cpp` - shared projection math.
- `obstacle_memory.cpp` + `obstacle_memory_node.cpp` - persistent obstacle memory
  source and yellow/grid remembered obstacles.
- `planner_node.cpp` - current lidar overlay for A*.
- `lidar_debug_node.cpp` - red current hits, yellow remembered hits, CSV/PPM
  snapshots.
- `urban_mvp.yaml` - enables `swap_lidar_xy_to_local_frame=true` and
  `compensate_lidar_attitude=true` for all relevant nodes.

# Timeline/history

- Before `536469b`, lidar endpoints were effectively 2D: beam angle + yaw +
  optional XY swap.
- `536469b Compensate lidar projection for vehicle attitude` introduced:
  - shared `lidar_projection.hpp/.cpp`;
  - PX4 attitude subscriptions in planner/obstacle memory/debug;
  - projected altitude filtering;
  - tests for zero-tilt legacy behavior and pitch altitude rejection.
- Later commits improved stability, static map visualization and defaults, but
  did not change the core frame math in `projectLidarBeam`.
- `75fc4ba Enable all obstacle sources by default` re-enabled persistent
  obstacle memory by default, so any projection artifact again affects the
  default planner hard source.

# Hypotheses/alternatives

- **Primary hypothesis:** `swap_lidar_xy_to_local_frame` is being applied at the
  wrong stage. It should not mutate the body-frame beam before roll/pitch. The
  correct transform should explicitly model lidar frame, vehicle FRD frame and
  NED/map frame.
- **Alternative hypothesis:** `VehicleAttitude.q` is valid for FRD->NED, but the
  simulated lidar/link mount has an additional static rotation that is not
  represented. This would also require a configurable sensor extrinsic transform.
- **Alternative hypothesis:** Some screenshots/logs are after collision/emergency
  stop. That explains roll near -pi and red disappearance after the crash, but
  does not explain why remembered yellow walls already became skewed before the
  crash.
- **Alternative hypothesis:** The altitude filter is too strict for debug mode
  near takeoff/landing. This explains missing red at low altitude, but not the
  yellow-vs-green angular mismatch at cruise altitude.

# Risk/impact

- High for visualization: red/yellow lidar overlays can be misleading or absent.
- High for planning when `use_obstacle_memory=true` or
  `use_current_lidar_obstacles=true`: A* can plan around phantom/skewed obstacle
  cells or lose valid streets.
- High for debugging: logs may suggest lidar is broken when the immediate cause
  is projection rejection.
- Medium for real-drone portability: raw PX4 attitude use without explicit
  sensor extrinsics is not portable across vehicle/sensor mounting conventions.

# Conclusions

Текущая attitude-aware projection не является надежной. Основная проблема не в
static map: зеленый слой не проходит через lidar projection и поэтому выглядит
правильно. Баг локализуется в shared lidar projection pipeline и его frame
conventions.

Наиболее вероятная root cause: 2D `swap_lidar_xy_to_local_frame` был
механически встроен внутрь 3D body-to-world projection. При ненулевых roll/pitch
это применяет наклоны к неправильным осям. Дополнительно pipeline принимает PX4
FRD->NED quaternion без явного lidar mount transform и без sanity-gating
непригодных attitude/altitude состояний.

Красный слой исчезает по другой, но связанной причине: altitude filter массово
отбрасывает beams при низкой/отрицательной altitude или после падения. Это
подтверждено `altitude_rejected=1080` в логах.

# Recommendations/next steps

1. Исправить projection API вокруг явных frames:
   - lidar scan frame;
   - vehicle body FRD;
   - PX4 local NED/map frame;
   - static sensor extrinsic rotation/translation.
2. Убрать/заменить текущую смесь `swap_lidar_xy_to_local_frame` +
   `projection_yaw_rad = -yaw` + raw roll/pitch. Если нужен swap для legacy map,
   он должен быть представлен как явная матрица frame conversion, применяемая в
   правильном порядке.
3. Добавить unit tests:
   - `swap_lidar_xy_to_local_frame=true` + nonzero roll;
   - `swap_lidar_xy_to_local_frame=true` + nonzero pitch;
   - FRD->NED known vectors from PX4 quaternion;
   - lidar mount extrinsic cases;
   - altitude rejection only for intended ground hits.
4. Добавить debug logs/CSV columns:
   - raw quaternion `q[0..3]`;
   - world direction `(x,y,z)`;
   - endpoint altitude;
   - rejection reason counts split by current/red and remembered/yellow.
5. Для visualization можно временно показывать rejected current hits отдельным
   цветом или счетчиком, чтобы отсутствие красного не выглядело как отсутствие
   lidar messages.
6. До фикса geometry projection не стоит считать `use_obstacle_memory=true`
   безопасным дефолтом для mission validation: этот источник может закреплять
   ошибочные yellow/memory hits как hard planning obstacles.

# Verification plan

После исправления projection:

1. Запустить targeted unit tests:
   - `ctest --test-dir build/drone_city_nav --output-on-failure -R lidar_projection`
2. Запустить full package tests:
   - `make test`
3. Запустить quality:
   - `./scripts/check_cpp_quality.sh`
4. Запустить headless debug mission с отдельным `LIDAR_DEBUG_DIR`, затем проверить:
   - red current hits существуют на cruise altitude;
   - `altitude_rejected` не доминирует при cruise altitude;
   - yellow remembered hits совпадают с green static map edges;
   - `MISSION_RESULT success=true`;
   - нет `A* did not find a path` из-за memory artifacts.

# Testing/verification implications

Текущие тесты недостаточны, потому что они сохраняют legacy zero-tilt behavior,
но не фиксируют корректную геометрию для реального включенного профиля
`swap_lidar_xy_to_local_frame=true` + `compensate_lidar_attitude=true`.

Любой будущий fix должен сначала добавить failing tests для этого профиля, иначе
высок риск снова получить визуально правильный zero-tilt случай и сломанный
tilted-flight случай.

# Open questions

- Какой точный lidar sensor frame в Gazebo/PX4 model: forward/right/down или
  другой orientation?
- Есть ли статический sensor mount rotation между body FRD и lidar scan frame,
  который нужно явно параметризовать?
- Нужно ли planner использовать altitude-rejected current hits как unknown/free,
  или полностью игнорировать их?
- Должен ли debug node очищать remembered hits после emergency stop, crash или
  reset pose, чтобы старые желтые артефакты не путали диагностику?
