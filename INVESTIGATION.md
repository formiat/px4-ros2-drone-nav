# Context/Task

Это дополненное investigation по проблеме отображения дрона в Gazebo 3D world.
Базовый факт из уточнения пользователя принимается как проверенный:

- дрон и жёлтый маркер под дроном не рендерятся / не отображаются в окне
  Gazebo с 3D-миром;
- это не версия про маленький размер модели, невнимательность, неверную камеру,
  неверный старт/финиш или неверный момент времени;
- пользователь вручную летал камерой по миру, включая область старта и область
  пути из RViz, и не обнаружил модель в 3D-мире;
- при этом RViz/ROS/PX4 показывают, что дрон физически существует в симуляции,
  публикует телеметрию и выполняет миссию/движение.

Задача этого раунда: доисследовать реальную причину отсутствия рендера дрона в
окне Gazebo 3D world. В отличие от предыдущего раунда, GUI-запуск был разрешён
и выполнен.

# Research questions

- На какой стороне находится проблема: PX4/Gazebo server вообще не создаёт
  модель, или модель есть на server-side, но не отображается Gazebo GUI/render
  client?
- Разрешается ли текущий SDF в полноценную модель с links, sensors, mesh visuals
  PX4 `x500_base` и кастомными yellow marker visuals?
- Есть ли сохранённые command/log excerpts, подтверждающие `gz model --list`,
  `gz model -m ... -l`, `gz model -m ... -l link -s` и `gz topic -l`?
- Даёт ли GUI-прогон новые признаки render/EGL/resource проблемы?
- Можно ли подтвердить visual output screenshot-ом из agent-сессии?
- Что означает `FAIL: planner publishes a path` в старом headless `run.out`,
  если ROS log одновременно содержит `Published path`?

# Scope and constraints

- Workspace root: `/home/formi/Documents/CppProjects/drone-gazebo`
- Branch: `main`
- Стартовый HEAD этого раунда: `d0915c6 Revise render investigation evidence`
- GUI-запуск разрешён inbox-ом и выполнен локально.
- Удалённый доступ запрещён: SSH/HTTP на удалённые целевые системы не
  использовались.
- Notion policy: `optional`; Notion task id и GitLab MR в prompt отсутствуют,
  поэтому Notion/GitLab не читались.
- Изменяется только tracked-документ `INVESTIGATION.md`; transport files
  `.agent-io/inbox.txt` и `.agent-io/outbox.txt` не коммитятся.
- Screenshot из agent-сессии получить не удалось: `gnome-screenshot` упал на
  Wayland/X11 fallback, `import` не сделал image file; `wmctrl` и `xdotool` в
  окружении отсутствуют.

# Detected stack/profiles

Стек workspace:

- C++ ROS 2 workspace;
- `colcon` / ament CMake;
- Gazebo Sim / `gz`;
- PX4 SITL;
- shell/Python launch helpers;
- SDF-модели и Gazebo world assets.

Прочитанные обязательные инструкции:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

Почему выбран C/C++ profile: в репозитории есть `Makefile`, `CMakeLists.txt`,
`*.cpp`, `*.hpp`, ROS 2 package и `colcon` workflow.

Rust profile не применялся: команда `rg --files -g 'Cargo.toml' -g 'Cargo.lock'
-g '*.rs'` не нашла Rust manifest/source files.

# Repo-approved commands found

Repo-approved workflow найден в `AGENTS.md`, `README.md`, `CONTRIBUTING.md` и
`Makefile`.

Host workflow, приоритетный для agents:

- `make host-build`
- `make host-test`
- `make host-quality`
- `make host-format`
- `make host-sim-headless`
- `make host-sim-gui`

Container workflow:

- `./scripts/dev_shell.sh`
- внутри контейнера: `make build`, `make test`, `make quality`,
  `make sim-headless`, `make sim-gui`

В этом раунде использован native host workflow через
`./scripts/run_city_mvp_host.sh`, а также `gz` CLI через `./scripts/host_shell.sh`.

# Observed symptom

Проверенный пользователем симптом:

- В Gazebo 3D world не отображаются модель дрона и жёлтый ground marker.
- Это не объясняется направлением камеры, масштабом, невнимательностью,
  ошибкой старта/финиша или неверным моментом времени.
- Дрон существует в симуляции: PX4 spawn log, ROS telemetry/path logs и
  Gazebo transport queries подтверждают server-side модель.

Новые наблюдения из GUI-прогона:

- GUI-прогон стартует и Gazebo принимает follow-camera команды для
  `x500_lidar_2d_0`.
- State confirmation `/gui/currently_tracked` остаётся недоступным, поэтому
  accepted service calls нельзя считать доказательством корректной камеры.
- В Gazebo log присутствуют `libEGL warning` и `egl: failed to create dri2
  screen`.
- Launcher сейчас глушит stdout/stderr процесса `gz sim -g`, поэтому полные
  GUI-client render/resource logs не сохраняются.

# Immediate cause

Immediate cause на текущем уровне доказанности:

модель создаётся и существует на стороне Gazebo server, но проблема проявляется
после server-side entity creation: на границе Gazebo GUI/render-client,
visual-scene delivery, resource loading или локального EGL/render stack.

Почему это подтверждено:

- PX4 сообщает `Spawning Gazebo model` и `model: x500_lidar_2d_0`:
  `log-host/investigate_render_gui_20260619_000134/px4_city_mvp.log:23-25`.
- `gz model --list` в GUI-прогоне видит `x500_lidar_2d_0`:
  `log-host/investigate_render_gui_20260619_000134/gui_queries.txt:7-76`.
- `gz model -m x500_lidar_2d_0 -l` видит `base_link`:
  `log-host/investigate_render_gui_20260619_000134/gui_queries.txt:77-100`.
- `gz topic -l` видит sensor/model topics для `x500_lidar_2d_0`, включая lidar
  scan topics:
  `log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:1-33`.
- `/world/generated_city/pose/info` содержит moving pose для
  `x500_lidar_2d_0`:
  `log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:34-43`.
- `gz sdf -k` в предыдущем сохранённом headless evidence возвращал `Valid.`:
  `log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:3-14`.
- `gz sdf -p` в предыдущем сохранённом headless evidence показывал
  `base_link_visual`, x500 mesh URIs, rotor visuals,
  `yellow_drone_locator_core`, `yellow_ground_projection_disc`:
  `log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:21-46`.

Следовательно, текущие доказательства не поддерживают версии:

- PX4 не заспаунил модель;
- модель отсутствует в server-side state;
- отсутствует lidar link/sensor;
- SDF невалиден;
- старт/финиш перепутаны;
- пользователь просто не увидел модель.

Точная причина внутри GUI/render-client остаётся unresolved boundary: GUI был
запущен, но agent не смог получить screenshot, а launcher не сохраняет
stdout/stderr `gz sim -g`.

# Causal chain / why chain

1. Наблюдаемый симптом: дрон и ground marker не рендерятся в Gazebo 3D world.
2. Immediate technical cause: проблема не в отсутствии server-side модели.
   Модель `x500_lidar_2d_0` есть в Gazebo transport state, links/sensor/topics
   присутствуют, `/pose/info` публикует её положение.
3. Почему это произошло не на PX4/ROS уровне: PX4 spawn log подтверждает
   создание модели, ROS log подтверждает planner/path/offboard activity, а
   `gz model` подтверждает server-side model state.
4. Почему это сужает область до GUI/render: source SDF валиден и раскрывается в
   визуальную модель с mesh visuals и yellow marker visuals, но пользователь
   видит отсутствие рендера в 3D world. Значит сбой находится после SDF
   resolution и server-side entity creation.
5. Почему GUI/render stack теперь является более сильной гипотезой: GUI-прогон
   дал `libEGL warning` / `egl: failed to create dri2 screen` в Gazebo log, а
   именно EGL/render stack отвечает за визуализацию.
6. Почему точный GUI-client root cause ещё не доказан: `scripts/run_city_mvp.sh`
   запускает `gz sim -g > /dev/null 2>&1`, поэтому stderr/stdout GUI-клиента
   теряются; screenshot из agent-сессии снять не удалось.
7. Почему code-level model layout всё ещё подозрителен: проект использует
   локальные кастомные SDF-модели вместо upstream PX4 `x500_lidar_2d` и
   `lidar_2d_v2` (`scripts/run_city_mvp.sh:208-232`). В `bbcc6a3` marker
   visuals были перенесены из dedicated wrapper link в merged child lidar link.
   Это смешало sensor model и human-visibility marker.

# Evidence per causal link

| Звено causal chain | Evidence |
| --- | --- |
| Модель создаётся PX4/Gazebo | `px4_city_mvp.log:23-25`: spawn и `model: x500_lidar_2d_0`. |
| Gazebo server видит модель | `gui_queries.txt:7-76`: `Available models` заканчивается `x500_lidar_2d_0`. |
| Server-side links существуют | `gui_queries.txt:77-100`: `base_link` с parent `x500_lidar_2d_0`. Старый full headless dump также содержит rotors и lidar link. |
| Lidar/model topics существуют | `gui_scene_queries.txt:1-33`: `/model/x500_lidar_2d_0/...`, `/scan`, `/scan/points`. |
| Pose публикуется | `gui_scene_queries.txt:34-43`: `/pose/info` содержит `name: "x500_lidar_2d_0"` и позицию. |
| GUI follow target доступен service layer-у | `gz_city_mvp.log:21-26`: первый timeout, затем 3 accepted attempts. |
| Render stack выдаёт warning | `gz_city_mvp.log:9-18`: `libEGL warning` и `egl: failed to create dri2 screen`. |
| GUI-client logs неполные | `scripts/run_city_mvp.sh:400-402`: `gz sim -g > /dev/null 2>&1 &`. |
| Runtime использует локальные модели | `scripts/run_city_mvp.sh:208-232`: PX4 originals для `x500_lidar_2d`/`lidar_2d_v2` пропускаются, затем symlink на local models. |
| Yellow marker visuals находятся внутри lidar model | `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`. |
| SDF валиден и содержит visuals | `sdf_expand_excerpt.txt:14`, `sdf_expand_excerpt.txt:21-46`. |
| `FAIL: planner publishes a path` не означает отсутствие path | `run.out:22-23` даёт FAIL, но `ros_city_mvp.log:34`, `:52`, `:112`, `:145`, `:152`, `:172`, `:247` содержат `Published path`. |
| Причина FAIL в validator pattern | `scripts/validate_city_mvp_headless.py:232-235` ищет старый формат `Published path: waypoints=[1-9]`. |

# Root cause / unresolved boundary

Доказанная граница:

- дрон не отсутствует в симуляции;
- server-side model, links, sensor, topics и pose присутствуют;
- SDF валиден и содержит визуальные элементы;
- проблема находится после server-side model creation: Gazebo GUI/render-client,
  visual-scene delivery, resource loading или локальный EGL/render stack.

Новые GUI-evidence:

- `gz_city_mvp.log` содержит `libEGL warning` и `egl: failed to create dri2
  screen`;
- GUI follow camera service принимает target `x500_lidar_2d_0`, но не даёт
  reliable state confirmation;
- screenshot capture из agent-сессии не сработал;
- stdout/stderr GUI client сейчас явно отправлены в `/dev/null`, поэтому
  полная render-client диагностика отсутствует.

Нерешённая точная причина:

- неизвестно, получает ли GUI visual entries для `x500_lidar_2d_0`;
- неизвестно, fail происходит на mesh resource loading, material/render state,
  scene subscription, culling, merged-include visual transform, EGL backend или
  другой части GUI renderer.

Главный actionable system-level suspect:

- локальный EGL/render stack: `libEGL warning` / `egl: failed to create dri2
  screen`.

Главный actionable code-level suspect:

- commit `bbcc6a3 Attach drone marker to lidar visual link`;
- он удалил dedicated `visibility_marker_link` из
  `drone_city_nav/models/x500_lidar_2d/model.sdf`;
- он добавил marker visuals внутрь
  `drone_city_nav/models/lidar_2d_v2/model.sdf`;
- поэтому locator/ground marker теперь зависит от merged child lidar model.

Confidence:

- server-side model exists: high;
- boundary is after server-side creation: high;
- EGL/render stack contributes: medium;
- marker-in-lidar layout contributes to marker regression: medium;
- exact GUI root cause: unresolved.

# Sources checked

- `.agent-io/inbox.txt`
- `AGENTS.md`
- `README.md`
- `CONTRIBUTING.md`
- `Makefile`
- `INVESTIGATION.md` до правки
- `scripts/run_city_mvp.sh`
- `scripts/gazebo_gui_control.py`
- `scripts/validate_city_mvp_headless.py`
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`
- `log-host/investigate_render_headless_20260618_233648/run.out`
- `log-host/investigate_render_headless_20260618_233648/px4_city_mvp.log`
- `log-host/investigate_render_headless_20260618_233648/ros_city_mvp.log`
- `log-host/investigate_render_transport_20260618_234601/run.out`
- `log-host/investigate_render_transport_20260618_234601/px4_city_mvp.log`
- `log-host/investigate_render_transport_20260618_234601/ros_city_mvp.log`
- `log-host/investigate_render_transport_20260618_234601/transport_queries.txt`
- `log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt`
- `log-host/investigate_render_gui_20260619_000134/gz_city_mvp.log`
- `log-host/investigate_render_gui_20260619_000134/px4_city_mvp.log`
- `log-host/investigate_render_gui_20260619_000134/ros_city_mvp.log`
- `log-host/investigate_render_gui_20260619_000134/gui_queries.txt`
- `log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt`
- Git history around `fa88422`, `bbcc6a3`, `b4b9e8c`, `41bb70e`,
  `1d7be4a`, `2d054e2`, `718deec`, `cf8b9b1`, `9f0ff31`, `d0915c6`

# Evidence

Ключевые команды:

- `ENABLE_RVIZ=false SMOKE_DURATION_S=120 DRONE_GAZEBO_LOG_DIR=... ./scripts/run_city_mvp_host.sh`
- `gz model --list`
- `gz model -m x500_lidar_2d_0 -l`
- `gz topic -l`
- `gz topic -e -t /world/generated_city/pose/info`
- `gz topic -e -t /world/generated_city/scene/info`
- `gz sdf -k drone_city_nav/models/x500_lidar_2d/model.sdf`
- `gz sdf -p drone_city_nav/models/x500_lidar_2d/model.sdf | rg ...`

GUI-run evidence:

```text
log-host/investigate_render_gui_20260619_000134/px4_city_mvp.log:23-25
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: generated_city, model: x500_lidar_2d_0
```

```text
log-host/investigate_render_gui_20260619_000134/gui_queries.txt:16-76
Available models:
...
    - x500_lidar_2d_0
```

```text
log-host/investigate_render_gui_20260619_000134/gui_queries.txt:86-100
- Link [225]
  - Name: base_link
  - Parent: x500_lidar_2d_0 [224]
```

```text
log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:1-33
/gui/currently_tracked
/gui/track
/model/x500_lidar_2d_0/command/motor_speed
/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points
/world/generated_city/pose/info
/world/generated_city/scene/info
```

```text
log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:34-43
## scene info x500/visual excerpt
## pose info x500 excerpt
686-  }
688-pose {
689:  name: "x500_lidar_2d_0"
690-  id: 224
691-  position {
692-    x: 50.402766800818213
```

```text
log-host/investigate_render_gui_20260619_000134/gz_city_mvp.log:9-18
libEGL warning: pci id for fd 50: 10de:2520, driver (null)
libEGL warning: egl: failed to create dri2 screen
...
libEGL warning: egl: failed to create dri2 screen
```

```text
log-host/investigate_render_gui_20260619_000134/gz_city_mvp.log:21-26
Waiting for Gazebo GUI follow target 'x500_lidar_2d_0' (1/60): Service call timed out
Gazebo GUI follow camera command accepted: target=x500_lidar_2d_0 ...
WARNING: Gazebo GUI follow camera command accepted but state confirmation is unavailable
```

Mission-side signal from the same GUI run:

```text
telemetry_samples=209
speed_min=0.00 speed_mean=1.42 speed_max=8.58
local_clearance_min=0.58 local_clearance_mean=2.88 local_clearance_max=4.25
published_paths=5
```

The run reached goal-braking near the mission goal:

```text
log-host/investigate_render_gui_20260619_000134/ros_city_mvp.log:486+
distance_to_mission_goal=0.86
speed_limit_reason=goal
allowed_speed=0.00
```

SDF excerpts from the previous saved transport investigation:

```text
log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:14
Valid.

log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:23-46
<visual name='base_link_visual'>
model://x500_base/meshes/NXP-HGD-CF.dae
...
<visual name='yellow_drone_locator_core'>
<visual name='yellow_ground_projection_disc'>
```

Headless validator mismatch:

```text
scripts/validate_city_mvp_headless.py:232-235
result.require(
    "planner publishes a path",
    ros_log,
    r"Published path: waypoints=[1-9]",
)
```

Текущий log format содержит `Published path: reason=... waypoints=...`, поэтому
validator pattern не совпадает. Это отдельная диагностическая проблема, не
опровергающая server-side render investigation.

# Evidence references

- `scripts/run_city_mvp.sh:208-232`: runtime resources; локальные модели
  `x500_lidar_2d` и `lidar_2d_v2` подменяют PX4 originals.
- `scripts/run_city_mvp.sh:349-360`: `GZ_SIM_RESOURCE_PATH`.
- `scripts/run_city_mvp.sh:393-416`: Gazebo server/headless/GUI launch branch.
- `scripts/run_city_mvp.sh:400-402`: GUI client logs are redirected to
  `/dev/null`.
- `scripts/run_city_mvp.sh:421-435`: PX4 spawn env, включая `PX4_GZ_MODEL_POSE`.
- `scripts/gazebo_gui_control.py:146-156`: follow-state confirmation читает
  `/gui/currently_tracked` только `0.2s`.
- `scripts/gazebo_gui_control.py:159-228`: follow camera завершает best-effort
  без подтверждённого tracked state.
- `scripts/validate_city_mvp_headless.py:232-235`: stale regex для planner path.
- `drone_city_nav/models/x500_lidar_2d/model.sdf:3-15`: wrapper includes и
  `LidarJoint`.
- `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`: yellow marker visuals
  внутри lidar model.
- `drone_city_nav/models/lidar_2d_v2/model.sdf:153-178`: lidar sensor.
- `log-host/investigate_render_gui_20260619_000134/px4_city_mvp.log:23-25`:
  spawn и model name.
- `log-host/investigate_render_gui_20260619_000134/gui_queries.txt:7-76`:
  model list.
- `log-host/investigate_render_gui_20260619_000134/gui_queries.txt:77-100`:
  `base_link`.
- `log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:1-33`:
  GUI/model/sensor/scene topics.
- `log-host/investigate_render_gui_20260619_000134/gui_scene_queries.txt:34-43`:
  pose info for `x500_lidar_2d_0`.
- `log-host/investigate_render_gui_20260619_000134/gz_city_mvp.log:9-18`:
  EGL warnings.
- `log-host/investigate_render_gui_20260619_000134/gz_city_mvp.log:21-26`:
  follow-camera accepted but unconfirmed.
- `log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:14`,
  `:21-46`: SDF validity and visuals.
- `log-host/investigate_render_transport_20260618_234601/run.out:22-23`: timeout
  and validator fail.
- `log-host/investigate_render_transport_20260618_234601/ros_city_mvp.log:34`,
  `:52`, `:112`, `:145`, `:152`, `:172`, `:247`: actual `Published path`.
- `fa88422`: added dedicated `visibility_marker_link` in wrapper model.
- `bbcc6a3`: moved marker visuals into lidar visual link.
- `2d054e2`: introduced Python Gazebo GUI/control diagnostics.

# Findings

1. High confidence: дрон существует на стороне Gazebo server. Это подтверждают
   PX4 spawn log, `gz model --list` и `/pose/info`.
2. High confidence: server-side модель имеет ожидаемые links и lidar/model
   topics.
3. High confidence: SDF валиден и раскрывается в модель с mesh visuals и yellow
   marker visuals.
4. High confidence: текущая проблема относится к boundary после server-side
   entity creation, а не к PX4 spawn, ROS planner/offboard или отсутствию модели.
5. Medium confidence: EGL/render stack является реальным suspect, потому что
   GUI-прогон дал `libEGL warning` и `egl: failed to create dri2 screen`.
6. Medium confidence: `bbcc6a3` является code-level suspect для marker
   regression, потому что перенёс visual locator внутрь merged lidar child link.
7. Medium confidence: это не полностью объясняет отсутствие base x500 mesh
   visuals; значит точная render-причина остаётся unresolved boundary.
8. High confidence: `FAIL: planner publishes a path` в старом `run.out` является
   отдельным validator mismatch: текущий log format изменился, а regex остался
   старым. ROS log содержит реальные `Published path`.
9. High confidence: текущий launcher недостаточно логирует GUI-render проблемы,
   потому что `gz sim -g` пишет в `/dev/null`.

# Relevant code paths

- `scripts/run_city_mvp.sh`
  - подготовка runtime resources;
  - Gazebo server/headless/GUI launch;
  - GUI client stdout/stderr redirection;
  - PX4 spawn environment;
  - запуск validation в smoke/headless mode.
- `scripts/gazebo_gui_control.py`
  - follow camera service call;
  - follow offset service call;
  - `/gui/track` publication;
  - state confirmation.
- `scripts/validate_city_mvp_headless.py`
  - validator pattern, который сейчас не соответствует фактическому
    `Published path` log format.
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
  - wrapper для PX4 x500 + lidar.
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
  - custom lidar model;
  - текущие yellow marker visuals;
  - lidar sensor config.
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
  - upstream comparison.
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`
  - x500 mesh visual definitions.

# Timeline/history

- `fa88422 Add bright drone visibility markers`
  - добавил dedicated `visibility_marker_link` прямо в
    `x500_lidar_2d/model.sdf`.
- `bbcc6a3 Attach drone marker to lidar visual link`
  - удалил dedicated marker link из wrapper model;
  - добавил yellow marker и ground projection visuals в `lidar_2d_v2/link`.
- `b4b9e8c Add Gazebo GUI drone follow camera`
  - первая follow-camera реализация, которая ранее работала по наблюдениям
    пользователя.
- `41bb70e Keep Gazebo GUI simulation unpaused`
  - исторически добавлял GUI config path.
- `1d7be4a Restore original Gazebo follow camera launch`
  - убрал GUI config launch path.
- `2d054e2 Stabilize Gazebo launch diagnostics`
  - перевёл Gazebo GUI/control логику в `scripts/gazebo_gui_control.py`.
- `9f0ff31 Update Gazebo render investigation`
  - предыдущий investigation update.
- `d0915c6 Revise render investigation evidence`
  - предыдущий русскоязычный investigation update с headless/transport evidence.
- `log-host/investigate_render_gui_20260619_000134`
  - новый GUI-прогон с EGL warnings и transport evidence.

# Hypotheses/alternatives

## H1: GUI/render-client проблема с visual scene или resource loading

Confidence: medium-high.

Immediate cause: server-side модель существует, но GUI не рендерит результат.

Cause-of-cause: failure находится после `gz sim` server state и SDF expansion:
на стороне GUI scene subscription, render resource loading, material/render
state, culling или visual-scene transfer.

Evidence:

- `px4_city_mvp.log:23-25`
- `gui_queries.txt:7-100`
- `gui_scene_queries.txt:1-43`
- `sdf_expand_excerpt.txt:14`, `:21-46`

Что опровергнет:

- GUI/render diagnostics покажет, что visual entries вообще не отправляются в
  GUI scene;
- dedicated primitive marker attached to `base_link` тоже не появится;
- mesh resource errors появятся в GUI logs и укажут точную resource failure.

## H2: локальный EGL/render backend ломает отображение

Confidence: medium.

Immediate cause: Gazebo render path получает EGL errors.

Cause-of-cause: несовместимость/проблема локального EGL/driver/session backend
для Gazebo render client/server.

Evidence:

- `gz_city_mvp.log:9-18`: `libEGL warning`, `egl: failed to create dri2 screen`.

Ограничение evidence:

- из-за `gz sim -g > /dev/null 2>&1` неизвестно, какие warnings/errors были
  именно у GUI client;
- EGL warnings могут относиться к server/render sensor path, а не только к GUI.

Что подтвердит:

- сохранение GUI stderr/stdout покажет resource/render errors;
- запуск с другим render backend / GPU env изменит поведение;
- Gazebo GUI с captured logs покажет failure loading visuals.

## H3: marker regression из-за переноса visuals в merged lidar child link

Confidence: medium.

Immediate cause: yellow marker visuals больше не attached как отдельная link
wrapper-модели.

Cause-of-cause: commit `bbcc6a3` перенёс marker visuals в `lidar_2d_v2/link`,
который включается через `merge="true"`.

Почему это стало возможным: human-visibility marker и sensor model оказались
смешаны в одном SDF, хотя lidar model должна описывать sensor, а не внешний
locator для оператора.

Evidence:

- `fa88422` добавил dedicated marker link;
- `bbcc6a3` удалил его и добавил marker visuals в lidar model;
- `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`.

Что опровергнет:

- возвращение dedicated marker link не вернёт marker rendering;
- GUI logs покажут независимую render/resource причину.

## H4: `model://x500` vs bare `x500`

Confidence: low-medium.

PX4 upstream использует `<uri>x500</uri>`, локальный wrapper использует
`<uri>model://x500</uri>`. Однако текущий SDF раскрывается как `Valid`, а
server-side model создаётся и содержит expected links. Поэтому это не сильное
объяснение текущей проблемы.

Что опровергнет/подтвердит:

- временно заменить URI на upstream-style bare `x500` и сравнить GUI render;
- проверить GUI resource logs.

## H5: Follow camera failure

Confidence: real issue, but rejected as root cause for non-render.

Follow camera может быть сломана или ненадёжна, но это не объясняет отсутствие
дрона после ручного осмотра 3D world.

Evidence:

- `scripts/gazebo_gui_control.py:146-156`, `:159-228`;
- `gz_city_mvp.log:21-26`.

Что опровергнет как отдельную проблему:

- GUI follow camera стабильно подтверждает tracked state и follow behavior.

## H6: user/camera/scale/location mistake

Status: rejected.

Эта гипотеза противоречит уточнённому базовому факту и не должна использоваться
в выводах.

# Risk/impact

- Симуляция может быть функционально корректной, но непригодной для визуальной
  проверки в Gazebo 3D world.
- RViz остаётся полезным для planner/lidar/map debugging, но не заменяет
  Gazebo 3D validation.
- Если marker visuals остаются внутри lidar model, любые будущие изменения
  sensor model могут снова ломать human visibility aid.
- Ненадёжный follow camera может маскировать GUI/render проблему, но не должен
  подменять её диагностику.
- Headless validation сейчас имеет отдельный stale-regex bug по
  `planner publishes a path`; это может давать ложный FAIL в smoke runs.
- Пока GUI stderr/stdout теряются, render regressions будут плохо
  расследоваться.

# Conclusions

- Дрон не отсутствует в симуляции: server-side model `x500_lidar_2d_0` создана
  и доступна через Gazebo transport.
- Links, lidar sensor/model topics и pose info присутствуют.
- SDF валиден и содержит both x500 mesh visuals and yellow marker visuals.
- Поэтому текущий доказанный boundary: Gazebo GUI/render-client, visual-scene
  delivery, resource loading или EGL/render stack.
- Новый GUI-прогон добавил важную улику: `libEGL warning` и `egl: failed to
  create dri2 screen`.
- Точная причина всё ещё не доказана, потому что screenshot не удалось снять, а
  launcher глушит `gz sim -g` logs.
- Главный actionable system-level step: перестать глушить GUI logs и повторить
  GUI-прогон с сохранением stderr/stdout.
- Главный actionable code-level step: вынести locator visuals обратно в
  dedicated link в `x500_lidar_2d`, fixed к `base_link`, чтобы marker не зависел
  от lidar sensor model.
- Отдельно нужно поправить stale regex в `scripts/validate_city_mvp_headless.py`,
  потому что сейчас он не соответствует актуальному формату `Published path`.

# Recommendations/next steps

1. Изменить GUI launch logging: не отправлять `gz sim -g` в `/dev/null`, а
   писать в отдельный `gz_gui_city_mvp.log`.
2. Повторить GUI-прогон и проверить GUI log на mesh/material/resource/render
   errors.
3. Вернуть dedicated `visibility_marker_link` в
   `drone_city_nav/models/x500_lidar_2d/model.sdf`, fixed к `base_link`.
4. Убрать yellow locator / ground projection visuals из
   `drone_city_nav/models/lidar_2d_v2/model.sdf`; lidar model должна оставаться
   sensor model.
5. После правки выполнить GUI validation вручную: проверить, что дрон и marker
   реально отображаются в Gazebo 3D world.
6. Если marker появится, а x500 mesh нет, дальше расследовать mesh/material
   resource loading.
7. Если marker и mesh оба не появятся, дальше расследовать GUI scene
   subscription/render state и EGL backend.
8. Исправить headless validator regex:
   вместо `Published path: waypoints=[1-9]` учитывать текущий формат
   `Published path: reason=... waypoints=N`.
9. Follow camera чинить отдельно: сначала дождаться `gz model --list`, затем
   применять `/gui/follow`/offset, не считать best-effort accepted state
   полноценным подтверждением.

# Verification/falsification steps for findings

Уже выполнено:

- short headless run с отдельным log dir
  `log-host/investigate_render_transport_20260618_234601`;
- GUI run с отдельным log dir
  `log-host/investigate_render_gui_20260619_000134`;
- `gz model --list`;
- `gz model -m x500_lidar_2d_0 -l`;
- `gz model -m x500_lidar_2d_0 -l link -s` в предыдущем transport run;
- `gz topic -l`;
- `gz topic -e -t /world/generated_city/pose/info`;
- `gz topic -e -t /world/generated_city/scene/info`;
- `gz sdf -k`;
- `gz sdf -p` с `SDF_PATH`, включающим local и PX4 model dirs;
- проверка PX4/ROS logs на spawn/path/mission evidence;
- проверка validator regex.

Что проверить следующим изменением:

- GUI run с отдельным `gz_gui_city_mvp.log`;
- screenshot / ручное подтверждение actual render output пользователем;
- dedicated marker link attached to wrapper `base_link`;
- сравнение render behavior до/после выноса marker из lidar model.

# Follow-up verification implications

- Если после сохранения GUI logs появятся mesh/material/resource errors, нужно
  чинить resource path / material / model URI, а не planner/PX4.
- Если GUI logs чистые, но screenshot/ручная проверка всё ещё показывает пустой
  дрон, нужно проверять scene delivery/render state и возможно запуск Gazebo с
  другим render backend / GPU env.
- Если dedicated marker link появляется, а mesh дрона нет, marker regression и
  x500 mesh regression являются разными проблемами.
- Если dedicated marker link тоже не появляется, вероятнее system-level
  render/EGL или GUI scene issue.
- Если `gz scene/info` после улучшенного logging продолжит не давать visual
  entries для x500, нужно искать причину между server scene creation и GUI
  delivery.

# Open questions

- Какие exact stderr/stdout сообщения выдаёт `gz sim -g`, если не глушить их в
  `/dev/null`?
- Относятся ли текущие `libEGL warning` к GUI client, server render sensor path
  или обоим?
- Получает ли GUI visual entries для `x500_lidar_2d_0`?
- Исчезает ли проблема после возврата dedicated marker link в wrapper model?
- Исчезает ли проблема после запуска Gazebo с другим render backend / GPU env?
- Почему `gnome-screenshot` в agent-сессии не может снять Wayland/X11 fallback,
  хотя GUI-запуск разрешён?
