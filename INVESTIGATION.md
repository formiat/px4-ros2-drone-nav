# Context/Task

Это дополненное investigation по проблеме отображения дрона в Gazebo 3D world.
Базовый факт из уточнения пользователя принимается как проверенный:

- дрон и жёлтый маркер под дроном именно не рендерятся / не отображаются в окне
  Gazebo с 3D-миром;
- это не версия про маленький размер модели, невнимательность, неверную камеру,
  неверный старт/финиш или неверный момент времени;
- пользователь вручную летал камерой по миру, включая область старта и область
  пути из RViz, и не обнаружил модель в 3D-мире;
- при этом RViz/ROS/PX4 показывают, что дрон физически существует в симуляции,
  публикует телеметрию и выполняет миссию/движение.

Задача этого раунда: доисследовать реальную причину отсутствия рендера дрона в
окне Gazebo 3D world без GUI-запусков. Разрешены только headless-прогоны,
локальные логи, локальный код и локальные CLI.

# Research questions

- На какой стороне находится проблема: PX4/Gazebo server вообще не создаёт
  модель, или модель есть на server-side, но не отображается Gazebo GUI/render
  client?
- Разрешается ли текущий SDF в полноценную модель с links, sensors, mesh visuals
  PX4 `x500_base` и кастомными yellow marker visuals?
- Есть ли сохранённые command/log excerpts, подтверждающие `gz model --list`,
  `gz model -m ... -l`, `gz model -m ... -l link -s` и `gz topic -l`?
- Что означает `FAIL: planner publishes a path` в headless `run.out`, если ROS
  log одновременно содержит `Published path`?
- Какие гипотезы остаются рабочими, а какие нужно отвергнуть после уточнения
  пользователя?

# Scope and constraints

- Workspace root: `/home/formi/Documents/CppProjects/drone-gazebo`
- Branch: `main`
- Стартовый HEAD этого раунда: `9f0ff31 Update Gazebo render investigation`
- GUI не запускался, потому что inbox явно ограничил дополнительную проверку
  headless-режимом.
- Удалённый доступ запрещён: SSH/HTTP на удалённые целевые системы не
  использовались.
- Notion policy: `optional`; Notion task id и GitLab MR в prompt отсутствуют,
  поэтому Notion/GitLab не читались.
- Изменяется только tracked-документ `INVESTIGATION.md`; transport files
  `.agent-io/inbox.txt` и `.agent-io/outbox.txt` не коммитятся.

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

В этом раунде GUI-команды пропущены из-за headless-only ограничения. Для
проверки использовались `make`/script-backed headless launch и `gz` CLI через
`./scripts/host_shell.sh`.

# Observed symptom

Проверенный симптом:

- В Gazebo 3D world не отображаются модель дрона и жёлтый ground marker.
- Это не объясняется направлением камеры, масштабом, невнимательностью,
  ошибкой старта/финиша или неверным моментом времени.
- Дрон существует в симуляции: PX4 spawn log, ROS telemetry/path logs и
  Gazebo transport queries подтверждают server-side модель.

Отдельный вторичный симптом:

- Gazebo GUI follow camera принимает команды, но state confirmation недоступен.
- Это может мешать удобному просмотру, но не является допустимым объяснением
  отсутствия рендера после ручного осмотра 3D world.

# Immediate cause

Immediate cause на текущем уровне доказанности:

модель создаётся и существует на стороне Gazebo server, но проблема проявляется
на границе Gazebo GUI/render-client или visual-scene delivery/rendering.

Почему это подтверждено:

- PX4 сообщает `Spawning Gazebo model` и `model: x500_lidar_2d_0`:
  `log-host/investigate_render_transport_20260618_234601/px4_city_mvp.log:21-24`.
- `gz model --list` в headless run видит `x500_lidar_2d_0`:
  `log-host/investigate_render_transport_20260618_234601/transport_queries.txt:12-72`.
- `gz model -m x500_lidar_2d_0 -l` видит links `base_link`, `rotor_0`,
  `rotor_1`, `rotor_2`, `rotor_3`, `link`:
  `transport_queries.txt:84-86`, `transport_queries.txt:209-267`.
- `gz model -m x500_lidar_2d_0 -l link -s` видит sensor `lidar_2d_v2`:
  `transport_queries.txt:318-346`.
- `gz topic -l` видит sensor/model topics для `x500_lidar_2d_0`, включая lidar
  scan topics:
  `transport_queries.txt:363-390`.
- `gz sdf -k` возвращает `Valid.`:
  `sdf_expand_excerpt.txt:3-14`.
- `gz sdf -p` показывает `base_link_visual`, x500 mesh URIs, rotor visuals,
  `yellow_drone_locator_core`, `yellow_ground_projection_disc`:
  `sdf_expand_excerpt.txt:21-46`.

Следовательно, текущие доказательства не поддерживают версии:

- PX4 не заспаунил модель;
- модель отсутствует в server-side сцене;
- отсутствует lidar link/sensor;
- SDF невалиден;
- старт/финиш перепутаны;
- пользователь просто не увидел модель.

Точная причина внутри GUI/render-client остаётся unresolved boundary, потому что
в этом раунде GUI запускать нельзя.

# Causal chain / why chain

1. Наблюдаемый симптом: дрон и ground marker не рендерятся в Gazebo 3D world.
2. Immediate technical cause: проблема не в отсутствии server-side модели.
   Модель `x500_lidar_2d_0` есть в Gazebo transport state, links/sensor/topics
   присутствуют.
3. Почему это произошло не на PX4/ROS уровне: PX4 spawn log подтверждает
   создание модели, ROS log подтверждает planner/path/offboard activity, а
   `gz model` подтверждает server-side model state.
4. Почему это сужает область до GUI/render: source SDF валиден и раскрывается в
   визуальную модель с mesh visuals и yellow marker visuals, но GUI 3D window
   не отображает результат. Значит сбой находится после SDF resolution и
   server-side entity creation.
5. Почему такая ситуация стала возможной в текущем коде: проект использует
   локальные кастомные SDF-модели вместо upstream PX4 `x500_lidar_2d` и
   `lidar_2d_v2` (`scripts/run_city_mvp.sh:208-232`). В `bbcc6a3` marker
   visuals были перенесены из dedicated wrapper link в merged child lidar link.
   Это смешало sensor model и human-visibility marker.
6. Actionable root-cause candidate: visual locator должен жить в wrapper-модели
   `x500_lidar_2d` как отдельный fixed link к `base_link`, а не внутри
   `lidar_2d_v2/link`. Это не доказывает причину исчезновения mesh visuals
   `x500_base`, но это самая конкретная code-level точка, которую можно
   исправить и проверить.
7. Explicit unresolved boundary: без GUI/render introspection нельзя доказать,
   получает ли GUI visual entries и не рисует их, или visual entries не доходят
   до render-client.

# Evidence per causal link

| Звено causal chain | Evidence |
| --- | --- |
| Модель создаётся PX4/Gazebo | `px4_city_mvp.log:21-24`: world ready, pose, `Spawning Gazebo model`, `model: x500_lidar_2d_0`. |
| Gazebo server видит модель | `transport_queries.txt:12-72`: `Available models` заканчивается `x500_lidar_2d_0`. |
| Server-side links существуют | `transport_queries.txt:84-86`: `base_link`; `transport_queries.txt:209-267`: `rotor_0..3` и `link`. |
| Lidar sensor существует | `transport_queries.txt:318-346`: `link`, sensor `lidar_2d_v2`, range and scan config. |
| Sensor/model topics существуют | `transport_queries.txt:363-390`: command topics, `/world/.../lidar_2d_v2/scan`, `/scan/points`, scene/pose topics. |
| Runtime использует локальные модели | `scripts/run_city_mvp.sh:208-232`: PX4 originals для `x500_lidar_2d`/`lidar_2d_v2` пропускаются, затем symlink на local models. |
| Gazebo получает runtime model path | `scripts/run_city_mvp.sh:349-360`: `GZ_SIM_RESOURCE_PATH` указывает на runtime models/worlds. |
| PX4 spawn pose задаётся явно | `scripts/run_city_mvp.sh:421-435`: `PX4_GZ_MODEL_POSE` и `PX4_GZ_WORLD`. |
| Wrapper SDF включает x500 и lidar | `drone_city_nav/models/x500_lidar_2d/model.sdf:3-15`. |
| Yellow marker visuals находятся внутри lidar model | `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`. |
| SDF валиден и содержит visuals | `sdf_expand_excerpt.txt:14`, `sdf_expand_excerpt.txt:21-46`. |
| `FAIL: planner publishes a path` не означает отсутствие path | `run.out:22-23` даёт FAIL, но `ros_city_mvp.log:34`, `:52`, `:112`, `:145`, `:152`, `:172`, `:247` содержат `Published path`. |
| Причина FAIL в validator pattern | `scripts/validate_city_mvp_headless.py:232-235` ищет `Published path: waypoints=[1-9]`, а текущий log формат `Published path: reason=... waypoints=...`. |

# Root cause / unresolved boundary

Доказанная граница:

- дрон не отсутствует в симуляции;
- server-side model, links, sensor и topics присутствуют;
- SDF валиден и содержит визуальные элементы;
- проблема находится на стороне Gazebo GUI/render-client или доставки visual
  scene в render-client.

Нерешённая точная причина:

- headless mode не позволяет проверить фактическое состояние GUI render tree;
- неизвестно, получает ли GUI visual entries для `x500_lidar_2d_0`;
- неизвестно, fail происходит на mesh resource loading, material/render state,
  scene subscription, culling, merged-include visual transform или другой части
  GUI renderer.

Главный actionable suspect:

- commit `bbcc6a3 Attach drone marker to lidar visual link`;
- он удалил dedicated `visibility_marker_link` из
  `drone_city_nav/models/x500_lidar_2d/model.sdf`;
- он добавил marker visuals внутрь
  `drone_city_nav/models/lidar_2d_v2/model.sdf`;
- поэтому locator/ground marker теперь зависит от merged child lidar model.

Confidence: medium. Это объясняет исчезновение marker visuals лучше, чем
исчезновение всей базовой mesh-модели `x500_base`; поэтому точная render-причина
остаётся unresolved boundary.

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
- Git history around `fa88422`, `bbcc6a3`, `b4b9e8c`, `41bb70e`,
  `1d7be4a`, `2d054e2`, `718deec`, `cf8b9b1`, `9f0ff31`

# Evidence

Ключевые команды:

- `HEADLESS=1 ENABLE_RVIZ=false SMOKE_DURATION_S=50 DRONE_GAZEBO_LOG_DIR=log-host/investigate_render_transport_20260618_234601 ./scripts/run_city_mvp_host.sh`
- `gz model --list`
- `gz model -m x500_lidar_2d_0 -l`
- `gz model -m x500_lidar_2d_0 -l link -s`
- `gz topic -l | rg 'x500_lidar_2d_0|scene|pose|scan'`
- `gz sdf -k drone_city_nav/models/x500_lidar_2d/model.sdf`
- `gz sdf -p drone_city_nav/models/x500_lidar_2d/model.sdf | rg ...`

Сохранённые transport excerpts:

```text
log-host/investigate_render_transport_20260618_234601/transport_queries.txt:12
Available models:
...
log-host/investigate_render_transport_20260618_234601/transport_queries.txt:72
    - x500_lidar_2d_0
```

```text
log-host/investigate_render_transport_20260618_234601/transport_queries.txt:84-86
- Link [225]
  - Name: base_link
  - Parent: x500_lidar_2d_0 [224]

log-host/investigate_render_transport_20260618_234601/transport_queries.txt:209-267
- Link [243] / rotor_0
- Link [247] / rotor_1
- Link [251] / rotor_2
- Link [255] / rotor_3
- Link [259] / link
```

```text
log-host/investigate_render_transport_20260618_234601/transport_queries.txt:332-346
- Sensor [271]
  - Name: lidar_2d_v2
  - Parent: x500_lidar_2d_0 [224]
  - Range: Min 0.1, Max 30
  - Horizontal scan: Samples 720
```

```text
log-host/investigate_render_transport_20260618_234601/transport_queries.txt:376-381
/world/generated_city/model/x500_lidar_2d_0/link/base_link/sensor/...
/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points
```

SDF excerpts:

```text
log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:14
Valid.

log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:23-31
<visual name='base_link_visual'>
model://x500_base/meshes/NXP-HGD-CF.dae
<link name='rotor_0'>
<visual name='rotor_0_visual'>
model://x500_base/meshes/1345_prop_ccw.stl

log-host/investigate_render_transport_20260618_234601/sdf_expand_excerpt.txt:43-46
<link name='link'>
<visual name='yellow_drone_locator_core'>
<visual name='yellow_ground_projection_disc'>
<joint name='LidarJoint' type='fixed'>
```

Headless run signal:

```text
log-host/investigate_render_transport_20260618_234601/run.out:22-23
Headless run reached 50s timeout.
FAIL: planner publishes a path
```

Counter-evidence showing the planner did publish paths:

```text
log-host/investigate_render_transport_20260618_234601/ros_city_mvp.log:34
Published path: reason=computed_path waypoints=4 ...

log-host/investigate_render_transport_20260618_234601/ros_city_mvp.log:52
Published path: reason=computed_path waypoints=4 ...
```

Validator mismatch:

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
- `log-host/investigate_render_transport_20260618_234601/px4_city_mvp.log:21-24`:
  world ready, pose, spawn, model name.
- `log-host/investigate_render_transport_20260618_234601/transport_queries.txt:12-72`:
  model list.
- `log-host/investigate_render_transport_20260618_234601/transport_queries.txt:84-86`,
  `:209-267`: model links.
- `log-host/investigate_render_transport_20260618_234601/transport_queries.txt:318-346`:
  lidar link/sensor.
- `log-host/investigate_render_transport_20260618_234601/transport_queries.txt:363-390`:
  model/sensor/scene topics.
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
   PX4 spawn log и `gz model --list`.
2. High confidence: server-side модель имеет ожидаемые links и lidar sensor.
3. High confidence: модель публикует transport topics, включая lidar scan и
   scene/pose topics.
4. High confidence: SDF валиден при корректном `SDF_PATH` и раскрывается в
   модель с mesh visuals и yellow marker visuals.
5. Medium-high confidence: текущая проблема относится к GUI/render-client
   boundary, а не к PX4 spawn, ROS planner/offboard или отсутствию модели.
6. Medium confidence: `bbcc6a3` является главным code-level suspect для marker
   regression, потому что перенёс visual locator внутрь merged lidar child link.
7. Medium confidence: это не полностью объясняет отсутствие base x500 mesh
   visuals; значит точная render-причина остаётся unresolved boundary.
8. High confidence: `FAIL: planner publishes a path` в `run.out` является
   отдельным validator mismatch: текущий log format изменился, а regex остался
   старым. ROS log содержит реальные `Published path`.

# Relevant code paths

- `scripts/run_city_mvp.sh`
  - подготовка runtime resources;
  - Gazebo server/headless/GUI launch;
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
  - предыдущий investigation update; был полезен технически, но написан на
    английском и не содержал сохранённых CLI excerpts.

# Hypotheses/alternatives

## H1: GUI/render-client проблема с visual scene или resource loading

Confidence: medium-high.

Immediate cause: server-side модель существует, но GUI не рендерит результат.

Cause-of-cause: failure находится после `gz sim` server state и SDF expansion:
на стороне GUI scene subscription, render resource loading, material/render
state, culling или visual-scene transfer.

Evidence:

- `px4_city_mvp.log:21-24`
- `transport_queries.txt:12-72`
- `transport_queries.txt:84-86`, `:209-267`, `:318-346`, `:363-390`
- `sdf_expand_excerpt.txt:14`, `:21-46`

Что опровергнет:

- GUI/render diagnostics покажет, что visual entries вообще не отправляются в
  GUI scene;
- dedicated primitive marker attached to `base_link` тоже не появится;
- mesh resource errors появятся в GUI logs и укажут точную resource failure.

## H2: marker regression из-за переноса visuals в merged lidar child link

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

## H3: `model://x500` vs bare `x500`

Confidence: low-medium.

PX4 upstream использует `<uri>x500</uri>`, локальный wrapper использует
`<uri>model://x500</uri>`. Однако текущий SDF раскрывается как `Valid`, а
server-side model создаётся и содержит expected links. Поэтому это не сильное
объяснение текущей проблемы.

Что опровергнет/подтвердит:

- временно заменить URI на upstream-style bare `x500` и сравнить GUI render;
- проверить GUI resource logs.

## H4: Follow camera failure

Confidence: real issue, but rejected as root cause for non-render.

Follow camera может быть сломана или ненадёжна, но это не объясняет отсутствие
дрона после ручного осмотра 3D world.

Evidence:

- `scripts/gazebo_gui_control.py:146-156`, `:159-228`;
- старые GUI logs с `state confirmation is unavailable`.

Что опровергнет как отдельную проблему:

- GUI follow camera стабильно подтверждает tracked state и follow behavior.

## H5: user/camera/scale/location mistake

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

# Conclusions

- Дрон не отсутствует в симуляции: server-side model `x500_lidar_2d_0` создана
  и доступна через Gazebo transport.
- Links, lidar sensor и topics присутствуют.
- SDF валиден и содержит both x500 mesh visuals and yellow marker visuals.
- Поэтому текущий доказанный boundary: Gazebo GUI/render-client или доставка
  visual scene в render-client.
- Главный actionable code-level suspect: marker/locator visuals находятся не в
  wrapper-модели дрона, а внутри merged `lidar_2d_v2/link`.
- Следующий инженерный шаг: вынести locator visuals обратно в dedicated link в
  `x500_lidar_2d`, fixed к `base_link`, и затем выполнить GUI-проверку.
- Отдельно нужно поправить stale regex в `scripts/validate_city_mvp_headless.py`,
  потому что сейчас он не соответствует актуальному формату `Published path`.

# Recommendations/next steps

1. Вернуть dedicated `visibility_marker_link` в
   `drone_city_nav/models/x500_lidar_2d/model.sdf`, fixed к `base_link`.
2. Убрать yellow locator / ground projection visuals из
   `drone_city_nav/models/lidar_2d_v2/model.sdf`; lidar model должна оставаться
   sensor model.
3. Добавить простой primitive marker directly attached to the drone wrapper, not
   to the lidar child link.
4. После правки выполнить GUI validation вручную: проверить, что дрон и marker
   реально отображаются в Gazebo 3D world.
5. Если marker появится, а x500 mesh нет, дальше расследовать mesh/material
   resource loading.
6. Если marker и mesh оба не появятся, дальше расследовать GUI scene
   subscription/render state.
7. Исправить headless validator regex:
   вместо `Published path: waypoints=[1-9]` учитывать текущий формат
   `Published path: reason=... waypoints=N`.
8. Follow camera чинить отдельно: сначала дождаться `gz model --list`, затем
   применять `/gui/follow`/offset, не считать best-effort accepted state
   полноценным подтверждением.

# Verification/falsification steps for findings

Уже выполнено в headless mode:

- short headless run с отдельным log dir
  `log-host/investigate_render_transport_20260618_234601`;
- `gz model --list`;
- `gz model -m x500_lidar_2d_0 -l`;
- `gz model -m x500_lidar_2d_0 -l link -s`;
- `gz topic -l`;
- `gz sdf -k`;
- `gz sdf -p` с `SDF_PATH`, включающим local и PX4 model dirs;
- проверка PX4/ROS logs на spawn/path evidence;
- проверка validator regex.

Оставшаяся falsification, невозможная в этом раунде из-за headless-only
ограничения:

- GUI run после model cleanup;
- проверка фактического render output в Gazebo 3D world;
- сбор GUI/render logs с resource/scene diagnostics.

# Follow-up verification implications

- Passing headless mission не доказывает GUI rendering.
- `gz model` доказывает server-side model state, но не доказывает, что GUI
  renderer нарисовал visual nodes.
- Следующая правка model SDF должна обязательно завершаться GUI-проверкой, иначе
  render boundary останется открытым.
- Stale headless validator regex нужно чинить отдельно, иначе smoke-run может
  возвращать ложный FAIL даже при наличии опубликованных paths.

# Open questions

- Получает ли Gazebo GUI visual entries для `x500_lidar_2d_0`, или они теряются
  до render-client?
- Если dedicated primitive marker вернуть в wrapper model, появится ли он в 3D
  world?
- Если marker появится, почему не отображались/не отображаются x500 mesh visuals?
- Есть ли в GUI/render logs mesh/material/resource errors, которых не видно в
  headless server logs?
- Влияет ли `/gui/track` publication на follow-camera state, или follow-camera
  проблема целиком связана с timing/state confirmation?
