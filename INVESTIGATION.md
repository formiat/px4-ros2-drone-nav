# Context/Task

> **Обновлено (второй раунд):** добавлены находки по bbcc6a3, анализ merge=true+joint конфликта,
> данные из реальных лог-файлов и уточнённые root cause гипотезы для Bug #1.

Два регрессионных бага на ветке `main`:

1. **Bug #1**: 3D-модель дрона не отображается в окне Gazebo 3D (не в RViz). Дрон физически существует, летит и добирается до финиша — это видно в RViz.
2. **Bug #2**: Следящая камера (follow camera) в Gazebo GUI не работает / не запускается. Вместо неё остаётся активной дефолтная свободная камера.

Контекст истории: на коммите `718deec` дрон был виден. На `b4b9e8c` следящая камера работала. Несколько промежуточных коммитов пытались починить оба бага — безуспешно.

# Research questions

- Почему 3D-модель дрона не отображается в Gazebo GUI при текущем коде на `main`?
- Почему следящая камера не активируется в текущей реализации?
- Связаны ли Bug #1 и Bug #2 между собой (следствие или независимые)?
- Что конкретно изменилось относительно рабочих коммитов (`718deec`, `b4b9e8c`)?
- Что уже было попытано в последних коммитах, чтобы не повторять те же подходы?

# Scope and constraints

- Исследован только локальный код, git-история и локальные CLI/файлы.
- SSH, HTTP, удалённые системы не использовались.
- Notion policy: `optional`; Notion task id в промпте не указан — Notion не читался.
- GitLab/MR в промпте не указан — GitLab не читался.
- Симуляция не запускалась; выводы основаны на diff/code evidence и подтверждённых пользователем good/bad коммитах.

# Detected stack/profiles

- Стек: ROS 2 workspace (ament CMake / colcon), Gazebo Sim (gz-sim), PX4 SITL, C++.
- Прочитаны обязательные профили:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- Rust profile не применялся: затронутые файлы — shell-скрипты, SDF-модели, Python-скрипт.
- Протоколы:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

# Repo-approved commands found

Из `AGENTS.md`, `README.md`, `Makefile`:

- `make host-build` — сборка на хосте
- `make host-test` — тесты на хосте
- `make host-quality` — качество кода
- `make host-format` — форматирование
- `make host-sim-gui` — запуск симуляции с GUI
- `make host-sim-headless` — headless-запуск

Для данного investigation-раунда GUI и simulation команды не запускались — задача исследовательская. Verification команды ограничены `git`, `grep`, `cat`.

# Observed symptom

## Bug #1: Дрон не отображается в Gazebo 3D

- `718deec`: 3D-модель дрона в Gazebo GUI отображается корректно. Следящей камеры нет.
- Текущий `main`: 3D-модель дрона отсутствует в Gazebo GUI. Дрон физически существует (виден в RViz).
- Spawn координаты: `x=-57, y=-27, z=0.3` (неизменны с `718deec`).

## Bug #2: Следящая камера не работает

- `b4b9e8c`: следящая камера (`/gui/follow` service) работает.
- `41bb70e`: следящая камера сломалась (добавлен `--gui-config`).
- Текущий `main`: следящей камеры по-прежнему нет. Дефолтная свободная камера активна.

# Immediate cause

## Bug #1

На текущем `main` `prepare_runtime_resources` в `scripts/run_city_mvp.sh` (строки 208–232) всегда подменяет PX4-модели `x500_lidar_2d` и `lidar_2d_v2` на кастомные из `drone_city_nav/models/`. Текущая кастомная `x500_lidar_2d/model.sdf` — include-based (17 строк), без inline визуалов. Визуальные маркеры (жёлтая сфера, arms) — в `lidar_2d_v2/model.sdf`, привязаны к lidar link (link). Дрон спавнится в координатах `(-57, -27, 0.3)`, что далеко от дефолтной позиции камеры в Gazebo GUI (окрестности мирового origin). Без работающей следящей камеры (Bug #2) камера не направлена на дрон, и он не виден в viewport.

Дополнительная возможность: кастомная `x500_lidar_2d/model.sdf` использует `<uri>model://x500</uri>` с `merge="true"`, тогда как PX4-оригинал использует `<uri>x500</uri>`. Если merge по `model://x500` в текущей среде работает иначе (или вообще не работает), visual-компонент quadcopter frame (mesh из `x500_base/meshes/`) может не рендериться. В таком случае видна только жёлтая сфера из `lidar_2d_v2`, которую пользователь не считает «3D-моделью дрона».

## Bug #2

`configure_follow_camera` в `scripts/gazebo_gui_control.py` (строки 159–246) завершает работу после `required_accepted_attempts=3` с предупреждением «best-effort», даже если `_confirm_tracking` возвращает `None`. Функция вызывает `/gui/follow` service на каждой итерации; `/gui/follow` может возвращать `data: true` до того, как модель `x500_lidar_2d_0` реально появилась в сцене (или сразу после — но до подтверждения state). `_confirm_tracking` читает топик `/gui/currently_tracked` с duration 0.2s — слишком мало для надёжного ответа.

# Causal chain / why chain

## Bug #1: Дрон не виден в Gazebo 3D

**Observed symptom:**
Gazebo GUI не показывает 3D-модель дрона. Дрон летит (видно в RViz).

**Immediate technical cause:**
Gazebo GUI-камера направлена на area вблизи world origin (0, 0), а дрон находится в `-57, -27, 0.3`. Без следящей камеры дрон находится вне viewport с дефолтной позиции.

**Why that happened:**
Follow camera (Bug #2) не активируется, поэтому камера остаётся в дефолтной позиции Gazebo GUI, не направленной на spawn area дрона.

**Why that condition was possible:**
`prepare_runtime_resources` (e998cb9) сделал использование кастомных моделей безусловным, убрав `if [[ -n "${headless}" ]]`. При этом кастомные визуальные маркеры (жёлтая сфера) были перемещены в `lidar_2d_v2` (bbcc6a3), a `x500_lidar_2d/model.sdf` стал чисто include-based без собственных inline визуалов. Рендеринг дрона целиком зависит от (a) работы `model://x500` merge и (b) follow camera.

**Actionable root cause / unresolved boundary:**
- Root cause Bug #1 = последствие Bug #2 (camera не следит за дроном → дрон вне viewport).
- Дополнительный independent component (неподтверждённый без live-run): `model://x500` merge через `<uri>model://x500</uri>` в кастомной `x500_lidar_2d/model.sdf` может вести себя иначе, чем bare `<uri>x500</uri>` в PX4-оригинале. Если merge не даёт visual-компоненты из x500_base (meshes), пользователь не видит quadcopter frame. Жёлтая сфера из lidar_2d_v2 должна быть видна, но не воспринимается как «дрон».

## Bug #2: Следящая камера не работает

**Observed symptom:**
Камера остаётся в дефолтном свободном режиме. Follow camera не активна.

**Immediate technical cause:**
`configure_follow_camera` в `gazebo_gui_control.py:159–246` завершает работу по `accepted_attempts >= required_accepted_attempts (3)` раньше, чем подтверждается реальное слежение.

**Why that happened:**
`_confirm_tracking` (`gazebo_gui_control.py:146–156`) читает `/gui/currently_tracked` с `-d 0.2` (0.2s duration). Если топик не публикуется в течение 0.2s (недоступен в данной версии Gazebo или слишком медленный), функция возвращает `None`. При `tracking_state is None` и `accepted_attempts < 3` цикл продолжается и на следующей итерации снова вызывает `/gui/follow`. После 3 принятых ответов (`data: true`) код выходит с "best-effort", не зная, реально ли камера следит. `/gui/follow` может возвращать `data: true` при принятии команды, даже если камера фактически не перешла в режим слежения (например, из-за того что модель ещё не заспавнена).

**Why that condition was possible:**
Оригинальная bash-реализация (b4b9e8c) требовала только ОДНОГО `data: true` ответа от `/gui/follow` и сразу возвращала 0. В коммите 2d054e2 она была заменена Python-скриптом с добавленными проверками: `required_accepted_attempts=3` и `_confirm_tracking`. Это усложнило логику без гарантии надёжности — подтверждение через `/gui/currently_tracked` ненадёжно при 0.2s timeout.

Дополнительный фактор (d169f72): после каждого успешного вызова `/gui/follow` публикуется `gz.msgs.CameraTrack` в `/gui/track` (`_publish_track`, `gazebo_gui_control.py:115–143`). Если этот топик конкурирует с `/gui/follow` service или отправляется до реального появления модели, это может сбивать состояние камеры.

**Actionable root cause:**
Функция `configure_follow_camera` в Python-реализации не воспроизводит семантику оригинального bash-кода. Оригинал: один `data: true` → успех. Текущий: требует трёх принятий + недостижимого state confirmation. Это приводит к тому, что функция «успешно» завершается (return 0) по "best-effort" пути, не гарантируя активацию слежения. Всё это происходит, пока `gz sim -g` уже запущен без `--gui-config` (что само по себе правильно — откат 1d7be4a был верным).

# Evidence per causal link

## Bug #1

| Звено | Evidence |
|-------|----------|
| Кастомные модели всегда используются | `git show e998cb9 -- scripts/run_city_mvp.sh`: убраны строки `if [[ -n "${headless}" && ( "${model_name}" == "x500_lidar_2d" || ... ) ]]` и `if [[ -n "${headless}" ]]; then ln -s ...`; `scripts/run_city_mvp.sh:208–232` (current HEAD) |
| x500_lidar_2d — чисто include-based, без inline визуалов | `drone_city_nav/models/x500_lidar_2d/model.sdf:1–17` |
| Желтые маркеры — в lidar_2d_v2 | `drone_city_nav/models/lidar_2d_v2/model.sdf:77–152` — `yellow_drone_locator_core` (sphere r=0.32), `yellow_drone_locator_arm_x/y` (cylinders), `yellow_ground_projection_beam` |
| x500 использует mesh-based visuals из x500_base | `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`: `model://x500_base/meshes/NXP-HGD-CF.dae`, `model://x500_base/meshes/5010Base.dae` |
| PX4-оригинал x500_lidar_2d использует bare `x500` URI | `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf:5`: `<uri>x500</uri>` (без `model://`) |
| Кастомный использует `model://x500` | `drone_city_nav/models/x500_lidar_2d/model.sdf:5`: `<uri>model://x500</uri>` |
| Spawn координаты неизменны с 718deec | `scripts/run_city_mvp.sh:122–124`: `spawn_x_m="${SIM_START_X_M:--57}"`, одинаково на 718deec и HEAD |
| World SDF без GUI camera | `drone_city_nav/worlds/generated_city.sdf:1–100`: нет `<gui>` секции с camera pose |

## Bug #2

| Звено | Evidence |
|-------|----------|
| Оригинальный bash: один `data: true` → return 0 | `git show b4b9e8c:scripts/run_city_mvp.sh`: `configure_gazebo_gui_follow_camera` — if `grep -q "data: true"` → echo + `return 0` |
| Текущий Python: `required_accepted_attempts=3` | `scripts/gazebo_gui_control.py:164`: `required_accepted_attempts: int = 3` |
| `_confirm_tracking` с 0.2s timeout | `scripts/gazebo_gui_control.py:146–156`: `["topic", "-e", "-t", "/gui/currently_tracked", "-d", "0.2"]`, timeout=1.0 |
| Best-effort exit при accepted=3 | `scripts/gazebo_gui_control.py:223–228`: `if accepted_attempts >= required_accepted_attempts: print("WARNING: ...best-effort."); return 0` |
| `_publish_track` вызывается после каждого accepted | `scripts/gazebo_gui_control.py:199`: `track_response = _publish_track(...)` — вызов внутри блока `if response_is_true(follow_response)` |
| Python-делегация появилась в 2d054e2 | `git show 2d054e2 -- scripts/run_city_mvp.sh`: `+  python3 "${repo_root}/scripts/gazebo_gui_control.py"` для обоих вызовов |
| `--gui-config` добавлен в 41bb70e | `git show 41bb70e -- scripts/run_city_mvp.sh`: `gz_gui_args+=(--gui-config "${gazebo_gui_config_file}")` |
| `--gui-config` убран в 1d7be4a | `git show 1d7be4a -- scripts/run_city_mvp.sh`: удалена вся секция `find_default_gazebo_gui_config`, `prepare_gazebo_gui_config`, `--gui-config` args |
| Текущий GUI запуск без `--gui-config` | `scripts/run_city_mvp.sh:401`: `gz sim -g > /dev/null 2>&1 &` |
| `configure_world_running` параллелен с follow | `scripts/run_city_mvp.sh:403–411`: оба запущены через `&` |

# Root cause / unresolved boundary

## Bug #1

**Root cause (actionable):** Bug #1 на текущем `main` — прямое следствие Bug #2. Без следящей камеры Gazebo GUI-камера остаётся в дефолтной позиции (вблизи world origin), а дрон спавнится в `(-57, -27, 0.3)` — вне viewport.

**Дополнительный independent root cause (unresolved boundary):** Кастомная `x500_lidar_2d/model.sdf` использует `<uri>model://x500</uri>` с `merge="true"`, а PX4-оригинал — `<uri>x500</uri>`. Если в текущей среде gz-sim это различие приводит к разному resolution поведению, visual mesh quadcopter frame может не загружаться. В этом случае жёлтая сфера из `lidar_2d_v2` (`drone_city_nav/models/lidar_2d_v2/model.sdf:77`) видна, но пользователь её не воспринимает как «3D-модель дрона». Для закрытия этой boundary нужен live-run с `gz model --model-name x500_lidar_2d_0 --link` и скриншот.

## Bug #2

**Root cause (actionable):** `configure_follow_camera` в `scripts/gazebo_gui_control.py:159–246` перешёл от семантики «один успешный ответ сервиса → выход» к семантике «три принятых + state confirmation». Confirmation через `/gui/currently_tracked` с 0.2s timeout ненадёжна. Результат: функция выходит по best-effort после 3 принятых вызовов, но реальное слежение камеры не гарантировано.

**Unresolved boundary:** Точное поведение `/gui/follow` при вызове до/после спавна модели (возвращает ли `data: true` для несуществующей модели) без live-прогона с логами `gz_city_mvp.log` нельзя доказать окончательно. Также неизвестно, влияет ли `_publish_track` (публикация в `/gui/track`) конкурентно с `/gui/follow` service на финальное состояние камеры.

# Sources checked

- `.agent-io/inbox.txt`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- `AGENTS.md`, `README.md`, `CONTRIBUTING.md`, `Makefile`
- `scripts/run_city_mvp.sh` (HEAD и коммиты: 718deec, cf8b9b1, fa88422, e998cb9, b4b9e8c, 41bb70e, 5e006b2, 0e838e1, bbcc6a3, b53d43c, 1d7be4a, d169f72, 6a3638f, 2d054e2)
- `scripts/gazebo_gui_control.py`
- `scripts/gazebo_process_cleanup.py`
- `drone_city_nav/models/x500_lidar_2d/model.sdf` (HEAD и коммиты: 718deec, cf8b9b1, 0e838e1)
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
- `drone_city_nav/worlds/generated_city.sdf`
- `drone_city_nav/launch/city_nav.launch.py`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf` (grep по visual/mesh)
- `INVESTIGATION.md` из коммита `6a3638f` (предыдущее расследование)
- Git diffs: `718deec..cf8b9b1`, `718deec..HEAD`, а также `git show` для каждого ключевого коммита

# Evidence

## Ключевые file:line ссылки

- `scripts/run_city_mvp.sh:208–232` — `prepare_runtime_resources`: кастомные модели всегда
- `scripts/run_city_mvp.sh:122–125` — spawn coords: `SIM_START_X_M:--57`, `SIM_START_Y_M:--27`
- `scripts/run_city_mvp.sh:401` — `gz sim -g > /dev/null 2>&1 &` (без `--gui-config`)
- `scripts/run_city_mvp.sh:403–411` — parallel unpause + follow camera
- `scripts/gazebo_gui_control.py:159–246` — `configure_follow_camera`
- `scripts/gazebo_gui_control.py:164` — `required_accepted_attempts: int = 3`
- `scripts/gazebo_gui_control.py:146–156` — `_confirm_tracking` c 0.2s
- `scripts/gazebo_gui_control.py:115–143` — `_publish_track`
- `scripts/gazebo_gui_control.py:223–228` — best-effort exit
- `drone_city_nav/models/x500_lidar_2d/model.sdf:5` — `<uri>model://x500</uri>`
- `drone_city_nav/models/lidar_2d_v2/model.sdf:77–90` — `yellow_drone_locator_core` sphere r=0.32
- `drone_city_nav/worlds/generated_city.sdf` — нет `<gui>` секции с camera pose

## Ключевые commit hash-ы

| Hash | Описание | Значимость |
|------|----------|------------|
| `718deec` | Handle host PX4 protobuf compatibility | Дрон виден, PX4 default модели в GUI |
| `e998cb9` | Stabilize static-map obstacle overlays | **КЛЮЧЕВОЙ**: убрал headless условие → кастомные модели всегда |
| `fa88422` | Add bright drone visibility markers | Добавил visibility_marker_link в x500_lidar_2d/model.sdf |
| `b4b9e8c` | Add Gazebo GUI drone follow camera | Follow camera **работала** (bash inline) |
| `41bb70e` | Keep Gazebo GUI simulation unpaused | Follow camera **сломалась** (`--gui-config`) |
| `bbcc6a3` | Attach drone marker to lidar visual link | Перенёс маркеры из x500_lidar_2d в lidar_2d_v2; x500_lidar_2d → 17 строк |
| `1d7be4a` | Restore original Gazebo follow camera launch | Убрал `--gui-config`, вернул `gz sim -g` |
| `d169f72` | Publish Gazebo camera tracking command | Добавил `_publish_track` → `/gui/track` |
| `2d054e2` | Stabilize Gazebo launch diagnostics | Заменил bash-функции Python-скриптом |

# Findings

1. **Bug #1 = прямое следствие Bug #2.** Дрон находится в `(-57, -27, 0.3)`. Без следящей камеры дефолтная Gazebo GUI-камера направлена на area вблизи origin. Дрон вне viewport. World SDF не определяет стартовую позицию камеры.

2. **Bug #1, independent component.** Кастомная `x500_lidar_2d/model.sdf` использует `<uri>model://x500</uri>`, PX4-оригинал — `<uri>x500</uri>`. Визуальные mesh из `x500_base` (`NXP-HGD-CF.dae`, `5010Base.dae`) могут не рендериться, если `model://x500` merge работает иначе, чем bare `x500`. В этом случае видна только жёлтая сфера из `lidar_2d_v2`, но не quadcopter frame.

3. **Bug #2: Python-реализация follow camera логически отличается от рабочего bash-оригинала.** Оригинал (b4b9e8c): один `data: true` → выход. Текущий: `required_accepted_attempts=3` + `_confirm_tracking` с 0.2s. Best-effort выход не гарантирует активацию слежения.

4. **`--gui-config` (41bb70e) уже убран в 1d7be4a.** Возврат к bare `gz sim -g` — правильный шаг. Это НЕ текущая проблема.

5. **Stale process cleanup уже добавлен (5e006b2, 2d054e2).** `gazebo_process_cleanup.py` убивает все `gz sim` процессы перед стартом. Stale-сцена — не текущая проблема.

6. **Уже попытано, не помогло:** `--gui-config` с `start_paused=false` (41bb70e), дополнительный `/gui/track` publish (d169f72), `_confirm_tracking` через `/gui/currently_tracked` (2d054e2).

# Relevant code paths

- `scripts/run_city_mvp.sh` → `prepare_runtime_resources` (строки 208–232) — выбор моделей
- `scripts/run_city_mvp.sh` → Gazebo subshell (строки 376–416) — запуск server/GUI/follow/unpause
- `scripts/gazebo_gui_control.py` → `configure_follow_camera` (строки 159–246)
- `scripts/gazebo_gui_control.py` → `_confirm_tracking` (строки 146–156)
- `scripts/gazebo_gui_control.py` → `_publish_track` (строки 115–143)
- `drone_city_nav/models/x500_lidar_2d/model.sdf` — include-based drone model
- `drone_city_nav/models/lidar_2d_v2/model.sdf` — lidar + visual markers
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf` — PX4 original

# Timeline/history

```
718deec  2026-06-16  Handle host PX4 protobuf compatibility         [дрон виден; PX4 default models в GUI]
cf8b9b1  2026-06-16  Clean up simulation child processes            [cleanup change; модели не менялись]
5cfa9cd  2026-06-16  Document native and container workflows
fa88422  2026-06-16  Add bright drone visibility markers             [+visibility_marker_link в x500_lidar_2d]
e998cb9  2026-06-17  Stabilize static-map obstacle overlays         [КЛЮЧ: убрал headless guard → кастомные модели всегда]
b4b9e8c  2026-06-18  Add Gazebo GUI drone follow camera             [следящая камера РАБОТАЛА; bash inline]
41bb70e  2026-06-18  Keep Gazebo GUI simulation unpaused            [камера СЛОМАЛАСЬ; добавлен --gui-config]
5e006b2  2026-06-18  Add Gazebo tracking controls and stale cleanup [stale cleanup добавлен]
89addd4  2026-06-18  Clarify Gazebo camera tracking controls
0e838e1  2026-06-18  Remove Gazebo camera env controls
bbcc6a3  2026-06-18  Attach drone marker to lidar visual link       [маркеры → lidar_2d_v2; x500_lidar_2d → 17 строк]
b53d43c  2026-06-18  Restore Gazebo follow camera toggle
1d7be4a  2026-06-18  Restore original Gazebo follow camera launch   [убрал --gui-config; вернул gz sim -g]
d169f72  2026-06-18  Publish Gazebo camera tracking command         [добавил /gui/track publish в bash]
9966cde  2026-06-18  Document Gazebo GUI regression investigation   [предыдущий INVESTIGATION.md]
6a3638f  2026-06-18  Revise Gazebo GUI investigation                [ревизия; добавил configure_world_running bash]
2d054e2  2026-06-18  Stabilize Gazebo launch diagnostics            [bash → Python (gazebo_gui_control.py)]
5761734  2026-06-18  Tighten Gazebo helper safety checks
2f7bb76  2026-06-18  removed plan
bf4dfd8  2026-06-18  removed investigation
f8f7357  2026-06-18  Remove static-only planner fallback
b2b38c9  2026-06-18  Add planner diagnostic counters
bac9dbd  2026-06-18  Remove offboard clearance suppression
9ad5363  2026-06-18  Make tracking overspeed limit opt-in           [HEAD]
```

# Hypotheses/alternatives

## Bug #1: Hypothesis A — Camera position (leading)

- Immediate cause: дрон вне дефолтного viewport (origin area).
- Cause-of-cause: follow camera не работает (Bug #2).
- Deeper/root cause: Python-реализация follow camera не воспроизводит рабочий bash-оригинал.
- Evidence: spawn coords `(-57, -27)` неизменны; world SDF без GUI camera; at 718deec дрон был виден (возможно, при ручной навигации, или PX4 default model видна с большего расстояния).
- Confidence: high для «follow camera отсутствует → дрон вне viewport».
- Falsification: если пользователь вручную навигирует камеру к `(-57, -27)` и видит дрон с желтой сферой, Hypothesis A подтверждается.

## Bug #1: Hypothesis B — `model://x500` merge failure (supplementary)

- Immediate cause: quadcopter frame mesh (из `x500_base`) не рендерится.
- Cause-of-cause: `<uri>model://x500</uri>` в кастомной `x500_lidar_2d/model.sdf` resolved иначе, чем bare `<uri>x500</uri>` в PX4-оригинале.
- Evidence: PX4 original: `<uri>x500</uri>`; кастомный: `<uri>model://x500</uri>`; mesh refs в x500_base через `model://x500_base/meshes/...`.
- Confidence: low-medium. Оба URI должны работать при корректном `GZ_SIM_RESOURCE_PATH`.
- Falsification: `gz model --model-name x500_lidar_2d_0 --link` в живой симуляции — если link visuals пустые у base_link, merge не работает. Если жёлтая сфера видна — merge работает для lidar_2d_v2.

## Bug #2: Hypothesis — `_confirm_tracking` ненадёжен + early best-effort exit (leading)

- Immediate cause: follow camera «успешно» завершается без реального подтверждения слежения.
- Cause-of-cause: `required_accepted_attempts=3` при недостижимом `_confirm_tracking` (0.2s) → best-effort exit после 3 итераций.
- Evidence: `gazebo_gui_control.py:223–228`; оригинал b4b9e8c — один `data: true` → return 0.
- Confidence: high.
- Falsification: если убрать `required_accepted_attempts` и вернуть семантику «один success → return 0», и камера начнёт следить — причина подтверждена.

## Bug #2: Alternative — `_publish_track` конкурирует с `/gui/follow`

- Гипотеза: публикация `CameraTrack` в `/gui/track` (добавлена в d169f72) может конкурировать или сбивать `/gui/follow` service.
- Evidence: `gazebo_gui_control.py:199` — `_publish_track` вызывается сразу после каждого успешного `/gui/follow`.
- Confidence: low. Нужен live-тест с отключённым `_publish_track`.
- Falsification: отключить `_publish_track` и проверить, начинает ли камера следить только по `/gui/follow`.

# Risk/impact

- Без следящей камеры оператор не видит дрон в Gazebo GUI и не может визуально контролировать миссию.
- Если `model://x500` merge не работает (Hypothesis B), дрон виден только как жёлтая сфера — нет quadcopter frame mesh. Это затрудняет визуальную проверку ориентации и поведения дрона.
- Попытка снова добавить `--gui-config` или `CameraTrackingConfig` через GUI config (41bb70e-стиль) уже доказала свою неработоспособность.
- Добавление `_confirm_tracking` с увеличенным таймаутом без устранения `required_accepted_attempts` не решит проблему.

# Conclusions

1. **Bug #1** на текущем `main` — прямое следствие Bug #2. Нет следящей камеры → дрон вне viewport. Дополнительный independent компонент (видимость quadcopter mesh) требует live-верификации.

2. **Bug #2**: Python-реализация `configure_follow_camera` (2d054e2) семантически отличается от рабочего bash-оригинала (b4b9e8c). Ключевое отличие: `required_accepted_attempts=3` + ненадёжный `_confirm_tracking` вместо «один success → exit».

3. **Уже исправлено правильно**: `--gui-config` убран (1d7be4a), stale cleanup добавлен (5e006b2+), `configure_world_running` реализован отдельно.

4. **Направление фикса Bug #2**: вернуть семантику «один успешный `/gui/follow` → выход». Удалить или переработать `required_accepted_attempts=3` и `_confirm_tracking`. Оставить `_publish_track` только если он не конкурирует.

5. **Направление фикса Bug #1 (independent component)**: заменить `<uri>model://x500</uri>` на `<uri>x500</uri>` в `drone_city_nav/models/x500_lidar_2d/model.sdf` — точное соответствие PX4-оригиналу. Это убирает потенциальный риск разного resolution behavior.

# Recommendations/next steps

## Немедленно (Bug #2 → следящая камера)

**1. Упростить `configure_follow_camera` до рабочей семантики:**

В `scripts/gazebo_gui_control.py`, функция `configure_follow_camera` (строки 159–246):
- Убрать `required_accepted_attempts` как barring condition.
- Оставить `required_accepted_attempts` только как «количество попыток подтверждения», но делать return 0 СРАЗУ после первого `response_is_true(follow_response)`, не ждать подтверждения через `_confirm_tracking`.
- `_confirm_tracking` оставить как optional best-effort logging, но не блокирующим условием.
- Итого: один успешный ответ `/gui/follow` (`data: true`) + один вызов `/gui/follow/offset` → return 0.

**2. Разобраться с `_publish_track`:**
- Если `_publish_track` в `/gui/track` конкурирует с `/gui/follow`, его следует либо убрать, либо оставить только как дополнительный reinforcement ПОСЛЕ успешного выхода из retry-loop, а не внутри каждой итерации.

## После Bug #2 (Bug #1 — видимость drone frame)

**3. Заменить URI в `drone_city_nav/models/x500_lidar_2d/model.sdf`:**
```xml
<!-- Текущее (потенциально разное resolution) -->
<uri>model://x500</uri>

<!-- Заменить на (точное соответствие PX4-оригиналу) -->
<uri>x500</uri>
```
`drone_city_nav/models/x500_lidar_2d/model.sdf:5` и строка 8 (lidar_2d_v2 URI — там `model://` уже правильный, т.к. custom модель).

**4. Верифицировать видимость:** после фикса Bug #2, запустить `make host-sim-gui` и подтвердить, что камера следит за дроном и quadcopter frame mesh отображается.

# Verification/falsification steps for findings

## Bug #2

1. **Фиксируем «один success → return 0»:**
   - Изменить `configure_follow_camera` (gazebo_gui_control.py:159–246): после первого `response_is_true(follow_response)` → вызвать `/gui/follow/offset` → return 0.
   - Запустить `make host-sim-gui`.
   - Если камера начинает следить — confirmed: `required_accepted_attempts=3` был причиной.
   - Если не следит — копать `/gui/follow` service behavior в этой версии gz-sim.

2. **Изолировать `_publish_track`:**
   - Временно убрать вызов `_publish_track` из `configure_follow_camera`.
   - Если после фикса (1) камера работает без `_publish_track` — оставить его убранным.
   - Если с `_publish_track` камера не работает, но без него работает — `_publish_track` конкурирует с `/gui/follow`.

3. **Проверить `/gui/currently_tracked` топик:**
   - `gz topic -e -t /gui/currently_tracked -d 5.0` во время работающей симуляции с дроном.
   - Если топик публикуется — `_confirm_tracking` нужен с более долгим timeout.
   - Если не публикуется — `_confirm_tracking` следует убрать.

## Bug #1

4. **Ручная навигация к дрону:**
   - Запустить `make host-sim-gui` с BROKEN следящей камерой.
   - Вручную в Gazebo GUI навигировать к `(-57, -27)`.
   - Если видна жёлтая сфера + quadcopter frame: Bug #1 = чисто следствие Bug #2.
   - Если видна только сфера, нет frame: Hypothesis B (model://x500 merge) — менять URI.
   - Если ничего не видно: проверить `gz model --model-name x500_lidar_2d_0`.

5. **Проверить URI resolution:**
   - Изменить `drone_city_nav/models/x500_lidar_2d/model.sdf:5` с `model://x500` на `x500`.
   - Запустить `make host-sim-gui`.
   - Если quadcopter frame появляется — Hypothesis B подтвердилась.

# Follow-up verification implications

- Headless smoke test (`make host-sim-headless`) не верифицирует Gazebo GUI camera tracking и видимость 3D-модели. Нужен отдельный ручной GUI check после каждого изменения, затрагивающего `gazebo_gui_control.py` или `x500_lidar_2d/model.sdf`.
- При изменении `configure_follow_camera` запускать `make host-test-scripts` для unit-тестов Python (в `scripts/tests/`).
- Перед коммитом изменений в model.sdf: запустить `make host-format` (не применимо к SDF), `make host-quality`.

# Open questions

1. Возвращает ли `/gui/follow` `data: true` до спавна модели `x500_lidar_2d_0` в текущей версии gz-sim? (Нужен live-log `gz_city_mvp.log` с временными метками от `configure_follow_camera`.)
2. Публикует ли Gazebo топик `/gui/currently_tracked` в текущей версии gz-sim на данном хосте?
3. Является ли `<uri>model://x500</uri>` vs `<uri>x500</uri>` значимым различием в gz-sim Harmonic при merge include?
4. Достаточно ли `make host-sim-gui` (через `Makefile`) или нужен `scripts/run_city_mvp_host.sh` напрямую для воспроизведения обоих багов?

---

# Дополнительные находки (второй раунд расследования)

Следующие факты установлены при детальном анализе log-файлов, runtime-симлинков и git-истории
commit `bbcc6a3`.

## Подтверждённые факты из log-файлов

**`log-host/gz_city_mvp.log` (последний запуск 22:36 Jun 18):**
```
Gazebo stale cleanup: no conflicting Gazebo processes found        ← stale-process не причина
libEGL warning: egl: failed to create dri2 screen (10de:2520)     ← косметика, NVIDIA работает отдельно
Waiting for Gazebo GUI follow target 'x500_lidar_2d_0' (1/60): Service call timed out
Gazebo GUI follow camera command accepted: ...accepted_attempts=1/2/3
WARNING: Gazebo GUI follow camera command accepted but state confirmation is unavailable
```

**`log-host/px4_city_mvp.log`:**
```
INFO  [init] Gazebo simulator 8.10.0
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: generated_city, model: x500_lidar_2d_0   ← spawn УСПЕШЕН
```

**Вывод:** модель заспавнена. Stale-процессов нет. Проблема — в GUI, не в spawn.

## Runtime симлинки: build/ vs build-host/

`build/gazebo_city_mvp/models/` — **stale Docker-среда**:
- `x500_lidar_2d -> /workspace/drone_city_nav/models/x500_lidar_2d` (нет `/workspace`)
- `x500_base -> /workspace/external/...`

`build-host/gazebo_city_mvp/models/` — **корректные симлинки**:
- `x500_lidar_2d -> /home/formi/.../drone_city_nav/models/x500_lidar_2d`
- `lidar_2d_v2 -> /home/formi/.../drone_city_nav/models/lidar_2d_v2`

`run_city_mvp_host.sh` использует `COLCON_BUILD_BASE=build-host` → корректная среда. Это не причина бага.

## Детальный анализ bbcc6a3

Коммит `bbcc6a3` (Jun 18 10:31, «Attach drone marker to lidar visual link»):

**До `bbcc6a3` (fa88422):** в `x500_lidar_2d/model.sdf` был прямой link `visibility_marker_link`
с pose `0 0 0` и `VisibilityMarkerJoint` (type=fixed, parent=base\_link). Жёлтые маркеры
(yellow\_body\_plate, yellow\_arm\_x/y, yellow\_rotor\_*, yellow\_ground\_projection\_beam/disc)
— все в этом прямо-объявленном link.

**После `bbcc6a3`:** маркеры удалены из `x500_lidar_2d`, добавлены в `lidar_2d_v2/link`.
`x500_lidar_2d/model.sdf` стал 17-строчным include-only файлом. Новые маркеры в `lidar_2d_v2`:
- `yellow_drone_locator_core`: sphere r=0.32, pose=(-0.12, 0, -0.12) в frame link
- `yellow_drone_locator_arm_x/y`: cylinder r=0.05 len=1.35
- `yellow_ground_projection_beam`: cylinder r=0.08 len=18.0
- `yellow_ground_projection_disc`: cylinder r=1.8 len=0.035

## Анализ двойного позиционирования link (merge=true + joint)

`x500_lidar_2d/model.sdf` содержит:
```xml
<include merge="true">
  <uri>model://lidar_2d_v2</uri>
  <pose>0.12 0 0.26 0 0 0</pose>   ← ПОЗИЦИОНИРОВАНИЕ #1 через include pose
</include>
<joint name="LidarJoint" type="fixed">
  <parent>base_link</parent>
  <child>link</child>
  <pose relative_to="base_link">-0.1 0 0.26 0 0 0</pose>   ← ПОЗИЦИОНИРОВАНИЕ #2 через joint
</joint>
```

При `merge="true"` в sdformat 14.8.0:
- link `link` из `lidar_2d_v2` получает начальную позу: merge-offset `(0.12, 0, 0.26)` + link pose `(0, 0, 0)` = `(0.12, 0, 0.26)` в frame родительской модели.
- `LidarJoint` (fixed, parent=`base_link`) кинематически ограничивает `link` в позиции `(-0.1, 0, 0.26)` от `base_link`.

**Конфликт:** начальная merge-поза `(0.12, 0, 0.26)` ≠ joint-поза `(-0.1, 0, 0.26)`.
Physics resolves kinematic constraint (joint wins). Но рендерер Ogre2 в Gazebo Harmonic
может не синхронизировать visual transform link'а с joint-enforced runtime позой,
если начальная поза (из merge) была другой. Это приводит к тому, что визуалы
рендерятся в замороженной merge-позе или не появляются вовсе.

**В `fa88422` этой проблемы не было**: `visibility_marker_link` имел pose `0 0 0` и был
ограничен `VisibilityMarkerJoint` с pose `0 0 0` — конфликта не было.

## Вычисление ожидаемой мировой позиции жёлтой сферы

- Spawn: `(-57, -27, 0.3)`
- `x500_base` model root pose: `(0, 0, 0.24)` → `base_link` at world `(-57, -27, 0.54)`
- `LidarJoint` offset from `base_link`: `(-0.1, 0, 0.26)` → `link` at `(-57.1, -27, 0.80)`
- Sphere local pose in `link`: `(-0.12, 0, -0.12)` → sphere center at world `(-57.22, -27, 0.68)`
- Sphere radius = 0.32 m → видима в z ∈ [0.36, 1.0] над землёй — должна быть хорошо видна

Если камера следит за дроном с offset `(-12, 0, 6)`, сфера должна ясно отображаться.
Если камера НЕ следит, дрон на расстоянии 84 м от мирового центра и вне дефолтного viewport.

## SDF-версии и x500\_lidar\_2d структура

| Файл | SDF version | visual type |
|------|-------------|-------------|
| PX4 `lidar_2d_v2` | 1.6 | mesh `lidar_2d_v2.dae` |
| custom `lidar_2d_v2` | **1.9** | box+cylinder примитивы + yellow sphere |
| `x500_base` | 1.9 | mesh (NXP-HGD-CF.dae, 5010Base.dae и др.) |
| PX4 `x500_lidar_2d` | 1.9 | `<uri>x500</uri>` (bare) |
| custom `x500_lidar_2d` | 1.9 | `<uri>model://x500</uri>` ← потенциальная разница |

## Обновлённая оценка гипотез

### Гипотеза A (ВЫСОКАЯ уверенность): bbcc6a3 merge=true + joint конфликт

Перемещение маркеров в `lidar_2d_v2/link` создало двойное позиционирование: include-pose
при merge ≠ joint-pose. В Gazebo Harmonic (gz-sim8 / sdformat 14.8.0) это может приводить
к тому, что визуалы merged link не отслеживаются рендерером корректно.

До `bbcc6a3`: `visibility_marker_link` в `x500_lidar_2d` — прямой link без merge-конфликта.
После `bbcc6a3`: `link` из `lidar_2d_v2` — merged link с конфликтующими позами. Это объясняет
невидимость жёлтых маркеров ДАЖЕ при работающей камере.

**Проверка:** убрать `<pose>0.12 0 0.26</pose>` из include-блока lidar\_2d\_v2 в
`x500_lidar_2d/model.sdf` → тогда link позиционируется только через LidarJoint.

### Гипотеза B (ВЫСОКАЯ уверенность, независимая): отсутствие follow camera

Подтверждена логом: `state confirmation is unavailable`. Камера не следит за дроном.
Дрон в `(-57, -27)` — вне дефолтного viewport. Даже если маркеры рендерятся корректно,
пользователь их не видит без follow camera.

### Гипотеза C (СРЕДНЯЯ): `model://x500` vs bare `x500` URI

PX4-оригинал `x500_lidar_2d` использует `<uri>x500</uri>`.
Custom использует `<uri>model://x500</uri>`.
Теоретически оба должны работать при корректном `GZ_SIM_RESOURCE_PATH`.
Но если `model://` prefix вызывает иное поведение при resolve → mesh-визуалы из x500_base
не загружаются → только жёлтая сфера (или ничего, если Гипотеза A тоже активна).

## Рекомендованные фиксы (дополнение)

**Фикс X (для Гипотезы A): убрать include-pose из merge lidar\_2d\_v2**

В `drone_city_nav/models/x500_lidar_2d/model.sdf` (строка 9):
```xml
<!-- Убрать: -->
<pose>0.12 0 0.26 0 0 0</pose>
```
Тогда позиционирование `link` определяется только `LidarJoint`. Нет конфликта merge/joint.

**Фикс Y (для Гипотезы C): заменить URI**

В `drone_city_nav/models/x500_lidar_2d/model.sdf` строка 5:
```xml
<!-- Текущее: -->
<uri>model://x500</uri>
<!-- Заменить на: -->
<uri>x500</uri>
```

**Комбинированный фикс:** применить оба. Плюс исправить follow camera (Bug #2).

