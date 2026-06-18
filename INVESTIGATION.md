# Context/Task

Нужно исследовать две локальные регрессии Gazebo GUI:

1. Между `718deec` и `cf8b9b1` 3D-модель дрона перестала отображаться в окне 3D-мира, при этом дрон существует и летит, что видно в RViz.
2. На `b4b9e8c` следящая камера в Gazebo 3D работает, но сценарий/миссия визуально не стартует. На `41bb70e` сценарий стартует, но вместо следящей камеры снова работает свободная камера.

Факты, проверенные пользователем, приняты как базовые и не оспариваются.

# Research questions

- Что изменилось между `718deec` и `cf8b9b1`, если модель дрона исчезла только в Gazebo 3D GUI?
- Что изменилось между `b4b9e8c` и `41bb70e`, если следящая камера была рабочей, а затем стала свободной?
- Почему на `b4b9e8c` сценарий/миссия не стартует визуально?
- Какое направление фикса следует выбрать, не смешивая управление симуляцией и настройку камеры?

# Scope and constraints

- Исследовался только локальный код, локальная git-история и локальные CLI/артефакты.
- SSH, HTTP и удаленные целевые системы не использовались.
- Notion policy в inbox: `optional`; Notion task id в prompt не указан, поэтому Notion не читался.
- GitLab/MR в prompt не указан, поэтому GitLab не читался.
- GUI-прогоны в рамках этого раунда не запускались; выводы основаны на подтвержденных пользователем good/bad коммитах и локальном diff/code evidence.
- По завершении исследования HEAD возвращен на ветку `main`.

# Detected stack/profiles

- Стек: ROS 2 workspace, Gazebo, PX4 SITL, C++/ament CMake, helper shell scripts.
- Прочитаны обязательные профили:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- Rust profile не применялся: в target workspace основная затронутая область для этого исследования - shell launch scripts и C++/ROS workspace; Rust manifest в workspace не обнаруживался как relevant project stack.
- Также прочитаны обязательные протоколы:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- `make host-build`
- `make host-test`
- `make host-test-scripts`
- `make host-quality`
- `make host-format`
- `make host-sim-gui`
- `make host-sim-headless`
- current-environment equivalents: `make build`, `make test`, `make quality`, `make sim-gui`, `make sim-headless`

Для этого investigation round выполнены только non-GUI / non-mission verification commands, потому что задача была исследовательская и визуальные факты уже предоставлены пользователем. После reviewer feedback дополнительно исправлено расхождение между рекомендацией и текущим launch script: GUI остается на прямом `gz sim -g`, а world unpause выполняется отдельной Gazebo transport командой.

# Observed symptom

## Problem 1

- `718deec`: 3D-модель дрона в Gazebo GUI отображается корректно.
- `cf8b9b1`: 3D-модель дрона в Gazebo GUI не отображается, хотя RViz показывает, что дрон существует и летит.

## Problem 2

- `b4b9e8c`: следящая камера в Gazebo GUI работает, но сценарий/миссия визуально не стартует; виден скачущий `play/pause` / процент.
- `41bb70e`: сценарий/миссия стартует, но следящая камера заменяется обычной свободной камерой.

# Immediate cause

## Problem 1

Между `718deec` и `cf8b9b1` не менялись SDF-модели, world, spawn pose, PX4 model target или GUI camera code. Единственный diff - cleanup логика в `scripts/run_city_mvp.sh`.

Доказанная immediate boundary: между good/bad коммитами изменился только lifecycle/cleanup в `scripts/run_city_mvp.sh`; SDF-модели, world, spawn pose, PX4 model target и GUI camera code не менялись.

Stale/duplicate Gazebo process state остается ведущей гипотезой, но не закрытым root cause: `cf8b9b1` улучшает cleanup при завершении, однако не очищает уже существующие stale Gazebo server processes перед стартом нового запуска. Без live `ps`/GUI snapshot в момент bad run нельзя доказать, что GUI действительно показывал stale scene.

## Problem 2

Для `b4b9e8c` immediate cause нестарта сценария: GUI использует default Gazebo GUI config, где `WorldControl` имеет `<start_paused>true</start_paused>`. При запуске `gz sim -g` без отдельного world-control unpause GUI может навязать paused state поверх server run mode.

Для `41bb70e` immediate cause поломки следящей камеры: commit меняет способ запуска GUI с прямого `gz sim -g` на `gz sim -g --gui-config <runtime_config>`. Код настройки `/gui/follow` при этом остается тем же, поэтому регрессия локализуется в изменении GUI config launch path.

# Causal chain / why chain

## Problem 1 leading hypothesis chain

Observed symptom:
`cf8b9b1` показывает мир без 3D-модели дрона, но RViz видит текущую миссию.

Immediate technical boundary:
Gazebo GUI показывает мир без 3D-модели дрона, но единственный code diff между good/bad commits находится в cleanup логике. Это ограничивает доказанную причину областью process lifecycle.

Why that happened:
Ведущая гипотеза: GUI показывает scene от stale/duplicate Gazebo process или process state, не совпадающий с текущей ROS/PX4 сессией. До `cf8b9b1` cleanup убивал только top-level background jobs shell-скрипта, а не их descendants. Gazebo server/GUI запускаются внутри background subshell, поэтому дочерние `gz sim` процессы могли переживать остановку скрипта.

Why that condition was possible:
`cf8b9b1` добавил descendant cleanup только на exit текущего запуска. Он не добавил startup cleanup, который удалял бы stale server processes, оставшиеся от предыдущего запуска.

Actionable finding / unresolved boundary:
Actionable finding - process lifecycle был неполным: cleanup был сфокусирован на shutdown, а не на startup isolation. Точный GUI-level механизм невидимости модели требует live GUI/process capture, поэтому stale-scene explanation остается гипотезой. Направление фикса: перед запуском нового мира убивать stale `gz sim` server processes для runtime world path и логировать найденные PIDs. Это позднее действительно появилось в `main` как `stop_stale_gazebo_servers`.

Confidence: medium-high.

What would falsify:
Свежий запуск `cf8b9b1` после полной остановки всех `gz sim` processes / reboot все равно стабильно не показывает модель, а `ps` перед стартом подтверждает отсутствие stale Gazebo. Тогда нужно копать Gazebo GUI rendering/plugin state вне diff `718deec..cf8b9b1`.

## Problem 2 primary chain

Observed symptom:
На `b4b9e8c` follow camera работает, но миссия/сценарий не стартует визуально; на `41bb70e` миссия стартует, но follow camera ломается.

Immediate technical cause:
`41bb70e` решает проблему pause через замену всего GUI config, а не через отдельную команду управления миром.

Why that happened:
Default Gazebo GUI config содержит `WorldControl` со `start_paused=true`. `b4b9e8c` запускает GUI как `gz sim -g`, поэтому default config остается активным. `41bb70e` копирует default config, заменяет `<start_paused>true</start_paused>` на `false`, затем запускает GUI с `--gui-config`.

Why that condition was possible:
В одном изменении были смешаны две независимые ответственности:

- состояние симуляции (`paused/running`);
- camera tracking GUI behavior.

Исправление pause через GUI config меняет весь путь инициализации Gazebo GUI, хотя для сценария нужен только server/world unpause.

Actionable root cause:
Нужно разделить управление симуляцией и настройку камеры. Следящая камера должна запускаться тем же путем, что в `b4b9e8c` (`gz sim -g` + `CameraTracking` command), а проблему paused state нужно решать server-side командой `WorldControl { pause: false }`, не подменяя GUI config. Текущий `scripts/run_city_mvp.sh` обновлен в этом направлении: после запуска Gazebo GUI он отправляет `/world/generated_city/control` с `pause: false`.

Confidence: high for regression boundary (`b4b9e8c..41bb70e`); medium for exact internal Gazebo reason why `--gui-config` disables/neutralizes follow camera, because без live GUI instrumentation нельзя увидеть active camera state.

What would falsify:
Если на `41bb70e` при запуске с `GZ_GUI_CONFIG_FILE` pointing to unmodified default config follow camera снова работает, причина не в `--gui-config` path, а конкретно в patched `start_paused=false` runtime config или timing. Если follow не работает с любым `--gui-config`, причина именно в custom GUI config launch path.

# Evidence per causal link

## Problem 1 evidence

- `git diff --stat 718deec..cf8b9b1` показал только один измененный файл: `scripts/run_city_mvp.sh`.
- `git diff --name-status 718deec..cf8b9b1`:

```text
M scripts/run_city_mvp.sh
```

- `git diff 718deec..cf8b9b1 -- drone_city_nav/models/x500_lidar_2d/model.sdf drone_city_nav/models/lidar_2d_v2/model.sdf drone_city_nav/worlds/generated_city.sdf drone_city_nav/config/urban_mvp.yaml` не дал diff.
- В `718deec:scripts/run_city_mvp.sh:219-241` cleanup собирает только `jobs -pr` и убивает эти PIDs.
- В `cf8b9b1:scripts/run_city_mvp.sh:219-267` cleanup добавляет `collect_descendant_pids` и убивает descendants, но эта логика остается только под `trap cleanup EXIT INT TERM`.
- В `main:scripts/run_city_mvp.sh:168-204` позднее добавлен startup guard `stop_stale_gazebo_servers`, который ищет `gz sim` по runtime world path и останавливает stale PIDs перед запуском.

## Problem 2 evidence

- `b4b9e8c:scripts/run_city_mvp.sh:390` запускает GUI напрямую:

```bash
gz sim -g > /dev/null 2>&1 &
```

- `b4b9e8c:scripts/run_city_mvp.sh:324-346` настраивает follow camera через `/gui/follow` и `/gui/follow/offset`.
- Default Gazebo GUI config в host env содержит `CameraTracking` plugin и `WorldControl`:
  - `$CONDA_PREFIX/share/gz/gz-sim8/gui/gui.config:74-82` содержит `<plugin filename="CameraTracking" name="Camera Tracking">`.
  - `$CONDA_PREFIX/share/gz/gz-sim8/gui/gui.config:138-140` содержит `<play_pause>true</play_pause>` и `<start_paused>true</start_paused>`.
- `41bb70e:scripts/run_city_mvp.sh:202-245` добавляет генерацию runtime GUI config и заменяет `start_paused` на `false`.
- `41bb70e:scripts/run_city_mvp.sh:441-445` запускает GUI уже так:

```bash
gz sim "${gz_gui_args[@]}" > /dev/null 2>&1 &
```

где `gz_gui_args` включает `--gui-config "${gazebo_gui_config_file}"`.

- `git diff b4b9e8c..41bb70e -- scripts/run_city_mvp.sh README.md` показывает, что follow function не менялась; ключевое изменение - runtime GUI config и `--gui-config`.
- `gz msg -i gz.msgs.WorldControl` подтвердил локально, что Gazebo имеет `WorldControl.pause`, то есть unpause можно делать через transport command без GUI config mutation.
- Текущий `main:scripts/run_city_mvp.sh` после revision добавляет `configure_gazebo_world_running`, который отправляет `pause: false` в `/world/${world_name}/control` и не использует `--gui-config`.

# Root cause / unresolved boundary

## Problem 1

Root cause не закрыт как доказанный факт. Доказано, что причина вероятнее всего не в модели, не в камере и не в spawn: между good/bad коммитами эти области не менялись. Ведущая гипотеза - неполная изоляция Gazebo process lifecycle между запусками: на момент `cf8b9b1` уже есть cleanup descendants на exit, но нет startup cleanup stale server state.

Unresolved boundary: без live process list в момент плохого GUI-прогона нельзя доказать, что GUI показывал именно stale process/scene. Для закрытия boundary нужен снимок `ps -eo pid,ppid,cmd | grep 'gz sim'` до старта и во время плохого запуска.

## Problem 2

Root cause - pause был исправлен на неправильном уровне абстракции. Вместо команды управления миром был заменен GUI config, а это изменило поведение camera tracking. Следящая камера и состояние симуляции должны управляться независимо.

Unresolved boundary для текущего `main`: в `main` уже нет runtime `--gui-config`, но есть дополнительная публикация `/gui/track`. Если на текущем `main` follow camera все еще не работает, требуется live verification активной GUI camera state через `/gui/currently_tracked` / `/gui/camera/pose` или воспроизводимый GUI log. Код сейчас логирует принятие transport command и отдельно unpause world command, но не проверяет, что активная камера действительно перешла в FOLLOW.

# Sources checked

- `.agent-io/inbox.txt`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- `README.md`
- `CONTRIBUTING.md`
- `Makefile`
- `scripts/run_city_mvp.sh` на `main`
- `scripts/run_city_mvp.sh` на `718deec`, `cf8b9b1`, `b4b9e8c`, `41bb70e`
- Default Gazebo GUI config: `$CONDA_PREFIX/share/gz/gz-sim8/gui/gui.config`
- Git diffs:
  - `718deec..cf8b9b1`
  - `b4b9e8c..41bb70e`
  - `41bb70e..main`

# Evidence

Команды, использованные как evidence:

```bash
git diff --stat 718deec..cf8b9b1
git diff 718deec..cf8b9b1 -- scripts/run_city_mvp.sh scripts/run_city_mvp_host.sh README.md Makefile
git diff --name-status 718deec..cf8b9b1
git diff --stat b4b9e8c..41bb70e
git diff b4b9e8c..41bb70e -- scripts/run_city_mvp.sh README.md
git show 718deec:scripts/run_city_mvp.sh
git show cf8b9b1:scripts/run_city_mvp.sh
git show b4b9e8c:scripts/run_city_mvp.sh
git show 41bb70e:scripts/run_city_mvp.sh
git show main:scripts/run_city_mvp.sh
./scripts/host_shell.sh bash -lc 'gz msg -i gz.msgs.WorldControl'
./scripts/host_shell.sh bash -lc 'cfg=$(ls $CONDA_PREFIX/share/gz/gz-sim*/gui/gui.config | head -1); nl -ba "$cfg" | sed -n "30,90p;120,145p"'
bash -n scripts/run_city_mvp.sh
git diff --check
make host-format
make host-quality
make host-test-scripts
```

# Evidence references

- `718deec:scripts/run_city_mvp.sh:219-241` - old cleanup only kills top-level jobs.
- `cf8b9b1:scripts/run_city_mvp.sh:219-267` - cleanup now collects descendants, but still only at exit/trap.
- `main:scripts/run_city_mvp.sh:168-204` - later startup stale Gazebo cleanup.
- `b4b9e8c:scripts/run_city_mvp.sh:324-346` - first follow camera service setup.
- `b4b9e8c:scripts/run_city_mvp.sh:390-397` - direct `gz sim -g` GUI launch plus follow setup.
- `41bb70e:scripts/run_city_mvp.sh:202-245` - runtime GUI config copy and `start_paused=false` patch.
- `41bb70e:scripts/run_city_mvp.sh:441-445` - GUI launch through `--gui-config`.
- `$CONDA_PREFIX/share/gz/gz-sim8/gui/gui.config:74-82` - default `CameraTracking` plugin exists.
- `$CONDA_PREFIX/share/gz/gz-sim8/gui/gui.config:138-140` - default `WorldControl` starts paused.
- `main:scripts/run_city_mvp.sh:408-449` - current `main` sends `/world/${world_name}/control` with `pause: false`.
- `main:scripts/run_city_mvp.sh:501-510` - current `main` launches direct `gz sim -g`, starts world unpause, then configures Gazebo GUI follow camera without `--gui-config`.

# Findings

1. `cf8b9b1` is not a model/rendering diff. It is a process cleanup diff. Therefore the confirmed visible/invisible split should be investigated through Gazebo process lifecycle first, not SDF or PX4 spawn.
2. The later presence of `stop_stale_gazebo_servers` in `main` matches the likely fix direction for problem 1: clean stale Gazebo server processes at startup, not only descendants at exit. This remains a leading hypothesis until live bad-run process evidence is captured.
3. `b4b9e8c` correctly introduced follow camera via `CameraTracking` transport services but still used default Gazebo GUI config, whose `WorldControl` starts paused.
4. `41bb70e` solved pause by mutating and passing a GUI config. That is the first code change that can explain the camera regression because the follow function itself did not change.
5. Correct architectural fix direction: preserve the direct GUI launch path that made tracking work, and unpause the world through Gazebo transport/server control instead of GUI config replacement. The current launch script now follows this direction.

# Relevant code paths

- `scripts/run_city_mvp.sh`
  - runtime resource generation
  - Gazebo server and GUI startup
  - Gazebo GUI follow camera setup
  - cleanup/trap and stale process handling
- Gazebo GUI default config:
  - `CameraTracking`
  - `WorldControl`
- PX4/Gazebo model resource path:
  - `PX4_GZ_MODELS`
  - `GZ_SIM_RESOURCE_PATH`

# Timeline/history

```text
718deec 2026-06-16 22:18:14 -0300 Handle host PX4 protobuf compatibility
cf8b9b1 2026-06-16 22:25:41 -0300 Clean up simulation child processes
b4b9e8c 2026-06-18 10:06:22 -0300 Add Gazebo GUI drone follow camera
41bb70e 2026-06-18 10:10:07 -0300 Keep Gazebo GUI simulation unpaused
5e006b2 2026-06-18 10:18:33 -0300 Add Gazebo tracking controls and stale server cleanup
```

`5e006b2` уже содержит более правильный startup cleanup direction для stale Gazebo, но также позднее добавлял GUI tracking controls/panel, что пользователь счел неудобным.

# Hypotheses/alternatives

## Hypothesis A: stale Gazebo process caused invisible drone in `cf8b9b1`

- Immediate cause: GUI/render scene не соответствует текущей PX4/ROS сессии.
- Cause-of-cause: stale Gazebo processes от предыдущего запуска пережили остановку.
- Deeper/root cause: teardown-only cleanup без startup isolation.
- Evidence: `718deec..cf8b9b1` меняет только cleanup; моделей/world/spawn diff нет; later `main` добавляет startup stale cleanup.
- Confidence: medium-high.
- Falsification: чистый запуск без единого stale `gz sim` процесса все равно воспроизводит невидимость на `cf8b9b1`.

## Hypothesis B: `--gui-config` caused follow-camera regression in `41bb70e`

- Immediate cause: активная GUI camera остается свободной вместо `CameraTracking`.
- Cause-of-cause: GUI запускается с runtime config через `--gui-config`, хотя в `b4b9e8c` запуск был direct `gz sim -g`.
- Deeper/root cause: pause-state fix был реализован через GUI config mutation, а не через world control.
- Evidence: `b4b9e8c..41bb70e` меняет GUI launch/config, follow function unchanged; user facts show exact regression boundary.
- Confidence: high for regression boundary; medium for Gazebo internal mechanism.
- Falsification: на `41bb70e` direct `gz sim -g` с отдельным unpause все равно ломает camera tracking.

## Alternative: SDF/model change broke visibility between `718deec` and `cf8b9b1`

- Status: rejected for this pair.
- Evidence: no diff in model/world/config files between `718deec` and `cf8b9b1`; only `scripts/run_city_mvp.sh` cleanup changed.

# Risk/impact

- Если лечить pause через GUI config, можно снова ломать camera tracking или UI plugin initialization.
- Если полагаться только на exit cleanup, повторные GUI-прогоны могут подключаться к stale Gazebo process state.
- Если follow camera setup проверяет только transport response, можно получить ложный успех: команда принята, но активная 3D camera не перешла в follow mode.
- Видимые GUI-регрессии не покрываются headless smoke checks; нужны отдельные ручные/CLI verification hooks для GUI camera state.

# Conclusions

1. Для проблемы 1 доказанная граница - не SDF и не камера: `cf8b9b1` меняет только cleanup. Неполная изоляция процессов Gazebo между запусками остается наиболее вероятной гипотезой, но требует live process evidence для окончательного подтверждения.
2. Для проблемы 2 `b4b9e8c` сломал сценарий потому, что default Gazebo GUI config содержит `start_paused=true`.
3. `41bb70e` сломал follow camera потому, что исправил pause через runtime GUI config и запуск `--gui-config`, изменив путь инициализации Gazebo GUI. Следящая камера на `b4b9e8c` работала при прямом `gz sim -g`.
4. Правильное направление фикса: не использовать GUI config как способ управления pause. Запускать GUI прямым путем, а unpause делать отдельной server/world control командой. В текущем script это направление реализовано.

# Recommendations/next steps

1. Для сценария/миссии:
   - сохранить direct GUI launch `gz sim -g`;
   - после старта GUI/server отправлять Gazebo transport command:

```bash
gz service \
  -s /world/generated_city/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'pause: false'
```

2. Для follow camera:
   - не возвращать видимую `CameraTrackingConfig` panel;
   - использовать `CameraTracking` transport command, но добавить проверку фактического состояния через `/gui/currently_tracked` или другой доступный GUI status topic/service;
   - при необходимости публиковать `gz.msgs.CameraTrack` в `/gui/track` повторно до подтверждения, а не один раз.

3. Для невидимого дрона в Gazebo GUI:
   - сохранить startup stale cleanup (`stop_stale_gazebo_servers`) или усилить его, чтобы он ловил не только server by world path, но и связанные GUI/client processes текущего runtime;
   - логировать `ps`-снимок при старте GUI-прогона: найденные stale PIDs, новый server PID, GUI PID;
   - считать stale-scene explanation гипотезой до такого capture, а не закрытым root cause.

4. Не смешивать в одном изменении:
   - SDF/visual marker changes;
   - GUI camera tracking;
   - world pause/start control;
   - stale process cleanup.

# Verification/falsification steps for findings

## For problem 1

1. Перед запуском bad commit:

```bash
ps -eo pid,ppid,cmd | grep -E 'gz sim|generated_city' | grep -v grep
```

2. Запустить `cf8b9b1` после полной очистки Gazebo processes.
3. Если дрон станет видимым, stale process hypothesis подтверждается.
4. Если не станет видимым, сохранить:
   - `ps` до старта;
   - `ps` во время старта;
   - `log-host/gz_city_mvp.log`;
   - список Gazebo services/topics.

## For problem 2

1. На `b4b9e8c` подтвердить, что direct GUI launch и follow работают, но world paused из-за default GUI config.
2. На `41bb70e` временно исключить `--gui-config`, но выполнить world unpause через `/world/generated_city/control`.
3. Если follow camera возвращается, regression cause подтвержден.
4. Проверить active camera tracking не только по логу `Gazebo GUI follow camera configured`, но и по GUI tracking state topic/service.

# Follow-up verification implications

- Headless smoke validation полезна для миссии и PX4/ROS, но она не доказывает корректность Gazebo GUI camera tracking и видимость 3D model.
- Для GUI-регрессий нужен отдельный lightweight verification hook:
  - active Gazebo services/topics;
  - current tracked entity;
  - camera pose changes after target movement;
  - отсутствие stale duplicate `gz sim` processes before launch.

# Open questions

- Какой exact transport topic/service в Gazebo Harmonic надежно сообщает active camera tracking state в текущем host env: `/gui/currently_tracked`, `/gui/camera/pose` или другой endpoint?
- Достаточно ли текущий `stop_stale_gazebo_servers` чистит только server process, или нужно также останавливать stale GUI client processes, которые могут показывать старую scene?
- На текущем `main` после `d169f72` follow camera все еще broken в GUI, или regression уже отличается от исходного `41bb70e` case?
