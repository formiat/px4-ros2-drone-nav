# Context

Нужно запланировать исправление всего, что найдено в `INVESTIGATION.md` и
остается актуальным на текущей ветке `main`.

Текущий `main` уже содержит часть корректного направления:

- GUI запускается прямым путем `gz sim -g`, без `--gui-config`
  (`scripts/run_city_mvp.sh:501`).
- World unpause выполняется отдельно через Gazebo world control
  (`scripts/run_city_mvp.sh:408`, `scripts/run_city_mvp.sh:425`,
  `scripts/run_city_mvp.sh:429`).
- Есть startup cleanup stale Gazebo server processes
  (`scripts/run_city_mvp.sh:169`, `scripts/run_city_mvp.sh:205`).

Оставшиеся актуальные проблемы:

1. Startup cleanup слишком узкий: сейчас он ищет только `gz sim` с конкретным
   runtime world path (`scripts/run_city_mvp.sh:173-181`). Пользователь прямо
   указал, что если проблема в stale/duplicate Gazebo processes, решение должно
   быть постоянным и капитальным: перед запуском аккуратно убивать все
   conflicting Gazebo instances, а не один раз вручную.
2. Follow-camera все еще не имеет надежной runtime-проверки фактического
   состояния GUI camera. Текущий код проверяет только принятие `/gui/follow`
   и `/gui/follow/offset`, затем публикует `/gui/track`
   (`scripts/run_city_mvp.sh:336-405`), но не доказывает, что активная камера
   действительно перешла в follow mode.
3. Поведение GUI-launch helpers почти не покрыто автотестами: `scripts/tests`
   сейчас содержит тесты валидаторов, но нет тестов для `run_city_mvp.sh`,
   stale-process selection, world unpause и follow-camera setup.
4. Документация описывает cleanup как остановку stale servers "for the same
   runtime world" (`README.md:85-93`), что уже недостаточно для целевого
   капитального решения.

# Investigation context

`INVESTIGATION.md` прочитан и использован как входные данные.

Ключевые выводы расследования:

- Проблема 1 (`718deec` OK, `cf8b9b1` bad): между коммитами не менялись SDF,
  world, spawn pose, PX4 target или camera code. Менялась cleanup-логика в
  `scripts/run_city_mvp.sh`. Stale/duplicate Gazebo process state остается
  ведущей гипотезой, но не закрытым root cause без live `ps`/GUI capture.
- Проблема 2a (`b4b9e8c`): follow-camera работала, но GUI мог навязать paused
  state из default Gazebo GUI config с `WorldControl.start_paused=true`.
- Проблема 2b (`41bb70e`): попытка лечить pause через runtime `--gui-config`
  смешала управление состоянием симуляции и camera tracking, из-за чего
  follow-camera регрессировала.
- Правильное направление: не использовать GUI config для pause, запускать GUI
  прямым путем, unpause делать через world control, stale processes чистить на
  старте, camera tracking подтверждать отдельной проверкой.

То, что уже реализовано на `main`, нужно закрепить тестами и усилить. То, что
осталось гипотезой, нужно закрывать инструментированием и безопасным cleanup.

# Detected stack/profiles

- Основной стек: ROS 2 workspace, Gazebo Harmonic, PX4 SITL, C++/ament CMake,
  helper shell/Python scripts.
- Прочитаны обязательные protocol/profile файлы:
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  - `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`
- Rust profile не применяется: `rg --files -g 'Cargo.toml' -g 'Cargo.lock'
  -g '*.rs'` не нашел Rust manifest/source в workspace.
- Существующие build dirs и compile databases есть в `build-host/` и `build/`;
  новые ad-hoc CMake dirs создавать не нужно.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- `make host-build`
- `make host-test`
- `make host-test-scripts`
- `make host-quality`
- `make host-format`
- `make host-sim-gui`
- `make host-sim-headless`
- `make host-speed-sweep`
- container equivalents: `make build`, `make test`, `make test-scripts`,
  `make quality`, `make format`, `make sim-gui`, `make sim-headless`

Для routine agent work предпочтителен native host workflow через `host-*`
targets.

# Affected components

- Main simulation launcher:
  - `scripts/run_city_mvp.sh`
    - stale cleanup: `stop_stale_gazebo_servers()` at
      `scripts/run_city_mvp.sh:169`
    - GUI follow setup: `configure_gazebo_gui_follow_camera()` at
      `scripts/run_city_mvp.sh:336`
    - world unpause: `configure_gazebo_world_running()` at
      `scripts/run_city_mvp.sh:408`
    - Gazebo server/GUI launch block at `scripts/run_city_mvp.sh:476-517`
- Host wrapper:
  - `scripts/run_city_mvp_host.sh`
- Script tests:
  - existing `scripts/tests/test_validate_city_mvp_headless.py`
  - new tests to add under `scripts/tests/`
- Documentation:
  - `README.md:73-93`
  - possibly `docs/MVP_SIMULATION.md` if GUI troubleshooting instructions need
    a dedicated note
- Optional helper modules to make launch behavior testable:
  - new `scripts/gazebo_process_cleanup.py`
  - new `scripts/gazebo_gui_control.py` or `scripts/lib/gazebo_gui_control.sh`

# Implementation steps

1. Заменить узкий cleanup на тестируемый cleanup conflicting Gazebo processes.

   Files:
   - `scripts/run_city_mvp.sh`
   - new `scripts/gazebo_process_cleanup.py`
   - new `scripts/tests/test_gazebo_process_cleanup.py`

   Code anchors:
   - replace/narrow `stop_stale_gazebo_servers()` at
     `scripts/run_city_mvp.sh:169-203`
   - keep call before ROS setup and before runtime resource generation at
     `scripts/run_city_mvp.sh:205`

   Materialized result:
   - Add a Python helper that parses `ps -eo pid=,ppid=,pgid=,cmd=` output and
     returns candidate PIDs for conflicting Gazebo processes.
   - Match all stale/conflicting Gazebo simulator processes, not only the
     current runtime world path.
   - Protect the current script process, its ancestors, and the current helper
     process from being selected.
   - Keep the actual kill sequence in one place: TERM, bounded wait, KILL for
     remaining candidates.
   - Log every candidate with `pid`, `ppid`, `pgid`, and command before killing.

   Suggested selection contract:

   ```python
   def is_conflicting_gazebo_process(cmd: str) -> bool:
       normalized = " ".join(cmd.split())
       return (
           " gz sim " in f" {normalized} "
           or normalized.endswith(" gz sim")
           or "/gz sim " in normalized
       )
   ```

   The implementation should include tests for:
   - stale `gz sim -s ... generated_city.sdf` is selected;
   - stale `gz sim -g` GUI process is selected;
   - current script PID / helper PID / ancestor PIDs are not selected;
   - unrelated commands containing words like `gazebo` in arguments are not
     selected;
   - multiple stale Gazebo processes are selected deterministically.

2. Add an explicit safety switch and dry-run mode for cleanup.

   Files:
   - `scripts/run_city_mvp.sh`
   - `scripts/gazebo_process_cleanup.py`
   - `README.md`
   - `scripts/tests/test_gazebo_process_cleanup.py`

   Code anchors:
   - launcher env var block near `scripts/run_city_mvp.sh:102-108`
   - cleanup call near `scripts/run_city_mvp.sh:205`

   Materialized result:
   - Default behavior: cleanup enabled because the user stated that multiple
     Gazebo instances are not planned on this machine.
   - Add `DRONE_GAZEBO_CLEAN_STALE_PROCESSES=true|false`.
   - Add `DRONE_GAZEBO_CLEAN_STALE_DRY_RUN=true|false` for diagnostics.
   - Dry-run logs candidates but does not kill.
   - Disabled mode logs a warning that stale process cleanup is off.

   Expected shell sketch:

   ```bash
   clean_stale_gazebo_processes() {
     if ! bool_is_true "${clean_stale_gazebo_processes_enabled}"; then
       echo "WARNING: stale Gazebo process cleanup is disabled"
       return 0
     fi
     python3 "${repo_root}/scripts/gazebo_process_cleanup.py" \
       --self-pid "$$" \
       --mode "${cleanup_mode}"
   }
   ```

3. Make Gazebo GUI control helpers testable without launching Gazebo.

   Files:
   - `scripts/run_city_mvp.sh`
   - new `scripts/lib/gazebo_gui_control.sh` or
     new `scripts/gazebo_gui_control.py`
   - new `scripts/tests/test_gazebo_gui_control.py`

   Code anchors:
   - extract `configure_gazebo_gui_follow_camera()` from
     `scripts/run_city_mvp.sh:336-405`
   - extract `configure_gazebo_world_running()` from
     `scripts/run_city_mvp.sh:408-449`

   Materialized result:
   - Move command construction and response parsing into a helper with injectable
     `gz` executable path or fake command runner.
   - Unit-test command payloads for:
     - `/gui/follow`;
     - `/gui/follow/offset`;
     - `/gui/track`;
     - `/world/${world_name}/control` with `pause: false`.
   - Unit-test response parsing:
     - `data: true` is success;
     - timeout/stderr is retryable failure;
     - malformed offset is rejected before calling `gz`.

4. Strengthen follow-camera setup with confirmation and retry semantics.

   Files:
   - `scripts/run_city_mvp.sh`
   - helper from step 3
   - `scripts/tests/test_gazebo_gui_control.py`

   Code anchors:
   - `configure_gazebo_gui_follow_camera()` behavior currently returns after
     the first accepted `/gui/follow` response at `scripts/run_city_mvp.sh:373`
   - `/gui/track` publish currently happens once at `scripts/run_city_mvp.sh:382`

   Materialized result:
   - Keep direct GUI launch; do not reintroduce `--gui-config`.
   - Publish `/gui/track` more than once during the follow wait window, or until
     a confirmation signal is observed.
   - Add a confirmation phase that tries known Gazebo GUI tracking state
     endpoints if available in the running GUI, for example `/gui/currently_tracked`
     or camera pose tracking topic. If no confirmation endpoint exists in this
     Gazebo build, log a clear `WARN` with the attempted endpoints and keep the
     accepted command path as best effort.
   - Do not show or depend on the visible `CameraTrackingConfig` panel.

   Pseudocode:

   ```text
   for attempt in 1..wait:
     call /gui/follow target
     call /gui/follow/offset offset
     publish /gui/track CameraTrack(FOLLOW, target, offset)
     if tracking_state_confirms(target):
       log confirmed
       return success
     sleep 1
   log warning with last responses
   ```

5. Keep world unpause separate and make it observable.

   Files:
   - `scripts/run_city_mvp.sh`
   - helper from step 3
   - `scripts/tests/test_gazebo_gui_control.py`
   - optionally `scripts/validate_city_mvp_headless.py`
   - `scripts/tests/test_validate_city_mvp_headless.py`

   Code anchors:
   - current unpause loop at `scripts/run_city_mvp.sh:408-449`
   - call site at `scripts/run_city_mvp.sh:504`

   Materialized result:
   - Keep `pause: false` world-control loop.
   - Log attempts, first success, and final confirmation in a stable format.
   - Add unit tests that require three consecutive accepted unpause responses,
     matching current behavior.
   - Add optional validator checks for `Gazebo world running command confirmed`
     when GUI logs are supplied. This should be optional because headless runs do
     not launch GUI.

6. Add static regression tests that forbid the old broken GUI config path.

   Files:
   - new `scripts/tests/test_run_city_mvp_launch_contract.py`
   - `scripts/run_city_mvp.sh`

   Materialized result:
   - Test that `scripts/run_city_mvp.sh` does not contain `--gui-config` in the
     Gazebo GUI launch path.
   - Test that `gz sim -g` is still present for GUI launch.
   - Test that `pause: false` is sent to `/world/${world_name}/control`.
   - Test that stale cleanup is called before runtime resources and before
     `gz sim` launch.

   This is intentionally a contract test. It protects the regression found in
   `41bb70e` where pause was fixed by changing the GUI config path.

7. Update docs to reflect the stronger permanent cleanup policy.

   Files:
   - `README.md`
   - optionally `docs/MVP_SIMULATION.md`

   Code anchors:
   - current README GUI section at `README.md:73-93`

   Materialized result:
   - Replace "same runtime world" wording with "conflicting Gazebo simulator
     processes".
   - Document default cleanup behavior and the disable/dry-run environment
     variables.
   - Document that multiple Gazebo instances are intentionally unsupported for
     this project workflow.
   - Document where to look in `log-host/gz_city_mvp.log` for cleanup,
     world-unpause, and follow-camera diagnostics.

8. Add launch-log validation for the GUI path.

   Files:
   - new `scripts/validate_gazebo_gui_launch_log.py`
   - new `scripts/tests/test_validate_gazebo_gui_launch_log.py`
   - optionally `Makefile`

   Materialized result:
   - Parse `log-host/gz_city_mvp.log`.
   - Validate:
     - stale cleanup ran or explicitly logged no candidates;
     - world unpause was confirmed;
     - follow camera was configured or produced a clear warning;
     - no `--gui-config` launch marker appears.
   - Add `make host-test-scripts` coverage through Python unittest discovery.

9. Add a bounded GUI smoke procedure, but keep it separate from headless CI.

   Files:
   - `README.md`
   - optionally `scripts/run_city_mvp.sh` if a machine-readable GUI status log
     flag is needed

   Materialized result:
   - Document a short manual/live verification command for the user:
     `make host-sim-gui`.
   - Define expected log lines for success:
     - stale cleanup summary;
     - `Gazebo world running command confirmed`;
     - follow camera configured/confirmed;
     - no warnings about `--gui-config`.
   - Keep this as manual fallback because Gazebo GUI visual correctness cannot
     be fully proven by current headless tests.

# Verification plan

For the implementation batch:

1. Script syntax and static contract checks:
   - `bash -n scripts/run_city_mvp.sh`
   - `python3 -m unittest discover scripts/tests`
   - `make host-test-scripts`
2. C++/workspace quality gate:
   - `make host-format`
   - `make host-quality`
3. Headless smoke after script changes:
   - `make host-sim-headless`
   - inspect `log-host/gz_city_mvp.log`, `log-host/px4_city_mvp.log`, and
     `log-host/ros_city_mvp.log` for startup regressions.
4. GUI-specific verification:
   - `make host-sim-gui`
   - use the new GUI launch-log validator against `log-host/gz_city_mvp.log`.
   - visually confirm drone model and follow camera once, because actual Gazebo
     3D rendering/camera behavior is not fully observable in headless tests.
5. Container compatibility, only if launch helper paths or dependencies become
   container-sensitive:
   - `./scripts/dev_shell.sh`
   - inside container: `make test-scripts`, `make quality`

Skipped by default:

- Remote/SSH/HTTP checks are forbidden by workflow policy.
- Full visual GUI assertion is not automatable with the current project
  tooling; the plan adds log validation and fake-`gz` unit tests, then keeps a
  single manual visual check as fallback.

# Testing strategy

## Category 1: без рефакторинга

Use for simple text/log contracts:

- `scripts/tests/test_run_city_mvp_launch_contract.py`
  - assert `--gui-config` is not used;
  - assert direct `gz sim -g` launch remains;
  - assert world-control `pause: false` path remains;
  - assert stale cleanup call appears before Gazebo launch.
- `scripts/tests/test_validate_gazebo_gui_launch_log.py`
  - happy path: cleanup summary, world running confirmed, follow configured;
  - negative path: missing world unpause confirmation fails;
  - edge case: follow-camera warning is reported as warning, not silent pass.

## Category 2: лёгкий рефакторинг

Use for behavior currently embedded in `run_city_mvp.sh`:

- Extract stale process selection into `scripts/gazebo_process_cleanup.py`.
  - happy path: stale server and GUI candidates selected;
  - negative path: unrelated commands and protected PIDs not selected;
  - edge case: command with extra whitespace, absolute `gz` path, wrapper path.
- Extract GUI command construction/response parsing into a helper.
  - happy path: fake `gz` returns `data: true` for follow/offset/unpause;
  - negative path: timeout or `data: false` retries and warns;
  - edge case: malformed offset never calls fake `gz`.

## Category 3: тяжёлый

Use only if light tests do not provide enough confidence:

- Add an optional pseudo-terminal or fake-process integration test that starts
  disposable dummy `gz sim`-named processes and verifies cleanup kills only those
  candidates. This is more fragile because process names and shell wrappers vary
  by OS.
- Add GUI smoke automation with a virtual display only if the local Gazebo GUI
  can run reproducibly under the test harness. Until then, visual GUI behavior
  remains a documented manual fallback.

# Risks and tradeoffs

- Aggressive cleanup is intentionally broader than the current world-path-only
  filter. This matches the user's workflow assumption that multiple Gazebo
  instances are unsupported, but it must still avoid killing the current script,
  shell ancestors, helper process, and unrelated commands.
- Adding Python helpers for launch behavior improves testability but adds one
  more script dependency path. Keep helpers dependency-free and covered by
  `make host-test-scripts`.
- Repeated `/gui/track` publishing can be noisy if the GUI is not ready. Use
  throttled logs and bounded attempts.
- Treating unavailable GUI tracking-state endpoints as fatal could make the
  launcher fail on Gazebo builds that do not expose those endpoints. Prefer:
  command accepted = best effort, explicit state confirmation = stronger
  success, missing endpoint = warning with diagnostics.
- Actual 3D model visibility cannot be fully proven by headless tests. The best
  automated coverage is process cleanup, command construction, log validation,
  and absence of the known-bad `--gui-config` path.

# Что могло сломаться

- Поведение:
  - cleanup может убить активный пользовательский Gazebo процесс. Проверка:
    unit tests for process selection, dry-run mode, and logs listing candidate
    PIDs before kill.
  - cleanup может пропустить stale process with unusual command shape. Проверка:
    tests with `/path/to/gz sim`, wrapper commands, extra whitespace, GUI/server
    variants.
  - launcher can hang waiting for GUI confirmation. Проверка: bounded waits,
    fake-`gz` timeout tests, and `make host-sim-headless` unaffected by GUI.
- API/контракты:
  - environment variables for cleanup can be misread. Проверка: tests for
    `DRONE_GAZEBO_CLEAN_STALE_PROCESSES` and
    `DRONE_GAZEBO_CLEAN_STALE_DRY_RUN` parsing.
  - log validator can become too strict. Проверка: tests for warning vs failure
    cases and docs describing required log lines.
- Интеграции:
  - host/container paths can differ. Проверка: keep helpers path-relative to
    `repo_root`; run `make host-test-scripts`, and optionally container
    `make test-scripts`.
  - Gazebo topic/service names can differ by version. Проверка: fake-`gz` unit
    tests for command construction plus real GUI smoke as fallback.
- Производительность/ресурсы:
  - repeated GUI control calls add startup overhead. Проверка: bounded
    `GZ_GUI_FOLLOW_WAIT_S` and logs showing attempt counts.
  - process scanning at startup is cheap, but should happen once. Проверка:
    logs and unit tests for single cleanup invocation before launch.

# Open questions

- Which Gazebo GUI tracking confirmation endpoint is reliable in the local
  Harmonic build: `/gui/currently_tracked`, `/gui/camera/pose`, or only
  accepted service/topic responses? If none is reliable, the implementation
  should keep confirmation as best-effort warning and rely on manual GUI smoke
  for final visual proof.
- Should the cleanup helper kill only `gz sim` or also other Gazebo-related
  client wrappers if they are observed in local `ps` output during a bad run?
  Initial implementation should start with `gz sim` because that is the known
  server/GUI command path.
- Should `DRONE_GAZEBO_CLEAN_STALE_PROCESSES=false` be documented as a
  developer escape hatch only, or should it be hidden to avoid users disabling a
  safety fix?
