# Context

Нужно добавить `Goal-Biased Receding-Horizon Trajectory Rollout Planner` как основной
источник XY-маршрута только для `no-static` режима. Static-режим сохраняет текущий
pipeline на A*. В no-static A* не удаляется: он становится асинхронным recovery/guide
planner для локальных тупиков. Existing vertical profile, speed profile, known-passage
constraints, mandatory known-solid validation, safe trajectory truncation и PX4 offboard
control сохраняются.

Целевой контракт: ровно один orchestration layer публикует `ExecutableTrajectory`.
Rollout и A* не конкурируют как независимые publishers.

# Investigation context

`INVESTIGATION.md` в workspace отсутствует. План основан на прямом чтении текущего
кода, конфигурации, тестов и repository workflow.

# Detected stack/profiles

- ROS 2 C++ workspace (`CMakeLists.txt`, `.cpp/.hpp`, ROS messages, `colcon`).
- PX4 offboard + Gazebo simulation.
- Прочитаны обязательные profiles: `generic.md` и `cpp.md`.
- Прочитаны `AGENTS.md`, `README.md`, `CONTRIBUTING.md`,
  `CPP_BEST_PRACTICES.md`.
- Notion/GitLab не упомянуты в task prompt; при `notion_policy=optional` удалённые
  reads не выполнялись. SSH/HTTP не использовались.

# Repo-approved commands found

- Host/container wrappers: `./scripts/build.sh`, `./scripts/test.sh`,
  `./scripts/sim_headless.sh`, `./scripts/sim_gui.sh`.
- Container entry: `./scripts/dev_shell.sh`.
- В контейнере: `make build`, `make test`, `make test-scripts`, `make format`,
  `make quality`, `make sim-headless`, `make sim-gui`.
- Для реализации использовать только container workflow. Перед commit после C++
  изменений: `make format`, затем `make quality`.

# Affected components

- `drone_city_nav/include/drone_city_nav/receding_horizon_trajectory_planner.hpp`
  и новый `.cpp`: чистый rollout core, generation, validation, scoring.
- `drone_city_nav/src/planner_node_runtime.cpp:223`: выбор normal rollout,
  safe-truncation и recovery triggers вместо безусловного no-static A*.
- `drone_city_nav/src/planner_node_route_publication.cpp`: A* recovery route как guide,
  а не параллельная offboard-команда.
- `drone_city_nav/src/planner_node_lifecycle.cpp:19-110`,
  `planner_node.hpp`: worker snapshots, state machine и единственный publisher.
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:128-215`:
  downstream finalization победившего локального route.
- `drone_city_nav/src/planning_grid_builder.cpp` и
  `planning_grid_snapshot.*`: immutable raw/prohibited/planning snapshots и revisions.
- `drone_city_nav/msg/ExecutableTrajectory.msg`: явная семантика local horizon, если
  её нельзя надёжно вывести из существующих полей.
- `drone_city_nav/src/px4_offboard_node_*`: отсутствие terminal capture на
  `LOCAL_HORIZON`, сохранение текущих mission-goal и temporary-hold семантик.
- `drone_city_nav/src/planner_node_config.cpp`,
  `drone_city_nav/config/urban_mvp.yaml`: no-static-only rollout/recovery параметры.
- Diagnostics и unit/integration tests в `drone_city_nav/tests`.

# Implementation steps

1. **Добавить чистое rollout-ядро и типизированный контракт результата.**
   Создать `receding_horizon_trajectory_planner.hpp/.cpp` с
   `RolloutPlannerConfig`, `RolloutInput`, `RolloutCandidate`,
   `RolloutResult` и reject/quality diagnostics. Генерировать конечный,
   детерминированный набор кривых из pose, velocity и preferred target; sampling
   должен быть ограничен horizon и не зависеть от полного размера города.

   ```cpp
   result = rollout.plan({
     .navigation = snapshot.navigation,
     .preferred_target = activeGuideLookaheadOrMissionGoal(),
     .raw_grid = snapshot.raw_grid,
     .prohibited_grid = snapshot.prohibited_grid,
     .planning_grid = snapshot.planning_grid,
   });
   ```

   Raw occupied является hard reject. Planning/prohibited clearance, progress,
   lateral deviation, heading change, curvature и continuity формируют score.
   Перебор выполняется tiers `planning -> prohibited -> raw-clear`, чтобы degraded
   физически свободный полёт оставался возможен.

2. **Сделать кандидаты сразу динамически исполнимыми.**
   Использовать current XY velocity/tangent как начальные условия и параметрические
   дуги/короткие polynomial samples, а не запускать corridor/optimizer для каждого
   candidate. Проверять полный sampled span по grids и known solid volumes.
   Downstream `finalizeStitchedTrajectory()` вызывается один раз для победителя;
   existing vertical/passages/speed stages не дублируются.

3. **Добавить no-static planner state machine и arbiter.**
   В `planner_node.hpp`/новом `no_static_planner_orchestrator.*` ввести состояния
   `kDirectGoalRollout`, `kAstarRecoveryRunning`, `kAstarGuidedRollout`,
   `kTemporaryHold`. Static branch продолжает существующий A* flow без изменения.
   Только arbiter может передать candidate в current publication path; generation
   и `PlanningGridVersion` блокируют stale results.

4. **Реализовать стабильный executable prefix и hysteresis.**
   Проецировать current pose на опубликованную trajectory, сохранять configurable
   prefix на 1-2 секунды и заменять только suffix. Новый rollout принимается, если
   текущий suffix blocked/исчерпывается либо улучшение score превышает threshold.
   Проверить stitch position/tangent/Z и пересчитать speed profile для итоговой
   trajectory. Safe truncation остаётся fallback, если свежего suffix нет.

5. **Разделить terminal semantics.**
   Добавить тип endpoint `MISSION_GOAL | LOCAL_HORIZON | TEMPORARY_REPLAN_HOLD`
   в core/message contract и ROS conversion. В offboard `LOCAL_HORIZON` не запускает
   terminal slowdown/final hold; buffer exhaustion без следующего suffix переводит
   выполнение в существующий safe-truncation/temporary-hold flow. Настоящий goal и
   temporary hold сохраняют текущую state-machine семантику.

6. **Понизить A* до recovery guide в no-static.**
   В `planner_node_runtime.cpp:239-386` заменить no-static immediate A* trigger на
   progress monitor: no valid rollout N cycles, отсутствие продвижения T секунд,
   repeated left/right switching или temporary hold. A* выполняется текущим worker
   асинхронно. Его результат сохраняется как guide polyline; rollout получает
   lookahead point на guide, а не допускает вторую непосредственную публикацию.
   Existing partial repair применяется к blocked recovery guide; full A* остаётся
   fallback. Static flow не меняется.

7. **Добавить конфигурацию и диагностику.**
   В `planner_node_config.cpp` и `urban_mvp.yaml` добавить no-static-only:
   enable flag, horizon, cycle period, heading/curvature/speed samples, scoring
   weights, prefix duration, hysteresis, progress timeout, recovery lookahead.
   Валидировать bounds. Логировать generation, mode, candidate counts/reject reasons,
   selected tier/score, planning duration, prefix/suffix lengths, recovery trigger,
   guide revision и transition. Расширить summary counters без изменения существующих
   A* counters.

8. **Добавить Category 1 automated tests и ROS contract coverage.**
   Новый `receding_horizon_trajectory_planner_test.cpp`: прямой open-space candidate,
   left/right обход, raw collision reject, degraded tier, dynamics, deterministic
   tie-break, no candidate. Новый orchestrator test: hysteresis, immutable prefix,
   stale generation/grid rejection, recovery trigger/exit, single-publication rule.
   Обновить config tests, message/round-trip tests и offboard tests: local horizon не
   тормозит как mission goal; buffer exhaustion безопасно удерживается. Добавить
   integration test, доказывающий, что static mode продолжает выбирать A*, а
   no-static blocker сначала обрабатывается rollout.

# Verification plan

Mandatory local verification после реализации:

1. `./scripts/dev_shell.sh make format` — форматирует только изменённый C++ по
   repository policy.
2. `./scripts/dev_shell.sh make quality` — общий контракт нужен из-за новых shared
   headers, ROS message, planner/offboard integration и generated message code;
   точечный unit target не покрывает эти compile/serialization границы.
3. `./scripts/dev_shell.sh make test-scripts` — только если меняются headless
   validator/scripts; иначе skipped как не относящийся к контракту.

Optional/CI verification:

- Три `ENABLE_STATIC_MAP=false` headless runs для runtime latency, oscillation,
  recovery и mission behavior.
- Один static headless run как regression проверки неизменённого A* режима.
- GUI/RViz run только для визуального исследования trajectory churn; это не замена
  автотестам.

# Testing strategy

Категория 1 обязательна и реализуема без отдельного рефакторинга: pure unit tests
rollout/scoring, state-machine/arbiter tests, config clamps, message round-trip,
offboard endpoint semantics и planner mode integration.

Happy path: свободный прямой полёт и локальный обход. Negative path: все candidates
пересекают raw occupied, stale worker result, invalid samples. Edge cases: нулевая
velocity, goal внутри horizon, старт рядом со стеной, равные scores, смена grid
revision, recovery result после возврата direct rollout.

Категория 2 optional follow-up: deterministic recorded lidar/grid replay benchmark
для сравнения A* и rollout latency/trajectory churn. Он требует выделения replay
harness и не блокирует основную реализацию.

# Risks and tradeoffs

- **Локальные тупики:** rollout не является complete planner. Смягчение: измеримый
  progress detector и A* recovery guide.
- **Дёрганье от частых updates:** prefix/hysteresis обязательны; нельзя публиковать
  новый полный path каждый tick.
- **Ложный terminal slowdown:** local horizon требует отдельной wire semantics и
  offboard tests.
- **CPU load:** фиксировать верхнюю границу candidates/samples и логировать P50/P95;
  не запускать тяжёлый trajectory pipeline на каждом candidate.
- **Grid race:** worker использует только immutable snapshot и revision; stale
  результаты отбрасываются.
- **Passage/vertical mismatch:** rollout заменяет только XY source; mandatory
  known-solid validation и existing vertical profile остаются gate.
- **Два publisher-а:** запрещены контрактом arbiter; A* guide не публикуется напрямую
  в normal recovery path.
- **Поведение static mode:** mode dispatch и regression test должны доказать
  byte/semantic-equivalent выбор старого A* pipeline.

# Open questions

1. **Нужен ли новый field в `ExecutableTrajectory.msg` для endpoint semantics?**
   Recommended decision: добавить явный enum. Rationale: выводить local horizon из
   path length/path ID хрупко и может вызвать terminal hold. Подтверждение: проверить
   все consumers сообщения при реализации.

2. **Какой первый candidate generator выбрать?**
   Recommended decision: bounded constant-curvature/polynomial fan с current velocity
   boundary condition, не переиспользовать full optimizer внутри перебора. Это
   обеспечивает предсказуемую latency; параметры уточнить recorded-grid benchmark.

3. **Должен ли A* recovery path когда-либо публиковаться напрямую?**
   Recommended decision: нет в production normal flow; использовать его как guide.
   Direct publication оставить только отдельным явно диагностируемым emergency
   fallback, если rollout core возвращает internal failure, а A* trajectory проходит
   все текущие validators.

4. **Какие начальные частоты и horizon?**
   Recommended decision: начать с 5 Гц и 25 м (lidar effective range 30 м), prefix
   1.0 с; перейти к 10 Гц только после P95 profiling. Это сохраняет запас sensor range
   и ограничивает CPU.

5. **Нужна ли полная 3D rollout-оптимизация?**
   Recommended decision: нет в первой задаче. Сохранить существующий vertical/passage
   pipeline; 3D rollout вынести в отдельный follow-up после доказательства XY MVP.
