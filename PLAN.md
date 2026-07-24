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
  `drone_city_nav/src/planner_node.hpp`: worker snapshots, state machine и
  единственный publisher.
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:128-215`:
  downstream finalization победившего локального route.
- `drone_city_nav/src/planning_grid_builder.cpp` и
  `planning_grid_snapshot.*`: immutable raw/prohibited/planning snapshots и revisions.
- `drone_city_nav/msg/ExecutableTrajectory.msg`,
  `drone_city_nav/src/planner_node_debug_publication.cpp:158-218`,
  `drone_city_nav/src/px4_offboard_node_trajectory.cpp:415-475,900-930`:
  wire contract, producer и consumer семантики local horizon.
- `drone_city_nav/src/px4_offboard_node_control.cpp:166-211`,
  `drone_city_nav/src/px4_offboard_node_control.cpp:228-243,369-452`,
  `drone_city_nav/src/px4_offboard_node_replan.cpp:62-99`,
  `drone_city_nav/src/terminal_capture_state_machine.cpp`: terminal capture,
  permanent final-goal latch, cruise/hold predicates и bounded local-horizon
  exhaustion transition.
- `drone_city_nav/src/planner_node_config.cpp`,
  `drone_city_nav/config/urban_mvp.yaml`: no-static-only rollout/recovery параметры.
- `drone_city_nav/include/drone_city_nav/px4_offboard_node_config.hpp`,
  `drone_city_nav/src/px4_offboard_node_config.cpp`: bounded local-horizon buffer и
  successor timeout.
- `drone_city_nav/CMakeLists.txt:23-34,70-137,185-202,625-650,652-681,757-767`:
  регистрация message, новых core/node sources и каждого нового/изменённого gtest.
- Конкретные tests:
  `tests/receding_horizon_trajectory_planner_test.cpp`,
  `tests/no_static_planner_orchestrator_test.cpp`,
  `tests/trajectory_planner_test.cpp`,
  `tests/known_passage_solid_validation_test.cpp`,
  `tests/terminal_capture_state_machine_test.cpp`,
  `tests/offboard_velocity_follower_terminal_capture_test.cpp`,
  `tests/planner_node_config_test.cpp`, `tests/px4_offboard_node_config_test.cpp`,
  `tests/ros_conversions_test.cpp`.

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
   Добавить `src/receding_horizon_trajectory_planner.cpp` в
   `drone_city_nav_core` (`drone_city_nav/CMakeLists.txt:70-137`) и зарегистрировать
   `receding_horizon_trajectory_planner_test` рядом с
   `trajectory_planner_test` (`CMakeLists.txt:625-638`). Результат шага: pure C++
   API генерирует и ранжирует bounded XY candidates без ROS/worker side effects.

2. **Разделить дешёвый XY ranking и обязательную post-vertical 3D validation.**
   Использовать current XY velocity/tangent как начальные условия и параметрические
   дуги/короткие polynomial samples, а не запускать corridor/optimizer для каждого
   candidate. В `receding_horizon_trajectory_planner.cpp` pre-score проверяет только:
   конечность/структуру XY samples, current dynamics, raw occupied hard collision,
   traversability tiers и 2D clearance. Он **не** объявляет known-solid clear:
   текущий обязательный 3D gate в
   `trajectory_planner.cpp:111-130` работает только после
   `applyVerticalProfile()`.

   Ранжированный shortlist (recommended max `3`) последовательно передаётся в
   `finalizeStitchedTrajectory()` (`trajectory_planner.cpp:740-769`). Для каждого
   finalist existing `applyVerticalProfileStage()` назначает Z, затем
   `validateTrajectoryAgainstKnownPassageSolids()` выполняет mandatory 3D check.
   Если finalist отклонён по vertical/known-solid/passage contract, orchestration
   пробует следующий ranked candidate в том же immutable snapshot; после исчерпания
   bounded shortlist возвращает `kNoValidatedCandidate` и инициирует recovery/hold.

   ```cpp
   for (const RolloutCandidate& candidate : result.rankedShortlist(max_finalists)) {
     TrajectoryPlannerResult finalized = finalizeStitchedTrajectory(...);
     if (finalized.valid &&
         finalized.stats.known_passage_solid_validation.valid) {
       return finalized;
     }
   }
   return noValidatedCandidate();
   ```

   Обновить `tests/trajectory_planner_test.cpp` и
   `tests/known_passage_solid_validation_test.cpp`: лучший XY candidate с неверным
   post-vertical Z отклоняется, второй ranked candidate принимается; все finalists
   solid-invalid дают recovery, а не публикацию. Результат шага: bounded latency не
   противоречит mandatory 3D validation.

3. **Добавить no-static planner state machine и arbiter.**
   Создать
   `include/drone_city_nav/no_static_planner_orchestrator.hpp` и
   `src/no_static_planner_orchestrator.cpp`; в
   `drone_city_nav/src/planner_node.hpp:150-180,421-582` хранить его state/snapshot.
   Ввести состояния
   `kDirectGoalRollout`, `kAstarRecoveryRunning`, `kAstarGuidedRollout`,
   `kTemporaryHold`. Static branch продолжает существующий A* flow без изменения.
   Только arbiter может передать candidate в current publication path; generation
   и `PlanningGridVersion` блокируют stale results.
   Подключить core source и новый `no_static_planner_orchestrator_test` в
   `CMakeLists.txt:70-137` и test section. В
   `planner_node_lifecycle.cpp:109` переиспользовать единственный `planning_worker_`;
   не создавать второй publisher/thread. Результат: один типизированный decision
   (`publish`, `keep`, `request_recovery`, `hold`) на planning generation.

4. **Реализовать стабильный executable prefix и hysteresis.**
   В новом orchestrator использовать
   `ExecutableTrajectoryArtifact`/`updateExecutableTrajectoryProgress()` из
   `trajectory_repair.cpp:225` и текущий artifact в
   `planner_node.hpp:560`. Проецировать current pose, сохранять configurable
   prefix на 1-2 секунды и заменять только suffix. Новый rollout принимается, если
   текущий suffix blocked/исчерпывается либо улучшение score превышает threshold.
   Сшивку выполнять через новый pure helper в
   `no_static_planner_orchestrator.cpp`, финализацию — через
   `finalizeStitchedTrajectory()`; position/tangent/Z и speed profile проверяются
   до передачи в `publishTrajectoryPath()` в
   `planner_node_debug_publication.cpp:172-218`. Safe truncation в
   `planner_node_runtime.cpp:256-430` остаётся fallback.
   Tests в `no_static_planner_orchestrator_test.cpp`: immutable prefix, hysteresis,
   blocked-prefix bypass, stale generation/grid rejection.

5. **Разделить terminal semantics.**
   Добавить constants и поле `uint8 endpoint_semantics` со значениями
   `MISSION_GOAL | LOCAL_HORIZON | TEMPORARY_REPLAN_HOLD` в
   `msg/ExecutableTrajectory.msg`; schema уже зарегистрирована в
   `CMakeLists.txt:23-34`. Заполнять поле в обоих producer paths
   `planner_node_debug_publication.cpp:158-161,204-218`, читать и валидировать в
   `px4_offboard_node_trajectory.cpp:415-475`.

   Сохранить active semantics в `px4_offboard_node.hpp:470-515` и передать флаг
   terminal eligibility в `computeTerminalCaptureState()`:
   `px4_offboard_node_control.cpp:172-201` /
   `terminal_capture_state_machine.cpp`.

   Отдельно провести semantics через независимый final-goal path:

   - в `px4_offboard_node.hpp:430-475` хранить
     `active_trajectory_endpoint_semantics_`,
     `local_horizon_exhaustion_active_` и monotonic timestamp начала low-buffer;
   - в `px4_offboard_node_control.cpp:228-243` разделить общий геометрический
     predicate `activeTrajectoryEndpointReached()` и
     `missionGoalReached()`, который возвращает true только для
     `MISSION_GOAL`;
   - в `px4_offboard_node_replan.cpp:62-78` разрешить
     `updateFinalGoalHold()` устанавливать `final_goal_hold_active_` только при
     `MISSION_GOAL`; `LOCAL_HORIZON` никогда не меняет permanent latch;
   - обновить `velocityCruiseReady()` и `shouldHoldPosition()` в
     `px4_offboard_node_control.cpp:166-169,449-452`, чтобы конец local path сам по
     себе не трактовался как mission completion;
   - добавить `updateLocalHorizonExhaustionHold()` в
     `px4_offboard_node_replan.cpp` и вызывать его в control tick рядом с
     `updateFinalGoalHold()` (`px4_offboard_node_control.cpp:37-40`).

   Bounded exhaustion contract: offboard вычисляет remaining `s` по active samples.
   Когда `LOCAL_HORIZON` остаётся без принятого successor, remaining `s` не больше
   configurable `local_horizon_min_buffer_m` и это состояние непрерывно длится
   `local_horizon_successor_timeout_s` (recommended `0.5 s`), устанавливается
   `local_horizon_exhaustion_active_`. С этого момента terminal capture разрешается,
   terminal target остаётся endpoint текущего local path; после выполнения обычных
   distance/speed criteria защёлкивается **temporary** hold с отдельной reason
   `local_horizon_exhausted`. При приёме successor до latch timer/state очищается.
   При приёме successor после temporary hold он снимает только temporary state и
   возобновляет cruise. `final_goal_hold_active_` и mission success не меняются.
   Параметры объявить/валидировать в
   `include/drone_city_nav/px4_offboard_node_config.hpp`,
   `src/px4_offboard_node_config.cpp` и `config/urban_mvp.yaml`; обновить
   `tests/px4_offboard_node_config_test.cpp`.

   ```cpp
   if (endpoint_semantics == LOCAL_HORIZON &&
       remaining_s <= local_horizon_min_buffer_m &&
       successor_missing_for >= local_horizon_successor_timeout_s) {
     local_horizon_exhaustion_active_ = true; // enables terminal capture
   }
   if (local_horizon_exhaustion_active_ && activeTrajectoryEndpointReached()) {
     latchTemporaryHold("local_horizon_exhausted");
   }
   ```

   Вынести чистое transition decision в новый
   `include/drone_city_nav/local_horizon_execution_state.hpp` /
   `src/local_horizon_execution_state.cpp`, добавить source и
   `local_horizon_execution_state_test` в `CMakeLists.txt`. Tests:
   `LocalHorizonNeverLatchesFinalGoal`,
   `MissionGoalStillLatchesFinalGoal`,
   `LowBufferBeforeTimeoutKeepsCruise`,
   `LowBufferTimeoutEnablesTerminalCapture`,
   `EndpointCaptureLatchesTemporaryHold`,
   `SuccessorClearsPendingOrActiveTemporaryHold`.
   Обновить `terminal_capture_state_machine_test`,
   `offboard_velocity_follower_terminal_capture_test`, `ros_conversions_test` и
   message round-trip case. Результат: wire contract однозначен для producer,
   consumer, terminal capture и permanent/temporary hold paths.

6. **Понизить A* до recovery guide в no-static.**
   В `planner_node_runtime.cpp:239-386` заменить no-static immediate A* trigger на
   progress monitor: no valid rollout N cycles, отсутствие продвижения T секунд,
   repeated left/right switching или temporary hold. A* выполняется текущим worker
   асинхронно. Его результат сохраняется как guide polyline; rollout получает
   lookahead point на guide, а не допускает вторую непосредственную публикацию.
   Existing partial repair применяется к blocked recovery guide; full A* остаётся
   fallback. Static flow не меняется.
   Конкретно: mode dispatch добавить перед current
   `planner_core_.evaluateStablePath()` в `planner_node_runtime.cpp:239`; A* result
   из `planner_node_route_publication.cpp:560-602` в no-static записывать в
   orchestrator как guide revision вместо непосредственной publication, а static
   branch оставлять прежней. Tests:
   `no_static_planner_orchestrator_test.cpp` (`NoProgressRequestsRecovery`,
   `RecoveryGuideChangesPreferredTarget`, `StableProgressReturnsDirect`) и
   `planner_runtime_state_test.cpp` (`StaticModeKeepsAStarAction`).

7. **Добавить конфигурацию и диагностику.**
   В `include/drone_city_nav/planner_node_config.hpp`,
   `src/planner_node_config.cpp:370-405,676-705`,
   `src/planner_node.hpp` и `config/urban_mvp.yaml:213-229` добавить no-static-only:
   enable flag, horizon, cycle period, heading/curvature/speed samples, scoring
   weights, prefix duration, hysteresis, progress timeout, recovery lookahead.
   Валидировать bounds. Логировать generation, mode, candidate counts/reject reasons,
   selected tier/score, planning duration, prefix/suffix lengths, recovery trigger,
   guide revision и transition. Расширить summary counters без изменения существующих
   A* counters в `include/drone_city_nav/planner_diagnostics_format.hpp` /
   `src/planner_diagnostics_format.cpp` и publication logging в
   `src/planner_node_debug_publication.cpp`. Обновить
   `tests/planner_node_config_test.cpp` (defaults, clamps, invalid shortlist) и
   `tests/planner_diagnostics_format_test.cpp`. Результат: все knobs bounded, а
   причины rollout/recovery измеримы.

8. **Добавить Category 1 automated tests и ROS contract coverage.**
   Новый `receding_horizon_trajectory_planner_test.cpp`: прямой open-space candidate,
   left/right обход, raw collision reject, degraded tier, dynamics, deterministic
   tie-break, no candidate. Новый orchestrator test: hysteresis, immutable prefix,
   stale generation/grid rejection, recovery trigger/exit, single-publication rule.
   Обновить config tests, message/round-trip tests и offboard tests: local horizon не
   тормозит как mission goal; buffer exhaustion безопасно удерживается. Добавить
   integration test, доказывающий, что static mode продолжает выбирать A*, а
   no-static blocker сначала обрабатывается rollout.
   Зарегистрировать **каждый** новый target в `drone_city_nav/CMakeLists.txt`:
   `receding_horizon_trajectory_planner_test` и
   `no_static_planner_orchestrator_test`, `local_horizon_execution_state_test` link
   к `drone_city_nav_core`; существующие
   ROS/config/offboard targets расширяются без создания дубликатов. Добавить новые
   production `.cpp` в `drone_city_nav_core` и, только если orchestration останется
   node-owned, `planner_node` source list (`CMakeLists.txt:185-202`). Результат:
   новые production/test units реально входят в colcon/CTest build graph.

# Verification plan

Mandatory local verification после реализации:

1. `./scripts/dev_shell.sh make format` — форматирует только изменённый C++ по
   repository policy.
2. `./scripts/dev_shell.sh make build` — материализует новые core/node targets,
   gtest binaries и generated `ExecutableTrajectory` message до scoped CTest.
3. Первым test pass запустить только новые и materially changed targets:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
     --output-on-failure \
     -R '^(receding_horizon_trajectory_planner_test|no_static_planner_orchestrator_test|local_horizon_execution_state_test|trajectory_planner_test|known_passage_solid_validation_test|terminal_capture_state_machine_test|offboard_velocity_follower_test|planner_runtime_state_test|planner_diagnostics_format_test|planner_node_config_test|px4_offboard_node_config_test|ros_conversions_test)$'
   ```

   Этот exact regex локализует failures rollout/scoring, shortlist post-vertical
   validation, orchestrator/recovery, endpoint/final-hold state, config и ROS
   conversion contracts. Имена здесь являются CTest target names из implementation
   step 8, а не отдельным списком тестов, которые ещё нужно написать.
4. `./scripts/dev_shell.sh make quality` — отдельная широкая cross-component
   проверка после scoped pass. Она нужна не «на всякий случай», а потому что
   изменение shared headers и ROS message затрагивает generated interfaces,
   `drone_city_nav_core`, `drone_city_nav_ros_adapters`, planner/offboard linkage и
   static-mode consumers, которые один regex unit targets не компилирует и не
   проверяет целиком.
5. `./scripts/dev_shell.sh make test-scripts` — только если меняются headless
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
