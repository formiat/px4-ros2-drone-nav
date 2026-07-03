# Context

Планируем оптимизацию `Thread pool для racing_line candidates`.

Текущая реализация в `drone_city_nav/src/racing_line.cpp` создаёт новый набор
`std::thread` внутри каждого `evaluateCandidateBatch(...)`:

```text
optimizeRacingLine(...)
  for each optimizer iteration:
    tasks = candidateTasksForStep(...)
    evaluateCandidateBatch(...)
      create worker_buffers
      create N std::thread
      evaluate candidates by fixed slices
      join all std::thread
```

По последнему headless-прогону из `log/final_trajectory_samples/latest_summary.json`:

```text
trajectory_total_duration_ms = 794.066
trajectory_racing_line_duration_ms = 642.689
racing_candidate_chunks = 80
racing_parallel_workers_used = 16
racing_candidate_parallel_batches = 80
racing_candidate_threads_launched = 1280
racing_candidate_thread_launch_duration_ms = 64.549
racing_candidate_worker_buffer_prepare_duration_ms = 2.394
racing_candidate_batch_wall_duration_ms = 452.392
```

Прямо подтверждённый overhead от запуска потоков: около `64.5 ms` за refined
build. Цель оптимизации: создать локальный worker pool один раз на один вызов
`optimizeRacingLine(...)`, сохранить прежнее разбиение задач и прежний порядок
выбора winner, но убрать пересоздание `std::thread` на каждом chunk.

# Investigation context

`INVESTIGATION.md` в `workspace_root` отсутствует.

Существующий `PLAN.md` перед началом работы отсутствовал, поэтому этот файл
создаётся как новый план.

Прочитаны обязательные локальные протоколы:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`

В пользовательском запросе нет Notion task id и нет GitLab MR, поэтому
дополнительные read-only запросы в Notion/GitLab не требуются.

# Detected stack/profiles

Стек workspace:

- ROS 2 workspace.
- Основной пакет: `drone_city_nav`.
- C++20, `ament_cmake`, `colcon`.
- Core library: `drone_city_nav_core`.
- Тесты: `ament_add_gtest(...)` в `drone_city_nav/CMakeLists.txt`.

Прочитанные project profiles:

- `generic.md`: обязателен для любого workspace.
- `cpp.md`: применим, потому что есть `drone_city_nav/CMakeLists.txt`,
  `drone_city_nav/src/*.cpp`, `drone_city_nav/include/*.hpp`,
  `drone_city_nav/tests/*.cpp`.

Rust profile не применялся: в затронутом workspace нет признаков Rust-части
для этой задачи.

# Repo-approved commands found

Из `README.md`, `CONTRIBUTING.md`, `Makefile` и `AGENTS.md`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/sim_headless.sh`
- `./scripts/sim_gui.sh`
- `./scripts/stop_sim.sh`
- `./scripts/dev_shell.sh make build`
- `./scripts/dev_shell.sh make test`
- `./scripts/dev_shell.sh make test-scripts`
- `./scripts/dev_shell.sh make quality`
- `./scripts/dev_shell.sh make format`
- внутри контейнера допустим scoped `ctest --test-dir build/drone_city_nav --output-on-failure`

Для будущей реализации использовать только container workflow. Не запускать
ad-hoc top-level CMake.

# Affected components

- `drone_city_nav/src/racing_line.cpp`
  - `CandidateWorkBuffer` около `src/racing_line.cpp:148`
  - `desiredWorkerCount(...)` около `src/racing_line.cpp:229`
  - `evaluateCandidateBatch(...)` около `src/racing_line.cpp:1519`
  - optimizer loop в `optimizeRacingLine(...)` около `src/racing_line.cpp:1830`
  - `mergeCandidateStats(...)` около `src/racing_line.cpp:1627`
- `drone_city_nav/include/drone_city_nav/racing_line.hpp`
  - `RacingLineConfig`
  - `RacingLineStats`
- `drone_city_nav/src/planner_node_publish.cpp`
  - ROS summary строка `racing_line[...]` около `src/planner_node_publish.cpp:294`
- `drone_city_nav/src/trajectory_diagnostics_io.cpp`
  - JSON append/parse racing stats около `src/trajectory_diagnostics_io.cpp:380`
    и `src/trajectory_diagnostics_io.cpp:985`
- `drone_city_nav/tests/racing_line_test.cpp`
  - `DefaultParallelCandidateEvaluationMatchesSingleWorkerResult`
  - `LocalCandidatePrefilterKeepsFullObjectiveScoring`
- `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
- `drone_city_nav/tests/offboard_blackbox_test.cpp`
- `drone_city_nav/CMakeLists.txt`
  - существующие test targets уже есть, новых targets скорее всего не нужно.

# Implementation steps

1. Добавить локальный RAII worker pool в `drone_city_nav/src/racing_line.cpp`.

   Anchor: рядом с `CandidateWorkBuffer` и batch helpers, перед
   `evaluateCandidateBatch(...)`.

   Материализуемый результат:

   - новый internal type в anonymous namespace, например
     `RacingCandidateWorkerPool`;
   - pool создаёт `worker_count` потоков один раз;
   - destructor выставляет stop-флаг, будит worker-ы и делает `join`;
   - `run(active_workers, task_count, callback)` блокируется до завершения всех
     worker-ов;
   - worker-ы используют прежнее статическое разбиение на slices:

     ```cpp
     begin = task_count * worker_index / active_workers;
     end = task_count * (worker_index + 1U) / active_workers;
     ```

   Псевдокод контракта:

   ```cpp
   class RacingCandidateWorkerPool {
   public:
     explicit RacingCandidateWorkerPool(std::size_t worker_count,
                                        RacingLineStats& stats);
     ~RacingCandidateWorkerPool();

     template <class Fn>
     void run(std::size_t active_workers, std::size_t task_count, Fn&& fn);

   private:
     // mutex + condition_variable + generation counter + remaining_workers
     // workers never outlive run() callback references because run() waits.
   };
   ```

   Важно: callback должен жить до завершения `run(...)`; метод обязан дождаться
   всех worker-ов перед возвратом, чтобы ссылки на `tasks`, `results`,
   `base_offsets`, `best_points` не пережили stack frame.

2. Вынести per-build workspace для candidate evaluation.

   Файл: `drone_city_nav/src/racing_line.cpp`.

   Anchors:

   - `CandidateWorkBuffer` около `src/racing_line.cpp:148`
   - `evaluateCandidateBatch(...)` около `src/racing_line.cpp:1519`
   - optimizer loop около `src/racing_line.cpp:1830`

   Материализуемый результат:

   - добавить internal struct, например:

     ```cpp
     struct CandidateBatchWorkspace {
       std::vector<CandidateBatchResult> results;
       std::vector<CandidateWorkBuffer> worker_buffers;
     };
     ```

   - `worker_buffers` создаются/reserve один раз под `max_worker_count`, а не на
     каждый chunk;
   - `results` переиспользуется между iterations через `resize(tasks.size())`;
   - для каждого task всё равно перезаписывается `results[task_index].order` и
     `results[task_index].candidate`;
   - stale элементы после shrink не используются.

3. Переписать `evaluateCandidateBatch(...)` на использование pool/workspace.

   Файл: `drone_city_nav/src/racing_line.cpp`.

   Anchor: `evaluateCandidateBatch(...)` около `src/racing_line.cpp:1519`.

   Материализуемый результат:

   - сигнатура принимает `CandidateBatchWorkspace& workspace` и optional/local
     `RacingCandidateWorkerPool* pool`;
   - serial path (`resolved_workers == 1`) остаётся простым single-thread loop;
   - parallel path больше не создаёт `std::vector<std::thread>` внутри batch;
   - `pool.run(...)` вызывает тот же `evaluate_one(task_index, buffer)`;
   - `results[task_index]` заполняется по тому же индексу;
   - `mergeCandidateStats(...)` и winner selection остаются после batch и в
     прежнем последовательном порядке.

   Нельзя менять:

   - порядок `candidateTasksForStep(...)`;
   - условие winner selection;
   - `best_cost`, `offsets`, `best_points`, `best_score` во время parallel
     evaluation;
   - scoring math.

4. Создать pool один раз внутри `optimizeRacingLine(...)`.

   Файл: `drone_city_nav/src/racing_line.cpp`.

   Anchor: optimizer setup перед циклом `for (std::size_t iteration = 0U; ...)`
   около `src/racing_line.cpp:1830`.

   Материализуемый результат:

   - вычислить максимальный worker count для build через текущую
     `desiredWorkerCount(config.parallel_workers, max_possible_work_items)`;
   - создать `RacingCandidateWorkerPool` только если worker count > 1;
   - при малом количестве tasks внутри iteration использовать `active_workers =
     desiredWorkerCount(..., tasks.size())`;
   - pool не хранится в planner/global state и уничтожается при выходе из
     `optimizeRacingLine(...)`.

   Выбранный вариант: вариант A, локальный pool на один refined build.

5. Обновить статистику и логи без потери observability.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`
   - `drone_city_nav/src/racing_line.cpp`
   - `drone_city_nav/src/planner_node_publish.cpp`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`

   Материализуемый результат:

   - `candidate_threads_launched` после оптимизации считает реально созданные
     pool threads; ожидаемо около `parallel_workers_used`, а не
     `chunks * workers`;
   - `candidate_parallel_batches` остаётся числом parallel batch executions;
   - `candidate_thread_launch_duration_ms` измеряет startup pool threads;
   - добавить или явно переименовать метрику ожидания batch completion, чтобы не
     путать её с чистым OS thread join overhead. Предпочтительный вариант:
     добавить `candidate_batch_wait_duration_ms` и публиковать как
     `batch_wait=...ms`; старое поле `candidate_thread_join_wait_duration_ms`
     оставить только если нужен backward-compatible JSON;
   - в ROS summary и JSON должны быть видны:

     ```text
     chunks
     parallel_batches
     workers
     threads
     batch_wall
     buffer_prepare
     thread_launch
     batch_wait
     ```

   - после следующего headless-прогона ожидаемые признаки:

     ```text
     racing_candidate_chunks ~= 80
     racing_parallel_workers_used ~= 16
     racing_candidate_threads_launched ~= 16
     racing_candidate_thread_launch_duration_ms << 64.5ms
     ```

6. Обновить unit tests для deterministic behavior.

   Файл: `drone_city_nav/tests/racing_line_test.cpp`.

   Anchors:

   - `DefaultParallelCandidateEvaluationMatchesSingleWorkerResult`
   - добавить новый тест, например
     `ParallelCandidateEvaluationReusesWorkersAcrossChunks`

   Материализуемый результат:

   - сохранить проверку, что single-worker и parallel samples совпадают:

     ```cpp
     EXPECT_DOUBLE_EQ(sequential.samples[i].point.x, parallel.samples[i].point.x);
     EXPECT_DOUBLE_EQ(sequential.samples[i].point.y, parallel.samples[i].point.y);
     EXPECT_DOUBLE_EQ(sequential.samples[i].racing_offset_m,
                      parallel.samples[i].racing_offset_m);
     ```

   - добавить проверку reuse:

     ```cpp
     EXPECT_GT(parallel.stats.candidate_chunks, 1U);
     EXPECT_EQ(parallel.stats.candidate_parallel_batches,
               parallel.stats.candidate_chunks);
     EXPECT_LE(parallel.stats.candidate_threads_launched,
               parallel.stats.parallel_workers_used);
     ```

   - для serial config (`parallel_workers = 1`) проверить:

     ```cpp
     EXPECT_FALSE(result.stats.parallel_candidate_evaluation_used);
     EXPECT_EQ(result.stats.candidate_threads_launched, 0U);
     EXPECT_EQ(result.stats.candidate_parallel_batches, 0U);
     ```

7. Обновить serialization/blackbox tests.

   Файлы:

   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`

   Материализуемый результат:

   - новые или переименованные racing stats записываются в JSON;
   - parser восстанавливает значения;
   - blackbox JSON содержит эти поля;
   - тесты покрывают happy-path serialization и отсутствие regressions для
     существующих racing metrics.

8. Не менять алгоритм racing-line scoring.

   Файл: `drone_city_nav/src/racing_line.cpp`.

   Материализуемый результат:

   - не менять weights, candidate generation, DP windows, lower-bound shadow,
     speed profile formula, collision checks;
   - в diff не должно быть изменений в `scoreForCandidate(...)`,
     `candidateTasksForStep(...)`, `mergeCandidateStats(...)` кроме вызовов,
     необходимых для workspace/pool wiring.

# Verification plan

Минимальные проверки после реализации:

1. Форматирование изменённых C++ файлов:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Сборка пакета:

   ```bash
   ./scripts/dev_shell.sh make build
   ```

3. Scoped C++ tests:

   ```bash
   ./scripts/dev_shell.sh bash -lc 'source install/setup.bash && colcon --log-base log test --build-base build --install-base install --packages-select drone_city_nav --ctest-args -R "(racing_line_test|trajectory_diagnostics_io_test|offboard_blackbox_test)" --output-on-failure'
   ```

4. Проверка test results:

   ```bash
   ./scripts/dev_shell.sh bash -lc 'colcon --log-base log test-result --test-result-base build --all'
   ```

5. Полный quality gate перед commit:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

6. После автотестов желательно выполнить headless smoke для performance signal:

   ```bash
   ./scripts/sim_headless.sh
   ```

   Если smoke слишком дорогой для текущего раунда, явно зафиксировать skipped
   check и причину. Автотесты выше обязательны.

# Testing strategy

## 1. Без рефакторинга / узкая проверка

- `racing_line_test`
  - identical output для single-worker и parallel;
  - serial path не создаёт pool threads;
  - parallel path создаёт threads один раз и переиспользует их между chunks.
- `trajectory_diagnostics_io_test`
  - JSON write/read новых pool metrics.
- `offboard_blackbox_test`
  - blackbox JSON содержит pool/batch metrics.

## 2. Лёгкий scope

- `make build`
- scoped `colcon test` по трём затронутым тестам;
- `colcon test-result`.

## 3. Тяжёлый / интеграционный scope

- `make quality` как обязательный pre-commit gate.
- `./scripts/sim_headless.sh` как performance/flight smoke:
  - проверить `racing_candidate_threads_launched`;
  - проверить `racing_candidate_thread_launch_duration_ms`;
  - проверить, что trajectory остаётся valid/traversable;
  - проверить `MISSION_RESULT success=true`.

# Risks and tradeoffs

- Data race в `results`: каждый task должен писать только в свой
  `results[task_index]`. Проверяется review и unit test на identical output.
- Data race в buffers/cache: у каждого worker должен быть свой
  `CandidateWorkBuffer`, включая `candidate_segment_cache` и
  `full_path_segment_cache`.
- Lifetime bug: callback в `pool.run(...)` захватывает stack references
  (`tasks`, `results`, `base_offsets`, `best_points`). `run(...)` обязан ждать
  завершения всех worker-ов перед возвратом.
- Deadlock: pool должен корректно обрабатывать пустые tasks, `active_workers=1`,
  `active_workers < pool_size`, stop in destructor.
- Недиагностируемое изменение поведения: winner selection должен остаться
  последовательным и проходить по `candidates` в прежнем порядке.
- Метрики могут изменить семантику: особенно `join_wait`. Нужно явно разделить
  OS thread startup/shutdown и batch completion wait, чтобы будущие логи не
  вводили в заблуждение.
- Производительность: thread pool уберёт примерно `50-75 ms` из текущих
  `~642 ms racing_line`, но не решит основную стоимость candidate scoring
  (`racing_candidate_score_duration_ms` и `racing_candidate_speed_profile_duration_ms`).

# Open questions

- Блокирующих вопросов нет.
- Неблокирующее решение на этапе реализации: оставить старое JSON-поле
  `racing_candidate_thread_join_wait_duration_ms` как backward-compatible alias
  или заменить его новым `racing_candidate_batch_wait_duration_ms`. Предпочтение:
  добавить новое поле и сохранить старое только если это не усложнит тесты.
