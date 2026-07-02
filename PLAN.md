# Context

Планируем оптимизацию времени построения refined racing trajectory через `Top-N full scoring`.
Цель: не выполнять дорогой full score почти для всех racing-line candidates, а сначала
отсортировать кандидатов по дешёвой/local оценке и запускать full score только для лучших `N`.

Основание из последнего рана:

- `local_candidate_evaluations = 12160`
- `local_candidate_full_score_fallbacks = 10533`
- fallback rate: примерно `86.6%`
- `racing_full_candidate_score_duration_ms = 2619.4 ms` aggregate
- `trajectory_racing_line_duration_ms = 635.6 ms` wall time
- `trajectory_total_duration_ms = 787.8 ms`

Ожидаемый первый безопасный режим: `N=128` по умолчанию, параметр
`racing_line_top_n_full_score_candidates`. Победитель по-прежнему выбирается только по full score.
Cheap/local score используется только как preselection.

# Investigation context

`INVESTIGATION.md` отсутствует. Текущий `PLAN.md` отсутствовал до этого раунда, поэтому план
создаётся с нуля.

Локально изучены:

- `README.md` и `CONTRIBUTING.md`: подтверждён только container workflow.
- `CPP_BEST_PRACTICES.md`: C++20, минимальные изменения, явные контракты, тесты и диагностика.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`:
  Notion policy `optional`, prompt не содержит Notion task, чтение Notion не требуется.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`:
  prompt не содержит GitLab/MR, GitLab read не требуется.
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
  и `project_profiles/cpp.md`: применимы к этому ROS 2/C++ workspace.

# Detected stack/profiles

Стек:

- ROS 2 workspace.
- Основной пакет: `drone_city_nav`.
- C++20, ament CMake, `colcon`.
- Top-level `Makefile` оборачивает `colcon`/`ctest`.
- Фронтенда, БД и миграций в затрагиваемой области нет.

Применённые verification profiles:

- `generic.md`: обязателен для любого workspace.
- `cpp.md`: обязателен, потому что есть `CMakeLists.txt`, `Makefile`, `.cpp/.hpp`.

Rust profile не применялся: в затрагиваемом workspace задача относится к C++/ROS 2 коду, а
`Cargo.toml`/`.rs` для целевой области не используются.

# Repo-approved commands found

Команды из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/dev_shell.sh`
- внутри container shell: `make build`
- внутри container shell: `make test`
- внутри container shell: `make quality`
- внутри container shell: `make format`
- scoped CTest после build: `ctest --test-dir build/drone_city_nav --output-on-failure`

Для реализации этого плана перед коммитом использовать:

1. `./scripts/dev_shell.sh make format`
2. `./scripts/dev_shell.sh make quality`

Для быстрых scoped проверок во время разработки можно использовать container-команды:

- `./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R 'racing_line_test|planner_node_config_test|trajectory_diagnostics_io_test|offboard_blackbox_test'`

Если build dir отсутствует или устарел, сначала выполнить repo-approved build через:

- `./scripts/dev_shell.sh make build`

# Affected components

Основные production files:

- `drone_city_nav/include/drone_city_nav/racing_line.hpp:16`
  - `RacingLineConfig`: добавить параметр `top_n_full_score_candidates{128U}`.
  - `RacingLineStats`: добавить диагностику Top-N.
- `drone_city_nav/src/racing_line.cpp:66`
  - `EvaluatedCandidate`: сейчас хранит local/full score результат одного candidate.
- `drone_city_nav/src/racing_line.cpp:618`
  - `evaluateLocalOffsetPath(...)`: текущая cheap/local оценка.
- `drone_city_nav/src/racing_line.cpp:1099`
  - `updateBestCandidate(...)`: full path evaluation + full objective score.
- `drone_city_nav/src/racing_line.cpp:1143`
  - `evaluateCandidateSnapshot(...)`: сейчас может выполнять full score сразу во время
    candidate evaluation.
- `drone_city_nav/src/racing_line.cpp:1485`
  - `evaluateCandidateBatch(...)`: параллельная evaluation batch.
- `drone_city_nav/src/racing_line.cpp:1730`
  - optimizer iteration loop, где выбирается `iteration_winner`.

Конфиг:

- `drone_city_nav/src/planner_node_config.cpp:185`
  - загрузка `racing_line_*` параметров.
- `drone_city_nav/config/urban_mvp.yaml:122`
  - YAML defaults для racing-line параметров.

Диагностика и логи:

- `drone_city_nav/src/planner_node_publish.cpp:183`
  - log summary для trajectory/racing_line stats.
- `drone_city_nav/src/trajectory_diagnostics_io.cpp:209`
  - JSON summary writer/parser.
- `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp:21`
  - populated stats и round-trip checks.
- `drone_city_nav/tests/offboard_blackbox_test.cpp`
  - blackbox JSON fields.

Тесты racing line:

- `drone_city_nav/tests/racing_line_test.cpp:171`
  - deterministic test.
- `drone_city_nav/tests/racing_line_test.cpp:198`
  - parallel vs single worker equivalence.
- `drone_city_nav/tests/racing_line_test.cpp:246`
  - local prefilter/full objective scoring.

# Implementation steps

1. Добавить параметр Top-N в конфиг racing line.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp:16`
   - `drone_city_nav/src/planner_node_config.cpp:185`
   - `drone_city_nav/config/urban_mvp.yaml:122`
   - `drone_city_nav/tests/planner_node_config_test.cpp`

   Материализуемый результат:

   - `RacingLineConfig::top_n_full_score_candidates{128U}`.
   - ROS параметр `racing_line_top_n_full_score_candidates`, clamp, например `[0, 100000]`.
   - `0` трактовать как disabled/full behavior для отладки, но default должен быть `128`.
   - YAML default тоже `128`, чтобы не было рассинхрона config/default.
   - Тест проверяет, что YAML/default config содержит и правильно читает новый параметр.

2. Разделить candidate evaluation на preview phase и full-score phase.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:66`
   - `drone_city_nav/src/racing_line.cpp:1143`
   - `drone_city_nav/src/racing_line.cpp:1485`
   - `drone_city_nav/src/racing_line.cpp:1730`

   Материализуемый результат:

   - Добавить internal struct, например:

     ```cpp
     struct CandidatePreview {
       std::size_t order{};
       std::size_t local_rank{};
       EvaluatedCandidate candidate{};
       double cheap_score{std::numeric_limits<double>::infinity()};
       bool locally_valid{false};
       bool force_full_score{false};
     };
     ```

   - `evaluateCandidateBatch(...)` сначала строит candidates с local/cheap score без
     обязательного full score для каждого candidate.
   - Full score не должен запускаться внутри `evaluateCandidateSnapshot(...)` для всех
     кандидатов по умолчанию.
   - Для кандидатов, где local score невозможен (`requires_full_score=true`), пометить
     `force_full_score=true`, но не превращать это автоматически в full score для всех.

3. Реализовать stable Top-N selection.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:1730`

   Материализуемый результат:

   - После preview phase собрать кандидатов, пропустить noop/invalid, стабильно отсортировать:

     ```text
     force_full_score first
     locally_valid first
     lower cheap_score first
     original order as tie-breaker
     ```

   - Выбрать top `N = config.top_n_full_score_candidates`.
   - Если `N == 0`, оставить прежнее поведение: full score для всех кандидатов, чтобы иметь
     controlled regression/baseline mode.
   - Если кандидатов меньше `N`, full score для всех кандидатов.
   - Финальный winner iteration выбирать только среди full-scored candidates по full score.

4. Гарантировать safety inclusions, чтобы снизить риск от грубой оценки.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:1730`

   Материализуемый результат:

   - Помимо обычного top-N всегда включать:
     - лучший candidate по cheap score;
     - лучший locally valid candidate по estimated local time;
     - лучший candidate по curvature/shape cheap component, если это выделено отдельно;
     - candidate с `force_full_score=true` в пределах разумного лимита или отдельной квоты;
     - no-op/current baseline отдельно учитывать как baseline, но не считать как candidate,
       который может победить без full objective.
   - Если для первого этапа слишком дорого выделять отдельные cheap components, минимально
     включить `force_full_score` и best cheap score, а компоненты добавить в diagnostics позже
     в том же PR.

5. Добавить full-score execution только для selected candidates.

   Файлы:

   - `drone_city_nav/src/racing_line.cpp:1099`
   - `drone_city_nav/src/racing_line.cpp:1143`
   - `drone_city_nav/src/racing_line.cpp:1485`

   Материализуемый результат:

   - Вынести full scoring в helper, который принимает preview/candidate и делает:
     - `pointsFromOffsets(...)` при необходимости;
     - `evaluatePathCached(...)`;
     - `scoreForCandidate(...)`;
     - обновление timing/cache stats.
   - Параллельность сохранить: full scoring selected candidates можно выполнять через тот же
     worker-count/chunk механизм, но итоговый выбор должен быть deterministic по `order`.
   - Не ломать существующие cache counters:
     `candidate_segment_cache_*`, `full_path_segment_cache_*`.

6. Добавить Top-N stats и runtime diagnostics.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/racing_line.hpp:48`
   - `drone_city_nav/src/racing_line.cpp:1557`
   - `drone_city_nav/src/planner_node_publish.cpp:183`
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp:209`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp:21`
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`

   Материализуемый результат:

   Добавить поля:

   - `top_n_full_score_candidates`
   - `top_n_full_score_selected`
   - `top_n_full_score_skipped`
   - `top_n_full_score_forced`
   - `top_n_best_full_score_local_rank`
   - `top_n_full_score_reduction_ratio`
   - `top_n_preview_sort_duration_ms`
   - `top_n_full_score_selection_duration_ms`

   Эти поля должны попадать:

   - в ROS log summary `planning_clearance final trajectory`;
   - в final trajectory summary JSON;
   - в blackbox JSONL.

7. Обновить тесты racing line на поведение Top-N.

   Файлы:

   - `drone_city_nav/tests/racing_line_test.cpp:171`
   - `drone_city_nav/tests/racing_line_test.cpp:198`
   - `drone_city_nav/tests/racing_line_test.cpp:246`

   Материализуемый результат:

   - Тест `TopNFullScoringReducesFullScoreFallbacks`:
     - сравнить `config.top_n_full_score_candidates = 0` и `128`/малое значение на одном
       corridor fixture;
     - проверить, что при Top-N `local_candidate_full_score_fallbacks` меньше либо равно
       baseline;
     - проверить `top_n_full_score_skipped > 0` на fixture, где candidates достаточно много.
   - Тест deterministic:
     - два запуска с одинаковым `top_n_full_score_candidates` дают одинаковые samples/cost.
   - Тест parallel equivalence:
     - single worker и parallel worker дают одинаковый результат при включённом Top-N.
   - Negative/edge:
     - `top_n_full_score_candidates=0` сохраняет full-score-all режим.
     - `top_n_full_score_candidates` больше числа candidates ведёт себя как full-score-all.

8. Обновить config/diagnostics tests.

   Файлы:

   - `drone_city_nav/tests/planner_node_config_test.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`

   Материализуемый результат:

   - Проверить чтение YAML/default `racing_line_top_n_full_score_candidates=128`.
   - Проверить JSON fragment contains all new Top-N keys.
   - Проверить round-trip parse/write для новых fields.
   - Проверить blackbox содержит новые fields.

9. Обновить runtime log summary для headless debugging.

   Файлы:

   - `drone_city_nav/src/planner_node_publish.cpp:183`

   Материализуемый результат:

   - В строке `planning_clearance final trajectory` добавить компактный блок:

     ```text
     top_n[limit=128 selected=... skipped=... forced=... best_rank=... reduction=...]
     ```

   - По логам должно быть понятно:
     - включён ли Top-N;
     - сколько candidates не получили full score;
     - не оказался ли winner на границе `N`.

10. Не менять стратегию построения траектории за пределами candidate preselection.

    Файлы:

    - `drone_city_nav/src/racing_line.cpp`

    Материализуемый результат:

    - DP seed, active windows, corridor input, turn smoothing, speed profile finalization не
      менять семантически.
    - Full objective остаётся единственным критерием победителя.
    - Изменение результата допустимо только как следствие того, что candidate не попал в Top-N;
      это должно быть явно видно по diagnostics.

# Verification plan

Обязательные команды после реализации:

1. `./scripts/dev_shell.sh make format`
2. Scoped tests:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R 'racing_line_test|planner_node_config_test|trajectory_diagnostics_io_test|offboard_blackbox_test'
   ```

3. Full quality before commit:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

Симуляцию автоматически не запускать без явной команды пользователя.

# Testing strategy

Категория 1: без рефакторинга / низкий риск

- `trajectory_diagnostics_io_test`: JSON fields/round-trip для новых Top-N stats.
- `offboard_blackbox_test`: новые поля в blackbox JSON.
- `planner_node_config_test`: default/YAML parameter read.

Категория 2: лёгкий алгоритмический риск

- `racing_line_test`: deterministic Top-N result.
- `racing_line_test`: single-worker vs parallel equivalence.
- `racing_line_test`: `N=0` отключает Top-N и сохраняет прежний full-score-all режим.
- `racing_line_test`: `N` больше числа candidates не режет full scoring.
- `racing_line_test`: малый `N` снижает `local_candidate_full_score_fallbacks` и увеличивает
  `top_n_full_score_skipped`.

Категория 3: тяжёлый/интеграционный риск

- `make quality`: build + весь CTest + format dry-run + scoped clang-tidy/cppcheck через
  repo-approved script.
- Headless simulation не входит в автоматическую проверку этого раунда без явной команды, но после
  реализации полезно вручную/по команде пользователя проверить:
  - `trajectory_racing_line_duration_ms`;
  - `top_n_best_full_score_local_rank`;
  - отсутствие деградации формы траектории в RViz/final trajectory dump;
  - отсутствие роста `prohibited_replans`.

# Risks and tradeoffs

- Поведение racing line может измениться: хороший candidate может иметь плохой cheap/local rank и
  не попасть в Top-N.
- Чем меньше `N`, тем выше риск деградации траектории; поэтому default `128`, а не `64`.
- Full score aggregate time должен упасть, но wall-time выигрыш зависит от parallel worker
  scheduling и от того, насколько speed profile остаётся bottleneck.
- Если `force_full_score` candidates окажется слишком много, Top-N может почти не дать эффекта.
  Это надо увидеть в `top_n_full_score_forced`.
- Если selected winner часто имеет local rank около `N`, это сигнал, что `N` слишком мал или
  cheap score плохо коррелирует с full objective.
- Изменения в diagnostics/blackbox расширяют JSON schema; downstream анализаторы должны
  терпимо относиться к новым fields.
- Производительность может улучшиться, но код станет сложнее: появятся две фазы evaluation и
  дополнительные stats.

# Open questions

- Нужно ли оставлять `N=0` как явный debug режим? В плане да, потому что он нужен для тестового
  baseline и сравнения качества.
- Нужно ли сразу делать adaptive `N`? Нет. Первый шаг: фиксированный параметр `128`, затем по
  логам решать, можно ли снижать до `64` или делать adaptive policy.
- Нужно ли считать Top-N глобально на все optimizer iterations? Нет. Безопаснее выбирать Top-N
  внутри каждого candidate batch/iteration, чтобы каждая итерация сохраняла локальную конкуренцию
  кандидатов.
