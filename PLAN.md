# Context

Планируем только этап `Passage Matching And Validation` для фичи `3D passage traversal`.

Цель этапа: по уже построенной 2D/3D executable trajectory понять, пересекает ли она footprint известной passage structure, и если пересекает, проходит ли соответствующий участок через один из allowed opening volume. В этом этапе поведение дрона и геометрия траектории не меняются: результат работает в shadow/diagnostic mode и нужен для headless-отладки следующих этапов.

Текущая база уже есть:

- `drone_city_nav/include/drone_city_nav/known_passage_map.hpp:13` описывает `PassageOpening`, включая `center`, `normal_xy`, `width_m`, `height_m`, `depth_m`, `min_z_m`, `max_z_m`.
- `drone_city_nav/include/drone_city_nav/known_passage_map.hpp:27` описывает axis-aligned `PassageStructure` через `center`, `size_x_m`, `size_y_m`, `z_min_m`, `z_max_m`.
- `drone_city_nav/src/known_passage_map.cpp:158` уже грузит line-based `known_passages.passages3d`.
- `drone_city_nav/include/drone_city_nav/trajectory.hpp:54` уже хранит `TrajectoryPointSample::z_m`.
- `drone_city_nav/src/trajectory_planner.cpp:70` сейчас присваивает всем samples постоянную высоту `default_altitude_m`.
- `drone_city_nav/src/planner_node_inputs.cpp:205` уже загружает `KnownPassageMap` в `PlannerNode::known_passages_`.
- `drone_city_nav/src/planner_node_trajectory_publication.cpp:212` является удобной точкой интеграции, потому что через неё проходят и обычные, и refined trajectory перед публикацией.

# Investigation context

`INVESTIGATION.md` в workspace сейчас отсутствует, поэтому входными данными были только `inbox.txt`, локальные repo instructions и текущий код.

Дополнительно проверены:

- `AGENTS.md`
- `CPP_BEST_PRACTICES.md`
- `README.md`
- `CONTRIBUTING.md`
- `drone_city_nav/CMakeLists.txt`
- текущие passage map, trajectory planner, planner node publication, diagnostics JSON/parser и tests.

# Detected stack/profiles

Стек: ROS 2 workspace, основной пакет `drone_city_nav`, C++20, ament CMake/colcon, Gazebo/PX4 SITL.

Применяемые профили:

- `generic.md`: обязателен для любого workspace; использован для discovery repo-approved commands и scoped verification strategy.
- `cpp.md`: применим, потому что repo содержит `CMakeLists.txt`, `.cpp`, `.hpp`, gtest targets и C++ build workflow.

Notion/GitLab протоколы прочитаны. В user prompt нет Notion task id и нет GitLab/MR запроса, поэтому внешние Notion/GitLab чтения не нужны. Remote HTTP/SSH не используются.

# Repo-approved commands found

Только container workflow:

- `./scripts/build.sh` - approved build через dev container/colcon.
- `./scripts/test.sh` - approved unit test wrapper.
- `./scripts/dev_shell.sh`, затем внутри container:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
  - `ctest --test-dir build/drone_city_nav --output-on-failure`

Для C++ изменений перед commit нужно форматировать изменённые C++ файлы через `make format` и запускать `make quality`. Для этого planning-only коммита C++ файлы не меняются.

# Affected components

- Новый core-модуль validation:
  - `drone_city_nav/include/drone_city_nav/known_passage_validation.hpp`
  - `drone_city_nav/src/known_passage_validation.cpp`
  - `drone_city_nav/tests/known_passage_validation_test.cpp`

- Existing passage data model:
  - `drone_city_nav/include/drone_city_nav/known_passage_map.hpp:13`
  - `drone_city_nav/src/known_passage_map.cpp:105`

- Existing trajectory samples:
  - `drone_city_nav/include/drone_city_nav/trajectory.hpp:54`
  - `drone_city_nav/src/trajectory.cpp:256`

- Planner stats/diagnostics:
  - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:43`
  - `drone_city_nav/src/trajectory_diagnostics_io_json_summary.cpp:53`
  - `drone_city_nav/src/trajectory_diagnostics_io_parser.cpp:7`
  - `drone_city_nav/tests/trajectory_diagnostics_io_test_helpers.hpp`
  - `drone_city_nav/tests/trajectory_diagnostics_io_json_fields_test.cpp`
  - `drone_city_nav/tests/trajectory_diagnostics_io_roundtrip_test.cpp`

- Planner node integration/config/logs:
  - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:58`
  - `drone_city_nav/src/planner_node_config.cpp:49`
  - `drone_city_nav/src/planner_node.hpp:168`
  - `drone_city_nav/src/planner_node.hpp:300`
  - `drone_city_nav/src/planner_node_trajectory_publication.cpp:212`
  - `drone_city_nav/src/planner_node_debug_publication.cpp:149`
  - `drone_city_nav/config/urban_mvp.yaml:79`

- Docs:
  - `docs/configuration.md:47`
  - `docs/navigation_pipeline.md:14`
  - `docs/diagnostics.md`
  - `docs/rviz.md`

# Implementation steps

1. Добавить pure core API для passage validation.

   Файлы:

   - создать `drone_city_nav/include/drone_city_nav/known_passage_validation.hpp`
   - создать `drone_city_nav/src/known_passage_validation.cpp`
   - обновить `drone_city_nav/CMakeLists.txt:42`, добавив новый `.cpp` в `drone_city_nav_core`

   Материализуемый результат:

   - появится отдельная функция без ROS-зависимостей:

   ```cpp
   enum class KnownPassageValidationReason {
     kDisabled,
     kNoMap,
     kInvalidTrajectory,
     kNoStructureIntersection,
     kMatchedOpening,
     kStructureWithoutOpening,
     kOpeningVolumeMiss,
   };

   struct KnownPassageValidationConfig {
     bool enabled{true};
     double min_opening_overlap_m{0.5};
     double clearance_margin_m{0.0};
     std::size_t max_diagnostics{8U};
   };

   struct KnownPassageValidationSpan {
     std::string structure_id;
     std::string opening_id;
     double entry_s_m{0.0};
     double exit_s_m{0.0};
     double overlap_m{0.0};
     double clearance_m{0.0};
     KnownPassageValidationReason reason{
         KnownPassageValidationReason::kNoStructureIntersection};
     bool valid{false};
   };

   struct KnownPassageValidationSummary {
     bool enabled{false};
     bool valid{true};
     std::size_t structures_checked{0U};
     std::size_t structures_intersected{0U};
     std::size_t opening_matches{0U};
     std::size_t violations{0U};
     KnownPassageValidationReason worst_reason{
         KnownPassageValidationReason::kNoStructureIntersection};
     std::vector<KnownPassageValidationSpan> diagnostics;
   };

   [[nodiscard]] KnownPassageValidationSummary
   validateKnownPassageTraversal(std::span<const TrajectoryPointSample> samples,
                                 const KnownPassageMap* map,
                                 const KnownPassageValidationConfig& config);
   ```

   Строковый helper `knownPassageValidationReasonName(...)` должен возвращать stable snake_case значения для logs/JSON.

2. Реализовать `trajectory intersects passage structure footprint`.

   Файл:

   - `drone_city_nav/src/known_passage_validation.cpp`

   Anchor/данные:

   - `PassageStructure` footprint сейчас axis-aligned rectangle: `center +/- size_x/2`, `center +/- size_y/2` (`known_passage_map.cpp:55` уже использует такую семантику).
   - `TrajectoryPointSample` даёт сегменты `samples[i] -> samples[i + 1]` со stationing `s_m`.

   Алгоритм:

   - для каждого sample segment делать 2D line-rectangle clipping по structure footprint;
   - использовать Liang-Barsky или эквивалентный half-space clipping, чтобы не зависеть от плотности sample step;
   - вернуть `entry_s_m`, `exit_s_m`, индекс первого/последнего сегмента и clipped endpoints.

   Pseudocode:

   ```cpp
   std::optional<ClipInterval> clipSegmentToStructureFootprint(
       TrajectoryPointSample a,
       TrajectoryPointSample b,
       PassageStructure structure) {
     Rect r = footprintRect(structure);
     // clip p(t)=a.xy + t*(b.xy-a.xy), t in [0,1]
     // if overlap exists:
     return ClipInterval{
       .t0 = t_enter,
       .t1 = t_exit,
       .s0 = lerp(a.s_m, b.s_m, t_enter),
       .s1 = lerp(a.s_m, b.s_m, t_exit),
       .p0 = lerp3(a, b, t_enter),
       .p1 = lerp3(a, b, t_exit),
     };
   }
   ```

   Материализуемый результат:

   - validator находит все spans, где trajectory физически входит в XY footprint passage structure.
   - если spans нет, summary valid=true, reason=`no_structure_intersection`, violations=0.

3. Реализовать `trajectory crosses opening volume`.

   Файл:

   - `drone_city_nav/src/known_passage_validation.cpp`

   Anchor/данные:

   - opening orientation задаётся `PassageOpening::normal_xy` (`known_passage_map.hpp:17`), нормализуется parser’ом (`known_passage_map.cpp:84`).
   - opening dimensions: `width_m`, `depth_m`, `min_z_m`, `max_z_m`.

   Алгоритм:

   - для каждого clipped structure span и каждого opening этой structure:
     - построить локальную систему:
       - `u = dot(p.xy - opening.center.xy, opening.normal_xy)` - направление прохода через opening;
       - `v = dot(p.xy - opening.center.xy, lateral)` - ширина opening;
       - `z = p.z_m`;
     - clipped segment должен пересекать 3D box:
       - `u in [-depth_m/2, depth_m/2]`
       - `v in [-width_m/2, width_m/2]`
       - `z in [min_z_m, max_z_m]`
     - считать overlap stationing `opening_overlap_m`;
     - match засчитывать только если `opening_overlap_m >= min_opening_overlap_m`.

   Pseudocode:

   ```cpp
   std::optional<OpeningMatch> clipSegmentToOpeningVolume(
       ClipInterval structure_span,
       PassageOpening opening,
       KnownPassageValidationConfig config) {
     LocalSegment s = toOpeningFrame(structure_span, opening);
     Box3 box{
       .u = {-opening.depth_m / 2.0, opening.depth_m / 2.0},
       .v = {-opening.width_m / 2.0, opening.width_m / 2.0},
       .z = {opening.min_z_m, opening.max_z_m},
     };
     Clip3D clipped = clipLineToBox(s, box);
     if (!clipped.valid || clipped.overlap_s_m < config.min_opening_overlap_m) {
       return std::nullopt;
     }
     return OpeningMatch{..., .clearance_m = min(lateral_clearance, vertical_clearance)};
   }
   ```

   Материализуемый результат:

   - validator умеет отличать trajectory, которая пересекла footprint через разрешённый volume, от trajectory, которая прошла сквозь wall/body structure.
   - для нескольких openings выбирается лучший match по максимальному `overlap_m`, затем по максимальному `clearance_m`.

4. Добавить no-over-building rule как shadow validation, без reject trajectory.

   Файлы:

   - `drone_city_nav/src/known_passage_validation.cpp`
   - `drone_city_nav/src/planner_node_trajectory_publication.cpp:212`

   Правило:

   - если trajectory не пересекает structure footprint, valid=true;
   - если пересекает footprint и есть opening match для соответствующего span, valid=true;
   - если пересекает footprint, но нет opening match, это `violation` с reason:
     - `structure_without_opening`, если structure openings пустой;
     - `opening_volume_miss`, если openings есть, но trajectory не попала в volume / не выдержала `min_opening_overlap_m` / clearance margin.

   Важно: в этом этапе `publishTrajectoryResult(...)` не должен отменять публикацию path из-за violation. Он только пишет summary/logs/JSON.

   Материализуемый результат:

   - фича даёт диагностику “эта траектория прошла через известное passage-здание корректно/некорректно”, но не влияет на flight behavior.

5. Добавить config для shadow validator.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp:58`
   - `drone_city_nav/src/planner_node_config.cpp:49`
   - `drone_city_nav/src/planner_node.hpp:320`
   - `drone_city_nav/config/urban_mvp.yaml:79`
   - `drone_city_nav/tests/planner_node_config_test.cpp`
   - `drone_city_nav/tests/px4_offboard_config_test.cpp`

   Конкретные параметры:

   ```yaml
   known_passage_validation_enabled: true
   known_passage_validation_min_opening_overlap_m: 0.5
   known_passage_validation_clearance_margin_m: 0.0
   known_passage_validation_max_diagnostics: 8
   ```

   Материализуемый результат:

   - `PlannerNodeConfig` содержит `KnownPassageValidationConfig`.
   - параметры clamp’ятся:
     - overlap: `[0.0, 1000.0]`
     - clearance margin: `[0.0, 1000.0]`
     - max diagnostics: `[0, 100]`
   - tests проверяют defaults, custom values и clamping.

6. Расширить `TrajectoryPlannerStats` и diagnostics contract.

   Файлы:

   - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:43`
   - `drone_city_nav/src/trajectory_diagnostics_io_json_summary.cpp:53`
   - `drone_city_nav/src/trajectory_diagnostics_io_parser.cpp:7`
   - `drone_city_nav/tests/trajectory_diagnostics_io_test_helpers.hpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_json_fields_test.cpp`
   - `drone_city_nav/tests/trajectory_diagnostics_io_roundtrip_test.cpp`

   Добавить в stats:

   ```cpp
   KnownPassageValidationSummary known_passage_validation{};
   ```

   JSON fields:

   - `known_passage_validation_enabled`
   - `known_passage_validation_valid`
   - `known_passage_structures_checked`
   - `known_passage_structures_intersected`
   - `known_passage_opening_matches`
   - `known_passage_violations`
   - `known_passage_validation_reason`
   - `known_passage_diag_count`
   - для первых `max_diagnostics` spans:
     - `known_passage_diag0_structure_id`
     - `known_passage_diag0_opening_id`
     - `known_passage_diag0_entry_s_m`
     - `known_passage_diag0_exit_s_m`
     - `known_passage_diag0_overlap_m`
     - `known_passage_diag0_clearance_m`
     - `known_passage_diag0_reason`
     - `known_passage_diag0_valid`

   Материализуемый результат:

   - planner diagnostics JSON и parser roundtrip сохраняют summary + capped spans.
   - top-level JSON uniqueness test остаётся зелёным.
   - headless logs/blackbox analysis могут увидеть exact reason без RViz.

7. Интегрировать validation в planner publication path.

   Файл:

   - `drone_city_nav/src/planner_node_trajectory_publication.cpp:212`

   Конкретное изменение:

   - после runtime traversability check и до большого `RCLCPP_INFO` сделать локальную копию stats:

   ```cpp
   TrajectoryPlannerStats stats = trajectory_result.stats;
   stats.known_passage_validation = validateKnownPassageTraversal(
       trajectory_result.samples,
       known_passages_ ? &*known_passages_ : nullptr,
       known_passage_validation_config_);
   ```

   - в logs и `publishTrajectoryPath(...)` использовать `stats`, а не `trajectory_result.stats`.
   - `writeCorridorSamplesDump(...)` и candidate dumps можно оставить на исходном result: validation не меняет samples/candidates.

   Материализуемый результат:

   - validation выполняется на final samples, которые реально публикуются.
   - refined async trajectory тоже получает diagnostics, потому что `planner_node_refinement.cpp:229` вызывает тот же `publishTrajectoryResult(...)`.
   - trajectory rejection/hold path не меняются.

8. Добавить компактные RCLCPP diagnostics в final trajectory log.

   Файл:

   - `drone_city_nav/src/planner_node_trajectory_publication.cpp:318`

   Добавить отдельный блок в существующий final trajectory log:

   ```text
   known_passage_validation[enabled=true valid=false checked=2 intersected=1
   matches=0 violations=1 reason=opening_volume_miss
   first(structure=arch_01 opening=<none> s=[42.50,48.75]
   overlap=0.00 clearance=-1.25)]
   ```

   Материализуемый результат:

   - по обычному headless log сразу видно, был ли проход через passage и почему он невалиден.
   - список всех spans не раздувает основной log; полная детализация идёт в JSON diagnostics.

9. Добавить unit tests validator-а.

   Файлы:

   - `drone_city_nav/tests/known_passage_validation_test.cpp`
   - `drone_city_nav/CMakeLists.txt:308`

   Обязательные сценарии:

   - `NoMapIsValidWithNoMapReason`: enabled=true, map=nullptr.
   - `DisabledSkipsValidation`: enabled=false.
   - `NoStructureIntersectionIsValid`: samples обходят footprint.
   - `StructureWithoutOpeningReportsViolation`: footprint пересечён, openings пустой.
   - `OpeningVolumeMatchIsValid`: trajectory проходит через center, z внутри `[min_z,max_z]`.
   - `OpeningVolumeMissesByAltitude`: XY попадает, но `z_m` выше/ниже opening.
   - `OpeningVolumeMissesByLateralOffset`: z правильный, но `v` вне width.
   - `OpeningVolumeMissesByDepth`: XY пересекает footprint, но не пересекает depth slab opening.
   - `ChoosesBestOpeningByOverlapThenClearance`: два opening, выбирается лучший.
   - `HandlesSegmentStartingInsideFootprint`: entry_s корректен, если первый sample уже внутри structure.
   - `RejectsInvalidTrajectorySamples`: меньше 2 samples, non-finite z или non-monotonic s.
   - `CapsDiagnostics`: больше spans, чем `max_diagnostics`.

   Материализуемый результат:

   - геометрия validator-а покрыта happy-path, negative-path и edge cases без ROS/Gazebo.

10. Обновить diagnostics tests.

    Файлы:

    - `drone_city_nav/tests/trajectory_diagnostics_io_test_helpers.hpp`
    - `drone_city_nav/tests/trajectory_diagnostics_io_json_fields_test.cpp`
    - `drone_city_nav/tests/trajectory_diagnostics_io_roundtrip_test.cpp`

    Материализуемый результат:

    - `populatedStats()` заполняет `known_passage_validation`.
    - JSON contains tests проверяют новые fields.
    - roundtrip parser проверяет summary и хотя бы один `known_passage_diag0_*`.

11. Обновить docs.

    Файлы:

    - `docs/configuration.md:47`
    - `docs/navigation_pipeline.md:14`
    - `docs/diagnostics.md`
    - `docs/rviz.md`

    Материализуемый результат:

    - docs явно говорят, что known passages теперь не только annotations/markers, а ещё shadow validation layer.
    - docs фиксируют no-over-building rule как diagnostic-only на текущем этапе.
    - docs перечисляют новые config params и diagnostics fields.
    - docs не обещают vertical profile builder, trajectory repair или lidar policy в этом этапе.

# Verification plan

После реализации:

1. Форматировать изменённые C++ файлы:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Собрать workspace:

   ```bash
   ./scripts/build.sh
   ```

3. Запустить scoped C++ tests:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav --output-on-failure -R 'known_passage_validation|known_passage_map|planner_node_config|px4_offboard_config|trajectory_diagnostics_io'
   ```

4. Запустить script-level contracts:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

5. Запустить repo quality gate:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

Simulation run не является обязательной проверкой для этого этапа, потому что поведение trajectory/control не должно измениться. Если позже потребуется ручная интеграционная проверка, достаточно headless run и анализа `log/*trajectory*`/`/drone_city_nav/trajectory_diagnostics`, но это fallback, не замена автотестов.

# Testing strategy

1. Без рефакторинга / pure additions.

   - `known_passage_validation_test`: покрыть геометрию footprint/opening volume, no-over-building reason, clearance, capped diagnostics.
   - `known_passage_map_test`: оставить существующий parser coverage; при необходимости добавить только маленький тест на consistency fields, если validator потребует новую invariant.
   - `trajectory_diagnostics_io_*`: покрыть serialization/parser roundtrip новых validation fields.

2. Лёгкий рефакторинг / integration glue.

   - `planner_node_config_test`: defaults/custom/clamp для validation config.
   - `px4_offboard_config_test`: YAML contract содержит новые params.
   - Проверить, что `publishTrajectoryResult(...)` использует локальную `stats` копию с validation summary и не меняет `trajectory_result.samples`.

3. Тяжёлый / deferred, не делать в этом этапе.

   - Hard rejection trajectory при passage violation.
   - Local XY passage insertion.
   - Vertical profile builder и speed caps по `z(s)`.
   - Lidar/memory policy inside known passage structure.
   - RViz highlighting конкретного matched/violated trajectory span.
   - Rotated/polygon structure footprints.

# Risks and tradeoffs

- Текущая `PassageStructure` footprint axis-aligned. Это соответствует текущему формату, но не покрывает rotated/polygon buildings. Для Stage 3 это осознанный tradeoff: сначала валидируем существующий contract.
- Сегментный clipping важен: простая проверка sample points может пропустить короткое пересечение между samples.
- Пока `z_m` обычно constant (`trajectory_planner.cpp:70`), validation часто будет показывать altitude miss для openings выше/ниже cruise altitude. Это нормально: vertical profile builder будет следующим этапом.
- Shadow mode не защищает от “пролёта сквозь wall” в runtime, но исключает риск сломать текущие успешные 2D прогоны.
- Diagnostics могут раздуть JSON/logs. Поэтому нужен `known_passage_validation_max_diagnostics` и компактный summary в основном log.
- `height_m` и `[min_z_m, max_z_m]` сейчас оба есть в opening. Для validation использовать authoritative `[min_z_m, max_z_m]`; `height_m` оставить как annotation/visual dimension, не вводить новый parser reject без отдельного решения.

# Open questions

- Нужны ли rotated или polygon footprints в `.passages3d` v2? Для текущего этапа не блокирует, потому что существующий format уже axis-aligned.
- Должен ли будущий hard validation требовать проход именно по направлению `normal_xy`, или достаточно volume intersection? Для shadow Stage 3 выбираем volume intersection + overlap length, чтобы не ломать будущие U/side approaches до появления локальной вставки.
- Какой clearance margin станет hard threshold в будущем? Для Stage 3 default `0.0` и только диагностика.
