# План ускорения расчёта пути и гоночной траектории

## Context

Нужно спланировать единый пакет оптимизаций для самых тяжёлых этапов построения
пути/траектории. По последнему доступному GUI-рану из `log/ros_city_mvp.log`
узкие места такие:

- `grid / inflation`: `1659.3 ms`
- `PlanningCore clearance_field`: `878.5 ms`
- `racing_line`: `1226.5 ms`
- `A*`: `96.8 ms`, сейчас не главный bottleneck

Текущий pipeline уже использует final racing trajectory как runtime path:
`PlannerNode::publishPathFromPathCells()` строит trajectory через
`planRacingTrajectory()` и не публикует rough A* route при невалидной trajectory
([drone_city_nav/src/planner_node_publish.cpp:137](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/planner_node_publish.cpp:137),
[drone_city_nav/src/planner_node_publish.cpp:149](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/planner_node_publish.cpp:149)).

План должен включать вместе:

1. единый distance/clearance field и static precompute;
2. static/dynamic split для inflation/clearance;
3. замену глобального brute-force `racing_line` на более структурный/windowed
   optimizer;
4. async baseline -> refined trajectory;
5. достаточные автотесты и headless-friendly логи/дампы.

## Investigation context

`INVESTIGATION.md` отсутствует. `PLAN.md` до этого запуска отсутствовал, поэтому
создаётся новый план.

Локально изучены:

- repo workflow: `README.md`, `CONTRIBUTING.md`, `Makefile`,
  `CPP_BEST_PRACTICES.md`;
- grid/inflation: `OccupancyGrid2D::rebuildInflation()`
  ([drone_city_nav/src/occupancy_grid.cpp:233](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/occupancy_grid.cpp:233));
- static/dynamic builder: `PlanningGridBuilder::build()`
  ([drone_city_nav/src/planning_grid_builder.cpp:173](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/planning_grid_builder.cpp:173));
- clearance: `ClearanceField2D::build()` и `ClearanceFieldCache::getOrBuild()`
  ([drone_city_nav/src/clearance_field.cpp:73](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/clearance_field.cpp:73),
  [drone_city_nav/src/clearance_field.cpp:159](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/clearance_field.cpp:159));
- PlannerCore timings: `PlannerCore::computePath()`
  ([drone_city_nav/src/planner_core.cpp:537](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/planner_core.cpp:537));
- corridor reuse: `buildCorridor(CorridorInput, ...)`
  ([drone_city_nav/src/corridor.cpp:305](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/corridor.cpp:305));
- racing optimizer: `optimizeRacingLine()`
  ([drone_city_nav/src/racing_line.cpp:630](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/racing_line.cpp:630));
- trajectory pipeline: `planRacingTrajectory()`
  ([drone_city_nav/src/trajectory_planner.cpp:101](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/trajectory_planner.cpp:101));
- relevant tests: `planning_grid_builder_test.cpp`, `clearance_field_test.cpp`,
  `corridor_test.cpp`, `racing_line_test.cpp`, `trajectory_planner_test.cpp`.

Текущее состояние важно учитывать:

- `PlanningGridBuilder` уже имеет in-memory static cache
  ([drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp:79](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp:79)).
- Static/dynamic split уже частично есть: cached static grids merge with dynamic
  inflated grids
  ([drone_city_nav/src/planning_grid_builder.cpp:237](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/planning_grid_builder.cpp:237)).
- Но dynamic inflation всё ещё вызывает `rebuildInflation()` два раза на
  `dynamic_raw`, а `rebuildInflation()` сейчас проходит по occupied cells и
  размазывает окрестность.
- `ClearanceField2D::build()` сейчас отдельный bounded Dijkstra/priority queue,
  не общий distance transform.
- `racing_line` сейчас оценивает тысячи full-path candidate snapshots. Даже в
  parallel branch фактически есть только два рабочих буфера и один `std::async`
  на пару `-step/+step`
  ([drone_city_nav/src/racing_line.cpp:674](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/racing_line.cpp:674),
  [drone_city_nav/src/racing_line.cpp:717](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/racing_line.cpp:717)).

## Detected stack/profiles

- Основной стек: ROS 2 workspace, C++20, `ament_cmake`, `colcon`, пакет
  `drone_city_nav`.
- Rust manifests/source в workspace не найдены; Rust profile не применялся.
- Прочитаны обязательные профили оркестратора:
  - `generic.md`, потому что он обязателен для любого workspace;
  - `cpp.md`, потому что есть `Makefile`, `CMakeLists.txt`, `.cpp/.hpp`.
- Прочитаны протоколы Notion/GitLab. Они не потребовали CLI-доступа, потому что
  пользовательский prompt не содержит Notion task или GitLab MR.

## Repo-approved commands found

Команды из `README.md`, `CONTRIBUTING.md`, `Makefile`:

- build: `./scripts/build.sh`, внутри контейнера `make build`;
- tests: `./scripts/test.sh`, внутри контейнера `make test`;
- script tests: внутри контейнера `make test-scripts`;
- quality: внутри контейнера `make quality`;
- format changed C++: внутри контейнера `make format`;
- simulation: `./scripts/sim_headless.sh`, `./scripts/sim_gui.sh`;
- cleanup: `./scripts/stop_sim.sh`, dry-run: `./scripts/stop_sim.sh --dry-run`.

Для реализации использовать только container workflow. Для планирования код не
собирался и симуляция не запускалась.

## Affected components

- `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`
- `drone_city_nav/src/occupancy_grid.cpp`
- новые `drone_city_nav/include/drone_city_nav/distance_field.hpp`,
  `drone_city_nav/src/distance_field.cpp`
- `drone_city_nav/include/drone_city_nav/clearance_field.hpp`
- `drone_city_nav/src/clearance_field.cpp`
- `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
- `drone_city_nav/src/planning_grid_builder.cpp`
- `drone_city_nav/include/drone_city_nav/planner_core.hpp`
- `drone_city_nav/src/planner_core.cpp`
- `drone_city_nav/src/planner_node_inputs.cpp`
- `drone_city_nav/src/planner_node_publish.cpp`
- `drone_city_nav/include/drone_city_nav/corridor.hpp`
- `drone_city_nav/src/corridor.cpp`
- `drone_city_nav/include/drone_city_nav/racing_line.hpp`
- `drone_city_nav/src/racing_line.cpp`
- новые, если реализация разрастается: `racing_line_windows.hpp/.cpp`,
  `racing_line_dp.hpp/.cpp`, `trajectory_build_state.hpp/.cpp`
- `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`
- `drone_city_nav/src/trajectory_planner.cpp`
- diagnostics/log IO:
  `drone_city_nav/include/drone_city_nav/trajectory_diagnostics.hpp`,
  `drone_city_nav/src/trajectory_diagnostics_io.cpp`,
  `drone_city_nav/src/offboard_blackbox.cpp`
- config/docs/tests:
  `drone_city_nav/config/urban_mvp.yaml`, `drone_city_nav/CMakeLists.txt`,
  `drone_city_nav/tests/*`.

## Implementation steps

1. **Добавить единый `DistanceField2D` как source of truth для расстояний до препятствий.**

   Файлы:
   - добавить `drone_city_nav/include/drone_city_nav/distance_field.hpp`;
   - добавить `drone_city_nav/src/distance_field.cpp`;
   - подключить в `drone_city_nav/CMakeLists.txt`;
   - добавить `drone_city_nav/tests/distance_field_test.cpp`.

   Материализуемый результат:
   - новый тип `DistanceField2D` умеет строить поле расстояний от
     `CellState::kOccupied` или от `isProhibited()`;
   - API поддерживает `distanceAt(GridIndex)`, `contains(GridIndex)`,
     `bounds()`, `source()`, `maxDistanceM()`, `distancesM()`;
   - добавить `DistanceFieldBuildStats` с `source_cells`, `width`, `height`,
     `algorithm`, `duration_ms` для логов.

   Предлагаемый алгоритм: separable exact Euclidean Distance Transform
   Felzenszwalb-Huttenlocher, `O(width * height)`, с квадратами расстояний в
   клетках и переводом в метры. Для runtime можно хранить `double` distance in
   meters; для mask comparison использовать `distance <= radius + 0.5 *
   resolution`.

   Псевдокод контракта:

   ```cpp
   enum class DistanceFieldSource { kOccupied, kProhibited };

   struct DistanceFieldBuildRequest {
     const OccupancyGrid2D& grid;
     DistanceFieldSource source;
     double max_distance_m; // 0/inf means uncapped, finite cap keeps inf outside cap.
   };

   DistanceField2D DistanceField2D::build(const DistanceFieldBuildRequest& request);
   ```

   Автотесты:
   - happy-path: одна occupied cell даёт `0`, `1.0`, `sqrt(2)`;
   - negative-path: free/unknown cells не являются source;
   - edge-case: вне `max_distance_m` остаётся infinity/capped sentinel;
   - equivalence: на fixture grid сравнить prohibited-mask с текущей
     `rebuildInflation()` для радиусов `0`, `1.1`, `2.0`, `4.0`.

2. **Перевести inflation на distance field, сохранив контракт `OccupancyGrid2D`.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`;
   - `drone_city_nav/src/occupancy_grid.cpp`;
   - `drone_city_nav/tests/planning_grid_builder_test.cpp`;
   - новый/обновлённый `distance_field_test.cpp`.

   Материализуемый результат:
   - `OccupancyGrid2D::rebuildInflation(double radius_m)` внутри использует
     `DistanceField2D` от occupied cells вместо nested loop по occupied cells;
   - добавить helper `applyInflationFromDistanceField(const DistanceField2D&,
     double radius_m)` или private/internal function, чтобы grid builder мог
     применять уже построенный field без повторного build;
   - сохранить семантику margin из текущего кода:

   ```cpp
   const double radius_with_margin = radius_m + 0.5 * grid.resolution();
   inflated[cell] = occupied_distance[cell] <= radius_with_margin;
   ```

   Автотесты:
   - existing `PlanningGridBuilder.StaticOnlyBuildsInflatedGrid` должен пройти
     без изменения ожидаемой геометрии;
   - добавить property-style deterministic fixture: старая reference-функция в
     тесте размазывает occupied cells текущим способом, новый EDT должен дать тот
     же inflated mask.

3. **Расширить `PlanningGridBuilder` до явного static/dynamic distance-field cache.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`;
   - `drone_city_nav/src/planning_grid_builder.cpp`;
   - `drone_city_nav/tests/planning_grid_builder_test.cpp`.

   Материализуемый результат:
   - `StaticGridCache` хранит не только `raw_grid/prohibited_grid/planning_grid`,
     но и static occupied distance field, static prohibited/planning inflation
     masks, timing/cache stats;
   - dynamic sources (`memory_grid`, `current_lidar_grid`) собираются в
     `dynamic_raw`, для них строится dynamic distance field один раз, после чего
     получаются dynamic prohibited/planning masks;
   - итоговый результат сохраняет текущий raw input contract:

   ```text
   raw = static_raw OR memory_raw OR current_lidar_raw
   prohibited_inflated = static_inflated(radius)
                      OR dynamic_inflated(radius)
   planning_inflated = static_inflated(radius + planning_clearance)
                    OR dynamic_inflated(radius + planning_clearance)
   ```

   - не использовать inflated cells входных memory/lidar grids как raw obstacle
     evidence; существующий тест
     `SourceInflatedCellsAreNotReusedAsRawObstacles`
     ([drone_city_nav/tests/planning_grid_builder_test.cpp:183](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/tests/planning_grid_builder_test.cpp:183))
     должен оставаться смысловым guard.

   Автотесты:
   - cached static only equals full uncached build;
   - cached static + memory equals full uncached build;
   - cached static + current lidar equals full uncached build;
   - cache invalidates on static cells/bounds/inflation/planning clearance changes;
   - dynamic sources do not mutate cached static grids.

4. **Сделать `ClearanceField2D` лёгким adapter/wrapper над `DistanceField2D`.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/clearance_field.hpp`;
   - `drone_city_nav/src/clearance_field.cpp`;
   - `drone_city_nav/tests/clearance_field_test.cpp`.

   Материализуемый результат:
   - убрать priority-queue propagation из `ClearanceField2D::build()`;
   - `ClearanceField2D::build()` вызывает `DistanceField2D::build()` и сохраняет
     прежний публичный API;
   - `ClearanceFieldCache` использует fingerprint и параметры поля, но больше не
     копирует большие snapshots без необходимости. Достаточно строгого
     `OccupancyGridFingerprint` + bounds/source/radius; если оставить snapshots
     как paranoid check, это должно быть отдельным stats-флагом, чтобы видеть
     цену.

   Автотесты:
   - существующие тесты в `clearance_field_test.cpp` остаются валидными;
   - добавить тест, что cache hit не перестраивает field и не меняет pointer;
   - добавить тест invalidation для occupied/inflated/source/radius.

5. **Протянуть единое clearance/distance поле через planner -> corridor -> trajectory без повторных build.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`;
   - `drone_city_nav/include/drone_city_nav/planner_core.hpp`;
   - `drone_city_nav/src/planner_core.cpp`;
   - `drone_city_nav/src/planner_node_inputs.cpp`;
   - `drone_city_nav/src/planner_node_publish.cpp`;
   - `drone_city_nav/src/corridor.cpp`;
   - `drone_city_nav/tests/planner_core_test.cpp`;
   - `drone_city_nav/tests/trajectory_planner_test.cpp`;
   - `drone_city_nav/tests/corridor_test.cpp`.

   Материализуемый результат:
   - добавить overload или input struct для `PlannerCore::computePath()`:

   ```cpp
   struct PathComputationInput {
     const OccupancyGrid2D& grid;
     Point2 current_position;
     Point2 goal;
     AStarConfig astar;
     const ClearanceField2D* prohibited_clearance_field = nullptr;
     bool clearance_field_cache_hit = false;
   };
   ```

   - если field передан и покрывает `clearance_diagnostic_radius_m`, не строить
     новый field в `PlannerCore::computePath()`;
   - `TrajectoryPlannerInput` уже умеет принимать `prohibited_clearance_field`
     ([drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:59](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/include/drone_city_nav/trajectory_planner.hpp:59));
     сохранить этот путь и гарантировать, что `corridor.clearance_build=0.0ms`
     при reuse;
   - обновить lifetime: поле должно жить минимум до завершения
     `publishPathFromPathCells()`.

   Автотесты:
   - `PlannerCore` с provided clearance field даёт те же raw/smoothed clearance
     values и выставляет `prohibited_clearance_field_cache_hit=true`;
   - `TrajectoryPlanner.ReusesProvidedClearanceFieldForCorridor` остаётся и
     проверяет `clearance_field_reused=true`;
   - negative-path: provided field с другим bounds/source/radius не используется.

6. **Добавить подробные timing/cache diagnostics для grid/distance/clearance.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`;
   - `drone_city_nav/src/planner_node_inputs.cpp`;
   - `drone_city_nav/src/planner_diagnostics_format.cpp`;
   - `drone_city_nav/tests/planner_diagnostics_format_test.cpp`;
   - `drone_city_nav/src/trajectory_diagnostics_io.cpp`;
   - `drone_city_nav/tests/trajectory_diagnostics_io_test.cpp`;
   - `drone_city_nav/src/offboard_blackbox.cpp`;
   - `drone_city_nav/tests/offboard_blackbox_test.cpp`.

   Материализуемый результат:
   - в `Planning summary` добавить отдельные поля:

   ```text
   timing[grid_total=... overlay_static=... overlay_dynamic=...
          static_distance=... dynamic_distance=...
          prohibited_mask=... planning_mask=...
          clearance_field=... clearance_reused=...]
   distance_field[algorithm=edt source=occupied static_cache_hit=...]
   ```

   - в trajectory summary/blackbox оставить и расширить:
     `clearance_field_reused_by_corridor`, `corridor_clearance_field_cache_hit`,
     `racing_line_window_count`, `racing_line_active_window_count`,
     `racing_line_dp_states`, `racing_line_async_refined`.

   Автотесты:
   - JSON diagnostics parse round-trip;
   - blackbox JSON содержит новые поля и остаётся parseable.

7. **Заменить глобальный brute-force `racing_line` на windowed optimizer.**

   Файлы:
   - `drone_city_nav/include/drone_city_nav/racing_line.hpp`;
   - `drone_city_nav/src/racing_line.cpp`;
   - при необходимости новые `racing_line_windows.hpp/.cpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:
   - добавить построение `RacingLineWindow` из corridor samples;
   - активные окна строятся вокруг heading/curvature changes и ширинных
     изменений; почти прямые длинные участки не оптимизируются full-path
     candidates;
   - endpoint offsets в каждом окне фиксируются/сшиваются, чтобы не было скачка
     между оптимизированным окном и прямой;
   - current global optimizer path не остаётся runtime fallback; его можно
     оставить временно только в unit tests/reference helpers, если это нужно для
     equivalence checks.

   Псевдокод:

   ```cpp
   windows = detectActiveWindows(corridor_samples,
                                 heading_span_threshold,
                                 curvature_threshold,
                                 width_change_threshold,
                                 pre_margin_m,
                                 post_margin_m);
   offsets = zeros(corridor_samples.size());
   for (window : windows) {
     window_offsets = optimizeWindow(window, fixed_start_offset, fixed_end_offset);
     splice(offsets, window, window_offsets);
   }
   final_points = pointsFromOffsets(corridor_samples, offsets);
   final_score = scoreFullTrajectoryOnce(final_points);
   ```

   Автотесты:
   - straight corridor: `active_window_count == 0`, candidate evaluations резко
     меньше, output deterministic;
   - wide corner: есть active window, output traversable, estimated time не хуже
     centerline при заданном tolerance;
   - two corners separated by straight: два окна, offsets continuous at joins;
   - blocked centerline: optimizer всё ещё может выбрать lateral offset;
   - deterministic test сравнивает два запуска bitwise/double equality как
     текущий `RacingLine.ResultIsDeterministic`
     ([drone_city_nav/tests/racing_line_test.cpp:115](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/tests/racing_line_test.cpp:115)).

8. **Внутри active window заменить local brute-force на DP/control-point optimizer.**

   Файлы:
   - новые `drone_city_nav/include/drone_city_nav/racing_line_dp.hpp`;
   - новые `drone_city_nav/src/racing_line_dp.cpp`;
   - `drone_city_nav/src/racing_line.cpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`.

   Материализуемый результат:
   - discretize lateral offsets для каждого control point внутри window;
   - использовать dynamic programming по сечениям:

   ```text
   state(i, k) = best cost up to sample i with offset candidate k
   transition = segment length + curvature proxy + offset_delta + collision penalty
   final full score = one full trajectory evaluation + traversal time estimate
   ```

   - candidate collision проверять по `cellsOnLine()` только для локальных
     transition segments, а не пересобирать full path тысячи раз;
   - `RacingLineStats` получает `dp_states`, `dp_transitions`,
     `window_eval_duration_ms`, `full_final_score_duration_ms`.

   Автотесты:
   - DP на открытом широком повороте выбирает более широкий радиус, чем
     centerline, при включённом time/curvature cost;
   - collision negative-path: запрещённые offset candidates отбрасываются;
   - edge-case: узкий corridor с одним допустимым offset возвращает валидную
     centerline-like trajectory без oscillation.

9. **Если DP/control-point optimizer недостаточно быстро/плавно работает, добавить spline post-fit внутри window как часть того же pipeline.**

   Файлы:
   - `drone_city_nav/src/racing_line.cpp`;
   - возможно `drone_city_nav/src/trajectory.cpp`;
   - `drone_city_nav/tests/racing_line_test.cpp`;
   - `drone_city_nav/tests/trajectory_test.cpp`.

   Материализуемый результат:
   - после DP получить sparse control offsets;
   - интерполировать offsets Catmull-Rom/cubic Hermite по `s_m`;
   - resample в corridor samples и validate:
     `not prohibited`, `within left/right bound`, no heading jump above hard
     threshold;
   - это не отдельный runtime mode, а refinement step единственного racing-line
     pipeline.

   Автотесты:
   - spline interpolation не выходит за corridor bounds;
   - не ухудшает `max_heading_delta` на тестовом tight corner;
   - при invalid spline откатывается к DP result внутри того же window, а не к
     rough A* route.

10. **Добавить quick baseline trajectory и async refined racing trajectory.**

    Файлы:
    - `drone_city_nav/include/drone_city_nav/trajectory_planner.hpp`;
    - `drone_city_nav/src/trajectory_planner.cpp`;
    - `drone_city_nav/src/planner_node_publish.cpp`;
    - возможно новые `trajectory_build_state.hpp/.cpp`;
    - `drone_city_nav/tests/trajectory_planner_test.cpp`;
    - `drone_city_nav/tests/planner_path_publication_test.cpp`.

    Материализуемый результат:
    - synchronous quick baseline строится быстро из corridor centerline/initial
      DP seed, проходит traversability + speed profile и может быть опубликован
      сразу;
    - refined racing trajectory строится async и заменяет baseline только если:
      - `planner_generation/path_id` всё ещё актуален;
      - start/goal соответствуют текущему запросу;
      - trajectory traversable;
      - estimated time/shape diagnostics лучше baseline или не хуже заданного
        tolerance;
    - запрещён fallback на rough A* route как runtime path.

    Псевдокод:

    ```cpp
    baseline = buildBaselineTrajectory(route, grid, corridor, speed_config);
    publish(baseline, quality = "baseline");
    startAsync(refineRacingTrajectory(snapshot));
    if (future.ready() && future.generation == current_generation &&
        refined.valid && refined.traversable) {
      publish(refined, quality = "refined");
    }
    ```

    Автотесты:
    - stale async result rejected by generation mismatch;
    - invalid refined result does not replace valid baseline;
    - valid refined result replaces baseline and preserves final goal endpoint;
    - logs expose `trajectory_quality=baseline/refined`.

11. **Обновить конфиг только для новых диагностических/algorithm parameters с безопасными дефолтами.**

    Файлы:
    - `drone_city_nav/config/urban_mvp.yaml`;
    - `drone_city_nav/src/planner_node_config.cpp`;
    - `drone_city_nav/tests/planner_node_config_test.cpp`;
    - `drone_city_nav/tests/px4_offboard_config_test.cpp`, если проверяет YAML.

    Материализуемый результат:
    - новые параметры должны быть включены по умолчанию и совпадать между YAML и
      C++ defaults;
    - не добавлять kill-switch для legacy rough/A* runtime path;
    - параметры допустимы только для tuning window/DP sizes, async worker count,
      diagnostics, cache limits.

    Пример:

    ```yaml
    racing_line_window_pre_margin_m: 25.0
    racing_line_window_post_margin_m: 25.0
    racing_line_dp_offset_step_m: 1.0
    racing_line_async_refinement_workers: 1
    distance_field_algorithm: edt
    ```

12. **Добавить headless-oriented dump/metrics для сравнения до/после.**

    Файлы:
    - `drone_city_nav/src/trajectory_diagnostics_io.cpp`;
    - `drone_city_nav/src/final_trajectory_debug_io.cpp`;
    - `drone_city_nav/src/corridor_samples_io.cpp`;
    - `drone_city_nav/src/planner_node_publish.cpp`;
    - tests для IO round-trip.

    Материализуемый результат:
    - `log/final_trajectory_samples/latest_summary.json` содержит breakdown:
      grid/distance/clearance/window/DP/async;
    - corridor dump содержит `window_id`, `active_window`, `selected_offset_m`,
      `distance_to_prohibited_m`;
    - ROS log содержит компактный one-line summary, пригодный для `rg`.

## Verification plan

После реализации:

1. Format changed C++:

   ```bash
   ./scripts/dev_shell.sh
   make format
   ```

2. Scoped/unit verification через контейнерный workflow:

   ```bash
   ./scripts/test.sh
   ```

   При необходимости внутри dev shell для отдельных целей:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure -R 'distance_field|clearance_field|planning_grid_builder|corridor|racing_line|trajectory_planner|trajectory_diagnostics_io|offboard_blackbox'
   ```

3. Repository quality gate:

   ```bash
   ./scripts/dev_shell.sh
   make quality
   ```

4. Headless smoke/debug validation после успешных unit/quality:

   ```bash
   ./scripts/sim_headless.sh
   python3 scripts/analyze_lidar_projection_snapshots.py \
     log/lidar_debug/snapshots.jsonl \
     --static-map drone_city_nav/worlds/generated_city.map2d
   ```

5. Ручной GUI-прогон не является обязательным автотестом, но полезен как
   финальная визуальная проверка после headless:

   ```bash
   ./scripts/sim_gui.sh
   ```

## Testing strategy

### Категория 1: без рефакторинга / минимальные проверки

- `distance_field_test.cpp`: EDT distances, radius cap, source modes.
- `clearance_field_test.cpp`: прежний API на новом implementation.
- `planning_grid_builder_test.cpp`: equivalence cached/uncached static+dynamic.
- `racing_line_test.cpp`: deterministic output, no rough path fallback,
  window stats present.
- `trajectory_diagnostics_io_test.cpp`, `offboard_blackbox_test.cpp`: JSON fields
  parseable.

### Категория 2: лёгкий рефакторинг / локальные integration tests

- `PlannerCore` overload with supplied clearance field: same path/clearance as
  internal build, but field reuse flags set.
- `Corridor` uses provided field and does not rebuild when radius/source/bounds
  match.
- `TrajectoryPlanner` builds baseline and refined result with explicit quality
  statuses.
- `RacingLine` windowed optimizer tests:
  straight/no active windows, one corner/one active window, two corners/two
  windows, blocked centerline negative path.

### Категория 3: тяжёлый integration/headless scope

- `./scripts/test.sh` full C++ test suite.
- `make quality` inside container.
- `./scripts/sim_headless.sh` with logs inspected for:
  - reduced `grid`/`clearance_field`/`racing_line` timing;
  - first baseline publication latency;
  - refined replacement latency;
  - no rough A* runtime publication;
  - no new prohibited trajectory intersections.

## Risks and tradeoffs

- **Поведение trajectory может измениться.** Windowed/DP optimizer меняет
  алгоритм, поэтому нужно сравнивать shape metrics, estimated time, traversability
  и headless mission result.
- **Inflation equivalence.** EDT должен сохранить текущую continuous-radius
  семантику `radius + 0.5 * resolution`; ошибка здесь меняет safety envelope.
- **Static/dynamic split.** Нельзя использовать inflated cells из memory/lidar как
  raw obstacle evidence. Это уже важный контракт проекта.
- **Lifetime поля clearance.** Если передавать pointer на cached/temporary field
  через `PlannerCore -> trajectory`, он должен жить до конца публикации trajectory.
- **Async refined trajectory.** Нужна защита от stale generation/path_id, иначе
  можно применить траекторию для старой pose/старого route.
- **Threading.** DP/window/refinement и async worker не должны блокировать ROS/PX4
  setpoint loop и не должны писать в shared state без синхронизации.
- **CPU/memory.** EDT хранит `double` distance per cell; для больших grids это
  приемлемо, но нужно логировать размеры и не размножать поля без необходимости.
- **Diagnostics overhead.** Новые логи не должны снова стать bottleneck; JSON
  history dump должен быть bounded/один раз на rebuild.

## Open questions

1. Нужен ли persistent on-disk precompute для static city map в дополнение к
   in-memory static cache? Для GUI стартов это может убрать cold-start rebuild,
   но добавит вопрос invalidation по версии map/config.
2. Какой целевой SLA для первого baseline path и refined trajectory: например
   `<500 ms` для baseline и `<1 s` для refined? Сейчас ориентир из логов:
   `~4 s` total до публикации.
3. Допустимо ли публиковать baseline trajectory сразу в полёте при replan, если
   refined ещё строится, или для runtime replan нужно сначала hold до refined?
   Технически baseline валиден, но поведение может быть менее “racing”.
4. Нужно ли сохранять старый global brute-force optimizer как test-only reference
   на время миграции, или удалить сразу после DP/windowed реализации?
