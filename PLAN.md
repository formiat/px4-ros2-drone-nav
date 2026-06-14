# План: статическая 2D карта города как третий источник препятствий

## Context

Нужно спланировать доработку навигации так, чтобы дрон мог учитывать не только
накопленную карту по лидару и моментальные лидарные попадания, но и заранее
подготовленную статическую 2D карту текущего города. Важное требование:
статическая карта должна быть полноценным источником препятствий. Если
статическая карта говорит, что в клетке есть здание, а лидар в данный момент его
не видит, планировщик всё равно должен считать эту клетку занятой.

Все три источника должны включаться и выключаться перед запуском:

1. статическая карта города;
2. накопленная obstacle memory по лидару;
3. текущий lidar overlay.

По умолчанию все три источника должны быть включены.

## Investigation context

`PLAN.md` и `INVESTIGATION.md` до начала работы в репозитории отсутствовали.

Прочитаны обязательные локальные инструкции оркестратора:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`;
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`.

Notion и GitLab не использовались: в задаче нет Notion task id, GitLab issue,
MR или прямого запроса на удалённые GitLab-действия.

Текущая архитектура:

- `drone_city_nav/src/obstacle_memory_node.cpp:332` интегрирует лидар в
  `ObstacleMemoryGrid` и публикует raw/inflated occupancy grid.
- `drone_city_nav/src/planner_node.cpp:258` конвертирует
  `nav_msgs::msg::OccupancyGrid` из obstacle memory в `OccupancyGrid2D`.
- `drone_city_nav/src/planner_node.cpp:304` строит planning grid, сейчас
  требуя наличие memory grid.
- `drone_city_nav/src/planner_node.cpp:332` берёт memory grid как базу.
- `drone_city_nav/src/planner_node.cpp:557` накладывает текущие lidar hits.
- `drone_city_nav/src/planner_node.cpp:335` выполняет inflation уже после
  наложения источников.
- `drone_city_nav/config/urban_mvp.yaml:1` задаёт размеры grid для memory.
- `drone_city_nav/config/urban_mvp.yaml:35` задаёт параметры planner.
- `drone_city_nav/config/urban_mvp.yaml:171` содержит список зданий для
  mission monitor; его можно использовать как исходные данные для карты
  текущего города.
- `scripts/run_city_mvp.sh:36` принимает единый params file.
- `scripts/run_city_mvp.sh:287` запускает ROS launch и уже умеет делать
  headless smoke-check по логам.

## Detected stack/profiles

- Generic profile: прочитан; применим для поиска repo-approved commands,
  фиксации skipped checks и выбора минимального scope проверки.
- C++ profile: прочитан; применим, потому что пакет написан на C++20 и
  собирается через ROS 2 `ament_cmake`/`colcon`.
- ROS 2 workspace: пакет `drone_city_nav`, сборка через `colcon`, не через
  ad-hoc top-level CMake.
- Gazebo + PX4 SITL: запуск через `scripts/run_city_mvp.sh`.
- Форматирование и quality gate уже закреплены в репозитории через scripts и
  Makefile.

## Repo-approved commands found

- `./scripts/dev_shell.sh` - вход в dev container с host UID/GID.
- `make build` - `colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
- `make test` - сборка и `ctest --test-dir build/drone_city_nav --output-on-failure`.
- `make quality` - `./scripts/check_cpp_quality.sh`.
- `./scripts/format_cpp_changed.sh` - форматирование изменённых C++ файлов.
- `make sim-headless` - headless запуск через `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`.
- `make sim-gui` - GUI запуск симуляции.
- `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=<seconds> ./scripts/run_city_mvp.sh` - полный headless-прогон с mission monitor.

## Affected components

- `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp` и
  `drone_city_nav/src/occupancy_grid.cpp`: базовая grid-модель, inflation,
  blocked/occupied/free semantics.
- Новый core-модуль `drone_city_nav/include/drone_city_nav/static_city_map.hpp`
  и `drone_city_nav/src/static_city_map.cpp`: загрузка и rasterization
  статической карты.
- Новый core-модуль `drone_city_nav/include/drone_city_nav/grid_overlay.hpp`
  и `drone_city_nav/src/grid_overlay.cpp`: union overlay источников с
  правилом, что occupied не может быть очищен другим источником.
- `drone_city_nav/src/planner_node.cpp`: загрузка static map, source toggles,
  построение planning grid из трёх источников, расширенные логи.
- `drone_city_nav/src/obstacle_memory_node.cpp`: необязательный runtime toggle
  для отключения накопления memory source на стороне node; основной source toggle
  всё равно должен быть в planner.
- `drone_city_nav/config/urban_mvp.yaml`: дефолтные параметры всех трёх
  источников, путь к статической карте, сохранение текущих параметров lidar.
- `drone_city_nav/launch/city_nav.launch.py`: проброс параметров без
  расхождения GUI/headless.
- `scripts/run_city_mvp.sh`: env-overrides перед запуском для всех трёх
  источников.
- `drone_city_nav/worlds/generated_city.sdf` и новый файл рядом с ним,
  например `drone_city_nav/worlds/generated_city.map2d`: карта текущего города.
- Новый gtest `drone_city_nav/tests/static_city_map_test.cpp`: parser и
  rasterization tests.
- Новый gtest `drone_city_nav/tests/grid_overlay_test.cpp`: merge precedence
  tests.
- Расширение `drone_city_nav/tests/planner_core_test.cpp`: A* regression на
  static-only obstacle после union overlay.
- `docs/MVP_SIMULATION.md` и `README.md`: описание формата карты,
  переключателей и headless-диагностики.

## Implementation steps

1. Зафиксировать простой версионированный формат статической карты рядом с
   world file.

   Рекомендуемый MVP-формат: line-oriented `*.map2d` без новой зависимости от
   YAML/JSON парсера. Он проще для C++ parser tests и не добавляет лишних
   системных зависимостей.

   Пример `drone_city_nav/worlds/generated_city.map2d`:

   ```text
   drone_city_nav_static_map_v1
   frame_id map
   bounds -10.0 -10.0 0.5 115.0 175.0
   # rect <id> <center_x_m> <center_y_m> <size_x_m> <size_y_m> <height_m>
   rect building_001 9.0 9.0 8.0 8.0 28.0
   ```

   `bounds` должны совпадать с текущими grid-параметрами из
   `drone_city_nav/config/urban_mvp.yaml:9`. Высоту здания нужно хранить даже
   если первый MVP использует 2D blocking без altitude filtering: это оставит
   место для будущего режима, где низкие здания не блокируют полёт на большой
   высоте.

2. Создать карту для текущего города.

   Источник данных: текущие building volumes в
   `drone_city_nav/config/urban_mvp.yaml:171` и соответствующая геометрия в
   `drone_city_nav/worlds/generated_city.sdf`. Сгенерировать или вручную
   перенести все прямоугольники в `generated_city.map2d`, сохранив deterministic
   ordering по `x`, затем `y`.

   Для текущей упрощённой Manhattan-локации все дома должны попасть в карту как
   axis-aligned rectangles. Это важно, чтобы mission monitor, Gazebo world и
   planner смотрели на один и тот же город.

3. Добавить loader/rasterizer в core library.

   Новый API:

   ```cpp
   struct StaticCityMapRect {
     std::string id;
     Point2 center;
     double size_x_m;
     double size_y_m;
     double height_m;
   };

   struct StaticCityMap {
     std::string frame_id;
     GridBounds bounds;
     std::vector<StaticCityMapRect> rectangles;
   };

   StaticCityMap loadStaticCityMap(const std::filesystem::path& path);
   OccupancyGrid2D rasterizeStaticCityMap(const StaticCityMap& map,
                                          double min_blocking_height_m);
   ```

   Подключить `src/static_city_map.cpp` в `drone_city_nav/CMakeLists.txt:28`
   внутри target `drone_city_nav_core`. Parser должен явно валидировать version,
   positive resolution, positive sizes, finite numbers, duplicate ids и выход
   прямоугольников за bounds.

4. Реализовать union merge источников препятствий.

   Добавить `drone_city_nav/include/drone_city_nav/grid_overlay.hpp` и
   `drone_city_nav/src/grid_overlay.cpp`, затем подключить `src/grid_overlay.cpp`
   в `drone_city_nav/CMakeLists.txt:28` внутри target `drone_city_nav_core`.
   Helper должен быть уровня core, чтобы тестировать merge без ROS node:

   - `overlayOccupiedCells(destination, source)` - занятые клетки source
     становятся занятыми в destination.
   - `overlayKnownMemoryCells(destination, memory)` - occupied из memory
     становится occupied, free может стать free только если destination ещё
     unknown; free не должен очищать static occupied или current lidar occupied.
   - `overlayCurrentLidarCells(destination, current_lidar_grid)` - occupied из
     текущего лидара становится occupied и не очищается другими источниками.

   `applyCurrentLidarOverlay(...)` в `drone_city_nav/src/planner_node.cpp:557`
   остаётся node-level, потому что там есть `LaserScan`, pose и параметры
   преобразования, но результат должен накладываться тем же правилом
   `overlayCurrentLidarCells`.

   Итоговое правило planning grid:

   ```text
   blocked = static_occupied OR memory_occupied OR current_lidar_occupied
   free = memory_free only where no source says occupied
   unknown = everything else
   inflation = after all enabled source overlays
   ```

5. Переделать `planner_node` так, чтобы он больше не зависел обязательно от
   obstacle memory.

   Сейчас `planner_node.cpp:324` публикует hold path, если `memory_grid_` ещё
   отсутствует. После доработки должна быть функция вроде
   `buildPlanningGrid(now_ns)`, которая:

   - создаёт empty grid из static map bounds, memory grid bounds или отдельных
     fallback planner bounds params;
   - накладывает static map, если `use_static_map=true`;
   - накладывает memory grid, если `use_obstacle_memory=true` и grid свежая;
   - накладывает current lidar, если `use_current_lidar_obstacles=true`;
   - выполняет `rebuildInflation(inflation_radius_m_)`;
   - возвращает grid и source stats для логов.

   Если все источники выключены, planner должен не лететь "куда-нибудь", а
   публиковать hold path с явным warning.

6. Добавить параметры и дефолты.

   В `planner_node`:

   - `use_static_map: true`;
   - `static_map_path: <share>/worlds/generated_city.map2d` или путь из config;
   - `static_map_min_blocking_height_m: 0.0`;
   - `use_obstacle_memory: true`;
   - `use_current_lidar_obstacles: true` уже есть и должен остаться;
   - `planning_grid_origin_x`, `planning_grid_origin_y`,
     `planning_grid_width_m`, `planning_grid_height_m`,
     `planning_grid_resolution_m` как fallback, если memory/static отключены.

   В `obstacle_memory_node` опционально добавить `mapping_enabled: true`, чтобы
   можно было отключить накопление без удаления node из launch. Но planner-side
   `use_obstacle_memory` является обязательным source toggle.

7. Добавить env-overrides в запуск.

   В `scripts/run_city_mvp.sh` добавить переменные:

   - `ENABLE_STATIC_MAP=true|false`;
   - `ENABLE_OBSTACLE_MEMORY=true|false`;
   - `ENABLE_CURRENT_LIDAR=true|false`;
   - `STATIC_CITY_MAP_PATH=<path>` опционально.

   Скрипт должен генерировать временный params overlay в `build/...` или
   передавать параметры через launch так, чтобы GUI и headless использовали
   один и тот же путь. Дефолт без env должен соответствовать текущему запуску
   плюс `use_static_map=true`.

8. Расширить headless-логи.

   В `Planning summary` из `planner_node.cpp:388` добавить source-level
   статистику:

   - `static[enabled loaded occupied_cells rects path=...]`;
   - `memory[enabled seen fresh occupied free unknown]`;
   - `current_lidar[enabled used fresh hits occupied_cells outside]`;
   - итоговые `occupied`, `inflated`, `free`, `unknown`;
   - причину hold/fallback, если grid построить нельзя.

   В startup-лог добавить одну строку конфигурации источников, чтобы в
   headless-логе сразу было видно, с чем запущен planner.

9. Добавить debug publication для статической карты.

   В `drone_city_nav/src/planner_node.cpp` добавить параметр
   `static_map_grid_topic` со значением по умолчанию
   `/drone_city_nav/static_map_grid`, publisher
   `static_map_pub_` с `rclcpp::QoS{1}.transient_local()` и функцию
   `publishStaticMapGrid(const OccupancyGrid2D&)`. Topic должен публиковаться
   после успешной загрузки static map и повторно при старте planner, чтобы RViz
   и headless subscriber могли отличить ошибку загрузки карты от ошибки
   lidar/memory.

   В `drone_city_nav/config/urban_mvp.yaml` добавить `static_map_grid_topic`.
   В `docs/MVP_SIMULATION.md` описать, что `/drone_city_nav/static_map_grid`
   показывает только статический слой, а `/drone_city_nav/occupancy_grid`
   показывает итоговый planning grid после union overlay и inflation.

10. Добавить unit tests и CMake targets.

   Создать `drone_city_nav/tests/static_city_map_test.cpp` и добавить в
   `drone_city_nav/CMakeLists.txt:116`:

   ```cmake
   ament_add_gtest(static_city_map_test tests/static_city_map_test.cpp)
   target_compile_features(static_city_map_test PRIVATE cxx_std_20)
   target_link_libraries(static_city_map_test drone_city_nav_core)
   enable_project_warnings(static_city_map_test)
   ```

   В `static_city_map_test` покрыть:

   - валидный `*.map2d` загружается;
   - invalid version отклоняется;
   - rectangle outside bounds отклоняется;
   - rasterized rectangle даёт occupied клетки в ожидаемом месте;
   - height threshold может исключить низкое здание, если параметр включён.

   Создать `drone_city_nav/tests/grid_overlay_test.cpp` и добавить в
   `drone_city_nav/CMakeLists.txt:116`:

   ```cmake
   ament_add_gtest(grid_overlay_test tests/grid_overlay_test.cpp)
   target_compile_features(grid_overlay_test PRIVATE cxx_std_20)
   target_link_libraries(grid_overlay_test drone_city_nav_core)
   enable_project_warnings(grid_overlay_test)
   ```

   В `grid_overlay_test` покрыть:

   - static occupied остаётся occupied, даже если memory говорит free;
   - memory occupied блокирует путь без static;
   - current lidar occupied блокирует путь даже при пустой memory;
   - disabled source не влияет на итоговую grid;
   - inflation выполняется после union overlay.

   Расширить существующий `drone_city_nav/tests/planner_core_test.cpp` тестом
   `AStarAvoidsStaticOnlyObstacleAfterOverlay`: собрать grid, наложить static
   obstacle через `overlayOccupiedCells`, выполнить A* и проверить, что путь не
   проходит через occupied/inflated клетки.

11. Обновить документацию.

   В `docs/MVP_SIMULATION.md` описать:

   - формат `generated_city.map2d`;
   - смысл трёх источников препятствий;
   - env-переключатели;
   - какие строки искать в `log/ros_city_mvp.log`;
   - чем static map отличается от lidar memory.

   В `README.md` добавить короткую ссылку на этот раздел, не дублируя всю
   документацию.

12. После реализации выполнить scoped formatting, quality gate и commit.

   Форматировать только изменённые C++ файлы через
   `./scripts/format_cpp_changed.sh`. Затем запустить
   `./scripts/check_cpp_quality.sh` или `make quality` в dev container.

## Verification plan

Для самого планового артефакта достаточно проверить git diff и выполнить
repo-approved quality gate перед commit.

Для будущей реализации:

1. `./scripts/format_cpp_changed.sh`.
2. `make quality` или `./scripts/check_cpp_quality.sh`.
3. `ctest --test-dir build/drone_city_nav --output-on-failure`.
4. Headless default:

   ```bash
   HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=240 ./scripts/run_city_mvp.sh
   ```

5. Headless source-toggle matrix:

   ```bash
   HEADLESS=1 SMOKE_DURATION_S=120 ENABLE_STATIC_MAP=false ENABLE_OBSTACLE_MEMORY=true ENABLE_CURRENT_LIDAR=true ./scripts/run_city_mvp.sh
   HEADLESS=1 SMOKE_DURATION_S=120 ENABLE_STATIC_MAP=true ENABLE_OBSTACLE_MEMORY=false ENABLE_CURRENT_LIDAR=false ./scripts/run_city_mvp.sh
   HEADLESS=1 SMOKE_DURATION_S=120 ENABLE_STATIC_MAP=true ENABLE_OBSTACLE_MEMORY=true ENABLE_CURRENT_LIDAR=false ./scripts/run_city_mvp.sh
   ```

6. Проверить в `log/ros_city_mvp.log`, что есть строки:

   - `Planner obstacle sources: static=true memory=true current_lidar=true`;
   - `static[enabled=true loaded=true ...]`;
   - `memory[enabled=true ...]`;
   - `current_lidar[enabled=true ...]`;
   - `Published path: waypoints=...`;
   - `MISSION_RESULT success=true` для полного mission check.

## Testing strategy

Категория 1: без рефакторинга.

Scope:

- добавить загрузку `generated_city.map2d` прямо в
  `drone_city_nav/src/planner_node.cpp`;
- добавить параметры source toggles в `planner_node`;
- оставить merge-логику внутри `planner_node::replanAndPublish()`;
- не добавлять `static_city_map` и `grid_overlay` core-модули.

Tradeoffs:

- плюс: минимальный diff;
- минус: parser, rasterizer и merge precedence окажутся привязаны к ROS node,
  их сложнее покрыть быстрыми unit tests;
- минус: выше риск регрессии, где memory free случайно очищает static occupied.

Автоматизированные tests/log checks:

- расширить только `drone_city_nav/tests/planner_core_test.cpp` невозможно
  полноценно, потому что ключевая логика останется в ROS node;
- можно добавить ограниченный node-level smoke через headless logs:
  `static[enabled=true loaded=true ...]` и `Published path: waypoints=...`;
- gap: negative parser cases и merge precedence не будут надёжно проверены
  unit-тестами.

Эта категория не рекомендована для задачи: она формально быстрее, но слишком
слабо покрывает главный риск объединения трёх источников.

Категория 2: лёгкий рефакторинг.

Scope:

- добавить `drone_city_nav/include/drone_city_nav/static_city_map.hpp` и
  `drone_city_nav/src/static_city_map.cpp`;
- добавить `drone_city_nav/include/drone_city_nav/grid_overlay.hpp` и
  `drone_city_nav/src/grid_overlay.cpp`;
- подключить оба `.cpp` файла в `drone_city_nav_core` через
  `drone_city_nav/CMakeLists.txt:28`;
- добавить `drone_city_nav/tests/static_city_map_test.cpp` target
  `static_city_map_test`;
- добавить `drone_city_nav/tests/grid_overlay_test.cpp` target
  `grid_overlay_test`;
- расширить `drone_city_nav/tests/planner_core_test.cpp` тестом
  `AStarAvoidsStaticOnlyObstacleAfterOverlay`;
- в `planner_node` оставить только ROS glue: параметры, загрузку файла,
  вызовы core helpers, публикацию debug topics и логирование.

Tradeoffs:

- плюс: основной алгоритмический контракт проверяется быстрыми gtest без
  Gazebo/PX4;
- плюс: merge precedence становится явным и переиспользуемым;
- плюс: implementation остаётся умеренным по размеру и не меняет модель города;
- минус: появляются два новых core-модуля и два новых test targets.

Автоматизированные tests/log checks:

- `static_city_map_test`: happy-path загрузка, invalid version, rectangle outside
  bounds, rasterization, height threshold edge case;
- `grid_overlay_test`: static occupied wins over memory free, memory occupied
  works without static, current lidar occupied works without memory, disabled
  source ignored, inflation after union overlay;
- `planner_core_test`: A* не проходит через static-only obstacle после overlay;
- `ctest --test-dir build/drone_city_nav -R 'static_city_map_test|grid_overlay_test|planner_core_test' --output-on-failure`;
- `./scripts/check_cpp_quality.sh`;
- headless log checks для default run:
  `Planner obstacle sources: static=true memory=true current_lidar=true`,
  `static[enabled=true loaded=true ...]`,
  `memory[enabled=true ...]`,
  `current_lidar[enabled=true ...]`.

Эта категория рекомендована для текущей задачи: она закрывает главный риск
объединения источников и остаётся достаточно небольшой для MVP.

Категория 3: тяжёлый рефакторинг.

Scope:

- сделать единый source-of-truth для города, из которого генерируются
  `generated_city.sdf`, `generated_city.map2d` и `mission_monitor_node`
  building volumes;
- вынести источники препятствий в отдельный `obstacle_source_manager` node или
  library contract;
- добавить launch/integration tests для matrix источников;
- пересобрать документацию вокруг генератора города, а не вокруг ручных списков
  rectangles.

Tradeoffs:

- плюс: меньше риск рассинхронизации Gazebo world, static map и mission monitor;
- плюс: лучше масштабируется на будущие города;
- минус: большой scope, много файлов, больше риск побочных регрессий в уже
  работающем MVP;
- минус: требует отдельного проектирования генератора и e2e checks, что выходит
  за рамки текущего planning request.

Автоматизированные tests/log checks:

- tests генератора: один input даёт ожидаемые SDF/map2d/mission volumes;
- integration tests source manager;
- полный headless mission matrix для static-only, memory-only, lidar-only и
  all-enabled modes;
- дополнительные checks install/share путей для generated artifacts.

Эта категория не рекомендована сейчас: это правильное долгосрочное направление,
но для текущей задачи оно переусложняет MVP.

## Risks and tradeoffs

- `*.map2d` проще и надёжнее для C++ MVP, но менее стандартен, чем YAML/PGM.
  Это осознанный tradeoff: не добавляем новую зависимость ради простой карты из
  прямоугольников.
- Если static map и Gazebo world разойдутся, planner будет избегать
  несуществующих зданий или не видеть реальные. Нужно держать map file рядом с
  world file и по возможности генерировать оба из одного описания в будущем.
- Если memory grid bounds и static map bounds не совпадут, merge может молча
  потерять клетки. Нужно валидировать bounds/resolution и логировать mismatch.
- Если включить только current lidar без static/memory, plannerу всё равно
  нужны grid bounds. Поэтому нужен fallback grid config.
- 2D карта не учитывает высоту здания. Для текущего MVP это нормально, потому
  что город тестируется как 2D obstacle field на фиксированной высоте. Высоту
  всё равно сохраняем в формате для будущей фильтрации.

## Что могло сломаться

- Дрон может начать строить путь по статической карте, но obstacle memory будет
  публиковать free cells поверх домов, если merge precedence реализовать
  неправильно.
- GUI/RViz может показывать итоговую inflated grid, но не отдельный static
  layer; из-за этого будет сложно понять, загружена ли карта.
- Headless smoke-check может проходить даже при отключённой static map, если
  regex проверяет только наличие path. Нужно добавить source-specific log
  checks.
- Static map path может работать из source tree, но сломаться после install,
  если не установить `worlds/*.map2d` через CMake.
- Env-overrides могут начать расходиться с YAML params, если script генерирует
  overlay не для всех nodes.

## Open questions

- Нужно ли в первом implementation pass учитывать высоту зданий при 2D
  blocking, или пока считать все rectangles препятствиями независимо от высоты?
  Для текущего города безопаснее блокировать все здания.
- Должна ли статическая карта быть единственным source of truth для mission
  monitor building volumes в будущем? Сейчас план только синхронизирует данные,
  но не убирает список из `urban_mvp.yaml`.
- Нужен ли отдельный RViz layer `/drone_city_nav/static_map_grid` как обязательная
  часть MVP, или достаточно расширенных логов и итоговой occupancy grid?
