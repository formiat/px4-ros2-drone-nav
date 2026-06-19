# Context

Нужно подготовить архитектурный план для приведения работы с препятствиями к
строгой схеме:

1. Все источники препятствий публикуют и передают дальше только raw obstacles.
2. Planner собирает raw sources в одну карту.
3. Inflation выполняется один раз перед передачей карты в A*.
4. Planning-facing код использует единый термин `prohibited`.
5. `occupied` и `inflated` остаются только как внутренние/debug-состояния grid.
6. `hit_obstacle_depth_m` и `current_lidar_obstacle_depth_m` трактуются как
   sensor preprocessing, а не как safety inflation.

`astar_turn_cost_weight` не считается проблемой и не должен удаляться: штраф за
повороты нужен, потому что он делает путь более прямым.

# Investigation context

`INVESTIGATION.md` отсутствует.

Ключевые наблюдения по текущему коду:

1. `PlanningGridBuilder` уже содержит правильную точку финального inflation:
   `drone_city_nav/src/planning_grid_builder.cpp:114`
   вызывает `planning_grid.rebuildInflation(config.inflation_radius_m)` после
   overlay всех источников.

2. `obstacle_memory_node` всё ещё хранит и публикует отдельную inflated memory
   карту:
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:75`
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:87`
   - `drone_city_nav/src/obstacle_memory.cpp:230`
   - `drone_city_nav/src/obstacle_memory_node.cpp:171`
   - `drone_city_nav/src/obstacle_memory_node.cpp:441`
   - `drone_city_nav/src/obstacle_memory_node.cpp:512`

3. `lidar_debug_node` сейчас может смотреть на inflated memory topic через
   конфиг:
   `drone_city_nav/config/urban_mvp.yaml:175`.

4. `ros_conversions` может опубликовать inflated cells как `80`, а затем другой
   код может прочитать эти значения как occupied при threshold `65`:
   - `drone_city_nav/src/ros_conversions.cpp:44`
   - `drone_city_nav/src/ros_conversions.cpp:74`

5. A* уже в основном работает через prohibited:
   - `drone_city_nav/src/astar_planner.cpp:80`
   - `drone_city_nav/src/astar_planner.cpp:176`
   - `drone_city_nav/src/astar_planner.cpp:232`

6. Planning-facing диагностика всё ещё смешивает `occupied` и `prohibited`:
   - `drone_city_nav/src/planner_node.cpp:733`
   - `drone_city_nav/src/planner_core.cpp:188`
   - `drone_city_nav/src/planner_core.cpp:202`

7. Lidar hit depth сейчас реализован как расширение попадания лидара вглубь
   препятствия, а не как safety inflation:
   - `drone_city_nav/src/current_lidar_overlay.cpp:9`
   - `drone_city_nav/src/current_lidar_overlay.cpp:50`
   - `drone_city_nav/src/obstacle_memory.cpp:144`
   - `drone_city_nav/src/obstacle_memory.cpp:199`
   - `drone_city_nav/config/urban_mvp.yaml:37`
   - `drone_city_nav/config/urban_mvp.yaml:97`

Что это значит: lidar hit depth не должен раздувать препятствие ради safety. Он
только делает raw sensor hit чуть толще, чтобы одиночное попадание не терялось
из-за дискретизации grid. Safety-запас должен появляться только на этапе
inflation в `PlanningGridBuilder`.

# Detected stack/profiles

Обнаруженный стек:

1. ROS 2 workspace.
2. C++20.
3. ament CMake / colcon.
4. Gazebo + PX4 SITL simulation.
5. Python helper scripts and script tests.

Прочитанные orchestrator профили:

1. `generic`
2. `cpp`

Прочитанные протоколы:

1. Notion access protocol: Notion не требуется, потому что prompt не содержит
   Notion task.
2. GitLab access protocol: GitLab не требуется, потому что prompt не содержит
   GitLab/MR задачи.

# Repo-approved commands found

Документированные команды из `README.md`, `CONTRIBUTING.md`, `Makefile` и
`drone_city_nav/CMakeLists.txt`:

1. Пользовательские wrapper scripts:
   - `./scripts/build.sh`
   - `./scripts/test.sh`
   - `./scripts/sim_gui.sh`
   - `./scripts/sim_headless.sh`

2. Интерактивный контейнер:
   - `./scripts/dev_shell.sh`

3. Команды внутри контейнера:
   - `make build`
   - `make test`
   - `make test-scripts`
   - `make quality`
   - `make format`
   - `make sim-gui`
   - `make sim-headless`

4. Репозиторный workflow:
   - использовать только container workflow;
   - предпочитать `colcon` через `Makefile` и scripts;
   - не запускать ad-hoc top-level CMake commands;
   - перед commit форматировать изменённые C++ файлы через `make format`;
   - перед commit запускать `make quality`.

# Affected components

1. Core grid:
   - `drone_city_nav/include/drone_city_nav/occupancy_grid.hpp`
   - `drone_city_nav/src/occupancy_grid.cpp`

2. Planning grid assembly:
   - `drone_city_nav/include/drone_city_nav/planning_grid_builder.hpp`
   - `drone_city_nav/src/planning_grid_builder.cpp`

3. A* and path validation:
   - `drone_city_nav/src/astar_planner.cpp`
   - `drone_city_nav/include/drone_city_nav/planner_core.hpp`
   - `drone_city_nav/src/planner_core.cpp`
   - `drone_city_nav/src/path_smoothing.cpp`

4. Planner ROS adapter:
   - `drone_city_nav/src/planner_node.cpp`
   - `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
   - `drone_city_nav/src/planner_node_config.cpp`

5. Offboard prohibited-grid consumer:
   - `drone_city_nav/src/px4_offboard_node.cpp`

6. Obstacle memory:
   - `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp`
   - `drone_city_nav/src/obstacle_memory.cpp`
   - `drone_city_nav/src/obstacle_memory_node.cpp`

7. Current lidar overlay:
   - `drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp`
   - `drone_city_nav/src/current_lidar_overlay.cpp`

8. ROS conversions:
   - `drone_city_nav/include/drone_city_nav/ros_conversions.hpp`
   - `drone_city_nav/src/ros_conversions.cpp`

9. Lidar debug and RViz:
   - `drone_city_nav/src/lidar_debug_node.cpp`
   - `drone_city_nav/rviz/city_nav_debug.rviz`

10. Debug scripts, config and docs:
   - `scripts/record_debug_bag.sh`
   - `drone_city_nav/config/urban_mvp.yaml`
   - `drone_city_nav/config/real_drone_template.yaml`
   - `docs/MVP_SIMULATION.md`
   - `README.md`
   - `CONTRIBUTING.md`

11. Tests:
    - `drone_city_nav/CMakeLists.txt`
    - `drone_city_nav/tests/planning_grid_builder_test.cpp`
    - `drone_city_nav/tests/obstacle_memory_test.cpp`
    - `drone_city_nav/tests/current_lidar_overlay_test.cpp`
    - `drone_city_nav/tests/ros_conversions_test.cpp`
    - `drone_city_nav/tests/planner_node_config_test.cpp`
    - `scripts/tests/test_topic_contract.py`
    - `drone_city_nav/tests/lidar_debug_renderer_test.cpp`
    - `drone_city_nav/tests/lidar_snapshot_writer_test.cpp`

# Implementation steps

1. Зафиксировать planning-facing термин `prohibited`.

   Что сейчас:

   - `OccupancyGrid2D` хранит `occupied` и `inflated`, а `isProhibited()`
     объединяет оба состояния:
     `drone_city_nav/src/occupancy_grid.cpp:90`.
   - A* уже использует `isProhibited()`.
   - Часть path safety диагностики всё ещё отдельно считает `occupied` и
     `prohibited`.

   Что не так:

   - Planning-facing API и логи не должны подсказывать A*, почему зона
     запрещена.
   - Если планирование видит `occupied` отдельно от `inflated`, в коде легко
     снова добавить source-specific поведение.

   Как было бы лучше:

   - В planning-facing коде использовать только `prohibited`.
   - `occupied` и `inflated` оставить только для source/debug/statistics.

   Что сделать:

   - В `drone_city_nav/include/drone_city_nav/planner_core.hpp` удалить
     planning-facing API `pathSegmentOccupiedLengthM`.
   - В `drone_city_nav/src/planner_core.cpp:188` удалить реализацию
     `pathSegmentOccupiedLengthM`.
   - В `drone_city_nav/src/planner_node.cpp:748` удалить call site
     `pathSegmentOccupiedLengthM(...)`; summary path safety должен использовать
     только `pathSegmentProhibitedLengthM(...)`.
   - В `drone_city_nav/src/planner_node.cpp:733` переписать summary path safety
     так, чтобы planning decision логировал:

     ```text
     prohibited_segments=<n> prohibited_length_m=<x>
     ```

     а `occupied`/`inflated` оставались только в grid stats/debug summary.

   - Добавить unit test в `drone_city_nav/tests/planner_core_test.cpp` или
     расширить существующий `planner_core_test`, если файл уже есть: path
     validation должен считать occupied и inflated одинаково запрещёнными.

2. Переименовать lidar hit depth как sensor preprocessing.

   Что сейчас:

   - `ObstacleMemoryConfig::hit_obstacle_depth_m`:
     `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:35`.
   - Planner current lidar overlay использует
     `current_lidar_obstacle_depth_m`:
     `drone_city_nav/config/urban_mvp.yaml:97`.
   - Эти параметры добавляют клетки вглубь препятствия до общего inflation.

   Что не так:

   - Названия похожи на obstacle inflation.
   - Из-за этого легко принять source preprocessing за safety radius.

   Как было бы лучше:

   - Назвать это `sensor_hit_depth_m`.
   - В логах явно писать `sensor_hit_depth`, а не `obstacle_depth`.
   - Держать simulation default минимальным: `1.0 m`.

   Что сделать:

   - В `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` заменить
     `hit_obstacle_depth_m` на `sensor_hit_depth_m`.
   - В `drone_city_nav/src/obstacle_memory.cpp:144` и
     `drone_city_nav/src/obstacle_memory.cpp:199` заменить имена переменных и
     stats labels.
   - В `drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp`
     заменить `obstacle_depth_m` на `sensor_hit_depth_m`.
   - В `drone_city_nav/src/current_lidar_overlay.cpp:9` и
     `drone_city_nav/src/current_lidar_overlay.cpp:50` обновить имена
     параметров и комментарии.
   - В `drone_city_nav/src/planner_node.cpp:177` заменить лог:

     ```text
     lidar_overlay: sensor_hit_depth_m=<value>
     ```

   - В `drone_city_nav/config/urban_mvp.yaml`:
     - `hit_obstacle_depth_m` -> `sensor_hit_depth_m`;
     - `current_lidar_obstacle_depth_m` -> `current_lidar_sensor_hit_depth_m`;
     - оставить оба значения `1.0`.
   - В `drone_city_nav/config/real_drone_template.yaml` сделать такие же
     rename. Если значение сейчас больше simulation default, оставить его
     только при явном комментарии в docs, что это deployment calibration, а не
     inflation.
   - Обновить `drone_city_nav/tests/obstacle_memory_test.cpp` и
     `drone_city_nav/tests/current_lidar_overlay_test.cpp`: при
     `sensor_hit_depth_m=0.0` должен отмечаться только endpoint, при
     `sensor_hit_depth_m=1.0` endpoint должен расширяться вглубь obstacle.

3. Сделать `PlanningGridBuilder` единственным runtime-местом inflation.

   Что сейчас:

   - `PlanningGridBuilder` уже делает финальный inflation.
   - `ObstacleMemoryGrid` дополнительно умеет строить `inflated_grid_`.
   - `obstacle_memory_node` публикует `/drone_city_nav/obstacle_memory_inflated_grid`.

   Что не так:

   - В системе появляется второй inflated artifact.
   - Его можно случайно подать обратно в planner как raw input.
   - Debug topic выглядит как usable planning topic.

   Как было бы лучше:

   - `obstacle_memory_node` публикует только raw memory.
   - Planner объединяет raw static, raw memory и raw current lidar.
   - `PlanningGridBuilder` один раз вызывает inflation.

   Что сделать:

   - В `drone_city_nav/include/drone_city_nav/obstacle_memory.hpp` удалить:
     - `rebuildInflation`;
     - `inflatedGrid`;
     - `countInflatedCells`;
     - поле `inflated_grid_`.
   - В `drone_city_nav/src/obstacle_memory.cpp:230` удалить реализацию
     `ObstacleMemoryGrid::rebuildInflation`.
   - В `drone_city_nav/src/obstacle_memory_node.cpp` удалить:
     - параметр `obstacle_memory_inflated_grid_topic`;
     - publisher `inflated_grid_pub_`;
     - вызов `memory_->rebuildInflation(inflation_radius_m_)`;
     - публикацию inflated memory grid.
   - В `drone_city_nav/src/obstacle_memory_node.cpp:459` оставить лог только
     по raw memory:

     ```text
     obstacle_memory: raw_occupied=<n> sensor_hit_depth_cells=<n>
     ```

   - В `drone_city_nav/tests/planning_grid_builder_test.cpp` добавить тест:
     если raw source содержит occupied cells, builder добавляет inflation один
     раз и итоговая карта имеет prohibited cells.
   - В `drone_city_nav/tests/planning_grid_builder_test.cpp` добавить тест:
     если source grid случайно содержит inflated cells, builder не должен
     считать их raw obstacles до финального rebuild. Это защищает от повторного
     inflation.

4. Развести raw, prohibited и debug topics.

   Что сейчас:

   - Planner читает raw memory topic:
     `drone_city_nav/src/planner_node.cpp:120`.
   - Planner публикует итоговую карту в `occupancy_grid_topic`:
     `drone_city_nav/src/planner_node.cpp:138`.
   - Debug/RViz местами используют название
     `/drone_city_nav/obstacle_memory_inflated_grid`.

   Что не так:

   - Название `occupancy_grid` не говорит, raw это карта или уже planning
     output.
   - `obstacle_memory_inflated_grid` выглядит как самостоятельный source.
   - Старый topic можно случайно использовать как input planner.

   Как было бы лучше:

   - Raw source topics:
     - `/drone_city_nav/obstacle_memory_grid`
     - static map source внутри planner
     - current lidar source внутри planner
   - Planning output:
     - `/drone_city_nav/prohibited_grid`
   - Debug visualization:
     - `/drone_city_nav/prohibited_obstacle_points`
     - `/drone_city_nav/lidar_current_points`
     - `/drone_city_nav/lidar_remembered_points`

   Что сделать:

   - В `drone_city_nav/include/drone_city_nav/planner_node_config.hpp`
     заменить `occupancy_grid_topic` на `prohibited_grid_topic`.
   - В `drone_city_nav/src/planner_node.cpp:138` публиковать итоговый grid в
     `prohibited_grid_topic`.
   - В `drone_city_nav/config/urban_mvp.yaml` заменить planner output topic на:

     ```yaml
     prohibited_grid_topic: /drone_city_nav/prohibited_grid
     ```

   - В `drone_city_nav/config/urban_mvp.yaml:175` заменить
     `lidar_debug_node.occupancy_grid_topic` на `/drone_city_nav/prohibited_grid`.
   - В `drone_city_nav/config/real_drone_template.yaml` сделать такие же
     изменения.
   - В `drone_city_nav/src/px4_offboard_node.cpp:187` переименовать параметр
     `occupancy_grid_topic` в `prohibited_grid_topic` и сменить default на
     `/drone_city_nav/prohibited_grid`.
   - В `drone_city_nav/src/px4_offboard_node.cpp:210` подписку оставить на
     final planner output, но переименовать local variables/logs/member-facing
     terminology с `occupancy_grid` на `prohibited_grid`.
   - В `drone_city_nav/src/px4_offboard_node.cpp:873` сохранить local clearance
     diagnostics на новом prohibited-grid topic. Диагностика должна продолжать
     использовать final planner grid, а не raw memory/debug grid.
   - В `drone_city_nav/config/urban_mvp.yaml:126` и
     `drone_city_nav/config/real_drone_template.yaml:126` заменить
     `px4_offboard_node.occupancy_grid_topic` на:

     ```yaml
     prohibited_grid_topic: /drone_city_nav/prohibited_grid
     ```

   - В `drone_city_nav/rviz/city_nav_debug.rviz` заменить display
     `Raw Obstacle Memory Grid`, если он смотрит на inflated topic, на понятное
     имя `Prohibited Grid`.
   - Удалить все config/docs references на
     `/drone_city_nav/obstacle_memory_inflated_grid`.
   - В `scripts/tests/test_topic_contract.py` добавить grep-contract test,
     который проверяет:
     - `px4_offboard_node` не объявляет `occupancy_grid_topic`;
     - `urban_mvp.yaml` и `real_drone_template.yaml` используют
       `px4_offboard_node.prohibited_grid_topic`;
     - `/drone_city_nav/occupancy_grid` не используется как final planner grid.

5. Защитить ROS conversions от raw/prohibited смешивания.

   Что сейчас:

   - `occupancyGridToRos()` может encode inflated cells как `80`.
   - `occupancyGridFromRos()` с threshold `65` может прочитать `80` как
     occupied.

   Что не так:

   - Debug/prohibited topic может стать raw input и создать двойной inflation.

   Как было бы лучше:

   - Raw parser принимает только raw occupancy semantics.
   - Prohibited publisher явно публикует final planning output.

   Что сделать:

   - В `drone_city_nav/include/drone_city_nav/ros_conversions.hpp` разделить
     конфиги:

     ```cpp
     struct RawOccupancyGridFromRosConfig {
       double occupied_threshold = 100.0;
       bool reject_inflated_values = true;
     };

     struct ProhibitedGridToRosConfig {
       bool include_inflation = true;
     };
     ```

   - В `drone_city_nav/src/ros_conversions.cpp` добавить raw parser, который
     не превращает `80` в occupied. Если встречается значение `1..99`, логика
     должна либо игнорировать его, либо возвращать diagnostic warning. Для
     текущего проекта предпочтительно warning plus ignore, чтобы debug topic не
     мог стать raw obstacle source.
   - В planner subscriber на obstacle memory использовать raw parser.
   - В planner publisher на prohibited grid использовать prohibited publisher.
   - В `drone_city_nav/tests/ros_conversions_test.cpp` добавить тесты:
     - raw `100` становится occupied;
     - raw `80` не становится occupied;
     - prohibited publisher пишет occupied `100`, inflated `80`.

6. Обновить lidar debug под новый contract.

   Что сейчас:

   - `lidar_debug_node` подписывается на topic с именем `occupancy_grid_topic`.
   - Он публикует current lidar, remembered points и inflated points.

   Что не так:

   - Название `inflated_obstacle_points` привязано к способу построения карты.
   - После единого inflation в planner debug node должен смотреть на final
     prohibited output, а не на memory inflated.

   Как было бы лучше:

   - `lidar_debug_node` получает `/drone_city_nav/prohibited_grid`.
   - Debug pointcloud называется `prohibited_obstacle_points`.
   - Внутри renderer можно всё ещё различать raw occupied cells и inflated
     cells по цвету.

   Что сделать:

   - В `drone_city_nav/src/lidar_debug_node.cpp:123` переименовать config field
     `occupancy_grid_topic` в `prohibited_grid_topic`.
   - В `drone_city_nav/src/lidar_debug_node.cpp:320` заменить publication label
     `inflated` на `prohibited`.
   - В `drone_city_nav/src/lidar_debug_node.cpp:504` оставить count
     `occupied_value_100` и `inflated_value_80` как debug detail, но summary
     должен называться `prohibited_grid`.
   - В `drone_city_nav/rviz/city_nav_debug.rviz` обновить display name на
     `Prohibited Safety Cells`.
   - В `drone_city_nav/tests/lidar_debug_renderer_test.cpp` и
     `drone_city_nav/tests/lidar_snapshot_writer_test.cpp` обновить expected
     labels/topic names.

7. Обновить rosbag/debug recording script.

   Что сейчас:

   - `scripts/record_debug_bag.sh:15` записывает
     `/drone_city_nav/obstacle_memory_inflated_grid`.
   - `scripts/record_debug_bag.sh:16` записывает
     `/drone_city_nav/occupancy_grid`.

   Что не так:

   - Debug bag сохраняет старый inflated memory topic.
   - Debug bag сохраняет старое ambiguous имя final planner grid.

   Как было бы лучше:

   - Bag содержит raw memory и final prohibited planner output.
   - Bag не содержит удалённый inflated memory topic.

   Что сделать:

   - В `scripts/record_debug_bag.sh` удалить
     `/drone_city_nav/obstacle_memory_inflated_grid`.
   - В `scripts/record_debug_bag.sh` заменить `/drone_city_nav/occupancy_grid`
     на `/drone_city_nav/prohibited_grid`.
   - В `README.md:125` и `docs/MVP_SIMULATION.md:410` обновить описание debug
     recording, чтобы оно называло raw/prohibited topics.
   - В `scripts/tests/test_topic_contract.py` проверить, что
     `record_debug_bag.sh` не содержит old topics и содержит
     `/drone_city_nav/prohibited_grid`.

8. Обновить документацию и конфиги.

   Что сейчас:

   - `docs/MVP_SIMULATION.md` упоминает
     `/drone_city_nav/obstacle_memory_inflated_grid`.
   - В config есть old topic names и old hit depth names.

   Что не так:

   - Документация закрепляет старую схему с несколькими inflated artifacts.

   Как было бы лучше:

   - Документация описывает один pipeline:

     ```text
     raw sources -> overlay -> single inflation -> prohibited grid -> A*
     ```

   Что сделать:

   - В `docs/MVP_SIMULATION.md:341` заменить старый RViz topic на
     `/drone_city_nav/prohibited_grid`.
   - В `docs/MVP_SIMULATION.md:354` удалить описание
     `/drone_city_nav/obstacle_memory_inflated_grid`.
   - В `docs/MVP_SIMULATION.md:535` переписать раздел про hit endpoints:
     `sensor_hit_depth_m` это sensor preprocessing, не inflation.
   - В `README.md` добавить короткое описание raw/prohibited/debug topic
     contract.
   - В `CONTRIBUTING.md` добавить правило: planner input topics must be raw;
     debug/prohibited topics must never be wired back into planner as raw input.

9. Добавить headless-friendly логи.

   Что сейчас:

   - Логи уже есть, но часть терминологии смешана.

   Что не так:

   - По headless логам сложно быстро понять, есть ли double inflation или topic
     mixup.

   Как было бы лучше:

   - Startup logs явно показывают source topics, output topic и inflation owner.
   - Runtime logs показывают raw source counts и final prohibited counts.

   Что сделать:

   - В `drone_city_nav/src/planner_node.cpp` при старте логировать:

     ```text
     planning_grid_contract: raw_sources=[static,memory,current_lidar] inflation_owner=planner output=/drone_city_nav/prohibited_grid
     ```

   - В `drone_city_nav/src/planning_grid_builder.cpp` в planning summary
     логировать:

     ```text
     raw_static=<n> raw_memory=<n> raw_current_lidar=<n> prohibited=<n> inflation_radius_m=<r>
     ```

   - В `drone_city_nav/src/obstacle_memory_node.cpp` логировать, что node
     publishes raw memory only.

10. Удалить старые references и проверить отсутствие accidental wiring.

   Что сейчас:

   - В repo есть references на `obstacle_memory_inflated_grid`.

   Что не так:

   - Даже если код исправлен, старый topic в docs/config/RViz создаёт риск
     возврата double inflation.

   Как было бы лучше:

   - Старый topic полностью отсутствует.

   Что сделать:

   - После изменений выполнить:

     ```bash
     rg "obstacle_memory_inflated|inflated_grid_topic|occupancy_grid_topic"
     ```

   - Дополнительно выполнить:

     ```bash
     rg "/drone_city_nav/occupancy_grid|occupancy_grid_topic" \
       drone_city_nav scripts docs README.md CONTRIBUTING.md
     ```

   - Старые `occupancy_grid_topic` references должны отсутствовать в planner,
     offboard, lidar debug, YAML configs и debug scripts.
   - Оставить `inflated` references только там, где речь о внутреннем состоянии
     `OccupancyGrid2D`, debug visualization или final prohibited grid rendering.

# Verification plan

1. Проверка formatting для C++ изменений:

   ```bash
   ./scripts/dev_shell.sh make format
   ```

2. Основная проверка качества:

   ```bash
   ./scripts/dev_shell.sh make quality
   ```

3. Script tests, если менялись shell/python scripts или script-facing config
   helpers:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

4. Targeted tests для изменённых C++ компонентов:

   ```bash
   ./scripts/dev_shell.sh make test
   ```

5. Smoke simulation с static map:

   ```bash
   SMOKE_DURATION_S=90 ./scripts/sim_headless.sh
   ```

6. Smoke simulation без static map:

   ```bash
   ENABLE_STATIC_MAP=false SMOKE_DURATION_S=90 ./scripts/sim_headless.sh
   ```

7. Log checks после headless runs:

   ```bash
   rg "planning_grid_contract|prohibited_grid|raw_memory|inflation_owner|mission_result" log
   rg "obstacle_memory_inflated_grid|hold_unsafe_path|double inflation" log
   ```

   Второй `rg` должен не находить runtime usage старого topic.

# Testing strategy

Категория 1: без рефакторинга.

1. `ros_conversions_test.cpp`:
   - raw parser не должен считать value `80` occupied;
   - prohibited publisher должен сохранять occupied `100` и inflated `80`.

2. `current_lidar_overlay_test.cpp`:
   - `sensor_hit_depth_m=0.0` даёт только endpoint;
   - `sensor_hit_depth_m=1.0` даёт endpoint плюс depth cells.

3. `obstacle_memory_test.cpp`:
   - memory stores raw cells only;
   - hit depth stats считаются как sensor preprocessing, не inflation.

4. `scripts/tests/test_topic_contract.py`:
   - запрещает `obstacle_memory_inflated_grid` в runtime configs/scripts/docs;
   - запрещает `occupancy_grid_topic` в planner/offboard/lidar_debug contracts;
   - требует `/drone_city_nav/prohibited_grid` в planner output, offboard input,
     lidar debug input и debug bag recording.

Категория 2: лёгкий рефакторинг.

1. `planning_grid_builder_test.cpp`:
   - static, memory и current lidar raw sources overlay-ятся до inflation;
   - final inflation выполняется один раз;
   - source inflated cells не превращаются во второй raw obstacle source.

2. `planner_node_config_test.cpp`:
   - новые topic/parameter names читаются из YAML;
   - old inflated memory topic больше не нужен;
   - `prohibited_grid_topic` имеет ожидаемый default.

3. Offboard config/topic contract:
   - `px4_offboard_node` default prohibited-grid topic равен
     `/drone_city_nav/prohibited_grid`;
   - local clearance diagnostics остаются подключены к final prohibited grid;
   - startup logs говорят `prohibited_grid`, а не `occupancy_grid`.

4. `lidar_debug_renderer_test.cpp`:
   - renderer корректно различает current lidar, raw/prohibited grid и
     inflated debug cells.

Категория 3: тяжёлый refactor / integration.

1. Headless static-map smoke:
   - planner стартует;
   - prohibited grid публикуется;
   - mission не падает на отсутствующем old topic.

2. Headless no-static smoke:
   - obstacle memory raw cells поступают в planner;
   - current lidar raw overlay участвует в builder;
   - prohibited grid появляется только из planner output.

3. Log assertions:
   - в логах есть `inflation_owner=planner`;
   - в логах нет `obstacle_memory_inflated_grid`;
   - в логах offboard subscriptions есть `prohibited_grid=`;
   - replan/path logs говорят `prohibited`, а не planning-facing `occupied`.

# Risks and tradeoffs

1. Удаление `/drone_city_nav/obstacle_memory_inflated_grid` сломает любые
   внешние launch/config/RViz файлы, которые всё ещё на него подписаны.

2. Переименование `occupancy_grid_topic` в `prohibited_grid_topic` требует
   обновить все configs и tests одновременно. Частичное изменение приведёт к
   пустой debug map или отсутствию path safety input.

3. Более строгий raw parser может игнорировать legacy values `1..99`. Это
   правильно для защиты от double inflation, но может сломать внешние источники,
   которые публикуют probabilistic occupancy values.

4. Уменьшение или переименование hit depth параметров может визуально изменить
   lidar memory contour. Это не должно менять safety radius, потому что safety
   radius должен задаваться только final inflation.

5. Если оставить compatibility aliases для старых topic names, будет проще
   мигрировать, но риск случайного wiring старого inflated topic обратно в
   planner останется. Для текущего проекта лучше сделать резкое удаление.

6. `occupied` и `inflated` нельзя полностью удалить из grid implementation,
   потому что debug/RViz должны показывать реальные obstacle cells и inflated
   safety cells разными цветами.

# Open questions

1. Блокирующих вопросов нет.

2. Единственный совместимый, но нежелательный вариант: временно оставить aliases
   для old topic/parameter names. В этом плане aliases не предлагаются, потому
   что цель именно убрать риск повторного inflation и случайного topic mixup.
