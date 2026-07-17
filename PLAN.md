# План реализации 3D-визуализации происхождения точек obstacle memory

## Context

Нужно дополнить существующую RViz-визуализацию obstacle memory вторым слоем,
который показывает реальное 3D-положение lidar endpoint, впервые переведшего
активную XY-ячейку памяти в состояние `occupied`. Существующий слой
`/drone_city_nav/raw_memory_obstacle_points` должен остаться без изменений: он
показывает центры активных 2D-ячеек на `z=0.05 м` и тем самым отражает именно то,
что видит двумерный planner.

Новый слой должен публиковаться на
`/drone_city_nav/raw_memory_obstacle_points_3d`, содержать ровно по одной точке
на каждый active provenance record с валидным `occupancy_trigger` XYZ, не
показывать удалённую историю и не выполнять визуальную инфляцию. Это
односторонняя диагностика: новый topic не должен становиться входом planner,
replanning, offboard control или mission monitor.

Отдельный feature-enable параметр не нужен: публикация небольшого debug cloud
будет следовать существующему debug cadence obstacle memory, а RViz display
будет включён по умолчанию. Если при реализации всё же появится enable-параметр,
его default и значение в штатном YAML должны быть `true`.

## Investigation context

`INVESTIGATION.md` в workspace отсутствует. План составлен по текущему коду и
контрактам репозитория.

Текущее состояние подтверждает пригодность уже существующих данных:

- `MemoryCellProvenance` хранит `occupancy_trigger`, `last_hit`, диапазон Z и
  счётчик принятых попаданий
  (`drone_city_nav/include/drone_city_nav/obstacle_memory.hpp:53-66`);
- `LidarBeamProjection` содержит `endpoint_map_m` и `endpoint_xyz_valid`
  (`drone_city_nav/include/drone_city_nav/lidar_projection.hpp:48-65`);
- `ObstacleMemoryGrid::activeProvenance()` содержит только provenance активных
  occupied-ячеек, а `reset()` и переход ячейки из occupied уже удаляют
  неактуальную provenance (`drone_city_nav/src/obstacle_memory.cpp:104-140,
  294-305, 358-365`);
- `obstacle_memory_node` владеет grid и provenance одновременно и уже имеет
  отдельный debug cadence через `obstacle_memory_debug_publish_period_s`
  (`drone_city_nav/src/obstacle_memory_node.cpp:692-735`);
- существующий `buildLidarDebugPointCloud()` задаёт стандартный XYZ/FLOAT32
  layout и применяет обязательный `gazeboAlignedRvizZ()`
  (`drone_city_nav/src/lidar_debug_pointclouds.cpp:62-100`);
- существующий наземный cloud строится отдельно из OccupancyGrid в
  `lidar_debug_node` и поэтому может быть сохранён без поведенческих изменений
  (`drone_city_nav/src/lidar_debug_node_lifecycle.cpp:77-82`).

## Detected stack/profiles

- Основной стек: C++20, ROS 2 Jazzy, `ament_cmake`/`colcon`, `rclcpp`,
  `sensor_msgs::msg::PointCloud2`, RViz2, Python `unittest` для script-contract
  тестов.
- Применены обязательные project profiles:
  - `generic.md`, потому что он обязателен для любого workspace;
  - `cpp.md`, потому что пакет содержит `CMakeLists.txt`, `.cpp/.hpp`, C++ gtest и
    документированный colcon workflow.
- Rust profile не применяется: затрагиваемая реализация и build/test workflow
  проекта не используют Rust.
- Notion не читался: prompt не содержит Notion task/ID, а policy запуска
  `optional`.
- GitLab не читался: prompt не содержит MR/GitLab context.

## Repo-approved commands found

Репозиторий требует только container workflow (`AGENTS.md`, `README.md`,
`CONTRIBUTING.md`):

- `./scripts/build.sh` — containerized `make build`/`colcon build`;
- `./scripts/test.sh` — containerized build и полный CTest package scope;
- `./scripts/dev_shell.sh make test-scripts` — Python script-contract tests;
- `./scripts/dev_shell.sh make format` — форматирование только изменённых C++
  файлов;
- `./scripts/dev_shell.sh make quality` — обязательный non-mutating quality gate
  перед commit;
- внутри container для scoped regression:
  `ctest --test-dir build/drone_city_nav -R lidar_debug_pointclouds_test --output-on-failure`.

`./scripts/sim_gui.sh` намеренно не входит в verification этой задачи: prompt
прямо запрещает GUI-прогон после реализации, а корректность cloud layout и
wiring покрывается автоматическими тестами. Ad-hoc top-level CMake команды не
используются.

## Affected components

- `drone_city_nav/include/drone_city_nav/lidar_debug_pointclouds.hpp` и
  `drone_city_nav/src/lidar_debug_pointclouds.cpp`: сбор trigger XYZ и построение
  переменного по Z `PointCloud2`.
- `drone_city_nav/src/obstacle_memory_node.cpp`: параметр topic, publisher и
  публикация нового cloud из `activeProvenance()` в существующем debug cadence.
- `drone_city_nav/config/urban_mvp.yaml`: штатное имя нового topic в секции
  `obstacle_memory_node`.
- `drone_city_nav/rviz/city_nav_debug.rviz` и
  `drone_city_nav/rviz/city_nav_debug_top_down.rviz`: отдельный включённый слой
  `Raw Memory Hit Origins 3D`.
- `scripts/record_debug_bag.sh`: запись нового диагностического topic.
- `drone_city_nav/tests/lidar_debug_pointclouds_test.cpp` и
  `scripts/tests/test_topic_contract.py`: data-contract и wiring regression
  coverage.
- `docs/rviz.md`, `docs/obstacle_mapping.md`, `docs/configuration.md` и при
  необходимости `docs/diagnostics.md`: семантика двух memory layers и
  diagnostics-only гарантия.
- `drone_city_nav/CMakeLists.txt` менять не потребуется, если новые helpers
  останутся в уже подключённом `drone_city_nav_ros_adapters` и существующем
  `lidar_debug_pointclouds_test`. Изменить его следует только если реализация
  обоснованно вынесет cloud builder в новый translation unit.

## Implementation steps

1. **Добавить детерминированное построение 3D memory-trigger cloud.**
   В `drone_city_nav/include/drone_city_nav/lidar_debug_pointclouds.hpp` и
   `drone_city_nav/src/lidar_debug_pointclouds.cpp` добавить helper, который
   принимает `ObstacleMemoryGrid::activeProvenance()` или эквивалентный
   `std::unordered_map<std::size_t, MemoryCellProvenance>`, сортирует записи по
   linear cell index для стабильного output и выбирает точный
   `record.occupancy_trigger.beam.projection.endpoint_map_m`. Запись включается
   только при `endpoint_xyz_valid=true` и finite X/Y/Z; `last_hit`, cell center и
   `min/max Z` не подменяют trigger. Добавить overload/build helper для
   `std::span<const Point3>`, сохраняющий существующий XYZ/FLOAT32 layout и
   применяющий `gazeboAlignedRvizZ(point.z)` отдельно к каждой точке. Существующий
   `Point2 + fixed z` API и его поведение оставить совместимыми.

   Ожидаемый контракт:

   ```cpp
   for (index : sorted(active_provenance.keys())) {
     const auto& projection = active_provenance.at(index)
                                  .occupancy_trigger.beam.projection;
     if (projection.endpoint_xyz_valid && finite(projection.endpoint_map_m)) {
       points.push_back(projection.endpoint_map_m);
     }
   }
   // RViz payload stores (x, y, gazeboAlignedRvizZ(z)).
   ```

   Материализуемый результат: один неинфлированный 3D point на каждую активную
   ячейку с валидным trigger XYZ и пустой cloud при отсутствии таких записей.

2. **Опубликовать новый topic непосредственно из authoritative owner памяти.**
   В `drone_city_nav/src/obstacle_memory_node.cpp` добавить
   `sensor_msgs::msg::PointCloud2` publisher с reliable + transient-local QoS и
   параметром `raw_memory_3d_pointcloud_topic`, default
   `/drone_city_nav/raw_memory_obstacle_points_3d`. Создать publisher рядом с
   `raw_grid_pub_`/`provenance_pub_` (`obstacle_memory_node.cpp:292-303`) и
   публиковать cloud в ветке `publish_debug` рядом с raw grid/provenance
   (`obstacle_memory_node.cpp:726-735`), используя тот же stamp/frame_id и
   текущий `memory_->activeProvenance()`.

   Старый `/drone_city_nav/raw_memory_obstacle_points` продолжит строиться
   `lidar_debug_node` из OccupancyGrid на фиксированной высоте и не будет
   переподключён к новому publisher. Новый publisher не должен иметь
   subscribers/call sites в planner или control. Отдельный enable flag не
   добавлять; если это окажется необходимо по эксплуатационной причине, default
   должен быть `true`, а выключение должно влиять только на публикацию.

   Материализуемый результат: новый transient cloud обновляется примерно раз в
   секунду вместе с debug grid/provenance, удалённые из active provenance ячейки
   исчезают при следующей публикации, runtime scoring и replanning не меняются.

3. **Подключить topic к штатной визуализации и debug artifacts.**
   В `drone_city_nav/config/urban_mvp.yaml` добавить имя topic в секцию
   `obstacle_memory_node`. В обеих конфигурациях RViz добавить включённый по
   умолчанию `PointCloud2` display `Raw Memory Hit Origins 3D` с отдельным
   контрастным цветом, `Style: Spheres`, малым фиксированным размером и без
   decay/инфляции; существующий `Raw Memory Cells` оставить побитово
   неизменённым. В `scripts/record_debug_bag.sh` добавить новый topic, чтобы
   происхождение memory blocker можно было исследовать после run.

   Материализуемый результат: пользователь независимо включает 2D planning view
   и 3D provenance view, а default RViz показывает оба. В публикации используется
   существующая Z-компенсация; X/Y дополнительно не переставляются, поскольку это
   уже делает намеренный `gazebo_aligned_map_tf`.

4. **Зафиксировать автоматические контракты данных и wiring.**
   Расширить `drone_city_nav/tests/lidar_debug_pointclouds_test.cpp` тестами:

   - happy path: два active provenance record дают две точки с точными trigger
     X/Y и инвертированным для RViz Z;
   - trigger-vs-last: изменение `last_hit` не меняет публикуемую точку;
   - negative path `endpoint_xyz_valid=false`: trigger пропускается, даже если
     его X/Y/Z finite;
   - отдельные negative cases с non-finite X, Y и Z: каждый такой trigger
     пропускается при `endpoint_xyz_valid=true`, не превращаясь в точку на нуле;
   - edge case: пустая provenance даёт корректный `PointCloud2` ширины 0;
   - one-record-per-cell: `min/max Z` и `accepted_hit_count` не размножают точки.

   Обновить `scripts/tests/test_topic_contract.py`, чтобы он проверял не только
   default topic в YAML, enabled-state display в обеих RViz-конфигурациях,
   запись topic в debug bag и отсутствие подключения нового topic к planner/
   offboard inputs, но и положительный source-level wiring contract в
   `obstacle_memory_node.cpp`:

   - parameter declaration содержит default
     `/drone_city_nav/raw_memory_obstacle_points_3d`;
   - publisher создаётся с reliable + transient-local QoS;
   - единственный publish call нового cloud находится внутри существующего
     `if (publish_debug)` в `publishMemorySnapshot()`, а не в scan callback или
     безусловной snapshot-ветке;
   - builder получает `memory_->activeProvenance()`, stamp текущего snapshot и
     `frame_id_`, поэтому cloud согласован с grid/provenance publication.

   Тест должен извлекать тело `publishMemorySnapshot()` и соответствующий
   `publish_debug` block (brace-aware helper либо эквивалентная ограниченная
   проверка), чтобы падать при удалении publisher/publish call или переносе
   публикации на каждый scan. Полноценный node integration test не добавлять,
   если для него потребуется новый production seam: существующий source-contract
   pattern в этом script test закрывает wiring без несоразмерного рефакторинга.
   При добавлении отдельного source file зарегистрировать его и существующий test
   target в `drone_city_nav/CMakeLists.txt`.

   Материализуемый результат: ошибки выбора `last_hit`, повторная Z-инверсия,
   потеря publisher/QoS/debug-cadence/stamp-frame wiring и случайное включение
   cloud в control path ловятся без GUI.

5. **Обновить пользовательскую и диагностическую документацию.**
   В `docs/rviz.md` явно разделить `Raw Memory Cells 2D` и
   `Raw Memory Hit Origins 3D`, объяснить, что первый показывает cell center на
   земле, второй — исходный `occupancy_trigger.endpoint_map_m`. В
   `docs/obstacle_mapping.md` и `docs/diagnostics.md` указать, что 3D cloud
   строится только из active sparse provenance, не хранит историю, не содержит
   inflation и не участвует в scoring/planning. В `docs/configuration.md`
   документировать topic parameter и его штатное default-значение.

   Материализуемый результат: по документации нельзя спутать 3D provenance cloud
   с `/remembered_lidar_points`, prohibited grid или полноценной 3D obstacle
   memory.

6. **Выполнить repo-approved verification и создать отдельный локальный
   commit.** После реализации выполнить `./scripts/dev_shell.sh make format` для
   изменённых C++ файлов, scoped `lidar_debug_pointclouds_test`, полный
   `./scripts/test.sh`, `./scripts/dev_shell.sh make test-scripts` и обязательный
   `./scripts/dev_shell.sh make quality`. Проверить `git diff` на отсутствие
   изменений control/planner semantics и создать новый локальный commit без
   amend/rebase/push. GUI simulation не запускать.

   Материализуемый результат: форматированный, протестированный и отдельно
   коммитнутый diagnostics-only change set с явно записанными skipped/failing
   checks, если они возникнут.

## Verification plan

1. В container выполнить scoped regression:

   ```bash
   ./scripts/dev_shell.sh ctest --test-dir build/drone_city_nav \
     -R lidar_debug_pointclouds_test --output-on-failure
   ```

   Если test binary ещё не собран после изменения, сначала выполнить
   `./scripts/build.sh`.

2. Проверить package build и полный CTest scope:

   ```bash
   ./scripts/test.sh
   ```

3. Проверить Python topic/config/RViz/bag contracts:

   ```bash
   ./scripts/dev_shell.sh make test-scripts
   ```

   В частности, `test_topic_contract.py` должен доказать положительный wiring
   publisher-а в `obstacle_memory_node`: default topic, reliable +
   transient-local QoS, единственный publish внутри `publish_debug`, общий
   snapshot stamp и `frame_id_`. Это обязательный acceptance signal, поскольку
   GUI-прогон запрещён и проверки только YAML/RViz/bag не доказывают runtime
   публикацию.

4. Перед commit выполнить обязательные проверки:

   ```bash
   ./scripts/dev_shell.sh make format
   ./scripts/dev_shell.sh make quality
   ```

5. Не запускать `./scripts/sim_gui.sh`. Автоматически проверить cloud layout,
   точный trigger source, invalid-point filtering, Z-компенсацию и wiring.
   Headless simulation не обязательна для acceptance этой diagnostics-only
   фичи: она не умеет визуально оценить RViz и не добавляет покрытия поверх
   PointCloud/unit/topic-contract тестов. Если исполнитель всё же запускает
   headless smoke как дополнительную проверку, это не заменяет перечисленные
   тесты и не должно сопровождаться GUI.

## Testing strategy

### Категория 1: без рефакторинга

- Дополнить существующий `lidar_debug_pointclouds_test` новым Point3/provenance
  контрактом, включая независимые случаи invalid flag и non-finite X/Y/Z.
- Дополнить `test_topic_contract.py` проверками YAML, RViz, bag и положительного
  source wiring publisher/QoS/debug-cadence/stamp-frame в memory node.
- Это основной рекомендуемый объём: существующий `Point2` API и node boundaries
  сохраняются.

### Категория 2: лёгкий рефакторинг

- Реализовать общий внутренний XYZ cloud writer и делегировать ему текущий
  fixed-Z overload, чтобы layout и `gazeboAlignedRvizZ()` не дублировались.
- Добавить небольшой pure helper для сортировки/фильтрации active provenance.
- Рефакторинг допустим только при сохранении текущих Point2 tests и public API.

### Категория 3: тяжёлый рефакторинг

- Перенос всех debug pointcloud publishers в отдельный node, подписка на atomic
  snapshot или изменение obstacle-memory transport не требуются и не
  рекомендуются для этой задачи.
- Полноценная 3D voxel memory, хранение всех historical hits и altitude-aware
  planning projection находятся вне scope.

## Risks and tradeoffs

- **Семантическая неоднозначность “точка памяти”.** Active memory является одной
  XY-ячейкой с несколькими возможными hits. Выбор `occupancy_trigger` намеренный:
  он отвечает на вопрос, какое наблюдение создало occupied cell. `last_hit` и
  Z-range остаются в provenance/logs, но не визуализируются отдельными точками.
- **Неполный cloud при старых/invalid records.** Records без валидного finite XYZ
  будут пропущены, поэтому количество 3D points может быть меньше числа 2D
  occupied cells. Это честнее, чем рисовать выдуманный Z=0; разницу следует
  документировать и при необходимости логировать счётчиком.
- **RViz frame convention.** Пропуск `gazeboAlignedRvizZ()` отзеркалит cloud под
  картой, а повторная перестановка X/Y сместит его относительно зданий. Unit test
  должен защищать Z, script contract — намеренный TF.
- **DDS/CPU overhead.** Один XYZ/FLOAT32 point занимает 12 байт payload; при
  тысячах active cells и частоте около 1 Гц нагрузка существенно меньше уже
  публикуемого provenance/snapshot. Публикация должна оставаться в debug cadence,
  а не происходить на каждый lidar beam.
- **Размер `obstacle_memory_node.cpp`.** Файл сейчас содержит 907 строк при
  repository contract 1000 строк. Логику фильтрации/сериализации cloud нельзя
  разворачивать внутри node; она должна остаться в тестируемом helper, иначе
  будет нарушен size contract и ухудшится сопровождаемость.
- **Случайное влияние на runtime.** Новый topic не должен использоваться как
  planner input. Проверяется code review и script-contract тестом; существующий
  atomic `/obstacle_memory_snapshot` остаётся единственным memory input planner.
- **Bag/storage growth.** Запись ещё одного cloud увеличит debug bag, но cloud
  публикуется редко и содержит только активные trigger points. Это приемлемая
  цена за воспроизводимую 3D-диагностику.

## Open questions

1. **Нужен ли enable-параметр для нового publisher?**
   Recommended decision: не добавлять. Topic diagnostics-only, cadence уже
   ограничен и требование ожидает фичу включённой по умолчанию. Если измеренная
   нагрузка окажется значимой, добавить параметр позднее с default `true` и без
   влияния на memory integration.

2. **Показывать trigger endpoint или центр XY-ячейки с trigger Z?**
   Recommended decision: точный `occupancy_trigger.endpoint_map_m`. Он показывает
   реальное спроецированное наблюдение; наземный слой уже показывает дискретный
   cell center. Для подтверждения достаточно существующего provenance contract
   и unit test с endpoint, не совпадающим с центром ячейки.

3. **Нужно ли визуализировать `last_hit`, `min_z` и `max_z` дополнительными
   точками/линиями?** Recommended decision: нет. Это нарушило бы требование “одна
   маленькая сфера на active provenance record”, создало бы визуальную инфляцию и
   затруднило бы связь с occupancy trigger. Эти данные остаются в topic/logs.

4. **Нужно ли подписывать `lidar_debug_node` на atomic memory snapshot?**
   Recommended decision: нет. `obstacle_memory_node` уже владеет согласованными
   grid/provenance и может собрать cloud без дополнительной сериализации,
   DDS-копии и callback ordering. Snapshot transport остаётся runtime контрактом
   planner, а новый cloud — производным debug output.

5. **Нужен ли GUI acceptance run?** Recommended decision: нет и в этой задаче он
   запрещён prompt. Корректность данных проверяется unit/script tests; визуальное
   предпочтение цвета не является основанием нарушать запрет. После merge
   пользователь сможет оценить слой в обычном GUI run отдельно.
