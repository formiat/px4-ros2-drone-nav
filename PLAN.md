# План: Stage 4, Vertical Profile Builder для 3D passage traversal

## Context

Нужно добавить первый реально меняющий траекторию 3D-этап: поверх уже построенной 2D executable trajectory построить гладкий профиль высоты `z(s)` для пролёта через заранее известные passages/openings. На этом этапе не добавляем распознавание проёмов сенсорами, не меняем XY-геометрию, не добавляем отдельный вес "лететь через passage дешевле/дороже". Старый 2D pipeline остаётся базой:

```text
A* -> corridor -> trajectory optimizer -> turn smoothing -> shape cleanup
```

Новый этап должен встать после финальной XY-геометрии и до speed profile:

```text
final XY samples -> vertical profile z(s) -> speed profile with vertical caps
```

Целевой результат: если trajectory пересекает known passage footprint и matched opening найден, samples получают плавный профиль `cruise altitude -> gate altitude -> cruise altitude`, speed profile учитывает вертикальные ограничения, RViz/debug dumps показывают настоящую 3D-траекторию.

## Investigation Context

`INVESTIGATION.md` в репозитории отсутствует. Текущий `PLAN.md` также отсутствовал до этого планирования.

Что уже есть в коде:

- [drone_city_nav/include/drone_city_nav/trajectory.hpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/include/drone_city_nav/trajectory.hpp): `TrajectoryPointSample` уже содержит `z_m`; есть `assignTrajectorySampleAltitude()`, `trajectorySampleAltitudeAtS()`, `trajectoryAltitudeStats()`.
- [drone_city_nav/src/trajectory_planner.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/trajectory_planner.cpp:70): `finalizeResult()` сейчас принудительно выставляет всем samples `config.default_altitude_m`. Это место нужно заменить на осознанный vertical-profile финализатор.
- [drone_city_nav/include/drone_city_nav/known_passage_map.hpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/include/drone_city_nav/known_passage_map.hpp): passages уже описаны как 3D-структуры/openings с `center`, `normal_xy`, `width_m`, `height_m`, `depth_m`, `min_z_m`, `max_z_m`, `approach_distance_m`, `exit_distance_m`.
- [drone_city_nav/src/known_passage_validation.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/known_passage_validation.cpp:98): уже есть математика clipping trajectory segment к footprint и opening volume, но она спрятана внутри `.cpp`.
- [drone_city_nav/src/trajectory_speed_planner.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/trajectory_speed_planner.cpp:118): speed profile сейчас ограничивает скорость по XY-curvature/goal; вертикальных caps нет.
- [drone_city_nav/src/offboard_trajectory_state.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/offboard_trajectory_state.cpp:52): offboard уже сохраняет высоту из `nav_msgs/Path` в `TrajectoryPointSample::z_m`.
- [drone_city_nav/src/px4_offboard_node_control.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/px4_offboard_node_control.cpp:272): vertical velocity setpoint сейчас держит `cruise_altitude_m_`, а не `trajectory z(s)`. Это нужно изменить, иначе 3D path будет только визуальным.
- [drone_city_nav/src/trajectory_debug_markers.cpp](/home/formi/Documents/CppProjects/drone-gazebo/drone_city_nav/src/trajectory_debug_markers.cpp:91): RViz speed/curvature markers уже рисуются на `sample.z_m`, но отдельного altitude/vertical-cap слоя нет.

## Detected Stack/Profiles

- Профили планирования: `generic`, `cpp`.
- Стек: ROS 2 workspace, C++20, ament CMake, colcon.
- Основной пакет: `drone_city_nav`.
- Сборка/тесты выполняются через container workflow и репозиторные scripts/Makefile.

## Repo-Approved Commands Found

- `./scripts/build.sh`
- `./scripts/test.sh`
- `./scripts/dev_shell.sh`
- внутри dev shell: `make format`, `make quality`, `make test`, `make test-scripts`
- scoped CTest после сборки: `ctest --test-dir build/drone_city_nav --output-on-failure`

## Affected Components

- Trajectory data/model: `trajectory.hpp/cpp`.
- Known passage geometry/matching: `known_passage_validation.hpp/cpp`, новый shared matcher.
- Planner integration: `trajectory_planner.hpp/cpp`, `planner_node_config.cpp`, `urban_mvp.yaml`.
- Speed profile: `trajectory_speed_planner.hpp/cpp`, speed diagnostics.
- Offboard runtime altitude following: `px4_offboard_node_control.cpp`, `px4_offboard_node_telemetry.cpp`, node state/diagnostics fields.
- Diagnostics/dumps: `trajectory_diagnostics_io_*`, `final_trajectory_debug_io.cpp`, `offboard_blackbox.hpp/cpp`, blackbox/offboard trajectory stats.
- RViz markers: `trajectory_debug_markers.cpp`.
- Tests: `known_passage_validation_test.cpp`, новый `trajectory_vertical_profile_test.cpp`, `trajectory_speed_planner_test.cpp`, `trajectory_planner_test.cpp`, `offboard_velocity_follower`/node-state tests, diagnostics roundtrip/json/csv tests, config tests.

## Implementation Steps

1. Добавить модуль вертикального профиля.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/trajectory_vertical_profile.hpp`
   - `drone_city_nav/src/trajectory_vertical_profile.cpp`
   - `drone_city_nav/CMakeLists.txt`

   Материализованный результат:
   - типы `VerticalProfileConfig`, `VerticalProfileStats`, `VerticalProfilePassageDiagnostic`, `VerticalProfileResult`;
   - функция:

   ```cpp
   [[nodiscard]] VerticalProfileResult applyVerticalProfile(
       std::span<TrajectoryPointSample> samples,
       const KnownPassageMap* map,
       const KnownPassageValidationConfig& validation_config,
       const VerticalProfileConfig& config,
       double cruise_altitude_m);
   ```

   Базовые параметры:
   - `enabled`;
   - `gate_clearance_margin_m`;
   - `max_vertical_speed_mps`;
   - `max_vertical_accel_mps2`;
   - `max_vertical_jerk_mps3`;
   - `max_climb_angle_deg`;
   - `min_transition_distance_m`;
   - `max_transition_distance_m`;
   - `max_diagnostics`.

   Начальные дефолты для обсуждения в коде/config: `max_vertical_speed_mps=2.5`, `max_vertical_accel_mps2=2.0`, `max_vertical_jerk_mps3=6.0`, `max_climb_angle_deg=12.0`, `gate_clearance_margin_m=0.5`.

2. Вынести reusable known-passage matcher из validation.

   Файлы:
   - `drone_city_nav/include/drone_city_nav/known_passage_matching.hpp`
   - `drone_city_nav/src/known_passage_matching.cpp`
   - обновить `known_passage_validation.cpp`.

   Сейчас функции `findFootprintSpans()`, `clipSampleSegmentToFootprint()`, `findBestOpeningMatch()` локальные в `known_passage_validation.cpp`. Их нужно вынести в shared API, чтобы validation и vertical profile builder не расходились.

   Новый тип:

   ```cpp
   struct KnownPassageTraversalMatch {
     std::string structure_id;
     std::string opening_id;
     PassageOpening opening;
     double entry_s_m;
     double exit_s_m;
     double overlap_m;
     double clearance_m;
     bool valid;
     KnownPassageValidationReason reason;
   };
   ```

   Новый API:

   ```cpp
   [[nodiscard]] std::vector<KnownPassageTraversalMatch>
   findKnownPassageTraversalMatches(
       std::span<const TrajectoryPointSample> samples,
       const KnownPassageMap* map,
       const KnownPassageValidationConfig& config);
   ```

   `validateKnownPassageTraversal()` после этого строит summary из этих matches. Поведение validation должно остаться тем же; меняется ownership математики.

3. Построить `z(s)` для каждого matched opening.

   Файл: `trajectory_vertical_profile.cpp`.

   Алгоритм:

   ```text
   for each matched opening:
     gate_z = clamp(opening.center.z,
                    opening.min_z_m + margin,
                    opening.max_z_m - margin)

     gate_start_s = match.entry_s_m
     gate_end_s   = match.exit_s_m

     approach_start_s = gate_start_s - effective_approach_distance
     exit_end_s       = gate_end_s + effective_exit_distance

     z(s):
       cruise_z before approach_start_s
       smoothstep5(cruise_z -> gate_z) on approach
       gate_z inside opening span
       smoothstep5(gate_z -> cruise_z) on exit
       cruise_z after exit_end_s
   ```

   Использовать quintic smootherstep:

   ```cpp
   h(t) = 6*t^5 - 15*t^4 + 10*t^3
   ```

   Причина: нулевая первая и вторая производная на концах, меньше вертикальных рывков на стыках, чем linear/cubic.

   Если несколько profile windows пересекаются:
   - сначала сортировать по `entry_s_m`;
   - near windows объединять/пересчитывать как последовательные altitude waypoints;
   - если требования конфликтуют физически, `VerticalProfileResult.valid=false`, `reason="overlapping_infeasible_windows"`.

4. Проверить вертикальные constraints на самом профиле.

   Файл: `trajectory_vertical_profile.cpp`.

   После назначения `z_m` вычислить finite differences по `s`:

   ```text
   dz_ds
   d2z_ds2
   d3z_ds3
   max_climb_angle = atan(abs(dz_ds))
   ```

   Инварианты:
   - endpoints остаются на `cruise_altitude_m`;
   - `z_m` finite для всех samples;
   - inside opening span: `opening.min_z_m + margin <= z <= opening.max_z_m - margin`;
   - `atan(abs(dz_ds)) <= max_climb_angle`;
   - если профиль нужен для matched passage, но constraints невыполнимы, не публиковать trajectory как valid.

   В stats сохранить:
   - `vertical_profile_enabled`;
   - `vertical_profile_active`;
   - `vertical_profile_passages_matched`;
   - `vertical_profile_passages_profiled`;
   - `vertical_profile_min_z_m/max_z_m`;
   - `vertical_profile_max_abs_dz_ds`;
   - `vertical_profile_max_abs_d2z_ds2`;
   - `vertical_profile_max_abs_d3z_ds3`;
   - `vertical_profile_infeasible_count`;
   - diagnostic entries with `structure_id`, `opening_id`, `entry_s_m`, `exit_s_m`, `gate_z_m`, `approach_start_s_m`, `exit_end_s_m`, `reason`.

5. Интегрировать vertical profile в `trajectory_planner`.

   Файлы:
   - `trajectory_planner.hpp`
   - `trajectory_planner.cpp`
   - `planner_node_config.hpp/cpp`
   - `planner_node_trajectory_publication.cpp`

   Конкретные точки:
   - `TrajectoryPlannerConfig` добавить `VerticalProfileConfig vertical_profile{};`.
   - `TrajectoryPlannerInput` добавить `const KnownPassageMap* known_passage_map{nullptr};` или передать map через config/node call site. Предпочтительно через input, потому что map является runtime artifact, а не static config.
   - В `planBaselineTrajectory()` и `planOptimizedTrajectory()` после финальной XY geometry и до `buildTrajectorySpeedProfile()` вызвать `applyVerticalProfile()`.
   - `finalizeResult()` больше не должен безусловно делать `assignTrajectorySampleAltitude(result.samples, config.default_altitude_m)`. Новая логика:

   ```cpp
   if (!result.stats.vertical_profile.applied) {
     assignTrajectorySampleAltitude(result.samples, config.default_altitude_m);
   }
   computeCurvatureStats(...);
   computeVerticalProfileStats(...);
   ```

   - В `planner_node_trajectory_publication.cpp` убрать дублирующее позднее решение, которое валидирует known passages уже после speed profile, и оставить validation summary из planner result. Если отдельная публикационная validation остаётся как safety check, она должна использовать уже назначенный `z_m`.

6. Добавить vertical speed caps в speed profile.

   Файлы:
   - `trajectory_speed_planner.hpp`
   - `trajectory_speed_planner.cpp`
   - `velocity_control_config.hpp/cpp`

   Изменения:
   - `SpeedConstraintType` расширить:

   ```cpp
   enum class SpeedConstraintType {
     kNone,
     kArc,
     kVerticalProfile,
     kGoal,
   };
   ```

   - В `TrajectorySpeedSample` добавить поля диагностики:
     - `vertical_speed_limit_mps`;
     - `vertical_slope_dz_ds`;
     - `vertical_accel_limit_mps`;
     - `vertical_jerk_limit_mps`.

   - В `geometricSpeedSampleFromPointSample()` брать соседние samples или precomputed vertical derivatives и считать cap:

   ```text
   v_z cap:     v <= max_vertical_speed / abs(dz_ds)
   a_z cap:     v <= sqrt(max_vertical_accel / abs(d2z_ds2))
   jerk_z cap:  v <= cbrt(max_vertical_jerk / abs(d3z_ds3))
   angle check: abs(dz_ds) <= tan(max_climb_angle)
   ```

   - Итоговый geometric limit:

   ```cpp
   geometric_limit = min(xy_curvature_limit, vertical_profile_limit, cruise_speed);
   reason = source that produced lowest finite limit;
   ```

   - `topSpeedProfileConstraints()` должен показывать `source=vertical_profile`, если вертикальный cap стал главным.

7. Обновить config и fingerprints.

   Файлы:
   - `velocity_control_config.hpp/cpp`
   - `planner_node_config.cpp`
   - `px4_offboard_node_config.cpp`
   - `config/urban_mvp.yaml`
   - config tests.

   Параметры vertical speed caps влияют на speed profile construction, значит добавить их в `speedProfileConstructionConfigFingerprint()`:

   ```text
   vertical_profile_max_vertical_speed_mps
   vertical_profile_max_vertical_accel_mps2
   vertical_profile_max_vertical_jerk_mps3
   vertical_profile_max_climb_angle_deg/rad
   ```

   Runtime vertical following параметры (`altitude_hold_kp_`, `max_vertical_speed_mps_`, если они останутся node-level) должны быть явно отделены от construction fingerprint. Если один и тот же `max_vertical_speed_mps` используется и для speed cap, и для PX4 setpoint clamp, это нужно назвать явно или разделить:

   - `vertical_profile_max_vertical_speed_mps` для planning/speed profile;
   - `altitude_setpoint_max_vertical_speed_mps` для runtime vertical controller.

8. Переключить offboard vertical target на trajectory altitude.

   Файлы:
   - `drone_city_nav/src/px4_offboard_node_control.cpp`
   - `drone_city_nav/src/px4_offboard_node_telemetry.cpp`
   - node header/state fields, если новые `last_*` поля потребуются для telemetry/blackbox.

   Текущий якорь: `verticalVelocitySetpointNed()` держит `cruise_altitude_m_`.

   Новый контракт:

   ```cpp
   double targetAltitudeForCurrentTrajectory() const {
     if (last_velocity_plan_valid_ && finalTrajectoryReady()) {
       const double z = trajectorySampleAltitudeAtS(
           final_trajectory_samples_, last_velocity_plan_.trajectory_s_m);
       if (std::isfinite(z)) {
         return z;
       }
     }
     return cruise_altitude_m_;
   }
   ```

   Затем:

   ```cpp
   last_altitude_error_m_ = target_altitude_m - current_altitude_m_;
   vz_ned = -clamp(last_altitude_error_m_ * altitude_hold_kp_, ...);
   ```

   Диагностика:
   - `target_altitude_m`;
   - `trajectory_altitude_target_valid`;
   - `altitude_error_m`;
   - `vertical_velocity_setpoint_mps`.

   Эти значения должны быть сохранены как runtime state, доступный обычной headless telemetry, а не остаться локальными переменными внутри control path. Конкретный результат:
   - `logTelemetry()` в `px4_offboard_node_telemetry.cpp` печатает `altitude[target=... trajectory_target_valid=... error=... vz_setpoint=...]`;
   - `writeFlightBlackbox()` передаёт те же значения в `OffboardBlackboxRecord`;
   - при invalid/non-finite trajectory altitude target логируется `trajectory_altitude_target_valid=false`, `target_altitude_m=cruise_altitude_m_`, и fallback явно виден в telemetry/blackbox.

   Terminal position capture остаётся как сейчас: у финиша position setpoint владеет режимом стабилизации.

9. Добавить dumps/logs для vertical profile.

   Файлы:
   - `trajectory_diagnostics_io_csv.cpp`
   - `final_trajectory_debug_io.cpp`
   - `trajectory_diagnostics_io_json_summary.cpp`
   - `trajectory_diagnostics_io_json_fields.cpp`
   - `trajectory_diagnostics_io_parser.cpp`
   - `px4_offboard_node_trajectory.cpp`
   - `px4_offboard_node_telemetry.cpp`
   - `offboard_blackbox.hpp`
   - `offboard_blackbox.cpp`
   - `offboard_blackbox_test.cpp`

   CSV `final_trajectory_samples` расширить колонками:

   ```text
   vertical_slope_dz_ds,
   vertical_speed_limit_mps,
   vertical_accel_limit_mps,
   vertical_jerk_limit_mps,
   vertical_constraint_active,
   vertical_profile_passage_id
   ```

   Planner/offboard summary log добавить:

   ```text
   vertical_profile[enabled=true active=true passages=1 min_z=... max_z=...
                    max_slope=... min_vertical_speed_cap=... infeasible=0]
   ```

   JSON summary добавить top-level поля `vertical_profile_*` и diagnostic array/count по passages.

   Headless runtime blackbox contract расширить явными altitude-control полями:

   ```json
   "altitude_control": {
     "target_altitude_m": 12.5,
     "trajectory_altitude_target_valid": true,
     "altitude_error_m": -0.4,
     "vertical_velocity_setpoint_mps": 0.8
   }
   ```

   Если implementation решит не добавлять отдельный object, допустимы поля внутри существующего `velocity_command`, но contract должен быть явным и покрытым `offboard_blackbox_test.cpp`. Главное требование: по `log/offboard_blackbox.jsonl` должно быть понятно, следовал ли controller `z(s)` или fallback-нул на cruise altitude.

10. Добавить RViz altitude/profile markers.

    Файлы:
    - `trajectory_debug_markers.hpp/cpp`
    - `trajectory_debug_markers_test.cpp`

    Текущие speed/curvature markers уже используют `sample.z_m`. Добавить третий marker namespace:

    ```text
    final_trajectory_altitude_colormap
    ```

    Цвет: normalized altitude между min/max z. Для vertical speed cap можно добавить четвёртый optional namespace:

    ```text
    final_trajectory_vertical_cap_colormap
    ```

    На первом implementation pass достаточно altitude colormap + существующих speed colors, потому что `source=vertical_profile` будет виден в speed diagnostics/CSV.

11. Обновить known-passage validation после появления `z(s)`.

    Файлы:
    - `known_passage_validation.cpp`
    - `known_passage_validation_test.cpp`

    После vertical profile builder validation должна проходить для matched opening, где раньше constant cruise altitude мог давать `opening_volume_miss`. Добавить regression:

    ```text
    constant z=18 пересекает opening [8..12] -> miss
    after applyVerticalProfile gate_z=10 -> matched_opening
    ```

12. Обновить tests.

    Новые/изменённые тесты:

    - `trajectory_vertical_profile_test.cpp`
      - no map / disabled -> constant cruise altitude;
      - matched opening -> smooth cruise-gate-cruise;
      - gate altitude clamped with margin;
      - endpoints stay cruise;
      - max climb angle violation marks invalid;
      - overlapping incompatible windows marks invalid;
      - diagnostics include structure/opening/span/gate_z.

    - `trajectory_speed_planner_test.cpp`
      - vertical slope creates `SpeedConstraintType::kVerticalProfile`;
      - vertical cap participates in backward braking pass;
      - top constraints include `source=vertical_profile`.

    - `trajectory_planner_test.cpp`
      - planner assigns non-constant z when route crosses known passage;
      - speed profile built after z assignment;
      - result invalid if required vertical profile infeasible.

    - `offboard_trajectory_state_test.cpp`
      - path message z survives sample retention.

    - `offboard_velocity_follower` or node-level test
      - vertical target uses trajectory altitude at projection s;
      - fallback to `cruise_altitude_m` when trajectory altitude target is invalid/non-finite;
      - fallback exposes `trajectory_altitude_target_valid=false` in diagnostic state.

    - diagnostics tests:
      - CSV header/row include vertical fields;
      - JSON fields contain vertical stats;
      - parser roundtrip preserves vertical stats.

    - `offboard_blackbox_test.cpp`
      - blackbox JSON contains `target_altitude_m`;
      - blackbox JSON contains `trajectory_altitude_target_valid`;
      - blackbox JSON contains `altitude_error_m`;
      - blackbox JSON contains `vertical_velocity_setpoint_mps`;
      - fallback case serializes cruise target and invalid trajectory target flag.

    - telemetry/headless diagnostics contract
      - helper or node-level test verifies telemetry state fields are populated before `writeFlightBlackbox()`;
      - log format includes target altitude and validity flag, so headless run can distinguish `trajectory z(s)` following from cruise-altitude fallback.

    - config tests:
      - YAML/default params load;
      - clamps/sanitize;
      - fingerprint changes when vertical profile construction caps change.

## Verification Plan

1. Format changed C++ files:

   ```bash
   ./scripts/dev_shell.sh
   make format
   ```

2. Build:

   ```bash
   ./scripts/build.sh
   ```

3. Scoped unit tests after build:

   ```bash
   ctest --test-dir build/drone_city_nav --output-on-failure \
     -R 'known_passage|trajectory_vertical_profile|trajectory_speed_planner|trajectory_planner|offboard_trajectory_state|trajectory_diagnostics_io|trajectory_debug_markers|planner_node_config|px4_offboard_node_config|velocity_control_config'
   ```

4. Script-level tests:

   ```bash
   ./scripts/dev_shell.sh
   make test-scripts
   ```

5. Full package tests:

   ```bash
   ./scripts/test.sh
   ```

6. Quality gate before commit:

   ```bash
   ./scripts/dev_shell.sh
   make quality
   ```

7. Optional integration validation after implementation:

   ```bash
   ./scripts/sim_headless.sh
   ```

   Для headless run проверять не только успешность полёта, но и:
   - `final_trajectory_samples.csv` содержит non-constant `z_m`;
   - `top_speed_constraint source=vertical_profile` появляется на подъёме/спуске, если профиль требует cap;
   - RViz/debug markers строятся на разных высотах;
   - offboard telemetry target altitude следует `projection_z_m`, а не всегда `cruise_altitude_m`.

## Testing Strategy

1. Без рефакторинга

   - Прямые unit tests для `trajectory_vertical_profile`.
   - Расширить `trajectory_speed_planner_test` вертикальными caps.
   - Расширить diagnostics CSV/JSON tests.
   - Расширить config/fingerprint tests.
   - Проверить, что без known passages или при disabled vertical profile поведение остаётся constant altitude.

2. Лёгкий рефакторинг

   - Вынести shared known-passage matcher из validation `.cpp`, покрыть его regression tests.
   - Перенести altitude target selection в маленькую pure/helper функцию, чтобы не тестировать всю PX4 node state machine ради одной формулы.
   - Обновить `finalizeResult()` так, чтобы altitude assignment был явным этапом, а не скрытым unconditional overwrite.

3. Тяжёлый рефакторинг

   - Если vertical caps начнут требовать больше контекста, выделить отдельный `SpeedConstraintEvaluator` вместо расширения `geometricSpeedSampleFromPointSample()`.
   - Если passages начнут требовать XY-вставки, добавить полноценный executable trajectory artifact с per-sample metadata и stitch points. Это не входит в Stage 4.
   - Если offboard altitude controller окажется недостаточным, выделить отдельный vertical setpoint smoother с accel/jerk state. В Stage 4 сначала используем существующий altitude P-control + speed caps.

## Risks And Tradeoffs

- Это первый этап, который меняет trajectory behavior, поэтому риск выше, чем у diagnostic-only stages.
- Vertical-only подход не сможет починить ситуацию, где 2D XY path проходит мимо opening. На этом этапе он работает только если XY-траектория уже пересекает footprint/opening в плане.
- Smooth `z(s)` может сильно снизить скорость через vertical caps. Это правильно физически, но может выглядеть как неожиданное торможение, если passage altitude далеко от cruise altitude.
- `max_climb_angle` нельзя исправить снижением скорости; если профиль слишком крутой по геометрии, нужно расширять transition window или признавать profile infeasible.
- Existing offboard vertical P-control может быть слишком простым для агрессивного `z(s)`. Speed caps должны сделать профиль достаточно медленным, но после первого run нужно смотреть altitude tracking error.
- Shared matcher refactor должен сохранить текущую known-passage validation семантику. Это критично, потому что validation уже покрывает safety contract для structures/openings.
- RViz уже рисует line markers на z, но плоская static map останется плоской до отдельного stage с volumetric building visualization.

## Open Questions

- Начальные вертикальные limits: принять предложенные `2.5 m/s`, `2.0 m/s^2`, `6.0 m/s^3`, `12 deg` или задать более мягкие значения для первого run?
- При infeasible vertical profile hard-fail trajectory сразу или публиковать 2D constant-altitude fallback с явным warning? Я предлагаю hard-fail только если trajectory пересекает known structure и должен пройти через opening; иначе есть риск визуально "валидного" пути через стену.
- Должен ли `gate_z` быть всегда center.z или лучше выбирать ближайшую к cruise altitude высоту внутри `[min_z+margin, max_z-margin]`? Для плавности лучше ближайшую к cruise altitude, для визуальной читаемости opening center проще. Я предлагаю ближайшую к cruise altitude valid height, а center использовать только если cruise outside opening interval.
- Нужно ли уже в этом stage добавлять отдельный RViz marker для volumetric passages/buildings, или оставить это следующему stage? Для Stage 4 достаточно altitude trajectory markers; volumetric buildings лучше отдельным commit.
