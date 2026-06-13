# Investigation: drone movement speed limits

## Context/Task

Задача: исследовать, можно ли увеличить скорость движения дрона до максимально
возможной физически и технологически в текущей связке Gazebo + ROS 2 + PX4 SITL.

Исследование выполнено только по локальному репозиторию и локальной копии
`external/PX4-Autopilot`. Удаленные источники, SSH, HTTP, Notion и GitLab не
использовались.

## Research questions

1. Где в текущем проекте фактически ограничивается горизонтальная скорость?
2. Есть ли явный параметр `max_speed_mps` или эквивалент?
3. Какие ограничения задает наш ROS 2 offboard follower, а какие остаются внутри
   PX4?
4. Можно ли увеличить скорость только конфигом?
5. Что нужно сделать, если нужен настоящий технологический максимум, а не просто
   более агрессивное движение в MVP?

## Scope and constraints

В scope включены:

- `drone_city_nav` ROS 2 package;
- launch/config/script files used by MVP simulation;
- локальная копия PX4 Autopilot under `external/PX4-Autopilot`;
- git history текущего репозитория.

В scope не включены:

- реальный запуск speed sweep в SITL;
- tuning PX4 параметров с последующей валидацией;
- изменение production-кода;
- удаленное чтение документации или upstream репозиториев.

## Detected stack/profiles

Прочитаны профили оркестратора:

- `docs/project_profiles/generic.md`;
- `docs/project_profiles/cpp.md`.

Выбран профиль C++/ROS 2 workspace. Репозиторий использует `colcon` через
`Makefile`, а не top-level ad-hoc CMake workflow.

## Repo-approved commands found

Repo-approved commands найдены в `README.md`, `CONTRIBUTING.md` и `Makefile`:

- `./scripts/dev_shell.sh`;
- `make build`;
- `colcon build --packages-select drone_city_nav --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`;
- `make test`;
- `ctest --test-dir build/drone_city_nav --output-on-failure`;
- `make quality`;
- `./scripts/check_cpp_quality.sh`;
- `./scripts/format_cpp_changed.sh`;
- `make sim-gui`;
- `./scripts/run_city_mvp.sh`;
- `make sim-headless`;
- `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp.sh`.

## Sources checked

- `README.md`;
- `CONTRIBUTING.md`;
- `Makefile`;
- `docs/MVP_SIMULATION.md`;
- `drone_city_nav/config/urban_mvp.yaml`;
- `drone_city_nav/config/real_drone_template.yaml`;
- `drone_city_nav/src/px4_offboard_node.cpp`;
- `drone_city_nav/src/planner_node.cpp`;
- `drone_city_nav/src/mission_monitor_node.cpp`;
- `scripts/run_city_mvp.sh`;
- `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4013_gz_x500_lidar_2d`;
- `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`;
- `external/PX4-Autopilot/src/modules/mc_pos_control/*params.c`;
- git history for speed/offboard/setpoint-related commits.

## Evidence

В текущем проекте нет отдельного параметра `max_speed_mps` для offboard flight.
Горизонтальная скорость получается из сочетания наших position setpoints и
внутренних PX4 position-controller limits.

Текущий offboard node публикует setpoints с частотой 10 Hz:

- `drone_city_nav/src/px4_offboard_node.cpp:136`.

Ключевые параметры follower объявлены здесь:

- `max_setpoint_distance_m`, clamp `[0.5, 50.0]`:
  `drone_city_nav/src/px4_offboard_node.cpp:65`;
- `max_commanded_target_step_m`, clamp `[0.01, 10.0]`:
  `drone_city_nav/src/px4_offboard_node.cpp:67`;
- `lookahead_distance_m`, clamp `[0.0, 50.0]`:
  `drone_city_nav/src/px4_offboard_node.cpp:69`.

Текущий MVP config задает:

- `max_setpoint_distance_m: 1.5`:
  `drone_city_nav/config/urban_mvp.yaml:85`;
- `max_commanded_target_step_m: 0.25`:
  `drone_city_nav/config/urban_mvp.yaml:86`;
- `lookahead_distance_m: 2.5`:
  `drone_city_nav/config/urban_mvp.yaml:87`;
- `replan_period_s: 1.5`:
  `drone_city_nav/config/urban_mvp.yaml:70`;
- `max_lidar_range_m: 35.0`:
  `drone_city_nav/config/urban_mvp.yaml:63`.

Так как timer period равен 0.1 s, `max_commanded_target_step_m: 0.25` означает
верхний темп движения commanded target примерно `0.25 / 0.1 = 2.5 m/s`. Это не
гарантирует фактическую скорость ровно 2.5 m/s, потому что PX4 еще сглаживает и
ограничивает движение, но это текущий явный ограничитель на стороне нашего
offboard follower.

Offboard mode сейчас position-based:

- `msg.position = true`:
  `drone_city_nav/src/px4_offboard_node.cpp:429`;
- `msg.velocity = false`:
  `drone_city_nav/src/px4_offboard_node.cpp:430`;
- `TrajectorySetpoint.position` заполняется:
  `drone_city_nav/src/px4_offboard_node.cpp:445`;
- `TrajectorySetpoint.velocity` выставляется в нули:
  `drone_city_nav/src/px4_offboard_node.cpp:450`.

Целевая точка дополнительно ограничивается радиусом вокруг текущей позиции:

- `limitedTarget()` clamps target to `max_setpoint_distance_m`:
  `drone_city_nav/src/px4_offboard_node.cpp:530`;
- `smoothedCommandTarget()` limits per-tick target movement:
  `drone_city_nav/src/px4_offboard_node.cpp:546`;
- `clampCommandedTargetToCurrent()` repeats the current-position radius clamp:
  `drone_city_nav/src/px4_offboard_node.cpp:571`.

PX4 SITL launch script сейчас не задает speed/acceleration/tuning parameters:

- script sends only `CBRK_SUPPLY_CHK` and `NAV_DLL_ACT`:
  `scripts/run_city_mvp.sh:185`;
- PX4 model target defaults to `gz_x500_lidar_2d`:
  `scripts/run_city_mvp.sh:28`.

Локальная PX4 airframe chain:

- `4013_gz_x500_lidar_2d` only selects `x500_lidar_2d` and sources
  `4001_gz_x500`: `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4013_gz_x500_lidar_2d:8`;
- `4001_gz_x500` sets simulator, rotor allocation and `MPC_THR_HOVER`, but does
  not override horizontal speed limits:
  `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500:8`,
  `external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500:50`.

PX4 default multicopter limits in the local checkout include:

- `MPC_XY_VEL_MAX` default `12.f`, declared max `20 m/s`:
  `external/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_control_limits_params.c:35`;
- `MPC_XY_CRUISE` default `5.f`, declared max `20 m/s`:
  `external/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:35`;
- `MPC_ACC_HOR` default `3.f`, declared max `15 m/s^2`:
  `external/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:76`;
- `MPC_ACC_HOR_MAX` default `5.f`, declared max `15 m/s^2`:
  `external/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_mode_params.c:100`;
- `MPC_JERK_MAX` default `8.f`, declared max `500 m/s^3`:
  `external/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_mode_params.c:115`.

Mission monitor measures speed but does not command speed:

- `current_speed_mps = speed2D(msg)`:
  `drone_city_nav/src/mission_monitor_node.cpp:189`;
- speed is used to confirm stopped-at-goal:
  `drone_city_nav/src/mission_monitor_node.cpp:234`.

## Evidence references

Command excerpts:

- `git log --oneline --all --regexp-ignore-case --grep='speed\|velocity\|setpoint\|offboard\|faster' -n 30`
  found:
  - `d4b8f71 Increase offboard speed tuning`;
  - `451a215 Dampen offboard trajectory feedforward`;
  - `d1df7bd Reduce offboard maneuver aggressiveness`;
  - `49835a0 Clamp smoothed offboard target near vehicle`;
  - `40745fc Smooth offboard position setpoints`;
  - `c0298e0 Stabilize offboard navigation setpoints`.

Relevant diffs:

- `d4b8f718ef66cbcb50273a83ece50006ad7c00bc` increased MVP tuning from
  `max_setpoint_distance_m: 0.9` to `1.5`, from
  `max_commanded_target_step_m: 0.15` to `0.25`, and from
  `lookahead_distance_m: 1.5` to `2.5`.
- `d1df7bdf8cfca708af89595aac789522c967a042` earlier reduced maneuver
  aggressiveness down to `max_setpoint_distance_m: 0.3` and
  `max_commanded_target_step_m: 0.05`.
- `49835a0406b4f925fca545f399af5e51374b06a9` added clamping of the smoothed
  commanded target near the vehicle.
- `451a2151485d3f4b6375676baaa7f3033c398f12` changed trajectory velocity
  feedforward from NaN while path-valid to zero velocity.

Documentation excerpt:

- `docs/MVP_SIMULATION.md:258` says the default offboard tuning advances
  setpoints about three times faster than the initial conservative MVP tuning.

## Findings

1. Current effective speed is mainly limited by our own offboard follower config,
   not by an explicit mission speed parameter.
2. With current config, commanded target advance is capped at about `2.5 m/s`.
3. PX4 position control likely can track faster than the current MVP config,
   because local PX4 defaults show `MPC_XY_VEL_MAX=12 m/s` and parameter metadata
   allows up to `20 m/s`.
4. The current mode is position-setpoint control. This is acceptable for a stable
   MVP, but it is not the cleanest architecture for requesting a specific cruise
   speed or for exploring a technological maximum.
5. A config-only increase is possible, but the safety envelope is constrained by
   `35 m` lidar range, `1.5 s` replanning period, A* path shape, obstacle
   inflation, PX4 acceleration limits and building-clearance checks.
6. The planner does not own speed. It publishes paths; speed is an emergent
   property of offboard follower parameters and PX4 control limits.
7. The real-drone template is more conservative and lacks
   `max_commanded_target_step_m`, so any speed-tuning work should avoid blindly
   applying simulation aggression to real hardware configs.

## Relevant code paths

- `drone_city_nav/src/px4_offboard_node.cpp`: position setpoint generation,
  target lookahead, target distance clamp, commanded target smoothing, PX4
  command publication.
- `drone_city_nav/config/urban_mvp.yaml`: current simulation tuning.
- `drone_city_nav/src/planner_node.cpp`: path replanning cadence and lidar/grid
  usage.
- `drone_city_nav/src/mission_monitor_node.cpp`: validation of movement, collision
  clearance, goal reach and stopped-at-goal state.
- `scripts/run_city_mvp.sh`: Gazebo/PX4/ROS orchestration and PX4 parameter
  injection.
- `external/PX4-Autopilot/src/modules/mc_pos_control/*params.c`: local PX4
  position-controller limit definitions.

## Timeline/history

- `c0298e0` and `40745fc` introduced/stabilized offboard position setpoint
  behavior.
- `49835a0` added a current-position clamp for smoothed commanded targets,
  reducing large jumps after replanning.
- `d1df7bd` reduced aggressiveness to very conservative values while debugging
  stability and collision behavior.
- `451a215` damped trajectory feedforward by setting velocity to zero.
- `d4b8f71` raised the simulation tuning to the current values and documents that
  the default offboard tuning advances setpoints about three times faster than
  the initial conservative MVP tuning.

## Hypotheses/alternatives

Hypothesis A: config-only speed increase.

- Increase `max_commanded_target_step_m`, `max_setpoint_distance_m` and
  `lookahead_distance_m`.
- Optionally reduce `replan_period_s`.
- Pros: small change, no interface redesign.
- Cons: speed remains indirect; high values can cause corner cutting,
  late obstacle reaction, or unstable PX4 chasing behavior.

Hypothesis B: explicit velocity-aware offboard follower.

- Add `desired_speed_mps`, acceleration limit and braking distance logic.
- Publish velocity setpoints or position setpoints with deliberate velocity
  feedforward after confirming PX4 semantics in SITL.
- Pros: speed becomes a first-class controlled value.
- Cons: larger code change; requires more tests and SITL validation.

Hypothesis C: tune PX4 internal limits as part of simulation profile.

- Set `MPC_XY_VEL_MAX`, `MPC_XY_CRUISE`, `MPC_ACC_HOR`,
  `MPC_ACC_HOR_MAX` and possibly jerk-related parameters in the PX4 startup flow.
- Pros: required for exploring the actual upper envelope.
- Cons: unsafe without sweep tests; may hide planner/sensor limitations.

## Risk/impact

Increasing speed raises these risks:

- less time to react to newly seen buildings with `max_lidar_range_m=35.0`;
- stale obstacle data if the vehicle moves far during `replan_period_s=1.5`;
- larger overshoot near corners and narrow Manhattan-grid streets;
- more frequent emergency stops or building-clearance failures;
- stronger dependence on PX4 acceleration/tilt/jerk parameters;
- simulation tuning accidentally leaking into real-drone config.

At `10 m/s`, `35 m` lidar range gives only about `3.5 s` of raw forward sensing
range before an obstacle. At `20 m/s`, that drops to about `1.75 s`. Those are
not validated safe numbers for the current planner; they are timing estimates to
frame the risk.

## Conclusions

The current simulation is not running at the technological maximum. It is
intentionally limited by the offboard follower to roughly `2.5 m/s` commanded
target advance.

A moderate speed increase is feasible by configuration, probably starting with
`max_commanded_target_step_m` in the `0.5..1.0` range, corresponding to
approximately `5..10 m/s` commanded target advance at the current 10 Hz timer,
plus matching increases to `max_setpoint_distance_m` and `lookahead_distance_m`.
This must be validated, not assumed safe.

The true upper bound for the current local PX4 defaults is likely below or around
the PX4 horizontal velocity limit, with `MPC_XY_VEL_MAX` defaulting to `12 m/s`
and metadata allowing up to `20 m/s`. Reaching or safely using those values in
this city-navigation task requires explicit PX4 tuning and SITL sweep
validation.

## Recommendations/next steps

1. Add an explicit simulation-only `desired_speed_mps` concept instead of relying
   only on per-tick target step.
2. Keep `max_commanded_target_step_m` as a safety limiter, but derive its default
   from `desired_speed_mps * controller_period_s`.
3. Add logs for actual 2D speed, requested speed, commanded target step, target
   lead distance and active PX4 speed/acceleration params.
4. Implement a speed sweep script, for example `3, 5, 7, 10, 12 m/s`, using
   `HEADLESS=1 MISSION_CHECK=1`.
5. For each speed, require mission success, no building-clearance violation, no
   crash detection, and acceptable minimum goal distance.
6. Only after simulation sweep succeeds, consider setting PX4 `MPC_*` speed and
   acceleration params in `scripts/run_city_mvp.sh` behind explicit environment
   variables.
7. Do not apply aggressive SITL tuning to `real_drone_template.yaml` without a
   separate real-hardware safety review.

## Verification plan

For config-only trials:

1. Change only simulation config.
2. Run `make build` if C++ is unchanged but launch/config behavior changed.
3. Run `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp.sh`.
4. Inspect logs for actual speed, distance to goal, clearance and emergency stop.
5. Repeat with increasing speed until the first failure, then bisect down.

For code changes:

1. Format changed C++ files with `./scripts/format_cpp_changed.sh`.
2. Run `./scripts/check_cpp_quality.sh`.
3. Run the headless mission check.
4. Commit only after the quality and mission checks are understood.

## Testing/verification implications

This investigation changed only documentation. Production code and simulation
config were not changed, so build, unit tests and SITL mission run were not
necessary for this artifact.

Performed artifact checks:

- `test -s INVESTIGATION.md`;
- `git diff --check`;
- required-section presence check with `grep`.

## Open questions

1. What maximum speed target is desired for MVP UX: faster demo, realistic city
   flight, or stress test?
2. Should simulation expose PX4 `MPC_*` params through documented environment
   variables?
3. Do we want speed to be path-aware, reducing speed near corners and narrow
   passages?
4. Should the mission monitor enforce a maximum speed during safety validation,
   or only detect collisions and goal completion?
5. Do we need a separate high-speed city map with wider streets to distinguish
   controller speed limits from planner/sensor limits?
