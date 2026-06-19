# GUI Drone Visibility And Diagnostics Plan

## Context

The current task is a planning workflow, not an implementation workflow. The
goal is to turn the findings from `INVESTIGATION.md` into a concrete fix plan
for the Gazebo GUI drone visibility / follow-camera diagnostics issues, with
enough automated tests and logs to debug future headless and GUI runs without
guessing.

The user-visible symptoms are:

- The PX4 drone flies and is visible in RViz telemetry, but the Gazebo 3D world
  can fail to show the drone and its yellow ground marker.
- Gazebo follow-camera behavior regressed after earlier camera/model changes.
- GUI launch diagnostics are weak because Gazebo GUI stdout/stderr is discarded.
- Headless validation has at least one stale log-pattern expectation.

## Investigation context

`INVESTIGATION.md` concludes that the drone is created server-side:

- PX4 spawns `x500_lidar_2d_0`.
- Gazebo topics and `/pose/info` contain the model, links, and visual names.
- PX4 and ROS control continue to operate, so the failure boundary is after
  server-side model creation: Gazebo GUI/render client, visual scene delivery,
  resource loading, or local EGL/render stack.

Important concrete findings from the investigation and follow-up file reads:

- `scripts/run_city_mvp.sh:400` launches the Gazebo GUI as
  `gz sim -g > /dev/null 2>&1`, which discards the exact logs needed for GUI
  rendering failures.
- `scripts/validate_city_mvp_headless.py:232` still expects
  `Published path: waypoints=...`, while the current planner logs
  `Published path: reason=... waypoints=...` at
  `drone_city_nav/src/planner_node.cpp:1187`.
- `scripts/gazebo_gui_control.py:146` checks `/gui/currently_tracked` with only
  `-d 0.2`, so successful camera tracking can be reported as best-effort even
  when the state was not really observed.
- `drone_city_nav/models/lidar_2d_v2/model.sdf:77` stores the yellow drone
  locator and ground projection visuals inside the lidar sensor model.
  The wrapper model `drone_city_nav/models/x500_lidar_2d/model.sdf:3` currently
  only includes `x500`, includes `lidar_2d_v2`, and adds `LidarJoint`.
- `INVESTIGATION.md:688` records H4: the local wrapper uses
  `<uri>model://x500</uri>` at `drone_city_nav/models/x500_lidar_2d/model.sdf:5`,
  while upstream PX4 uses bare `<uri>x500</uri>` at
  `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf:5`.
  The investigation rates this as low-medium confidence because the current SDF
  is valid and the server-side model exists, but it still needs an explicit plan
  closure.
- `scripts/validate_gazebo_gui_launch_log.py:31` validates only the combined
  Gazebo launcher log. It has no way to validate a separate GUI log or scene
  diagnostics artifact.
- `scripts/tests/test_gazebo_gui_control.py`,
  `scripts/tests/test_validate_city_mvp_headless.py`,
  `scripts/tests/test_validate_gazebo_gui_launch_log.py`, and
  `scripts/tests/test_run_city_mvp_launch_contract.py` already provide the
  right script-level test surface for these fixes.

## Detected stack/profiles

- Repository type: ROS 2 workspace with an ament CMake package,
  `drone_city_nav`.
- Primary languages and file types touched by this plan: Bash, Python, SDF/XML,
  Markdown. C++ is relevant for log anchors and existing tests but is not
  necessarily required for the first fix batch.
- Build profile: C++ / ROS 2 / colcon through the repository `Makefile`.
- Rust profile: not applicable. No `Cargo.toml`, `Cargo.lock`, or `*.rs` files
  were found in this repository.
- Protocols read for this planning workflow:
  `notion_access_protocol.md`, `gitlab_access_protocol.md`, generic profile,
  and C++ profile.

## Repo-approved commands found

Use commands from the repository root.

- Default native workflow:
  - `make host-build`
  - `make host-test`
  - `make host-test-scripts`
  - `make host-quality`
  - `make host-format`
  - `make host-sim-headless`
  - `make host-sim-gui`
- Current-environment/container workflow:
  - `make build`
  - `make test`
  - `make test-scripts`
  - `make quality`
  - `make format`
  - `make sim-headless`
  - `make sim-gui`
- Direct simulation entry points documented in `README.md`:
  - `./scripts/run_city_mvp_host.sh`
  - `HEADLESS=1 SMOKE_DURATION_S=90 ./scripts/run_city_mvp_host.sh`
- For routine agent work, prefer the native host workflow through
  `./scripts/host_shell.sh` and the `host-*` targets.
- Do not use ad-hoc top-level CMake commands; use colcon through the approved
  targets.

## Affected components

- `scripts/run_city_mvp.sh`
  - Gazebo GUI process launch and log redirection.
  - GUI follow-camera orchestration.
  - Optional scene diagnostics capture after spawn / GUI startup.
- `scripts/gazebo_gui_control.py`
  - Follow-camera command confirmation.
  - `/gui/currently_tracked` polling robustness.
- `scripts/validate_city_mvp_headless.py`
  - Planner path publication regex.
- `scripts/validate_gazebo_gui_launch_log.py`
  - GUI log validation.
  - Scene diagnostics validation.
- `scripts/tests/test_*`
  - Existing script-level tests for launcher contract, GUI control, GUI log
    validation, and headless validation.
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
  - Drone wrapper model, base model URI, and visual locator ownership.
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
  - Lidar sensor model; should not own whole-drone visibility helpers.
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
  - Upstream reference for the wrapper model URI style.
- `docs/MVP_SIMULATION.md` and `README.md`
  - Launch and diagnostics documentation after implementation.

## Implementation steps

1. Preserve Gazebo GUI stderr/stdout in a dedicated log file.

   Change `scripts/run_city_mvp.sh:66-70` to introduce
   `gz_gui_log_file="${GZ_GUI_LOG_FILE:-${run_log_dir}/gz_gui_city_mvp.log}"`,
   create/truncate it near `scripts/run_city_mvp.sh:199-200`, print it in the
   startup diagnostics near `scripts/run_city_mvp.sh:363`, and replace
   `scripts/run_city_mvp.sh:400-401` with a command that appends the GUI client
   output to that file:

   ```bash
   gz sim -g >> "${gz_gui_log_file}" 2>&1 &
   ```

   Keep the server/world/orchestration output in `gz_log_file`; do not merge
   the GUI stream into `/dev/null`.

2. Add deterministic Gazebo scene diagnostics capture.

   Add a small Python helper, for example
   `scripts/capture_gazebo_scene_diagnostics.py`, that collects bounded
   snapshots with timeouts:

   - `/world/generated_city/pose/info`
   - `/world/generated_city/scene/info`
   - `/gui/currently_tracked`

   The helper should write raw artifacts under a configurable directory such as
   `${run_log_dir}/gazebo_scene_debug/` and print a short summary:

   - whether `x500_lidar_2d_0` appears;
   - whether `base_link_visual`, rotor visuals, lidar visuals, and yellow
     locator / ground projection visual names appear;
   - whether current GUI tracking state mentions `x500_lidar_2d_0`;
   - any command timeout as a warning, not as an immediate launch failure.

   Call this helper from `scripts/run_city_mvp.sh` after `startup_sleep_s` and
   before ROS launch, with an opt-out such as
   `ENABLE_GZ_SCENE_DIAGNOSTICS=false`. This makes GUI/no-GUI visibility issues
   inspectable from log artifacts.

3. Extend GUI log validation to consume the new artifacts.

   Update `scripts/validate_gazebo_gui_launch_log.py:31` so it can accept:

   - the existing launcher log;
   - optional `--gui-log`;
   - optional `--scene-diagnostics-dir`.

   The validator should require the existing stale-cleanup and unpause markers
   from the launcher log, check that GUI follow-camera status is logged, and
   then warn on known render-stack messages in the GUI log while failing on
   clear launch errors. Scene diagnostics should fail only when the raw data is
   present and contradicts a required invariant, for example the drone model is
   absent from `/pose/info` after spawn.

4. Fix headless validator path-publication matching.

   Update `scripts/validate_city_mvp_headless.py:232-235` from the stale regex
   to a pattern compatible with the current planner log:

   ```python
   r"Published path:\s+(?:reason=\S+\s+)?waypoints=[1-9]\d*"
   ```

   Add a unit test in `scripts/tests/test_validate_city_mvp_headless.py` where
   the synthetic ROS log contains
   `Published path: reason=initial_plan waypoints=12 segments=...`.

5. Make follow-camera confirmation less brittle.

   Refactor `scripts/gazebo_gui_control.py:146-156` so `_confirm_tracking`
   accepts a deadline/sample duration parameter and polls long enough to observe
   a real `/gui/currently_tracked` message. Keep the command best-effort, but
   distinguish these outcomes in logs:

   - command accepted and state confirmed;
   - command accepted, state topic unavailable;
   - command rejected or target never became available.

   Update `scripts/tests/test_gazebo_gui_control.py:51-86` to cover:

   - state confirmed on `/gui/currently_tracked`;
   - accepted command with empty tracking topic logs a warning;
   - malformed offset still avoids any `gz` calls.

6. Move yellow drone locator visuals out of the lidar sensor model.

   Modify `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152` so the lidar
   model contains only lidar collisions, lidar visuals, and the `gpu_lidar`
   sensor. Move the yellow locator core, locator arms, ground beam, and ground
   projection disc into `drone_city_nav/models/x500_lidar_2d/model.sdf`.

   Preferred implementation: add a dedicated visual-only link in the wrapper
   model, fixed to `base_link`, with no collision geometry:

   ```xml
   <link name="visibility_marker_link">
     ...
   </link>
   <joint name="VisibilityMarkerJoint" type="fixed">
     <parent>base_link</parent>
     <child>visibility_marker_link</child>
   </joint>
   ```

   Validate SDF syntax with `gz sdf -k` when the tool is available in the host
   environment. If Gazebo requires inertial data for the new link, add tiny,
   explicit inertial values rather than coupling the marker to the lidar mass.

7. Close H4 by normalizing the wrapper base-model URI to upstream PX4 style.

   Change `drone_city_nav/models/x500_lidar_2d/model.sdf:4-6` from:

   ```xml
   <include merge="true">
     <uri>model://x500</uri>
   </include>
   ```

   to the upstream-style form used at
   `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf:4-6`:

   ```xml
   <include merge="true">
     <uri>x500</uri>
   </include>
   ```

   Expected result: the local wrapper no longer differs from PX4 upstream on the
   base model URI. This does not assume H4 is the root cause; it removes a
   low-medium confidence divergence while the new GUI logs and scene diagnostics
   verify whether resource loading still fails.

8. Add SDF structure tests for visibility helpers and the base-model URI.

   Add `scripts/tests/test_drone_model_sdf_contract.py` using Python XML
   parsing. The tests should assert:

   - `drone_city_nav/models/x500_lidar_2d/model.sdf` contains
     `visibility_marker_link` and `VisibilityMarkerJoint`;
   - the wrapper contains the expected yellow visual names;
   - `drone_city_nav/models/lidar_2d_v2/model.sdf` still contains the lidar
     `gpu_lidar` sensor;
   - the lidar model no longer contains `yellow_drone_locator_*` or
     `yellow_ground_projection_*` visuals;
   - `drone_city_nav/models/x500_lidar_2d/model.sdf` uses bare `x500` for the
     base include and `model://lidar_2d_v2` for the lidar include.

   This prevents the same ownership regression from silently returning.

9. Add launch-contract tests for GUI log preservation and diagnostics.

   Extend `scripts/tests/test_run_city_mvp_launch_contract.py` so it checks:

   - `gz sim -g` is not redirected to `/dev/null`;
   - `GZ_GUI_LOG_FILE` / `gz_gui_log_file` exists in the launcher;
   - scene diagnostics capture is called or explicitly logged;
   - stale cleanup still runs before Gazebo launch.

10. Update GUI launch documentation.

   Update `README.md` and `docs/MVP_SIMULATION.md` after implementation to
   document:

   - where the GUI log is written;
   - how to run GUI diagnostics validation;
   - how to disable scene diagnostics if needed;
   - what a successful follow-camera / scene diagnostics summary should look
     like.

## Verification plan

Run these checks after implementation:

1. Static file sanity:

   ```bash
   git diff --check
   ```

2. SDF syntax check when Gazebo CLI is available in the host environment:

   ```bash
   ./scripts/host_shell.sh bash -lc \
     'export SDF_PATH="$PWD/drone_city_nav/models:$PWD/external/PX4-Autopilot/Tools/simulation/gz/models"; \
      gz sdf -k drone_city_nav/models/x500_lidar_2d/model.sdf'
   ```

3. Script unit tests:

   ```bash
   make host-test-scripts
   ```

4. Full repository quality gate:

   ```bash
   make host-quality
   ```

5. Headless simulation smoke with mission validation:

   ```bash
   HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp_host.sh
   ```

6. GUI diagnostics smoke without relying on manual viewing:

   ```bash
   ENABLE_RVIZ=false SMOKE_DURATION_S=120 ./scripts/run_city_mvp_host.sh
   python3 scripts/validate_gazebo_gui_launch_log.py \
     log-host/gz_city_mvp.log \
     --gui-log log-host/gz_gui_city_mvp.log \
     --scene-diagnostics-dir log-host/gazebo_scene_debug
   ```

7. Optional manual GUI confirmation after automated checks pass:

   ```bash
   make host-sim-gui
   ```

   Manual confirmation should only verify what automation cannot prove from the
   current logs: that the Gazebo 3D viewport visibly renders the yellow drone
   locator and ground projection.

## Testing strategy (categories 1/2/3: no refactor / light / heavy)

1. No refactor tests:

   - Update the headless validator regex test in
     `scripts/tests/test_validate_city_mvp_headless.py`.
   - Add launcher contract assertions in
     `scripts/tests/test_run_city_mvp_launch_contract.py`.
   - Add GUI log validator tests in
     `scripts/tests/test_validate_gazebo_gui_launch_log.py` for split
     launcher/GUI logs and scene diagnostics warnings.

2. Light refactor tests:

   - Extend `scripts/tests/test_gazebo_gui_control.py` with fake
     `CommandRunner` responses for confirmed and unconfirmed follow-camera
     state.
   - Add `scripts/tests/test_drone_model_sdf_contract.py` to guard SDF marker
     ownership, base-model URI normalization, and lidar sensor preservation.
   - Add unit tests for any new scene diagnostics parser/helper functions with
     synthetic `gz topic` outputs.

3. Heavy/integration tests:

   - Run `make host-quality`, which builds the package and runs C++ tests.
   - Run a full 5-minute headless mission:
     `HEADLESS=1 MISSION_CHECK=1 SMOKE_DURATION_S=300 ./scripts/run_city_mvp_host.sh`.
   - Run a bounded GUI diagnostics smoke and validate logs with
     `validate_gazebo_gui_launch_log.py`.
   - Use manual Gazebo viewing only as a final visual check, not as the sole
     verification method.

## Risks and tradeoffs

- Capturing GUI logs adds another log file, but this is low risk and directly
  addresses the current debugging blind spot.
- Gazebo GUI rendering can still fail for host-specific EGL/driver reasons.
  The plan separates server-side model presence from client-side rendering
  logs so those failures are distinguishable.
- A visual-only marker link may require inertial data depending on Gazebo SDF
  validation behavior. If so, use a tiny explicit inertial block and keep the
  link collision-free.
- The ground projection is an approximate visual helper, not a physics object.
  It should not be used as evidence of collision clearance.
- Follow-camera should remain best-effort. A missing GUI tracking state should
  not fail a headless or non-GUI mission, but it should be visible in logs.
- Scene diagnostics can be sparse if a Gazebo topic does not publish within the
  timeout. Treat missing diagnostic samples as warnings unless they contradict
  required runtime facts that are also available elsewhere.
- Scope guardrail: keep the first implementation batch limited to GUI logging,
  GUI/scene diagnostics, SDF wrapper/model-layout fixes, validators, tests, and
  docs. Do not change planner behavior, lidar projection, obstacle inflation, or
  PX4 offboard control in this batch; those changes would make the render
  regression harder to attribute.
- Normalizing `<uri>model://x500</uri>` to bare `<uri>x500</uri>` should match
  PX4 upstream, but it still touches Gazebo resource resolution. Verify with
  `gz sdf -k`, the SDF contract test, GUI log resource checks, and scene
  diagnostics before treating H4 as closed.

## Open questions

- Should `validate_gazebo_gui_launch_log.py` treat known EGL warnings as hard
  failures, or should they remain warnings unless the drone model is absent from
  Gazebo scene/pose diagnostics?
- Should the yellow locator be implemented as a dedicated fixed link, or should
  it be attached as additional visuals directly under the merged `base_link` if
  SDF validation rejects a visual-only link?
- Is the current fixed ground-projection length acceptable for all configured
  flight altitudes, or should a later task replace it with a dynamic Gazebo
  plugin / marker system?
- Do we want the bounded GUI diagnostics smoke to become a Make target, for
  example `make host-sim-gui-smoke`, after the first implementation is stable?
