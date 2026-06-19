# Context/Task

This investigation supplements the previous Gazebo GUI / drone rendering
analysis with the clarified base fact from the user:

- the drone is not merely hard to see;
- the camera is not merely pointed at the wrong place;
- the user did not just miss the model on screen;
- the drone and its ground marker are not rendered/displayed in the Gazebo 3D
  world window, while RViz and logs show that the simulated drone exists and
  flies.

The task is to investigate the real reason for the absence of the drone in the
Gazebo 3D world window, using only headless checks for this round.

# Research Questions

- Is the drone missing from the simulation/server side, or only from the Gazebo
  GUI/render side?
- Does the current SDF resolve the PX4 x500 base model, lidar model, mesh
  visuals, and yellow marker visuals?
- Which recent changes are still relevant to the rendering regression?
- Which previous explanations must be rejected after the user's clarification?
- What remains unresolved because GUI rendering could not be inspected in this
  headless-only round?

# Scope And Constraints

- Repository root:
  `/home/formi/Documents/CppProjects/drone-gazebo`
- Branch at investigation time: `main`
- Starting HEAD: `42671b5 Revise investigation per reviewer feedback`
- GUI runs were intentionally not used. The prompt allowed additional
  investigation only in headless mode.
- No remote systems were used.
- Notion and GitLab protocols were read, but no task id or MR URL was provided,
  so neither Notion nor GitLab was queried.
- The investigation updates the existing `INVESTIGATION.md`; it does not
  implement a code fix.

# Detected Stack/Profiles

Detected stack:

- C++ ROS 2 workspace
- `colcon` / ament CMake
- Gazebo Sim / `gz`
- PX4 SITL
- shell/Python launch helpers
- SDF model and world assets

Profiles and protocols read:

- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/notion_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/gitlab_access_protocol.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/generic.md`
- `/home/formi/Documents/RustProjects/multi-agent-orchestrator-rs/docs/project_profiles/cpp.md`

# Repo-Approved Commands Found

Repository-approved workflow is documented in `AGENTS.md`, `README.md`,
`CONTRIBUTING.md`, and `Makefile`.

Host workflow, preferred for agents:

- `make host-build`
- `make host-test`
- `make host-quality`
- `make host-format`
- `make host-sim-headless`
- `make host-sim-gui`

Container workflow:

- `./scripts/dev_shell.sh`
- inside the container: `make build`, `make test`, `make quality`,
  `make sim-headless`, `make sim-gui`

For this investigation, GUI commands were skipped by prompt constraint. A short
headless run and non-GUI `gz` queries were used.

# Observed Symptom

Verified base fact:

- In Gazebo 3D world, the drone model and the yellow ground marker are not
  rendered/displayed.
- This is not a camera-position explanation and not a "too small to notice"
  explanation.
- The drone exists physically in the simulation, flies, publishes telemetry, and
  is visible through RViz-derived state/logs.

Separate secondary symptom:

- Gazebo GUI follow camera commands are accepted, but state confirmation is not
  available in logs. This affects convenience of viewing, but it is not an
  acceptable explanation for the verified absence of rendered drone visuals.

# Immediate Cause

The immediate cause is narrowed to the Gazebo GUI/render-client boundary:

- PX4 and Gazebo server create the model `x500_lidar_2d_0`.
- `gz model --list` in a headless run lists `x500_lidar_2d_0`.
- `gz model -m x500_lidar_2d_0 -l` lists the expected links:
  `base_link`, `rotor_0`, `rotor_1`, `rotor_2`, `rotor_3`, and `link`.
- The lidar sensor and model topics are present.
- Static SDF expansion with `SDF_PATH` succeeds and contains both PX4 x500 mesh
  visuals and custom yellow marker visuals.

Therefore the current evidence does not support:

- spawn failure;
- wrong start/finish position;
- missing physical model;
- missing lidar link;
- SDF syntax failure;
- user/camera misidentification.

The exact render-client subcause remains unresolved in this headless-only
round. The strongest code-level suspect is still the marker/model composition
around `bbcc6a3`, where marker visuals were moved from a dedicated parent-model
link into the merged `lidar_2d_v2/link`. However, because the expanded SDF still
contains the base x500 mesh visuals, this is a suspect, not a fully proven root
cause.

# Causal Chain / Why Chain

1. User observes that the drone and yellow ground marker are not rendered in the
   Gazebo 3D world window.
2. RViz/logs show the drone exists and flies, so the symptom is not a navigation
   or PX4 spawn failure.
3. A headless Gazebo transport query confirms the server-side model exists:
   `x500_lidar_2d_0` is listed as an available model.
4. The same query confirms expected links and sensors exist on the model.
5. Static SDF expansion confirms that the source model resolves to a complete
   model with PX4 x500 mesh visuals and custom yellow locator visuals.
6. Therefore the failure boundary is after server-side model creation and SDF
   resolution: Gazebo GUI/render-client visual presentation.
7. The most relevant current code-level suspect is the custom visual composition
   introduced by marker changes, especially moving marker visuals into the
   merged lidar child model instead of keeping them in a dedicated link attached
   directly to `base_link`.
8. Follow-camera failure can make viewing inconvenient, but it does not explain
   a model that is absent after manually searching the 3D world.

# Evidence Per Causal Link

| Causal link | Evidence |
| --- | --- |
| The model is spawned by PX4/Gazebo | `log-host/px4_city_mvp.log` contains `Spawning Gazebo model` and `world: generated_city, model: x500_lidar_2d_0`. |
| The server-side model exists | Headless query `gz model --list` listed `x500_lidar_2d_0`. |
| The server-side model has expected links | Headless query `gz model -m x500_lidar_2d_0 -l` listed `base_link`, `rotor_0..3`, and `link`. |
| The lidar link/sensor exists | Headless query listed sensor `lidar_2d_v2` on link `link`; topic list contained `/world/generated_city/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan`. |
| Current wrapper uses custom local model | `scripts/run_city_mvp.sh:208-232` builds runtime resources and symlinks local `x500_lidar_2d` and `lidar_2d_v2`. |
| Current wrapper includes x500 and lidar | `drone_city_nav/models/x500_lidar_2d/model.sdf:3-15`. |
| Custom marker visuals are inside lidar model | `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`. |
| SDF resolves and expands with visuals | `gz sdf -k` with `SDF_PATH` returned `Valid`; `gz sdf -p` output included `base_link_visual`, rotor visuals, `yellow_drone_locator_core`, arms, beam, and disc. |
| PX4 upstream wrapper uses bare `x500` URI | `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf:4-16`. |
| Current wrapper uses `model://x500` | `drone_city_nav/models/x500_lidar_2d/model.sdf:4-8`. Static expansion works with `SDF_PATH`, so this is not currently strong evidence of resource failure. |
| Follow camera is accepted but unconfirmed | `log-host/gz_city_mvp.log` contains three accepted follow commands and `state confirmation is unavailable`. |

# Root Cause / Unresolved Boundary

Confirmed root boundary:

- The drone is present in the Gazebo server simulation.
- The model has expected links and sensors.
- The source SDF is valid when resolved with the appropriate model search path.
- The rendered absence is therefore on the Gazebo GUI/render-client side or in
  the visual-scene transfer/loading path, not in PX4 mission logic or ROS
  navigation.

Unresolved exact subcause:

- Headless mode cannot directly prove which visual nodes the GUI renderer
  receives, culls, fails to load, or fails to draw.
- The investigation could not inspect the actual GUI render tree because GUI
  execution was disallowed for this round.

Most likely code-level suspect:

- `bbcc6a3 Attach drone marker to lidar visual link` moved the yellow marker
  visuals out of a dedicated `visibility_marker_link` in
  `x500_lidar_2d/model.sdf` and into `lidar_2d_v2/link`.
- This made marker rendering depend on the merged child lidar model and mixed
  the locator visuals with the lidar sensor link.
- If Gazebo GUI has a render/client-side issue with these merged child visuals,
  the marker can disappear even while the physical model and sensors exist.
- This does not fully explain why base x500 mesh visuals are also not visible,
  because SDF expansion still contains those visuals. That part remains the main
  unresolved boundary.

Rejected explanations:

- "The user did not look at the right place."
- "The camera is pointed wrong."
- "The drone is too small."
- "The drone spawned at a random/wrong point."
- "The drone does not exist in simulation."
- "PX4 did not spawn the model."

# Sources Checked

- `.agent-io/inbox.txt`
- `AGENTS.md`
- `README.md`
- `CONTRIBUTING.md`
- `Makefile`
- `INVESTIGATION.md` before this update
- `scripts/run_city_mvp.sh`
- `scripts/gazebo_gui_control.py`
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf`
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`
- `log-host/gz_city_mvp.log`
- `log-host/px4_city_mvp.log`
- `log-host/ros_city_mvp.log`
- Headless run logs under
  `log-host/investigate_render_headless_20260618_233648/`
- Git history for relevant commits:
  `fa88422`, `bbcc6a3`, `b4b9e8c`, `41bb70e`, `5e006b2`,
  `1d7be4a`, `d169f72`, `2d054e2`, `718deec`, `cf8b9b1`,
  and current `42671b5`

# Evidence

Commands run:

- `cat /home/formi/Documents/CppProjects/drone-gazebo/.agent-io/inbox.txt`
- `cat` for the required Notion/GitLab/profile documents
- `git status --short --branch`
- `git log --oneline --decorate --date=iso --format='%h %ad %s' -20`
- `rg` over model, launch, and helper files
- `nl -ba` over current and upstream SDF/script files
- `git show --stat --oneline fa88422 bbcc6a3`
- `git show fa88422:drone_city_nav/models/x500_lidar_2d/model.sdf`
- `git diff --stat 718deec..cf8b9b1 -- scripts/run_city_mvp.sh`
- `git diff --unified=80 718deec..cf8b9b1 -- scripts/run_city_mvp.sh`
- `./scripts/host_shell.sh bash -lc 'command -v gz; gz --versions; gz sdf --help; gz model --help'`
- `./scripts/host_shell.sh bash -lc 'export SDF_PATH=...; gz sdf -k drone_city_nav/models/x500_lidar_2d/model.sdf; gz sdf -p ...'`
- Temporary bare-URI SDF comparison with `gz sdf -k` and `gz sdf -p`
- Short headless run:
  `HEADLESS=1 ENABLE_RVIZ=false SMOKE_DURATION_S=45 DRONE_GAZEBO_LOG_DIR=log-host/investigate_render_headless_20260618_233648 ./scripts/run_city_mvp_host.sh`
- During that headless run:
  `gz model --list`,
  `gz model -m x500_lidar_2d_0 -l`,
  `gz model -m x500_lidar_2d_0 -l link -s`,
  `gz topic -l`

One attempted command was invalid:

- `./scripts/host_shell.sh 'command -v gz && ...'`
- Reason: `host_shell.sh` expects command arguments, not a single shell string.
  The correct form is `./scripts/host_shell.sh bash -lc '...'`.

# Evidence References

- `scripts/run_city_mvp.sh:208-232`: runtime model resources are prepared; local
  `x500_lidar_2d` and `lidar_2d_v2` replace PX4 originals.
- `scripts/run_city_mvp.sh:349-360`: `GZ_SIM_RESOURCE_PATH` points Gazebo to the
  runtime model/world directories.
- `scripts/run_city_mvp.sh:393-416`: Gazebo server/headless/GUI launch path.
- `scripts/run_city_mvp.sh:421-435`: PX4 is launched with
  `PX4_GZ_MODEL_POSE` and `PX4_GZ_WORLD`.
- `scripts/gazebo_gui_control.py:146-156`: follow-state confirmation reads
  `/gui/currently_tracked` for only `0.2s`.
- `scripts/gazebo_gui_control.py:159-228`: follow camera exits best-effort
  after accepted commands without confirmed tracking state.
- `drone_city_nav/models/x500_lidar_2d/model.sdf:3-15`: current wrapper include
  and fixed lidar joint.
- `drone_city_nav/models/lidar_2d_v2/model.sdf:77-152`: yellow marker and
  ground projection visuals are currently inside the lidar model.
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf:4-16`:
  upstream PX4 wrapper structure for comparison.

# Findings

1. High confidence: the drone is not missing from the simulation. It is spawned
   as `x500_lidar_2d_0` and exists in the Gazebo server.
2. High confidence: the current model has expected physics/sensor links in the
   server-side state.
3. High confidence: the current SDF can resolve into a complete model with x500
   mesh visuals and yellow marker visuals when the model path is supplied.
4. Medium-high confidence: the reported absence is a GUI/render-client problem,
   not a PX4/ROS/navigation problem.
5. Medium confidence: the marker relocation in `bbcc6a3` is the most relevant
   code-level rendering suspect because it moved visual locator geometry into a
   merged child lidar link.
6. Medium confidence: follow-camera logic is broken or at least unreliable, but
   it is a separate issue and must not be used as the explanation for a model
   that is not rendered after manual inspection.
7. High confidence: the old `718deec -> cf8b9b1` visibility boundary remains
   unresolved by code inspection. That diff changes only cleanup logic executed
   on script exit, so it cannot directly explain live rendering behavior.

# Relevant Code Paths

- `scripts/run_city_mvp.sh`
  - runtime resource setup
  - Gazebo server/GUI launch
  - PX4 model spawn environment
  - follow camera helper invocation
- `scripts/gazebo_gui_control.py`
  - follow camera service call
  - follow offset service call
  - optional `/gui/track` publication
  - state confirmation logic
- `drone_city_nav/models/x500_lidar_2d/model.sdf`
  - wrapper model for PX4 x500 + lidar
- `drone_city_nav/models/lidar_2d_v2/model.sdf`
  - custom lidar model
  - current yellow drone locator and ground projection visuals
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
  - upstream comparison point
- `external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`
  - x500 mesh visual definitions

# Timeline/History

- `fa88422 Add bright drone visibility markers`
  - added a dedicated `visibility_marker_link` directly in
    `x500_lidar_2d/model.sdf`.
- `bbcc6a3 Attach drone marker to lidar visual link`
  - removed the dedicated marker link from `x500_lidar_2d`;
  - added yellow marker and ground projection visuals inside
    `lidar_2d_v2/link`.
- `b4b9e8c Add Gazebo GUI drone follow camera`
  - first follow-camera implementation that previously worked according to user
    history.
- `41bb70e Keep Gazebo GUI simulation unpaused`
  - introduced a GUI config path in that historical period.
- `1d7be4a Restore original Gazebo follow camera launch`
  - removed the GUI config launch path.
- `2d054e2 Stabilize Gazebo launch diagnostics`
  - switched follow/unpause control to `scripts/gazebo_gui_control.py`.
- `42671b5 Revise investigation per reviewer feedback`
  - previous investigation state before this update.

# Hypotheses/Alternatives

## H1: GUI/render-client visual issue with custom merged lidar visuals

Status: plausible, medium confidence.

The marker was moved into `lidar_2d_v2/link`, which is included with
`merge="true"` into `x500_lidar_2d`. If the GUI renderer mishandles these
merged visuals or their transforms/materials, the yellow marker can disappear.
This is the most actionable model-level suspect.

Falsification:

- Move the yellow marker back into a dedicated link in `x500_lidar_2d` attached
  to `base_link`.
- Keep `lidar_2d_v2` close to the upstream sensor model.
- Run GUI and verify marker/drone rendering.

## H2: GUI/render-client resource loading issue for mesh visuals

Status: plausible, medium confidence.

SDF expansion includes mesh URIs such as `model://x500_base/meshes/...`.
Headless server loads the model, but GUI may fail to load/render mesh resources.
The available logs do not currently show a mesh-load error, so this remains
unproven.

Falsification:

- Capture GUI logs with resource/render verbosity.
- Check whether primitive yellow marker visuals render when x500 meshes do not.
- Add a simple primitive visual directly to `base_link` and compare.

## H3: `model://x500` vs bare `x500` URI

Status: weakened, low-medium confidence.

PX4 upstream uses bare `<uri>x500</uri>` while the current local wrapper uses
`<uri>model://x500</uri>`. Static SDF expansion works with `SDF_PATH`, and the
headless server creates the model, so this is not strong enough to explain the
current symptom by itself.

Falsification:

- Switch to the upstream bare URI and compare expanded SDF and GUI behavior.

## H4: Follow-camera failure

Status: real but separate.

Follow-camera state is not confirmed in logs, but this cannot be the root
explanation for a model that is not rendered after manual inspection of the 3D
world.

Falsification:

- Fix follow camera independently and verify whether the drone still fails to
  render.

## H5: User/camera/scale/location mistake

Status: rejected.

This contradicts the clarified base fact and must not be used as an explanation.

# Risk/Impact

- The simulation may be functionally correct while unusable for visual
  inspection in Gazebo 3D.
- RViz remains useful for path/lidar/map debugging, but it does not replace
  Gazebo 3D visual validation.
- If marker visuals are kept inside the lidar child model, future model changes
  may keep mixing sensor configuration with human visibility aids.
- Follow-camera instability can hide or confuse GUI issues, even when it is not
  their root cause.

# Conclusions

- The drone is present server-side and flies.
- The current source SDF is valid and contains visual definitions.
- The verified failure is at the Gazebo GUI/render-client boundary, not at PX4
  spawn or ROS navigation.
- The most practical next model-level fix is to remove locator visuals from the
  lidar child model and attach a dedicated marker/visibility link directly in
  `x500_lidar_2d` to `base_link`.
- Follow-camera should be fixed separately, but not treated as the reason for
  non-rendered drone visuals.

# Recommendations/Next Steps

1. Restore a dedicated `visibility_marker_link` in
   `drone_city_nav/models/x500_lidar_2d/model.sdf`, fixed to `base_link`.
2. Remove yellow locator and ground-projection visuals from
   `drone_city_nav/models/lidar_2d_v2/model.sdf`.
3. Keep the custom lidar model focused on lidar sensor geometry/sensor config;
   keep human visibility aids in the wrapper drone model.
4. Add a simple primitive yellow visual directly attached to the drone wrapper
   so marker rendering does not depend on x500 mesh resource loading.
5. Keep a headless diagnostic script or documented command that verifies:
   model exists, links exist, sensors exist, and scan topics exist.
6. Fix follow camera separately:
   - wait until `x500_lidar_2d_0` appears in `gz model --list`;
   - call `/gui/follow` and `/gui/follow/offset`;
   - remove or isolate `/gui/track` publication until proven necessary;
   - do not report success as "confirmed" unless state is actually observed.
7. After the model change, run a GUI validation manually because headless
   checks cannot prove final rendering.

# Verification/Falsification Steps For Findings

Headless verification already performed:

- Run a short headless simulation.
- Query `gz model --list`.
- Verify `x500_lidar_2d_0` is present.
- Query `gz model -m x500_lidar_2d_0 -l`.
- Verify expected links and sensor exist.
- Query `gz topic -l`.
- Verify lidar topics exist.
- Run `gz sdf -k` and `gz sdf -p` with `SDF_PATH` containing local and PX4 model
  directories.

Remaining GUI-only verification:

- After moving marker visuals back to a dedicated parent-model link, run
  `make host-sim-gui`.
- Confirm that the drone and marker render in Gazebo 3D without relying on
  RViz.
- Confirm that manual camera navigation can see the model.
- Then separately validate follow-camera behavior.

# Follow-Up Verification Implications

- A passing headless mission does not prove Gazebo GUI rendering.
- `gz model --list` and `gz model -m ... -l` are useful to prove server-side
  existence, but they cannot prove GUI visual drawing.
- Any future visual fix must include a GUI check by the user or an automated
  render/screenshot check if we add one later.
- If the dedicated marker link renders but x500 mesh does not, the next focus
  should be mesh/material resource loading.
- If neither dedicated primitive marker nor x500 mesh renders, the next focus
  should be Gazebo GUI scene subscription/render state, not the model SDF.

# Open Questions

- Does Gazebo GUI receive the `x500_lidar_2d_0` visual scene entries but fail to
  draw them, or are those visual entries absent from the GUI scene?
- Do primitive marker visuals render if attached directly to `base_link` in the
  wrapper model?
- Do x500 mesh visuals fail independently from the marker visuals?
- Does `/gui/track` publication interfere with `/gui/follow`, or is follow
  camera failure purely a timing/state-confirmation problem?
- Can we add an automated offscreen GUI render/screenshot check later without
  making normal development dependent on a visible GUI?
