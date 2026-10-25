# Scenarios

Everything this repository can fly or show, one command each, for someone who
has just cloned it and wants to know what there is to look at. Every command
runs from the repository root on a Linux host with Docker, git and the NVIDIA
container runtime, after one bootstrap:

```bash
./scripts/bootstrap.sh --no-run
```

That builds the dev image, PX4 SITL and the workspace and fetches the urban
environment; the first time it downloads several gigabytes and builds for
tens of minutes, and it is skipped thereafter. Stop any scenario, including
one that hung, with:

```bash
./scripts/stop_sim.sh
```

Run one scenario at a time. A second simulator beside the first competes for
the GPU and the real-time factor, and the flight it measures is not the
flight the stack would fly alone.

## The Flights

| Scenario | Command | What you see |
|---|---|---|
| **Urban point-to-point, GUI** — the default, and the demo video | `./scripts/sim_urban_point_to_point_gui.sh` | Gazebo and RViz side by side. One drone takes off from a pad in the DARPA SubT urban world and flies to a goal 400 to 630 m away through rooms, corridors, doorways and two shafts, with no map, no GNSS, no magnetometer and no lidar: a forward stereo pair and six small time-of-flight sensors. Since roadmap item 17 the location has no light of its own: the drone carries a light over the pair's field, which flickers every 5 to 30 s (specification A10). In RViz the pink cloud is the obstacle memory it builds as it goes, the cyan fan the current stereo depth, orange the route it is committed to, the magenta sphere the goal. About five to six minutes of flight at 1.5 to 1.8 m/s. |
| **Urban point-to-point, headless** — the acceptance flight | `./scripts/sim_urban_point_to_point_headless.sh` | No windows. The same flight on the clear location, no smoke anywhere (specification F4, the owner's rule of 2026-10-24; from 2026-10-17 to 2026-10-24 it carried the location's smoke), the route volume the mission check asks the vehicle to cross and the memory snapshot the west hall's, ending in the mission check: no crash, the goal reached with the drone's **true** position inside 2.0 m of it, the mean speed, and every diagnostic ([`testing.md`](testing.md)). Everything lands in `log/runs/<run-id>/`, the manifest binding the commit, the configuration and the world by hash. |
| **Return home from a goal outside the location** — roadmap item 19's flight; **set aside** by the owner on 2026-10-08, neither flown nor recorded until item 19's proof is flown in item 18's blocking scenario (roadmap item 24) | `./scripts/sim_urban_return_home_headless.sh`, `./scripts/sim_urban_return_home_gui.sh`; on the lidar `./scripts/sim_urban_return_home_lidar_headless.sh`, `./scripts/sim_urban_return_home_lidar_gui.sh` | The acceptance flight with the goal placed behind the location's outer walls. The mission monitor floods the obstacle memory from the drone every 10 s; when the goal is given up (`GOAL_UNREACHABLE` in the log) it replaces the goal with the start, and the drone flies home back along the trail it flew out on and holds there as at any goal. On the stereo set the goal is given up by the light's battery (`trigger=battery`): the scenario carries 720 s of light, so the drone explores for two to three minutes and is home within five or six. On the lidar, which carries no light, the goal must be given up by the proof, and in this location the proof does not close ([`technical_debt.md`](technical_debt.md)): the lidar flight explores until its window ends. The check counts the return as the outcome asked for because the manifest records the injection, and it confirms from the location's truth grid (the voxelized collision world, [`environment_candidates.md`](environment_candidates.md)) that no way leads from the start to that goal. The truth grid is a local artifact the headless command checks for first. |
| **The same flight on the 3D lidar** | `./scripts/sim_urban_point_to_point_lidar_gui.sh`, `./scripts/sim_urban_point_to_point_lidar_headless.sh` | The lidar on the airframe, a full spherical scan in RViz, memory filling in every direction to about 35 m, and the flight at 2.4 to 2.7 m/s: the drone flies as fast as it can stop inside the range its sensor is guaranteed to have resolved, and the lidar resolves further than the cameras. Localization is a lidar-inertial estimator, still no GNSS. |
| **The carried light fails** — roadmap item 17 | `./scripts/sim_urban_light_failure_headless.sh`, `./scripts/sim_urban_light_failure_gui.sh` | The dark flight with the drone's light failing worse and worse: outages from the twentieth second of flight, growing to 4.5 s of dark every 8 s, never announced to the drone. It stops when it cannot see, judges its light unreliable from its frames (`GOAL_UNREACHABLE trigger=unreliable_light` in the log) and flies home, about a minute in. |
| **A low battery at launch** — roadmap item 17 | `./scripts/sim_urban_low_battery_headless.sh`, `./scripts/sim_urban_low_battery_gui.sh` | The dark flight with 240 s of light in the battery, too little to reach B. The drone flies towards B while its charge covers the way home with a margin (specification K11), turns when it no longer does (`trigger=battery`, some 70 s in) and returns. |
| **A long flight under failures** — roadmap item 17; **set aside** by the owner on 2026-10-08, neither flown nor recorded without the owner's word | `./scripts/sim_urban_long_failures_headless.sh`, `./scripts/sim_urban_long_failures_gui.sh` | B and back to the start, under the moderate flicker and the camera stream's failures: 0.2 to 1 s of frames lost, or frames handed over 0.1 to 0.5 s late for 1 to 3 s, every 5 to 30 s. It ends at the start either way: through B, or with B given up for a light judged unreliable (`GOAL_UNREACHABLE trigger=unreliable_light`). |
| **The light lost** — roadmap item 17 | `./scripts/sim_urban_light_lost_headless.sh`, `./scripts/sim_urban_light_lost_gui.sh` | A short flight to B in which the light is all but gone from the forty-fifth second: outages of a minute to the dark a second apart, the harshest the owner's range allows, with the stream failing too. The drone stops where it cannot see, holds on dead reckoning, judges its light unreliable, and after three seconds of dead reckoning descends level at 0.5 m/s onto whatever floor is below it. The check asks only that it stays whole. |
| **The clear location** — the reference without smoke | `./scripts/sim_urban_clear_headless.sh`, `./scripts/sim_urban_clear_gui.sh` | The ordinary flight under its reference name of 2026-10-17 to 2026-10-24, when the ordinary flight carried the location's smoke: since 2026-10-24 the two are the same flight (`SMOKE_ZONES=""`, F4, the owner's rule). |
| **The dense plumes alone** — roadmap item 18's measurement | `./scripts/sim_urban_smoke_headless.sh`, `./scripts/sim_urban_smoke_gui.sh` | **Not flown since 2026-10-24** (the owner's rule, F4: no smoke flight; the command stays as the record of the measurement). The ordinary flight with four plumes of white smoke (specification A13) beside the route where the location is wide, one on the route in the start hall to fly round, none closing the way: the start hall at (8,27) and (12,22), the tunnel's mouth at (18,-10), the east hall at (68,-20), 1/m. The pair sees nothing through them, the ring and the lidar return from them as fail evidence (K37, K38), the braking contract reads the collapse of its range, and the stereo set sees a plume's face as an obstacle (K44); the mission check fails a true position inside a plume (R7). |
| **Smoke closes the way** — roadmap items 18 and 24 | `./scripts/sim_urban_smoke_blocked_headless.sh`, `./scripts/sim_urban_smoke_blocked_gui.sh` | **Not flown since 2026-10-24** (the owner's rule, F4: no smoke flight; the command stays as the record of the measurement). One plume of 3/m fills the north corridor at (4,43), 4.4 m wide: no route to B exists. On the lidar the plume is in the memory from outside (K37) and item 19's proof gives the goal up (`trigger=topological`); on the stereo set the plume is the frame's largest lit silence that comes nearer and grows as the vehicle flies toward it (K44), its face marked in the memory from 1.5 to 2.5 m before it, the halo too, the corridor closes and the planner searches the location for another way; the stereo set cannot prove there is none (its lamp reaches 5 m, the proof's flood runs through the halls it has not seen), so, as the goal-outside flight does (A8), it launches with 720 s of light and comes home on its charge (K11, `trigger=battery`). The vehicle has no rung, hold or return of its own for smoke (the owner, 2026-10-18; K40 withdrawn) and never enters the plume in truth (R7). The way home is the way it came. |
| **The moderate rooms alone** — roadmap item 23's measurement | `./scripts/sim_urban_smoke_rooms_headless.sh`, `./scripts/sim_urban_smoke_rooms_gui.sh` | **Not flown since 2026-10-24** (the owner's rule, F4: no smoke flight; the command stays as the record of the measurement). The ordinary flight with two rooms filled wall to wall with moderate smoke, 0.1/m (a visibility of about 30 m): the west hall at (-20,22.5), which the circuit's way to B crosses, and the tunnel's mouth at (18,-6), which the way passes by; the rest of the location clear (two rooms on the way carried the stereo set's estimator 6 to 7 m off by B, r1664). The vehicle flies through them: the braking contract ties its speed to the range the frame measures (K9), the lidar reads the walls through the smoke as its last return (A13), and the memory of the west hall is read against the truth after the flight (`tools/room_memory_truth.py`). The mean flight speed through them is the requirement under smoke. |
| **The large dark hall** — roadmap item 29 | `./scripts/sim_dark_hall_headless.sh`, `./scripts/sim_dark_hall_gui.sh` | A world of its own (`environments/fixtures/dark_hall`): a closed hall 80 x 60 m and 20 m high with no light, the pad 4 m from the west wall, the vehicle facing it as it stands, and the goal 64 m east at 6 m. At that height the carried light reaches the floor only at the bottom edge of the frame and nothing ahead: empty darkness, which a camera vehicle's frames cannot tell from a failed light. The ordinary flight's moderate flicker flies with it. Accepted on 2026-10-14 (r1488 to r1490, the videos in `log/videos/2026-10-14`). |
| **The dark void** — roadmap item 31 | `./scripts/sim_dark_void_headless.sh`, `./scripts/sim_dark_void_gui.sh` | The same hall crossed at 12 m: the carried light reaches neither the floor 12 m below nor the ceiling 8 m above, the pair matches nothing and the tracker holds nothing (specification K46), and the vehicle flies on through the void it measured. The goal by the east wall, 4.5 m from it at 12 m (`dark_void_point_to_point_scenario.json`: a wall within the light's reach at the end, the goal within 2 m by the truth as always); `DARK_VOID_SCENARIO_PATH=drone_city_nav/config/dark_void_mid_point_to_point_scenario.json` puts it in the hall's middle, an arrival on dead reckoning alone, judged by the truth within the estimator's published uncertainty (two sigma) plus 2 m. |
| **The camera flight with GNSS** | `./scripts/sim_urban_point_to_point_gnss_gui.sh`, `./scripts/sim_urban_point_to_point_gnss_headless.sh` | Nothing visible changes. GNSS only changes what the autopilot fuses as its position; it is the comparison baseline, and the records show it in the manifest and in the truth-at-goal figure. |
| **Cooperative traffic, GUI** | `./scripts/sim_cooperative_traffic_urban_gui.sh` | Four drones in the urban world exchanging flight intents and choosing complementary maneuvers to keep 5 m apart; a spectator camera follows one and moves on if it is lost. Flies the `gnss` profile. **Not flight-verified since the interception missions were removed** (roadmap item 15 waits for it), and this workstation holds two lidar vehicles at real time, not four: expect the simulation to run below real time. |
| **Cooperative traffic, headless** | `./scripts/sim_cooperative_traffic_urban_headless.sh` | The same with the cooperative referee's separation gates instead of windows. |
| **A static-map flight** | `ENABLE_STATIC_MAP=true STATIC_OCCUPANCY_3D_PATH=<occupancy3d> ./scripts/sim_urban_point_to_point_gui.sh` | The flight against a precomputed 3D map instead of one built in flight; kept for comparison, not what the project is about. No environment has such a map yet (roadmap item 11), and the request refuses to start without one. |

Every scenario is a named script, and inside `./scripts/dev_shell.sh` a
`make sim-*` target of the same name. Each named scenario sets only what makes
it that scenario and runs the point-to-point mission's own target, so the
flights differ in nothing but what their names say. The switches underneath
(`CAMERA_PROFILE`, `NAVIGATION_SENSOR_PROFILE`, `LOCALIZATION_PROFILE`,
`MISSION_GOALS_XYZ_M`, `SMOKE_DURATION_S`, and
`BLANK_PANELS="x,y,z,yaw,width,height;..."` (roadmap item 18 stage 0b's test object: across the route at `8,21,9.5,1.5708,3,2`, beside it at `8,23,9.5,0,3,2`; `tools/panel_memory.py` reads what the memory held of one), uniform matte
panels for roadmap item 17 stage 2) still work on any of them, for a
combination no name covers; the estimator is not named when GNSS is off,
because each sensor set has its own ([`localization.md`](localization.md)).

## The Worlds Alone

| Scenario | Command | What you see |
|---|---|---|
| **Environment demo** | `./scripts/sim_environment_demo.sh` (the urban world; another with `ENVIRONMENT_DEMO_ID=<id>`) | Only Gazebo, only the world: no PX4, no ROS, no drone, no mission. Fly the free camera through it with the mouse to see where the flights go. |

The IDs, from [`environment_candidates.md`](environment_candidates.md):
`urban_circuit_practice_01` (the one every flight uses),
`cave_circuit_practice_01`, `finals_prize_round_world_07`,
`tunnel_circuit_practice_01`, `cave_world`, `industrial_warehouse`,
`aws_robomaker_small_warehouse`. The first three are versioned release
artifacts and load without further steps; the rest are evaluation candidates
that need their source assets cached and say so if they are absent. Only the
urban world has been flown.

## The Pilot, The Operator And Injected Failures

Every flight ends landed and disarmed: after the goal, or after the way home,
the vehicle holds its position and descends until PX4 lands and disarms it
(specification K30). The stack never arms the vehicle and never switches its
mode (K29): in the simulation an evaluation pilot, `scripts/sim_pilot.py`,
does it with the commands a radio or a ground station sends PX4, once at the
start, and its record lands in `log/runs/<run-id>/pilot.csv`. These switches
work on any scenario above; they are evaluation knowledge the vehicle never
receives:

| Switch | What it does |
|---|---|
| `PILOT_EVENTS="ACTION@SECONDS;..."` | the pilot's actions on the autopilot's clock: `hold`, `posctl`, `land`, `kill` (the motors killed in flight), `rc_loss` (the radio falls silent); and the operator's commands on `/drone_city_nav/operator_command` (K31): `cmd:stop`, `cmd:land`, `cmd:home`, `cmd:goal:X,Y,Z`. Example: `PILOT_EVENTS="cmd:stop@80;cmd:goal:0.749,21.247,9.6@100"` |
| `PILOT_RC=1` | a radio heard from the start (the pilot's sticks centred), so that `posctl` is possible and `rc_loss` is a loss |
| `MAIN_BATTERY_DRAIN_S=SECONDS` | PX4's simulated battery drains from full to 5 % over that time; the mission weighs it against the way home and returns (K32). Without it the battery stays full |
| `IMU_FAULTS=holes` | holes in the autopilot's IMU stream as the camera estimator receives it, 2 s at 60 s and 4 s at 120 s; the estimator starts again on the autopilot's pose after each (K28) |
| `LIGHT_ZONES="x,y,z,radius,falloff;..."` | the carried light out inside a radius and fading over the falloff around world points ([`camera_perception.md`](camera_perception.md)); `2,42.9,9.4,3,0.3` cuts it 0.2 m from a wall, the loss beside a wall |
| `SMOKE_ZONES="x,y,z,sx,sy,sz,sigma;..."` | boxes of white smoke in the world frame, sigma their extinction coefficient per metre (the visibility about 3/sigma m); the ordinary flight sets none (the clear location, F4, the owner's rule of 2026-10-24; no smoke flight is flown), the smoke scenarios set their own; read by every sensor through the smoke model and drawn in the world half of a recording, not in RViz (specification A13, K26, [`obstacle_mapping.md`](obstacle_mapping.md)); `SMOKE_SEED` the seed of their form. Not with `STREAM_FAULTS`, which relays the pair's images too |
| `SMOKE_FACE=true` | the stereo set's smoke face detector on (specification K44): the depth node lays a dense plume's face in the memory and bounds the motion before it. Off by default since 2026-10-24 (the owner's rule): the ordinary flight carries no smoke detection |

A GNSS-denied PX4 refuses `hold` (its Hold mode needs a global position);
the pilot's means there are `posctl` with a radio, `land` and `kill`.

## Reading A Flight Afterwards

A headless flight writes `log/runs/<run-id>/`: the manifest, the mission
check's verdict, the resource record, the estimator's health, the planner's
and controller's diagnostics and the true trajectory from the simulator. A
GUI flight writes the simulator's output to `log/gz_drone_nav.log` and its
window's to `log/gz_gui_drone_nav.log`, which is where to look if no window
appears ([`troubleshooting.md`](troubleshooting.md)). The GUI flight keeps
Gazebo and RViz open after the result so the world can be inspected.

## Recording A Flight With Nobody At The Desk

`./tools/record_flight.sh SCENARIO RUN DIRECTORY` flies one scenario on the
stereo set as its headless flight with the Gazebo window and RViz open, and
leaves two split videos in `DIRECTORY`: the 3D world on the left, RViz on
the right, one with the third-person view and one with the top-down view.
A video is the whole flight, from the windows standing to the flight's end,
at the flight's own pace, never cut and never shorter than a minute. `./tools/record_batch.sh DIRECTORY PASSES FIRST_RUN` flies
every scenario above on the stereo set round robin and writes `index.md` beside the videos.
Nobody moves a view, and nothing but the two pictures is in the frame. A
recorded flight is flown at a real-time factor of 0.6 (`REAL_TIME_FACTOR`),
which the reference workstation holds beside the windows; one the host's
verdict counts is a flight like any other
([`specification.md`](specification.md) A7, A11, K25;
[`roadmap.md`](roadmap.md) item 20). The world's half is as dark as the
location is, and flights on the lidar, which carries no light, are not
recorded.

`./tools/render_flight.sh SCENARIO RUN DIRECTORY` makes the same two files without slowing the flight: the scenario flies as its ordinary headless flight at real time with no window, recording what RViz shows, and the pictures are drawn afterwards, the world held paused with the vehicle moved through its true poses and its light as it was, RViz playing the record; the rotors stand still in the rendered world. It is how a flight is recorded since 2026-10-13 (specification A11).

Whenever RViz is open, the evaluation overlay ([`rviz.md`](rviz.md),
specification K26) writes over the vehicle what it has decided and draws its
estimate against the truth, so that each scenario reads without a word:

| Scenario | What the overlay shows |
|---|---|
| Ordinary dark flight | `TO GOAL` to `GOAL REACHED`; the light's bar dips and recovers with the moderate flicker, and the speed with it; the two trails lie on each other |
| The carried light fails | the bar dips longer and longer, then `LIGHT UNRELIABLE -> HOME` in amber and the way back along the trail |
| The light lost | both bars fall to zero (`SEES` to 4 %), `DEAD RECKONING` in red, `LEVEL DESCENT`, `LANDED`; the white estimate and the green truth part by the drift of the reckoning, and the vehicle is down before the drift reaches a wall |
| A low battery at launch | the charge counts down beside the bar; `BATTERY -> HOME` when it no longer covers the way home with its margin |
| A long flight under failures (set aside) | `TO GOAL`, `GOAL REACHED`, then `TO GOAL` again for the start, under the flicker and the stream's failures |
| A goal outside the location (set aside) | the exploration, then `BATTERY -> HOME` and the way back |

## What Is Planned And Not Yet Runnable

So that nobody searches for a script that does not exist. The roadmap
([`roadmap.md`](roadmap.md)) carries, undelivered as of 2026-10-03:

- **item 15** — cooperative traffic over a channel that behaves like radio,
  and a shared frame without GNSS; until then the cooperative scenario above
  is a test bench for the separation algorithm, not a model of the air;
- **item 18** — smoke and transient obstacles such as a person crossing the
  frame (its thermal channel was measured and set aside: the core returns with
  item 32); today a moving body leaves a trail in the map until the sensor
  looks there again (the memory forgets nothing by time, K10);
- **item 23** — rooms filled with moderate smoke the vehicle flies through,
  one of them on the way in every flight;
- **item 24** — item 19's proof flown: B physically unreachable and given
  up by the proof, not by the battery; closed by item 18's blocking
  scenario;
- **item 20** — the simulation slowed on purpose so that four vehicles fit
  the workstation and a recorded flight counts as an acceptance one (the
  recording itself is built, above).

None of these has a command yet. What exists is in the two tables above.
