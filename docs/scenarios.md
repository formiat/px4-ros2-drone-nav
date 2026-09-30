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
| **Urban point-to-point, GUI** — the default, and the demo video | `./scripts/sim_urban_point_to_point_gui.sh` | Gazebo and RViz side by side. One drone takes off from a pad in the DARPA SubT urban world and flies to a goal 400 to 630 m away through rooms, corridors, doorways and two shafts, with no map, no GNSS, no magnetometer and no lidar: a forward stereo pair and six small time-of-flight sensors. Since roadmap item 17 the location has no light of its own: the drone carries a light over the pair's field, which flickers every 5 to 30 s, and a zone that fails the light lies in a corner no way to B passes (specification A10). In RViz the pink cloud is the obstacle memory it builds as it goes, the cyan fan the current stereo depth, orange the route it is committed to, the magenta sphere the goal. About five to six minutes of flight at 1.5 to 1.8 m/s. |
| **Urban point-to-point, headless** — the acceptance flight | `./scripts/sim_urban_point_to_point_headless.sh` | No windows. The same flight, ending in the mission check: no crash, the goal reached with the drone's **true** position inside 2.0 m of it, the mean speed, and every diagnostic ([`testing.md`](testing.md)). Everything lands in `log/runs/<run-id>/`, the manifest binding the commit, the configuration and the world by hash. |
| **Return home from a goal outside the location** — roadmap item 19's flight | `./scripts/sim_urban_return_home_headless.sh`, `./scripts/sim_urban_return_home_gui.sh`; on the lidar `./scripts/sim_urban_return_home_lidar_headless.sh`, `./scripts/sim_urban_return_home_lidar_gui.sh` | The acceptance flight with the goal placed behind the location's outer walls. The mission monitor floods the obstacle memory from the drone every 10 s; when the goal is given up (`GOAL_UNREACHABLE` in the log) it replaces the goal with the start, and the drone flies home and holds there as at any goal: about 1 km out and back. The check counts the return as the outcome asked for because the manifest records the injection, and it confirms from the location's truth grid (the voxelized collision world, [`environment_candidates.md`](environment_candidates.md)) that no way leads from the start to that goal. The truth grid is a local artifact the headless command checks for first. |
| **The same flight on the 3D lidar** | `./scripts/sim_urban_point_to_point_lidar_gui.sh`, `./scripts/sim_urban_point_to_point_lidar_headless.sh` | The lidar on the airframe, a full spherical scan in RViz, memory filling in every direction to about 35 m, and the flight at 2.4 to 2.7 m/s: the drone flies as fast as it can stop inside the range its sensor is guaranteed to have resolved, and the lidar resolves further than the cameras. Localization is a lidar-inertial estimator, still no GNSS. |
| **The camera flight in the lit world** — roadmap item 17's comparison | `./scripts/sim_urban_point_to_point_lit_gui.sh`, `./scripts/sim_urban_point_to_point_lit_headless.sh` | The flight as it was before roadmap item 17: the location lit, the carried light steady, no zone. Kept to compare with the ordinary flight, which is dark (below). |
| **The carried light fails** — roadmap item 17 | `./scripts/sim_urban_light_failure_headless.sh`, `./scripts/sim_urban_light_failure_gui.sh` | The dark flight with the drone's light failing worse and worse: outages from the first minute, growing toward a minute each, never announced to the drone. It stops when it cannot see, judges its light unreliable from its frames (`GOAL_UNREACHABLE trigger=unreliable_light` in the log) and flies home. |
| **A low battery at launch** — roadmap item 17 | `./scripts/sim_urban_low_battery_headless.sh`, `./scripts/sim_urban_low_battery_gui.sh` | The dark flight with 240 s of light in the battery, too little to reach B. The drone weighs the charge against the way to B, gives B up while the charge still covers the way home (`trigger=battery`) and returns. |
| **A zone over B** — roadmap item 17 | `./scripts/sim_urban_zone_across_b_headless.sh`, `./scripts/sim_urban_zone_across_b_gui.sh` | The dark flight with a zone that fails the drone's light laid over B itself (the one place in this location a zone closes the way; anywhere else the drone flies around it). The light fades as the drone nears B, it stops where it can no longer see, marks the dark it looks into as a wall, and once the dark around B leaves no room for its body it gives B up (`GOAL_UNREACHABLE trigger=topological`, the proof's verdict `goal_region_closed`) and flies home. |
| **A long flight under failures** — roadmap item 17 | `./scripts/sim_urban_long_failures_headless.sh`, `./scripts/sim_urban_long_failures_gui.sh` | B and back to the start, under the moderate flicker and the camera stream's failures: 0.2 to 1 s of frames lost, or frames handed over 0.1 to 0.5 s late for 1 to 3 s, every 5 to 30 s. It ends at the start either way: through B, or with B given up for a light judged unreliable (`GOAL_UNREACHABLE trigger=unreliable_light`). |
| **The light lost** — roadmap item 17 | `./scripts/sim_urban_light_lost_headless.sh`, `./scripts/sim_urban_light_lost_gui.sh` | A short flight in which the light is all but gone from the forty-fifth second: outages of a minute to the dark a second apart, the harshest the owner's range allows, with the stream failing too. The drone stops where it cannot see, judges its light unreliable, turns back the way it came on its dead reckoning, and when that runs out the autopilot lands it. The check asks only that it stays whole. |
| **The camera flight with GNSS** | `./scripts/sim_urban_point_to_point_gnss_gui.sh`, `./scripts/sim_urban_point_to_point_gnss_headless.sh` | Nothing visible changes. GNSS only changes what the autopilot fuses as its position; it is the comparison baseline, and the records show it in the manifest and in the truth-at-goal figure. |
| **Cooperative traffic, GUI** | `./scripts/sim_cooperative_traffic_urban_gui.sh` | Four drones in the urban world exchanging flight intents and choosing complementary maneuvers to keep 5 m apart; a spectator camera follows one and moves on if it is lost. Flies the `gnss` profile. **Not flight-verified since the interception missions were removed** (roadmap item 15 waits for it), and this workstation holds two lidar vehicles at real time, not four: expect the simulation to run below real time. |
| **Cooperative traffic, headless** | `./scripts/sim_cooperative_traffic_urban_headless.sh` | The same with the cooperative referee's separation gates instead of windows. |
| **A static-map flight** | `ENABLE_STATIC_MAP=true STATIC_OCCUPANCY_3D_PATH=<occupancy3d> ./scripts/sim_urban_point_to_point_gui.sh` | The flight against a precomputed 3D map instead of one built in flight; kept for comparison, not what the project is about. No environment has such a map yet (roadmap item 11), and the request refuses to start without one. |

Every scenario is a named script, and inside `./scripts/dev_shell.sh` a
`make sim-*` target of the same name. Each named scenario sets only what makes
it that scenario and runs the point-to-point mission's own target, so the
flights differ in nothing but what their names say. The switches underneath
(`CAMERA_PROFILE`, `NAVIGATION_SENSOR_PROFILE`, `LOCALIZATION_PROFILE`,
`MISSION_GOALS_XYZ_M`, `SMOKE_DURATION_S`, `WORLD_ILLUMINATION=dark`, and in
the dark world `BLANK_PANELS="x,y,z,yaw,width,height;..."`, uniform matte
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

## Reading A Flight Afterwards

A headless flight writes `log/runs/<run-id>/`: the manifest, the mission
check's verdict, the resource record, the estimator's health, the planner's
and controller's diagnostics and the true trajectory from the simulator. A
GUI flight writes the simulator's output to `log/gz_drone_nav.log` and its
window's to `log/gz_gui_drone_nav.log`, which is where to look if no window
appears ([`troubleshooting.md`](troubleshooting.md)). The GUI flight keeps
Gazebo and RViz open after the result so the world can be inspected.

## What Is Planned And Not Yet Runnable

So that nobody searches for a script that does not exist. The roadmap
([`roadmap.md`](roadmap.md)) carries, undelivered as of 2026-09-27:

- **item 15** — cooperative traffic over a channel that behaves like radio,
  and a shared frame without GNSS; until then the cooperative scenario above
  is a test bench for the separation algorithm, not a model of the air;
- **item 17** — a carried light that fails and a zone that fails it, and
  the vehicle's answers; the dark world, the carried light and the camera
  noise are flown (above);
- **item 18** — smoke, transient obstacles such as a person crossing the
  frame, and a thermal channel; today a moving body stays in the map until
  the sensor looks there again;
- **item 20** — the simulation slowed on purpose so that four vehicles fit
  the workstation, and a recording of the flight written without a screen.

None of these has a command yet. What exists is in the two tables above.
