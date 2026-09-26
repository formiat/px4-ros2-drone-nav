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
| **Urban point-to-point, GUI** — the default, and the demo video | `./scripts/sim_urban_point_to_point_gui.sh` | Gazebo and RViz side by side. One drone takes off from a pad in the DARPA SubT urban world and flies to a goal 400 to 630 m away through rooms, corridors, doorways and two shafts, with no map, no GNSS, no magnetometer and no lidar: a forward stereo pair and two small time-of-flight sensors. In RViz the pink cloud is the obstacle memory it builds as it goes, the cyan fan the current stereo depth, orange the route it is committed to, the magenta sphere the goal. About five to six minutes of flight at 1.5 to 1.8 m/s. |
| **Urban point-to-point, headless** — the acceptance flight | `./scripts/sim_urban_point_to_point_headless.sh` | No windows. The same flight, ending in the mission check: no crash, the goal reached with the drone's **true** position inside 2.0 m of it, the mean speed, and every diagnostic ([`testing.md`](testing.md)). Everything lands in `log/runs/<run-id>/`, the manifest binding the commit, the configuration and the world by hash. |
| **The same flight on the 3D lidar** | `CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar ./scripts/sim_urban_point_to_point_gui.sh` | The lidar on the airframe, a full spherical scan in RViz, memory filling in every direction to about 35 m, and the flight at 2.4 to 2.7 m/s: the drone flies as fast as it can stop inside the range its sensor is guaranteed to have resolved, and the lidar resolves further than the cameras. Localization is a lidar-inertial estimator, still no GNSS. |
| **Either flight with GNSS** | add `LOCALIZATION_PROFILE=gnss` | Nothing visible changes. GNSS only changes what the autopilot fuses as its position; it is the comparison baseline, and the records show it in the manifest and in the truth-at-goal figure. |
| **Cooperative traffic, GUI** | `./scripts/sim_cooperative_traffic_urban_gui.sh` | Four drones in the urban world exchanging flight intents and choosing complementary maneuvers to keep 5 m apart; a spectator camera follows one and moves on if it is lost. Flies the `gnss` profile. **Not flight-verified since the interception missions were removed** (roadmap item 15 waits for it), and this workstation holds two lidar vehicles at real time, not four: expect the simulation to run below real time. |
| **Cooperative traffic, headless** | `./scripts/sim_cooperative_traffic_urban_headless.sh` | The same with the cooperative referee's separation gates instead of windows. |
| **A static-map flight** | `ENABLE_STATIC_MAP=true LIDAR_PROFILE=none ./scripts/sim_urban_point_to_point_gui.sh` | The flight against a precomputed map instead of one built in flight; kept for comparison, not what the project is about. |

The sensor set and the position source are independent switches, so the
four combinations of cameras or lidar with or without GNSS are all one
command away; the estimator is not named when GNSS is off, because each sensor
set has its own ([`localization.md`](localization.md)). The same variables
work on the headless scripts and on the `make sim-*` targets inside
`./scripts/dev_shell.sh`.

## The Worlds Alone

| Scenario | Command | What you see |
|---|---|---|
| **Environment demo** | `ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh` | Only Gazebo, only the world: no PX4, no ROS, no drone, no mission. Fly the free camera through it with the mouse to see where the flights go. |

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
([`roadmap.md`](roadmap.md)) carries, undelivered as of 2026-09-24:

- **item 15** — cooperative traffic over a channel that behaves like radio,
  and a shared frame without GNSS; until then the cooperative scenario above
  is a test bench for the separation algorithm, not a model of the air;
- **item 17** — flight in the dark and over surfaces without texture, with a
  carried light that can fail; today the world has no light source at all
  and every camera flight is lit by a uniform fill;
- **item 18** — smoke, transient obstacles such as a person crossing the
  frame, and a thermal channel; today a moving body stays in the map until
  the sensor looks there again;
- **item 19** — the return home from a goal proven unreachable;
- **item 20** — the simulation slowed on purpose so that four vehicles fit
  the workstation, and a recording of the flight written without a screen.

None of these has a command yet. What exists is in the two tables above.
