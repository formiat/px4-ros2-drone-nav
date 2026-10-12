# PX4 Parameter Sets

The stack flies PX4 in offboard mode; which PX4 parameters it needs differs
between the simulation and a real vehicle (roadmap item 27 block 1,
specification K34).

| Set | Where | For |
| --- | --- | --- |
| Simulation | `scripts/px4_parameter_runtime.sh`: the estimator's sources and the battery exported as `PX4_PARAM_*` before PX4 starts, the speed profile typed into its console | every flight in Gazebo |
| Real vehicle | `drone_city_nav/config/px4/real_vehicle.params` (QGroundControl's format) | a vehicle with this stack on its companion computer |

## What differs and why

| Parameter | Simulation | Real vehicle | Why |
| --- | --- | --- | --- |
| `CBRK_SUPPLY_CHK` | 894281 (check off) | 0 (check on) | the simulated vehicle has no power module to check |
| `COM_RC_IN_MODE` | 1 (joystick only) | 0 (radio) | the simulation has no radio; its evaluation pilot (`scripts/sim_pilot.py`) sends PX4 the commands a radio or a ground station would |
| `NAV_RCL_ACT`, `NAV_DLL_ACT` | PX4's defaults (the simulation sets `NAV_DLL_ACT 0`, as the default) | PX4's defaults | the stack never overrides the autopilot's own reaction to a lost link: it sees it, reports it and lets PX4 act (K29) |
| `COM_OF_LOSS_T` | scaled with the simulation's speed factor by PX4's own startup | 1.0 s | PX4's default |
| Geofence `GF_*` | off | on, as an example the size of the location, holding at the fence | PX4 evaluates the fence on a global position: in a GNSS-denied flight it is not evaluated, and the mission's own flight envelope bounds the flight |
| `SIM_BAT_*` | the simulated battery full (`SIM_BAT_MIN_PCT 100`), or drained over `MAIN_BATTERY_DRAIN_S` | none | PX4's battery simulator reports neither current nor time left and drained to its 50 % floor in a minute; the mission's return on the main battery (K32) is flown with it drained on purpose |
| `BAT1_*`, `BAT_*_THR`, `COM_LOW_BAT_ACT` | PX4's defaults | the battery's cells and capacity, PX4's thresholds and its return/land action | with a capacity PX4 estimates the time left the mission weighs against the way home |
| `EKF2_*` | the localization profile's (`LOCALIZATION_PROFILE`) | the same sources: external odometry, no GNSS, no magnetometer | the estimators of the stack are the vehicle's position source |
| `COM_DISARM_LAND` | PX4's default, 2 s | 2 s | every mission ends with a landing and this disarm (K30) |

## The pilot and the stack

The stack never arms the vehicle and never switches its mode (K29): the pilot
does, with the radio or a ground station, and in the simulation the
evaluation pilot does with the same PX4 commands. The stack flies the vehicle
while it is armed in offboard; a mode the pilot chooses, or the autopilot's
own failsafe, wins and is never undone by the stack.
