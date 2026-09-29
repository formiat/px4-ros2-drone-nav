# Specification: Requirements, Invariants And Decisions

The project's technical specification in one place: the requirements a flight
is judged by, the invariants the navigation keeps, the conditions of the runs
that accept a change, and the numbers the work relies on. Every entry says who
set it:

- **Owner** — stated or decided by the project owner;
- **Agent** — decided by an agent (a person or an AI assistant working on the
  repository) without the owner's validation, with the objective justification
  written beside it;
- **Inherited** — present in the code or the documents before this file
  existed (2026-09-27), its source not recorded; the owner classifies it when
  it is next touched.

**The rules of the file.** An agent may add an entry and may change an Agent
or Inherited entry when a measurement or the code demands it. Every change is
recorded in the change log at the end — the entry, the old and the new value,
the justification, the commit — and reported to the owner in the same working
session; a change without a line in the log is not made. An Owner entry is
changed by the owner, or by an agent: the owner allowed agents on 2026-09-27
to change Owner entries too, recorded and reported the same way, and the log
line says so. Only objective, important figures belong here; small tuning
parameters stay in the code with their comments. A value that exists both here
and in code or configuration is the code's value; this file says why it is
that value, and a change to one is a change to the other.

Read this file before planning work on navigation, acceptance or a roadmap
item, and update it in the same commit as the change it records. The
roadmap ([`roadmap.md`](roadmap.md)) holds what is planned, the register
([`technical_debt.md`](technical_debt.md)) what is known to be wrong and set
aside, and this file what everything else is measured against.

## Requirements

| ID | Statement | Source, since | Justification, reference |
|---|---|---|---|
| R1 | The vehicle never crashes: no crash event and no contact with a static obstacle. | Owner, 2026-09-19 | The first of the project's two requirements ([`testing.md`](testing.md)). |
| R2 | The vehicle always reaches its goal, judged in truth: at every goal acknowledgement its true position is inside the 2.0 m capture radius of the goal. | Owner, 2026-09-19 | The mission monitor judges by the estimate, which drifts; the truth is read by the check and never in the control loop. |
| R3 | The mean flight speed exceeds 1.2 m/s on the stereo set and 2.4 m/s on the 3D lidar, both without GNSS, measured on the simulation clock between mission readiness and the result. | Owner, 2026-09-19; simulation clock 2026-09-25 | The second requirement; the only speed target. |
| R4 | Nothing else fails a flight: every other measurement of the check is a note. | Owner, 2026-09-19 | [`testing.md`](testing.md). |
| R5 | What a series shows to be wrong is repaired with its measured cause; debt holds only what is (a) very hard, (b) in need of a deep rework, or (c) in need of the owner's decision, and says which. | Owner, 2026-09-19 | [`technical_debt.md`](technical_debt.md). |

## Navigation Invariants

| ID | Statement | Source, since | Justification, reference |
|---|---|---|---|
| I1 | Space the sensor has not looked at — unknown, in the sense of unexplored: "not seen because never looked at" — is traversable at no penalty: no cost, no inflation, no gate. The protection in unknown space is the braking contract, not the map. | Owner | Otherwise the vehicle is confined to its sensor's radius. |
| I2 | Space is prohibited only by measurement: observed occupied, or observed unobservable (the sensor looked and its measured range stayed below the physical margin: smoke, darkness, glare). Both decay when not confirmed and lift on re-observation. No prohibition comes from configuration, from knowledge of the location or from the vehicle's history; no cost on free observable space. | Owner, 2026-09-23 | Replaces the earlier ban on prohibited zones. |
| I3 | No latch, hold or release gate keyed on the vehicle's history; every hard prohibition keeps the exit guarantee: the vehicle's own position and its observed path are never closed. Braking to rest on a validated finite trajectory is allowed. | Owner, 2026-09-04 | An indefinite stall at a wall is a failure of R2. |
| I4 | Vertical motion is free in the planner's time model. | Owner, 2026-09-01 | The vehicle must take openings and shafts below it. |
| I5 | Production code does not adapt to a location: no named world, spawn or opening altitude, opening coordinates or passage identity. Scenario coordinates are regression inputs only. | Owner | Generality of the navigation. |
| I6 | Safety and the quality of the flight come before speed; a collision is a defect; safety is not bought by a broad reduction of speed. | Owner | Paradigm of the project. |
| I7 | The vehicle does not know how its light fails: nothing of the emitter's failures, its driver or an injector reaches the navigation; it concludes about the illumination from its frames only. It does know the light's battery charge, as any airframe knows its batteries. | Owner, 2026-09-24; the battery 2026-09-27 | Roadmap item 17 stage 5. |
| I8 | The vehicle has no time limit: no budget of flight time decides its mission. | Owner, 2026-09-27 | The time-bound return in the mission monitor (K4) was never meant as one; removed with item 17 stage 5 (2026-09-29). |

## The Vehicle, The Missions And Their Defaults

| ID | Statement | Source, since | Justification, reference |
|---|---|---|---|
| V1 | The default sensor set is the stereo pair with the time-of-flight sensors, no lidar; the 3D lidar on request. | Owner, 2026-09-19 | Item 14. |
| V2 | The default localization is `visual_inertial` (VIO), without GNSS or magnetometer; the lidar profile uses `lidar_inertial` (LIO); `gnss` only on request, and the multi-vehicle launches stay on it until item 15. | Owner, 2026-09-20 | Item 16. |
| V3 | The speed profile: cruise 6.5 m/s, absolute limit 10 m/s, horizontal acceleration 4 m/s², the same with and without a static map. | Owner; cruise raised from 5 on 2026-09-03 by the owner's permission | The braking ceiling admits no more at 30 m of guaranteed lidar range. |
| V4 | Two missions remain, urban point-to-point and cooperative traffic, on Urban Circuit Practice 01. | Owner, 2026-09-17 | The interception missions, the radar and the grid city were removed. |
| V5 | The autopilot fuses the external odometry with a fixed noise of 0.3 m and 0.05 rad, not the estimator's variances. | Inherited (item 16) | r561: at 0.1 m a 0.4 m correction was gated out and the flight lost. |
| V6 | The capture radius of a goal is 2.0 m (`mission_goal_capture_radius_m`). | Inherited | The radius R2 is judged by. |

## Acceptance And Runs

| ID | Statement | Source, since | Justification, reference |
|---|---|---|---|
| A1 | A change is accepted by five flights on the stereo set, then five on the 3D lidar, both without GNSS, on one commit, each inspected before the next; a commit between flights restarts the series; a failure is repaired with its measured cause and the series flown again. | Owner, 2026-09-19 | |
| A2 | No cooperative flights until roadmap item 15 closes; after it, five single and five cooperative flights. | Owner, 2026-09-18 | |
| A3 | Flights strictly one at a time, `./scripts/stop_sim.sh` before and after each. | Owner, 2026-09-07 | Parallel or orphaned simulators distort the results. |
| A4 | No flight under foreign load on the host; foreign processes are left alone; a flight under load is voided and flown again. | Owner, 2026-09-25 | |
| A5 | Build, test and simulation in the container only; `make format`, `make build`, `make test-scripts` and `make quality` green before every commit. | Owner | [`CONTRIBUTING.md`](../CONTRIBUTING.md). |
| A6 | A flight whose failure coincides with a freeze of the whole container (a gap of 2 s or more in the resource sampler's one-second record) is voided as flown under foreign load, and flown again. | Agent, 2026-09-27 | Another task's disk writes froze every process for 3.7 and 7.3 s while the simulation ran on; the camera estimator took IMU holes of 1.9 and 3.6 s and both flights crashed (r720, r723). The quiet-host gate of the flight script reads CPU load only and did not see it. |
| A7 | A flight whose real-time factor lies noticeably below its profile's norm is not counted: 0.82 to 1.00 on the stereo set, 1.00 on the lidar. | Agent, 2026-09-25 | The norms of the series on 173155d6. |
| A9 | A contact made by the body (not a rotor), level (roll and pitch under 15 degrees), under 1.0 m/s and under 0.5 m/s horizontally is a landing, not a crash; a vehicle that disarms before its mission's end has landed, which ends the mission (`vehicle_landed`) as a failure to reach its goal. | Agent, 2026-09-29 | Roadmap item 17 stage 5's ladder ends in a landing when no position is left, and its short flights under the harshest failures accept a vehicle landed or flying; the autopilot's blind landing met the floor at 0.73 m/s and was counted a crash (r790). |
| A8 | The run's window (`SMOKE_DURATION_S`) is the test harness's wall-clock timeout only; nothing of the vehicle reads it. Acceptance flights fly an 1800 s window, the goal-outside flights of item 19 an 1800 s one as well. | Agent, 2026-09-29 | Since the time-bound return is gone (I8) a window no longer decides a return; ordinary flights took up to 424 s and the doubled path of item 19 1000 to 1500 m, and the camera flights in the dark run slower. |

## Decided For Items Not Yet Built

| ID | Statement | Source, since | Justification, reference |
|---|---|---|---|
| F1 | Once roadmap item 17 lands, a moderate flicker of the carried light runs in every acceptance flight, its dark stretches never long enough to reach the "unreliable" judgment, and the mean speed of R3 is measured under it; the speeds several flights under it show are adopted as the current requirement. | Owner, 2026-09-27 | The series show that the vehicle flies normally with it; a lower speed is expected. |
| F2 | A severe failure of the light, worsening until the vehicle judges it unreliable and flies home, is a scenario of its own. | Owner, 2026-09-27 | Roadmap item 17 stage 5. |
| F5 | Once roadmap item 17 lands, at least one zone that fails the light as the vehicle approaches it (the "magnetic anomaly") is in every flight, placed anywhere it does not block the way to B, and the vehicle does not enter its darkness; a separate scenario lays the zone across the way to B and the vehicle flies home. | Owner, 2026-09-27 | Roadmap item 17 stage 7. |
| F6 | Darkness is not unknown. Unknown, that is unexplored ("not seen because still far"), is free at no penalty and explored; darkness ("not seen although close enough to see") is observed unobservable and a prohibition equal to a physical obstacle. Roadmap item 17 teaches the vehicle the distinction and is not complete without it. | Owner, 2026-09-27 | Invariants I1 and I2; roadmap item 17. |
| F3 | A light judged unreliable from the frames sends the vehicle home through item 19's substitution. | Owner, 2026-09-27 | Roadmap item 17 stage 5. |
| F7 | The carried light runs on a battery whose charge the vehicle knows. A mission ends at B with no return after it, so an ordinary flight's charge covers the way to B only. When the vehicle sees that the charge will not reach B, it gives B up for the start, and decides it while the charge still covers the way home. The light flickers whatever its charge. The return on the battery switches itself on with the carried light (camera profile) and off on the lidar profile. It replaces the time-bound return (K4). | Owner, 2026-09-27 | Roadmap item 17 stage 5. |
| F8 | The light's battery drains uniformly and linearly with time, independent of everything, the flicker included: neither a realistic lamp failure nor a realistic discharge is simulated. | Owner, 2026-09-27 | Simplicity of the simulation; roadmap item 17 stage 5. |
| F9 | A named scenario starts with a battery too low to reach B at all; the vehicle sees it will not reach B, gives B up while the charge still covers the way home, and returns. It is flown in item 17's acceptance: five flights return to the start in truth without a collision. | Owner, 2026-09-27 | Roadmap item 17 stage 5 and completion. |
| F10 | Roadmap item 22: flight in total darkness with no illumination of the vehicle's own (no lamp, no infrared or ultraviolet illuminator, no projector), on passive thermal cameras; the time-of-flight sensors are the one allowed emitter. | Owner, 2026-09-27 | A light of another colour is the same as the lamp. |
| F11 | Roadmap item 17 stage 3 compares five ways of lighting the scene (continuous flood, strobed near-infrared flood, active stereo, time-of-flight camera, laser line) on confident range, price and average electrical power; energy is a criterion beside price. | Owner, 2026-09-27 | A continuous flood costs tens of watts against about seven for a cheap 3D lidar; [`illumination_options.md`](illumination_options.md). |
| F12 | The camera vehicle carries a strobed near-infrared flood synchronized with the stereo pair's global shutter, over the pair's whole field; in the simulation it is the spot light of the vehicle model at intensity 2, cones 2.1 and 2.4 rad. | Agent, 2026-09-28 | Roadmap item 17 stage 3 under F11: the full 6.4 m range as flown in the dark location (r771, r781), about 1.5 W on average against 15 to 100 W for a continuous flood and 6.5 W for the lidar, some tens of dollars; [`illumination_options.md`](illumination_options.md). |
| F13 | A surface the stereo pair cannot match is not read as a prohibition: the braking contract's measured range (K9) falls with the share of the frame observed, and nothing else changes. No remedy of roadmap item 17 stage 2 (a wider window, another matcher, a projector) is built. | Agent, 2026-09-29 | Roadmap item 17 stage 2 measured that a texture-poor surface gives absent depth, not wrong depth (at least 0.73 of its matches within 0.25 m); a uniform panel across the route entered memory whole over the approach and was flown over (r783); [`camera_perception.md`](camera_perception.md). |
| F4 | Once roadmap item 18 lands, its smoke sensors are always on board; every location has smoky places that never block the way from A to B; smoke is constant in place and volume and changes only its shape; a separate scenario blocks the way with smoke and the vehicle flies home. | Owner, 2026-09-27 | Roadmap item 18. |

## Numbers The Stack Relies On

| ID | Value | Source, since | Justification, reference |
|---|---|---|---|
| K1 | An autopilot position reset up to 3.0 m is flown on (the execution revoked, the search restarted); a larger one closes the navigation. | Agent under the owner's delegation, 2026-09-27 | 0.4 to 4.1 m of drift over item 19's doubled path; r699 stood ten minutes after a 1.02 m reset. Commit d71a5c6f. |
| K11 | The return on the carried light's battery (F7): the goal is given up for the start when the charge left, in seconds of light, falls below 9.0 times the committed route's length left to B over the flight's mean speed (floored at 0.5 m/s), with a 20 s reserve; before a route reaches B, the straight line to it. The rule weighs the charge once the vehicle has flown 20 m, so that its mean speed is measured; at the start the floor alone asked 1172 s of light for a flight of about 300 s (r787). An ordinary flight launches with 3600 s of light. | Agent, 2026-09-29 | On five dark camera flights the way flown took 2.5 to 3.7 times that estimate at the median and at most 8.7 times (r773, r776, r781, r784, r785): the route through unknown space is the shortest the vehicle may find. At launch the estimate is about 1550 s for the 85 m route. |
| K12 | The carried light is judged unreliable (F3) once one outage, the braking contract's measured range below the range the sensor set guarantees, has lasted 5 s, or once outages have taken 60 % of the last 120 s. | Agent, 2026-09-29 | The moderate flicker dipped the range for at most 1.8 s and 1.5 % of the time (r785), no more than a flight without it (2.6 s, 3 %, r781); the outage is judged while the vehicle still holds on dead reckoning (K13), before its drift grows: 0.1 to 0.3 m through outages of up to 6 s, 1.5 to 2.1 m through 8 s ones (r795); at 10 s the severe failure of r789 outlived the hold and the autopilot landed blind on a structure. |
| K13 | The ladder's second rung (roadmap item 17 stage 5's open question, decided): past its 1.0 s unaided timeout the camera estimator keeps publishing a pose sound in every other respect for 10 s more, declared dead reckoning; through it the filter's height is the one the features last gave, moved by what the vehicle's barometer saw since (a pseudo-measurement of 0.3 m). After that it falls silent, and the autopilot lands. | Agent, 2026-09-29 | Replayed on the recorded dark flight r779 with its frames blanked, the IMU alone drifted 0.2 to 0.6 m horizontally in 5 to 10 s and 2 m in 20 s, and vertically 2 to 8 m in 5 to 10 s and 30 m in 20 s. Silence after 1 s left the autopilot 5 s before it landed blind (r789). A height left out (NaN) dropped the whole position from the autopilot's fusion, which takes it only when every axis is finite (r791); a height frozen in the message alone left the filter's own drifting, and it jumped by metres when the features returned (r792). With the hold the replayed height drift over 10 s dark fell from 7.8 to about 1.1 m; a height held still was not the vehicle's, which sank 1.4 m in 5 s, and the filter, told it stood still, bent its attitude and drifted 1.7 m sideways (r793): the barometer measures the height instead. |
| K5 | Item 19's goal outside the location: (200, 100, 10) m. | Agent, 2026-09-26 | Behind the outer walls of Urban Circuit Practice 01 and inside the memory's grid; the owner decided the injection is by the goal alone. |
| K7 | The guaranteed forward range of the stereo set is 6.4 m. | Inherited (item 14); re-measured by the agent 2026-09-28 | The confident depth measured on the location's surfaces. Re-measured by the same procedure in the dark world by the carried light, with the camera noise, the gain and the noise mask (roadmap item 17 stages 1 and 3, r782): disparity error p90 0.26 to 0.37 px from 2 to 8 m against 0.29 to 0.49 in item 14, 98 % of matched pixels within 0.25 m at 4 to 6 m and 90 % at 6 to 8 m; the procedure gives 7.1 m, and 6.4 m is kept. |
| K8 | The camera estimator is unhealthy, and falls silent, after an IMU hole longer than its 1 s unaided timeout (for the rest of the flight) and while its velocity is less certain than 1 m/s along any direction. | Agent, 2026-09-27 | r720 and r723 crashed on a diverging estimate published as healthy after holes of 1.9 and 3.6 s; the flying filter holds 0.08 to 0.18 m/s. Silence hands the vehicle to the autopilot's failsafe landing; an initialisation in flight is the register's L6. |
| K9 | The braking contract reads the forward sensor's latest frame (roadmap item 17 stage 0): its range scales from the configured one at 0.3 of the frame's beams observing anything down to the physical margin at 0.02, and the evidence age charged is the frame's measured age instead of the constant 600 ms. The stereo frame's whole is the 76 800 pixels it samples (1280 x 960 every fourth). | Agent, 2026-09-27 | A blind pair was flown at the speed 6.4 m admits. With matches within the image noise discarded, lit stereo frames observe 0.30 to 0.48 (r767), dark ones by a carried light that lit only the centre of the field 0.11 to 0.21 (r768); with the light over the whole field the dark flights measured the full range (r770 to r781), the lidar 0.59 to 0.98 (r737 to r741). |
| K10 | An occupied voxel of the obstacle memory returns to unknown once it has gone unconfirmed for 30 s per confirmation it collected (roadmap item 17 stage 8). | Agent, 2026-09-27 | On a static location decay only costs: at 2 s and 10 s per confirmation the camera flights r757, r758 lost route availability (96.2 % against 97.1 to 98.5) and held 3.6 to 3.7 % of the time, with 125 000 to 375 000 voxels decayed; its use is transient bodies (items 18, 21) and darkness. |

## Change Log

Every change of an entry, newest last: the entry, from, to, by whom, why, and
the commit. The first lines record the changes of the week this file was
opened, so that the numbers above carry their history from the start.

| Date | Entry | From | To | By | Justification | Commit |
|---|---|---|---|---|---|---|
| 2026-09-26 | K4 | margin 1.5 | 2.0 | Agent | Returns took up to 1.65 times the flight out; r687 ended 24 m short. | 62f4f511 |
| 2026-09-26 | A8 | 600 s | 900 s for injected flights, 1800 s for ordinary ones | Agent | The time-bound return fired inside ordinary flights; the doubled path needs 900 s. | journal |
| 2026-09-27 | K1 | 0.33 m | 3.0 m | Agent under the owner's delegation | r699: a 1.02 m reset held the vehicle ten minutes; drift over a doubled path 0.4 to 4.1 m. | d71a5c6f |
| 2026-09-27 | K3 | gain 0.5, steps 0.25 m and 0.5 degrees | gain 0.2, steps 0.1 m and 0.2 degrees | Agent | The target jittered by 0.2 to 0.35 m between registrations (r709). | 3f965181 |
| 2026-09-27 | A6 | — | new | Agent | Container-wide freezes under another task's disk writes crashed r720 and r723. | this file |
| 2026-09-27 | F1 to F4 | — | new | Owner | Decisions for items 17 and 18. | this file |
| 2026-09-27 | F2, F5, F6 | F2: the severe failure and the zone across B | F2: the severe failure; F5: a zone in every flight and a scenario across B; F6: darkness is not unknown | Owner | Addition to item 17. | this commit |
| 2026-09-27 | I7, I8, F1, F3, F7 | I7: nothing of the light reaches the vehicle; F3: the time-bound return reworked into the judgment | I7: its battery charge is known; I8: no time limit; F1: speeds under the flicker adopted; F3: the judgment alone; F7: the return on the light's battery, camera profile only | Owner | The owner's decisions on item 17. | this commit |
| 2026-09-27 | K2, K3, K6 | entries | removed | Owner | Small tuning parameters do not belong here; they stay in the code with their comments. | this commit |
| 2026-09-27 | the rules | Owner entries changed by agents only with a permission per decision | agents may change Owner entries, recorded and reported | Owner | The owner's permission. | this commit |
| 2026-09-27 | K8 | — | new | Agent | The first repair of the register's L6, asked for by the owner. | this commit |
| 2026-09-27 | F8, F9 | — | new | Owner | The battery's linear drain and the low-battery scenario. | this commit |
| 2026-09-27 | F7, F9 | "the charge covers only the way back"; "too low to reach B and return" | the charge is weighed against the way to B, since no return follows B; the decision is taken while the way home is still covered; "too low to reach B at all" | Owner | The owner's correction: a mission ends at B. | this commit |
| 2026-09-27 | F10 | — | new | Owner | Roadmap item 22. | this commit |
| 2026-09-27 | F11 | three options by price | five options by range, price and power | Owner | The discussion of low-power illumination. | this commit |
| 2026-09-27 | K9 | constant range 6.4 m / 14 m and evidence age 600 ms | measured from the frame | Agent | Roadmap item 17 stage 0. | this commit |
| 2026-09-27 | (none) | — | the shaft (53,-7) left as the price of I1, class (c) | Agent | Measured: a dead end for the 0.82 m envelope, occluded from outside; no fix within I1, I3 and the hover drift. The owner asked for a fix; reported. | journal |
| 2026-09-27 | K10 | occupancy kept for the flight | decay at 2 s per confirmation | Agent | Roadmap item 17 stage 8. | this commit |
| 2026-09-27 | K10 | 2 s per confirmation | 30 s | Agent | r757 (2 s) and r758 (10 s): availability 96.2 %, holds 3.6 to 3.7 %. | this commit |
| 2026-09-27 | K9 | thresholds 0.5 / 0.05 | 0.3 / 0.02 | Agent | The noise mask lowered what a frame observes: lit 0.30 to 0.48, dark 0.11 to 0.21; at 0.5 the dark world stood at the margin (r768). | this commit |
| 2026-09-28 | F12 | — | new | Agent | Stage 3's comparison under F11: the strobed near-infrared flood; the measurements and the energy estimate are in illumination_options.md. | this commit |
| 2026-09-28 | K7 | 6.4 m (item 14, lit, noiseless) | 6.4 m kept | Agent | Re-measured under item 17 stages 1 and 3 (dark, carried light, noise, gain, noise mask): the procedure gives 7.1 m (r782). | this commit |
| 2026-09-29 | F13 | — | new | Agent | Roadmap item 17 stage 2's measurement decided the owner's open question of 2026-09-25: a matched nothing is absent depth, not a prohibition. | this commit |
| 2026-09-29 | K4 | the time-bound return | removed | Agent under I8 and F7 | Roadmap item 17 stage 5 lands the battery and the light's judgment in its place; the monitor no longer reads the run's window. | this commit |
| 2026-09-29 | A8 | 900 s for injected flights, 1800 s for ordinary ones, while the time-bound return exists | the window is the harness's timeout only, 1800 s | Agent | Nothing of the vehicle reads the window. | this commit |
| 2026-09-29 | K11, K12 | — | new | Agent | F7 and F3 built; margins and thresholds measured on the flights named. | this commit |
| 2026-09-29 | K12, K13 | an outage of 10 s; silence after 1 s | an outage of 6 s; 10 s of declared dead reckoning without height | Agent | r789: the autopilot landed blind on a structure 5 s after the estimator fell silent, before the 10 s judgment. | this commit |
| 2026-09-29 | A9 | — | new | Agent | r790: a landing counted as a crash. | this commit |
| 2026-09-29 | K13 | the height left out | the last healthy height held | Agent | r791: the autopilot fuses an external position only when every axis is finite. | this commit |
| 2026-09-29 | K13 | the height frozen in the published message | the height held inside the filter by a 0.5 m pseudo-measurement | Agent | r792: the filter's drifted height jumped 5.8 m at the light's return, the autopilot reset by 5.2 m. | this commit |
| 2026-09-29 | K13 | the height held still | the height moved by the barometer's change | Agent | r793: the vehicle sank 1.4 m and drifted 1.7 m in a hold that assumed a constant height. | this commit |
| 2026-09-29 | K12 | an outage of 6 s | an outage of 5 s | Agent | The severe failure's plateau lowered to 6 s of dark (r795: 8 s outages drifted the hold 1.5 to 2.1 m). | this commit |
