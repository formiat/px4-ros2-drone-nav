# Forks And Mentions

What others do with this repository. The owner's rule of 2026-11-17: the forks
of `formiat/px4-ros2-drone-nav`, the copies that are not GitHub forks, and the
publications about the repository or its forks are looked at once a week;
what is useful is brought here, a commit of someone else's with its author
named (`git cherry-pick -x` for a commit taken as it is; `Based-on:` and
`Original-author:` lines for one rewritten here; a line in the changelog), and
only after it has been read, gated and flown as any change is. A report of the
code flown on a real vehicle is recorded here and may be cited.

## 2026-11-22 (whose vehicle could fly it)

A flight of this code on someone else's vehicle was weighed in the owner's
discussion of that day; no fork shows a vehicle that could make one.

- **naipi11:** none in sight. Their own words (`d59ba2ac`, 2026-11-14): "a
  target vehicle's braking distance and its camera's confident range stay
  unestablished until one is flown". Between our `4fc128d9` and their head
  `9bde562b` the copy gains no recording from sensors of its own, no
  calibration and no parameters of an airframe (read here by the names of
  the changed files). Whether a vehicle exists is a question to ask, and
  asking it is the owner's.
- **Lotus56:** a printed airframe (print files and photos); its README names
  no electronics and says "our plan, not yet working code"; unchanged since
  2026-10-26. A weak candidate, and by its subject not one to join: the
  interception of aircraft, which this project removed (the Completed items
  1 to 5).

## 2026-11-20 (the second review; 33 forks listed)

The window 2026-11-12 to 2026-11-20. `naipi11/PX4-ROS2-NAV` has no commit
after 2026-11-16 (its head `9bde562b`); every other fork holds no commit of
its own that is not here; one new fork, `kaveh-hooshmandi/px4-ros2-drone-nav`
(2026-11-14), a copy; `Lotus56/the_interceptor_drone` is no longer listed as
a fork and is unchanged since 2026-10-26. No sign of a real vehicle. What of
naipi11's work, read for the first review, is not here and applies (the
agent's reading, taken by the owner's word of 2026-11-21; their code read by
its diffs, not built here; roadmap item 43 points 5 to 7):

- **The cleanup tells checkouts apart by the image or the mount** (theirs:
  `20121042`, by the mount alone). Here
  `scripts/cleanup_sim_processes.sh`, `container_belongs_to_repository`,
  still takes a container for this repository's when it runs the dev image
  *or* mounts the repository: a second checkout on the host runs the same
  image, and one checkout's `stop_sim.sh` between its flights stops the
  other's flight, which then reads as the host's or the autopilot's failure.
  A base was flown here in a copy beside the working tree on 2026-11-19 and
  20 (`072941ab`), one flight at a time; nothing was stopped, by the order
  alone. To take: the mount alone.
  **Taken on 2026-11-22** (144622d8, their name in the commit and the
  changelog), with a half this review had not read: the process half of
  the cleanup bounded to its own pid namespace (theirs: `bb3e6d20`; on the
  host `ps` lists every container's processes, and the cleanup selected
  them by their command lines). Both rewritten here: the root's mount
  alone, where theirs takes any path under it; the filter where the live
  process table is read, where theirs passes a test's argument through the
  selection.
- **One flight a checkout** (theirs: `54635cd6`, `cc30b16a`: a lock
  directory with its owner's pid in `run_drone_nav_sim.sh`; a second flight
  refused with the owner named, a dead owner's lock taken over). Here there
  is no lock: a series started twice overwrites a run's record under one
  number, and two flights of one checkout share the models' directory, the
  world and PX4. To take.
  **Taken on 2026-11-22** (480c9b57, d7c7a30d; their name in the commit and
  the changelog): the rule, not the code. A pid written by one flight means
  nothing to the next here (a container and a pid namespace a flight), and
  the first flight is stopped by the second launcher's cleanup on the host,
  before the flight script; the lock is `flock` on a file the host and the
  containers share, taken by the launcher before its cleanup
  (`scripts/flight_lock_runtime.sh`). Their move of the lock to a file of
  its own (`cc30b16a`) was not needed.
- **The clause of a refused latest-evidence revalidation** (theirs:
  `b7b2a6d2`, with the path's points built once in `d7ed1713` and the unused
  boolean removed in `9bde562b`). Here `revalidateFiniteExecution`
  (`execution_supervisor_3d_horizon.cpp`) returns a boolean: inside
  `evidence_not_current` an obstacle that stands on the path is not told
  from a revalidation that could not be made (a view incomplete, no raw
  world). The other half of what was taken on 2026-11-18 for the control's
  evidence. To take, with the points built once from the start.
- **The guard of PX4 SITL's MAVLink ports: not taken.** A foreign pymavlink
  client took their port 14550 and spoiled four flights (their host carries
  another simulator). This host flies one simulator and no other MAVLink
  client; the foreign load that voided flights here (an Android emulator)
  was processor time, not datagrams. To be weighed again when a second
  MAVLink client or simulator stands on the host; the lock above answers
  the case of two flights of one checkout.

## 2026-11-17 (the first review; 34 forks, 174 stars)

| Fork or copy | Its own work | What is in it |
|---|---|---|
| `naipi11/PX4-ROS2-NAV` (a copy, not a fork; MIT; from our `4fc128d9`) | 34 commits, 2026-11-12 to 2026-11-16 | see below |
| `naipi11/px4-ros2-drone-nav` | the branch `archive/incoming-fixes-20261108`, 33 commits over v0.5.0; the pull request #2 (closed by its author on 2026-11-12: "moving this work to our own repository") | see below |
| `hundong2/px4-ros2-drone-nav` | 1 commit (2026-09-20) | a Korean README, a course of six chapters, an HTML map of the architecture with links to the code; the state of 2026-09-17 |
| `Lotus56/the_interceptor_drone` | 5 commits (2026-10) | an interceptor drone's plan and a printed frame; this project kept whole under `legacy/`; no code of its own |
| the other forks | none | copies |

### naipi11

An engineer working with an agent by this repository's own rules (a changelog,
a register of debt, run numbers as evidence). What the copy holds since
2026-11-12, by its commit messages (not yet verified here):

- **A stop refused by its own commit evidence** (their run
  `20261113T091953Z-1573`): the vehicle flew its last 2.8 s at 1.8 m/s into a
  wall after seven stop attempts were refused at publication and twenty-four
  horizon commits were refused (`evidence_not_current`,
  `control_evidence_not_current`) with the feedback 20 ms old. Their work in
  `execution_supervisor_3d_horizon.cpp` and the publication of stops, with
  tests, and the tools `crash_window.py`, `stop_supersession.py`,
  `stop_without_successor.py`. Their measurement over 88 flights: 1681 of 1691
  stops replaced within a second, 113 ms at the median. **To verify here first**
  (roadmap item 43).
  Read again on 2026-11-17, their history whole: the crash's verdict was
  withdrawn by its authors the same day (`16b16932`: the autopilot had exited
  4 s before the contact, the status read was a stopped stream's last value;
  the first review here missed that commit). What they kept is the measure and
  a question: the refusal that precedes a stop named by its clause
  (`b7b2a6d2`: in one flight all 16 were occupied evidence, the world's
  answer), a stop costing about 0.1 m of a 0.4 m braking distance when a
  reroute follows and carrying the vehicle when none does (`9fbd7ee7`,
  `stop_without_successor.py`), and whether a stop should latch until the
  vehicle is at rest. Here the same lines read over the 1099 flights kept on
  2026-11-17 (`tools/flight_sweep.py`): 14 024 stops published, 7850 refused
  lines (one a second at most, 3024 of them refused at the commit), 221
  flights with five or more in a row and 45 in the longest (r3400, a flight
  voided for the host's load), 8133 revocations of a moving vehicle, the
  fastest at 15.1 m/s on the open sky: a refused stop here hands a moving
  vehicle to the offboard's blind brake.
  Done here on 2026-11-18 and 2026-11-19 (roadmap item 43 point 1,
  specification K86). Taken from them, with their name in the commit
  (`d1ad24fe`: `Based-on: naipi11/PX4-ROS2-NAV c2fed51a`, `Original-author:
  naipi11`) and in the CHANGELOG: the idea of naming the clause that leaves a
  commit's control evidence not current, and the clause names; their code was
  read (`b8592414`, `c2fed51a`, `b7b2a6d2`) and none of it copied. Not taken:
  their tools (ours, `tools/refused_stops.py`, reads what followed each
  refusal by the truth) and a stop latched until rest (a prohibition by
  history, I3). Measured here over the 1099 flights (`tools/refused_stops.py`):
  7599 revocations of a moving vehicle in the counted flights, 2673 of them
  with a certified stop refused only at its commit, 132 of those within 0.5 m
  of a surface against 5.7 to 16 percent of an Urban flight's airborne time
  spent that near anyway; their reading "a stop is the world's answer" holds
  for the stops refused in preparation and not for those 2673. The rule: such
  a stop is prepared once more from the newest input and committed through
  the same checks.
- **PX4 SITL stopping in flight** read from the ulogs (every topic ends at one
  stamp while the truth goes on): `scripts/px4_liveness_runtime.sh` records the
  autopilot's exit; a flight so ended is the host's, not the stack's.
- A guard of PX4 SITL's MAVLink ports against foreign command frames, the
  process cleanup bounded to its own pid namespace, one flight per checkout.
- `tools/airframe_braking.py`: the airframe's braking from the true pose.
- The pull request #2: the log line of a refused stop carries the evidence it
  was derived from (70 lines, three files).
- Taken on 2026-11-18: the clauses of the previous control's evidence
  (`c2fed51a`), rewritten on this tree and named in the log of a stop refused
  at its commit; their wider log line of `b8592414` covers the stop's
  preparation, not its commit, and was not taken.

From the earlier branch over v0.5.0 (ideas and tests; the code no longer
applies): the lattice edge the probe budget did not reach is cached as blocked
until an obstacle is removed (still so here:
`persistent_dstar_lite_planner_3d_lattice_edges.cpp`) and their resumable
refinement of such edges; the lidar's self filter of 0.9 m drops true returns
of walls between 0.4 and 0.9 m while the propellers end at 0.386 m (still 0.9
here); stereo matching on Middlebury and RGB-NIR pictures (roadmap items 43
and 51).

### hundong2

The idea taken: a short guide in English and a map of the architecture with
links to the code (roadmap item 49). His text describes the state of
2026-09-17 and is not copied.

### Lotus56

A sign of demand for target following (roadmap item 47) and a possible first
airframe. Interception of aircraft is a defence subject.
