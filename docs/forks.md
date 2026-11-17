# Forks And Mentions

What others do with this repository. The owner's rule of 2026-11-17: the forks
of `formiat/px4-ros2-drone-nav`, the copies that are not GitHub forks, and the
publications about the repository or its forks are looked at once a week;
what is useful is brought here, a commit of someone else's with its author
named (`git cherry-pick -x` for a commit taken as it is; `Based-on:` and
`Original-author:` lines for one rewritten here; a line in the changelog), and
only after it has been read, gated and flown as any change is. A report of the
code flown on a real vehicle is recorded here and may be cited.

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
- **PX4 SITL stopping in flight** read from the ulogs (every topic ends at one
  stamp while the truth goes on): `scripts/px4_liveness_runtime.sh` records the
  autopilot's exit; a flight so ended is the host's, not the stack's.
- A guard of PX4 SITL's MAVLink ports against foreign command frames, the
  process cleanup bounded to its own pid namespace, one flight per checkout.
- `tools/airframe_braking.py`: the airframe's braking from the true pose.
- The pull request #2: the log line of a refused stop carries the evidence it
  was derived from (70 lines, three files).

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
