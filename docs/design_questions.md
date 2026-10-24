# Design Questions

Questions the project owner asked about why the stack is built the way it
is, with the answers and the ideas they led to. These are answers and ideas,
not requirements: what is decided is in [specification.md](specification.md),
what is planned in [roadmap.md](roadmap.md), what is known to be wrong in
[technical_debt.md](technical_debt.md).

## Why Our Own Estimators And Memory, Not OpenVINS, VINS-Fusion, nvblox Or OctoMap?

Asked on 2026-10-13. In short: for two of the three parts the choice holds,
but not everything was done the best way.

**Where our own is justified.**

- **The obstacle memory.** The core of the project is the difference between
  unknown space (free, invariant I1) and space the sensor looked into and did
  not see (darkness or smoke as a measured prohibition with a guaranteed exit,
  specification F6 and K14), and a braking contract on the range the data
  actually confirm (K9). Neither OctoMap nor nvblox knows such a state;
  OctoMap does not build an ESDF either, and nvblox would need a layer of our
  own for it.
- **The visual-inertial estimator.** It is woven into the ladder a blinded
  vehicle descends: declared dead reckoning, the height from the barometer,
  silence after 10 s (K13), the restart on the autopilot's pose (K28), the
  judgment of the light (K12). Over a ready filter all of it would be a
  wrapper to build. On EuRoC MAV (Vicon room 1) our estimator holds an ATE of
  0.046 to 0.115 m (K33, [localization.md](localization.md)); OpenVINS and
  VINS-Fusion report from a few centimetres to about 0.1 m on the same
  recordings in their publications. The two were never run side by side.

**Where it could have been better.**

1. **A ready estimator as a reference from the start.** OpenVINS as a
   measuring tool (the register's reference visual-inertial system) was set
   aside because the image lacks Ceres and Boost.Filesystem, so the
   estimator is measured against the simulator's truth only, while much work
   went into its defects (fifteen in K28 alone, the IMU holes).
2. **What the ready ones have and ours lacks:** loop closure (VINS-Fusion),
   whence the drift of a first pass (the register's L1); the online
   calibration of the frames' time offset and the extrinsics (both). A real
   vehicle will need them.
3. **The lidar-inertial estimator.** For a real lidar FAST-LIO2 is likely
   better than ours: it compensates the motion within a sweep, ours does
   not, and for a Livox Mid-360 that would have to be written
   ([hardware_builds.md](hardware_builds.md)).
4. **The ESDF on the GPU.** nvblox computes the ESDF on a Jetson's GPU, our
   memory takes 0.6 to 0.8 of a CPU core. Measure on an Orin first: if the
   cores are short, this is the first candidate to move (roadmap item 26).

**Was it in vain, and should it be redone?** Not in vain, and not to be
redone; to be added to.

- The memory stays: nothing ready carries its semantics.
- The visual-inertial estimator stays: its accuracy on real recordings is of
  the ready ones' order, and the blindness ladder lives inside it; replacing
  it now would mean rebuilding the ladder around a foreign filter and flying
  every scenario again. What to add: a reference estimator as a measuring
  tool before the hardware (the cheapest step that says how good ours is;
  roadmap item 25, its next steps, and the register's reference
  visual-inertial system), loop closure (the register's unbounded drift,
  L1), the online time-offset and extrinsics calibration before real
  cameras (roadmap item 27 block 4).
- The lidar-inertial estimator serves the simulator (the lidar flies 2.2 to
  2.5 m/s, its registration 35 ms at p95). On a real lidar it lacks motion
  compensation: decide then between writing it and adopting FAST-LIO2, by
  replaying one public real recording (roadmap item 25: Hilti, NTU VIRAL)
  through both; not before.
- The ESDF on the GPU (nvblox): only if an Orin's measurement asks for it
  (roadmap item 26).

In hindsight our own estimator cost more time than integrating a ready one
would have, but that time bought the ladder the light scenarios rest on;
with a reference tool from the start some of its defects would have been
found sooner.

## Why CUDA?

Asked on 2026-10-13. The reason was not recorded, in the documents, the
commits or anywhere else. CUDA came on 2026-07-26 in three commits without a
rationale in their messages: an isolated, opt-in MPPI benchmark first
(3edaf360: "does not add CUDA to the normal ROS runtime targets; requires
the NVIDIA container runtime"), then a shadow planner (2ca511fa), then the
production stack (902164a8). It was measured on the development laptop
before it was adopted.

A reconstruction, not a record: MPPI is a GPU algorithm by nature,
thousands of independent rollouts, and its classic implementations run on
CUDA; every rollout step here sweeps the body against a 3D ESDF, hundreds of
lookups a step, which a CPU cannot carry at this count (roadmap item 28
measures it); the development laptop has an NVIDIA GPU; and above all the
same CUDA code runs on a Jetson, the usual companion computer of a drone with
heavy perception: one source from the laptop to the vehicle. The price is
the tie to NVIDIA, weighed in roadmap item 28, and with a product made in
numbers (specification R6) a real cost on every vehicle.

## Can The Stack Do Without CUDA?

Asked on 2026-10-13. CUDA is in the MPPI engine alone; a CPU cannot carry
its rollouts in flight (8 to 110 times short, measured), another GPU
interface (Vulkan compute or OpenCL) could, and the Jetson itself will fly
fewer rollouts than the development host. The ways, their size and the
order are roadmap item 28; the boards that would fit without CUDA, and
whether they come cheaper (5 to 20 percent of a vehicle) or simpler (in
hardware on the VOXL 2 alone, in software nowhere), are in
[hardware_builds.md](hardware_builds.md).

## Do We Need An arm64 Build?

Asked on 2026-10-13, for the day the stack goes onto a real vehicle. Yes:
every flight computer the hardware registry finds fitting is arm64 (the
Jetson Orin, the VOXL 2, the RK3588); only an x86 board with an integrated
GPU spares the port, and it is the heaviest choice, which a product made in
numbers weighs against (specification R6). The port is small in code and
real in the build: the sources carry no x86 instructions, the base image and
ROS 2 Jazzy exist for arm64, and the Orin runs Ubuntu 24.04 since JetPack 7.2;
what is missing is an onboard build without Gazebo, the Orin's CUDA
architecture (87) and CUDA 13, an arm64 image, and the stack measured on an
arm64 board. The steps are roadmap item 27, "The arm64 Build"; the first
three need no vehicle.

## Is The x500 The Right Vehicle?

Asked on 2026-10-13. The simulated vehicle is PX4's `x500`, a copy of
Holybro's X500 V2, with the project's sensors on it (`x500_lidar_3d`).

| Trait | The x500 | Read as |
|---|---|---|
| Size | 500 mm between motors, 11 inch rotors in the model: the motors 0.25 m from the centre, the rotor tips 0.39 m, about 0.77 m tip to tip. The stack models the body as a cylinder of 0.55 m radius and keeps an envelope of 0.82 m | Mid-to-large; large indoors and underground: a standard door is 0.8 to 0.9 m wide, and the stack wants 1.1 m for the body alone |
| Mass | 2.0 kg in the model, whose sensors weigh nothing (0.000001 kg each); the real frame alone 610 g, a payload of up to 1.5 kg without the battery | Heavy: a real camera build weighs some 2 to 2.5 kg |
| Flight time | about 18 min hovering without payload on 4S 5000 mAh, less with it | Short: most of the energy lifts the frame and the battery |
| Price | the frame kit without an autopilot about €380, the PX4 kit $508 to $1 150 | Cheap as a development platform |
| How common | PX4's default airframe in SITL and Holybro's reference kit, a large community | Common as a research platform, not as a product: an open frame, no enclosure, no weather sealing, no certification |
| Necessary or excessive | a payload of 1.5 kg against some 0.3 to 0.5 kg the camera build carries (computer and sensors), 0.5 to 0.7 kg the lidar build | Excessive for a product, convenient for development: carries any computer and sensor on trial, robust, easy to repair, the PX4 model fitted |

The class products go to, for comparison: ModalAI's Starling 2, 220 mm
across, 280 g at take-off, over 40 min of flight, a VOXL 2 on board, $2 949.99
as a development vehicle; the Starling 2 Max, 322 mm, 500 g, over 55 min, 500 g
of payload besides, $2 999.99 ([hardware_builds.md](hardware_builds.md)).

**What the routes ask of the size** (measured on 2026-10-13 on the final
code's series, `tools/airframe_size.py`: every 0.5 s of the
mission, the horizontal distance from the body's axis to the nearest truly
occupied voxel at the body's height, the 0.25 m truth grid):

| Series | Nearest wall: least | p5 | p50 | Within 1.0 m | Within 1.5 m | Within 2.0 m |
|---|---|---|---|---|---|---|
| Cameras, r1430 to r1434 | 0.57 to 0.88 m | 0.99 to 1.22 | 2.01 to 2.13 | 1 to 6 % | 15 to 22 % | 40 to 49 % |
| Lidar, r1425 to r1429 | 0.47 to 0.77 m | 0.96 to 1.13 | 1.92 to 2.04 | 2 to 6 % | 18 to 25 % | 48 to 56 % |

Half of every flight runs within 2 m of a wall, a fifth within 1.5 m. At the
narrowest points flown the wall stands 0.47 to 0.88 m from the axis: the
x500's real rotor tips (0.39 m) pass with 0.08 to 0.5 m, while the stack's
body (0.55 m) and envelope (0.82 m) are already in evidence there, and the
clearance laws hold the vehicle slow. A vehicle whose rotor tips stand
within 0.2 m of its axis (the 220 to 320 mm class) keeps the envelope's
0.27 m beyond them at all but the narrowest of these points.

**What a smaller body would give** (the same tool, a what-if on the same
flights: the clearance laws admit clearance over 0.075 s, the tube law, so a
body smaller by D gives every lateral sample D more clearance; each planned
tick's reference is recomputed against its other limits and the cruise,
and the path re-timed):

| Series | Today, over the planned ticks | Body 0.10 m smaller | 0.20 m | 0.30 m |
|---|---|---|---|---|
| Cameras, r1430 to r1434 | 1.55 to 1.66 m/s | 1.90 to 1.95 (+16 to 23 %) | 1.93 to 1.97 | 1.93 to 1.97 (+17 to 25 %) |
| Lidar, r1425 to r1429 | 1.98 to 2.48 m/s | 2.76 to 3.26 (+31 to 43 %) | 3.01 to 3.50 | 3.09 to 3.56 (+44 to 63 %) |

An optimistic bound: no dynamics, the same routes (a smaller body would also
open narrower and shorter ones, which counts the other way), and the lidar's
vertical constraints (ceilings, K27) counted as if they too gained, which
they do only if the body is also shorter. The camera saturates at a tenth
of a metre: beyond it the braking contract's 6.4 m of range and the
curvature law hold the speed. The lidar keeps gaining.

**A finding on the way: the stack's body is larger than the x500's.** The
body radius of 0.55 m was set as the x500's with its rotor tips (20f7a915);
in the model the rotor tips stand at 0.39 m. The 0.16 m between them is the
first tenth of a metre of the table above, with no new airframe, but it is
also today's only horizontal margin against the estimate's drift (K27
allows for the vertical error alone): taken away, the body needs a
horizontal allowance of its own first. The register holds it
([technical_debt.md](technical_debt.md)).

**What a smaller airframe takes** (roadmap item 30): a Gazebo model with its
real masses (the sensors' included), the PX4 airframe (geometry, gains,
hover thrust); every constant fitted to the x500 fitted again (the
accelerations of the speed profile, the lags, the rotor drag 0.106, the
yaw-rate lead, the tube's 0.075 s, the rest clearance, the lean law, the
time-of-flight reflex's distances, the body and the envelope, the landing
speeds); the sensors' extrinsics, which live in SDF, C++, YAML and Python at
once; the model's name in the scripts and the crash judge; then every
acceptance series again. L to XL. The airframe is open (specification V7)
and the product is made in numbers (R6), which a smaller, lighter vehicle
serves: shorter routes, more speed in tight places, longer flight, an easier
regulatory class (in the EU, as understood here, under 250 g C0, under 900 g
C1, under 4 kg C2).

**The verdict.** The x500 was the right start: cheap, known to everyone,
with a margin for any experiment. As a product it is too large, too heavy
and too hungry, and in tight places slower than it needs to be.

## What Does The Camera Vehicle Do In A Large Dark Space?

Asked on 2026-10-13: a camera vehicle at night over open ground at a height,
or in a very large closed space, gets no light back; does it take its light
for failed and not fly? And should a "large-space mode" treat darkness as
the absence of obstacles?

Until roadmap item 29 (2026-10-14), in empty darkness the frames saw nothing
within the light's reach: after 3.5 s the light was judged unreliable (K12)
and the vehicle turned home along its trail (r1471 and r1475 in the dark
hall, 0.4 m and 12 m from the start). Since then the light is judged by its
driver's reading and the time-of-flight ring, and a dark place under a sound
light is crossed at the speed the memory's observed range admits (r1488 to
r1490, about 0.5 m/s across 64 m). The larger trouble is the estimator: with nothing to track,
it dead reckons after 1 s (K13) and the vehicle descends 3 s later (K19),
whatever the light. A mode that reads darkness as free is not built:

1. It does not keep the position: the estimator still has nothing to track.
2. A lit surface without texture gives the stereo pair no depth: read as
   free, it would be flown into; read as blind, as now, it is safe.
3. A light that really fails in that mode flies the vehicle blind at speed.

The idea taken instead, roadmap item 29 (the owner, 2026-10-13,
specification F18): the light judged by its own reading (the driver's
current or a photodiode) and by the time-of-flight ring, whose units carry
their own emitter (a unit sees a surface where the frame is dark: the light
has failed); darkness with a sound light stays unobservable to the braking
contract, so no speed into it, but no false "unreliable light, home". For
truly large spaces the answer is another sensor: the lidar build (40 m and
its own emitter) or, in the open at a height, GNSS; a camera under a
carried light sees about 8 m at night.

## Why Does Item 19's Proof Flood Through The Body's Radius, And Why Does A Grazing Ray Carve Nothing Of A Surface?

The proof floods the memory from the vehicle through every voxel that is
not occupied, unknown included, and calls the goal unreachable when the
flood closes without it. A flood a voxel wide passes what the body does not:
evidence laid a ray apart (a plume's face marked from 20 m is a beam apart,
0.35 m against the 0.25 m voxel) and the holes a few voxels wide that the
sensor leaves in a floor. In the blocked flights of 2026-10-15 the plain
flood ran through such sieves to its budget every ten seconds (r1549), and
with the 26 neighbours it leaked under the floor (r1553 replayed offline).
A flood through the voxels no occupied voxel lies within the body's radius
of (0.55 m, specification K16) closes on both, and it closes only where the
body could not pass either: a gap of two voxels is under the body's width.

The holes themselves were the memory's. From a hover the lidar's rays graze
the floors and the corridor walls at a few degrees, hit them in rings a row
apart and, between the rings, pass through the voxels that hold the surface
without ending in them: eight such misses a scan against a score of 12
carved a floor voxel free in seconds, and the start's side of a corridor
closed by smoke opened again within a proof sample (r1564, r1566). A ray
within 15 degrees of a plane of the grid cannot tell whether a surface
parallel to that plane lies under or beside its path inside a voxel, so it
carves no occupied voxel that lies in such a surface, read from the voxel's
neighbours (specification K42); a lone voxel, a speckle or a dark mark, is
carved as before. The rule that marked both voxels of a face a hit lay
within the range noise of (K41) was tried the same day and withdrawn: it
thickened a quarter of the surfaces toward the air and cost the lidar's
ordinary flights 15 percent of their speed.

## Why Is There No Frontier Planner, And Is Exploration Forbidden?

Asked in the discussion of 2026-10-19, recorded on 2026-10-24. In short:
the frontier planner was removed as a competing producer of routes with a
blacklist of frontiers tried, two things the invariants forbid; a survey
mission that hands the single planner its next goal was never forbidden,
and since 2026-10-24 it is allowed in so many words (specification F20,
roadmap item 34).

The chronology, from the history:

- 2026-07-30 to 08-03: frontiers in the code; `0ead8354` "Disable frontier
  blacklist by default" (the blacklist: 15 s and a 6 m radius around a
  frontier already tried).
- 2026-08-17 to 08-22: item 12 as "Incremental Topological Exploration":
  the route to the goal first; the topology and the frontiers as the
  fallback when the route is blocked, for the choice of a branch, and for
  an exploration asked for explicitly.
- 2026-08-20 to 08-26: the frontiers over the topology; on 08-26 five fixes
  of the frontiers in a row and `da961b9b` "make observation frontier
  stops opt-in".
- 2026-08-27, `30438eaf`: item 12 rewritten as "Persistent Full-3D
  Strategic Navigation", one D* Lite; the observation-frontier stop
  policy, the information-gain transit preference and the frontier rank
  forbidden; the topology never a competing route pipeline and unknown or
  unvisited space never rewarded in transit.
- 2026-08-28: the removals, `6a8f483b` (the online topology), `2bcec916`
  (the route arbitration), `8b53e983` (the retired route producers,
  `observation_frontier.cpp` among them); the line of overview.md's
  non-capabilities came with the last.

The reasons, as the texts give them: a stop at the boundary of the
observed space prohibited unknown space and a preference for the unseen
paid for it, against I1 (unknown is free at no cost); several producers of
routes with an arbitration changed the route at every update, against one
owner of the route; and for a flight from A to B frontiers are not needed,
the planner flies through unknown as through free and repairs the route
when the walls appear. The blacklist was the owner's decision and is now
I3 (no prohibition by history); I1 is the owner's; the removal of the
frontier planner whole was the agent's wording when item 12 was rewritten,
before the specification marked its entries as the owner's or the agent's
(2026-09-27).

What was inconsistent until 2026-10-24: item 11 still named "item 12's
incremental exploration backend" as its acquisition path, a mechanism gone
since 2026-08-28. It names item 34's survey now.

The rule now (F20): the survey chooses goals for the one planner; in
transit nothing prefers unknown or a frontier; a frontier disappears by
measurement alone (seen, or outside the reachable component by K16's
flood), never by a list of attempts; the volume a survey covers is the
mission's order, as a goal point is, and its wording is the owner's when
item 34 starts.
