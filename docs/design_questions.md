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
its rollouts in flight (25 to 200 times short, measured), another GPU
interface (Vulkan compute or OpenCL) could, and the Jetson itself will fly
fewer rollouts than the development host. The ways, their size and the
order are roadmap item 28; the boards that would fit without CUDA, and
whether they come cheaper (5 to 20 percent of a vehicle) or simpler (in
hardware on the VOXL 2 alone, in software nowhere), are in
[hardware_builds.md](hardware_builds.md).

## What Does The Camera Vehicle Do In A Large Dark Space?

Asked on 2026-10-13: a camera vehicle at night over open ground at a height,
or in a very large closed space, gets no light back; does it take its light
for failed and not fly? And should a "large-space mode" treat darkness as
the absence of obstacles?

Today, in empty darkness the frames see nothing within the light's reach:
after 3.5 s the light is judged unreliable (K12) and the vehicle turns home
along its trail. The larger trouble is the estimator: with nothing to track,
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
