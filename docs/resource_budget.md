# Resource Budget

What the processes of one flight consume, measured on the flight itself, and
what that does and does not say about the computer the stack would fly on.
Every headless flight records it (`resources.csv` and `resources_host.json`
in the run directory, `scripts/capture_process_resources.py`), and the
mission check reads the record (`scripts/resource_budget_evidence.py`); see
[testing.md](testing.md). The figures here are from r345, the first recorded
flight, on the urban point-to-point mission
(`make sim-urban-point-to-point-headless`): 392.8 m in 142 s at 2.763 m/s,
no crash, tick 23.4 ms at p50 and 36.9 ms at p95, planner search 150 ms at
p50 and 161 ms at p95, the same flight the rest of the documentation measures
([performance.md](performance.md)). One flight is one sample; the record
exists so that every flight adds one.

## Host

| | |
|---|---|
| CPU | AMD Ryzen 9 5900HX, 8 cores / 16 threads |
| Memory | 30 GiB |
| GPU | NVIDIA GeForce RTX 3060 Laptop, 6 GiB, driver 580.126.09, CUDA 12.6 |
| Kernel | Linux 6.17 |
| Container | `drone-gazebo-dev:latest`, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17 SITL |
| Simulation | real-time factor 1.00 at p50 and at least, so the timings are real time |

## Processes

CPU is in cores (1.00 is one core fully busy), sampled once a second over
the flight from mission readiness to the result; memory is resident set
size; growth is the last tenth of the flight against the first, at the
median of each.

Onboard: what flies on the aircraft. PX4 lives on the flight controller, so
the simulated PX4 is not in this set; the simulator and the harness do not
exist there.

| Process | Cores p50 | Cores p95 | Cores max | RSS p95 | Growth | Threads | GPU memory |
|---|---|---|---|---|---|---|---|
| `production_mppi_node` | 1.81 | 2.38 | 2.57 | 592 MiB | +128 MiB | 33 | 206 MiB |
| `obstacle_memory_3d_node` | 0.83 | 1.08 | 1.10 | 135 MiB | +53 MiB | 19 | |
| `mppi_offboard_node` | 0.04 | 0.05 | 0.05 | 44 MiB | 0 | 16 | |
| `MicroXRCEAgent` | 0.03 | 0.04 | 0.05 | 28 MiB | 0 | 53 | |
| **Onboard together** | **2.73** | **3.36** | | **798 MiB** | | | **206 MiB** |

The two growths are the map: the obstacle memory and the controller's copy
of the raw occupancy grow with the volume the flight observes. The mission
check bounds an onboard process at 256 MiB of growth per flight, twice the
controller's figure; a leak at the tick rate crosses that fast (5392 ticks
losing 50 KiB each are 263 MiB).

Simulator and harness, for the record of what the workstation carries
besides the stack:

| Process | Cores p50 | Cores p95 | RSS p95 |
|---|---|---|---|
| Gazebo server (`ruby`, GPU lidar 360 x 181 at 10 Hz) | 0.59 | 0.61 | 1640 MiB |
| `px4` (SITL) | 0.19 | 0.20 | 20 MiB |
| `parameter_bridge`, heading source, mission monitor, crash node, visualization | 0.16 | 0.20 | 229 MiB |
| captures (setpoints, true pose, raw snapshots, resources) | 0.14 | 1.12 | 222 MiB |
| **Container as a whole (cgroup)** | **3.97 mean** | | **3031 MiB** |

The captures' p95 is the raw-snapshot capture writing a snapshot; the
resource capture itself is 0.02 cores and 31 MiB.

GPU: 42 percent utilisation at p50 and 51 at p95 of the RTX 3060 Laptop,
466 MiB of memory in use, 206 MiB of it the controller's. The controller
simulates its 8192 rollouts in 6.4 ms at p50 and 8.1 at p95 of GPU time
inside a controller step of 11.2 ms at p50; the rest of the GPU's time is
the simulated lidar's rendering, which does not fly.

## Transport

Every consumer of a ROS transport hop measures the delivery of each message
it receives, the middleware's receive timestamp against the source timestamp
the publisher's middleware set (`transport_latency_ros.hpp`); the mission
check reports the four hops and bounds the memory hop at 2.5 ms at p95
([testing.md](testing.md)). Measured on r346 (2 Hz memory transport) and the
r352 to r356 series (10 Hz):

| Hop | Message | Rate | Delivery p50 | p95 | max |
|---|---|---|---|---|---|
| Gazebo bridge to obstacle memory | `PointCloud2`, 63 700 returns | 10 Hz | 0.74 to 0.81 ms | 1.0 to 1.2 | 2.8 |
| obstacle memory to controller | `RawObstacleSnapshot3D` / `Delta3D` | 2 then 10 Hz | 0.15 to 0.16 ms | 0.25 to 0.32 | 11.6 |
| obstacle memory to controller | `LatestLidarObstacleScan` | 10 Hz | 1.3 ms | 1.5 to 1.8 | 4.2 |
| controller to offboard | `MppiTrajectoryHorizon` | 50 Hz | 0.06 to 0.08 ms | 0.10 to 0.14 | 1.0 |

The transport is not where the observation age comes from. The age the
planning tick sees on its obstacle memory splits, at the median, into the
time the memory takes from a scan's stamp to the publication of the update
that carries it, the delivery, and how long the delivered update has waited
for the tick, which is half the publication period:

| Memory transport | Observation age p50 | p95 | Scan to publication | Delivery | Waiting for the tick | Controller cores p50 | Memory cores p50 | Tick p50 |
|---|---|---|---|---|---|---|---|---|
| 2 Hz (r346) | 404 ms | 636 | 132 ms | 0.15 ms | 248 ms | 1.68 | 0.83 | 23.1 ms |
| 5 Hz (r348, r349) | 252 to 256 | 359 to 362 | 136 to 140 | 0.15 | 92 to 98 | 1.95 to 1.99 | 0.81 to 0.84 | 23.9 to 24.1 |
| 10 Hz (r350 to r356) | 184 to 200 | 252 to 274 | 114 to 136 | 0.16 | 48 to 56 | 1.93 to 2.32 | 0.83 to 1.04 | 24.1 to 25.0 |

The braking contract charges 600 ms of evidence age; at 2 Hz the p95 sat
above it. The memory now transports at the scan rate
(`obstacle_memory_3d_transport_rate_hz: 10.0`): the controller pays about
0.3 cores to ingest five times the updates and nothing more for ten, the
memory node is unchanged, and the tick grows by about a millisecond at
p50. What remains of the age is the memory's own scan-to-publication time,
the alignment wait and the integration, which no transport rate touches.
r347 flew under a foreign compiler build on the host and is excluded: its
lidar evidence age reached 2240 ms and its deliveries doubled.

Onboard, over the r352 to r356 series at 10 Hz: 3.03 to 3.43 cores at p50
and 3.85 to 4.14 at p95, 744 to 830 MiB; the r345 figures above are the
2 Hz baseline.

## Onboard Computer Class

What the measurements do not exclude, and what they cannot say.

The CUDA controller ties the stack to NVIDIA. That makes the Jetson Orin
family (Orin Nano, Orin NX, AGX Orin) the class of drone computer the
measurements do not exclude; the ModalAI VOXL 2, the Qualcomm RB5 and x86
modules without an NVIDIA GPU cannot run the MPPI as written.

Within that class, measured: 2.73 cores at p50 and 3.36 at p95, 798 MiB of
resident memory and 206 MiB of GPU memory. Every Orin module has at least
six cores and 8 GiB of memory shared between CPU and GPU, so the sizes fit.
The rates are where the assumptions begin, and each is an assumption, not a
measurement:

- Assumption: an Orin CPU core (Cortex-A78AE, up to 2.2 GHz) does less per
  second than a Zen 3 core at up to 4.6 GHz. The 3.36 cores at p95 become
  more cores of an Orin, and the two heavy processes are each one main
  thread with workers, so the per-core rate matters, not only the count.
  The persistent planner already spends its whole 150 ms budget on this
  host; on a slower core it returns a worse route inside the same budget,
  not a later one, and the budget itself is the parameter to revisit.
- Assumption: the Orin GPUs are smaller than the RTX 3060 Laptop (1024 CUDA
  cores on Orin NX and the 8 GiB Orin Nano, 2048 on AGX Orin, against 3840,
  per NVIDIA's published module specifications), so the 6.4 ms of rollout
  time grows. The 20 ms tick deadline bounds it, and the lever is the
  rollout count (8192, `rollouts` in `config/urban_mvp.yaml`), which the
  controller reads at start; fewer rollouts is a worse local optimum, and
  how much worse is not measured.
- Assumption: the simulated lidar is a dense 360 x 181 sphere at 10 Hz,
  651 thousand returns a second. A real sensor's rate and pattern set the
  obstacle memory's 0.83 cores and the raw sweeps' cost, up or down, and
  the guaranteed detection range the braking contract rests on
  ([navigation_pipeline.md](navigation_pipeline.md)).

Not in these figures at all: the sensor driver, a lidar-inertial
localization stack (roadmap item 13; the flights use PX4's EKF with
simulated GNSS), and the flight controller link. Nothing here was run on an
Orin: there is no arm64 build, and the only confirmation is a recorded
flight on the device itself.
