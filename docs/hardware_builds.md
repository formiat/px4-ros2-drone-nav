# Hardware Builds

A registry of the hardware a real vehicle for this stack could be built from:
the onboard computer, the perception sensors, the autopilot and the airframe,
each with whether it fits the stack and why, and what it costs. It is a sizing
exercise before roadmap item 27's later blocks, not a bill of materials:
nothing here has been bought or flown, and nothing of the stack has run on any
of it. What the stack needs is taken from its measurements in the simulator
([resource_budget.md](resource_budget.md), [camera_perception.md](camera_perception.md),
[localization.md](localization.md)); where a component's fit rests on an
assumption, the entry says so.

Prices are the ones found on 2026-10-13, in US dollars unless stated, with
their source; they move (NVIDIA raised every Jetson price by up to 101 percent
in July 2026) and are an order of magnitude, not a quote.

## What The Stack Asks Of The Hardware

| Need | Measured or modelled in the simulator | What it rules in or out |
|---|---|---|
| CUDA | The MPPI controller simulates 8192 rollouts on the GPU, 6.4 ms at p50 on an RTX 3060 Laptop (3840 CUDA cores) inside a 20 ms tick | An NVIDIA GPU onboard; anything without one cannot run the controller as written |
| CPU | The lidar profile's onboard processes 3.5 to 3.8 cores at p50, 4.4 to 4.7 at p95 of a Zen 3 core at up to 4.6 GHz (r405 to r434); the camera profile 4.9 at p50 and 5.7 at p95, 1.4 to 2.3 of them the stereo depth node's matching at 7.5 Hz (r576) | At least eight cores of the Jetson class, or the stereo matching moved off the CPU (to the camera, or to the GPU: roadmap item 26) |
| Memory | 0.8 to 0.95 GiB resident and 0.2 GiB of GPU memory | Any Jetson Orin module; 16 GiB leaves room for logs and a bag |
| Stereo pair | 1280 x 960, a 120 degree lens, a 0.20 m baseline, 7.5 Hz, global shutter; confident depth to 8.6 m, the braking contract takes 6.4 m (specification K9, K33) | The baseline and the lens set the top speed (the table in [camera_perception.md](camera_perception.md)): a shorter baseline sees less far and the vehicle flies slower |
| Time-of-flight ring | Six units (front, back, left, right, up, down), 8 x 8 zones over 45 x 45 degrees, up to 4 m (2.8 m used), 15 Hz | ST's VL53L5CX class; the blind descent's reflex (K19) and the landing on the floor below (K30) read it |
| 3D lidar (the lidar profile) | A full sphere, 360 x 181 at 10 Hz, 651 thousand points a second, no motion during a sweep; detection guaranteed to 14 m | Real lidars cover less of the sphere and sweep over time: the obstacle memory gets fewer points and the estimator needs motion compensation |
| IMU | The autopilot's, over uXRCE-DDS, about 110 Hz in SITL; the frames' 4 ms offset fitted in the simulator | Any PX4 autopilot; the time synchronisation is roadmap item 27 block 4 |
| Light | A wide spot light carried by the vehicle; its charge reported as a battery (K11); judged by what the frames see (K12) | A flood LED with a driver the computer can switch and read |
| Thermal camera (roadmap item 18 stage 3) | In the simulator the thermal pair of the exploration, Boson+ 640 class (640 x 512, 95 degrees, 0.20 m baseline), on the smoke rig; the wall's temperature is the only thermal texture the world carries; the stereo set's one core publishes the share of pixels warmer than the frame, which no decision reads yet (F4, 2026-10-17) | The accepted camera build mounts one core as a detector, the lidar build none: FLIR Lepton 3.5 (160 x 120, 57 degrees, 9 Hz, 164 USD, a gram, 150 mW, a VSYNC line) or a Chinese 256 x 192 module (about 300 USD, 25 Hz, no sync). Thermal stereo in a build someone would assemble is a pair of Chinese 640 x 512 OEM cores (1 000 to 1 600 USD, a sync line on a quote); Boson 320 (about 3 100 the pair) and Boson+ 640 (7 000 to 8 000) are the simulator's reference, not a build. Every core's flat-field shutter must be schedulable from the computer. |
| Autopilot and airframe | PX4 with uXRCE-DDS, offboard mode under the pilot's authority (K29); the vehicle model is the x500 (accelerations, lags, rotor drag fitted to it) | A Pixhawk-class autopilot; the X500 V2 airframe keeps the fitted model closest |

## Onboard Computers

| Computer | CPU, GPU, memory | Price | Fits | Why |
|---|---|---|---|---|
| NVIDIA Jetson Orin NX 16GB | 8 Arm Cortex-A78AE at 2.0 GHz, 1024 CUDA cores, 16 GiB | $999 the module (1 000 units; $599 before July 2026) [1] | **Yes, the baseline** | The CUDA controller runs; eight cores against 3.4 to 5.7 Zen 3 cores at p95 is the assumption to measure first: an A78AE core does less per second, and the persistent planner already spends its whole 150 ms budget on the development host, so on the Orin it returns a worse route in the same time. The GPU has a quarter of the RTX 3060's cores: the rollouts may have to be cut (`rollouts`), not measured |
| NVIDIA Jetson Orin NX 8GB | 6 cores at 2.0 GHz, 1024 CUDA cores, 8 GiB | $649 the module ($399 before) [1] | Marginal | Two cores fewer than the 16GB; the camera profile with the CPU matcher does not fit; the lidar profile perhaps, measured |
| NVIDIA Jetson Orin Nano 8GB | 6 cores at up to 1.7 GHz, 1024 CUDA cores, 8 GiB | $399 the module ($299 before) [1]; the Super developer kit $399 ($249 before) [2] | Bench only | Six slower cores against the stack's 3.4 to 5.7 Zen 3 cores at p95; the cheapest way to build for arm64 and run the stack on recorded data and on a bench (roadmap item 27's first steps) |
| NVIDIA Jetson AGX Orin 32GB / 64GB | 8 / 12 cores at 2.2 GHz, 1792 / 2048 CUDA cores, 32 / 64 GiB | $1 799 the 32GB module ($899 before) [2] | Compute yes, airframe no | The margin the NX lacks; 15 to 60 W and a larger heatsink and carrier push a 500 mm quadcopter to its payload; a larger airframe |
| NVIDIA Jetson AGX Thor | Blackwell GPU | $5 499 the developer kit [2] | No | Out of proportion to the stack's measured need and to the airframe |
| ModalAI VOXL 2 | Qualcomm QRB5165, 8 cores up to 3.09 GHz, 8 GiB, PX4 on board, 16 g | $1 249.99 [3] | No | No CUDA: the MPPI cannot run as written ([resource_budget.md](resource_budget.md)); otherwise the most integrated option (autopilot and computer in one, very light). A backend without CUDA would open it (below and roadmap item 28) |
| Raspberry Pi 5 | 4 Cortex-A76, no CUDA | not priced | No | No CUDA, four cores |
| An x86 board with a discrete NVIDIA GPU | the development host's class | not priced | No | Fits the compute, not a 500 mm quadcopter's mass and power |

An Orin NX module needs a carrier board (interfaces, the camera connectors,
the cooling): Seeed's A603 is listed at €165 to $309 [19]; a whole box with
the module, the carrier and a case (Seeed reComputer J4012) at $899, a price
likely from before July 2026 [20]. The arm64 build does not exist yet
(roadmap item 27 block 2).

## Stereo Cameras

The simulated pair is the yardstick: 0.20 m of baseline behind a 120 degree
lens at 1280 px gives 8.6 m of confident depth and 3.5 to 4.1 m/s of forward
speed by the braking contract; a confident range scales with the square root
of baseline times focal length ([camera_perception.md](camera_perception.md)).

| Camera | Baseline, field of view, sensor | Price | Fits | Why |
|---|---|---|---|---|
| Two global-shutter 1 MP cameras on a 0.20 m bar (Arducam's OV9281 stereo kit) | the baseline chosen; 1280 x 800; lenses chosen (120 degrees wanted) | $173.62 the kit [4] | **Yes, the simulated geometry** | The only option with the simulated baseline; the stack configures a pair by its calibration and rectifies it (K33), so a custom pair needs a calibration, not code. Open: the kit's hardware synchronisation (not confirmed by the listing), 120 degree lenses, and the matcher's 1.4 to 2.3 cores (on the CPU, or the GPU with roadmap item 26) |
| Luxonis OAK-D Pro W | 75 mm, 127 degrees horizontal (OV9282, 1280 x 800, global shutter), IR flood LED and dot projector, IMU, depth computed on the camera | $449 [5] | Yes, slower | The baseline is about 0.4 of the simulated one: about 0.6 of its confident range at a similar focal length, so a lower top speed (to measure). Its depth on the camera frees the onboard CPU, and its IR flood is light of its own in the dark. Needs a small node to hand its depth to the obstacle memory, which takes the stereo node's returns today |
| Intel RealSense D455 | 95 mm, 87 x 58 degrees, global shutter, IR projector, IMU | €622 excluding VAT (a distributor) [6] | Yes, narrow | Longer range than the OAK, but half the simulated vertical field (58 against 104 degrees): more of a motion goes unseen and is held to the unobserved speed |
| Intel RealSense D435i | 50 mm, 87 degrees | about €577 the D435if (a distributor) [6] | No | A short baseline: depth too short for the braking contract at speed |
| Stereolabs ZED X Mini | 50 mm, 110 degrees (2.2 mm lens), GMSL2, IMU | $549, plus $379 for a GMSL2 capture card [7] | No | A short baseline; the depth comes from the vendor's SDK on the GPU |

## Time-Of-Flight Ring

| Unit | Zones, field, range | Price | Fits | Why |
|---|---|---|---|---|
| ST VL53L5CX on a carrier (Pololu #3417) | 8 x 8 zones, 45 x 45 degrees (65 diagonal), up to 4 m, up to 15 Hz at 8 x 8 | $24.95 each, six $149.70 [8] | **Yes** | The simulated unit as it is ([camera_perception.md](camera_perception.md)); six on one I2C bus need their addresses set or a multiplexer. No driver in the stack yet |
| ST VL53L8CX on a carrier (Pololu #3419) | the same matrix, better in ambient light | not priced | Yes | The same contract |

## 3D Lidar

| Lidar | Field, rate, range, mass | Price | Fits | Why |
|---|---|---|---|---|
| Livox Mid-360 | 360 x 59 degrees (-7 to 52), 200 000 points a second, 10 Hz, 40 m at 10 percent reflectivity, 265 g [9] | $979 (Livox's list price, September 2026) [10] | **Yes, with work** | The range covers the 14 m guaranteed detection with margin. The field leaves the cones above 52 and below -7 degrees unseen (an up and a down time-of-flight unit, 2.8 m, cover the near part); a non-repetitive scan over 100 ms needs motion compensation, which the lidar-inertial estimator does not have |
| Unitree L2 | 360 x 96 degrees, 64 000 points a second, 30 m at 90 percent, 15 m at 10 percent, 230 g [11] | $419 [11] | Marginal | The wider field and the lowest price; 15 m on dark surfaces is the guaranteed range with nothing to spare, and a tenth of the simulated point rate reaches the obstacle memory |

## Autopilot, Airframe And The Rest

| Part | Price | Fits | Why |
|---|---|---|---|
| Holybro Pixhawk 6C | $211.99 [12] | Yes | PX4 with uXRCE-DDS, as in the simulator; the agent link is UDP only today (roadmap item 27 block 2) |
| Holybro X500 V2, PX4 development kit | $507.99 at Holybro (offered with the Pixhawk 6C or 6X; what the 6C kit includes to confirm) [13]; $946 to $1 150 with the 6X elsewhere [13] | Yes | The airframe the simulated model is fitted to, not a requirement (specification V7); 610 g, 500 mm, about 18 min of hover with no payload on 4S 5000 mAh, a payload of 1.5 kg without the battery at 70 percent throttle by Holybro, about 1 kg with a 4S 5000 mAh by a retailer [14] |
| Flood LED with a switchable driver that reports its state | not priced | Needed on the camera builds | The carried light (K11, K12); the light's own health reading lets the vehicle tell a failing light from a dark place, which today it cannot (specification I7; roadmap item 29, F18) |
| Radio control with a kill switch, telemetry radio, 4S battery | not priced | Needed | The pilot's authority and the kill switch (K29); the main battery's return (K32) |

### Complete Vehicles Of The Class Products Go To

| Vehicle | Size, mass | Flight time | On board | Price |
|---|---|---|---|---|
| ModalAI Starling 2 | 220 mm across, 280 g at take-off | over 40 min | VOXL 2, three tracking cameras, a colour camera, a time-of-flight camera, PX4 | $2 949.99 [24] |
| ModalAI Starling 2 Max | 322 mm, 500 g, 500 g of payload besides | over 55 min | VOXL 2, PX4 | $2 999.99 [24] |
| The x500 today | 500 mm, about 2 kg built | about 18 min without payload | any computer, a Pixhawk | $508 to $1 150 the kit |

Development vehicles both, not products; what they show is the class: a
VOXL 2 makes a vehicle of a few hundred grams that flies two to three
times as long as the x500. Whether the x500 is the vehicle at all is open
(specification V7); its size against the routes flown is in
[design_questions.md](design_questions.md) and roadmap item 30.

## Builds

Totals are of the priced parts; the carrier board (€165 to $309), lenses,
light, battery, radios, cabling and mounts come on top, and the autopilot's
$212 too if the airframe kit does not include it. The same builds counted
the other way, with the carrier board and the autopilot apart from a frame
kit without one, are compared with the builds without CUDA below.

| Build | Parts | Priced total | Status | Why |
|---|---|---|---|---|
| C1. Cameras, the simulated geometry | Orin NX 16GB, a 0.20 m global-shutter pair, six VL53L5CX, light, Pixhawk 6C, X500 V2 | $1 830 | **Recommended for the camera profile** | The geometry the speeds were measured with; the cheapest sensors; costs a calibration and the matcher's cores |
| C2. Cameras, depth on the camera | Orin NX 16GB, OAK-D Pro W, six VL53L5CX, light, Pixhawk 6C, X500 V2 | $2 106 | Fits, slower | Frees the CPU and brings its own IR light; a shorter range and a depth adapter to write |
| L1. Lidar | Orin NX 16GB, Livox Mid-360, an up and a down VL53L5CX, Pixhawk 6C, X500 V2 | $2 536 | **Recommended for the lidar profile** | The range the lidar profile's speed rests on; needs motion compensation in the estimator |
| L2. Lidar, cheap | Orin NX 16GB, Unitree L2, an up and a down VL53L5CX, Pixhawk 6C, X500 V2 | $1 976 | Marginal | Range and point rate at the edge of what the lidar profile assumes |
| B. Bench | Orin Nano Super developer kit with the camera of C1 or C2 | $573 to $848 | For the bench | The arm64 build, the stack on recorded data and the cameras on a bench (roadmap item 27's first steps); not for flight |
| Not fitting | a VOXL 2, a Raspberry Pi, an Orin Nano in flight, an AGX Orin on the X500 | — | No | No CUDA; too few cores; too heavy for the airframe |

## Without CUDA

What the boards above would be if the MPPI engine had a backend without CUDA
(roadmap item 28: a CPU cannot carry the rollouts in flight, so the backend
is another GPU interface, Vulkan compute or OpenCL). Against it stands the
Orin NX 16GB's GPU, 1.9 to 2.4 TFLOPS in FP32 [15], itself four and a half
to six times smaller than the development host's: every board here flies
fewer rollouts than 8192, and how many are enough is item 28's first step.

| Computer | CPU | GPU | Price, mass | Would fit | Why |
|---|---|---|---|---|---|
| An x86 board with a strong CPU and an integrated GPU, e.g. LattePanda Sigma (Core i5-1340P) | 12 cores, 16 threads, near the development host | Iris Xe, about 1.9 TFLOPS (80 execution units at 1.45 GHz, by arithmetic), the Orin NX's level | A$1 240 to 1 310; 438 g; 7 W headless to a 44 W processor [16] | In compute yes, with Vulkan or OpenCL | Heavy for the X500 V2's payload of about 1 kg |
| ModalAI VOXL 2 (QRB5165) | 1 Cortex-A77 at 3.09 GHz, 3 at 2.42, 4 Cortex-A55 | Adreno 650, OpenCL 2.0 and Vulkan 1.1 [17]; its FP32 rate not found, about 1 TFLOPS is a guess | $1 249.99 with the autopilot; 16 g [3] | Perhaps, with work | Fewer rollouts, the stereo depth on its own accelerators; four large cores against the stack's 3.4 to 5.7 Zen 3 cores at p95, to measure |
| Rockchip RK3588 (Radxa ROCK 5B+, 16 GiB) | 4 Cortex-A76 at 2.4 GHz, 4 Cortex-A55 | Mali-G610 MP4, 0.25 to 0.45 TFLOPS as measured [18] | about $120 to $190 [18] | Not for flight | A GPU four to eight times smaller than the Orin NX's and four large cores; a cheap bench or a machine for continuous integration |
| Raspberry Pi 5 | 4 Cortex-A76 | small | — | No | Neither the CPU nor the GPU |
| AMD Ryzen mini boards (Radeon 780M) | 8 Zen 4 cores | larger than the Iris Xe | not priced | In compute yes | Heavy and power-hungry |

### Cheaper Or Simpler Without CUDA?

The computer and the autopilot are where the builds differ:

| Computer and autopilot | Parts | Price | Mass | Flies the stack? |
|---|---|---|---|---|
| Jetson (today's builds) | Orin NX 16GB $999, an A603 carrier €165 to $309 [19], a Pixhawk 6C $212 | about $1 380 to $1 520 | the module, carrier, heatsink and Pixhawk, not weighed | Yes; an arm64 build and fewer rollouts to come |
| VOXL 2 | one board, PX4 on it | $1 250 | 16 g the board | Perhaps: a Vulkan or OpenCL backend, the CPU tight; whether its PX4 carries the uXRCE-DDS bridge the stack uses is not checked [21] |
| x86 with an integrated GPU (LattePanda Sigma) | the board A$1 240 to 1 310 (about $810 to $860), a Pixhawk 6C $212 | about $1 020 to $1 070 | 438 g, 7 to 44 W | In compute yes, with a Vulkan or OpenCL backend |
| RK3588 (ROCK 5B+) | the board $120 to $190, a Pixhawk 6C $212 | about $330 to $400 | light | No: the GPU and the CPU too small; a bench |

Whole builds, counted alike: an X500 V2 frame kit without an autopilot
(frame, motors, ESCs) at about €380, some $420 [22]; six VL53L5CX at $150
(two on the lidar builds, $50); light, battery and radios left out, the same
for all.

| Build | Perception | Total |
|---|---|---|
| Jetson with its own pair on CSI (C1) | $174 | about $2 135 to $2 265 |
| Jetson with an OAK-D Pro W (C2) | $449 | about $2 410 to $2 540 |
| VOXL 2 with an OAK-D Pro W | $449 | about $2 270 |
| x86 with an OAK-D Pro W | $449 | about $2 060, perhaps with a larger frame |
| Jetson with a Livox Mid-360 | $979 | about $2 840 to $2 970 |
| VOXL 2 with a Livox Mid-360 | $979 | about $2 700 (Ethernet through an expansion board, not checked) |
| x86 with a Livox Mid-360 | $979 | about $2 490 |

A camera pair of one's own on CSI ($174) belongs to its platform: an x86
board has no CSI and the VOXL 2 its own camera family, so the camera builds
without CUDA would most likely take a USB stereo camera at $449, and that
eats the saving. The cheapest camera build that flies, the Jetson with its
own pair, costs about what the builds without NVIDIA cost.

- **Cheaper:** a little. The VOXL 2 saves about $140 to $270 on the computer
  and the autopilot (5 to 10 percent of a build), an x86 board about $350 to
  $450 (15 to 20 percent) and weighs 0.3 to 0.4 kg more, which costs flight
  time and perhaps a larger frame. The cheap boards (RK3588, Raspberry Pi)
  do not carry the stack.
- **Simpler in hardware:** the VOXL 2 alone, one board in place of a
  computer, a carrier and a Pixhawk, some 100 to 200 g lighter (an estimate,
  nothing weighed). An x86 board is harder: mass, power, cooling.
- **Simpler in software:** none. Each needs a Vulkan or OpenCL backend (L to
  XL, weeks, roadmap item 28) and the controller tuned again on fewer
  rollouts. One thing goes the other way on x86: no arm64 port, which the
  Jetson still needs (item 27 block 2).
- **Whether it pays:** weeks of a backend against $150 to $450 a vehicle
  pays for a fleet of tens, or where mass and size matter, and the vehicle
  is meant to be made in numbers (specification R6): at a hundred vehicles
  the saving is $15 000 to $45 000, and the VOXL 2's 16 g and integration
  make a smaller, lighter vehicle possible. The larger lever is below.

## For A Product Made In Numbers

The vehicle is meant to be a product made in numbers (specification R6), so
the registry is read for the cost of a vehicle's parts and its mass, not for
one prototype.

- **The largest lever is the stack's own appetite for compute.** It needs
  3.4 to 5.7 Zen 3 cores at p95 and 6.4 ms of an 11 TFLOPS GPU a tick
  ([resource_budget.md](resource_budget.md)); the boards that carry it cost
  about $1 000 to $1 500 with the autopilot. A stack that flew on an Orin
  Nano ($399) or on a board of the RK3588 class ($120 to $190) would save
  $600 to $1 300 a vehicle, more than any choice of GPU interface. The
  levers: fewer rollouts (roadmap item 28, its first step), the stereo
  matching off the CPU (item 26), the persistent planner's budget (it spends
  its whole 150 ms, a setting), the memory's cost; each measured, none
  bought by flying worse (R1 to R3).
- **The camera build is the base product.** A lidar ($419 to $979) is a
  variant for those who pay for it; the camera pair of the project's own
  (calibrated by K33) over a packaged stereo camera at $449; the ToF ring's
  sensors are cheap.
- **One board for the computer and the autopilot**, the VOXL 2's class, or a
  system-on-module on a carrier board of the project's own: fewer parts, less
  mass, less cabling to assemble.
- **The prices here are single units**, but NVIDIA's $999 for the Orin NX
  16GB is already its price per thousand; at volume a module goes on a
  carrier board of one's own.

The questions that move the choice most, and the owner's answers of
2026-10-13 (specification R6, V7):

- **The market: not set.** The vehicle goes where it fits best and is most
  convenient. Should that be government customers in the United States, the
  rules on parts made in China (NDAA, the Blue UAS lists) would exclude much
  of this registry (Livox, Unitree, Rockchip, many hobby autopilots) and
  favour the VOXL 2, which ModalAI places in the Blue UAS Framework [23].
- **The target price: not set.** The hardware is made cheaper by adapting
  the software, never at the cost of the product's quality, the flight speed
  included.
- **The size and mass class: open.** The 500 mm quadcopter was chosen at the
  project's start and is neither a goal nor a constraint; a smaller or
  lighter vehicle is a candidate, and the boards that integrate the computer
  and the autopilot (the VOXL 2's class) are what makes it possible.

## What The Software Supports Today

| Sensor | In the simulator | For real hardware |
|---|---|---|
| Stereo pair | Gazebo's images taken inside the depth node's process | Plain `sensor_msgs/Image` with the pair's calibration and rectification (K33, run on EuRoC's 752 x 480 pair); no camera driver, the pair's stamps taken as equal |
| Depth from the camera itself | — | Not supported: the obstacle memory takes the depth node's returns; an adapter node is to write |
| Time-of-flight ring | Gazebo's `gpu_lidar` in the 8 x 8 layout | No driver |
| 3D lidar | An organised 360 x 181 cloud, no motion during a sweep | No driver adapter, no motion compensation |
| IMU | PX4's `sensor_combined` over uXRCE-DDS | Carries over; the cameras' and lidars' own IMUs are unused |
| Light | Set by the evaluation, its charge simulated | No control and no telemetry |
| Autopilot | PX4 SITL over uXRCE-DDS (UDP) | Carries over; the serial link and the parameter set for a real vehicle (K34) |

## What This Registry Does Not Know

- How the stack runs on any Orin: no arm64 build, no measurement. The first
  number to take is the camera and the lidar profile's cores and tick on an
  Orin NX 16GB with recorded data.
- How many rollouts the Orin NX's 1024 CUDA cores carry in the tick, and
  what fewer rollouts cost the controller.
- The mass and the power of each build against the X500 V2's payload and
  hover time; nothing has been weighed.
- Whether a real light's reach and spread match the simulated one, and how
  the camera's gain behaves with it.

## Sources

Prices and specifications as found on 2026-10-13.

1. Jetson module prices after the July 2026 increase: [CNX Software, 2026-07-22](https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/), [Notebookcheck](https://www.notebookcheck.net/Nvidia-quietly-increases-Jetson-prices-by-up-to-101.1348323.0.html).
2. Developer kits and AGX modules: [HWBusters, 2026-07-21](https://hwbusters.com/news/nvidia-jetson-prices-jump-up-to-101-the-249-orin-nano-super-is-now-399/).
3. [ModalAI VOXL 2](https://www.modalai.com/products/voxl-2).
4. [Arducam OV9281 stereo kit at RobotShop](https://www.robotshop.com/en/arducam-stereoscopic-camera-kit-w-1mp-ov9281-2x-camarray-stereo-camera-hat.html).
5. [OAK-D Pro W at Streakwave](https://streakwave.focuspointsap.com/luxonis-oak-d-pro-w-12mp-imx378-wide-fixed-2xov9282-ir); specifications: [Luxonis](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W).
6. [RealSense at RS Components](https://benl.rs-online.com/web/b/realsense); D455 specifications: [Framos](https://framos.com/en/products/depth-camera-d455-bulk-24722).
7. [ZED X Mini at Stereolabs](https://store.stereolabs.com/products/zed-x-mini-stereo-camera).
8. [Pololu VL53L5CX carrier](https://www.pololu.com/product/3417/specs).
9. [Livox Mid-360 specifications](https://www.livoxtech.com/mid-360/specs).
10. [DroneXL, 2026-09-21](https://dronexl.co/2026/09/21/livox-mid-360l-lidar-half-point-rate-robots/).
11. [Unitree L2](https://shop.unitree.com/collections/education-industry/products/unitree-4d-lidar-l2).
12. [Holybro Pixhawk 6C](https://holybro.com/products/pixhawk-6c).
13. [Holybro X500 V2 PX4 development kit](https://shop.holybro.com/x500-kit_p1180.html); [the 6X kit at Pyrodrone](https://pyrodrone.com/products/holybro-x500-v2-pixhawk-development-kit-with-the-pixhawk-6x-standard-m10-gps-915mhz-telemetry-radio).
14. [X500 V2 specifications at Elektor](https://www.elektor.com/products/holybro-x500-v2-arf-drone-kit), [mybotshop](https://www.mybotshop.de/Holybro-X500-V2-ARF-Kit_15).
15. [Jetson Orin NX 16GB at flopper.io](https://flopper.io/gpu/nvidia-jetson-orin-nx-16gb) (2.4 TFLOPS at 1173 MHz; 1.9 at 918 MHz by arithmetic).
16. [LattePanda Sigma at TechRadar](https://www.techradar.com/pro/lattepanda-sigma), [at Core Electronics](https://core-electronics.com.au/lattepanda-sigma-the-small-hackable-x86-windowslinux-single-board-computer-server-16gb-ram.html).
17. [Adreno 650 at Notebookcheck](https://www.notebookcheck.net/Adreno-650-vs-Adreno-640_9971_9446.247598.0.html).
18. [RK3588 boards at CNX Software](https://www.cnx-software.com/?p=94999), [ROCK 5B+ at RS](https://uk.rs-online.com/web/p/rock-sbc-boards/0162225?gb=s), [clpeak on a Mali-G610](https://monolithe.proj.lip6.fr/assets/benchmarks/clpeak/clpeak_opi5_gpu.txt).
19. [The A603 carrier at Robocraze](https://robocraze.com/products/seeed-studio-a603-carrier-board-for-jetson-orin%e2%84%a2-nx-orin%e2%84%a2-nano), [at rcdrone](https://rcdrone.top/ja/products/a603-carrier-board-for-jetson-orin%E2%84%A2-nx-orin%E2%84%A2-nano); Mouser lists it at €165.46.
20. [reComputer J4012 at Hackster](https://hackster.io/products/buy/111291).
21. [PX4's uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html) (nothing on the VOXL 2 there).
22. [X500 V2 ARF kit at Elektor](https://www.elektor.com/products/holybro-x500-v2-arf-drone-kit) (€379.95), [the frame kit at Flying Tech](https://www.flyingtech.co.uk/product/holybro-x500-v2-quadcopter-frame-kit/).
23. [ModalAI's VOXL 2 launch, 2022](https://www.modalai.com/blogs/press-releases/modalai%E2%93%A1-launches-voxl%E2%93%A1-2-the-smallest-smartest-blue-uas-framework-autopilot-at-16g).
24. [Starling 2](https://modalai.com/starling-2), [Starling 2 Max](https://modalai.com/starling-2-max) at ModalAI.
