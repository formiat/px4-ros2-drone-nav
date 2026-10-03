# Illumination Options For The Camera Vehicle

A study note for roadmap item 17 stage 3 (light on the vehicle), written on
2026-09-27 from a discussion with the project owner. It records the options
for lighting the scene from the vehicle other than a continuous flood lamp or
a 3D lidar, what each costs in energy, and which to measure first. Every
figure below is an order of magnitude to be replaced by a sourced datasheet
figure before stage 3 builds anything; none is a measurement of this project.

## The Problem With A Flood Lamp

Item 17's condition of 2026-09-20 is a price: the additions must not approach
the price of one ordinary 3D lidar. Energy points the same way. A continuous
LED flood that gives the stereo pair a confident 3 to 4 m over a 120 degree
field needs tens of watts: illuminance falls with the square of the distance,
and the whole field has to be flooded rather than spotted (the DARPA SubT
teams carried LED panels of tens of watts behind diffusers). A cheap
solid-state 3D lidar of the Livox Mid-360 class draws about 6 to 7 W. The
principle "light the whole field all the time" is what loses, not the camera.

Nothing here changes roadmap item 22, which flies with no illumination at all
on thermal cameras; this note is about lighting the scene cheaply.

## The Options

From the most promising to the least, for a stereo camera vehicle.

1. **Strobed illumination synchronized with the exposure.** The LEDs flash
   only while the global shutter is open, 1 to 2 ms per frame: at 15 frames a
   second that is 2 to 3 percent of the time, so the peak brightness of a
   flood lamp costs 30 to 50 times less on average — watts instead of tens of
   watts. The short exposure also removes motion blur. It needs a global
   shutter, which the pair needs anyway, and a trigger that ties the flash to
   the frame. Standard practice in industrial machine vision.
2. **Near infrared (850 or 940 nm) instead of white light.** Monochrome cameras
   without an infrared cut filter see it and people do not. It saves no
   energy by itself, a photon being a photon, but with a narrow band-pass
   filter on the cameras the ambient light stops competing and less power is
   needed for the same contrast. It combines naturally with option 1.
3. **Active stereo with a dot projector** (the RealSense D4xx class). A
   projector paints a pattern of infrared dots on the surfaces, so the stereo
   matcher finds correspondences on a blank wall and in the dark. The
   projector draws on the order of a watt and the whole module a few watts.
   It answers both of item 17's failures at once, too few photons and too
   little texture. Its range indoors is about 3 to 6 m. Whether gz-sim can
   render a projected pattern for the sensor cameras is to be checked before
   the option is chosen: it is not known to be supported out of the box.
4. **A time-of-flight camera** (flash ToF, the PMD or Infineon REAL3 class).
   Not a scanning lidar but a matrix of about 224 x 172 that lights the scene
   with its own infrared pulse and measures the range at every pixel: dense
   depth in total darkness with no stereo matching, for a fraction of a watt
   to about two watts, over 4 to 6 m. It is the larger sibling of the 8 x 8
   time-of-flight sensors the vehicle already carries. The simulator renders
   it as a depth camera. Its limits are the short range and the accuracy on
   dark and specular surfaces.
5. **A laser line or sheet.** One bright line across the scene gives sparse
   depth for very little energy. Too little for navigation alone; a
   supplement at most.
6. **Light only where and when it is needed.** Illuminate the cone in the
   direction of motion rather than the whole field, and set the power by the
   range the flight needs: the braking contract already ties speed to range,
   so a slower flight needs to see less far and less light. More sensitive
   sensors (large pixels, the Sony STARVIS class) and pixel binning lower the
   light needed in the same way. A policy over the options above, not an
   alternative to them.

## Recommendation

For item 17 stage 3: the stereo pair with a strobed near-infrared flood
(options 1 and 2), with a dot projector where the scene has no texture
(option 3); a time-of-flight camera (option 4) as the alternative source of
near range. The choice is made by measurement: stage 3 compares the options
on the confident range each gives, their price and their average electrical
power, and the vehicle carries the one that wins on all three or the owner's
choice between them.

## Measured In Simulation (2026-09-28)

Stage 3 was flown in the dark variant of the location (no ambient light),
the stereo pair's images carrying Gaussian noise of 0.01 of the range
(2.5 grey levels) and an automatic gain of 1 to 8. The vehicle carries one
spot light at the pair.

- **The light has to cover the pair's whole field.** A spot's angles in
  Gazebo are full cone angles; a cone of 1.3 rad lit 37 degrees either side
  of the axis while the braking contract answers for motion up to 60
  degrees off it by what the pair sees (r769). The light's inner cone now
  spans the pair's 120 degree width and its outer cone the 130 degree
  diagonal.
- **Range against the light's strength.** At the Gazebo intensity 8 the
  frames saturated at the lowest gain (r770, 1.58 m/s); at 2 the contract
  measured its full 6.4 m range on every frame and the flight ran 1.88 m/s
  (r771); at 0.5 the range fell to 2.6 m in places and the speed to 1.38
  m/s (r772). The vehicle carries intensity 2. There the matched surfaces
  at 6 m read 28 of 255 before the gain (median over the flight), eleven
  times the image noise, and at 0.5 about 8 to 11, three to four times it,
  which the matcher's noise mask begins to refuse.
- **The light moves with the cameras,** which broke the feature tracker of
  the visual-inertial odometry: surfaces brighten as the vehicle nears them
  and the light's falloff sweeps the scene. The tracker now follows the
  frame's texture (the grey level less its local mean); see
  [`localization.md`](localization.md).
- **Active stereo can be rendered.** Gazebo Harmonic draws an SDF
  `<projector>`'s pattern into the sensor cameras' images in total darkness
  (measured on rendered frames, not kept: 15.6 % of a wall's pixels lit by a dot pattern,
  none without it). The pattern is a decal, though: its brightness does not
  fall with distance, so the range of a projector would be the far clip it
  is given, a datasheet figure rather than a measurement. It is kept for
  stage 2 (surfaces without texture), where it is one of the remedies to
  compare.

### Energy

The simulation's light carries no photometric unit; its energy follows
from the signal it gives. A matched surface at the contract's 6.4 m needs
eleven times the noise, which for a real sensor limited by its own shot
noise (and three electrons of read noise) is about 130 electrons in a pixel.
With a 3 um pixel, an f/2.0 lens transmitting 0.9, surfaces reflecting 0.3
on average, the pair's 3.0 sr field and 7.5 frames a second:

| Light | Quantum efficiency | Light per frame at the scene | Average electrical power |
|---|---|---|---|
| Continuous white flood, 10 ms exposure | 0.65 at 550 nm | 0.058 J | about 15 W (73 W at a 2 ms exposure) |
| Continuous 850 nm flood, 10 ms exposure | 0.30 | 0.082 J | about 20 W (100 W at 2 ms) |
| Strobed 850 nm flood, synchronized with a global shutter | 0.30 | 0.082 J | about 1.5 W, whatever the exposure |
| Strobed white flood | 0.65 | 0.058 J | about 1.1 W |

LEDs are taken at a wall-plug efficiency of 0.4. A strobe spends the light
only while the shutter is open, so its average power is the light one
frame needs times the frame rate; a continuous light spends it for the
whole frame period, the exposure's fraction of which it wastes. The
cheap solid-state 3D lidar draws 6.5 W.

### Choice

The vehicle carries a **strobed near-infrared flood synchronized with the
pair's global shutter** (options 1 and 2): about 1.5 W on average, some
tens of dollars in LEDs, a driver and band-pass filters, and the contract's
full 6.4 m range as flown. Near infrared rather than white keeps the light
invisible to people and, behind a band-pass filter, clear of the ambient
light. In the simulation it is the spot light at intensity 2 over the
pair's field: a strobe lights the exposure exactly as a continuous light
does, and only the energy differs. The time-of-flight camera (option 4) was
not needed for the range; its place is item 22 (thermal flight), which
allows it. Active stereo (option 3) goes to stage 2. The laser line
(option 5) stays a supplement at most. Recorded in
[`specification.md`](specification.md) as F12.
