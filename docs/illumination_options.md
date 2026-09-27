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
