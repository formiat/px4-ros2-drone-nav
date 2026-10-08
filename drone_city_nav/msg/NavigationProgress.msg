# What the navigation knows at a planning tick that the mission monitor weighs
# (roadmap item 17 stage 5): whether the vehicle sees, and how far B is along
# its route. Published a few times a second.
std_msgs/Header header
# The forward range the sensor's latest frame alone stands behind (the braking
# contract's measured range, specification K9, before the memory answers back
# along the flown path, K15) and the range the sensor set guarantees when it
# sees: their ratio is what the mission monitor judges the carried light by
# (K12).
float64 sensor_measured_range_m
float64 sensor_guaranteed_range_m
# The share of its configured range the frame alone grants, which the measured
# range is made of: the range stops at the braking contract's physical margin
# (2.001 m of 6.4 m, 31 %, in the dark), the share does not (0.04 there).
# What the evaluation overlay shows as what the vehicle sees (K26).
float64 sensor_frame_share
# The committed route's length left to the mission goal; NaN while no
# committed route reaches it.
float64 route_remaining_m
