# What the navigation knows at a planning tick that the mission monitor weighs
# (roadmap item 17 stage 5): whether the vehicle sees, and how far B is along
# its route. Published a few times a second.
std_msgs/Header header
# The braking contract's measured forward range (specification K9), the
# range the sensor set guarantees when it sees, and the share of its beams
# the forward sensor's latest frame observed (negative when none was read).
float64 sensor_measured_range_m
float64 sensor_guaranteed_range_m
float64 sensor_observed_fraction
# The committed route's length left to the mission goal; NaN while no
# committed route reaches it.
float64 route_remaining_m
