# The bodies the obstacle memory follows moving, published with every scan it
# integrates (roadmap item 21, specification K66), stamped with the scan's
# acquisition time; an empty list says the memory follows none.
std_msgs/Header header
uint64 sequence
MovingBody3D[] bodies
