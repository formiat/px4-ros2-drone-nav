# A body the obstacle memory follows moving (roadmap item 21, specification
# K66): the mean of its voxels, the horizontal velocity its centre was measured
# moving at (zero while it stands), its horizontal reach and its bottom and
# top, all in the map frame, how long it has been followed, and whether the
# thermal core sighted a warm body in its returns (K64).
geometry_msgs/Point centre_map
geometry_msgs/Vector3 velocity_map_mps
float64 radius_m
float64 bottom_m
float64 top_m
float64 age_s
uint32 voxels
bool warm
