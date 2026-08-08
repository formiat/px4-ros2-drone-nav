# The scan stamp includes the calibrated sensor time offset and identifies the
# resolved first-beam acquisition pose. The body frame is fixed at that pose;
# points never use persistent obstacle memory.
std_msgs/Header header
uint64 sequence
uint64 pose_generation
geometry_msgs/Point frame_origin_map
geometry_msgs/Vector3 body_x_axis_map
geometry_msgs/Vector3 body_y_axis_map
geometry_msgs/Vector3 body_z_axis_map
geometry_msgs/Point32[] hit_points_body_frd
uint32 source_beam_count
uint32 invalid_beam_count
