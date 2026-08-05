uint8 STATUS_INITIALIZING=0
uint8 STATUS_TRACKING=1

std_msgs/Header header
uint64 track_id
uint64 source_scan_sequence
uint64 source_detection_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
bool position_valid
bool velocity_valid
uint8 status
