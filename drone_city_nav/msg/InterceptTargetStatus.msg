uint8 STATUS_ACTIVE=0
uint8 STATUS_INTERCEPTED=1
uint8 STATUS_REACHED_GOAL=2
uint8 STATUS_DESTROYED=3

std_msgs/Header header
uint64 mission_epoch
string target_id
uint64 target_detection_id
uint8 status
string capturing_interceptor_id
string detail
