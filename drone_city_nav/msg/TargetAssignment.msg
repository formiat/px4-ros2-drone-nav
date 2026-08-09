uint8 REASON_INITIAL=0
uint8 REASON_TARGET_SET_CHANGED=1
uint8 REASON_COST_IMPROVEMENT=2
uint8 REASON_REFRESH=3

std_msgs/Header header
uint64 mission_epoch
uint64 assignment_generation
string interceptor_id
uint64 target_detection_id
uint64 target_track_id
float64 estimated_intercept_time_s
uint8 reason
bool active
