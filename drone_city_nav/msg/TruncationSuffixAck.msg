uint8 DECISION_PENDING=0
uint8 DECISION_ACCEPTED=1
uint8 DECISION_REJECTED=2

std_msgs/Header header
uint64 path_id
uint64 truncation_generation
uint64 temporary_prefix_fingerprint
uint8 decision
string reason
