# The offboard producer, horizon producer, sequence, and header stamp identify
# the exact emitted feedback. Zero horizon identity with non-authoritative hold
# is a liveness heartbeat for the offboard process, not horizon revocation.
# Yaw acceleration is the applied horizon interval's yaw-rate derivative.
std_msgs/Header header
uint8 EXECUTION_MODE_PLANNED=0
uint8 EXECUTION_MODE_POSITION_HOLD=1
uint64 producer_instance_id
uint64 horizon_producer_instance_id
uint64 horizon_sequence
uint8 execution_mode
bool control_authoritative
geometry_msgs/Vector3 acceleration
float32 yaw_rate_radps
float32 yaw_acceleration_radps2
