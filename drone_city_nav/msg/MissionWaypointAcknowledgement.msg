# Durable planner-owned waypoint progress. The producer/acknowledgement tuple
# is ordered within one planner process. The producer identity intentionally is
# the same execution-horizon producer identity carried below, binding the
# acknowledgement to the process that committed the witnessed horizon.
# Aggregate counts make a dropped event
# or late-joining monitor recoverable from the newest retained sample.
std_msgs/Header header
uint64 producer_instance_id
uint64 acknowledgement_sequence
uint64 mission_epoch
uint32 completed_waypoint_index
uint32 completed_waypoint_count
uint32 waypoint_count
uint32 active_waypoint_index
bool mission_completed
geometry_msgs/Point completed_goal
uint64 horizon_producer_instance_id
uint64 horizon_sequence
uint64 offboard_producer_instance_id
builtin_interfaces/Time horizon_valid_from
builtin_interfaces/Time horizon_valid_until
builtin_interfaces/Time witness_stamp
geometry_msgs/Point route_target
geometry_msgs/Point stationary_hold_position
