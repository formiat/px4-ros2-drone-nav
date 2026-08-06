uint8 ROLE_UNSPECIFIED=0
uint8 ROLE_INTERCEPTOR=1
uint8 ROLE_EVADER=2

uint8 CAUSE_PHYSICAL_COLLISION=1
uint8 CAUSE_PROXIMITY_INTERCEPT=2
uint8 CAUSE_PROXIMITY_COLLISION=3

builtin_interfaces/Time stamp
uint64 mission_epoch
uint8 vehicle_role
string vehicle_id
uint8 death_cause
string detail
string drone_collision
string obstacle_collision
geometry_msgs/Point event_position
float64 altitude_m
float64 speed_mps
