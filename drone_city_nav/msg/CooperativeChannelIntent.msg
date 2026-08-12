uint8 PHASE_NONE=0
uint8 PHASE_APPROACH=1
uint8 PHASE_TRAVERSAL=2
uint8 PHASE_DEPARTURE=3

string channel_id
string conflict_resource_id
uint64 route_generation
uint8 phase
uint16 lane_index
uint16 lane_count
int8 direction_sign
float64 station_m
float64 distance_to_entry_m
float64 distance_to_exit_m
builtin_interfaces/Time predicted_entry
builtin_interfaces/Time predicted_exit
