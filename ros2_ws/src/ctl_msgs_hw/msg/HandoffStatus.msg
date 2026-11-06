# Latest source command accepted by the servo, and current handoff state.
std_msgs/Header header
std_msgs/Header source_header
uint64 handoff_epoch
bool complete
float64[] commanded_position
