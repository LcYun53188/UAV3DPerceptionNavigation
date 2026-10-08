# Published only after a successful BT tick, using the ROS Action root UUID.
# Sequence must strictly increase within the current coordinator instance.
unique_identifier_msgs/UUID mission_uuid
string coordinator_instance
uint64 tick_sequence
