std_msgs/Header header
unique_identifier_msgs/UUID mission_uuid
string coordinator_instance
uint64 event_sequence
string phase
string result_code
string reason
ControlSession control_session
string map_session
unique_identifier_msgs/UUID child_uuid
bool has_child
float64 total_remaining_s
bool mock
# PX4 root plan revision; changes on return override and checkpoint restoration.
uint32 plan_revision
# Checkpoint capability, separate from actual PAUSED/health admission.
bool can_resume_mission
int32 resume_step_index
