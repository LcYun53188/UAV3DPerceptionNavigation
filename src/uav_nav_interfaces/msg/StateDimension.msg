# A current fact, never a cached last-known authorization.
# UNKNOWN/INVALID values carry valid=false. Ages are -1 when no source exists.
string value
bool valid
string reason
float64 source_time_s
float64 source_age_s
float64 receive_age_s
