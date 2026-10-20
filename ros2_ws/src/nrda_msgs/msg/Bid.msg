# CBBA consensus message (/fleet/bids). Arrays are parallel, one entry per known task.
string robot_id
float64 stamp
int32 iteration
string[] task_ids
float64[] bids
string[] winners
float64[] timestamps
string[] bundle
