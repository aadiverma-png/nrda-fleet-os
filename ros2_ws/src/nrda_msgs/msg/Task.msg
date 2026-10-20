# A transport task released to the fleet (/fleet/tasks, transient local).
string task_id
float64 stamp
string pickup            # station name (picker zone, decant, overflow induct)
string dropoff           # station name (drop, packing induct)
int32 priority           # 1 LOW, 2 NORMAL, 3 HIGH, 4 CRITICAL
float64 deadline         # sim time, 0 = none
string assign_to         # operator override (AMR_<n>) or empty for decentralised CBBA
string kind
bool cancelled
