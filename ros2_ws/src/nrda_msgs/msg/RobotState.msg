# Broadcast by every edge agent at 10 Hz on /fleet/state. Also the heartbeat.
string robot_id          # AMR_<n>
float64 stamp            # sim time [s]
float64 x                # map frame [m]
float64 y
float64 th               # heading [rad]
float64 v                # [m/s], negative when reversing
float64 w                # [rad/s]
string mode              # idle | to_pickup | loading | to_drop | unloading | to_charger | charging | to_park | parked | paused | estop | fault
float32 battery          # state of charge 0..1
string task_id
string health            # HEALTHY | DEGRADED | COMM_LOSS | MOTION_FAILURE | EMERGENCY_STOP | FAILED | RECOVERING
string lane
string goal              # station it is heading to
string bypass            # visison controller state while a bypass runs
float32 compute_ms       # p95 planning time on board
float32[] path_x         # upcoming route polyline
float32[] path_y
string[] waiting_for     # wait-for graph edges
string station           # station it is docked at / serving
