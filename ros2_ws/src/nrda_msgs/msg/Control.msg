# Operator commands from the web console (/fleet/control, transient local).
float64 stamp
string command           # estop | release_estop | pause | resume | charge | fault | clear_fault | zone_rule
string target            # AMR_<n> or empty for the whole fleet
string value
