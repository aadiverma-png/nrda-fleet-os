# Collision / near-miss / avoided events from the independent safety monitor (/fleet/safety_events).
string event_id
float64 stamp
string level             # COLLISION | NEAR_MISS | AVOIDED
string[] robots
string other             # robot id, rack / object name
float64 x
float64 y
float64 rel_speed
string source            # contact | overlap | gap | risk
string detail
