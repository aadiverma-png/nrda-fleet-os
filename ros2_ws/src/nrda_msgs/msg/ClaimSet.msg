# Owner-authoritative snapshot of one robot's claims (/fleet/claims, 4 Hz and on change).
string robot_id
float64 stamp
uint32 seq
int32 priority
Claim[] claims
string[] occupied_keys   # resources its body covers now (physical precedence)
int32[] occupied_idx
