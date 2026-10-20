# One space-time claim: resource (key, idx) is needed during (start, end).
string key               # lane "<line>/<A|B>", station "S:<name>", free-space "F:<i>"
int32 idx                # cell index
float64 start
float64 end
float64 stamp            # when this claim was first made (precedence)
