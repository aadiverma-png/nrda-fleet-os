# Emergency alerts and directives. Directives never actuate: the receiver's arbiter decides.
string sender
float64 stamp
string kind              # risk | brake | reverse | yield | resume | obstacle | wfg_yield
string target
float64 x
float64 y
float64 value
string detail
