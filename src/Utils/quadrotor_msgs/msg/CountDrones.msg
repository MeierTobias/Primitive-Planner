# Interpreted as an enum, see C code
int8 type

# Always send the fragment info so that our neighbours can listen
# and update their neigbours table without us having to warn them explicitly
int64 sender
int64 rootUuid

# Can also be reserved values -1 (everyone) or -2 (my_children), see C code
int64 recipient

# How this is interpreted depends on the message type
int64 payload

# This is sent to ensure (in software) that only the nodes that are close to the drone will receive the message
geometry_msgs/Vector3 position
