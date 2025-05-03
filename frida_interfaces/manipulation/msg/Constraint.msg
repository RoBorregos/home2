# frida_interfaces/msg/Constraint.msg
geometry_msgs/Quaternion orientation
string frame_id
string target_link
float64[] tolerance_orientation
float64 weight
int32 parameterization