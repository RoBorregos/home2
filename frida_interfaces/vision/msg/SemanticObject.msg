# One object in the semantic map, fused over its detections.
string id
string label
string category                       # "" when objects.json does not list the label
geometry_msgs/PointStamped position   # map frame
string area                           # "" when outside every room polygon
string sublocation                    # "" when no furniture is close enough
float32 confidence                    # best detector score seen
float64 last_seen                     # seconds, ROS time
int32 observations
