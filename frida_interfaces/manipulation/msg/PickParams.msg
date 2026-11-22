geometry_msgs/PointStamped object_point
string object_name
float32 max_distance
float32 min_distance
bool closest
bool largest
bool in_configuration # if set to false it will move the arm to an appropiate joint configuration
bool from_shelf       # pick from the shelf of the last SCAN_SHELF
int32 shelf_level     # from_shelf: the level to look on first (1 = lowest), 0 = find it