float32 table_height
float32 table_height_tolerance

bool is_shelf

# When true, the place is into a trash bin
bool is_trash

string close_to
string special_request
geometry_msgs/PoseStamped forced_pose

# When true, the place flow does NOT move to the initial "table_stare" pose
bool skip_initial_pose

# Place on this level of the last SCAN_SHELF (1 = lowest); 0 = not a scanned shelf
int32 shelf_level
