float32 table_height
float32 table_height_tolerance

bool is_shelf

string close_to
string special_request
geometry_msgs/PoseStamped forced_pose

# When true, the place flow does NOT move to the initial "table_stare" pose
# first; it places from the arm's current position.
bool skip_initial_pose