# Type of message ("velocity", "absolute_point", "relative_point")
string command_type
# For velocity
float32 v_x
float32 v_y
float32 v_rot
# For absolute point
float32 absolute_x
float32 absolute_y
float32 absolute_z
# For relative point (relative to Spot-recognized April tag)
float32 offset_x
float32 offset_y
float32 april_pos_x
float32 april_pos_y
float32 april_rot_x
float32 april_rot_y
float32 april_rot_z
float32 april_rot_w
bool invert_heading