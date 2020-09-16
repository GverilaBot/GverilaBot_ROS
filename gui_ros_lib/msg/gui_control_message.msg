# Define controll command ()
# 1 - Joint position control 
# 2 - set position (xyz)
# 3 - set pose (position + orientation)
int8 command_type

# set robot joints
float64[] joints
# set robot pose 
gui_ros_lib/gui_pose pose
# set robot waypoints
gui_ros_lib/gui_pose[] waypoints
