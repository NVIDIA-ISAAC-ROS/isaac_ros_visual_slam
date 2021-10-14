# The frame id in the header is used as the reference frame of both the transforms below.
std_msgs/Header header

# The frame id of the child frame to which the transforms point.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
# rectified_tranform represents the most accurate position and orientation.
# It shall be consumed if robust tracking is needed. Note: It can result in sudden jumps.
geometry_msgs/Transform rectified_transform

# smooth_transform represents the position and orientation which corresponds to a smooth continous motion.
# It shall be consumed if using as another signal for e.g. in case of sensor fusion.
# Note: It might drift over time  
geometry_msgs/Transform smooth_transform

# Pure visual odometry return code:
# 0 - Unknown state
# 1 - Success
# 2 - Failed
# 3 - Success but invalidated by IMU
uint8 vo_state

# Integrator status:
# 0 - Unknown state
# 1 - Static
# 2 - Inertial
# 3 - IMU
uint8 integrator_state
