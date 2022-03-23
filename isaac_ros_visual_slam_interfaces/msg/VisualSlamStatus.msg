# This is a status message which gives insights about tracker status and execution time which can help in diagnostics and profiling.
# The frame id in the header is used as the reference frame of both the transforms below.
std_msgs/Header header

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

# The total time it takes to process the input frames to output messages in seconds.
float64 node_callback_execution_time

# Time it takes to get current pose out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time

# Mean time to get poses out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time_mean

# Max time to get poses out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time_max
