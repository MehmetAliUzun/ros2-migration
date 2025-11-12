# ROS time stamp
builtin_interfaces/Time stamp

# Configures to move to move with respect to:
# 1 - base frame
# 2 - tool frame
# 3 - custom frame
int8 feature

# Speed vector for translation and rotation in mm/s and rad/s
float64[6] speeds

# The custom_frame given as pose if the selected feature is 
float64[6] custom_frame