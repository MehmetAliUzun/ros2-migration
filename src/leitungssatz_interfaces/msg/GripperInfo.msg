# ROS time stamp
builtin_interfaces/Time stamp

# Gripper position in mm
float64 position_mm
# Gripper position in % [0-100]
float64 position_per
# Gripper position in device [0-255]
float64 position_device

# Error Code
int64 e_fault_code  

# Object status reported by the gripper. [0-3]
# 0 - MOVING
# 1 - STOPPED_OUTER_OBJECT
# 2 - STOPPED_INNER_OBJECT
# 3 - AT_DEST
int64 e_object_status     

# Connection status
bool connected
bool activated