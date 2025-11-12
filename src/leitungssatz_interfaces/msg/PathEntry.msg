# move type
# 0 - moveJ
# 1 - moveL
# 2 - moveP
# 3 - moveC
int8 move_type

# position_type
# 0 - PositionTcpPose
# 1 - PositionJoints
int8 position_type

# cartesian in meter and quaternion
geometry_msgs/Transform cartesian_goal

# joint goal in rad
float64[6] joint_goal

# tool speed [m/s]
float64  velocity

# tool acceleration [m/s^2] 
float64  acceleration

# blend radius in mm
float64  blend