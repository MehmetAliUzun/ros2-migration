cmake_minimum_required(VERSION 3.8)
project(leitungssatz_interfaces)

# Require ament CMake and rosidl generator support
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Common message dependencies (add/remove as needed)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# List every msg/srv file you put in this package under msg/ and srv/
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperInfo.msg"
  "msg/DashboardInfo.msg"
  "msg/JogControl.msg"
  "msg/PathEntry.msg"
  "srv/SetGripper.srv"
  "srv/SetCartTarget.srv"
  "srv/SetJointTarget.srv"
  "srv/SetTrajectory.srv"
  "srv/SetSpeedSlider.srv"
  "srv/SetClipGun.srv"
  "srv/SetRelCartTarget.srv"
  "srv/StopMotion.srv"
  "srv/StartJog.srv"
  "srv/GetGripperCalib.srv"
  "srv/GetTimeStamp.srv"
  "srv/Log.srv"
  "srv/Power.srv"
  "srv/SetPayload.srv"
  "srv/SetContactTarget.srv"
  "srv/SetForceTarget.srv"
  "srv/SetFreedrive.srv"
  "srv/GrabBuchse.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces
)

# Export rosidl runtime so consumers can link to generated headers
ament_export_dependencies(rosidl_default_runtime)

ament_package()