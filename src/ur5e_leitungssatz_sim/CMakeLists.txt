cmake_minimum_required(VERSION 3.8)
project(ur5e_leitungssatz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
#find_package(joint_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Your custom packages
find_package(leitungssatz_interfaces REQUIRED)
find_package(leitungssatz_ros2_driver REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (controller configs)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes if they exist
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install worlds if they exist
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install RViz configs if they exist
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install models for Gazebo if they exist
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
