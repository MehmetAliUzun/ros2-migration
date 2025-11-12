cmake_minimum_required(VERSION 3.8)
project(leitungssatz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CGAL_DATA_DIR "/usr/share/CGAL")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
# Try to find the driver package, but do not fail configure if it's not present.
# This breaks the circular dependency during bootstrapping: you can build the driver package first,
# then rebuild this package to pick up full integration.
find_package(leitungssatz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(CGAL REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Resolution.msg"
  "msg/VectorResolution.msg"
  "srv/AddTf2.srv"
  "srv/CheckComponents.srv"
  "srv/GetBeep.srv"
  "srv/GetBoardLocation.srv"
  "srv/GetFinished.srv"
  "srv/GetRedBlock.srv"
  "srv/GetTriangles.srv"
  "srv/GrabBuchse.srv"
  "srv/SelectCam.srv"
  "srv/SetCableOrientation.srv"
  "srv/SetExposure.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  rosidl_default_runtime
)

# Create a typesupport target variable once
rosidl_get_typesupport_target(rosidl_typesupport_cpp "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Create a library for the core classes
add_library(${PROJECT_NAME}_lib
  src/Robot.cpp
  src/Gripper.cpp
  src/Point.cpp
  src/PointLoader.cpp
  src/Task.cpp
  src/TaskScheduler.cpp
  src/Wrench.cpp
  src/Tasks/CableCheckTask.cpp
  src/Tasks/ForceTask.cpp
  src/Tasks/GrabBuchseTask.cpp
  src/Tasks/GripperTask.cpp
  src/Tasks/InterTask.cpp
  src/Tasks/MoveTask.cpp
  src/Tasks/SpiralTask.cpp
  src/Tasks/ToolChangeTask.cpp
  src/Tasks/WirePushTask.cpp
  src/Tasks/SetCableOrientationTask.cpp
  src/Utils/SimpleImagePublisher.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add driver dependency conditionally using DRIVER_DEPENDS variable
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  leitungssatz_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  image_transport
  cv_bridge
  Eigen3
  ${DRIVER_DEPENDS}
)

# link typesupport to the library (keyword form)
target_link_libraries(${PROJECT_NAME}_lib ${rosidl_typesupport_cpp} ${OpenCV_LIBS})

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}__rosidl_generator_cpp)

# --- executables: ensure ament_target_dependencies() comes before any target_link_libraries() calls ---

add_executable(main_program src/Programs/Main.cpp)
ament_target_dependencies(main_program
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(main_program ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(main_2terminals src/Programs/Main_2Terminals.cpp)
ament_target_dependencies(main_2terminals
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(main_2terminals ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(grab_buchse_program src/Programs/GrabBuchse.cpp)
ament_target_dependencies(grab_buchse_program
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(grab_buchse_program ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(force_test_program src/Programs/ForceTest.cpp)
ament_target_dependencies(force_test_program
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(force_test_program ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(put_buchse_program src/Programs/PutBuchseInAufnahme.cpp)
ament_target_dependencies(put_buchse_program
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(put_buchse_program ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(cable_check_program src/Programs/cableCheckProgram.cpp)
ament_target_dependencies(cable_check_program
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  ${DRIVER_DEPENDS}
)
target_link_libraries(cable_check_program ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

# Add executables for utility nodes
add_executable(point_recording src/point_recording.cpp)
ament_target_dependencies(point_recording
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  ${DRIVER_DEPENDS}
)
target_link_libraries(point_recording ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(tf2_publisher src/tf2_publisher.cpp)
ament_target_dependencies(tf2_publisher
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
)
target_link_libraries(tf2_publisher ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp})

add_executable(touch_detection_service src/touchDetection_service.cpp)
ament_target_dependencies(touch_detection_service
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  ${DRIVER_DEPENDS}
)
target_link_libraries(touch_detection_service ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp} CGAL::CGAL)

# executables for nodes including main() functions
add_executable(touch_detection_node src/touchDetection.cpp)
ament_target_dependencies(touch_detection_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  ${DRIVER_DEPENDS}
)
target_link_libraries(touch_detection_node ${PROJECT_NAME}_lib ${rosidl_typesupport_cpp} CGAL::CGAL tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)

add_executable(simple_image_publisher src/Utils/SimpleImagePublisher.cpp)
ament_target_dependencies(simple_image_publisher
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)
target_include_directories(simple_image_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(simple_image_publisher ${OpenCV_LIBS})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  main_program
  main_2terminals
  grab_buchse_program
  force_test_program
  put_buchse_program
  cable_check_program
  point_recording
  tf2_publisher
  touch_detection_service
  touch_detection_node
  simple_image_publisher
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

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

ament_package()