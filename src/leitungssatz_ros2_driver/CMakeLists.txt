cmake_minimum_required(VERSION 3.8)
project(leitungssatz_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ROS 2 runtime / API dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

# Shared interfaces package (generated messages & services)
find_package(leitungssatz_interfaces REQUIRED)
# If AddTf2.srv stays in leitungssatz, the driver must also find that package
find_package(leitungssatz REQUIRED)

# Optional UR RTDE library - if not present we provide a MOCK_RTDE option
option(ENABLE_UR_RTDE "Link against ur_rtde if available" ON)
if(ENABLE_UR_RTDE)
  find_package(ur_rtde QUIET)
  if(NOT ur_rtde_FOUND)
    message(WARNING "ur_rtde not found: building in mock mode (no real robot). Set ENABLE_UR_RTDE=OFF to silence.")
    set(ENABLE_UR_RTDE OFF)
  endif()
endif()

include_directories(include)

add_library(ur_hardware_interface SHARED
  src/hw_interface.cpp
)

# Ensure this target waits for rosidl-generated headers in the other packages.
# The generator targets' names are <pkg>__rosidl_generator_cpp
# If you depend on messages from both packages, add both as deps so CMake orders builds correctly.
if (TARGET leitungssatz__rosidl_generator_cpp)
  add_dependencies(ur_hardware_interface leitungssatz__rosidl_generator_cpp)
endif()

if (TARGET leitungssatz_interfaces__rosidl_generator_cpp)
  add_dependencies(ur_hardware_interface leitungssatz_interfaces__rosidl_generator_cpp)
endif()

if(ENABLE_UR_RTDE AND ur_rtde_FOUND)
  ament_target_dependencies(ur_hardware_interface
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    controller_manager
    realtime_tools
    geometry_msgs
    std_msgs
    sensor_msgs
    ament_index_cpp
    ur_rtde
    leitungssatz_interfaces
    leitungssatz
    tf2
    tf2_eigen
  )
else()
  target_compile_definitions(ur_hardware_interface PRIVATE MOCK_RTDE)
  ament_target_dependencies(ur_hardware_interface
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    controller_manager
    realtime_tools
    geometry_msgs
    std_msgs
    sensor_msgs
    ament_index_cpp
    leitungssatz_interfaces
    leitungssatz
    tf2
    tf2_eigen
  )
endif()

install(TARGETS ur_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# optional: install plugin xml if you have one
install(FILES resources/hardware_interface_plugin.xml DESTINATION share/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/ DESTINATION include)

ament_package()