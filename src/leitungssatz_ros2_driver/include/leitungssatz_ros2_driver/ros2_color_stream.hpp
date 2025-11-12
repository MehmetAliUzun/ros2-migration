#pragma once
#include <rclcpp/rclcpp.hpp>

// Defining your own ROS 2 stream color
#define ROS_BLACK_STREAM(logger, x)   RCLCPP_INFO_STREAM(logger, "\033[1;30m" << x << "\033[0m")
#define ROS_RED_STREAM(logger, x)     RCLCPP_INFO_STREAM(logger, "\033[1;31m" << x << "\033[0m")
#define ROS_GREEN_STREAM(logger, x)   RCLCPP_INFO_STREAM(logger, "\033[1;32m" << x << "\033[0m")
#define ROS_YELLOW_STREAM(logger, x)  RCLCPP_INFO_STREAM(logger, "\033[1;33m" << x << "\033[0m")
#define ROS_BLUE_STREAM(logger, x)    RCLCPP_INFO_STREAM(logger, "\033[1;34m" << x << "\033[0m")
#define ROS_MAGENTA_STREAM(logger, x) RCLCPP_INFO_STREAM(logger, "\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM(logger, x)    RCLCPP_INFO_STREAM(logger, "\033[1;36m" << x << "\033[0m")

#define ROS_BLACK_STREAM_COND(logger, c, x)   if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;30m" << x << "\033[0m"); }
#define ROS_RED_STREAM_COND(logger, c, x)     if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;31m" << x << "\033[0m"); }
#define ROS_GREEN_STREAM_COND(logger, c, x)   if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;32m" << x << "\033[0m"); }
#define ROS_YELLOW_STREAM_COND(logger, c, x)  if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;33m" << x << "\033[0m"); }
#define ROS_BLUE_STREAM_COND(logger, c, x)    if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;34m" << x << "\033[0m"); }
#define ROS_MAGENTA_STREAM_COND(logger, c, x) if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;35m" << x << "\033[0m"); }
#define ROS_CYAN_STREAM_COND(logger, c, x)    if (c) { RCLCPP_INFO_STREAM(logger, "\033[1;36m" << x << "\033[0m"); }

#define ROS_BLACK_STREAM_THROTTLE(logger, c, x)   RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;30m" << x << "\033[0m")
#define ROS_RED_STREAM_THROTTLE(logger, c, x)     RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;31m" << x << "\033[0m")
#define ROS_GREEN_STREAM_THROTTLE(logger, c, x)   RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;32m" << x << "\033[0m")
#define ROS_YELLOW_STREAM_THROTTLE(logger, c, x)  RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;33m" << x << "\033[0m")
#define ROS_BLUE_STREAM_THROTTLE(logger, c, x)    RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;34m" << x << "\033[0m")
#define ROS_MAGENTA_STREAM_THROTTLE(logger, c, x) RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM_THROTTLE(logger, c, x)    RCLCPP_INFO_STREAM_THROTTLE(logger, c, "\033[1;36m" << x << "\033[0m")