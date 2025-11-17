#pragma once
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "leitungssatz_interfaces/msg/dashboard_info.hpp"
#include "leitungssatz_interfaces/srv/set_speed_slider.hpp"

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_io_interface.h>

class DashboardClient
{
public:
  DashboardClient();
  ~DashboardClient();

  // initialize using an existing node (non-RT). robot_ip must be provided.
  bool init(const rclcpp::Node::SharedPtr & node, const std::string & robot_ip);

  // stop background thread and cleanup
  void shutdown();

  // poll / state helpers (non-RT)
  bool isInRemoteControl();
  void powerOn();
  void powerOff();
  void brakeRelease();
  void play();
  void unlockProtectiveStop();
  void restartSafety();

private:
  void run();

  rclcpp::Node::SharedPtr node_;
  std::string robot_ip_;

  std::unique_ptr<ur_rtde::DashboardClient> rtde_dashboard_;
  std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;

  // publisher for dashboard info
  rclcpp::Publisher<leitungssatz_interfaces::msg::DashboardInfo>::SharedPtr dashboard_info_pub_;

  // services (exposed to the same node namespace)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr brake_release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_safety_srv_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetSpeedSlider>::SharedPtr speed_slider_srv_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  // cached message
  leitungssatz_interfaces::msg::DashboardInfo dashboard_info_msg_;
};